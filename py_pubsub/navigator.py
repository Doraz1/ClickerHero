import os
import rclpy
from rclpy.node import Node

import sys
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from py_pubsub.PID_controller import PID
from py_pubsub.trajectory_planner import TrajectoryPlanner
from py_pubsub.closed_loop_nav import ClosedLoopNav
from py_pubsub.recovery_behavior_nav import RecoveryBehaviorNav
from py_pubsub.odom_imu_mismatch_handler import MismatchHandler
from std_msgs.msg import Float32MultiArray
import time
import math
import numpy as np
from rclpy.qos import qos_profile_sensor_data

robot_ns = os.environ['robot_namespace']
clicker_ind = os.environ['clicker_index']

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        desTrajSubName = f'/{robot_ns}/cmd_traj'
        self.desTrajSubscription = self.create_subscription(Float32MultiArray, desTrajSubName, self.desTraj_callback, 10)

        'These are used to update the prox sensor handler'
        proxSenSubName = f'/{robot_ns}/collided_1'
        self.proxSenSubscription = self.create_subscription(Int16, proxSenSubName, self.prox_sensor_1_callback, 10)
        self.collided_1 = False

        proxSenSubName = f'/{robot_ns}/collided_2'
        self.proxSenSubscription = self.create_subscription(Int16, proxSenSubName, self.prox_sensor_2_callback, 10)
        self.collided_2 = False

        wheelOdomSubName = f'/{robot_ns}/odom'
        self.wheelOdomSubscription = self.create_subscription(Odometry, wheelOdomSubName, self.wheel_odom_callback, 10)

        currPoseSubName = f'/camera_{clicker_ind}/pose/sample'
        self.proxSenSubscription = self.create_subscription(Odometry, currPoseSubName, self.currPose_callback, qos_profile=qos_profile_sensor_data)

        self.collided_mismatch = False

        cmdPubName = f'/{robot_ns}/cmd_vel'
        self.posePub = self.create_publisher(Twist, cmdPubName, 10)
        self.cmd_vel_prev = np.zeros(2)

        self.dt = 0.1  # seconds
#        self.dt = 1.0
        self.timer = self.create_timer(self.dt, self.publish_cmd)
        self.timer = self.create_timer(self.dt, self.handle_mismatch_state)

        self.prev_movement = ""

        self.trajectory_planner = TrajectoryPlanner(self)
        self.closedLoopNav = ClosedLoopNav(self)
        self.recoveryBehaviorNav = RecoveryBehaviorNav(self)
        self.mismatchHandler = MismatchHandler(self, self.dt)

        self.traj_dict = self.trajectory_planner.traj_dict
        self.min_vels = self.closedLoopNav.PID.min_vels
        self.max_vels = self.closedLoopNav.PID.max_vels
        self.a_lim = self.closedLoopNav.PID.a_lim

        self.perc = 0.0  # for initializing relative movements after collision
        self.get_logger().info('Successfully created Navigator')

    #callbacks
    def currPose_callback(self, msg):
        pose = self.mismatchHandler.get_pose_vec_from_odom_msg(msg)
        self.mismatchHandler.set_pose(pose)
        self.closedLoopNav.set_pose(pose)

    def wheel_odom_callback(self, msg):
        odom = self.mismatchHandler.get_pose_vec_from_odom_msg(msg)
        self.mismatchHandler.set_odom(odom)

    def desTraj_callback(self, msg):
        is_reset_msg = msg.data[0] == self.traj_dict["Reset"]
        is_stop_msg = msg.data[0] == self.traj_dict["Stop"]

        if is_reset_msg or is_stop_msg or self.prev_movement != "collided" or (self.prev_movement == "collided" and self.closedLoopNav.PID.goal_reached):
            self.trajectory_planner.set_traj(msg)
            self.reset_CL_navigator()

            if self.trajectory_planner.traj_type == self.traj_dict["Reset"]:
                self.get_logger().info(f'-----------Received request to reset X0-------------')
                self.closedLoopNav.update_home_pose()
        else:
            self.get_logger().info(f'Ignored new trajectory request since still collided')

    def prox_sensor_1_callback(self, msg):
#        self.get_logger().info(f'Received prox sensor message: {msg.data}')
        self.collided_1 = (msg.data == 1)

    def prox_sensor_2_callback(self, msg):
#        self.get_logger().info(f'Received prox sensor message: {msg.data}')
        self.collided_2 = (msg.data == 1)

    def handle_mismatch_state(self):
        self.collided_mismatch = self.mismatchHandler.handle_mismatch_state(self.collided_1 or self.collided_2)
#        self.get_logger().info(f"mismatch state in navigator: {self.collided_mismatch}")

    def reset_CL_navigator(self, collision_reset=False):
        # on new trajectory, reset PID movement phase and previous relative destination
        self.closedLoopNav.reset(collision_reset)

    def get_cmd(self):
        'Checks whether were collided, testing or moving forward and gets cmds from the trajectory, collision-recovery or closed loop planners'
        traj_type = self.trajectory_planner.traj_type
        
        stopped, testing, moving_closed_loop, movingHome, movingFreestyle, moving, moving_forward, collided, mismatch = self.check_current_movement(traj_type)

#        self.get_logger().info(f'publishing navigator cmd. collided: {collided} | testing: {testing} | homing: {movingHome} | freestyle: {movingFreestyle} goal reached: {self.closedLoopNav.PID.goal_reached}')

        if stopped:
            mv_x, mv_rz = 0.0, 0.0
#            self.get_logger().info(f'Stopping')
        elif collided or mismatch:
            mv_x, mv_rz = self.get_cmd_collided(collided, mismatch)
#            self.get_logger().info(f'Colliding')
        else:
            if moving_closed_loop and self.prev_movement != "Closed loop":
                'New request to move in CL - prioritize'
                self.get_logger().info(f'new request to move in CL')
                self.reset_CL_navigator()
                mv_x, mv_rz = self.get_cmd_closed_loop(traj_type)

            elif self.trajectory_planner.traj_moves != [] and self.prev_movement != "trajectory":
                'New request to move in OL - prioritize'
                self.reset_CL_navigator()
                mv_x, mv_rz = self.get_cmd_trajectory()
                self.get_logger().info(f'new request to move in OL')
            else:
                if testing or moving_closed_loop:  # closed loop already includes testing
                    mv_x, mv_rz = self.get_cmd_closed_loop(traj_type)
                elif movingHome:
                    mv_x, mv_rz = self.get_cmd_homing()
                elif movingFreestyle:
                    mv_x, mv_rz = self.get_cmd_freestyle()
                else:
                    mv_x, mv_rz = self.get_cmd_trajectory()

#        self.get_logger().info(f"cmd_x, cmd_rz = {mv_x, mv_rz}")
        smooth_mv_x, smooth_mv_rz = self.smooth_cmd_out([mv_x, mv_rz])
        return smooth_mv_x, smooth_mv_rz

    def check_current_movement(self, traj_type):
        stopped = (traj_type == self.traj_dict["Stop"])
        testing = (traj_type == self.traj_dict["Testing1"] or traj_type == self.traj_dict["Testing2"] or traj_type == self.traj_dict["Testing3"] or traj_type == self.traj_dict["Testing4"])
        movingHome = (traj_type == self.traj_dict["Return home"])
        movingFreestyle = (traj_type == self.traj_dict["Freestyle"])

        moving = movingHome or movingFreestyle or (self.trajectory_planner.traj_moves != [])
        moving_forward = self.cmd_vel_prev[0] < 0

        rotating_CL = traj_type == self.traj_dict["CL_rotate_cw"] or traj_type == self.traj_dict["CL_rotate_ccw"]
        CL_movement = (traj_type == self.traj_dict["CL_forward"] or rotating_CL)

        CL_movement = testing or CL_movement

        if not rotating_CL and (moving_forward or self.prev_movement == "collided"):
#            self.get_logger().info(f'Not rotating CL and moving forward - update collided value')
#            collided = self.recoveryBehaviorNav.check_collision()
#            collided = self.collided_mismatch
            collided = self.recoveryBehaviorNav.check_collision()
            mismatch = self.collided_mismatch
        else:
#            self.get_logger().info(f'Set collided to false since either rotating CL or both not moving forward and wasnt collided')
            collided = False
            mismatch = False

        return stopped, testing, CL_movement,  movingHome, movingFreestyle, moving, moving_forward, collided, mismatch

    # cmds based on movement
    def get_cmd_collided(self, collided, mismatch):
        use_pid = True
        if self.prev_movement != "collided":
            self.prev_movement = "collided"
            self.get_logger().info(f"Recovery behavior - turn away from trouble")

            'Use PID with relative movement'
#            if use_pid:
#                self.closedLoopNav.fixate_pose()  # fixates the pose we're moving relative to


        if use_pid:
            if self.closedLoopNav.PID.goal_reached:  # Reset the relative movement if the collision persists
            #    self.prev_movement = "collided and turned"
                if mismatch:
                    self.mismatchHandler.reset(explanation_str="since recovery behavior complete")
                mv_x, mv_rz = 0.0, 0.0
            else:
#                self.get_logger().info(f"Getting commands to rotate 90 degs for recovery")
                mv_x, mv_rz = self.closedLoopNav.get_cmds(des_pose=[0.0, 0.0, np.deg2rad(-90.0)], relative=True)
        else:
            'Some other recovery behavior - unimplemented'
            self.get_logger().info(f"Unimplemented recovery behavior - stop for now")
            mv_x, mv_rz = 0.0, 0.0
#            mv_x, mv_rz = self.recoveryBehaviorNav.get_cmds()

        return mv_x, mv_rz

    def get_cmd_closed_loop(self, traj_type):
        if self.prev_movement != "Closed loop":
            self.handle_percentage_after_collision()
            self.prev_movement = "Closed loop"

        factor = (100 - self.perc) / 100  # [0, 100] - percentage left over
        CL_linear_vel = 0.069
        CL_rot_vel = 0.85

        if traj_type == self.traj_dict["Testing1"]:
            mv_x, mv_rz = self.closedLoopNav.get_cmds(des_pose=factor*np.array([0.15, 0.0, 0.0]), relative=True)
        elif traj_type == self.traj_dict["Testing2"]:
            mv_x, mv_rz = self.closedLoopNav.get_cmds(des_pose=factor*np.array([-0.15, 0.0, 0.0]), relative=True)
        elif traj_type == self.traj_dict["Testing3"]:
            mv_x, mv_rz = self.closedLoopNav.get_cmds(des_pose=factor*np.array([0.0, 0.0, np.pi/2]), relative=True)
        elif traj_type == self.traj_dict["Testing4"]:
            mv_x, mv_rz = self.closedLoopNav.get_cmds(des_pose=factor*np.array([0.0, 0.0, -np.pi/2]), relative=True)

        elif traj_type == self.traj_dict["CL_forward"]:
            mv_x, mv_rz = self.closedLoopNav.get_cmds(des_pose=factor*np.array([CL_linear_vel, 0.0, 0.0]), relative=True)
        elif traj_type == self.traj_dict["CL_rotate_cw"]:
            mv_x, mv_rz = self.closedLoopNav.get_cmds(des_pose=factor*np.array([0.0, 0.0, -CL_rot_vel]), relative=True)
        elif traj_type == self.traj_dict["CL_rotate_ccw"]:
            mv_x, mv_rz = self.closedLoopNav.get_cmds(des_pose=factor*np.array([0.0, 0.0, CL_rot_vel]), relative=True)

        return mv_x, mv_rz

    def set_PID_perc(self, val):
        self.perc = val
        self.get_logger().info(f'Setting PID completion to {val}%')


    def get_cmd_homing(self):
        if self.prev_movement != "Return home":
            self.prev_movement = "Return home"
            self.reset_CL_navigator()
            self.get_logger().info(f"Using PID to move home")

        mv_x, mv_rz = self.closedLoopNav.get_cmds(des_pose=[0.0, 0.0, 0.0])

        return mv_x, mv_rz

    def get_cmd_freestyle(self):
        if self.prev_movement != "Freestyle":
            self.prev_movement = "Freestyle"
            self.reset_CL_navigator()
            self.get_logger().info(f"Moving freestyle")

        mv_x, mv_rz = 0.0, 0.0

        return mv_x, mv_rz

    def get_cmd_trajectory(self):
        'Get trajectory from trajectory planner'
        if self.prev_movement != "trajectory":
            self.prev_movement = "trajectory"

        if not self.trajectory_planner.traj_moves:
            mv_x, mv_rz = 0.0, 0.0
        else:
            mv_x, mv_rz = self.trajectory_planner.traj_moves.pop(0)

        return mv_x, mv_rz
    def handle_percentage_after_collision(self):
        if self.prev_movement == "collided":
            self.set_PID_perc(self.closedLoopNav.percentage_of_movement_completed)
            self.get_logger().info(f"Reinitializing test after collision with {self.perc}% of movement")
            self.reset_CL_navigator(collision_reset=True)
        else:
            self.reset_CL_navigator()


    def publish_cmd(self):
        mv_x, mv_rz = self.get_cmd()

        msg = Twist()
        msg.linear.x = mv_x
        msg.angular.z = mv_rz
        self.posePub.publish(msg)

        self.cmd_vel_prev = np.array([mv_x, mv_rz])

    def smooth_cmd_out(self, cmd_list):
        res = []

        #self.get_logger().info(f'cmd: {cmd_list} | cmd prev: {self.cmd_vel_prev} a_lim: {self.a_lim}')
        'Limit acceleration'
        for cmd, cmd_prev, a_lim in zip(cmd_list, self.cmd_vel_prev, self.a_lim):
            diff = cmd - cmd_prev
            diff_sign = math.copysign(1, diff)
            abs_diff = diff_sign*diff

            if abs_diff > a_lim:
                'Acceleration too high - Accelerate by maximal (absolute) amount in correct direction'
                res.append(cmd_prev + diff_sign*a_lim)
            else:
                res.append(cmd)

        out = []
        'Limit velocity'
        for i, cmd in enumerate(res):
            if cmd == 0:
                '0 remains a 0'
                out.append(cmd)
            else:
                'positives limited above 0, negs limited below zero'
                sgn = math.copysign(1, cmd)
#                print(f"cmd: {cmd}, sigm: {sgn}")

                if sgn > 0:
                    out.append(max(self.min_vels[i], min(self.max_vels[i], cmd)))
                else:
                    out.append(min(-self.min_vels[i], max(-self.max_vels[i], cmd)))

        #self.get_logger().info(f'cmd: {cmd_list} | smoothed cmd: {out}')

        return out

def main(args=None):
    rclpy.init(args=args)

    pub = Navigator()

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()
