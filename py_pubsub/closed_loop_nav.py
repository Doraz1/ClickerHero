import os
import rclpy
from rclpy.node import Node

import sys
from nav_msgs.msg import Odometry
from py_pubsub.PID_controller import PID
import numpy as np
from rclpy.qos import qos_profile_sensor_data

robot_ns = os.environ['robot_namespace']
clicker_ind = os.environ['clicker_index']

class ClosedLoopNav(Node):
    def __init__(self, nav):
        super().__init__('closed_loop_navigator')
        self.nav = nav

        self.curr_pose = np.zeros(3)
        self.des_pose = np.zeros(3)
        self.home_pose=np.zeros(3)

        self.new_dest_distance_thresh = 0.05

#        self.PID = PID(nav, Kp=2.0, Ki=0.1, Kd=0.01, self.get_logger())
        self.PID = PID(nav, Kp=2.0, Ki=0.1, Kd=0.0, logger=self.get_logger())
        self.initial_rad_err = 0.0  # for calculating percentage of movement completed
        self.percentage_of_movement_completed = 0.0

        self.prev_relative = False
        self.movement_x0 = np.zeros(3)
        self.get_logger().info('Successfully created closed loop navigator')

    def set_pose(self, pose):
        self.curr_pose = pose - self.home_pose
#        self.get_logger().info(f'Received pose msg: {np.round(self.curr_x, 2), np.round(self.curr_y, 2), np.round(self.curr_theta, 2)}')

    def update_home_pose(self):
        # Makes current location the new home
        self.get_logger().info('--Resetting home pose--')
        self.home_pose = self.curr_pose

    def reset(self, collision_reset):
        self.PID.reset_movement_phase(collision_reset)  # resets movement phase
        self.percentage_of_movement_completed = 0.0
        self.prev_relative = False

    def get_cmds(self, des_pose, relative=False, percentage=0):
        curr_pose = self.curr_pose
        des_pose = np.array(des_pose)

        'if changed from absolute to relative or vice versa, Stop current movement and initialize new one'
        self.check_relative_or_absolute(curr_pose, des_pose, relative)

        # Update desired pose based on requirement - absolute or spin incoming if relative
        des_pose = self.update_destination(curr_pose, des_pose, relative)

        err, err_prev, pid_val, pid_threshed, cmd, phase = self.PID.calc_cmd(curr_pose, des_pose)
        perc = self.radial_err_to_perc(err[0])  # only radial error

        if not self.PID.goal_reached:
            self.get_logger().info(f"Curr pose: {np.round(curr_pose, 2)}, des_pose: {np.round(des_pose,2)}, err: {np.round(err, 2)}, cmd: {np.round(cmd, 3)} | phase={phase} | {np.round(perc, 1)}% done")

        mv_x, mv_rz = cmd
        return mv_x, mv_rz

    def radial_err_to_perc(self, err):
        self.percentage_of_movement_completed = 100*(self.initial_rad_err - np.abs(err))/(self.initial_rad_err + 0.001)
        return self.percentage_of_movement_completed

    def check_relative_or_absolute(self, curr_pose, des_pose, relative):
        if relative:
            'Stop absolute movement and initialize relative movement'
            if not self.prev_relative:
                self.fixate_pose(curr_pose, des_pose)

        else:
            'Stop relative movement and initialize absolute movement'
            if self.prev_relative and not relative:
                self.release_pose(curr_pose, des_pose)

        self.prev_relative = relative

    def fixate_pose(self, curr_pose, des_pose):
#        self.get_logger().info(f'Fixing initial pose to move relative to to {np.round(curr_pose, 2)}!')
        self.movement_x0 = curr_pose

    def release_pose(self, curr_pose, des_pose):
        self.get_logger().info('Resetting relative movement')
        self.movement_x0 = np.zeros(3)

    def update_destination(self, curr_pose, des_pose, relative):
        if relative:
#            self.get_logger().info('relative destination update')
            theta = curr_pose[2]
            rotation_mat = np.array([[np.cos(theta), -np.sin(theta), 0],
                                     [np.sin(theta), np.cos(theta), 0],
                                     [0, 0, 1] ])

            des_pose_rel = np.matmul(rotation_mat, des_pose)
            des_pose = self.movement_x0 + des_pose_rel
        else:
#            self.get_logger().info('absolute destination update')
            pass  # des pose in the absolute destination
#            des_pose = des_pose

        delta_des_pose = np.linalg.norm(self.des_pose - des_pose)
        is_new_desired_pose = delta_des_pose >= self.new_dest_distance_thresh

        if is_new_desired_pose:
            self.handle_new_destination(des_pose)

        return self.des_pose

    def handle_new_destination(self, des_pose):
        self.get_logger().info(f"New incident desired pose: prev: {np.round(self.des_pose, 2)} | new: {np.round(des_pose, 2)}")
        self.des_pose = des_pose

        if self.PID.forced_to_reset:
            self.get_logger().info('forced to reset - dont reset error')
        else:
            radial_err = np.linalg.norm(des_pose[:2] - self.curr_pose[:2])
#            self.get_logger().info(f'resetting radial error from {self.initial_rad_err} to {radial_err}')
            self.initial_rad_err = radial_err
            self.PID.reset_err_prev()

    def euler_from_quaternion(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp =1 -  2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z*x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2*(y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    pub = ClosedLoopNav(None)

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()
