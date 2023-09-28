
import copy
import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

from std_msgs.msg import Float32MultiArray
import time
import math
import numpy as np

robot_ns = os.environ['robot_namespace']
clicker_ind = os.environ['clicker_index']


class MismatchHandler(Node):
    def __init__(self, parent, dt):
        self.parent = parent
        super().__init__('odom_IMU_mismatch_handler')

#        wheelOdomSubName = f'/{robot_ns}/odom'
#        self.wheelOdomSubscription = self.create_subscription(Odometry, wheelOdomSubName, self.wheel_odom_callback, 10)

#        currPoseSubName = f'/camera_{clicker_ind}/pose/sample'
#        self.proxSenSubscription = self.create_subscription(Odometry, currPoseSubName, self.currPose_callback, qos_profile=qos_profile_sensor_data)

#        mismatchPubName = f'/{robot_ns}/wheel_IMU_mismatch'
#        self.mismatchPub = self.create_publisher(Int16, mismatchPubName, 10)

        self.filter_size = 20

        self.prev_pose = np.array([0.0, 0.0, 0.0])
        self.pose = np.array([0.0, 0.0, 0.0])
        self.dPose_list = self.filter_size*[np.array([0.0, 0.0])]

        self.prev_odom = np.array([0.0, 0.0, 0.0])
        self.odom = np.array([0.0, 0.0, 0.0])
        self.dOdom_list = self.filter_size*[np.array([0.0, 0.0])]

        self.mean_dOdoms = self.filter_size*[0.0]
        self.mean_dPoses = self.filter_size*[0.0]

        self.streak_thresh = int(0.7*self.filter_size)
        self.mismatch_H_thresh = 0.0007
        self.mismatch_HH_thresh = 0.003
        self.mismatch_rate_thresh = 0.032  # 0.019
        self.minimal_mismatch_thresh = 0.0005

        self.mismatch = False
        self.firstOdom = True
        self.mismatch_timer = 0
        self.mismatch_timeout_iters = 7/dt  # sec/dt
        self.grace_iters = 1/dt  # sec/dt
        self.grace_counter = self.grace_iters
        self.get_logger().info('Successfully created mismatch handler!')

    #callbacks
    def set_odom(self, odom):
        self.odom = odom  # ros convention is x-front, y-left, z-up
#        self.get_logger().info(f'Received odom msg: {np.round(odom, 3)}')

        if self.firstOdom:
            self.reset()
            self.firstOdom = False

    def set_pose(self, pose):
        self.pose = pose  # x - right, y - up, z - back
 #       self.get_logger().info(f'Received pose msg: {np.round(pose, 3)}')

    def get_pose_vec_from_odom_msg(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orient_list = msg.pose.pose.orientation
        r, p, yaw = self.euler_from_quaternion(orient_list)
#        self.get_logger().info(f'Received {type} msg: {np.round(x, 2), np.round(y, 2), np.round(yaw, 2)}')

        return np.array([x, y, yaw])
    
    def reset(self, explanation_str=""):
        #reset offset between readings
        self.get_logger().info(f'Resetting the mismatch flag and the dOdom and dPose lists {explanation_str}')
        self.mismatch = False
        self.grace_counter = self.grace_iters
        self.dPose_list = self.filter_size*[np.array([0.0, 0.0])]
        self.dOdom_list = self.filter_size*[np.array([0.0, 0.0])]
        self.mean_dOdoms = self.filter_size*[0.0]
        self.mean_dPoses = self.filter_size*[0.0]
        self.prev_odom = copy.copy(self.odom)
        self.prev_pose = copy.copy(self.pose)

    def handle_mismatch_state(self, collided):
        'If wheel odom is radially very far from pose, weve collided - initiate recovery behavior'

#        if False:
        if (not self.mismatch) and (not collided):
            'Update Odom change'
            dOdom = self.odom[:2]-self.prev_odom[:2]
            self.cyclic_update(self.dOdom_list, dOdom)

            'Update Pose change'
            dPose = self.pose[:2]-self.prev_pose[:2]
            self.cyclic_update(self.dPose_list, dPose)

            'Get mean odom and pose changes'
            meanOdomChange, meanPoseChange, sumChangesOdom, sumChangesPose = self.get_mean_changes()

            'Calculate dxy for last N times'
            self.cyclic_update(self.mean_dOdoms, meanOdomChange)
            self.cyclic_update(self.mean_dPoses, meanPoseChange)
            amt_of_mismatch = np.abs(meanOdomChange - meanPoseChange)

            not_standing_mismatch =  amt_of_mismatch >= self.minimal_mismatch_thresh
            streak = self.get_consec_monotones_num(not_standing_mismatch)
#            self.get_logger().info(f'dOdom: {np.round(dOdom, 4)} | dPose: {np.round(dPose, 4)} | MeanOd: {np.round(meanOdomChange, 4)} | MeanPos: {np.round(meanPoseChange, 4)}, streak={streak}')

#            streak_cond = streak > self.streak_thresh and not_standing_mismatch  # distance between readings is moderately large for N iterations in a row
            streak_cond = streak > self.streak_thresh   # distance between readings is moderately large for N iterations in a row
            correct_direction_of_mismatch = (meanOdomChange > meanPoseChange > 0) or (meanOdomChange < meanPoseChange < 0)  # ensure collisions and not robot liftups or rotations
            rate_cond = amt_of_mismatch > self.mismatch_HH_thresh and correct_direction_of_mismatch  #  distance between readings is very large

            if (rate_cond or streak_cond) and correct_direction_of_mismatch:
                if self.grace_counter > 0:
                    self.get_logger().info(f'Ignoring mismatch condition since still in collision grace period')
                else:
                    if rate_cond:
                        self.get_logger().info(f'A mismatch of {np.round(np.abs(meanOdomChange - meanPoseChange), 4)} > {self.mismatch_HH_thresh} between IMU and odom was detected')
                    else:
                        self.get_logger().info(f'A mismatch streak of {streak} > {self.streak_thresh} was detected')

                    if not self.firstOdom:
                        'Not the initial error caused by initialization of encoders'
                        self.mismatch = True
                        self.mismatch_timer = self.mismatch_timeout_iters

            if self.grace_counter > 0:
                self.grace_counter -= 1

            self.prev_odom = copy.copy(self.odom)
            self.prev_pose = copy.copy(self.pose)
        elif self.mismatch:
            self.mismatch_timer -= 1
            if self.mismatch_timer <= 0:
                self.reset("since mismatch timed out")

        return self.mismatch

    def cyclic_update(self, buffer, newval):
        buffer.pop(0)
        buffer.append(newval)

    def get_mean_changes(self):
        sumOfChangesOdom = [sum(row[i] for row in self.dOdom_list) for i in range(2)]
        sumOfChangesPose = [sum(row[i] for row in self.dPose_list) for i in range(2)]
        meanOdomChange = np.linalg.norm(sumOfChangesOdom)/len(self.dOdom_list)
        meanPoseChange = np.linalg.norm(sumOfChangesPose)/len(self.dPose_list)
        return meanOdomChange, meanPoseChange, sumOfChangesOdom, sumOfChangesPose

    def get_consec_monotones_num(self, not_standing):
        'get number of monotonously increasing or decreasing diffs between the readings that signifies collision'
        diffs = [od-pos for od, pos in zip(self.mean_dOdoms, self.mean_dPoses)]  # check if gaining distance between means
        signs = [math.copysign(1, el) if abs(el) >= self.mismatch_H_thresh else 0 for el in diffs]
        trending_sign = signs[-1]
        streak = 0

#        if trending_sign != 0:
        if trending_sign == 1:  # only take odom increases, ignore camera incrases (due to rotations or lifting of robots usually)
            for sgn in signs[::-1]:
                if sgn == trending_sign:
                    streak += 1
                else:
                    break
#        self.get_logger().info(f'Diffs: {np.round(diffs, 4)} | signs: {signs} trending: {trending_sign} | streak = {streak}')
        return streak

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

    pub = MismatchHandler()

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()
