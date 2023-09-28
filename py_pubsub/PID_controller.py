# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import sys
import math
import numpy as np


class PID:
    def __init__(self, nav, Kp, Ki, Kd, logger=None):
        self.nav = nav
        self.kp_mat = np.array([[Kp, 0], [0, Kp]])
        self.ki_mat = np.array([[Ki, 0], [0, Ki]])
        self.kd_mat = np.array([[Kd, 0], [0, Kd]])
        self.logger = logger

        'Integrator bounds'
        integ_maximum = 0.06
#        integ_maximum = 0.02
        self.integ_max = [integ_maximum, integ_maximum]  # radial, angular
        self.integ_min = [-integ_maximum, -integ_maximum]  # radial, angular

        'Smooth start and stop'
        self.err_thresh_start_moving = [0.07, 0.08]  # required linear/angular for initializing movement
        self.err_thresh = [0.03, 0.02]  # required linear/angular for moving to next phase
#        self.err_thresh = [0.01, 0.02] 
        self.error_sign_norm_thresh = 0.05
        self.aziduch_dy_thresh = 0.0055

        self.min_vels = [0.003, 0.01]
        self.max_vels = [0.04, 0.4]  # for testing
#        self.max_vels = [0.08, 0.45]  for driving
        self.a_lim = [0.02, 0.2]

        'Convert errors to commands'
        self.err_to_cmd_mat = np.array([[-0.4, 0], [0, 1.0]])  # radial 0.2 error will create a 0.08, angular 0.4 will create a 0.4
        self.err_to_cmd_mat_rev = np.array([[0.4, 0], [0, -1.0]])

        self.cmd_prev = np.zeros(2)
        self.P_vals = np.zeros(2)
        self.I_vals = np.zeros(2)
        self.D_vals = np.zeros(2)

        self.movementPhase = 0
        self.new_movement = False
        self.curr_pose = np.zeros(3)

        self.err = np.zeros(3)
        self.err_prev = np.zeros(3)
        self.aziduch_err = 0

        'for calculating percentage of movement'
        self.initial_rad_err = 0.0
        self.initial_ang_err = 0.0

        self.drivingReverse = False
        self.goal_reached = False
        self.forced_to_reset = False

    def reset_movement_phase(self, collision_reset):
        self.movementPhase = 0

        if collision_reset:
            self.forced_to_reset = True

        self.reset_goal_reached()

    def reset_goal_reached(self):
        self.goal_reached = False
        self.showed_goal_reached_msg = True
        self.print("Resetting goal reached")

    def print(self, str):
        if self.logger:
            self.logger.info(str)

    def calc_cmd(self, curr_pose, des_pose):
        '''Calculate error
           translate cartesian error (dx, dy, dtheta) to radial (dr, dtheta). Get aziduch theta [-pi, pi]
           Update movement phase based on radial error
           Calculate PID based on error'''

        'Radial and angular error from current pose to desired pose'
        self.curr_pose = curr_pose

        err = self.get_error(curr_pose, des_pose)
#        self.print(f'err: {np.round(err, 2)} | aziduch err: {np.round(self.aziduch_err, 2)}')

        '0 - standstill - goal reached'
        '1 - Radially and angularly far - turning to face aziduch'
        '2 - Radially far and facing aziduch - moving straight to dest'
        '3 - Reached destination, far from dest angle - turning to face dest angle'
        self.__update_movement_phase(err)
  #      self.print(f'phase: {self.movementPhase}')
        'Threshold error based on phase - turn at phase 1, straight at phase 2 and turn at phase 3'
        err_threshed = self.__thresh_err(err)
#        self.print(f'threshed errors: {err_threshed}')

        'Calculate P, I and D contributions'
        pid_vals = self.get_pid_vals(err_threshed)
#        self.print(f'PID values: {pid_vals}')

        cmd_out_smooth = self.get_cmd_out(pid_vals)
#        self.print(f'cmd: {cmd_out_smooth}')

        return err_threshed, self.err_prev, pid_vals, pid_vals, cmd_out_smooth, self.movementPhase

    def get_error(self, curr_pose, des_pose):
        self.err = np.array(des_pose) - np.array(curr_pose) - self.err_prev # (dx, dy, dtheta)

        self.aziduch_err = self.__calc_aziduch_err()
        spherical_err = self.__cartesian_to_spherical_err()  # (dr, dtheta), thet_aziduch
        return spherical_err

    def __calc_aziduch_err(self):
        radial_err = np.linalg.norm(self.err[:-1])  # radial x, y distance
#        self.print(f"err = {np.round((err[0], err[1]), 3)}  - below thresh so taking positive aziduch err = {self.aziduch_err}")

        'Calculate aziduch while moving'
        if radial_err < self.aziduch_dy_thresh:
            err = 0.0
#            self.print(f"radial err of {np.round(self.err, 2)} is {np.round(radial_err, 5)}<{self.aziduch_dy_thresh} so taking positive aziduch err = {err}")
        else:
#        if np.abs(err[1]) < self.aziduch_dy_thresh:
#            self.aziduch_err = math.atan2(np.abs(err[1]), err[0])
            err = math.atan2(self.err[1], self.err[0]) - self.curr_pose[2]
#            self.print(f"radial err of {np.round(self.err, 2)} is {np.round(radial_err, 5)}>={self.aziduch_dy_thresh} so taking normal aziduch err = {err}")

        return err

    def __cartesian_to_spherical_err(self):
        limited_ang_err = self.limit_minus_pi_to_pi(self.err[2])
        radial_err = np.linalg.norm(self.err[:-1])  # radial x, y distance

        'Calculate error relative to current heading to determine sign of movement (reverse or not)'
        theta = self.curr_pose[2]
        heading_direction = np.array([np.cos(theta), np.sin(theta), 0])
        heading_direction_error = np.matmul(self.err, heading_direction)
        if radial_err < self.error_sign_norm_thresh:
            'Sign is small so error direction defined as forward to prevent instability'
            sign = 1
        else:
            sign = math.copysign(1, heading_direction_error)
#            self.print(f"sign: {sign} since theta={theta} => heading dir= {heading_direction} and err= {heading_direction_error}")

        spherical_err = np.array([sign*radial_err, limited_ang_err])
        return spherical_err

    def __update_movement_phase(self, err):

        'Initially turn so that you"re directly facing the point'
        'Then drive straight to it, minimizing radius'
        'Finally, turn to desired theta'

        'Check if robot should start moving - thresh is higher'
        rad_err, ang_err = err
        rad_thresh_init, ang_thresh_init = self.err_thresh_start_moving
#        self.aziduch_err = self.limit_minus_pi_to_pi(aziduch_relative - self.curr_pose[2])  # TODO this replaces the reverse function and disables it
        #self.calculate_if_reverse(aziduch)  # sets reverse driving

#        self.print(f"rad error:{np.round(rad_err, 2)} | angular err: {np.round(ang_err, 2)} | aziduch_err: {np.round(self.aziduch_err, 2)}")

        self.new_movement = self.movementPhase == 0 and (np.abs(rad_err)>rad_thresh_init or np.abs(ang_err)>ang_thresh_init)
#        self.new_movement = (self.movementPhase == 0 or (self.movementPhase == 3 and self.goal_reached)) and (np.abs(rad_err)>rad_thresh_init or np.abs(ang_err)>ang_thresh_init)

        if self.new_movement:
            #Reset errors and initial errors for % calculation
            self.handle_new_movement(rad_err, ang_err)


        if self.goal_reached and not self.showed_goal_reached_msg:
            self.print("Not calculating movement since goal reached")
            self.showed_goal_reached_msg = True
            return

        'Calculate PID errors'
        rad_thresh, ang_thresh = self.err_thresh
        aziduch_err_large = (np.abs(self.aziduch_err) > ang_thresh)
        rad_err_large = (np.abs(rad_err) > rad_thresh)
        ang_err_large = (np.abs(ang_err) > ang_thresh)

        if rad_err_large and self.movementPhase < 3:
            'far from destination and in destination-approaching-phases'
            if aziduch_err_large and self.movementPhase < 1:
                'Far from direct angle to destination - Turning to face destination'
                self.print(f"aziduch err: {np.round(self.aziduch_err, 2)} is large - turning towards aziduch of destination point")
                self.movementPhase = 1
            elif (not aziduch_err_large) and self.movementPhase < 2:
                'Facing destination - driving towards it'
                self.print(f"facing destination, aziduch err {np.round(self.aziduch_err, 2)} small and radial err: {np.round(rad_err, 2)} is large - moving straight to destination point")
                self.movementPhase = 2
        else:
            'Reached destination'
            if ang_err_large:
                if self.movementPhase < 3:
                    'Far from destination theta - turning'
                    self.print(f"reached destination and angular err: {np.round(ang_err, 2)} is large - turning towards goal theta")
                    self.movementPhase = 3
            else:
                'Reached destination radially and angularly - Goal reached'
                if not self.goal_reached:
                    self.print(f"Goal reached since rad,ang errors ({np.round(rad_err,4)}<{rad_thresh}, {np.round(ang_err, 4)}<{ang_thresh}) are small and phase is {self.movementPhase} - await new goal")
                self.err_prev = self.err
                self.goal_reached = True
                self.movementPhase = 0

    def handle_new_movement(self, rad_err, ang_err):
        self.reset_goal_reached()

        self.print('Handling new movement in PID and resetting goal reached')

        if self.forced_to_reset:
            self.print('Collided and forced to reset - dont reset percentage error')
            self.forced_to_reset = False
        else:
            self.print('Moving from a stand-still - reset error')
            self.initial_rad_err = rad_err
            self.initial_ang_err = ang_err

    def __thresh_err(self, err):
        """If radial error is large, turn to destination and drive straight to it.
           If it's small, dont move"""
        radial_err, angular_err = err

        if self.movementPhase == 1:
            angular_err = self.aziduch_err
        'Restrict movement when moving to absolute locations'
        if self.movementPhase != 2:
            'Only issue linear command at phase 2'
            radial_err = 0.0
        if self.movementPhase == 0 or self.movementPhase == 2:
            'Move angular only for aziduch or final convergence to destination angle'
            angular_err = 0.0

        threshed = [radial_err, angular_err]

        return np.array(threshed)


    def get_pid_vals(self, error):
        # Convert radial and angular errors to radial and angular PID values
        self.P_vals = np.matmul(self.kp_mat, error)

        self.I_vals = 0.95*self.I_vals + np.matmul(self.ki_mat, error)

        'limit integrator'
        for i in range(len(self.I_vals)):
            self.I_vals[i] = max(self.integ_min[i], min(self.I_vals[i], self.integ_max[i]))

        self.D_vals = np.matmul(self.kd_mat, error)
        pid = self.P_vals + self.I_vals + self.D_vals
#        print(f"P: {self.P_vals}, I: {self.I_vals}, D: {self.D_vals}")

        return pid

    def get_cmd_out(self, pid_vals):
        if self.movementPhase == 0:
            out = np.array([0.0, 0.0])
        else:
            'output vel cmd'
            if self.drivingReverse:
                print("\tREVERSED!")
                out = np.matmul(self.err_to_cmd_mat_rev, pid_vals)
            else:
                out = np.matmul(self.err_to_cmd_mat, pid_vals)

        #smoothed when reaches navigator
#        out_smooth = self.nav.smooth_cmd_out(out)
#        return out_smooth

        return out
    def limit_minus_pi_to_pi(self, angle):
        'angle is in rads'
        tmp_angle = angle % (2*np.pi)

        if tmp_angle > np.pi:
            tmp_angle -= 2*np.pi

        return tmp_angle

    def calculate_if_reverse(self, aziduch):
        reverse = False
        'Calculate aziduch'
        if np.abs(self.aziduch_err) > np.pi/2:
            'Move in reverse'
            reverse = True
 #           print(f"angular error: {aziduch}")
            if aziduch_err > np.pi/2:
                aziduch_err -= np.pi
            elif aziduch_err < -np.pi/2:
                aziduch_err += np.pi

        self.drivingReverse = reverse

    def __smooth(self, cmd_vec):
        res = []
        for cmd, cmd_prev, a_lim in zip(cmd_vec, self.cmd_prev, self.a_lim):
            diff = cmd - cmd_prev
            diff_sign = math.copysign(1, diff)
            abs_diff = diff_sign*diff
            if abs_diff > a_lim:
                'Acceleration too high - Accelerate by maximal (absolute) amount in correct direction'
                res.append(cmd_prev + diff_sign*a_lim)
            else:
                res.append(cmd)
        print(f"acceleration limited cmd: {res}")

        out = []
        'Limit final command'
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

        self.cmd_prev = out

        return out
    def reset_err_prev(self):
        self.err_prev = np.zeros(3)
def test_cum_err():
    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [0, 0, np.pi/8]
    print(f"\n**SHORT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    des_pose = [0, 0, 2*np.pi/8]
    print(f"\n**SHORT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    des_pose = [0, 0, 3*np.pi/8]
    print(f"\n**SHORT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    des_pose = [0, 0, 4*np.pi/8]
    print(f"\n**SHORT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

def test_lines():
    print(f"\n\n\t**FORWARD**")

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [1, 0, 0]
    print(f"\n**SHORT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [2, 0, 0]
    print(f"\n**LONG**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    print(f"\n\n\t**BACKWARD**")

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [-1, 0, 0]
    print(f"\n**SHORT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)


    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [-2, 0, 0]
    print(f"\n**LONG**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

def test_rotations():
    print(f"\n\n\t**CW**")

#    pid = PID(1, 1, 1)
#    curr_pose = [0, 0, 0]
#    des_pose = [0, 0, np.pi/1000]
#    print(f"\n**SHORT**: curr: {curr_pose}, des: {des_pose}")
#    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [0, 0, np.pi/100]
    print(f"\n**SHORT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

#    pid = PID(1, 1, 1)
#    curr_pose = [0, 0, 0]
#    des_pose = [0, 0, np.pi/8]
#    print(f"\n**SHORT**: curr: {curr_pose}, des: {des_pose}")
#    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [0, 0, np.pi/2]
    print(f"\n**LONG**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [0, 0, np.pi]
    print(f"\n**LONG**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [0, 0, 2*np.pi]
    print(f"\n**LONG**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    print(f"\n\n\t**CCW**")

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [0, 0, -np.pi/100]
    print(f"\n**SHORT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)


    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [0, 0, -np.pi/2]
    print(f"\n**LONG**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [0, 0, -np.pi]
    print(f"\n**LONG**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [0, 0, -2*np.pi]
    print(f"\n**LONG**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

def test_angles():
    print(f"\n\n\t**FORWARD**")

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [1, 1, 0]
    print(f"\n**LEFT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, np.pi/4]
    des_pose = [1, 1, 0]
    print(f"\n**STRAIGHT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [1, -1, 0]
    print(f"\n**RIGHT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    print(f"\n\n\t**BACKWARD**")

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [-1, -1, 0]
    print(f"\n**LEFT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, -np.pi/2 - np.pi/4]
    des_pose = [-1, -1, 0]
    print(f"\n**straight**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)

    pid = PID(1, 1, 1)
    curr_pose = [0, 0, 0]
    des_pose = [-1, 1, 0]
    print(f"\n**RIGHT**: curr: {curr_pose}, des: {des_pose}")
    threshed_theta = pid.calc_cmd(curr_pose, des_pose)
  


if __name__ == '__main__':

    test_lines()
#    test_rotations()
#    test_angles()
#    test_cum_err()
