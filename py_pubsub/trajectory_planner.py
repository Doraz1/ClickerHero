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


class TrajectoryPlanner:
    def __init__(self, parent):
        self.parent = parent

#        self.max_vels = [0.08, 0.45]
        self.max_vels = [0.06, 0.85]

        self.dt = self.parent.dt
        self.logger = self.parent.get_logger()

        self.traj_type = 0
        self.traj_len = 0.0
        self.traj_vel = 0.0
        self.traj_moves = []
        self.traj_dict = {
                         "Reset": -1,
                         "Stop": 0,
                         "Line": 1,
                         "Circle": 2,
                         "Return home": 3,
                         "Freestyle": 4,
                         "CL_forward": 6,
                         "CL_rotate_cw": 7,
                         "CL_rotate_ccw": 8,

                         "Testing1": 100,
                         "Testing2": 101,
                         "Testing3": 102,
                         "Testing4": 103,
                         }


    def set_traj(self, msg):
        self.traj_type = msg.data[0]
        self.traj_len = msg.data[1]
        self.traj_vel = msg.data[2]

        if self.traj_type == self.traj_dict["Stop"]:
            self.print(f'-----------Received request to stop movement-------------')
            self.traj_moves = []
        elif self.traj_type != self.traj_dict["Return home"] and self.traj_type != self.traj_dict["Freestyle"]:
            self.print(f'Received trajectory msg: {self.get_traj_state_name_from_dict(self.traj_type)} with length={self.traj_len} at velocity={np.round(self.traj_vel, 2)}.')
            self.traj_moves = self.update_trajectory(self.traj_type, self.traj_len, self.traj_vel)

    def get_traj_state_name_from_dict(self, state_num):
        state_name = list(self.traj_dict.keys())[list(self.traj_dict.values()).index(state_num)]
        return state_name

    def update_trajectory(self, traj_type, traj_len, traj_vel_frac):
        'Moves are a list of x, rz commands'

        vel_sign = math.copysign(1, traj_vel_frac)
        
        if abs(traj_vel_frac) > 1:
            traj_vel_frac = vel_sign

        if traj_type == -1:
            'reset signal'
            moves = self.reset()
        if traj_type == 0:
            moves = self.stop()
        elif traj_type == 1:
            'line'
            moves = self.generate_line_cmds(traj_len, traj_vel_frac)
        elif traj_type == 2:
            'circle'
            moves = self.generate_circle_cmds(traj_len, traj_vel_frac)
        elif traj_type == 6:
            'CL forward'
            moves = []
        elif traj_type == 7:
            'CL cw'
            moves = []
        elif traj_type == 8:
            'CL ccw'
            moves = []

        else:
            moves = None

        return moves

    def reset(self):
        return [-1.0, -1.0]

    def stop(self):
        return [0.0, 0.0]

    def generate_line_cmds(self, len, vel_frac):
        "We generate a line of given distance at given speed"
        
        lin_vel = vel_frac*self.max_vels[0]  # meter/sec
        rot_vel = vel_frac*self.max_vels[1]  # rad/sec

        num_cmds_till_line_end = int(len / (100*abs(lin_vel)*self.dt))  # len is in cms so we convert vel to cms instead of meters
        num_cmds_till_180_turn = int((np.pi/abs(rot_vel)) / self.dt)
        self.print(f"num cmds per second for L={len}: {num_cmds_till_line_end*self.dt}, num rot cmds per second: {num_cmds_till_180_turn*self.dt} lin_vel: {lin_vel} rot_vel: {rot_vel}")
        return num_cmds_till_line_end*[[lin_vel, 0.0]] + num_cmds_till_180_turn*[[0.0, rot_vel]] + num_cmds_till_line_end*[[lin_vel, 0.0]] + num_cmds_till_180_turn*[[0.0, -rot_vel]]

    def generate_circle_cmds(self, r, vel_frac):
        "We generate a circle of given radius at given speed"
        d = 16  #wheel distance in cms
        lin_vel = vel_frac*self.max_vels[0]  # meter/sec

        if r != 0.0:
            rot_vel = -10*(d/r)*lin_vel  # opposite sign of lin vel creates CCW rotation. value must be large enough to turn and small enough not to negate linear velocity
        else:
            rot_vel = self.max_vels[1]

        normalize_coeff = abs(rot_vel) / self.max_vels[1]
        if normalize_coeff > 1:
            'velocity too high - normalize'
            lin_vel /= normalize_coeff
            rot_vel /= normalize_coeff

        num_cmds_till_end = int(np.pi*(d/2) / (100*abs(lin_vel*rot_vel)*self.dt))  # len is in cms so we convert vel to cms instead of meters
        print(f"num cmds per second: {num_cmds_till_end*self.dt}")
        return num_cmds_till_end*[[lin_vel, rot_vel]]

    def print(self, str):
        self.logger.info(str)

def test():
    planner = TrajectoryPlanner({0: "stop"})
    moves = planner.update_trajectory(1, 5, 0.01)


if __name__ == '__main__':

    test()
