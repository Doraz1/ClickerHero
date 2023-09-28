#!/usr/bin/python3

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
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import RPi.GPIO as GPIO
import time


clicker_id = int(os.environ['clicker_index'])

class CollisionHandler(Node):
    def __init__(self):
        super().__init__('click_handler')

        self.filter_window_sz = 7
        self.prox1_state = 1
        self.prev_collided1_state = 0
        self.prox1_states_list = self.filter_window_sz*[self.prox1_state]

        self.prox2_state = 1
        self.prev_collided2_state = 0
        self.prox2_states_list = self.filter_window_sz*[self.prox2_state]

        topic_name = 'collided_1'
        self.publisher1 = self.create_publisher(Int16, topic_name, 10) # Queue size = 10
        topic_name = 'collided_2'
        self.publisher2 = self.create_publisher(Int16, topic_name, 10) # Queue size = 10

        # click parameters
        GPIO.setmode(GPIO.BCM)
        self.prox1_pin = 12
        GPIO.setup(self.prox1_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.prox2_pin = 24
        GPIO.setup(self.prox2_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        timer_dt = 0.05  # GPIO loop time
        self.dt = 0.1  # publisher time
        self.collision_timer = 0
        self.collision_timeout = 7/self.dt

        # detect clicked events
        self.create_timer(timer_dt, self.detect_collision_prox1)
        self.create_timer(timer_dt, self.detect_collision_prox2)
        self.create_timer(self.dt, self.publish_msgs)

        self.t0 = time.time()
        self.collision_time = 0
        self.get_logger().info('Successfully created proximity sensor handler')

    def detect_collision_prox1(self):
        self.detect_collision_prox(1)

    def detect_collision_prox2(self):
        self.detect_collision_prox(2)

    def detect_collision_prox(self, num):
#        pin_num = self.prox1_pin
        pin_num = getattr(self, f"prox{num}_pin")
        prev_states = getattr(self, f"prox{num}_states_list")

        # IO is flipped - 0 if collided and 1 if not
        curr_state = GPIO.input(pin_num)
#        self.get_logger().info(f'Curr state of prox sensor {num}: {curr_state}')
        prev_states.pop(0)
        prev_states.append(curr_state)

#        if sum(prev_states)/len(prev_states) < 1-1/self.filter_window_sz:
        if sum(prev_states)/len(prev_states) < 1 and self.collision_timer > 0:
            'Collision if any of previous readings was collision'
            setattr(self, f"prox{num}_state", 0)
            self.collision_timer -= 1  # count collision time
        else:
            setattr(self, f"prox{num}_state", 1)
            self.collision_timer = self.collision_timeout  # reset the collision timer


    def publish_msgs(self):
        self.publish_msg(1, deprecated=False)
#        self.publish_msg(1, deprecated=True)
        self.publish_msg(2, deprecated=False)
#        self.publish_msg(2, deprecated=True)

    def publish_msg(self, num, deprecated=False):
        collided = getattr(self, f"prox{num}_state") == 0
        prev_collided = getattr(self, f"prev_collided{num}_state")

        msg = Int16()
        if deprecated:
            msg.data = False
        else:
            msg.data = collided

        if not deprecated:
            if collided and not prev_collided:
                self.get_logger().info(f'Robot collided on sensor {num}')
            elif not collided and prev_collided:
                self.get_logger().info(f'Robot recovered from collision on sensor {num}')

        setattr(self, f"prev_collided{num}_state", collided)

        pub = getattr(self, f"publisher{num}")
        pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    collisionHandler = CollisionHandler()

    rclpy.spin(collisionHandler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
