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

class ClickHandler(Node):
    def __init__(self):
        super().__init__('click_handler')

        self.mw1_clicked = 0
        self.mw2_clicked = 0
        topic_name = 'clicked'
        self.publisher_ = self.create_publisher(Int16, topic_name, 10) # Queue size = 10

        # click parameters
        GPIO.setmode(GPIO.BCM)
        self.sw1_pin = 4
        GPIO.setup(self.sw1_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.sw2_pin = 5
        GPIO.setup(self.sw2_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        timer_dt = 0.05  # microswitch loop time
        self.dt = 0.1  # publisher time

        self.debounce_time = 0.003  # sec

        # detect clicked events
        self.sw1_state = 0
        self.sw2_state = 1
        self.create_timer(timer_dt, self.detect_edge_sw1)
        self.create_timer(timer_dt, self.detect_edge_sw2)
        self.create_timer(self.dt, self.publish_msg)

        self.t0 = time.time()
        self.click_time = 0
        self.min_time_between_clicks = 0.3  # sec

        self.get_logger().info('Successfully created click handler')

    def detect_edge_sw1(self):
        clicked = False
        pin_num = self.sw1_pin
        prev_state = self.sw1_state
        self.sw1_state = GPIO.input(pin_num)
 #       self.get_logger().info(f'Entered pin {pin_num}. state: {self.sw1_state}')

        if prev_state == 0 and self.sw1_state == 1:
            'True click! ensuring its the only one'
            if not self.is_double_click():
                self.get_logger().info(f'Robot clicked on pin {pin_num}')
                clicked = True
        if self.mw1_clicked == 0 and clicked:
            self.mw1_clicked = 1
#        self.publish_msg(clicked)

    def detect_edge_sw2(self):
        clicked = False
        pin_num = self.sw2_pin
        prev_state = self.sw2_state
        self.sw2_state = GPIO.input(pin_num)
#        self.get_logger().info(f'Entered pin {pin_num}. state: {self.sw2_state}')

        if prev_state == 1 and self.sw2_state == 0:
            'True click! ensuring its the only one'
            if not self.is_double_click():
                self.get_logger().info(f'Robot clicked on pin {pin_num}')
                clicked = True

        if self.mw2_clicked == 0 and clicked:
            self.mw2_clicked = 1
#        self.publish_msg(clicked)

    def is_double_click(self):
        'Check for double clicks'
        curr_click_time = time.time() - self.t0
        dt = curr_click_time - self.click_time  # time since last legit click

        if dt > self.min_time_between_clicks:
            self.click_time = curr_click_time
            return False

        else:
            return True

    def publish_msg(self):
        clicked = ((self.mw1_clicked == 1) or (self.mw2_clicked == 1))
#        self.get_logger().info(f'mw1: {self.mw1_clicked} mw2: {self.mw2_clicked}')

        msg = Int16()
        msg.data = clicked
        self.publisher_.publish(msg)

        self.mw1_clicked = 0
        self.mw2_clicked = 0

def main(args=None):
    rclpy.init(args=args)

    clickHandler = ClickHandler()

    rclpy.spin(clickHandler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
