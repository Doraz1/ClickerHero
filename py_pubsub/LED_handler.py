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

import sys
from py_pubsub.matrix import LEDMatrix
from std_msgs.msg import Int16

from threading import Thread
import time
import math


clicker_id = int(os.environ['clicker_index']) # get index, convert string to int

class LEDHandler(Node):

    def __init__(self):
        super().__init__('LED_handler')

        #led subscriber
        ledSubName = 'LED_cmd'
        self.subscription = self.create_subscription(Int16, ledSubName, self.led_callback, 10)

        LEDStatePubName = 'led_state'
        self.ledStatePub = self.create_publisher(Int16, LEDStatePubName, 10)

        #clicked subscriber
        clickSubName = 'clicked'
        self.clickSubscription = self.create_subscription(Int16, clickSubName, self.click_callback, 10)

        self.isClicked = 0

        self.dt = 0.1  # sec
        timer = self.create_timer(self.dt, self.publish_led_state)


        self.difficulty_dict = { 
                                0: {'initial_on_time':2.2, 'rate':8, 'num_blinks':3, 'allow_premature_turnoff':True},
                                1: {'initial_on_time':2.0, 'rate':8, 'num_blinks':3, 'allow_premature_turnoff':True},
                                2: {'initial_on_time':1.8, 'rate':8, 'num_blinks':3, 'allow_premature_turnoff':True},
                                3: {'initial_on_time':1.6, 'rate':8, 'num_blinks':3, 'allow_premature_turnoff':True},
                                4: {'initial_on_time':1.4, 'rate':8, 'num_blinks':2, 'allow_premature_turnoff':True},
                                5: {'initial_on_time':1.2, 'rate':8, 'num_blinks':2, 'allow_premature_turnoff':True},
                                6: {'initial_on_time':1.0, 'rate':8, 'num_blinks':2, 'allow_premature_turnoff':True},
                                7: {'initial_on_time':1.0, 'rate':8, 'num_blinks':1, 'allow_premature_turnoff':True},
                                8: {'initial_on_time':8.0, 'rate':8, 'num_blinks':1, 'allow_premature_turnoff':True},  # simon says slow recall phase
                                9: {'initial_on_time':1.5, 'rate':8, 'num_blinks':1, 'allow_premature_turnoff':False}}  # simon says fast teaching phase

        self.mat = LEDMatrix(self.get_logger(), self)

        self.ledStateDict =  {0: 'Off', 1: 'Green', 2: 'Blue', 3: 'Orange', 4: 'Red',5: 'White', 6: 'RES1', 7:'RES2', 8:'Blinking Success',  9: 'Blinking Failure'}

        self.off_state = self.get_led_state_from_dict("Off")
        self.green_state = self.get_led_state_from_dict("Green")
        self.blue_state = self.get_led_state_from_dict("Blue")
        self.orange_state = self.get_led_state_from_dict("Orange")
        self.red_state = self.get_led_state_from_dict("Red")
        self.white_state = self.get_led_state_from_dict("White")
        self.blink_success_state = self.get_led_state_from_dict("Blinking Success")
        self.blink_failure_state = self.get_led_state_from_dict("Blinking Failure")

        self.stateToColorDict = {
                                 self.green_state: LEDMatrix.GREEN,
                                 self.blue_state: LEDMatrix.BLUE,
                                 self.orange_state: LEDMatrix.ORANGE,
                                 self.red_state: LEDMatrix.RED,
                                 self.white_state: LEDMatrix.WHITE,
                                                                   }
        self.update_led_state(self.off_state)
        self.premature_turnoff_enabled = True
        self.get_logger().info('Successfully created LED handler')
    def get_led_state_from_dict(self, state):
        state_ind = list(self.ledStateDict.keys())[list(self.ledStateDict.values()).index(state)]
  #      self.get_logger().info(f'new state: {state_ind}: {state}')
        return state_ind

    def update_led_state(self, val):
        self.led_state = val
   #     self.get_logger().info(f'cmd prev updated to: {self.cmd_prev}')

    def led_callback(self, msg):
        # parse input each time the game sends an LED activation command

        led_commands = []
        num = msg.data

        while num != 0:
            num, remainder = divmod(num, 10)
            led_commands.append(remainder)

        # from smallest (ones) digit to largest(100000s) digit
        #10s digit is the difficulty level, ones digit is the request
        try:
            cmd = led_commands[0]  # led state dict defines this value
            if len(led_commands) == 1:
                difficulty = 0
            else:
                difficulty = int(led_commands[1])  # difficulty dict defines this value
#            self.get_logger().info(f'LED cmd: {cmd} dif: {difficulty}')
        except:
            self.get_logger().info(f'led command invalid: {led_commands} raw received: {msg.data}')
            return

        if cmd == self.off_state:
            'Turn LED off'
            self.mat.stop_execution()
#            self.mat.turn_off()
            self.get_logger().info(f'LED state turned off by LED handler')
        else:
            dif_dict = self.difficulty_dict[difficulty]
            self.lightup_matrix(cmd, dif_dict)

        self.update_led_state(cmd)

    def lightup_matrix(self, cmd, dif_dict):
        on_time = dif_dict['initial_on_time']
        r = dif_dict['rate']
        n_blinks = dif_dict['num_blinks']
        self.premature_turnoff_enabled = dif_dict['allow_premature_turnoff']

#        self.get_logger().info(f'Lighting LED mat on by LED handler')

        if not self.mat.isTurnedOn:
            'finished last lightup. otherwise its still lit up.'
            self.update_led_state(self.off_state)

#        self.get_logger().info(f'led_state: {self.led_state}, cmd: {cmd}')

        requested_green_lightup = (cmd == self.green_state)
        requested_blue_lightup = (cmd == self.blue_state)
        requested_orange_lightup = (cmd == self.orange_state)
        requested_red_lightup = (cmd == self.red_state)
        requested_white_lightup = (cmd == self.white_state)

        requested_lightup_cond = (requested_green_lightup or requested_blue_lightup or requested_orange_lightup or requested_red_lightup or requested_white_lightup)

        if requested_lightup_cond:
            state = cmd  # cmd  is a numerical representation of states
            color = self.stateToColorDict[state]
            self.get_logger().info(f'requested {self.ledStateDict[state]} lightup')

            self.t = Thread(target = lambda: self.mat.blink(color, initial_on_time=on_time, rate=r, num_blinks=n_blinks))
            self.t.start()
        else:
            self.get_logger().info(f'not blinking, and didnt request lightup. publishing {self.off_state}')


    def publish_led_state(self):
        'Handle matrix turning off after blinking on its own'
        if not self.mat.isTurnedOn:
            self.update_led_state(self.off_state)

        msg = Int16()
        msg.data = self.led_state
        self.ledStatePub.publish(msg)
#        self.get_logger().info(f'Publishing LED state: {self.led_state}')

    def click_callback(self, msg):
#        self.get_logger().info('LED handler received click message')

        prev_isClicked = self.isClicked
        self.isClicked = msg.data
        clicked = prev_isClicked == 0 and self.isClicked == 1
        #self.get_logger().info(f'prev clicked: {prev_isClicked} | clicked: {self.isClicked} => clicked={clicked}, premature_ena={self.premature_turnoff_enabled}')

        if clicked and self.premature_turnoff_enabled:
            self.mat.stop_execution()


def main(args=None):
    rclpy.init(args=args)

    LED_listener = LEDHandler()

    rclpy.spin(LED_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()
