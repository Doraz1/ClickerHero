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
from std_msgs.msg import Int16

from threading import Thread
import time
import math


class IsAlivePublisher(Node):

    def __init__(self):
        super().__init__('livelihood_subscriber')
        self.clicker_ind = int(os.environ["clicker_index"])

        livelihoodSubName = 'alive'
        self.alive_pub = self.create_publisher(Int16, livelihoodSubName, 10)

        timer_period = 1
        self.create_timer(timer_period, self.publish_alive_state)

    def publish_alive_state(self):
#        self.get_logger().info(f'robot {self.clicker_ind} alive')
        msg = Int16()
        msg.data = 1
        self.alive_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    livelihood_pub = IsAlivePublisher()

    rclpy.spin(livelihood_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()
