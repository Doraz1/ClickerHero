import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

robot_ns = os.environ['robot_namespace']
clicker_ind = os.environ['clicker_index']

class RecoveryBehaviorNav(Node):
    def __init__(self, nav):
        super().__init__('recovery_behavior_navigator')

        self.nav = nav

        self.prev_collided_1 = False
        self.prev_collided_2 = False

        self.moveClockwise = True
        self.get_logger().info('Successfully created recovery behavior navigator')

        self.recovery_turn_speed = 0.8
        self.recovery_linear_speed = 0.03

    def check_collision(self):
        new_collision_1 = self.prev_collided_1 == False and self.nav.collided_1 == True
        new_collision_2 = self.prev_collided_2 == False and self.nav.collided_2 == True

        'Both sensors up front'
#        if self.nav.collided_1:
        if new_collision_1:
            self.moveClockwise = True
            self.get_logger().info("-----------------recovering from collision -  moving clockwise--------------")
#        elif self.nav.collided_2:
        elif new_collision_2:
            self.moveClockwise = True
            self.get_logger().info("-----------------recovering from collision -  moving clockwise--------------")
#            self.get_logger().info("-----------------recovering from collision -  moving counter-clockwise--------------")

        self.prev_collided_1 = self.nav.collided_1
        self.prev_collided_2 = self.nav.collided_2

        return self.nav.collided_1 or self.nav.collided_2
#        return self.nav.collided_1, self.nav.collided_2  # front and back

    def get_cmds(self):
        collided = self.prev_collided_1 or self.prev_collided_2
        mv_x, mv_rz = 0.0, 0.0
#        sign = 1 if reverse else -1
        sign = 1 if self.moveClockwise else -1
        if collided:
            'Front and back'
#            mv_x = sign*self.recovery_linear_speed  # front and back only, otherwise comment this out
 #           mv_rz = self.recovery_turn_speed
            'Both up front'
            mv_rz = sign*self.recovery_turn_speed

        return mv_x, mv_rz



def main(args=None):
    rclpy.init(args=args)

    pub = RecoveryBehaviorNav(None)

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()
