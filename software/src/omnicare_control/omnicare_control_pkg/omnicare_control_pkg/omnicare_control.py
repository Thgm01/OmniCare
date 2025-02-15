import rclpy
from rclpy.node import Node

from omnicare_msgs.msg import MotorsPWM
from geometry_msgs.msg import Twist

import numpy as np


class OmniController(Node):
    def __init__(self):
        super().__init__('omni_controller')
        self.teleop_subscription = self.create_subscription(Twist,
                                                            'cmd_vel',
                                                            self.teleop_callback,
                                                            10)
        self.teleop_subscription

        self.motor_set_pwm_publisher = self.create_publisher(MotorsPWM, 'motors_pwm',
                                                             10)
        self.motor_set_pwm_publisher


    def teleop_callback(self, msg):
        motors_pwm_msg = MotorsPWM()

        v_x = -msg.linear.y
        v_y = msg.linear.x
        w_z = msg.angular.z

        l = 0.249375 
        r = 0.06
        transformation_matrix = 1 / r * np.array([[-1             , 0               , l],
                                                  [np.sin(np.pi/6), -np.cos(np.pi/6), l],
                                                  [np.sin(np.pi/6),  np.cos(np.pi/6), l]])

        motors_velocity = transformation_matrix @ np.array([v_x, v_y, w_z])
        motors_velocity = 255/(max(abs(motors_velocity))+ 1e-10) * motors_velocity

        print(motors_velocity)

        motors_pwm_msg.data[0] = int(motors_velocity[0])
        motors_pwm_msg.data[1] = int(motors_velocity[1])
        motors_pwm_msg.data[2] = int(motors_velocity[2])

        self.motor_set_pwm_publisher.publish(motors_pwm_msg)



def main(args=None):
    rclpy.init(args=args)

    omni_controller = OmniController()

    rclpy.spin(omni_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    omni_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()