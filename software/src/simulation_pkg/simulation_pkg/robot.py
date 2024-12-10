import rclpy
from rclpy.node import Node


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations


class Robot(Node):
    def __init__(self):
        super().__init__('Robot')
        self.get_logger().info ('Definindo buffer, listener e on_timertimer para acessar as TFs.')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info ("Im on simulation")
        self.timer = self.create_timer(0.1, self.on_timer)
    
    def on_timer(self):
        try:
            self.tf_right = self.tf_buffer.lookup_transform(
                "front_right_wheel_1",
                "base_link",
                rclpy.time.Time())

            _, _, self.right_yaw = tf_transformations.euler_from_quaternion(
                [self.tf_right.transform.rotation.x, self.tf_right.transform.rotation.y, 
                self.tf_right.transform.rotation.z, self.tf_right.transform.rotation.w]) 
            

            self.get_logger().info (
                f'yaw base_link to front_right_wheel_1: {self.right_yaw}')
            
            

        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform base_link to front_right_wheel_1: {ex}')
        
        
        # try:
        #     self.tf_left = self.tf_buffer.lookup_transform(
        #         "left_center_wheel",
        #         "left_leg_base",
        #         rclpy.time.Time())

        #     _, _, self.left_yaw = tf_transformations.euler_from_quaternion(
        #         [self.tf_left.transform.rotation.x, self.tf_left.transform.rotation.y, 
        #         self.tf_left.transform.rotation.z, self.tf_left.transform.rotation.w]) 
            

        #     self.get_logger().info (
        #         f'yaw left_leg_base to left_center_wheel: {self.left_yaw}')
            

        # except TransformException as ex:
        #     self.get_logger().info(
        #     f'Could not transform left_leg_base to left_center_wheel: {ex}')
    
    def run(self):
        self.get_logger().info ('Entrando no loop princial do nó.')
        while(rclpy.ok): 
            Failed to load plugin libgazebo_ros_control.so: libgazebo_ros_control.so: cannot open shared object file: No such file or directory
            rclpy.spin_once(self)









# Função principal
def main(args=None):
    # time.sleep(3)
    rclpy.init(args=args)
    node = Robot()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()  

