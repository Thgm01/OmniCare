import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque
from omnicare_msgs.msg import MotorsData,MotorsPWM


class RealtimePlotNode(Node):
    def __init__(self):
        super().__init__('realtime_plot_node')
        
        self.declare_parameter('motors_data', 'motors_data')
        self.declare_parameter('motors_pwm', 'motors_pwm')
        
        self.get_speed = self.get_parameter('motors_data').get_parameter_value().string_value
        self.set_speed = self.get_parameter('motors_pwm').get_parameter_value().string_value
        
        self.sub1 = self.create_subscription(MotorsData, self.get_speed, self.callback_get_speed, 10)
        self.sub2 = self.create_subscription(MotorsPWM, self.set_speed, self.callback_set_speed, 10)
        self.maxlen = 50
        self.data1 = deque(maxlen=self.maxlen)
        self.data2 = deque(maxlen=self.maxlen)
        self.time = deque(maxlen=self.maxlen)
        self.counter1 = 0
        self.counter2 = 0

        self.command_speed = False
        
    def callback_get_speed(self, msg):
        # print(msg)
        if self.command_speed:
            if self.counter1 <= self.maxlen:
                self.data1.append(msg.motor_speed)
                self.counter1 += 1
                self.check_and_plot()
    
    def callback_set_speed(self, msg):
        self.command_speed = True
        # print(msg)
        if self.counter2 <= self.maxlen:
            self.data2.append(msg.data[0])
        self.counter2 += 1
        self.check_and_plot()
        
    def check_and_plot(self):
        if self.counter1 >= self.maxlen and self.counter2 >= self.maxlen:
            self.plot_graph()
        
    def plot_graph(self):
        plt.figure()
        plt.plot(range(self.maxlen), list(self.data1), label=self.get_speed)
        plt.plot(range(self.maxlen), list(self.data2), label=self.set_speed)
        plt.xlabel('Amostras')
        plt.ylabel('Valores')
        plt.axhline(0, color='black', linewidth=1, linestyle='--')  # Linha horizontal no eixo y = 0
        plt.axvline(0, color='black', linewidth=1, linestyle='--')  # Linha vertical no eixo x = 0
        plt.legend()
        plt.title('Gráfico de Dados Acumulados')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = RealtimePlotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
