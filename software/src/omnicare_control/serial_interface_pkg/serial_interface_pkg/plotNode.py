import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque

class RealtimePlotNode(Node):
    def __init__(self):
        super().__init__('realtime_plot_node')
        
        self.declare_parameter('topic1', 'topic1')
        self.declare_parameter('topic2', 'topic2')
        
        self.topic1 = self.get_parameter('topic1').get_parameter_value().string_value
        self.topic2 = self.get_parameter('topic2').get_parameter_value().string_value
        
        self.sub1 = self.create_subscription(Float32, self.topic1, self.callback_topic1, 10)
        self.sub2 = self.create_subscription(Float32, self.topic2, self.callback_topic2, 10)
        self.maxlen = 3
        self.data1 = deque(maxlen=self.maxlen)
        self.data2 = deque(maxlen=self.maxlen)
        self.time = deque(maxlen=self.maxlen)
        self.counter1 = 0
        self.counter2 = 0
        
    def callback_topic1(self, msg):
        if self.counter1 <= self.maxlen:
            self.data1.append(msg.data)
        self.counter1 += 1
        self.check_and_plot()
    
    def callback_topic2(self, msg):
        if self.counter2 <= self.maxlen:
            self.data2.append(msg.data)
        self.counter2 += 1
        self.check_and_plot()
        
    def check_and_plot(self):
        if self.counter1 >= self.maxlen and self.counter2 >= self.maxlen:
            self.plot_graph()
        
    def plot_graph(self):
        plt.figure()
        plt.plot(range(3), list(self.data1), label=self.topic1)
        plt.plot(range(3), list(self.data2), label=self.topic2)
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
