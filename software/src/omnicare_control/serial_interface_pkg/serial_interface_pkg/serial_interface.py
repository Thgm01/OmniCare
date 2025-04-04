import rclpy
from rclpy.node import Node
import serial
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random  


from omnicare_msgs.msg import MotorsData
from omnicare_msgs.msg import MotorsPWM

class InterfacePublisher(Node):

    def __init__(self):
        super().__init__('setial_interface_publisher')
        
        # TODO: Colocar .rules
        self.usb_port = "/dev/usb-user"  # Substitua pelo nome da sua porta USB
        self.baud_rate = 115200           # Taxa de comunicação

        self.motors_data_publisher_ = self.create_publisher(MotorsData, 'motors_data', 10)
        self.motors_data_publisher_

        self.motors_pwm_subscriber_ = self.create_subscription(MotorsPWM, 'motors_pwm', self.motors_pwm_callback, 10)
        self.motors_pwm_subscriber_
        self.pwm_data = MotorsPWM()

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        try:
            self.ser = serial.Serial(self.usb_port, self.baud_rate, timeout=1)
        except serial.SerialException as e:
            print(f"Erro ao acessar a porta {self.usb_port}: {e}")
            self.ser = None

        # Listas para armazenar os dados
        self.x_data = []
        self.y_data_1 = []
        self.y_data_2 = []

        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], 'r-', label="Sensor 1")  # Linha vermelha
        self.line2, = self.ax.plot([], [], 'b-', label="Sensor 2")  # Linha azul


        # Configurações do gráfico
        self.ax.set_xlim(0, 100)  # Eixo X (tempo)
        self.ax.set_ylim(0, 10)   # Eixo Y (valores)
        self.ax.set_xlabel("Tempo")
        self.ax.set_ylabel("Valor")
        self.ax.set_title("Gráfico em Tempo Real - Duas Entradas")
        self.ax.legend()






    def timer_callback(self):
        motors_data_msg = MotorsData()

        if(self.ser == None):
            try:
                self.ser = serial.Serial(self.usb_port, self.baud_rate, timeout=1)
                print(f"Conectado à porta {self.usb_port} com baud rate {self.baud_rate}")
                print("Pressione Ctrl+C para sair.\n")

            except serial.SerialException as e:
                print(f"Erro ao acessar a porta {self.usb_port}: {e}")
                self.ser = None
        else:
            try:
                values = (self.pwm_data.data[MotorsPWM.MOTOR_0], self.pwm_data.data[MotorsPWM.MOTOR_1], self.pwm_data.data[MotorsPWM.MOTOR_2])
                
                # TODO: Reesrever essa função para ele conseguir mandar também os valores
                # data = struct.pack('<hhh', *values)
                # self.ser.write(data)

                self.get_logger().debug(f'PWM Data: {self.pwm_data.data[MotorsPWM.MOTOR_0]:03} / {self.pwm_data.data[MotorsPWM.MOTOR_1]:03} / {self.pwm_data.data[MotorsPWM.MOTOR_2]:03}' )

                # raw_data = self.ser.read(12)  # Lê 3 bytes enviados pelo STM32
                linha = self.ser.readline().decode('utf-8').strip()  # Lê 3 bytes enviados pelo STM32
                self.get_logger().info(f'{linha}')
                valores = list(map(int, linha.split()))
                print(valores)

                # #publicando a data no tópico
                self.motors_data_publisher_.publish(motors_data_msg)

                # Limpa o buffer logo após a leitura
                self.ser.reset_input_buffer()
            except serial.SerialException as e:
                self.ser = None

    def motors_pwm_callback(self, msg):
        self.pwm_data = msg
        self.get_logger().info(f'PWM Data: {self.pwm_data.data[MotorsPWM.MOTOR_0]:03} / {self.pwm_data.data[MotorsPWM.MOTOR_1]:03} / {self.pwm_data.data[MotorsPWM.MOTOR_2]:03}' )
        
        if(self.ser == None):
            try:
                self.ser = serial.Serial(self.usb_port, self.baud_rate, timeout=1)
                print(f"Conectado à porta {self.usb_port} com baud rate {self.baud_rate}")
                print("Pressione Ctrl+C para sair.\n")

            except serial.SerialException as e:
                print(f"Erro ao acessar a porta {self.usb_port}: {e}")
                self.ser = None
        else:
            """Callback function that sends the received ROS2 message over serial."""
            try:
                # Convert ROS2 message to a simple string (CSV format)
                pwm_string = f"m {self.pwm_data.data[MotorsPWM.MOTOR_0]} {self.pwm_data.data[MotorsPWM.MOTOR_1]} {self.pwm_data.data[MotorsPWM.MOTOR_2]}\n"

                # Send over serial
                self.ser.write(pwm_string.encode())

                self.get_logger().info(f"Sent over serial: {pwm_string.strip()}")

                # Animação do gráfico
                ani = animation.FuncAnimation(self.fig, self.update, interval=100)

                plt.show()


            except Exception as e:
                self.get_logger().error(f"Error sending data: {str(e)}")


    def update(self,frame):
        self.x_data.append(frame)  
        self.y_data_1.append(self.pwm_data.data[MotorsPWM.MOTOR_0])  # Simula entrada 1
        self.y_data_2.append(random.uniform(2, 8))  # Simula entrada 2

        # Mantém o tamanho fixo da janela de tempo
        if len(self.x_data) > 100:
            self.x_data.pop(0)
            self.y_data_1.pop(0)
            self.y_data_2.pop(0)

        self.line1.set_data(self.x_data, self.y_data_1)
        self.line2.set_data(self.x_data, self.y_data_2)
        
        self.ax.set_xlim(max(0, frame-100), frame)  # Move o eixo X conforme o tempo avança
        return self.line1, self.line2



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = InterfacePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()