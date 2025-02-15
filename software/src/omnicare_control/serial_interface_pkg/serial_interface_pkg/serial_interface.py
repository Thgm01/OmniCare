import rclpy
from rclpy.node import Node
import serial
import struct

from omnicare_msgs.msg import MotorsData
from omnicare_msgs.msg import MotorsPWM

class InterfacePublisher(Node):

    def __init__(self):
        super().__init__('setial_interface_publisher')
        
        # TODO: Colocar .rules
        self.usb_port = "/dev/ttyACM0"  # Substitua pelo nome da sua porta USB
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
                # Converte os valores para bytes (formato little-endian, int16 = 'h' * 3)
                data = struct.pack('<hhh', *values)
                self.ser.write(data)

                self.get_logger().debug(f'PWM Data: {self.pwm_data.data[MotorsPWM.MOTOR_0]:03} / {self.pwm_data.data[MotorsPWM.MOTOR_1]:03} / {self.pwm_data.data[MotorsPWM.MOTOR_2]:03}' )

                raw_data = self.ser.read(24)  # Lê 3 bytes enviados pelo STM32

                if len(raw_data) == 24:
                    # Interpreta como decimais
                    serial_motors_data = struct.unpack('6f', raw_data)  # converte em 6 floats de 4 bytes
                    
                    motors_data_msg.velocity_rpm[MotorsData.MOTOR_0] = serial_motors_data[0]
                    motors_data_msg.position_m[MotorsData.MOTOR_0] = serial_motors_data[1]

                    motors_data_msg.velocity_rpm[MotorsData.MOTOR_1] = serial_motors_data[2]
                    motors_data_msg.position_m[MotorsData.MOTOR_1] = serial_motors_data[3]

                    motors_data_msg.velocity_rpm[MotorsData.MOTOR_2] = serial_motors_data[4]
                    motors_data_msg.position_m[MotorsData.MOTOR_2] = serial_motors_data[5]

                    self.get_logger().info(f'{serial_motors_data[0]}, {serial_motors_data[1]}, {serial_motors_data[2]}, {serial_motors_data[3]}, {serial_motors_data[4]}, {serial_motors_data[5]}')

                    # #publicando a data no tópico
                    self.motors_data_publisher_.publish(motors_data_msg)

                    # Limpa o buffer logo após a leitura
                    self.ser.reset_input_buffer()
            except serial.SerialException as e:
                self.ser = None

    def motors_pwm_callback(self, msg):
        self.pwm_data = msg
        self.get_logger().info(f'PWM Data: {self.pwm_data.data[MotorsPWM.MOTOR_0]:03} / {self.pwm_data.data[MotorsPWM.MOTOR_1]:03} / {self.pwm_data.data[MotorsPWM.MOTOR_2]:03}' )


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