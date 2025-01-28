import rclpy
from rclpy.node import Node
import serial
import struct

from robot_msgs.msg import MotorsData
from robot_msgs.msg import MotorsPWM

class InterfacePublisher(Node):

    def __init__(self):
        super().__init__('setial_interface_publisher')
        
        self.usb_port = "/dev/ttyACM0"  # Substitua pelo nome da sua porta USB
        self.baud_rate = 115200           # Taxa de comunicação

        self.motors_data_publisher_ = self.create_publisher(MotorsData, 'motors_data', 10)
        self.motors_data_publisher_

        self.motors_pwm_subscriber_ = self.create_subscription(MotorsPWM, 'motors_pwm', self.motors_pwm_callback, 10)
        self.motors_pwm_subscriber_
        self.pwm_data = MotorsPWM()

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        motors_data_msg = MotorsData()

        try:
            # Inicializa a conexão serial
            with serial.Serial(self.usb_port, self.baud_rate, timeout=1) as ser:
                print(f"Conectado à porta {self.usb_port} com baud rate {self.baud_rate}")
                print("Pressione Ctrl+C para sair.\n")

                while rclpy.ok():

                    # TODO: Alterar para enviar as variaveis como int16
                    ser.write([256, 99, 99])
                    self.get_logger().debug(f'PWM Data: {self.pwm_data.data[MotorsPWM.MOTOR_1]:03} / {self.pwm_data.data[MotorsPWM.MOTOR_2]:03} / {self.pwm_data.data[MotorsPWM.MOTOR_3]:03}' )

                    raw_data = ser.read(24)  # Lê 3 bytes enviados pelo STM32

                    if len(raw_data) == 24:
                        # Interpreta como decimais
                        serial_motors_data = struct.unpack('6f', raw_data)  # converte em 6 floats de 4 bytes
                        
                        motors_data_msg.velocity_rpm[MotorsData.MOTOR_1] = serial_motors_data[0]
                        motors_data_msg.position_m[MotorsData.MOTOR_1] = serial_motors_data[1]

                        motors_data_msg.velocity_rpm[MotorsData.MOTOR_2] = serial_motors_data[2]
                        motors_data_msg.position_m[MotorsData.MOTOR_2] = serial_motors_data[3]

                        motors_data_msg.velocity_rpm[MotorsData.MOTOR_3] = serial_motors_data[4]
                        motors_data_msg.position_m[MotorsData.MOTOR_3] = serial_motors_data[5]

                        # #publicando a data no tópico
                        self.motors_data_publisher_.publish(motors_data_msg)

                        # Limpa o buffer logo após a leitura
                        ser.reset_input_buffer()


        except serial.SerialException as e:
            print(f"Erro ao acessar a porta {self.usb_port}: {e}")
        except KeyboardInterrupt:
            print("\nEncerrando o programa.")

    def motors_pwm_callback(self, msg):
        self.pwm_data = msg

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