import rclpy
from rclpy.node import Node
import serial
import struct

from std_msgs.msg import Int32MultiArray


class InterfacePublisher(Node):

    def __init__(self):
        super().__init__('interface_publisher')
        
        self.usb_port = "/dev/ttyACM0"  # Substitua pelo nome da sua porta USB
        self.baud_rate = 115200           # Taxa de comunicação

        self.publisher_ = self.create_publisher(Int32MultiArray, 'value', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int32MultiArray()

        try:
            # Inicializa a conexão serial
            with serial.Serial(self.usb_port, self.baud_rate, timeout=1) as ser:
                print(f"Conectado à porta {self.usb_port} com baud rate {self.baud_rate}")
                print("Pressione Ctrl+C para sair.\n")

                while rclpy.ok():
                    # 
                    ser.write([256, 99, 99])
                    # Lê uma linha de dados da USB
                    raw_data = ser.read(24)  # Lê 3 bytes enviados pelo STM32
                    # raw_data = ser.readline()  # Lê 3 bytes enviados pelo STM32
                    if len(raw_data) == 24:
                        # Interpreta como decimais
                        decimal_values = struct.unpack('6f', raw_data)  # converte em 6 floats de 4 bytes
                        # print(f"Dado recebido (bin): {raw_data}")
                        print(f"Dado convertido (dec): {decimal_values}")
                        
                        # Enviando de volta o valor
                        ser.write(raw_data)

                        # #publicando a data no tópico
                        msg.data = {1, 2,3 }
                        self.publisher_.publish(msg)

                        # Limpa o buffer logo após a leitura
                        ser.reset_input_buffer()

        except serial.SerialException as e:
            print(f"Erro ao acessar a porta {self.usb_port}: {e}")
        except KeyboardInterrupt:
            print("\nEncerrando o programa.")

        self.get_logger().info('Publishing: "%s"' % msg.data)


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