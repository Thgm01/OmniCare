import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Int32


class InterfacePublisher(Node):

    def __init__(self):
        super().__init__('interface_publisher')
        
        self.usb_port = "/dev/ttyUSB0"  # Substitua pelo nome da sua porta USB
        self.baud_rate = 115200           # Taxa de comunicação

        self.publisher_ = self.create_publisher(Int32, 'value', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int32()

        try:
            # Inicializa a conexão serial
            with serial.Serial(self.usb_port, self.baud_rate, timeout=1) as ser:
                print(f"Conectado à porta {self.usb_port} com baud rate {self.baud_rate}")
                print("Pressione Ctrl+C para sair.\n")

                while rclpy.ok():
                    # Lê uma linha de dados da USB
                    raw_data = ser.read(1)  # Lê como bytes
                    if raw_data:
                        # Interpreta como hexadecimal
                        hex_data = raw_data.hex()  # Dados como hexadecimal
                        decimal_values = int(hex_data, 16)
                        print(f"Dado recebido (hex): {hex_data}")
                        print(f"Dado convertido (dec): {decimal_values}")
                        
                        # Enviando de volta o valor
                        ser.write(raw_data)

                        #publicando a data no tópico
                        msg.data = decimal_values
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