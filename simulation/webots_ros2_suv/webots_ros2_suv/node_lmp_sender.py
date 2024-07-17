import json
from rclpy.node import Node
import rclpy
import socket
from std_msgs.msg import String
import traceback

UDP_SEND_IP = '192.168.95.1' # 192.168.95.248
UDP_SEND_PORT = 5666


class NodeLMPSender(Node):
    def __init__(self):
        try:
            super().__init__('node_lmp_sender')
            self.create_subscription(String, 'lmp_send', self.__lmp_send_callback, 1)
            self.socket_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.lmp_data = None
        except Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    def __lmp_send_callback(self, lmp_data):
        self.lmp_data = lmp_data.data
        self.lmp_data = (str(len(lmp_data.data)).zfill(6)) + lmp_data.data



def main(args=None):
    try:
        rclpy.init(args=args)
        lmp_sender = NodeLMPSender()

        while rclpy.ok():
            try:
                rclpy.spin_once(lmp_sender, timeout_sec=0.5)

                if lmp_sender.lmp_data:
                    # print(lmp_sender.lmp_data)s
                    lmp_sender.socket_send.sendto(lmp_sender.lmp_data.encode(), (UDP_SEND_IP, UDP_SEND_PORT))
                    lmp_sender.lmp_data = {}
            except KeyboardInterrupt:
                pass
            
        lmp_sender.destroy_node()
        rclpy.shutdown()
    except Exception as error:
        print(''.join(traceback.TracebackException.from_exception(error).format()))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
