import yaml
import sensor_msgs.msg
from ament_index_python.packages import get_package_share_directory
from PIL import Image
from .lib.world_model import WorldModel
from .lib.finite_state_machine import FiniteStateMachine
from .lib.map_server import start_web_server, MapWebServer
import threading

import rclpy
import numpy as np
import cv2
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus, PointCloud2, Imu
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from .lib.orientation import quaternion_from_euler, euler_from_quaternion
import traceback
from log_server import set_lidar_hz

SENSOR_DEPTH = 40


class NodeVisual(Node):
    def __init__(self):
        try:
            super().__init__('node_ego_controller')
            self._logger.info(f'Node Visual Started')
            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.create_subscription(Odometry, '/odom', self.__on_gps_message, qos)
            self.create_subscription(sensor_msgs.msg.Image, '/vehicle/camera/image_color', self.__on_image_message, qos)
            self.create_subscription(PointCloud2, '/vehicle/range_finder/point_cloud', self.__on_point_cloud, qos)

            self.__ackermann_publisher = self.create_publisher(AckermannDrive, 'cmd_ackermann', 1)

            self.__world_model = WorldModel()
            package_dir = get_package_share_directory("webots_ros2_suv")

            self.__fsm = FiniteStateMachine(f'{package_dir}/config/ego_states/robocross.yaml', self)

            # Примеры событий
            self.__fsm.on_event("start_move")
            # self.__fsm.on_event("stop")
            # self.__fsm.on_event("reset")

            # загружаем размеченную глобальную карту, имя файла которой берем из конфига map_config.yaml
            with open(f'{package_dir}/config/map_config.yaml') as file:
                with open(f'{package_dir}/config/global_maps/{yaml.full_load(file)["mapfile"]}') as mapdatafile:
                    self.__world_model.load_map(yaml.safe_load(mapdatafile))

            self.start_web_server()

        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    def start_web_server(self):
        self.__ws = MapWebServer(log=self._logger.info)
        threading.Thread(target=start_web_server, args=[self.__ws]).start()

    # @timeit
    def __on_image_message(self, data):
        image = data.data
        image = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
        analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_RGBA2RGB))

        # self.__world_model.rgb_image = np.asarray(analyze_image)
        # # вызов текущих обработчиков данных
        # self.__world_model = self.__fsm.on_data(self.__world_model, source="__on_image_message")
        # # вызов обработки состояний с текущими данными
        # self.__fsm.on_event(None, self.__world_model)

    def __on_point_cloud(self, data):
        try:
            self.__lidar_last_time = self.set_last_time(self.__lidar_last_time, set_lidar_hz)
            p = data
            print(PointCloud2(p))
            # p.header.stamp = self.get_clock().now().to_msg()
            # p.header.frame_id = 'base_link'
            # self.__pc_publisher.publish(p)
        except  Exception as err:
            print(f'{str(err)}')


def main(args=None):
    try:
        rclpy.init(args=args)
        path_controller = NodeVisual()
        rclpy.spin(path_controller)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
