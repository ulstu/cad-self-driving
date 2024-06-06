import os
import struct
from datetime import datetime

import yaml
import sensor_msgs.msg
from ament_index_python.packages import get_package_share_directory
from PIL import Image

from .lib import point_cloud2
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
from .log_server import set_lidar_hz
import traceback
import pathlib
import pickle
import os
import time
from traceback import format_exc
from .lib.param_loader import ParamLoader


SENSOR_DEPTH = 40
FPS = 1

DATACAMERA = f"{os.path.expanduser('~')}/ros2_ws/data/camera/"
DATALIDAR = f"{os.path.expanduser('~')}/ros2_ws/data/lidar/"


class NodeVisual(Node):
    def __init__(self):
        try:
            super().__init__('node_ego_controller')
            self._logger.info(f'Node Visual Started')
            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            param = ParamLoader()

            self.create_subscription(sensor_msgs.msg.Image, param.get_param("front_image_topicname"), self.__on_image_message, qos)
            self.create_subscription(PointCloud2, param.get_param("lidar_topicname"), self.__on_point_cloud, qos)

            self.__last_image_time = datetime.now()
            self.__lidar_last_time = datetime.now()

        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    # @timeit
    def __on_image_message(self, data):
        if (datetime.now() - self.__last_image_time).total_seconds() < 1 / FPS:
            return
        self.__last_image_time = datetime.now()
        image = data.data
        image = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
        analyze_image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
        img_filename = datetime.now().strftime("%Y%m%d-%H%M%S") + ".png"
        cv2.imwrite(os.path.join(DATACAMERA, img_filename), analyze_image)
        self._logger.info(f'saved: {os.path.join(DATACAMERA, img_filename)}')
        # self.__world_model.rgb_image = np.asarray(analyze_image)
        # # вызов текущих обработчиков данных
        # self.__world_model = self.__fsm.on_data(self.__world_model, source="__on_image_message")
        # # вызов обработки состояний с текущими данными
        # self.__fsm.on_event(None, self.__world_model)

    def set_last_time(self, last_time, set_func):
        cur_time = self.get_clock().now()

        time_diff = (cur_time - last_time).nanoseconds / 1e9
        if time_diff == 0.0:
            time_diff = 0.0
        else:
            time_diff = 1 / time_diff
        set_func(time_diff)
        return cur_time

    def __on_point_cloud(self, data):
        try:
            if (datetime.now() - self.__lidar_last_time).total_seconds() < 1 / FPS:
                return
            self.__lidar_last_time = datetime.now()
            # assert isinstance(data, PointCloud2)
            self._logger.info(f'type: {type(data)}')
            filename = datetime.now().strftime("%Y%m%d-%H%M%S")
            try:
                points = point_cloud2.read_points(data)
                lst = []
                for p in points:
                    lst.append(np.array(p))
                a = np.array(lst)
                np.save(os.path.join(DATALIDAR, filename), a)
                self._logger.info(f'saved: {os.path.join(DATALIDAR, filename)}')
            except Exception as err:
                self._logger.info(f'Exception occurred: {type(err).__name__}, Message: {format_exc()}')
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
