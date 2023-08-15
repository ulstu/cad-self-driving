import rclpy
import math
import numpy as np
import cv2
import traceback
import os
import pathlib
import sensor_msgs.msg
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus, PointCloud2, Imu
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from  .log_server import *
from fastseg import MobileV3Large
from fastseg.image import colorize, blend
from PIL import Image
from ament_index_python.packages import get_package_share_directory
import cameratransform as ct
import torch

PACKAGE_NAME = 'webots_ros2_suv'
SENSOR_DEPTH = 40
MAP_DEPTH = 480

class SegmentationNode(Node):
    def __init__(self):
        try:
            super().__init__('node_segmentation')
            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.create_subscription(sensor_msgs.msg.Image, '/vehicle/camera/image_color', self.__on_image_message, qos)
            self.create_subscription(sensor_msgs.msg.Image, '/vehicle/range_finder/image', self.__on_range_image_message, qos)
            self._logger.info('Segmentation Node initialized')
            package_dir = get_package_share_directory(PACKAGE_NAME)
            weights_path = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', 'mobilev3large-lraspp.pt')))
            if torch.cuda.is_available():
                self.seg_model = MobileV3Large.from_pretrained(weights_path).cuda().eval()
            else:
                self.seg_model = MobileV3Large.from_pretrained(weights_path).eval()
            self._logger.info('Segmentation Node initialized')
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))


    def __make_bev(self, labels):
        range_image = self.__cur_range_image.copy()
        cv2.imshow("range", range_image)
        if cv2.waitKey(25) & 0xFF == ord('q'):
           return
        # seg_image = labels.copy()
        # result_image = np.zeros((MAP_DEPTH + 1, range_image.shape[1]))
        # result_image[result_image == 0] = -1

        # width, height = seg_image.shape[1], seg_image[0]
        # source = np.float32([[0, 0], [100, 0], [100, 100], [0, 100]])
        # dest = np.float32([[0, 0], [-1000, 0], [-1000, -1000], [0, -1000]])
        # # source = np.float32([[0, int(width / 2)], [height, int(width / 2)], [0, int(width / 2) - 100], [height, 0, int(width / 2) - 100]])
        # # dest = np.float32([[0, int(width / 2)], [height, int(width / 2)], [0, int(width / 2) - 200], [height, 0, int(width / 2) - 80]])
        # homography, _ = cv2.findHomography(source, dest)
        # result_image = cv2.perspectiveTransform(seg_image, homography)

#         for i in range(range_image.shape[1]):
#             for j in range(range_image.shape[0]):
#                 #x_coef = (MAP_DEPTH / (0.5 * MAP_DEPTH + j)) / 3
#                 #new_i = int(i * x_coef)
#                 #new_i = new_i if new_i < 840 else 839
#                 new_i = int(i - (i - range_image.shape[1] / 2) * 1.4)
# #                self._logger.info(f'{new_i}')
#                 result_image[MAP_DEPTH - int(range_image[j, i] * MAP_DEPTH), new_i] = seg_image[j, i]

        # colorized = colorize(result_image)
        # cv2.imshow("map image", np.asarray(colorized))
        # if cv2.waitKey(25) & 0xFF == ord('q'):
        #    return
    def __on_range_image_message(self, data):
        image = np.frombuffer(data.data, dtype="float32").reshape((data.height, data.width, 1))
        image[image == np.inf] = SENSOR_DEPTH
        image[image == -np.inf] = 0
        self.__cur_range_image = image / SENSOR_DEPTH


    def __on_image_message(self, data):

        try:
            image = data.data
            image = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
            analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_RGBA2RGB))
            labels = self.seg_model.predict_one(analyze_image)
            colorized = colorize(labels)
            composited = blend(analyze_image, colorized)
            # cv2.imshow("seg image", np.asarray(analyze_image))
            cv2.imshow("colorized image", np.asarray(colorized))
            cv2.imshow("composited image", np.asarray(composited))
            if cv2.waitKey(25) & 0xFF == ord('q'):
                return
            self.__make_bev(np.asarray(labels))
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))


def main(args=None):
    try:
        rclpy.init(args=args)
        detector = SegmentationNode()
        rclpy.spin(detector)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except  Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
