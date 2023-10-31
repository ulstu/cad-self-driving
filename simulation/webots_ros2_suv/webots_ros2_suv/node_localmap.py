import rclpy
import numpy as np
import cv2
import traceback
import os
import pathlib
import sensor_msgs.msg
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus, PointCloud2, Imu
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from  .log_server import *
from fastseg import MobileV3Large
from fastseg.image import colorize, blend
from PIL import Image
from ament_index_python.packages import get_package_share_directory
import torch
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from .lib.map_builder import MapBuilder


PACKAGE_NAME = 'webots_ros2_suv'
SENSOR_DEPTH = 40
MAP_DEPTH = 480
FPS = 15 #0.3

class LocalMapNode(Node):
    def __init__(self):
        try:
            super().__init__('node_segmentation')
            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.create_subscription(sensor_msgs.msg.Image, '/vehicle/camera/image_color', self.__on_image_message, qos)
            self.create_subscription(sensor_msgs.msg.Image, '/vehicle/left_wing_camera/image_color', self.__on_left_image_message, qos)
            self.create_subscription(sensor_msgs.msg.Image, '/vehicle/range_finder/image', self.__on_range_image_message, qos)
            self._logger.info('Segmentation Node initialized')
            package_dir = get_package_share_directory(PACKAGE_NAME)
            weights_path = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', 'mobilev3large-lraspp.pt')))
            self.__last_image_time = datetime.now()
            if torch.cuda.is_available():
                self.seg_model = MobileV3Large.from_pretrained(weights_path).cuda().eval()
            else:
                self.seg_model = MobileV3Large.from_pretrained(weights_path).eval()
            self._logger.info('Segmentation Node initialized')

            self.__map_builder = MapBuilder(model_path=f'{package_dir}/resource/yolov8l.pt',
                                            ipm_config=f'{package_dir}/config/ipm_config.yaml')
            cv2.namedWindow('range', 1)
            cv2.namedWindow('composited image', 1)
            cv2.namedWindow('colorized seg', 1)
            cv2.namedWindow('result', 1)
            cv2.namedWindow('depth', 1)
            cv2.namedWindow('original', 1)

        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    def __on_left_image_message(self, data):
        pass

    def __save_image_files(self, labels, composited, source):
        range_image = self.__cur_range_image.copy()
        cv2.imshow("range", range_image)
        cv2.imshow("composited image", composited)
        if cv2.waitKey(25) & 0xFF == ord('q'):
           return
        img_filename = datetime.now().strftime("%Y%m%d-%H%M%S")
        base_path = "/home/hiber/"
        np.save(f"{base_path}{img_filename}_seg.npy", labels)
        np.save(f"{base_path}{img_filename}_range.npy", range_image)
        np.save(f"{base_path}{img_filename}_composited.npy", composited)
        np.save(f"{base_path}{img_filename}_source.npy", source)

    def __on_range_image_message(self, data):
        image = np.frombuffer(data.data, dtype="float32").reshape((data.height, data.width, 1))
        image[image == np.inf] = SENSOR_DEPTH
        image[image == -np.inf] = 0
        self.__cur_range_image = image / SENSOR_DEPTH

    def __process_frame(self, image, image_seg, image_depth):
        try:
            image = self.__map_builder.resize_img(image)
            image_seg = self.__map_builder.resize_img(image_seg.astype('float32'))
            image_depth = self.__map_builder.resize_img(image_depth.astype('float32'))

            results = self.__map_builder.detect_objects(image)
            tbs, widths = self.__map_builder.transform_boxes(results[0].boxes.data.cpu())
            depths = self.__map_builder.calc_box_distance(results[0].boxes.data, image_depth)

            ipm_image = self.__map_builder.generate_ipm(image_seg, is_mono=False, need_cut=False)
            colorized = colorize(ipm_image)
            colorized = np.asarray(colorized)

            pov = (image.shape[0], int(image.shape[1] / 2))
            pov = self.__map_builder.calc_bev_point(pov)
            colorized = colorized[:pov[1], :]

            for i in range(len(tbs)):
                cv2.rectangle(colorized, (int(tbs[i][0] - widths[i] / 2), int(tbs[i][1] - widths[i] / 2)),
                              (int(tbs[i][0] + widths[i] / 2), int(tbs[i][1] + widths[i] / 2)), (255, 0, 0), 2)
                cv2.putText(colorized, f"{i}", (int(tbs[i][0] - widths[i] / 2), int(tbs[i][1] - widths[i] / 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.circle(colorized, pov, 5, (0, 255, 0, 3))
            colorized = cv2.resize(colorized, (500, 500), cv2.INTER_AREA)

            cv2.imshow('original', image)
            cv2.imshow("colorized seg", colorized)
            cv2.imshow("composited image", np.asarray(colorize(image_seg)))
            cv2.imshow("result", self.__map_builder.plot_bboxes(image, results[0].boxes.data, score=False))

            cv2.imshow("depth", self.__map_builder.plot_bboxes(image_depth, results[0].boxes.data, score=False))
            if cv2.waitKey(2000) & 0xFF == ord('q'):
               return

        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    def __on_image_message(self, data):
        try:
            if (datetime.now() - self.__last_image_time).total_seconds() < 1 / FPS:
                return
            image = data.data
            image = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
            analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_RGBA2RGB))
            labels = self.seg_model.predict_one(analyze_image)
            colorized = colorize(labels)
            composited = blend(analyze_image, colorized)
            self.__process_frame(np.asarray(analyze_image), np.asarray(labels), np.asarray(self.__cur_range_image))
            #self.__save_image_files(np.asarray(labels), np.asarray(composited), analyze_image)
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))


def main(args=None):
    try:
        rclpy.init(args=args)
        detector = LocalMapNode()
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
