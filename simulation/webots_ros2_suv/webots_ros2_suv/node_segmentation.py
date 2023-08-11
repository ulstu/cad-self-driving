import rclpy
import math
import numpy as np
import cv2
import sensor_msgs.msg
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus, PointCloud2, Imu
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from  .log_server import *
import traceback
from fastseg import MobileV3Large
from fastseg.image import colorize, blend
from PIL import Image
import os
import pathlib
from ament_index_python.packages import get_package_share_directory
import cameratransform as ct

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
            self.seg_model = MobileV3Large.from_pretrained(weights_path).cuda().eval()
            self._logger.info('Segmentation Node initialized')
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    def scale_to_255(self, a, min, max, dtype=np.uint8):
        """ Scales an array of values from specified min, max range to 0-255
            Optionally specify the data type of the output (default is uint8)
        """
        return (((a - min) / float(max - min)) * 255).astype(dtype)

    # ==============================================================================
    #                                                         POINT_CLOUD_2_BIRDSEYE
    # ==============================================================================
    def point_cloud_2_birdseye(self, points,
                               res=0.1,
                               side_range=(-10., 10.),  # left-most to right-most
                               fwd_range=(-10., 10.),  # back-most to forward-most
                               height_range=(-2., 2.),  # bottom-most to upper-most
                               ):
        """ Creates an 2D birds eye view representation of the point cloud data.

        Args:
            points:     (numpy array)
                        N rows of points data
                        Each point should be specified by at least 3 elements x,y,z
            res:        (float)
                        Desired resolution in metres to use. Each output pixel will
                        represent an square region res x res in size.
            side_range: (tuple of two floats)
                        (-left, right) in metres
                        left and right limits of rectangle to look at.
            fwd_range:  (tuple of two floats)
                        (-behind, front) in metres
                        back and front limits of rectangle to look at.
            height_range: (tuple of two floats)
                        (min, max) heights (in metres) relative to the origin.
                        All height values will be clipped to this min and max value,
                        such that anything below min will be truncated to min, and
                        the same for values above max.
        Returns:
            2D numpy array representing an image of the birds eye view.
        """
        # EXTRACT THE POINTS FOR EACH AXIS
        x_points = points[:, 0]
        y_points = points[:, 1]
        z_points = points[:, 2]

        # FILTER - To return only indices of points within desired cube
        # Three filters for: Front-to-back, side-to-side, and height ranges
        # Note left side is positive y axis in LIDAR coordinates
        f_filt = np.logical_and((x_points > fwd_range[0]), (x_points < fwd_range[1]))
        s_filt = np.logical_and((y_points > -side_range[1]), (y_points < -side_range[0]))
        filter = np.logical_and(f_filt, s_filt)
        indices = np.argwhere(filter).flatten()

        # KEEPERS
        x_points = x_points[indices]
        y_points = y_points[indices]
        z_points = z_points[indices]

        # CONVERT TO PIXEL POSITION VALUES - Based on resolution
        x_img = (-y_points / res).astype(np.int32)  # x axis is -y in LIDAR
        y_img = (-x_points / res).astype(np.int32)  # y axis is -x in LIDAR

        # SHIFT PIXELS TO HAVE MINIMUM BE (0,0)
        # floor & ceil used to prevent anything being rounded to below 0 after shift
        x_img -= int(np.floor(side_range[0] / res))
        y_img += int(np.ceil(fwd_range[1] / res))

        # CLIP HEIGHT VALUES - to between min and max heights
        pixel_values = np.clip(a=z_points,
                               a_min=height_range[0],
                               a_max=height_range[1])

        # RESCALE THE HEIGHT VALUES - to be between the range 0-255
        pixel_values = self.scale_to_255(pixel_values,
                                    min=height_range[0],
                                    max=height_range[1])

        # INITIALIZE EMPTY ARRAY - of the dimensions we want
        x_max = 1 + int((side_range[1] - side_range[0]) / res)
        y_max = 1 + int((fwd_range[1] - fwd_range[0]) / res)
        im = np.zeros([y_max, x_max], dtype=np.uint8)

        # FILL PIXEL VALUES IN IMAGE ARRAY
        im[y_img, x_img] = pixel_values

        return im

    def __make_bev(self, labels):
        range_image = self.__cur_range_image.copy()
        cv2.imshow("range", range_image)
        if cv2.waitKey(25) & 0xFF == ord('q'):
           return
        return
        seg_image = labels.copy()
        result_image = np.zeros((MAP_DEPTH + 1, range_image.shape[1]))
        result_image[result_image == 0] = -1

        for i in range(range_image.shape[1]):
            for j in range(range_image.shape[0]):
                x_coef = (MAP_DEPTH / (0.5 * MAP_DEPTH + j)) / 3
                new_i = int(i * x_coef)
                new_i = new_i if new_i < 840 else 839
#                self._logger.info(f'{new_i}')
                result_image[MAP_DEPTH - int(range_image[j, i] * MAP_DEPTH), new_i] = seg_image[j, i]

        colorized = colorize(result_image)
        cv2.imshow("map image", np.asarray(colorized))
        if cv2.waitKey(25) & 0xFF == ord('q'):
           return
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
