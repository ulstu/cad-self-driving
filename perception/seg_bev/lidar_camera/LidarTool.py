import pathlib
import typing

import numpy as np
from sensor_msgs.msg import PointCloud2
from simulation.webots_ros2_suv.webots_ros2_suv.lib import point_cloud2


class LidarTool:
    def __init__(self):
        self.points = None

    def set_pc2(self, pc: PointCloud2):
        self.points = point_cloud2.read_points(pc)

    def set_npy(self, file: typing.Union[str, pathlib.Path]):
        self.points = np.load(file)
        self.points[~np.isfinite(self.points)] = 100
        print(self.points)

    def get_points(self):
        return self.points

    def get_XYPlane_colored_by_height(self, size, scale_factor=10):
        img_size = (size[0] * scale_factor, size[1] * scale_factor)  # Размер изображения

        x = self.points[:, 0]
        max_x = np.max(x[x != 100])
        x[x == 100] = max_x
        x[:] *= scale_factor
        x = x.astype(np.int16)
        y = self.points[:, 1]
        max_y = np.max(y[y != 100])
        y[y == 100] = max_y
        y[:] *= scale_factor
        y[:] += img_size[1] // 2
        y = y.astype(np.int16)

        z = self.points[:, 2]
        z[z == 100] = np.min(z)
        z_normal, miz, maz = self.normalize_points(z)
        print(z_normal, miz, maz)

        # # Создание пустого изображения
        image = np.zeros((img_size[0], img_size[1], 3), dtype=np.uint8)
        #
        # # Заполнение изображения
        for x, y, z in zip(x, y, z_normal):
            image[img_size[0] - int(y), int(x)] = z * 255

        return image, miz, maz

    def get_YZPlane_colored_by_height(self, size, scale_factor=10):
        # # Предполагаем, что point_cloud - это массив Nx3 (N точек, каждая имеет координаты x, y, z)
        #
        # # Нормализация x и y для размера изображения
        img_size = (size[0] * scale_factor, size[1] * scale_factor)  # Размер изображения

        x = self.points[:, 2]
        max_x = np.max(x[x != 100])
        x[x == 100] = max_x
        x[:] *= scale_factor
        x[:] += img_size[0] // 2
        x = x.astype(np.int16)
        y = self.points[:, 1]
        max_y = np.max(y[y != 100])
        y[y == 100] = max_y
        y[:] *= scale_factor
        y[:] += img_size[1] // 2
        y = y.astype(np.int16)

        z = self.points[:, 0]
        z[z == 100] = np.min(z)
        z_normal, miz, maz = self.normalize_points(z)
        print(z_normal, miz, maz)

        # # Создание пустого изображения
        image = np.zeros((img_size[0], img_size[1], 3), dtype=np.uint8)
        #
        # # Заполнение изображения
        for x, y, z in zip(x, y, z_normal):
            image[img_size[0] - int(y), int(x)] = z * 255
        # cv2.imshow("XY Point Cloud Image", cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE))
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return image, miz, maz

    def normalize_points(self, points):
        min_val = np.min(points)
        max_val = np.max(points)
        return (points - min_val) / (max_val - min_val), min_val, max_val

    def interpolate_color(self, normalized_z):
        # Линейная интерполяция между зеленым (0, 255, 0) и красным (255, 0, 0)
        green = (0, 255, 0)
        red = (255, 0, 0)
        return [int((red[i] - green[i]) * normalized_z) for i in range(3)]
