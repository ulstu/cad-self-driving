import pathlib
import typing

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from sensor_msgs.msg import PointCloud2

from simulation.webots_ros2_suv.webots_ros2_suv.lib import point_cloud2


class LidarTool:
    def __init__(self):
        self.points = None

    def set_pc2(self, pc: PointCloud2):
        self.points = point_cloud2.read_points(pc)
        self.points = point_cloud2.read_points(pc)

    def set_npy(self, file: typing.Union[str, pathlib.Path]):
        self.points = np.load(file)
        self.points = self.points[~np.isfinite(self.points)] = 100

    def get_points(self):
        return self.points

    def get_XYPlane_colored_by_height(self, scale_factor=10):
        img_size = (50 * scale_factor, 50 * scale_factor)  # Размер изображения

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

        z = self.points[:, 2]  # Исправлено для использования z в качестве высоты

        # Нормализация z для использования в качестве индекса цвета
        norm = Normalize(vmin=np.min(z), vmax=np.max(z))
        cmap = plt.get_cmap('viridis')  # Выбор цветовой карты

        # Создание пустого изображения
        image = np.zeros((img_size[0], img_size[1], 3), dtype=np.uint8)

        # Заполнение изображения
        for i in range(len(x)):
            color = cmap(norm(z[i]))[:3]  # Получение цвета для текущей высоты
            color = (np.array(color) * 255).astype(np.uint8)  # Преобразование цвета из [0,1] в [0,255]
            image[int(y[i]), int(x[i])] = color

        return image

    def normalize_points(self, points, scale):
        min_val = np.min(points)
        max_val = np.max(points)
        return (points - min_val) / (max_val - min_val) * scale
