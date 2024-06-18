import cv2
import numpy as np
from matplotlib import pyplot as plt


class IPMTransformer(object):
    def __init__(self, homography_matrix=None) -> None:
        # Конструктор класса IPMTransformer.
        # homography_matrix - необязательный аргумент, матрица гомографии, инициализируется пустой матрицей, если не предоставлена.
        self.__h = homography_matrix

    def calc_homography(self, pts_src, pts_dst):
        # Метод для вычисления матрицы гомографии на основе набора исходных точек (pts_src) и соответствующих точек назначения (pts_dst).
        # Использует функцию cv2.findHomography для выполнения вычислений.
        # Возвращает результат в виде матрицы гомографии (self.__h).
        self.__h, status = cv2.findHomography(pts_src, pts_dst, cv2.RANSAC)
        return self.__h

    def get_homography_matrix(self):
        # Метод для получения матрицы гомографии, которая была вычислена или установлена ранее.
        # Возвращает матрицу гомографии, хранящуюся в self.__h.
        return self.__h

    def get_ipm(self, im_src, dst_size=(1200, 800), horizont=0, is_mono=False, need_cut=True):
        # Метод для выполнения обратного преобразования перспективы (Inverse Perspective Mapping) на исходном изображении im_src.
        # im_src - исходное изображение, на которое будет применено обратное преобразование перспективы.
        # dst_size - размер целевого изображения после преобразования (по умолчанию (1200, 800)).
        # horizont - высота горизонтальной линии, используемой для обрезки изображения (по умолчанию 0).
        # is_mono - флаг, указывающий, является ли исходное изображение монохромным (по умолчанию False).
        # need_cut - флаг, указывающий, нужно ли обрезать изображение для удаления нулевых строк и столбцов в результате (по умолчанию True).
        # Возвращает преобразованное изображение после применения обратного преобразования перспективы.
        im_src = im_src[horizont:, :]  # Обрезаем изображение по горизонтали, если задана высота горизонтали

        im_dst = cv2.warpPerspective(im_src, self.__h, dst_size)  # Применяем матрицу гомографии

        if not need_cut:
            return im_dst
        if not is_mono:
            rows = np.sum(im_dst, axis=(1, 2))
            cols = np.sum(im_dst, axis=(0, 2))
        else:
            rows = np.sum(im_dst, axis=(1))
            cols = np.sum(im_dst, axis=(0))

        nonzero_rows = len(rows[np.nonzero(rows)])  # Находим ненулевые строки
        nonzero_cols = len(cols[np.nonzero(cols)])  # Находим ненулевые столбцы
        im_dst = im_dst[:nonzero_rows, :nonzero_cols]  # Обрезаем изображение до ненулевых строк и столбцов
        im_dst = cv2.resize(im_dst, (im_dst.shape[1], im_dst.shape[1]), interpolation=cv2.INTER_AREA)  # Изменяем размер
        return im_dst
