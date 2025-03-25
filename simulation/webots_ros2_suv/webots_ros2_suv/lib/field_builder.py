'''В OSM сначала долгота, потом широта'''


import math
import fields2cover as f2c
import numpy as np

from webots_ros2_suv.lib.linalg import VECTOR2, POINT, vector_angle
from webots_ros2_suv.lib.ReedsShepp import calc_optimal_path


# Константа: радиус Земли в метрах
R = 6378137  # WGS84


def gps_to_rect(dLon, dLat):
    """
    Перевод географических координат (широта, долгота) в прямоугольные (x, y) методом Гаусса-Крюгера.
    
    :param dLon: Долгота (в градусах)
    :param dLat: Широта (в градусах)
    :return: Координаты x, y в метрах
    """

    # Номер зоны Гаусса-Крюгера
    zone = int(dLon / 6.0 + 1)

    # Параметры эллипсоида Красовского
    a = 6378245.0  # Большая (экваториальная) полуось
    b = 6356863.019  # Малая (полярная) полуось
    e2 = (a ** 2 - b ** 2) / a ** 2  # Эксцентриситет
    n = (a - b) / (a + b)  # Приплюснутость

    # Параметры зоны Гаусса-Крюгера
    F = 1.0  # Масштабный коэффициент
    Lat0 = 0.0  # Начальная параллель (в радианах)
    Lon0 = (zone * 6 - 3) * math.pi / 180  # Центральный меридиан (в радианах)
    N0 = 0.0  # Условное северное смещение для начальной параллели
    E0 = zone * 1e6 + 500000.0  # Условное восточное смещение для центрального меридиана

    # Перевод широты и долготы в радианы
    Lat = dLat * math.pi / 180.0
    Lon = dLon * math.pi / 180.0

    # Вычисление переменных для преобразования
    v = a * F * (1 - e2 * (math.sin(Lat) ** 2)) ** -0.5
    p = a * F * (1 - e2) * (1 - e2 * (math.sin(Lat) ** 2)) ** -1.5
    n2 = v / p - 1
    M1 = (1 + n + 5.0 / 4.0 * n ** 2 + 5.0 / 4.0 * n ** 3) * (Lat - Lat0)
    M2 = (3 * n + 3 * n ** 2 + 21.0 / 8.0 * n ** 3) * math.sin(Lat - Lat0) * math.cos(Lat + Lat0)
    M3 = (15.0 / 8.0 * n ** 2 + 15.0 / 8.0 * n ** 3) * math.sin(2 * (Lat - Lat0)) * math.cos(2 * (Lat + Lat0))
    M4 = 35.0 / 24.0 * n ** 3 * math.sin(3 * (Lat - Lat0)) * math.cos(3 * (Lat + Lat0))
    M = b * F * (M1 - M2 + M3 - M4)
    I = M + N0
    II = v / 2 * math.sin(Lat) * math.cos(Lat)
    III = v / 24 * math.sin(Lat) * (math.cos(Lat)) ** 3 * (5 - (math.tan(Lat) ** 2) + 9 * n2)
    IIIA = v / 720 * math.sin(Lat) * (math.cos(Lat) ** 5) * (61 - 58 * (math.tan(Lat) ** 2) + (math.tan(Lat) ** 4))
    IV = v * math.cos(Lat)
    V = v / 6 * (math.cos(Lat) ** 3) * (v / p - (math.tan(Lat) ** 2))
    VI = v / 120 * (math.cos(Lat) ** 5) * (5 - 18 * (math.tan(Lat) ** 2) + (math.tan(Lat) ** 4) + 14 * n2 - 58 * (math.tan(Lat) ** 2) * n2)

    # Вычисление северного и восточного смещения (в метрах)
    N = I + II * (Lon - Lon0) ** 2 + III * (Lon - Lon0) ** 4 + IIIA * (Lon - Lon0) ** 6
    E = E0 + IV * (Lon - Lon0) + V * (Lon - Lon0) ** 3 + VI * (Lon - Lon0) ** 5

    return [E, -N]


def geo_to_cartesian(lat, lon):
    """
    Перевод географических координат (широта, долгота) в прямоугольные (x, y).
    
    :param lat: Широта (в градусах)
    :param lon: Долгота (в градусах)
    :return: Координаты x, y в метрах
    """
    # Перевод в радианы
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    # Вычисление x, y
    x = R * lon_rad
    y = R * math.log(math.tan(math.pi / 4 + lat_rad / 2))
    return x, y

def cartesian_to_geo(x, y):
    """
    Перевод прямоугольных координат (x, y) в географические (широта, долгота).
    
    :param x: Координата x (в метрах)
    :param y: Координата y (в метрах)
    :return: Широта и долгота в градусах
    """
    # Вычисление долготы
    lon_rad = x / R
    
    # Вычисление широты
    lat_rad = 2 * math.atan(math.exp(y / R)) - math.pi / 2
    
    # Перевод в градусы
    lat = math.degrees(lat_rad)
    lon = math.degrees(lon_rad)
    return lat, lon


def build_field(edges, position, logger=print):
    """
    Построение пути объезда поля из координат углов полигона.
    
    :param edges: Массив углов полигона
    :param position: Положение автомобиля (x, y, angle)
    :param logger: Указатель на логгер
    :return: (Набор путевых точек в глобальных координатах, в прямоугольных) 
    """
    
    robot = f2c.Robot(2.5, 20.0)
    const_hl = f2c.HG_Const_gen()

    square_edges = []
    first_edge = geo_to_cartesian(edges[0][1], edges[0][0])

    field_ring = f2c.LinearRing()
    for edge in edges:
        current_edge = geo_to_cartesian(edge[1], edge[0])
        field_ring.addPoint(current_edge[0], current_edge[1])    

    cells = f2c.Cells(f2c.Cell(field_ring))

    no_hl = const_hl.generateHeadlands(cells, 3.0 * robot.robot_width)
    bf = f2c.SG_BruteForce_NSwath()

    swaths = bf.generateSwaths(math.pi, robot.op_width, no_hl.getGeometry(0))

    snake_sorter = f2c.RP_Snake()
    snake_sorter.setSwaths(swaths)
    swaths = snake_sorter.genSortedSwaths()
    swaths.at(0).getPath().exportToWkt()

    for i in range(swaths.size()):
        current_point = swaths.at(i).getPoint(0)
        current_geo = cartesian_to_geo(current_point.getX(), current_point.getY())
        square_edges.append([current_geo[1],current_geo[0]])
        current_point = swaths.at(i).getPoint(1)
        current_geo = cartesian_to_geo(current_point.getX(), current_point.getY())
        square_edges.append([current_geo[1],current_geo[0]])


    destinations = []
    for i in range(swaths.size()):
        first_point = swaths.at(i).getPoint(0)
        second_point = swaths.at(i).getPoint(1)
        angle = vector_angle(VECTOR2(first_point.getX() - second_point.getX(), first_point.getY() - second_point.getY()))
        destinations.append(POINT(first_point.getX(), first_point.getY(), angle))
        destinations.append(POINT(second_point.getX(), second_point.getY(), angle))

    tmp_low_destintaions = destinations.copy()

    geo_edges = []
    for i in range(len(tmp_low_destintaions) - 1):
        s_x = tmp_low_destintaions[i].x
        s_y = tmp_low_destintaions[i].y
        s_yaw = tmp_low_destintaions[i].angle
        g_x = tmp_low_destintaions[i + 1].x
        g_y = tmp_low_destintaions[i + 1].y
        g_yaw = tmp_low_destintaions[i + 1].angle

        try:
            path_i = calc_optimal_path(s_x, s_y, s_yaw,
                                   g_x, g_y, g_yaw, 0.1, 4)
        except:
            logger("no reedsshepp!")
            return square_edges, []

        for j in range(len(path_i.x)):
            current_geo = cartesian_to_geo(path_i.x[j], path_i.y[j])
            geo_edges.append([current_geo[1],current_geo[0]])
            pass

    return geo_edges
