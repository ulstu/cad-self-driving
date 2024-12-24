'''В OSM сначала долгота, потом широта'''


import math
import fields2cover as f2c


# Константа: радиус Земли в метрах
R = 6378137  # WGS84

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


def build_field(edges, logger=print):
    """
    Построение пути объезда поля из координат углов полигона.
    
    :param edges: Массив углов полигона
    :param logger: Указатель на логгер
    :return: Набор путевых точек в глобальных координатах
    """
    
    robot = f2c.Robot(2.0, 6.0)
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

    return square_edges
