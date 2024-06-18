
from shapely.geometry import Point, Polygon
from shapely.ops import transform
from functools import partial
import pyproj
import math

EARTH_RADIUS_KM = 6371.0  # Радиус Земли в километрах

def is_point_in_polygon_epsg(lat, lon, polygon):
    """
    Проверяет, находится ли точка с координатами (lat, lon) внутри многоугольника с учетом кривизны Земли.
    polygon - это список точек, где каждая точка задана как [lat, lon].
    """
    # Преобразование координат в проекцию
    proj = partial(
        pyproj.transform,
        pyproj.Proj(init='epsg:4326'),  # исходная система координат (WGS84)
        pyproj.Proj(init='epsg:3857'))  # целевая система координат (Псевдо-Меркатор)

    # Создание объектов Point и Polygon с использованием shapely и преобразование их в целевую проекцию
    point_transformed = transform(proj, Point(lon, lat))
    polygon_transformed = transform(proj, Polygon(polygon))

    # Проверка принадлежности точки к многоугольнику
    return polygon_transformed.contains(point_transformed)


def is_point_in_polygon(lat, lon, polygon):
    """
    Проверяет, находится ли точка с координатами (lat, lon) внутри многоугольника.
    polygon - это список точек, где каждая точка задана как [lat, lon].
    """
    num = len(polygon)
    j = num - 1
    c = False
    for i in range(num):
        if ((polygon[i][1] > lon) != (polygon[j][1] > lon)) and \
                (lat < (polygon[j][0] - polygon[i][0]) * (lon - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) + polygon[i][0]):
            c = not c
        j = i
    return c

def calc_dist_point(p1, p2, log=None):
    """
    Возвращает дистанцию между двумя точками, заданными кортежами (lat, lon),
    используя формулу гаверсинуса.
    """
    if log:
        log(f'p1: {p1}, p2: {p2}')
    lat1, lon1 = p1[0], p1[1]#map(math.radians, p1)
    lat2, lon2 = p2[0], p2[1]#map(math.radians, p2)

    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    a = math.sin(delta_lat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2)**2
    c = 2 * math.asin(math.sqrt(a))

    distance_km = EARTH_RADIUS_KM * c
    return distance_km

def is_point_clear(x, y, radius, obstacles):
    """
    Check if the point and its surrounding radius are clear of obstacles.
    
    Parameters:
    x, y: int, coordinates of the point
    radius: int, radius of the vehicle
    obstacles: 2D numpy array, obstacle map where 100 indicates free space
    
    Returns:
    bool: True if the point and its surrounding radius are clear of obstacles, False otherwise
    """
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            if dx**2 + dy**2 <= radius**2:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < obstacles.shape[1] and 0 <= ny < obstacles.shape[0]) or obstacles[ny, nx] != 100:
                    return False
    return True

def is_line_clear(p1, p2, radius, obstacles):
    """
    Check if the line between p1 and p2 is clear of obstacles, considering the radius of the vehicle.
    
    Parameters:
    p1, p2: tuples, (x, y) coordinates of the two points
    radius: int, radius of the vehicle
    obstacles: 2D numpy array, obstacle map where 100 indicates free space
    
    Returns:
    bool: True if the line is clear of obstacles, False otherwise
    """
    x1, y1 = p1
    x2, y2 = p2
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        if not is_point_clear(x1, y1, radius, obstacles):
            return False
        if x1 == x2 and y1 == y2:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return True

def is_path_clear(path, radius, obstacles):
    """
    Check if the entire path is clear of obstacles, considering the radius of the vehicle.
    
    Parameters:
    path: list of tuples, (x, y) coordinates of the path points
    radius: int, radius of the vehicle
    obstacles: 2D numpy array, obstacle map where 100 indicates free space
    
    Returns:
    bool: True if the entire path is clear of obstacles, False otherwise
    """
    for i in range(len(path) - 1):
        if not is_line_clear(path[i], path[i + 1], radius, obstacles):
            return False
    return True
