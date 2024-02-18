
from shapely.geometry import Point, Polygon
from shapely.ops import transform
from functools import partial
import pyproj

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