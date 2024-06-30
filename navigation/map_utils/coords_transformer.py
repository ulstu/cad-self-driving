import math
from geopy.distance import geodesic, distance


def calc_bearing(pointA, pointB):
    """
    Вычисляет азимут между двумя точками по формуле
    geodesic     θ = atan2(sin(Δlong).cos(lat2),
                cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong))
    Параметры:
    pointA: кортеж чисел с плавающей запятой: (широта, долгота) начальной точки
    pointB: кортеж с плавающей запятой: (широта, долгота) точки назначения
    Возвращаемое значение:
    float: азимут в градусах от севера
    """
    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])
    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    # Convert from radians to degrees and normalize to 0-360
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

def get_relative_coordinates(lat_goal, lon_goal, pos, pov_point, loger=print):
    x, y, lat, lon, angle = pov_point[0], pov_point[1], pos[1], pos[0], pos[2]
    # Константа для перевода метров в пиксели
    meters_to_pixels = 15

    # Вычисление расстояния в метрах с помощью geopy
    start_coords = (lat, lon)
    goal_coords = (lon_goal, lat_goal)
    distance_meters = geodesic(start_coords, goal_coords).meters
    # loger(f'angle: {angle} angledeg: {math.degrees(angle)} x: {x} y: {y} distance: {distance_meters} pos: {pos} goal: {goal_coords}')

    # Определение направления движения к целевой точке в глобальной системе координат
    bearing = calc_bearing(start_coords, goal_coords)
    bearing_radians = math.radians(bearing)

    # Учитываем угол поворота автомобиля
    relative_bearing = bearing_radians - angle

    # Перевод расстояния в пиксели
    distance_pixels = distance_meters * meters_to_pixels
    # print(f'bearing_radians: {bearing_radians} {math.degrees(bearing_radians)} relative_bearing: {relative_bearing} {math.degrees(relative_bearing)} distance_pixels: {distance_pixels}')

    # Вычисляем смещение в координатах изображения
    delta_x = distance_pixels * math.sin(relative_bearing)
    delta_y = distance_pixels * math.cos(relative_bearing)

    # Вычисляем новые координаты на изображении
    goal_x = x + delta_x
    goal_y = y - delta_y  # Смещение вниз по оси Y уменьшает координату

    loger(f'delta_x: {delta_x} delta_y:{delta_y} goal_x: {goal_x} goal_y:{goal_y}')
    return (int(goal_x), int(goal_y))
    
src_points = [
            [
              43.63942884255326,
              56.33512098575622
            ],
            [
              43.639598892760944,
              56.33511974509497
            ],
            [
              43.63970771356792,
              56.335111200914355
            ],
            [
              43.63984501806154,
              56.3351123011264
            ],
            [
              43.64002260930398,
              56.335096758465596
            ],
            [
              43.64018288097418,
              56.335087487887506
            ],
            [
              43.64027909359169,
              56.33508128457524
            ],
            [
              43.64037754371192,
              56.33506267463247
            ],
            [
              43.64047599383216,
              56.33504530534432
            ]
          ]

def main():
    pos = [43.63939225773861, 56.33517639374398, 1.5]
    pov_point = (389, 753)

    for p in src_points:
        lat_goal = p[0]
        lon_goal = p[1]
        res = get_relative_coordinates(lat_goal, lon_goal, pos, pov_point)
        print(res)

main()