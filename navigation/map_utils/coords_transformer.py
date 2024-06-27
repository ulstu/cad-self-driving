import math

def gps_to_image_coords(x, y, lat, lon, angle, lat_goal, lon_goal):
    # Константа для перевода метров в пиксели
    meters_to_pixels = 15
    
    # Константа для перевода градусов в радианы
    degrees_to_radians = math.pi / 180
    
    # Расчет расстояния в метрах между текущей позицией и целью
    R = 6371000  # Радиус Земли в метрах
    dlat = (lat_goal - lat) * degrees_to_radians
    dlon = (lon_goal - lon) * degrees_to_radians
    a = math.sin(dlat / 2) ** 2 + math.cos(lat * degrees_to_radians) * math.cos(lat_goal * degrees_to_radians) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance_meters = R * c
    
    # Определение направления движения к целевой точке в глобальной системе координат
    bearing = math.atan2(
        math.sin(dlon) * math.cos(lat_goal * degrees_to_radians),
        math.cos(lat * degrees_to_radians) * math.sin(lat_goal * degrees_to_radians) -
        math.sin(lat * degrees_to_radians) * math.cos(lat_goal * degrees_to_radians) * math.cos(dlon)
    )
    
    # Учитываем угол поворота автомобиля
    relative_bearing = bearing - angle
    
    # Перевод расстояния в пиксели
    distance_pixels = distance_meters * meters_to_pixels
    
    # Вычисляем смещение в координатах изображения
    delta_x = distance_pixels * math.sin(relative_bearing)
    delta_y = distance_pixels * math.cos(relative_bearing)
    
    # Вычисляем новые координаты на изображении
    goal_x = x + delta_x
    goal_y = y + delta_y  # Смещение вниз по оси Y увеличивает координату

    return (goal_x, goal_y)

# Пример использования
x, y = 100, 100  # текущие координаты автомобиля в системе координат изображения
lat, lon = 55.751244, 37.618423  # текущие глобальные координаты автомобиля
angle = math.radians(0)  # угол поворота автомобиля в радианах (например, 0 градусов, то есть строго вверх)
lat_goal, lon_goal = 55.752244, 37.619423  # целевые глобальные координаты

goal_coords = gps_to_image_coords(x, y, lat, lon, angle, lat_goal, lon_goal)
print(goal_coords)
