import math
from geopy.distance import geodesic
import matplotlib.pyplot as plt

def calculate_initial_compass_bearing(pointA, pointB):
    """
    Calculates the bearing between two points.
    The formula used is the following:
        θ = atan2(sin(Δlong).cos(lat2),
                  cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong))
    Parameters:
    pointA : tuple of float : (lat, lon) of the starting point
    pointB : tuple of float : (lat, lon) of the destination point
    Returns:
    float : initial compass bearing in degrees from north
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

def gps_to_image_coords(x, y, lat, lon, angle, lat_goal, lon_goal):
    # Константа для перевода метров в пиксели
    meters_to_pixels = 15

    # Вычисление расстояния в метрах с помощью geopy
    start_coords = (lat, lon)
    goal_coords = (lat_goal, lon_goal)
    distance_meters = geodesic(start_coords, goal_coords).meters
    print(f'distance: {distance_meters}')

    # Определение направления движения к целевой точке в глобальной системе координат
    bearing = calculate_initial_compass_bearing(start_coords, goal_coords)
    bearing_radians = math.radians(bearing)
    print(f'bearing_radians: {bearing_radians} {math.degrees(bearing_radians)}')

    # Учитываем угол поворота автомобиля
    relative_bearing = bearing_radians - angle
    print(f'relative_bearing: {relative_bearing} {math.degrees(relative_bearing)}')

    # Перевод расстояния в пиксели
    distance_pixels = distance_meters * meters_to_pixels
    print(f'distance_pixels: {distance_pixels}')

    # Вычисляем смещение в координатах изображения
    delta_x = distance_pixels * math.sin(relative_bearing)
    delta_y = distance_pixels * math.cos(relative_bearing)
    print(f'delta_x: {delta_x} delta_y:{delta_y}')

    # Вычисляем новые координаты на изображении
    goal_x = x + delta_x
    goal_y = y - delta_y  # Смещение вниз по оси Y уменьшает координату

    return (goal_x, goal_y)

# Пример использования
x, y = 100, 800  # текущие координаты автомобиля в системе координат изображения
lat, lon = 54.328452, 48.399000  # текущие глобальные координаты автомобиля
angle = math.radians(270)  # угол поворота автомобиля в радианах (например, 45 градусов)
lat_goal, lon_goal = 54.328358, 48.398422  # целевые глобальные координаты

goal_coords = gps_to_image_coords(x, y, lat, lon, angle, lat_goal, lon_goal)
print(goal_coords)

# Визуализация
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))

# График географических координат
ax1.plot([lon, lon_goal], [lat, lat_goal], 'bo-', label='Path')
ax1.quiver(lon, lat, math.cos(angle), math.sin(angle), scale=1, color='r', label='Orientation')
ax1.set_xlabel('Longitude')
ax1.set_ylabel('Latitude')
ax1.set_title('Geographic Coordinates')
ax1.legend()
ax1.grid(True)

# График координат изображения
ax2.plot(x, y, 'bo', label='Current Position')
ax2.plot(goal_coords[0], goal_coords[1], 'ro', label='Goal Position')
ax2.annotate('Current Position', (x, y), textcoords="offset points", xytext=(-10, -10), ha='center')
ax2.annotate('Goal Position', (goal_coords[0], goal_coords[1]), textcoords="offset points", xytext=(-10, -10), ha='center')
ax2.quiver(x, y, 0, -1, scale=10, color='r', label='Upward Orientation')  # Автомобиль направлен вверх
ax2.set_xlim(0, 200)
ax2.set_ylim(0, 200)
ax2.set_aspect('equal', adjustable='box')
ax2.set_xlabel('X coordinates (pixels)')
ax2.set_ylabel('Y coordinates (pixels)')
ax2.set_title('Image Coordinates')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()
