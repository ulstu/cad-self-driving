import math
import numpy as np
import matplotlib.pyplot as plt

def calculate_angle(p1, p2, p3):
    """Calculate the angle between the line segments (p1p2) and (p2p3)."""
    v1 = (p1[0] - p2[0], p1[1] - p2[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])
    angle = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
    return abs(angle)

def filter_coordinates(coordinates, min_path_points_dist, angle_threshold_degrees):
    if not coordinates:
        return []

    angle_threshold = math.radians(angle_threshold_degrees)
    filtered_coords = [coordinates[0]]

    for i in range(1, len(coordinates)):
        if math.sqrt((filtered_coords[-1][0] - coordinates[i][0]) ** 2 + (filtered_coords[-1][1] - coordinates[i][1]) ** 2) >= min_path_points_dist:
            if len(filtered_coords) < 2 or calculate_angle(filtered_coords[-2], filtered_coords[-1], coordinates[i]) >= angle_threshold:
                filtered_coords.append(coordinates[i])

    return filtered_coords

# Генерация пути с 100 точками
np.random.seed(0)
x = np.linspace(0, 10, 100)
y = np.sin(x) + np.random.normal(0, 0.2, 100)
coordinates = list(zip(x, y))

# Параметры фильтрации
min_path_points_dist = 0.5  # Минимальное расстояние между точками
angle_threshold_degrees = 10  # Пороговый угол для фильтрации поворотов в градусах

filtered_coords = filter_coordinates(coordinates, min_path_points_dist, angle_threshold_degrees)

# Визуализация
plt.figure(figsize=(10, 5))
plt.plot(*zip(*coordinates), 'b-', label="Original Path")
plt.plot(*zip(*filtered_coords), 'r-', label="Filtered Path")
plt.scatter(*zip(*coordinates), c='blue', s=10)
plt.scatter(*zip(*filtered_coords), c='red', s=30)
plt.legend()
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Path Filtering")
plt.grid(True)
plt.show()
