import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.special import binom

def bezier_curve(control_points, num_points=100):
    """Compute a Bézier curve from control points."""
    n = len(control_points) - 1
    curve = []
    for t in np.linspace(0, 1, num_points):
        point = np.zeros(2)
        for i in range(n + 1):
            point += binom(n, i) * (t ** i) * ((1 - t) ** (n - i)) * np.array(control_points[i])
        curve.append(point)
    return np.array(curve)

def filter_coordinates(coordinates, min_path_points_dist, angle_threshold_degrees):
    if not coordinates:
        return []

    angle_threshold = math.radians(angle_threshold_degrees)
    filtered_coords = [coordinates[0]]

    for i in range(1, len(coordinates)):
        if math.sqrt((filtered_coords[-1][0] - coordinates[i][0]) ** 2 + (filtered_coords[-1][1] - coordinates[i][1]) ** 2) >= min_path_points_dist:
            if len(filtered_coords) < 2 or calculate_angle(filtered_coords[-2], filtered_coords[-1], coordinates[i]) <= angle_threshold:
                filtered_coords.append(coordinates[i])

    return filtered_coords

def calculate_angle(p1, p2, p3):
    """Calculate the angle between the line segments (p1p2) and (p2p3)."""
    v1 = (p1[0] - p2[0], p1[1] - p2[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])
    angle = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
    return abs(angle)

# Генерация пути с 100 точками
np.random.seed(0)
x = np.linspace(0, 10, 100)
y = np.sin(x) + np.random.normal(0, 0.5, 100)
coordinates = list(zip(x, y))

# Параметры фильтрации
min_path_points_dist = 0.5  # Минимальное расстояние между точками
angle_threshold_degrees = 45  # Пороговый угол для фильтрации поворотов в градусах

filtered_coords = filter_coordinates(coordinates, min_path_points_dist, angle_threshold_degrees)

# Создание Bézier-кривой
control_points = filtered_coords
bezier_path = bezier_curve(control_points)

# Визуализация
plt.figure(figsize=(10, 5))
plt.plot(*zip(*coordinates), 'b-', label="Original Path")
plt.plot(*zip(*filtered_coords), 'g-', label="Filtered Path")
plt.plot(bezier_path[:, 0], bezier_path[:, 1], 'r-', label="Bézier Path")
plt.scatter(*zip(*coordinates), c='blue', s=10)
plt.scatter(*zip(*filtered_coords), c='green', s=30)
plt.scatter(bezier_path[:, 0], bezier_path[:, 1], c='red', s=10)
plt.legend()
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Path Filtering and Bézier Path Generation")
plt.grid(True)
plt.show()
