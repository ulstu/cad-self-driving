import numpy as np
import cv2
import heapq
import dubins
from scipy.special import binom

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def is_collision_free(x, y, robot_radius, obstacles):
    for i in range(int(x - robot_radius), int(x + robot_radius) + 1):
        for j in range(int(y - robot_radius), int(y + robot_radius) + 1):
            if np.linalg.norm([x - i, y - j]) <= robot_radius:
                if not (0 <= i < obstacles.shape[1] and 0 <= j < obstacles.shape[0]) or obstacles[j, i] != 100:
                    return False
    return True

def astar(start, goal, obstacles, robot_radius, step_size=1, log=print):
    start = tuple(start)
    goal = tuple(goal)
    grid_size = (obstacles.shape[1], obstacles.shape[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    directions = [(-step_size, -step_size), (-step_size, 0), (step_size, -step_size), (0, -step_size), (step_size, 0)]
    log(f"robot: {robot_radius} step_size: {step_size}")
    while open_set:
        _, current = heapq.heappop(open_set)

        if heuristic(current, goal) < step_size:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in directions:
            # try:
            neighbor = (current[0] + dx, current[1] + dy)
            tentative_g_score = g_score[current] + heuristic(current, neighbor)

            if 0 <= neighbor[0] < grid_size[0] and 0 <= neighbor[1] < grid_size[1] and is_collision_free(neighbor[0], neighbor[1], robot_radius, obstacles):
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
            # except:
            #     log('error')

    return None

def kalman_filter_path(path, process_noise_scale=0.1):
    n = len(path)
    smoothed_path = np.zeros((n, 2))
    Q = np.eye(2) * process_noise_scale  # Process noise covariance
    R = np.eye(2) * 0.1  # Measurement noise covariance
    P = np.eye(2)  # Initial estimate error covariance
    X = np.array(path[0])  # Initial state

    for i in range(n):
        Z = np.array(path[i])  # Current measurement

        # Prediction step
        X = X  # State prediction
        P = P + Q  # Covariance prediction

        # Update step
        K = P @ np.linalg.inv(P + R)  # Kalman gain
        X = X + K @ (Z - X)  # State update
        P = (np.eye(2) - K) @ P  # Covariance update

        smoothed_path[i] = X

    return smoothed_path.tolist()

def is_path_clear(obstacles, path_segment):
    configurations, _ = path_segment.sample_many(0.1)
    for config in configurations:
        x, y = config[1], config[0]
        if not (0 <= x < obstacles.shape[0] and 0 <= y < obstacles.shape[1]) or obstacles[int(x), int(y)] != 100:
            return False
    return True

def bezier_curve(control_points, num_points=100):
    """Compute a Bézier curve from control points."""
    n = len(control_points) - 1
    curve = []
    for t in np.linspace(0, 1, num_points):
        point = np.zeros(2)
        for i in range(n + 1):
            point += binom(n, i) * (t ** i) * ((1 - t) ** (n - i)) * np.array(control_points[i])
        curve.append((int(point[0]), int(point[1])))
    return curve

def smooth_path_with_dubins(path, turning_radius, obstacles, sample_size=1, log=print):
    if not path:
        return []

    smooth_path = [path[0]]
    i = 0
    while i < len(path) - 1:
        q0 = (path[i][0], path[i][1], 0)
        for j in range(i + 1, len(path)):
            q1 = (path[j][0], path[j][1], 0)
            path_segment = dubins.shortest_path(q0, q1, turning_radius)
            if is_path_clear(obstacles, path_segment):
                configurations, _ = path_segment.sample_many(sample_size)
                smooth_path.extend([(int(config[0]), int(config[1])) for config in configurations])
                i = j - 1
                break
        i += 1
    return smooth_path

def plot_path(dubins_smoothed_path, kalman_smoothed_path, path, start, goal, obstacles):
    img = np.zeros((obstacles.shape[0], obstacles.shape[1], 3), dtype=np.uint8)
    
    # Отметим препятствия белым цветом
    img[obstacles != 100] = [255, 255, 255]

    # Отметим путь A* синим цветом
    for (x, y) in path:
        img[int(x), int(y)] = [255, 0, 0]

    # Отметим сглаженный путь фильтром Калмана зеленым цветом
    for (x, y) in kalman_smoothed_path:
        img[int(x), int(y)] = [0, 255, 0]

    # Отметим сглаженный путь кривыми Дубинса красным цветом
    for (x, y) in dubins_smoothed_path:
        img[int(x), int(y)] = [0, 0, 255]

    # Отметим начальную и конечную точки
    img[int(start[0]), int(start[1])] = [255, 255, 0]  # Начало - желтый цвет
    img[int(goal[0]), int(goal[1])] = [0, 255, 255]   # Конец - голубой цвет

    # Изменим масштаб изображения для лучшего визуального восприятия
    img = cv2.resize(img, (600, 600), interpolation=cv2.INTER_NEAREST)

    cv2.imshow("Path Planning", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    grid_size = (200, 200)
    obstacles = np.full(grid_size, 100, dtype=int)
    
    # Добавим прямоугольные препятствия
    obstacles[5:100, 5:150] = 0
    obstacles[120:150, 20:80] = 0
    
    start = (0, 0)
    goal = (190, 190)
    turning_radius = 0.1
    process_noise_scale = 0.05 # Попробуйте различные значения

    path = astar(start, goal, obstacles, robot_radius=1, step_size=0.5)

    if path:
        kalman_smoothed_path = kalman_filter_path(path, process_noise_scale)
        dubins_smoothed_path = smooth_path_with_dubins(kalman_smoothed_path, turning_radius, obstacles)
        plot_path(dubins_smoothed_path, kalman_smoothed_path, path, start, goal, obstacles)
    else:
        print("Путь не найден")
