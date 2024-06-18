import numpy as np

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

# Пример использования:
obstacles = np.full((200, 200), 100, dtype=int)
# Добавим прямоугольные препятствия
obstacles[50:150, 50:150] = 0

path = [(10, 10), (100, 100), (190, 190)]
radius = 5  # Радиус автомобиля

print(is_path_clear(path, radius, obstacles))  # Вернет False, так как путь пересекает препятствие
