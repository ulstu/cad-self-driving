import numpy as np
import cv2
import random


# Алгоритм RRT*
class Node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent

def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_random_point():
    return (random.randint(0, width-1), random.randint(0, height-1))

def get_nearest_node_index(nodes, random_point):
    distances = [distance(node.point, random_point) for node in nodes]
    min_index = distances.index(min(distances))
    return min_index

def step_from_to(p1, p2, step_size=20):
    if distance(p1, p2) < step_size:
        return p2
    else:
        theta = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        return int(p1[0] + step_size * np.cos(theta)), int(p1[1] + step_size * np.sin(theta))

def rrt_star(map, start, goal, iterations=5000):
    nodes = [Node(start)]
    for i in range(iterations):
        random_point = get_random_point()
        nearest_index = get_nearest_node_index(nodes, random_point)
        nearest_node = nodes[nearest_index]

        new_point = step_from_to(nearest_node.point, random_point)
        new_node = Node(new_point, nearest_node)
        nodes.append(new_node)

        # # Визуализация
        # cv2.line(map, nearest_node.point, new_node.point, (0, 0, 255), 3)  # Линии пути синие
        # cv2.imshow("RRT*", map)
        # cv2.waitKey(1)

        # Проверка на достижение цели
        if distance(new_node.point, goal) < max_step_size:
            print("Путь найден")
            return nodes

    return None  # Если путь не найден


def draw_path():

    pass

# Параметры карты
width, height = 600, 400
start = (50, 50)
goal = (550, 350)
free_space = 255  # Белый цвет для свободного пространства
max_step_size = 20  # Максимальный размер шага

# Создание карты
map = np.full((height, width), free_space, np.uint8)

# Запуск алгоритма RRT*
nodes = rrt_star(map, start, goal)

# Визуализация пути
if nodes:
    # Восстановление пути
    path = []
    last_node = nodes[-1]
    while last_node.parent is not None:
        path.append(last_node.point)
        last_node = last_node.parent
    path.append(start)

    map = cv2.cvtColor(map, cv2.COLOR_GRAY2RGB)
    # Рисование пути
    for i in range(len(path) - 1):
        cv2.line(map, path[i], path[i+1], (0, 255, 0), 2)
    cv2.circle(map, start, 5, (255, 0, 0), 3)  # Старт красный
    cv2.circle(map, goal, 5, (0, 255, 0), 3)  # Цель зеленая

    cv2.imshow("RRT* Path", map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Путь не найден")
