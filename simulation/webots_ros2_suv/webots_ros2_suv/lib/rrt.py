import random
import math
import cv2
import numpy as np

class PathNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

    def __str__(self):
        return f'({self.x}:{self.y})'

    def __repr__(self):
        return f'({self.x}:{self.y})'


class Vehicle:
    def __init__(self, length, width):
        self.length = length
        self.width = width

def rrt(start, goal, obstacles, max_iters, step_size, vehicle, logger=None):
    nodes = [start]
    for _ in range(max_iters):
        if random.random() < 0.5:
            rand_node = PathNode(random.uniform(0, obstacles.shape[1]), random.uniform(0, obstacles.shape[0]))
        else:
            rand_node = goal
        nearest_node = nodes[0]
        for node in nodes:
            if math.dist((node.x, node.y), (rand_node.x, rand_node.y)) < math.dist((nearest_node.x, nearest_node.y), (rand_node.x, rand_node.y)):
                nearest_node = node
        new_node = PathNode(nearest_node.x, nearest_node.y)
        theta = math.atan2(rand_node.y - nearest_node.y, rand_node.x - nearest_node.x)
        new_node.x += step_size * math.cos(theta)
        new_node.y += step_size * math.sin(theta)
        if not is_collision(new_node, obstacles, vehicle):
            new_node.parent = nearest_node
            nodes.append(new_node)
            if math.dist((new_node.x, new_node.y), (goal.x, goal.y)) < step_size:
                goal.parent = new_node
                nodes.append(goal)
                return nodes
    return None

def is_collision(node, obstacles, vehicle):
    x, y = int(node.x), int(node.y)
    half_length = vehicle.length / 2
    half_width = vehicle.width / 2

    for i in range(x - int(half_length), x + int(half_length) + 1):
        for j in range(y - int(half_width), y + int(half_width) + 1):
            if 0 <= i < len(obstacles) and 0 <= j < len(obstacles[0]) and obstacles[i][j] != 100:
                return True
    return False

def generate_path(nodes, goal_node):
    if not nodes:
        return []
    for n in nodes:
        if int(n.x) == int(goal_node.x) and int(n.y) == int(goal_node.y):
            goal_node = n
            break
    path = []
    current_node = goal_node
    while current_node:
        path.append((int(current_node.y), int(current_node.x)))
        current_node = current_node.parent
    return path[::-1]

def plot_rrt(nodes, start_node, goal_node, obstacles):
    path = generate_path(nodes, goal_node)
    if path:
        for i in range(len(path) - 1):
            p1 = (int(path[i][0]), int(path[i][1]))
            p2 = (int(path[i + 1][0]), int(path[i + 1][1]))
            cv2.line(obstacles, p1, p2, (0, 255, 0), 4)
    return obstacles


def make_sample():
    # Пример использования:
    start_node = PathNode(10, 10)
    goal_node = PathNode(90, 90)

    obstacles = np.full((100, 100), 100, dtype=np.uint8)
    # Задаем препятствия
    obstacles[20:40, 30:50] = 0
    obstacles[60:80, 20:40] = 0

    max_iterations = 1000
    step_size = 5

    result = rrt(start_node, goal_node, obstacles, max_iterations, step_size)
    if result:
        path = generate_path(goal_node)
        print("Путь найден:", path)
        plot_rrt(result, goal_node, obstacles)
    else:
        print("Путь не найден.")
