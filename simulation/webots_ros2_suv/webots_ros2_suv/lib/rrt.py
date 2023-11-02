import random
import math
import cv2
import numpy as np

class PathNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def rrt(start, goal, obstacles, max_iters, step_size):
    nodes = [start]
    for _ in range(max_iters):
        if random.random() < 0.5:
            rand_node = PathNode(random.uniform(0, 100), random.uniform(0, 100))
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
        if not is_collision(new_node, obstacles):
            new_node.parent = nearest_node
            nodes.append(new_node)
            if math.dist((new_node.x, new_node.y), (goal.x, goal.y)) < step_size:
                goal.parent = new_node
                nodes.append(goal)
                return nodes
    return None

def is_collision(node, obstacles):
    x, y = int(node.x), int(node.y)
    if 0 <= x < len(obstacles) and 0 <= y < len(obstacles[0]):
        return obstacles[x][y] != 100
    return True

def generate_path(goal_node):
    path = []
    current_node = goal_node
    while current_node:
        path.append((current_node.x, current_node.y))
        current_node = current_node.parent
    return path[::-1]

def plot_rrt(nodes, goal_node, obstacles):
    img = np.full((100, 100, 3), 255, dtype=np.uint8)
    for node in nodes:
        if node.parent:
            cv2.line(img, (int(node.x), int(node.y)), (int(node.parent.x), int(node.parent.y)), (255, 0, 0), 1)
    path = generate_path(goal_node)
    if path:
        for i in range(len(path) - 1):
            cv2.line(img, (int(path[i][0]), int(path[i][1])), (int(path[i + 1][0]), int(path[i + 1][1])), (0, 255, 0), 2)
    cv2.circle(img, (int(start_node.x), int(start_node.y)), 3, (0, 0, 255), -1)
    cv2.circle(img, (int(goal_node.x), int(goal_node.y)), 3, (128, 0, 128), -1)
    cv2.imshow("RRT Visualization", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

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
