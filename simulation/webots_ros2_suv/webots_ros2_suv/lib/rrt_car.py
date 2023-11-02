import random
import math
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class Vehicle:
    def __init__(self, length, width):
        self.length = length
        self.width = width

def rrt(start, goal, obstacles, max_iters, step_size, vehicle):
    nodes = [start]
    for _ in range(max_iters):
        if random.random() < 0.5:
            rand_node = Node(random.uniform(0, 100), random.uniform(0, 100))
        else:
            rand_node = goal
        nearest_node = nodes[0]
        for node in nodes:
            if math.dist((node.x, node.y), (rand_node.x, rand_node.y)) < math.dist((nearest_node.x, nearest_node.y), (rand_node.x, rand_node.y)):
                nearest_node = node
        new_node = Node(nearest_node.x, nearest_node.y)
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
            if 0 <= i < len(obstacles) and 0 <= j < len(obstacles[0]) and obstacles[i][j] != 0:
                return True
    return False

def generate_path(goal_node):
    path = []
    current_node = goal_node
    while current_node:
        path.append((current_node.x, current_node.y))
        current_node = current_node.parent
    return path[::-1]

def plot_rrt(nodes, goal_node, obstacles, vehicle):
    plt.figure(figsize=(8, 8))
    plt.imshow(obstacles, cmap='gray', origin='lower', extent=[0, 100, 0, 100])
    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='blue', linewidth=0.5)
    path = generate_path(goal_node)
    if path:
        x, y = zip(*path)
        plt.plot(x, y, color='green', linewidth=2)
    plt.scatter(start_node.x, start_node.y, color='red', s=100, marker='o')
    plt.scatter(goal_node.x, goal_node.y, color='purple', s=100, marker='x')
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.gca().invert_yaxis()
    plt.show()

# Пример использования:
start_node = Node(10, 10)
goal_node = Node(90, 90)

vehicle = Vehicle(length=10, width=5)

obstacles = [[0] * 100 for _ in range(100)]
# Задаем препятствия
obstacles[20:40][30:50] = [1] * 20
obstacles[60:80][20:40] = [1] * 20

max_iterations = 1000
step_size = 5

result = rrt(start_node, goal_node, obstacles, max_iterations, step_size, vehicle)
if result:
    path = generate_path(goal_node)
    print("Путь найден:", path)
    plot_rrt(result, goal_node, obstacles, vehicle)
else:
    print("Путь не найден.")
