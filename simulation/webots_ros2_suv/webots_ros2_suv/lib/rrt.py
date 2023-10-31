import random
import math
import matplotlib.pyplot as plt


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def rrt(start, goal, obstacles, max_iters, step_size):
    nodes = [start]

    for _ in range(max_iters):
        random_node = Node(random.uniform(0, 10), random.uniform(0, 10))  # Генерируем случайную точку
        nearest_node = nodes[0]

        for node in nodes:
            if math.dist((random_node.x, random_node.y), (node.x, node.y)) < math.dist((random_node.x, random_node.y), (
            nearest_node.x, nearest_node.y)):
                nearest_node = node

        new_node = Node(nearest_node.x + step_size * (random_node.x - nearest_node.x),
                        nearest_node.y + step_size * (random_node.y - nearest_node.y))

        if is_collision_free(nearest_node, new_node, obstacles):
            new_node.parent = nearest_node
            nodes.append(new_node)

            if math.dist((new_node.x, new_node.y), (goal.x, goal.y)) < step_size:
                return construct_path(new_node)

    return None


def is_collision_free(node1, node2, obstacles):
    for obstacle in obstacles:
        if (node1.x <= obstacle[0] <= node2.x or node1.x >= obstacle[0] >= node2.x) and (
                node1.y <= obstacle[1] <= node2.y or node1.y >= obstacle[1] >= node2.y):
            return False
    return True


def construct_path(node):
    path = [(node.x, node.y)]
    while node.parent:
        node = node.parent
        path.append((node.x, node.y))
    return list(reversed(path))


# Пример использования с визуализацией
start = Node(1, 1)
goal = Node(9, 9)
obstacles = [(4, 5), (7, 3), (6, 8)]  # Три препятствия
max_iters = 10000
step_size = 0.1

path = rrt(start, goal, obstacles, max_iters, step_size)

if path:
    print("Путь найден:", path)
else:
    print("Путь не найден")

# Визуализация
x, y = zip(*path)
obstacle_x, obstacle_y = zip(*obstacles)
plt.plot(x, y, marker='o', linestyle='-', label='Путь')
plt.scatter(obstacle_x, obstacle_y, color='red', marker='x', label='Препятствия')
plt.legend()
plt.grid(True)
plt.show()