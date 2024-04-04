import numpy as np
import cv2
import random
import math

class RRTTreeNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

    def __repr__(self):
        return f'x: {int(self.x)}; y: {int(self.y)}|'

def euclidean_distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

def is_collision(node, obstacles, car_length, car_width, logger=None):
    x, y = int(node.x), int(node.y)
    half_car_length = car_length / 2
    half_car_width = car_width / 2
    
    for i in range(x - int(half_car_length), x + int(half_car_length) + 1):
        for j in range(y - int(half_car_width), y + int(half_car_width) + 1):
            if 0 <= i < obstacles.shape[1] and 0 <= j < obstacles.shape[0]:
                if obstacles[j, i] != 100:
                    return True
    return False

def rrt_star(start, goal, obstacles, max_iters, step_size, car_length, car_width, logger=None):
    nodes = [start]
    
    for i in range(max_iters):
        rand_node = RRTTreeNode(random.uniform(0, obstacles.shape[1]), random.uniform(0, obstacles.shape[0]))
        nearest_node = min(nodes, key=lambda node: euclidean_distance(node, rand_node))
        
        theta = math.atan2(rand_node.y - nearest_node.y, rand_node.x - nearest_node.x)
        new_node = RRTTreeNode(nearest_node.x + step_size * math.cos(theta), nearest_node.y + step_size * math.sin(theta))
        
        if euclidean_distance(new_node, nearest_node) > step_size:
            continue
        
        if not is_collision(new_node, obstacles, car_length, car_width, logger):
            near_nodes = [node for node in nodes if euclidean_distance(node, new_node) <= step_size]
            min_cost_node = nearest_node
            min_cost = nearest_node.cost + euclidean_distance(nearest_node, new_node)
            
            for near_node in near_nodes:
                if near_node.cost + euclidean_distance(near_node, new_node) < min_cost:
                    min_cost_node = near_node
                    min_cost = near_node.cost + euclidean_distance(near_node, new_node)
            
            new_node.cost = min_cost
            new_node.parent = min_cost_node
            
            for near_node in near_nodes:
                if near_node.cost > new_node.cost + euclidean_distance(near_node, new_node):
                    near_node.parent = new_node
                    near_node.cost = new_node.cost + euclidean_distance(near_node, new_node)
        
            nodes.append(new_node)
            
    # Find the path with minimum cost to the goal
    path = []
    goal_node = min(nodes, key=lambda node: euclidean_distance(node, goal))

    current_node = goal_node
    while current_node is not None:
        path.append((int(current_node.x), int(current_node.y)))
        current_node = current_node.parent
    
    return path[::-1], nodes

def visualize_path(path, obstacles, car_length, car_width):
    #img = np.array(obstacles)
    img = np.ones((100, 100, 3), dtype=np.uint8) * 255
    
    for x, y in path:
        cv2.circle(img, (int(x), int(y)), 2, (0, 0, 255), -1)
    
    for i in range(obstacles.shape[0]):
        for j in range(obstacles.shape[1]):
            if obstacles[i, j] == 0:
                cv2.rectangle(img, (j, i), (j + 1, i + 1), (0, 0, 0), -1)
    
    cv2.imshow("RRT* Path", img)

if __name__ == "__main__":
    start_node = RRTTreeNode(10, 10)
    goal_node = RRTTreeNode(90, 90)
    
    obstacles = np.ones((100, 100), dtype=np.uint8) * 255
    obstacles[20:40, 30:50] = 0
    obstacles[60:80, 20:40] = 0
    
    max_iterations = 100
    step_size = 5
    car_length = 10  # Длина автомобиля
    car_width = 4    # Ширина автомобиля
    
    path = rrt_star(start_node, goal_node, obstacles, max_iterations, step_size, car_length, car_width)
    
    if path:
        print("Путь найден:", path)
        visualize_path(path, obstacles, car_length, car_width)
    else:
        print("Путь не найден.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
