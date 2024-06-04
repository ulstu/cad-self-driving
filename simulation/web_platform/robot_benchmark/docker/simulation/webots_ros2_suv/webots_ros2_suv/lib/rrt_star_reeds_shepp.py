"""

Path planning Sample Code with RRT with Reeds-Shepp path

author: AtsushiSakai(@Atsushi_twi)

"""
import copy
import math
import random
import sys
import pathlib
import matplotlib.pyplot as plt
import numpy as np
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from webots_ros2_suv.lib.reeds_shepp_path_planning import reeds_shepp_path_planning, plot_arrow
from webots_ros2_suv.lib.rrt_star import RRTStar

show_animation = True


class RRTStarReedsShepp(RRTStar):
    """
    Class for RRT star planning with Reeds Shepp path
    """

    class Node(RRTStar.Node):
        """
        RRT Node
        """

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, rand_area,
                 max_iter=200, step_size=5,
                 connect_circle_dist=50.0,
                 robot_radius=0.0
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand_x = rand_area[0]
        self.max_rand_x = rand_area[1]
        self.min_rand_y = rand_area[2]
        self.max_rand_y = rand_area[3]
        self.max_iter = max_iter
        self.step_size = step_size
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist
        self.robot_radius = robot_radius

        self.curvature = 1.0
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5

    def set_random_seed(self, seed):
        random.seed(seed)

    def planning(self, animation=True, search_until_max_iter=True):
        """
        planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            #print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(
                    new_node, self.obstacle_list, self.robot_radius):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)
                    self.try_goal_path(new_node)

            if animation and i % 5 == 0:
                self.plot_start_goal_arrow()
                self.draw_graph(rnd)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None

    def try_goal_path(self, node):

        goal = self.Node(self.end.x, self.end.y, self.end.yaw)

        new_node = self.steer(node, goal)
        if new_node is None:
            return

        if self.check_collision(
                new_node, self.obstacle_list, self.robot_radius):
            self.node_list.append(new_node)

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        # for (ox, oy, size) in self.obstacle_list:
        #     plt.plot(ox, oy, "ok", ms=30 * size)
        plt.imshow(self.obstacle_list, cmap='hot')

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        # plt.axis([0, 100, 0, 100])
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.01)

    def plot_start_goal_arrow(self):
        plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

    def steer(self, from_node, to_node):

        px, py, pyaw, mode, course_lengths = reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw, to_node.x,
            to_node.y, to_node.yaw, self.curvature, self.step_size)

        if not px:
            return None

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += sum([abs(l) for l in course_lengths])
        new_node.parent = from_node

        return new_node

    def calc_new_cost(self, from_node, to_node):

        _, _, _, _, course_lengths = reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw, to_node.x,
            to_node.y, to_node.yaw, self.curvature, self.step_size)
        if not course_lengths:
            return float("inf")

        return from_node.cost + sum([abs(l) for l in course_lengths])

    def get_random_node(self):

        rnd = self.Node(random.uniform(self.min_rand_x, self.max_rand_x),
                        random.uniform(self.min_rand_y, self.max_rand_y),
                        random.uniform(-math.pi, math.pi)
                        )

        return rnd

    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)
        print("goal_indexes:", len(goal_indexes))

        # angle check
        final_goal_indexes = []
        for i in goal_indexes:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.goal_yaw_th:
                final_goal_indexes.append(i)

        print("final_goal_indexes:", len(final_goal_indexes))

        if not final_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in final_goal_indexes])
        print("min_cost:", min_cost)
        for i in final_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        return path


def generate_scene_fixed_obstacles(size=100, obstacle_value_range=(101, 200), min_obstacle_size=5, max_obstacle_size=8, num_obstacles=15):
    """
    Генерирует матрицу сцены размером size x size, где пустые клетки имеют значение 100,
    а препятствия - случайные значения от obstacle_value_range[0] до obstacle_value_range[1].
    Размер препятствий варьируется от min_obstacle_size до max_obstacle_size.
    Количество препятствий зафиксировано на num_obstacles.
    """
    # Инициализация матрицы пустыми клетками
    scene = np.full((size, size), 100)

    for _ in range(num_obstacles):
        # Размер препятствия
        obstacle_width = random.randint(min_obstacle_size, max_obstacle_size)
        obstacle_height = random.randint(min_obstacle_size, max_obstacle_size)

        # Случайное значение для препятствия
        obstacle_value = random.randint(*obstacle_value_range)

        # Случайная позиция препятствия
        x_position = random.randint(0, size - obstacle_width)
        y_position = random.randint(0, size - obstacle_height)

        # Размещение препятствия на сцене
        scene[y_position:y_position+obstacle_height, x_position:x_position+obstacle_width] = obstacle_value

    return scene


def main(max_iter=100):
    print("Start " + __file__)

    # ====Search Path with RRT====
    # Генерация сцены с 7 препятствиями
    obstacleList = generate_scene_fixed_obstacles()


    # Визуализация сцены с 7 препятствиями и обозначением начальной и конечной точек
    #visualize_scene_with_points(obstacleList)


    # Set Initial parameters
    start = [5.0, 5.0, np.deg2rad(0.0)]
    goal = [95.0, 95.0, np.deg2rad(180.0)]

    rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
                                             obstacleList,
                                             rand_area=[0.0, 100.0], step_size=2, max_iter=100)
    path = rrt_star_reeds_shepp.planning(animation=True)

    # # Draw final path
    if path and show_animation:  # pragma: no cover
        rrt_star_reeds_shepp.draw_graph()
        plt.plot([x for (x, y, yaw) in path], [y for (x, y, yaw) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()