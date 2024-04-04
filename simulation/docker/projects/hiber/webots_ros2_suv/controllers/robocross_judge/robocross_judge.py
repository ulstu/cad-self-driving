from controller import Supervisor

import optparse
import math


class Pedestrian (Supervisor):
    """Control a Pedestrian PROTO."""

    def __init__(self):
        """Constructor: initialize constants."""
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.22
        self.speed = 1.15
        self.current_height_offset = 0
        self.joints_position_field = []

        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]
        self.human_height_offsets = [  # Изменение высоты человека при ходьбе
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
        self.human_body_angles = [  # Углы между частями тела и туловищем
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],  # левая рука
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],  # левое предплечье
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],  # левая кисть
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],  # правая рука
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],  # правое предплечье
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],  # правая кисть
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],  # левая нога
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],  # левая голень
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],  # левая ступня
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],  # правая нога
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],  # правая голень
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],  # правая ступня
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]  # голова
        ]
        Supervisor.__init__(self)

    def run(self):
        """Set the Pedestrian pose and position."""
        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--points", default="", help="Точки траектории движения в формате [x1 y1 z1, x2 y2 z2, ...]")
        opt_parser.add_option("--speed", type=float, default=0.5, help="Скорость ходьбы, м/c")
        options, args = opt_parser.parse_args()

        if not options.points or len(options.points.split(',')) < 2:
            print("Траектория должна быть задана списком, содержащим хотя бы две точки.")
            return
        if options.speed and options.speed > 0:
            self.speed = options.speed

        self.time_step = int(self.getBasicTimeStep())

        point_list = options.points.split(',')
        self.number_of_waypoints = len(point_list)
        self.waypoints = []
        for i in range(0, self.number_of_waypoints):
            self.waypoints.append([])
            self.waypoints[i].append(float(point_list[i].split()[0]))
            self.waypoints[i].append(float(point_list[i].split()[1]))
            self.waypoints[i].append(float(point_list[i].split()[2]))
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")

        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field.append(self.root_node_ref.getField(self.joint_names[i]))

        # compute waypoints distance
        self.waypoints_distance = []
        self.waypoints_z_distance = []
        for i in range(0, self.number_of_waypoints):
            x = self.waypoints[i][0] - self.waypoints[(i + 1) % self.number_of_waypoints][0]
            y = self.waypoints[i][1] - self.waypoints[(i + 1) % self.number_of_waypoints][1]
            z = self.waypoints[i][2] - self.waypoints[(i + 1) % self.number_of_waypoints][2]
            if i == 0:
                self.waypoints_distance.append(math.sqrt(x * x + y * y))
                self.waypoints_z_distance.append(z)
            else:
                self.waypoints_distance.append(self.waypoints_distance[i - 1] + math.sqrt(x * x + y * y))
                self.waypoints_z_distance.append(z)

        while not self.step(self.time_step) == -1:
            time = self.getTime()

            current_sequence = int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO) % self.WALK_SEQUENCES_NUMBER)
            # compute the ratio 'distance already covered between way-point(X) and way-point(X+1)'
            # / 'total distance between way-point(X) and way-point(X+1)'
            ratio = (time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO - \
                int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO))


            for i in range(0, self.BODY_PARTS_NUMBER):
                current_angle = self.human_body_angles[i][current_sequence] * (1 - ratio) + \
                    self.human_body_angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                self.joints_position_field[i].setSFFloat(current_angle)

            # adjust height
            self.current_height_offset = self.human_height_offsets[current_sequence] * (1 - ratio) + \
                self.human_height_offsets[(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio

            # move everything
            distance = time * self.speed



            relative_distance = distance - int(distance / self.waypoints_distance[self.number_of_waypoints - 1]) * \
                self.waypoints_distance[self.number_of_waypoints - 1]


            for i in range(0, self.number_of_waypoints):
                if self.waypoints_distance[i] > relative_distance:
                    break

            distance_ratio = 0
            if i == 0:
                distance_ratio = relative_distance / self.waypoints_distance[0]
            else:
                distance_ratio = (relative_distance - self.waypoints_distance[i - 1]) / \
                    (self.waypoints_distance[i] - self.waypoints_distance[i - 1])
            x = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][0] + \
                (1 - distance_ratio) * self.waypoints[i][0]
            y = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][1] + \
                (1 - distance_ratio) * self.waypoints[i][1]

            self.current_height_offset += self.waypoints[i][2] - distance_ratio * self.waypoints_z_distance[i]

            root_translation = [x, y, self.ROOT_HEIGHT + self.current_height_offset]
            angle = math.atan2(self.waypoints[(i + 1) % self.number_of_waypoints][1] - self.waypoints[i][1],
                               self.waypoints[(i + 1) % self.number_of_waypoints][0] - self.waypoints[i][0])
            rotation = [0, 0, 1, angle]

            self.root_translation_field.setSFVec3f(root_translation)
            self.root_rotation_field.setSFRotation(rotation)


controller = Pedestrian()
controller.run()
