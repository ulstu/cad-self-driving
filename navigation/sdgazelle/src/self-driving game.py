import pygame
import math


def det(arr):
    return (arr[0][0] * arr[1][1]) - (arr[0][1] * arr[1][0])


class POINT:

    def __init__(self, x, y, angle=0):
        self.x = x
        self.y = y
        self.angle = angle


def distance(A, B):
    return math.sqrt((A.x - B.x) ** 2 + (A.y - B.y) ** 2)


def is_between(A, B, C):
    a = distance(B, C)
    b = distance(C, A)
    c = distance(A, B)
    return a ** 2 + b ** 2 >= c ** 2 and a ** 2 + c ** 2 >= b ** 2


class STRAIGHT:

    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2

    def intersection_bool(self, x3, y3, x4, y4):
        try:
            if abs((self.x1 - self.x2) * (y3 - y4) - (self.y1 - self.y2) * (x3 - x4)) > 100:
                Px = ((self.x1 * self.y2 - self.y1 * self.x2) * (x3 - x4) - (self.x1 - self.x2) * (
                            x3 * y4 - y3 * x4)) / ((self.x1 - self.x2) * (y3 - y4) - (self.y1 - self.y2) * (x3 - x4))
            else:
                Px = ((self.x1 * self.y2 - self.y1 * self.x2) * (x3 - x4) - (self.x1 - self.x2) * (
                            x3 * y4 - y3 * x4)) / 100

            if abs((self.x1 - self.x2) * (y3 - y4) - (self.y1 - self.y2) * (x3 - x4)) > 100:
                Py = ((self.x1 * self.y2 - self.y1 * self.x2) * (y3 - y4) - (self.y1 - self.y2) * (
                            x3 * y4 - y3 * x4)) / ((self.x1 - self.x2) * (y3 - y4) - (self.y1 - self.y2) * (x3 - x4))
            else:
                Py = ((self.x1 * self.y2 - self.y1 * self.x2) * (x3 - x4) - (self.x1 - self.x2) * (
                            x3 * y4 - y3 * x4)) / 100
        except:
            return False

        if is_between(POINT(Px, Py), POINT(x3, y3), POINT(x4, y4)) and is_between(POINT(Px, Py),
                                                                                  POINT(self.x1, self.y1),
                                                                                  POINT(self.x2, self.y2)):
            return True
        else:
            return False

    def intersection(self, x3, y3, x4, y4):
        if abs((self.x1 - self.x2) * (y3 - y4) - (self.y1 - self.y2) * (x3 - x4)) > 100:
            Px = ((self.x1 * self.y2 - self.y1 * self.x2) * (x3 - x4) - (self.x1 - self.x2) * (x3 * y4 - y3 * x4)) / (
                        (self.x1 - self.x2) * (y3 - y4) - (self.y1 - self.y2) * (x3 - x4))
        else:
            Px = ((self.x1 * self.y2 - self.y1 * self.x2) * (x3 - x4) - (self.x1 - self.x2) * (x3 * y4 - y3 * x4)) / 100

        if abs((self.x1 - self.x2) * (y3 - y4) - (self.y1 - self.y2) * (x3 - x4)) > 100:
            Py = ((self.x1 * self.y2 - self.y1 * self.x2) * (y3 - y4) - (self.y1 - self.y2) * (x3 * y4 - y3 * x4)) / (
                        (self.x1 - self.x2) * (y3 - y4) - (self.y1 - self.y2) * (x3 - x4))
        else:
            Py = ((self.x1 * self.y2 - self.y1 * self.x2) * (x3 - x4) - (self.x1 - self.x2) * (x3 * y4 - y3 * x4)) / 100

        if is_between(POINT(Px, Py), POINT(x3, y3), POINT(x4, y4)) and is_between(POINT(Px, Py),
                                                                                  POINT(self.x1, self.y1),
                                                                                  POINT(self.x2, self.y2)):
            return POINT(Px, Py, int(distance(POINT(x3, y3), POINT(Px, Py))))
        else:
            return POINT(x4, y4, int(distance(POINT(x3, y3), POINT(x4, y4))))


class VECTOR2:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def median(self, x, y, k):
        self.x = (self.x - x) * k + x
        self.y = (self.y - y) * k + y
        return VECTOR2(self.x, self.y)

    def get_length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    # reference = VECTOR2(1, 0)

    def AngleOfReference(self, v):
        return self.NormalizeAngle(math.atan2(v.y, v.x) / math.pi * 180)

    def AngleOfVectors(self, first, second):
        return self.NormalizeAngle(self.AngleOfReference(first) - self.AngleOfReference(second))

    def NormalizeAngle(self, angle):
        if angle > -180:
            turn = -360
        else:
            turn = 360
        while not (angle > -180 and angle <= 180):
            angle += turn
        return angle


class CAMERA:
    def __init__(self, position=POINT(0, 0, 0)):
        self.position = position
        self.is_up = False
        self.is_left = False
        self.is_right = False
        self.is_down = False


class PLAYER:

    def __init__(self, position=POINT(0, 0, 0), velocity=0):
        self.position = position
        self.velocity = velocity
        self.vector = VECTOR2(0, 0)
        self.steering_angle = 0.0
        self.is_w = False
        self.is_a = False
        self.is_s = False
        self.is_d = False


def solve_angle(angle, add):
    angle += add
    angle %= 360
    return angle


def to_radians(angle):
    angle = float(angle - 90)
    angle = (angle / 180) * 3.14159
    return angle


def blit_rotate(surf, image, pos, originPos, angle):
    angle = -angle
    # calculate the axis aligned bounding box of the rotated image
    w, h = image.get_size()
    box = [pygame.math.Vector2(p) for p in [(0, 0), (w, 0), (w, -h), (0, -h)]]
    box_rotate = [p.rotate(angle) for p in box]
    min_box = (min(box_rotate, key=lambda p: p[0])[0], min(box_rotate, key=lambda p: p[1])[1])
    max_box = (max(box_rotate, key=lambda p: p[0])[0], max(box_rotate, key=lambda p: p[1])[1])

    # calculate the translation of the pivot
    pivot = pygame.math.Vector2(originPos[0], -originPos[1])
    pivot_rotate = pivot.rotate(angle)
    pivot_move = pivot_rotate - pivot

    # calculate the upper left origin of the rotated image
    origin = (pos[0] - originPos[0] + min_box[0] - pivot_move[0], pos[1] - originPos[1] - max_box[1] + pivot_move[1])

    # get a rotated image
    rotated_image = pygame.transform.rotate(image, angle)

    # rotate and blit the image
    surf.blit(rotated_image, origin)


# параметры окна
size = (1200, 800)
screen = pygame.display.set_mode(size)
clock = pygame.time.Clock()
screen.fill((200, 200, 200))
pygame.display.set_caption("Initial N")
pygame.mixer.init()

# изображения
trueno = pygame.image.load("trueno.png")

# Следование до точки
destinations = []
is_achieved = True
is_reverse = False
destination = POINT(0, 0, 0)
departure = POINT(0, 0, 0)
destination_angle = 0
pygame.font.init()
heading_font = pygame.font.SysFont("Arial", 20)

# пременные
fps = 60
player = PLAYER(POINT(200, 1000, 0), 0)
camera = CAMERA(POINT(0, 0, 0))
show_lines = 3

# карта стен
walls = [STRAIGHT(203, 170, 355, 104), STRAIGHT(355, 104, 1169, 100), STRAIGHT(1169, 100, 1516, 184),
         STRAIGHT(1516, 184, 1521, 397), STRAIGHT(1521, 397, 1395, 600), STRAIGHT(1395, 600, 1155, 829),
         STRAIGHT(1155, 829, 1196, 943), STRAIGHT(1196, 943, 1282, 979), STRAIGHT(1282, 979, 1946, 610),
         STRAIGHT(1946, 610, 2302, 556), STRAIGHT(2302, 556, 2512, 631), STRAIGHT(2512, 631, 2570, 998),
         STRAIGHT(2570, 998, 2342, 1295), STRAIGHT(2342, 1295, 1823, 1717), STRAIGHT(1823, 1717, 484, 1834),
         STRAIGHT(484, 1834, 51, 1683), STRAIGHT(51, 1683, 80, 300), STRAIGHT(80, 300, 203, 170),
         STRAIGHT(338, 488, 411, 372), STRAIGHT(411, 372, 1009, 333), STRAIGHT(1009, 333, 1200, 356),
         STRAIGHT(1200, 356, 1208, 411), STRAIGHT(1208, 411, 739, 746), STRAIGHT(739, 746, 642, 971),
         STRAIGHT(642, 971, 927, 1315), STRAIGHT(927, 1315, 1296, 1343), STRAIGHT(1296, 1343, 2090, 884),
         STRAIGHT(2090, 884, 2184, 1016), STRAIGHT(2184, 1016, 1621, 1419), STRAIGHT(1621, 1419, 774, 1426),
         STRAIGHT(774, 1426, 423, 1311), STRAIGHT(423, 1311, 338, 989), STRAIGHT(338, 989, 338, 488)]

# музыка
try:
    pygame.mixer.music.load("Space Boy.mp3")
    pygame.mixer.music.set_volume(0.05)
    # pygame.mixer.music.play()
except:
    pass

# отрисовка
runGame = True
while runGame:
    # Отслеживание событий для движения
    for event in pygame.event.get():
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                is_reverse = False
                destinations.append(POINT(pygame.mouse.get_pos()[0] - camera.position.x,
                                          pygame.mouse.get_pos()[1] - camera.position.y, 0))
                # destination = POINT(pygame.mouse.get_pos()[0] - camera.position.x,
                #                     pygame.mouse.get_pos()[1] - camera.position.y, 0)
                departure = POINT(player.position.x, player.position.y, 0)
            if event.button == 2:
                destinations.clear()
                is_achieved = True
            if event.button == 3:
                is_reverse = True
                destinations.append(POINT(pygame.mouse.get_pos()[0] - camera.position.x,
                                          pygame.mouse.get_pos()[1] - camera.position.y, 90))
                # destination = POINT(pygame.mouse.get_pos()[0] - camera.position.x,
                #                     pygame.mouse.get_pos()[1] - camera.position.y, 0)
                departure = POINT(player.position.x, player.position.y, 0)

        if event.type == pygame.KEYDOWN:
            if event.key == 99:
                is_achieved = False
            elif event.key == 118:
                is_achieved = True
            elif event.key == 27:
                runGame = False
            elif event.key == 108:
                if show_lines == 3:
                    show_lines = 0
                else:
                    show_lines += 1
            elif event.key == 100:
                player.is_d = True
            elif event.key == 97:
                player.is_a = True
            elif event.key == 119:
                player.is_w = True
            elif event.key == 115:
                player.is_s = True

            elif event.key == 1073741904:
                camera.is_left = True
            elif event.key == 1073741906:
                camera.is_up = True
            elif event.key == 1073741903:
                camera.is_right = True
            elif event.key == 1073741905:
                camera.is_down = True

        if event.type == pygame.KEYUP:
            if event.key == 27:
                runGame = False
            elif event.key == 100:
                player.is_d = False
            elif event.key == 97:
                player.is_a = False
            elif event.key == 119:
                player.is_w = False
            elif event.key == 115:
                player.is_s = False

            elif event.key == 1073741904:
                camera.is_left = False
            elif event.key == 1073741906:
                camera.is_up = False
            elif event.key == 1073741903:
                camera.is_right = False
            elif event.key == 1073741905:
                camera.is_down = False

    # Применение событий для машины
    if not is_achieved:
        if not is_reverse:
            if player.velocity < 5:
                player.velocity += 1 * (1 / 2)
            player.steering_angle = max(min(destination_angle, 3.5), -3.5)
        else:
            if player.velocity > -2:
                player.velocity -= 1 * (1 / 2)
            player.steering_angle = -max(min(destination_angle, 3.5), -3.5)
    else:
        if not is_reverse:
            if player.velocity > 1:
                player.velocity -= 1 * (1 / 2)
        else:
            if player.velocity < 0:
                player.velocity += 1 * (1 / 2)
        player.steering_angle = 0
    # if player.is_d and player.steering_angle <= 3.5:
    #     player.steering_angle += 0.2 * (1 / 2)
    # if player.is_a and player.steering_angle >= -3.5:
    #     player.steering_angle -= 0.2 * (1 / 2)
    # if player.steering_angle <= 3.5:
    #     player.steering_angle += 0.2 * (1 / 2)
    # if player.is_a and player.steering_angle >= -3.5:
    #     player.steering_angle -= 0.2 * (1 / 2)

    # Применение событий для камеры
    if camera.is_left:
        camera.position.x += 20
    if camera.is_right:
        camera.position.x -= 20
    if camera.is_up:
        camera.position.y += 20
    if camera.is_down:
        camera.position.y -= 20

    # перемещение

    camera.position.x -= (camera.position.x + player.position.x - 600) * 0.15 * (1 / 2)
    camera.position.y -= (camera.position.y + player.position.y - 400) * 0.15 * (1 / 2)

    player.position.angle = solve_angle(player.position.angle,
                                        player.steering_angle * (player.vector.get_length() / 10))

    direction_vector = VECTOR2(math.cos(to_radians(player.position.angle)) * player.velocity * (1 / 2),
                               math.sin(to_radians(player.position.angle)) * player.velocity * (1 / 2))
    player.vector.median(direction_vector.x, direction_vector.y, 0.7)

    player.position.x += player.vector.x
    player.position.y += player.vector.y

    # притормаживание
    if player.velocity > 0:
        player.velocity -= 0.25 * (1 / 2)
    # фигня какая-то
    if (not player.is_d) and (not player.is_a) and player.steering_angle > 0:
        player.steering_angle -= 0.1
    if (not player.is_a) and (not player.is_d) and player.steering_angle < 0:
        player.steering_angle += 0.1

    screen.fill((200, 200, 200))
    blit_rotate(screen, trueno, (player.position.x + camera.position.x, player.position.y + camera.position.y),
                (29, 76), player.position.angle)

    point_front = POINT(player.position.x + math.cos(to_radians(player.position.angle)) * 1000,
                        player.position.y + math.sin(to_radians(player.position.angle)) * 1000)
    point_half_left = POINT(player.position.x + math.cos(to_radians(player.position.angle + 35)) * 1000,
                            player.position.y + math.sin(to_radians(player.position.angle + 35)) * 1000)

    point_half_right = POINT(player.position.x + math.cos(to_radians(player.position.angle - 35)) * 1000,
                             player.position.y + math.sin(to_radians(player.position.angle - 35)) * 1000)
    point_left = POINT(player.position.x + math.cos(to_radians(player.position.angle + 90)) * 1000,
                       player.position.y + math.sin(to_radians(player.position.angle + 90)) * 1000)

    point_right = POINT(player.position.x + math.cos(to_radians(player.position.angle - 90)) * 1000,
                        player.position.y + math.sin(to_radians(player.position.angle - 90)) * 1000)

    for wall in walls:
        pygame.draw.line(screen, pygame.Color(255, 0, 0), [wall.x1 + camera.position.x, wall.y1 + camera.position.y],
                         [wall.x2 + camera.position.x, wall.y2 + camera.position.y], 2)

        point_front = wall.intersection(player.position.x, player.position.y,
                                        point_front.x, point_front.y)
        point_half_left = wall.intersection(player.position.x, player.position.y,
                                            point_half_left.x, point_half_left.y)
        point_half_right = wall.intersection(player.position.x, player.position.y,
                                             point_half_right.x, point_half_right.y)
        point_left = wall.intersection(player.position.x, player.position.y,
                                       point_left.x, point_left.y)
        point_right = wall.intersection(player.position.x, player.position.y,
                                        point_right.x, point_right.y)

    if show_lines == 0 or show_lines == 2:
        pygame.draw.line(screen, pygame.Color(0, 0, 255),
                         [player.position.x + camera.position.x, player.position.y + camera.position.y],
                         [point_front.x + camera.position.x, point_front.y + camera.position.y], 2)
        pygame.draw.circle(screen, pygame.Color(0, 0, 0),
                           (point_front.x + camera.position.x, point_front.y + camera.position.y), 5.0)
        pygame.draw.line(screen, pygame.Color(0, 0, 255),
                         [player.position.x + camera.position.x, player.position.y + camera.position.y],
                         [point_half_left.x + camera.position.x, point_half_left.y + camera.position.y], 2)
        pygame.draw.circle(screen, pygame.Color(0, 0, 0),
                           (point_half_left.x + camera.position.x, point_half_left.y + camera.position.y), 5.0)
        pygame.draw.line(screen, pygame.Color(0, 0, 255),
                         [player.position.x + camera.position.x, player.position.y + camera.position.y],
                         [point_half_right.x + camera.position.x, point_half_right.y + camera.position.y], 2)
        pygame.draw.circle(screen, pygame.Color(0, 0, 0),
                           (point_half_right.x + camera.position.x, point_half_right.y + camera.position.y), 5.0)
        pygame.draw.line(screen, pygame.Color(0, 0, 255),
                         [player.position.x + camera.position.x, player.position.y + camera.position.y],
                         [point_left.x + camera.position.x, point_left.y + camera.position.y], 2)
        pygame.draw.circle(screen, pygame.Color(0, 0, 0),
                           (point_left.x + camera.position.x, point_left.y + camera.position.y), 5.0)
        pygame.draw.line(screen, pygame.Color(0, 0, 255),
                         [player.position.x + camera.position.x, player.position.y + camera.position.y],
                         [point_right.x + camera.position.x, point_right.y + camera.position.y], 2)
        pygame.draw.circle(screen, pygame.Color(0, 0, 0),
                           (point_right.x + camera.position.x, point_right.y + camera.position.y), 5.0)

    is_crash = False
    front_corner_l = POINT(
        player.position.x + (75 * math.cos(to_radians(player.position.angle + 20))),
        player.position.y + (75 * math.sin(to_radians(player.position.angle + 20))))
    front_corner_r = POINT(
        player.position.x + (75 * math.cos(to_radians(player.position.angle - 20))),
        player.position.y + (75 * math.sin(to_radians(player.position.angle - 20))))
    rear_corner_l = POINT(
        player.position.x + (75 * math.cos(to_radians(player.position.angle + 160))),
        player.position.y + (75 * math.sin(to_radians(player.position.angle + 160))))
    rear_corner_r = POINT(
        player.position.x + (75 * math.cos(to_radians(player.position.angle - 160))),
        player.position.y + (75 * math.sin(to_radians(player.position.angle - 160))))

    # отрисовка "векторов" ноправлений
    if (show_lines == 1 or show_lines == 2) and len(destinations) > 0:
        # pygame.draw.line(screen, pygame.Color(0, 255, 0),
        #                  [player.position.x + camera.position.x, player.position.y + camera.position.y],
        #                  [player.position.x + (player.vector.x * 10) + camera.position.x,
        #                   player.position.y + (player.vector.y * 10) + camera.position.y], 3)

        # pygame.draw.line(screen, pygame.Color(255, 0, 0),
        #                  [player.position.x + camera.position.x, player.position.y + camera.position.y],
        #                  [player.position.x + (direction_vector.x * 10) + camera.position.x,
        #                   player.position.y + (direction_vector.y * 10) + camera.position.y], 3)

        angle_vector = VECTOR2(math.cos(to_radians(player.position.angle)) * 10 * (1 / 2),
                               math.sin(to_radians(player.position.angle)) * 10 * (1 / 2))
        # pygame.draw.line(screen, pygame.Color(255, 0, 0),
        #                  [player.position.x + camera.position.x, player.position.y + camera.position.y],
        #                  [player.position.x + (angle_vector.x * 20) + camera.position.x,
        #                   player.position.y + (angle_vector.y * 20) + camera.position.y], 3)


        pygame.draw.line(screen, pygame.Color(255, 255, 0),
                         [player.position.x + camera.position.x, player.position.y + camera.position.y],
                         [destinations[0].x + camera.position.x,
                          destinations[0].y + camera.position.y], 3)
        for destination_iterator in range(1, len(destinations)):
            if destinations[destination_iterator].angle == 0:
                pygame.draw.line(screen, pygame.Color(255, 150, 150),
                                 [destinations[destination_iterator].x + camera.position.x,
                                  destinations[destination_iterator].y + camera.position.y],
                                 [destinations[destination_iterator - 1].x + camera.position.x,
                                  destinations[destination_iterator - 1].y + camera.position.y], 3)
            else:
                pygame.draw.line(screen, pygame.Color(150, 150, 255),
                                 [destinations[destination_iterator].x + camera.position.x,
                                  destinations[destination_iterator].y + camera.position.y],
                                 [destinations[destination_iterator - 1].x + camera.position.x,
                                  destinations[destination_iterator - 1].y + camera.position.y], 3)

        if destinations[0].angle == 0:
            is_reverse = False
        else:
            is_reverse = True

        destination_vector = VECTOR2(destinations[0].x - player.position.x, destinations[0].y - player.position.y)
        destination_angle = destination_vector.AngleOfVectors(destination_vector, angle_vector)
        departure_vector = VECTOR2(destinations[0].x - departure.x, destinations[0].y - departure.y)
        finish_angle = destination_vector.AngleOfVectors(destination_vector, departure_vector)
        if not is_achieved and abs(finish_angle) > 90:
            destinations.pop(0)
            departure = POINT(player.position.x, player.position.y, 0)
            if len(destinations) == 0:
                is_achieved = True

        # pygame.draw.line(screen, pygame.Color(0, 160, 160),
        #                  [destination.x + camera.position.x, destination.y + camera.position.y],
        #                  [departure.x + camera.position.x, departure.y + camera.position.y], 3)

        # label = heading_font.render(str(destination_vector.AngleOfVectors(destination_vector, angle_vector)), True, (50, 50, 50))
        # label_rect = label.get_rect()
        # label_rect.center = (130, 25)
        # screen.blit(label, label_rect)
        #
        # label = heading_font.render(str(destination_vector.AngleOfVectors(destination_vector, departure_vector)), True, (50, 50, 50))
        # label_rect = label.get_rect()
        # label_rect.center = (130, 50)
        # screen.blit(label, label_rect)

    pygame.draw.circle(screen, pygame.Color(255, 0, 0),
                       [destination.x + camera.position.x, destination.y + camera.position.y], 5)

    for wall in walls:
        if wall.intersection_bool(front_corner_l.x, front_corner_l.y,
                                  front_corner_r.x, front_corner_r.y):
            is_crash = True
            '''pygame.draw.line(screen, pygame.Color(255, 0, 0),
                             [front_corner_l.x + camera.position.x, front_corner_l.y + camera.position.y],
                             [front_corner_r.x + camera.position.x, front_corner_r.y + camera.position.y], 3)'''

        if wall.intersection_bool(front_corner_l.x, front_corner_l.y,
                                  rear_corner_l.x, rear_corner_l.y):
            is_crash = True
            '''pygame.draw.line(screen, pygame.Color(255, 0, 0),
                             [front_corner_l.x + camera.position.x, front_corner_l.y + camera.position.y],
                             [rear_corner_l.x + camera.position.x, rear_corner_l.y + camera.position.y], 3)'''

        if wall.intersection_bool(rear_corner_r.x, rear_corner_r.y,
                                  front_corner_r.x, front_corner_r.y):
            is_crash = True
            '''pygame.draw.line(screen, pygame.Color(255, 0, 0),
                             [rear_corner_r.x + camera.position.x, rear_corner_r.y + camera.position.y],
                             [front_corner_r.x + camera.position.x, front_corner_r.y + camera.position.y], 3)'''

        if wall.intersection_bool(rear_corner_r.x, rear_corner_r.y,
                                  rear_corner_l.x, rear_corner_l.y):
            is_crash = True
            '''pygame.draw.line(screen, pygame.Color(255, 0, 0),
                             [rear_corner_r.x + camera.position.x, rear_corner_r.y + camera.position.y],
                             [rear_corner_l.x + camera.position.x, rear_corner_l.y + camera.position.y], 3)'''

        if is_crash:
            player = PLAYER(POINT(200, 1000, 0), 0)

    pygame.display.update()

    clock.tick(fps)

pygame.quit()
