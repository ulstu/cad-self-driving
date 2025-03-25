import math


def det(arr):
    return (arr[0][0] * arr[1][1]) - (arr[0][1] * arr[1][0])


def vector_angle(vector):
    return math.atan2(vector.y, vector.x)


def distance(A, B):
    return math.sqrt((A.x - B.x) ** 2 + (A.y - B.y) ** 2)


def is_between(A, B, C):
    a = distance(B, C)
    b = distance(C, A)
    c = distance(A, B)
    return a ** 2 + b ** 2 >= c ** 2 and a ** 2 + c ** 2 >= b ** 2


class POINT:

    def __init__(self, x, y, angle=0, direction = 1):
        self.x = x
        self.y = y
        self.angle = angle
        self.direction = direction

    def add(self, element):
        return POINT(self.x + element.x, self.y + element.y, self.angle, self.direction)

    def __repr__(self):
        return f"x:{self.x}, y:{self.y}, a:{self.angle}, d:{self.direction}"


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

    def rotate(self, radians):
        x_prime = self.x * math.cos(radians) - self.y * math.sin(radians)
        y_prime = self.x * math.sin(radians) + self.y * math.cos(radians)
        return VECTOR2(x_prime, y_prime)

    def mult(self, scalar):
        return VECTOR2(self.x * scalar, self.y * scalar)

    def __add__(self, other):
        return VECTOR2(self.x + other.x, self.y + other.y)

    def __repr__(self):
        return f"x: {self.x}, y: {self.y}"


def solve_angle(angle, add):
    angle += add
    angle %= 360
    return angle


def angle_difference(angle1, angle2):
    # Приведение углов к интервалу от 0 до 360 градусов
    angle1 = angle1 % 360
    angle2 = angle2 % 360

    # Вычисление разности углов
    diff = angle1 - angle2

    # Приведение разности к интервалу от -180 до 180 градусов
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360

    return diff