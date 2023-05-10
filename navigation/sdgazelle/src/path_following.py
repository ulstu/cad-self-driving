#!/usr/bin/env python3

import rospy
import time
import roslib
import math
import sys
import csv

from std_msgs.msg import String


class POINT:
    def __init__(self, x, y, angle=0):
        self.x = x
        self.y = y
        self.angle = angle


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


class PathFollower:
    def degree_to_int(self, degree):
        return degree / 360 * 4294967296

    def getpos_callback(self, data):
        # Получение текущих кординат от GPS (УГОЛ ПОВОРОТА ПОКА НЕ ПЕРЕДАЕТСЯ)
        data = str(data).replace("data:", "")
        vars = str(data).split(';')
        lat = self.degree_to_int(float(vars[0].split()[1]))
        lon = self.degree_to_int(float(vars[1].split()[1]))
        angle = float(vars[5].split()[1][:-1])
        position = POINT(lon, lat, angle)

        if len(self.destinations) > 0:
            # Расчет всех углов и векторов
            angle_vector = VECTOR2(math.cos(math.radians(position.angle)) * 10, math.sin(math.radians(position.angle)) * 10)
            destination_vector = VECTOR2(self.destinations[0].x - position.x, self.destinations[0].y - position.y)
            destination_angle = destination_vector.AngleOfVectors(destination_vector, angle_vector)
            departure_vector = VECTOR2(self.destinations[0].x - self.departure.x, self.destinations[0].y - self.departure.y)
            finish_angle = destination_vector.AngleOfVectors(destination_vector, departure_vector)
            print(destination_angle)

            # Проверка на завершение участка пути
            if not self.is_achieved and abs(finish_angle) > 90:
                self.destinations.pop(0)
                self.departure = POINT(position.x, position.y, 0)

                # Определение и отправка направления движения
                if self.destinations[0].angle == "point":
                    if self.drive_direction != 1:
                        self.serial_pub.publish("12 0")
                        time.sleep(2)
                        self.serial_pub.publish("11 1")
                        time.sleep(7)
                        self.serial_pub.publish("12 1")
                    self.drive_direction = 1
                elif self.destinations[0].angle == "return":
                    if self.drive_direction != -1:
                        self.serial_pub.publish("12 0")
                        time.sleep(2)
                        self.serial_pub.publish("11 6")
                        time.sleep(7)
                        self.serial_pub.publish("12 1")
                    self.drive_direction = -1

            # Отправка угла поворота руля
            print("Target angle:", str(max(min(destination_angle * 10, 500), -500)))
            self.serial_pub.publish("16 " + str(max(min(destination_angle * 10, 500), -500) * 3.7))
        else:
            # Окончание пути и установка рулевого колеса в центральное положение
            self.serial_pub.publish("16 0")
            self.is_achieved = True


    def __init__(self):
        self.is_init = False

        # self.is_turn = False
        # self.is_stop = False
        # self.pos_history = []
        self.is_achieved = False
        self.drive_direction = 0

        # Чтение целевых кординат из csv файла
        self.destinations = []
        with open(rospy.get_param('~osm.gpspath'), 'r') as f:
            for row in csv.reader(f, delimiter=','):
                self.destinations.append(POINT(-self.degree_to_int(float(row[1])), -self.degree_to_int(float(row[0])), row[2]))

        # Определение кординат точки отправления
            self.departure = POINT(self.destinations[0].x, self.destinations[0].y, 0)
            self.destinations.pop(0)

        self.serial_pub = rospy.Publisher("serialcode", String, queue_size=1)
        rospy.Subscriber("gpspos", String, self.getpos_callback)
        # rospy.Subscriber("carstate", String, self.carstate_callback)
        
        time.sleep(5)
        self.is_init = True
        

if __name__ == '__main__':
    try:
        rospy.init_node('path_follower', anonymous=True)
        detector = PathFollower()

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка path follower: {}'.format(traceback.format_exc()))
