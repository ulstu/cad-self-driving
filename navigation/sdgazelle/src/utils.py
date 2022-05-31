#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from enum import Enum
import rospy

class CarCmdParams(Enum):
    rpm = "rpm"
    transmission = "transmission"
    throttle = "throttle"
    velocity = "velocity"
    wheel = "wheel"

class CarState(Enum):
    INIT = 1        # начало инициализации автомобиля, всех его подсистем (запуск скриптов, калибровка датчиков и исполнительных механизмов и т.п.)
    WAIT = 2        # автомобиль готов к началу движения и ждет команды на старт
    MOVE_FRWD = 3   # автомобиль движется от старта до зоны разворота, объезжая препятствия
    MOVE_BACK = 4   # автомобиль движется от зоны разворота до финиша с объездом препятствий
    TURN = 5        # автомобиль выполняет разворот
    STOP = 6        # автомобиль останавливается в зоне финиша и готов к выключению (передача выключена, руль прямо, сцепление отжато, тормоз готов отпуститься при нажатии на кнопку после того, как будет зажат ручник)
    FINISHED = 7    # автомобиль готов к выключению

def clear_str(str):
    return str.translate(str.maketrans('','','\n\\ \t\"\'')).strip()

def calc_gps_distance(lat1, lon1, lat2, lon2):
    lat1 = float(lat1)
    lat2 = float(lat2)
    lon1 = float(lon1)
    lon2 = float(lon2)
    R = 6378.137
    dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
    dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    return d * 1000

def log(object, message):
    rospy.loginfo("{} || {}".format(type(object), message))
    pass