#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import csv
from std_msgs.msg import String
from utils import *
from enum import Enum
import sys
import traceback
import math
import time

class CarPositionController(object):
    points = []
    pos = {'klat': 0, 'klon': 0, 'velocity': 0, 'lat': 0, 'lon': 0}
    cur_state = CarState.INIT

    point_start = {'lat': 0, 'lon': 0}
    point_turn = {'lat': 0, 'lon': 0}
    point_stop = {'lat': 0, 'lon': 0}

    dist_after_start = 0
    dist_before_turn = 0
    dist_after_turn = 0
    dist_before_stop = 0
    dist_full = 0
    distpub = None
    controlpub = None
    turn_point_index = 0

    def getpos_callback(self, data):
        '''
        Callback для получения GPS позиции из топика
        '''
        data = str(data).replace("data:", "")
        vars = str(data).split(';')
        for v in vars:
            keyval = v.split(':')
            self.pos[clear_str(keyval[0])] = clear_str(keyval[1])
        if not rospy.get_param("~usekalman"):
            self.pos["klat"] = self.pos["lat"]
            self.pos["klon"] = self.pos["lon"]
        #rospy.loginfo("lat:{}; lon:{}".format(self.pos["klat"], self.pos["klon"]))


    def init_car(self):
        '''
        Инициализация всех систем автомобиля
        '''
        try:
            log(self, 'Начало инициализации всех систем автомобиля')
            self.publish_state("init")
            # необходимо дописать код по инициализации и проверке всех систем автомобиля
            # check GPS
            #while float(self.pos["klat"]) == 0.0:
            #    rospy.sleep(1)
            log(self, 'Все системы готовы к старту')
            rospy.sleep(0.5)
            self.cur_state = CarState.WAIT
        except:
            rospy.logerr('Ошибка инициализации: {}'.format(traceback.format_exc()))

    def wait_start_car(self):
        '''
        Ожидание команды "старт" (обработка нажатия кнопки)
        '''
        try:
            startkey_pressed = False
            log(self, 'Автомобиль готов и ждет команды для запуска движения по заданию')
            while not rospy.is_shutdown():
                # необходимо написать код, который реагирует на нажатие кнопки - старта 
                # алгоритма выполнения задания соревнования, после этого убрать break
                startkey_pressed = True
                if startkey_pressed:
                    break                    
                rospy.sleep(0.5)
            log(self, 'Автомобиль вышел из состояния ожидания')
            self.cur_state = CarState.MOVE_FRWD
            self.publish_state("start")
        except:
            rospy.logerr('Ошибка инициализации: {}'.format(traceback.format_exc()))

    def calc_distances(self):
        '''
        Вычисление дистанций от текущей точки до начала, до разворота, до финиша
        '''
        try:

            self.dist_after_start = 0
            self.dist_before_turn = 0
            self.dist_after_turn = 0
            self.dist_before_stop = 0
            self.dist_full = 0
            
            for i in range(len(self.points) - 1):
                self.dist_full += calc_gps_distance(self.points[i][0], self.points[i][1], self.points[i + 1][0], self.points[i + 1][1])

            if self.cur_state == CarState.MOVE_FRWD:
                search_index_start = 0
                search_index_finish = self.turn_point_index
            else:
                search_index_start = self.turn_point_index
                search_index_finish = len(self.points) - 1

            min_dist1 = min_dist2 = 10000000
            min_point_index = 0
            for i, e in enumerate(self.points[search_index_start:search_index_finish]):
                dist1 = calc_gps_distance(self.pos['klat'], self.pos['klon'], self.points[i + search_index_start][0], self.points[i + search_index_start][1])
                dist2 = calc_gps_distance(self.pos['klat'], self.pos['klon'], self.points[i + search_index_start + 1][0], self.points[i + search_index_start + 1][1])
                if (dist1 + dist2) < (min_dist1 + min_dist2):
                    min_dist1, min_dist2 = dist1, dist2
                    min_point_index = i + search_index_start

            # до текущего положения
            for i in range(min_point_index):
                self.dist_after_start += calc_gps_distance(self.points[i][0], self.points[i][1], self.points[i + 1][0], self.points[i + 1][1])
            self.dist_after_start += calc_gps_distance(self.points[min_point_index][0], self.points[min_point_index][1], self.pos['klat'], self.pos['klon'])

            # от текущего положения до точки разворота
            if (self.turn_point_index > min_point_index):
                self.dist_before_turn = calc_gps_distance(self.points[min_point_index + 1][0], self.points[min_point_index + 1][1], self.pos['klat'], self.pos['klon'])
                for i in range(min_point_index + 1, self.turn_point_index):
                    self.dist_before_turn += calc_gps_distance(self.points[i][0], self.points[i][1], self.points[i + 1][0], self.points[i + 1][1])

            # от точки разворота до текущей точки
            if (self.turn_point_index <= min_point_index):
                for i in range(self.turn_point_index, min_point_index):
                    self.dist_after_turn += calc_gps_distance(self.points[i][0], self.points[i][1], self.points[i + 1][0], self.points[i + 1][1])
                self.dist_after_turn += calc_gps_distance(self.points[min_point_index][0], self.points[min_point_index][1], self.pos["klat"], self.pos["klon"])

            # от текущей точки до финиша
            self.dist_before_stop = calc_gps_distance(self.pos["klat"], self.pos["klon"], self.points[min_point_index + 1][0], self.points[min_point_index + 1][1])
            for i in range(min_point_index + 1, len(self.points) - 1):
                self.dist_before_stop += calc_gps_distance(self.points[i][0], self.points[i][1], self.points[i + 1][0], self.points[i + 1][1])
            datastr = "state:{}; mpindx: {}; tpindx: {}; full: {}; after_start: {}; before_turn: {}; after_turn: {}; before_stop: {}".format(self.cur_state, min_point_index, self.turn_point_index, self.dist_full, self.dist_after_start, self.dist_before_turn, self.dist_after_turn, self.dist_before_stop)
            #log(self, datastr)
            self.distpub.publish(datastr)
            self.distpub.publish(datastr)
        except:
            rospy.logerr('Ошибка calc_distances: {}'.format(traceback.format_exc()))

    def publish_state(self, cmd):
        self.controlpub.publish("time:{};lat:{};lon:{};state:{};cmd:{}".format(time.ctime(time.time()), self.pos["klat"], self.pos["klon"], self.cur_state, cmd))

    def move_frwd_car(self):
        '''
        Движение до разворота
        '''
        try:
            log(self, 'Автомобиль начал движение к зоне разворота')
            prev_turn_dist = 1000000
            while not rospy.is_shutdown():
                self.calc_distances()
                #rospy.loginfo("prev_turn_dist {} dist_before_turn {}".format(prev_turn_dist, self.dist_before_turn));
                if ((prev_turn_dist < self.dist_before_turn) and (prev_turn_dist < 10)) or self.dist_before_turn < 1:
                    self.cur_state = CarState.TURN
                    break
                prev_turn_dist = self.dist_before_turn
                rospy.sleep(0.1)
            log(self, 'Автомобиль нашел зону разворота')
            self.cur_state = CarState.TURN
            self.publish_state("stop")
        except:
            rospy.logerr('Ошибка move_frwd: {}'.format(traceback.format_exc()))

    def calc_angle(self, p1, p2, p3):
        '''
        Определение переезда на основании угла между вершинами треугольника:
        p1 - реальное положение по GPS
        p2 - предадущая точка от целевой
        p3 - целевая точка
        '''
        a = calc_gps_distance(p1[0], p1[1], p2[0], p2[1])
        b = calc_gps_distance(p1[0], p1[1], p3[0], p3[1])
        c = calc_gps_distance(p3[0], p3[1], p2[0], p2[1])
        return math.acos((b**2 + c**2 - a**2) / (2 * b * c)) * 180 / math.pi

    def move_back_car(self):
        '''
        Движение после разворота до финиша
        '''
        try:
            log(self, 'Автомобиль начал движение от зоны разворота к финишу')
            self.publish_state("start")
            prev_stop_dist = 1000000
            prev_stop_diff = 1000000
            prev_alpha = 0
            while not rospy.is_shutdown():
                self.calc_distances()
                try:
                    alpha = self.calc_angle([self.pos["klat"], self.pos["klon"]], self.points[-2], self.points[-1])
                except:
                    reopy.loginfo("calc angle error")
                # условие требует проверки
                if ((prev_stop_dist < self.dist_before_stop or (prev_alpha < 90 and alpha >= 90)) and (prev_stop_dist < 20)) or (self.dist_before_stop < 2):
                    self.cur_state = CarState.STOP
                    break
                prev_stop_dist = self.dist_before_stop
                prev_alpha = alpha
                rospy.sleep(0.1)
            log(self, 'Автомобиль нашел зону финиша')
            self.cur_state = CarState.STOP
            self.publish_state("stop")
        except:
            rospy.logerr('Ошибка move_back: {}'.format(traceback.format_exc()))

    def turn_car(self):
        '''
        Выполнение разворота
        '''

        self.carstate.publish("turn")

        log(self, "Начало выполнения разворота")
        self.publish_state("turn")

    def stop_car(self):
        '''
        Остановка автомобиля
        '''
        self.carstate.publish("stop")
        
        log(self, 'Автомобиль выполнил остановку')
        self.cur_state = CarState.FINISHED

    def finish_way_car(self):
        '''
        Завершение выполнения задания
        '''
        self.publish_state("finish")
        logged = False
        while not rospy.is_shutdown():
            if not logged:
                log(self, "Автомобиль завершил выполнение задания")
                logged = True
            rospy.sleep(1)

    def control_state(self):
        turn_wait = True
        stop_wait = True
        while not rospy.is_shutdown():
            if self.cur_state == CarState.INIT:
                self.init_car()
            elif self.cur_state == CarState.WAIT:
                self.wait_start_car()
            elif self.cur_state == CarState.MOVE_FRWD:
                self.move_frwd_car()
            elif self.cur_state == CarState.MOVE_BACK:
                self.move_back_car()
            elif self.cur_state == CarState.TURN:
                if turn_wait:
                    self.turn_car()
                    turn_wait = False
            elif self.cur_state == CarState.STOP:
                if stop_wait:
                    self.stop_car()
                    stop_wait = False
            elif self.cur_state == CarState.FINISHED:
                self.finish_way_car()
            else:
                break
            rospy.sleep(0.1)

    def load_data(self):
        with open(rospy.get_param('~osm.gpspath'), 'r') as f:
            k = 0
            for rec in csv.reader(f, delimiter=','):
                self.points.append([rec[0], rec[1], rec[2]])
                if rec[2] == "start":
                    self.point_start['lat'] = float(rec[0])
                    self.point_start['lon'] = float(rec[1])
                elif rec[2] == "return":
                    self.point_turn['lat'] = float(rec[0])
                    self.point_turn['lon'] = float(rec[1])
                    self.turn_point_index = k
                elif rec[2] == "finish":
                    self.point_stop['lat'] = float(rec[0])
                    self.point_stop['lon'] = float(rec[1])
                k += 1
            if self.turn_point_index == 0:
                self.turn_point_index = len(self.points) - 1

    def cmpl_cmd_callback(self, data):
        log(self, data)
        if str(data).index("finishturn") >= 0:
            rospy.sleep(2.5)
            log(self, "Конец выполнения разворота")
            self.cur_state = CarState.MOVE_BACK
            self.publish_state("stop")

    def __init__(self):
        rospy.Subscriber("gpspos", String, self.getpos_callback)
        self.distpub = rospy.Publisher('distances', String, queue_size=10)
        self.controlpub = rospy.Publisher('carcmd', String, queue_size=10)
        self.turnpub = rospy.Publisher('serialcode', String, queue_size=10)
        self.carstate = rospy.Publisher('carstate', String, queue_size=10)
        rospy.Subscriber("completetask", String, self.cmpl_cmd_callback)
        self.load_data()
        for i in range(3):
            rospy.sleep(1)
            self.calc_distances()

if __name__ == '__main__':
    try:
        rospy.init_node('carstatecontrol', anonymous=True)
        ctrl = CarPositionController()
        ctrl.control_state()
    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка move_frwd: {}'.format(traceback.format_exc()))
