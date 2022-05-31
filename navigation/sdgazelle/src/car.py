#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import traceback
from utils import *
import obd
import csv
import serial

def get_code(code, value):
    left_piece = ((code + 32) << 2) + ((value >> 14) & 3)
    midle_piece = (value >> 7) & 127
    right_piece = value & 127
    #print(str(left_piece)+ " " + str(midle_piece) + " " + str(right_piece))
    return bytearray([left_piece, midle_piece, right_piece])

class Car(object):
    hist_cmd = []
    cur_cmd = {}
    statepub = None
    obd_connection = None
    actual_params = {}
    check_params = {
        CarCmdParams.rpm: [800, 3000],
        CarCmdParams.throttle: [-1, 100], # -1 - сигнал к тому, что ничего не нужно изменять
        CarCmdParams.transmission: [-1, 1],
        CarCmdParams.velocity: [0, 15],
        CarCmdParams.wheel: [-720, 720],
    }
    is_turning = False
    

    def send_data(self, data):
        
        while Serial.in_waiting:
            self.starter.publish("ready")
            Serial.reset_input_buffer()
        
        if int(data.data.split()[0]) == 5:
            self.is_turning = True
        if self.is_turning and int(data.data.split()[0]) != 16:
            Serial.write(get_code(int(data.data.split()[0]), int(float(data.data.split()[1]))))
        elif int(data.data.split()[0]) != 6:
            Serial.write(get_code(int(data.data.split()[0]), int(float(data.data.split()[1]))))

    def read_obd(self):
        '''
        Чтение данных о состонии автомобиля из OBD приемника
        Необходимо разработать систему проверку безопасности - что делать, если параметры выходят из заданных пределов
        '''
        if rospy.get_param("~simulation"):
            return 1111, 7, 10
        try:
            if rospy.get_param("~loadobd"):
                speed = obd.commands.SPEED  # select an OBD command (sensor)
                rspeed = round(self.obd_connection.query(speed).value.magnitude, 2)  # send the command, and parse the response
                rpm = obd.commands.RPM  
                rrpm = round(self.obd_connection.query(rpm).value.magnitude, 2)
                throttle = obd.commands.THROTTLE_POS 
                rthrottle = round(self.obd_connection.query(throttle).value.magnitude, 2)
                str = "velocity: {}; rpm: {}; throttlepos: {}".format(rspeed, rrpm, rthrottle)
                log(self, str)  
                # 1 - rpm, 2 - velocity, 3 - throttle
                return rrpm, rspeed, rthrottle
            else:
                return 0, 0, 0
        except:
            rospy.logerr("Не удалось получить данные о состоянии автомобиля:".format(traceback.format_exc()))
            return 0, 0, 0

    def read_wheel(self):
        '''
        НЕ ГОТОВО. Метод для чтения положения руля. Возвращаемое значение -720 градусов до 720 градусов
        '''
        return 0

    def publish_state(self):
        '''
        Получение параметров автомобиля и публикация этого состояния
        '''
        # получение данных об оборотах двигателя (из библиотеки obd II) и скорости из obd
        self.actual_params[CarCmdParams.rpm], self.actual_params[CarCmdParams.velocity], self.actual_params[CarCmdParams.throttle] = self.read_obd()
        # плучение данных об оборотах руля (код с opencv)
        self.actual_params[CarCmdParams.wheel] = self.read_wheel()
        # получение данных о текущей передачи КПП
        if CarCmdParams.transmission in self.cur_cmd:
            self.actual_params[CarCmdParams.transmission] = self.cur_cmd[CarCmdParams.transmission] 
        msg_template = "rpm:{};transmission:{};throttle:{};velocity:{};wheel:{}"
        self.statepub.publish(msg_template.format(
            self.actual_params[CarCmdParams.rpm], 
            self.actual_params[CarCmdParams.transmission], 
            self.actual_params[CarCmdParams.throttle], 
            self.actual_params[CarCmdParams.velocity], 
            self.actual_params[CarCmdParams.wheel]))

    def check_cmd(self):
        '''
        Ограничение неправильно переданных значений
        '''
        for key, val in self.check_params.items():
            if key in self.cur_cmd:
                if self.cur_cmd[key] < val[0]:
                    self.cur_cmd[key] = val[0]
                if self.cur_cmd[key] > val[1]:
                    self.cur_cmd[key] = val[1]

    def control(self):
        '''
        НЕ ГОТОВО! Формирование управляющих воздействий для автомобиля. 
        Работа с железом ТУТ!!!
        '''
        try:
            # передача управляющего воздействия на руль
            '''if (CarCmdParams.wheel in self.actual_params) and (CarCmdParams.wheel in self.cur_cmd):
                if (self.cur_cmd[CarCmdParams.wheel] != 0):
                    while self.actual_params[CarCmdParams.wheel] < self.cur_cmd[CarCmdParams.wheel] and not rospy.get_param("~simulation"):
                        if self.actual_params[CarCmdParams.wheel] > self.check_params[CarCmdParams.wheel][1]:
                            log(self, "Дальше вращать руль вправо нельзя")
                            break
                        # вращение руля вправо
                        pass
                    while (self.actual_params[CarCmdParams.wheel] > self.cur_cmd[CarCmdParams.wheel]) and not rospy.get_param("~simulation"):
                        if self.actual_params[CarCmdParams.wheel] < self.check_params[CarCmdParams.wheel][0]:
                            log(self, "Дальше вращать руль влево нельзя")
                            break
                        # вращение руля влево
                        pass'''

            #log(self, self.cur_cmd[CarCmdParams.transmission] if CarCmdParams.transmission in self.cur_cmd else -100)
            if (CarCmdParams.transmission in self.cur_cmd):
                self.cur_cmd[CarCmdParams.transmission] = int(self.cur_cmd[CarCmdParams.transmission])
                self.actual_params[CarCmdParams.transmission] = int(self.actual_params[CarCmdParams.transmission])
                if self.cur_cmd[CarCmdParams.transmission] != self.actual_params[CarCmdParams.transmission]:
                    log(self, "Переключение передачи с {} на {}".format(self.actual_params[CarCmdParams.transmission], self.cur_cmd[CarCmdParams.transmission]))
                    self.actual_params[CarCmdParams.transmission] = self.cur_cmd[CarCmdParams.transmission]
                    # смена передачи

            # установка скорости 
            #if (CarCmdParams.velocity in self.cur_cmd) and self.cur_cmd[CarCmdParams.velocity] != 0:
                #while (self.actual_params[CarCmdParams.velocity] < self.cur_cmd[CarCmdParams.velocity]) and not rospy.get_param("~simulation"):
                #    if (self.actual_params[CarCmdParams.transmission] != 0):
                #        if self.actual_params[CarCmdParams.rpm] > self.check_params[CarCmdParams.rpm][1]: 
                #            break
                        # увеличить значение на педали газа на определенный шаг
                        #log(self, "Увеличение скорости. Текущая: {}; Целевая: {}".format(self.actual_params[CarCmdParams.velocity], self.cur_cmd[CarCmdParams.velocity]))
                #       rospy.sleep(0.2)
                #    else:
                #        rospy.logerr("Попытка увеличить скорость {} без включенной передачи".format(self.cur_cmd[CarCmdParams.velocity]))
                #        break
                #while (self.actual_params[CarCmdParams.velocity] > self.cur_cmd[CarCmdParams.velocity]) and not rospy.get_param("~simulation"):
                #    if (self.actual_params[CarCmdParams.transmission] != 0):
                #        if self.actual_params[CarCmdParams.rpm] < self.check_params[CarCmdParams.rpm][0]: 
                #            break
                        # уменьшить значение на педали газа
                #        log(self, "Уменьшение скорости. Текущая: {}; Целевая: {}".format(self.actual_params[CarCmdParams.velocity], self.cur_cmd[CarCmdParams.velocity]))
                #        rospy.sleep(0.2)
                #    else:
                #        rospy.logerr("Попытка уменьшить скорость {} без включенной передачи".format(self.cur_cmd[CarCmdParams.velocity]))
                #        break
        except:
            rospy.logerr('Ошибка car control: {}'.format(traceback.format_exc()))


    def init_car(self):
        '''
        Инициализация начальных значений управления автомобилем
        '''
        self.cur_cmd[CarCmdParams.velocity] = 0
        self.cur_cmd[CarCmdParams.wheel] = 0
        self.cur_cmd[CarCmdParams.throttle] = 0
        self.cur_cmd[CarCmdParams.transmission] = 0

    
    def stop_car(self):
        self.cur_cmd[CarCmdParams.velocity] = 0
        self.cur_cmd[CarCmdParams.transmission] = 0
        # нажать на тормоз

    def finish_car(self):
        self.cur_cmd[CarCmdParams.velocity] = 0
        self.cur_cmd[CarCmdParams.wheel] = 0
        self.cur_cmd[CarCmdParams.throttle] = 0
        self.cur_cmd[CarCmdParams.transmission] = 0

    def cmd_callback(self, data):
        '''
        Callback для получения сигнала управления автомобилем (abstract commands )
        '''
        data = str(data).replace("data:", "")
        vars = str(data).split(';')
        cmd = {}
        for v in vars:
            keyval = v.split(':')
            cmd[clear_str(keyval[0])] = clear_str(keyval[1])
        self.cur_cmd = {}
        if "cmd" in cmd:
            if cmd["cmd"] == "init":
                self.init_car()
            elif cmd["cmd"] == "stop":
                self.stop_car()
            elif cmd["cmd"] == "finish":
                self.finish_car()
        
        self.check_cmd()
        log(self, "car command received: {}; vals: {}".format(data, self.cur_cmd))
        self.hist_cmd.append([self.cur_cmd, self.actual_params])
        self.control()

    def control_callback(self, data):
        '''
        Callback для получения сигнала управления автомобилем (car messages: transmission, velocity, throttle etc)
        '''
        data = str(data).replace("data:", "")
        vars = str(data).split(';')
        self.cur_cmd = {}
        for v in vars:
            keyval = v.split(':')
            key = clear_str(keyval[0])
            for name, value in CarCmdParams.__members__.items():
                if key in name:
                    self.cur_cmd[value] = float(clear_str(keyval[1]))
        self.check_cmd()
        self.hist_cmd.append([self.cur_cmd, self.actual_params])
        tr = self.cur_cmd[CarCmdParams.transmission] if CarCmdParams.transmission in self.cur_cmd else -100
        log(self, "car control message recieved: {}; tr: {}; vals: {}; actual: {}".format(data, tr, self.cur_cmd, self.actual_params))
        self.control()

    def start(self):
        while not rospy.is_shutdown():
            self.publish_state()
            rospy.sleep(0.5)
        with open(rospy.get_param('~histfilename'), 'w') as paramcsv:
            wr = csv.writer(paramcsv, quoting=csv.QUOTE_ALL)
            wr.writerows(self.hist_cmd)

    def __init__(self):
        if (not rospy.get_param("~simulation")) and rospy.get_param("~loadobd"):
            log(self, "connecting to OBD: {}".format(rospy.get_param("~obdport")))
            obd.logger.setLevel(obd.logging.DEBUG)
            self.obd_connection = obd.OBD(rospy.get_param("~obdport"))  # auto-connects to USB or RF port
        rospy.Subscriber("carcmd", String, self.cmd_callback)
        rospy.Subscriber("carcontrol", String, self.control_callback)
        rospy.Subscriber("serialcode", String, self.send_data)
        self.statepub = rospy.Publisher('carstate', String, queue_size=10)
        self.starter = rospy.Publisher('ready', String, queue_size=10)
        self.init_car()
        self.start()

if __name__ == '__main__':
    try:
        Serial = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
        rospy.init_node('car', anonymous=True)
        ctrl = Car()
        ctrl.control()
    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка car node: {}'.format(traceback.format_exc()))
