#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import matplotlib as plt
import numpy as np
import serial
import math
import time
import csv
import rospy
from utils import *

class GPSReader:
    @property
    def lat_dec(self):
        return self.__lat_dec

    @property
    def lon_dec(self):
        return self.__lon_dec

    @property
    def velocity(self):
        return self.__velocity

    def to_decimal(self, lat, lon):
        lat_dd = int(float(lat) / 100)
        lat_mm = float(lat) - lat_dd * 100
        lat_dec = lat_dd + lat_mm / 60
        lon_dd = int(float(lon) / 100)
        lon_mm = float(lon) - lon_dd * 100
        lon_dec = lon_dd + lon_mm / 60
        return lat_dec, lon_dec

    def read_all_data(self):
        try:
            while True:
                self.read_data()
                rospy.loginfo(
                    "time: {}; is_confidential: {}; lat: {}; lat_g:{}; lon: {}; lon_g:{}; velocity: {}; direction: {}; date: {}".format(
                        self.__time, self.__is_confidential, self.__lat, self.__lat_g, self.__lon, self.__lon_g, self.__velocity, self.__direction, self.__date))
                rospy.loginfo("lat:{}; lon:{}".format(self.__lat_dec, self.__lon_dec))
        except KeyboardInterrupt:
            pass

    def save_vals(self, count, filename):
        try:
            with open(filename, 'w') as gpscsv:
                wr = csv.writer(gpscsv, quoting=csv.QUOTE_ALL)
                i = 0
                while i < count:
                    prev_lat = float(self.__lat_dec)
                    self.read_data()
                    if (abs(prev_lat - self.__lat_dec) > 10**(-7)):
                        rospy.loginfo("i: {}; lat:{}; lon:{}; velocity: {}".format(i, self.__lat_dec, self.__lon_dec,
                                                                           self.__velocity))
                        wr.writerow([self.__lat_dec, self.__lon_dec])
                        i += 1
        except KeyboardInterrupt:
            pass

    def disconnect(self):
        self.__ser.close()  # close port

    def read_data(self):
        while not self.__ser.is_open:
            rospy.loginfo("GPS device not found")
            time.sleep(2)
        
        line = str(self.__ser.readline().decode())
        while not (line.startswith("$GNRMC") or line.startswith("$GPRMC")):
            line = str(self.__ser.readline().decode())

        if line.startswith("$GNRMC") or line.startswith("$GPRMC"):
            messages = line.split(',')
            self.__time = messages[1]
            self.__is_confidential = messages[2]
            if self.__is_confidential == 'A':
                self.__lat = messages[3]
                self.__lat_g = messages[4]
                self.__lon = messages[5]
                self.__lon_g = messages[6]
                self.__velocity = messages[7]
                self.__direction = messages[8]
                self.__date = messages[9]
                self.__lat_dec, self.__lon_dec = self.to_decimal(self.__lat, self.__lon)
                if self.__init_lat_dec == 0 or self.__init_lon_dec == 0:
                    self.__init_lat_dec, self.__init_lon_dec = self.__lat_dec, self.__lon_dec
            else:
                rospy.loginfo('GPS NMEA data received but not confidential')



    def __init__(self, port, baudrate):
        self.__ser = serial.Serial(port, baudrate)
        rospy.loginfo(self.__ser.name)  # check which port was really used
        self.__init_lat_dec, self.__init_lon_dec = 0, 0
        self.__lat_dec, self.__lon_dec = 0, 0
        self.__is_confidential = "V"
        self.read_data()
        while (self.__is_confidential == "V"):
            self.read_data()
            time.sleep(1)

