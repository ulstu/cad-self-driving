import matplotlib as plt
import numpy as np
import serial
import math
import time
import csv
import sys
import struct


def bytes_to_int(offset, paket):
    r = 0
    for i in range(4):
        d = i * 8
        r += int.from_bytes(paket[offset + i], 'little') << d
    return r


dict = { 'gps_state_status' : 0,
         'gps_int_longitude' : 8,
         'gps_int_latitude' : 4,
         'gps_altitude' : 12,
         'gps_velocity' : 16,
         'gps_yaw' : 20
         }


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

    @property
    def altitude(self):
        return self.__altitude

    @property
    def yaw(self):
        return self.__yaw


    def to_decimal(self, lat, lon):
        lat_dd = int(float(lat) / 100)
        lat_mm = float(lat) - lat_dd * 100
        lat_dec = lat_dd + lat_mm / 60
        lon_dd = int(float(lon) / 100)
        lon_mm = float(lon) - lon_dd * 100
        lon_dec = lon_dd + lon_mm / 60
        return lat_dec, lon_dec

    """def read_all_data(self):
                    try:
                        while True:
                            self.read_data()
                            rospy.loginfo(
                                "time: {}; is_confidential: {}; lat: {}; lat_g:{}; lon: {}; lon_g:{}; velocity: {}; direction: {}; date: {}".format(
                                    self.__time, self.__is_confidential, self.__lat, self.__lat_g, self.__lon, self.__lon_g, self.__velocity, self.__direction, self.__date))
                            rospy.loginfo("lat:{}; lon:{}".format(self.__lat_dec, self.__lon_dec))
                    except KeyboardInterrupt:
                        pass"""

    def save_vals(self, count, filename):
        try:
            with open(filename, 'w') as gpscsv:
                wr = csv.writer(gpscsv, quoting=csv.QUOTE_ALL)
                i = 0
                while i < count:
                    prev_lat = float(self.__lat_dec)
                    self.read_data()
                    if (abs(prev_lat - self.__lat_dec) > 10**(-7)):
                        print("i: {}; lat:{}; lon:{}; velocity: {}".format(i, self.__lat_dec, self.__lon_dec,
                                                                           self.__velocity))
                        wr.writerow([self.__lat_dec, self.__lon_dec])
                        i += 1
        except KeyboardInterrupt:
            pass

    def disconnect(self):
        self.__ser.close()  # close port

    def read_data(self):
        while not self.__ser.is_open:
            print("GPS device not found")
            time.sleep(2)
        
        self.__paket = [b'\x00']
        self.__paket[0] = self.__ser.read()

        while self.__paket[0] != b'\xff':
            self.__paket[0] = self.__ser.read()

        for byte in range(3):
            self.__paket.append(self.__ser.read())

        for byte in range(int.from_bytes(self.__paket[3], byteorder=sys.byteorder) + 4):
            self.__paket.append(self.__ser.read())

        self.__status = bool(bytes_to_int(dict['gps_state_status'] + 4, self.__paket) & (2 ** 16)) - 1

        if self.__status == -1:
            print('GPS NMEA data received but not confidential')
        else:

            self.__lon_dec = bytes_to_int(dict['gps_int_longitude'] + 4, self.__paket) * 360 / 4294967296
            self.__lat_dec = bytes_to_int(dict['gps_int_latitude'] + 4, self.__paket) * 360 / 4294967296
            self.__altitude = self.__paket[dict['gps_altitude'] + 4] + self.__paket[dict['gps_altitude'] + 5] + self.__paket[dict['gps_altitude'] + 6] + self.__paket[dict['gps_altitude'] + 7]
            self.__altitude = struct.unpack('f', self.__altitude)[0]
            self.__velocity = self.__paket[dict['gps_velocity'] + 4] + self.__paket[dict['gps_velocity'] + 5] + self.__paket[dict['gps_velocity'] + 6] + self.__paket[dict['gps_velocity'] + 7]
            self.__velocity = struct.unpack('f', self.__velocity)[0]
            self.__yaw = self.__paket[dict['gps_yaw'] + 4] + self.__paket[dict['gps_yaw'] + 5] + self.__paket[dict['gps_yaw'] + 6] + self.__paket[dict['gps_yaw'] + 7]
            self.__yaw = struct.unpack('f', self.__yaw)[0]
            # print(self.__status, self.__lat_dec, self.__lon_dec, self.__altitude, self.__velocity, self.__yaw)


    def __init__(self, port, baudrate):
        self.__ser = serial.Serial(port, baudrate)
        print(self.__ser.name)  # check which port was really used
        self.__init_lat_dec, self.__init_lon_dec = 0, 0
        self.__lat_dec, self.__lon_dec = 0, 0
        self.__velocity = 0
        self.__yaw = 0
        self.__paket = [b'\x00']
        self.read_data()
        self.read_data()