#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from pykalman import KalmanFilter
import csv
import time
import rospy
import numpy as np
from std_msgs.msg import String
from gpsreader import GPSReader
from utils import *


serial_port = ""
serial_baudrate = 0
gps_init_filename = ""
need_display = True


def save_init_data(count):
    gps = GPSReader(serial_port, serial_baudrate)
    gps.save_vals(count, gps_init_filename)
    gps.disconnect()

def load_gps_data():
    rospy.loginfo("starting GPS loading data")
    with open(gps_init_filename, 'r') as f:
        return list([float(rec[0]), float(rec[1])] for rec in csv.reader(f, delimiter=','))

def live_plotter_xy(x_vec, y1_data, y2_data, line1, line2, identifier='', pause_time=0.01):
    if line1 == [] or line2 == []:
        plt.ion()
        fig = plt.figure(figsize=(13, 6))
        ax = fig.add_subplot(111)
        line1, = ax.plot(x_vec, y1_data, 'r-o', alpha=0.8)
        line2, = ax.plot(x_vec, y2_data, 'b-o', alpha=0.8)
        plt.ylabel('Coordinates')
        plt.title(identifier)
        plt.show()

    line1.set_data(x_vec, y1_data)
    line2.set_data(x_vec, y2_data)
    plt.xlim(np.min(x_vec), np.max(x_vec))
    if np.min(y1_data) <= line1.axes.get_ylim()[0] or np.max(y1_data) >= line1.axes.get_ylim()[1]:
        plt.ylim([np.min(y1_data) - np.std(y1_data), np.max(y1_data) + np.std(y1_data)])
    plt.pause(pause_time)
    return line1, line2

def filter(measurements):
    init_measurements = measurements[:]
    pub = rospy.Publisher('gpspos', String, queue_size=10)
    rospy.loginfo("starting GPS filtering")

    initial_state_mean = [measurements[0][0],
                          0,
                          measurements[0][1],
                          0]

    transition_matrix = [[1, 1, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 1],
                         [0, 0, 0, 1]]

    observation_matrix = [[1, 0, 0, 0],
                          [0, 0, 1, 0]]

    times = list(range(len(measurements)))
    kf3 = KalmanFilter(transition_matrices=transition_matrix,
                       observation_matrices=observation_matrix,
                       initial_state_mean=initial_state_mean,
                       em_vars=['transition_covariance', 'initial_state_covariance'])

    kf3 = kf3.em(measurements, n_iter=5)
    (filtered_state_means, filtered_state_covariances) = kf3.filter(measurements)
    x_now = filtered_state_means[-1, :]
    P_now = filtered_state_covariances[-1, :]
    x_new = []

    gps = GPSReader(serial_port, serial_baudrate)
    rospy.loginfo("port: {}; baudrate: {}".format(serial_port, serial_baudrate))
    line1, line2 = [], []
    line3, line4 = [], []
    while not rospy.is_shutdown():
        gps.read_data()
        (x_now, P_now) = kf3.filter_update(filtered_state_mean=x_now,
                                           filtered_state_covariance=P_now,
                                           observation=[float(gps.lat_dec), float(gps.lon_dec)]
                                           #,observation_covariance = 10 * kf3.observation_covariance
                                           )
        # check not
        if not bool(rospy.get_param("~usekalman")):
            pub.publish("lat: {}; lon: {}; klat: {}; klon: {}; velocity: {}".format(gps.lat_dec, gps.lon_dec, gps.lat_dec, gps.lon_dec, gps.velocity))
        else:
            pub.publish("lat: {}; lon: {}; klat: {}; klon: {}; velocity: {}".format(gps.lat_dec, gps.lon_dec, x_now[0], x_now[2], gps.velocity))
            
        x_new.append(x_now)
        measurements.append([float(gps.lat_dec), float(gps.lon_dec)])
        times.append(times[-1] + 1)
        if need_display:        
            line1, line2 = live_plotter_xy(times, [a[0] for a in measurements], [a[0] for a in init_measurements] + [a[0] for a in x_new], line1, line2, "Latitude")
            #line3, line4 = live_plotter_xy(2, times, [a[1] for a in measurements], [a[1] for a in init_measurements] + [a[2] for a in x_new], line3, line4, "Longitude")
    gps.disconnect()

if __name__ == '__main__':
    try:
        rospy.init_node('gpsnode', anonymous=True)
        serial_port = rospy.get_param('~gpsport')
        serial_baudrate = int(rospy.get_param('~baudrate'))
        gps_init_filename = rospy.get_param('~gpsvals')
        need_display = bool(rospy.get_param('~need_display'))

        #rospy.loginfo("GPS kalman filtering initialization started")
        #save_init_data(5)
        #rospy.loginfo("GPS kalman filtering initialization finished")
        #rospy.loginfo("GPS kalman filtering started")
        gps_pos_publisher = rospy.Publisher('gpspos', String, queue_size=10)

        # uncomment for real GPS data
        if (not bool(rospy.get_param("~simulation"))):
            rospy.loginfo("starting real GPS data")
            filter(load_gps_data())
        else:
            rospy.loginfo("starting GPS simulation")
            points = []
            with open(rospy.get_param('~osm.gpssimulation'), 'r') as f:
                for rec in csv.reader(f, delimiter=','):
                    points.append([rec[0], rec[1], rec[2]])
            
            i = 0
            while not rospy.is_shutdown() and i < len(points):
                local_num = 0
                for k in range(3):
                    lat = points[i][0]
                    lon = points[i][1]
                    velocity = 0.006
                    rospy.loginfo("lat: {}; lon: {}; klat: {}; klon: {}; velocity: {}".format(lat, lon, lat, lon, velocity))
                    gps_pos_publisher.publish("lat: {}; lon: {}; klat: {}; klon: {}; velocity: {}".format(lat, lon, lat, lon, velocity))
                    rospy.sleep(float(rospy.get_param("~simulationdelay")))
                i += 1

    except rospy.ROSInterruptException:
        pass
