#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cherrypy
import json
from PIL import Image
from io import BytesIO
import os
import csv
import rospy
from utils import *

class MapServer(object):
    pass

if __name__ == '__main__':
    cherrypy.quickstart(MapServer(), '/', {'global':
                                               {'server.socket_host': '127.0.0.1',
                                                'server.socket_port': 8002,
                                                'tools.staticdir.root': '/home/hiber/ros2_ws/src/webots_ros2_suv/static',
                                                'log.error_file': 'site.log'
                                                },
                                           '/static': {
                                               'tools.staticdir.on': True,
                                               'tools.staticdir.dir': 'static',
                                               'tools.staticdir.index': 'index.html'
                                           }})