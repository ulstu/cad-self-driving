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
from std_msgs.msg import String

class MapServer(object):
    pos = {'klat': 0, 'klon': 0, 'velocity': 0, 'lat': 0, 'lon': 0}
    dist = {'mpindx':0, 'tpindx':0, 'full':0, 'after_start':0, 'before_turn':0, 'after_turn':0, 'before_stop':0}
    state = "no active"
    carstate = {}
    cmd = "stop"

    def __init__(self):
        rospy.init_node('webdisplaynode', anonymous=True)
        rospy.Subscriber("gpspos", String, self.getpos_callback)
        rospy.Subscriber("distances", String, self.getdist_callback)
        rospy.Subscriber("carcmd", String, self.getstatus_callback)
        rospy.Subscriber("carstate", String, self.getcarstate_callback)

    def getpos_callback(self, data):
        data = str(data).replace("data:", "")
        vars = str(data).split(';')
        for v in vars:
            keyval = v.split(':')
            self.pos[clear_str(keyval[0])] = clear_str(keyval[1])
        if rospy.get_param("~usekalman"):
            self.pos["klat"] = self.pos["lat"]
            self.pos["klon"] = self.pos["lon"]

    def getdist_callback(self, data):
        data = str(data).replace("data:", "")
        vars = str(data).split(';')
        for v in vars:
            keyval = v.split(':')
            self.dist[clear_str(keyval[0])] = clear_str(keyval[1])

    def getstatus_callback(self, data):
        data = str(data).replace("data:", "")
        vars = str(data).split(';')
        for v in vars:
            keyval = v.split(':')
            if clear_str(keyval[0]) == 'state':
                self.state = clear_str(keyval[1])
            if clear_str(keyval[0]) == 'cmd':
                self.cmd = clear_str(keyval[1])

    def getcarstate_callback(self, data):
        try:
            data = str(data).replace("data:", "")
            vars = str(data).split(';')
            for v in vars:
                keyval = v.split(':')
                self.carstate[clear_str(keyval[0])] = clear_str(keyval[1])
        except:
            print('web error')

    @cherrypy.expose
    def index(self):
        raise cherrypy.HTTPRedirect("/static/local_tiles.htm")

    @cherrypy.expose
    def gettile(self, zoom, x, y):
        cherrypy.response.headers['Content-Type'] = "image/png"
        filename = '{}/{}/{}/{}.png'.format(rospy.get_param('~osm.tiles.dir'), zoom, x, y)
        if not os.path.isfile(filename):
            filename = '{}/empty.png'.format(rospy.get_param('~osm.tiles.dir'))

        img = Image.open(filename)
        stream = BytesIO()
        img.save(stream, 'PNG')
        return stream.getvalue()

    @cherrypy.expose
    def getpath(self):
        with open(rospy.get_param('~osm.gpspath'), 'r') as f:
            return json.dumps({"points":list({"lat": float(rec[0]), "lon": float(rec[1]), "ptype": rec[2]} for rec in csv.reader(f, delimiter=','))})

    @cherrypy.expose
    @cherrypy.tools.json_out()
    @cherrypy.tools.json_in()
    def savepath(self):
        try:
            print(cherrypy.request.json)
            with open(rospy.get_param('~osm.gpspath'), 'w') as gpscsv:
                wr = csv.writer(gpscsv, quoting=csv.QUOTE_ALL)
                for r in cherrypy.request.json:
                    wr.writerow([r[0], r[1], 'point'])
        except:
            pass
        return json.dumps(cherrypy.request.json)

    @cherrypy.expose
    def setpoint(self, lat, lon, ptype):
        try:
            points = []
            with open(rospy.get_param('~osm.gpspath'), 'r') as f:
                for rec in csv.reader(f, delimiter=','):
                    points.append([rec[0], rec[1], rec[2]])

            with open(rospy.get_param('~osm.gpspath'), 'w') as gpscsv:
                wr = csv.writer(gpscsv, quoting=csv.QUOTE_ALL)
                for rec in points:
                    if (lat[0:11] == rec[0][0:11] and lon[0:11] == rec[1][0:11]):
                        log(self, "request for point: {} {} {}".format(lat, lon, ptype))
                        rec[2] = ptype
                    wr.writerow([rec[0], rec[1], rec[2]])
        except:
            return "error"
        return "done"


    @cherrypy.expose
    def getpos(self):
        return json.dumps({"lat": self.pos['klat'], "lon": self.pos['klon'], "velocity": self.pos['velocity'] })
    
    @cherrypy.expose
    def getdist(self):
        return json.dumps(
            {"state": self.state, 
            "cmd": self.cmd,
            "dist": 
                {
                    "before_turn": round(float(self.dist['before_turn']), 2), 
                    "after_start": round(float(self.dist['after_start']), 2),
                    'before_stop': round(float(self.dist['before_stop']), 2), 
                    'after_turn': round(float(self.dist['after_turn']), 2)
                }
            }
        )

    @cherrypy.expose
    def getcarstate(self):
        return json.dumps(self.carstate)

if __name__ == '__main__':
    cherrypy.quickstart(MapServer(), '/', {'global':
                            {'server.socket_host': rospy.get_param('~server.socket_host'),
                            'server.socket_port': rospy.get_param('~server.socket_port'),
                            'tools.staticdir.root': rospy.get_param('~tools.staticdir.root'),
                            'log.error_file': 'site.log'
                            },
                            '/static':{
                                'tools.staticdir.on': True,
                                'tools.staticdir.dir': 'static',
                                'tools.staticdir.index': rospy.get_param('~tools.staticdir.index')
                            }})
