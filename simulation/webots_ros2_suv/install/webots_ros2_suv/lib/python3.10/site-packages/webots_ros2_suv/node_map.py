import cherrypy
import json
from PIL import Image
from io import BytesIO
import os
import csv
import rclpy
import traceback
from rclpy.node import Node

BASE_PATH = '/home/hiber/ros2_ws/install/webots_ros2_suv/share/webots_ros2_suv/static/map-server/dist/'
STATIC_PATH = BASE_PATH
ASSETS_PATH = BASE_PATH + 'assets/'
class MapServer(Node):
    def __init__(self):
        try:
            super().__init__('node_web')
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    @cherrypy.expose
    def index(self):
        raise cherrypy.HTTPRedirect("/static/index.html")
def main(args=None):
    try:
        rclpy.init(args=args)
        cherrypy.quickstart(MapServer(), '/', {'global':
                                                   {'server.socket_host': '127.0.0.1',
                                                    'server.socket_port': 8008,
                                                    'tools.staticdir.root': STATIC_PATH,
                                                    'log.error_file': 'site.log'
                                                    },
                                               '/static': {
                                                   'tools.staticdir.on': True,
                                                   'tools.staticdir.dir': STATIC_PATH,
                                                   'tools.staticdir.index': 'index.html'
                                               },
                                               '/assets': {
                                                   'tools.staticdir.on': True,
                                                   'tools.staticdir.dir': ASSETS_PATH,
                                                   'tools.staticdir.index': 'index.html'
                                               }
                                               })
        rclpy.spin(detector)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except  Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()