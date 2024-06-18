import rclpy
import cherrypy
import traceback
import yaml
import os
import logging
import pathlib
import json
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from .orientation import euler_from_quaternion
from .coords_transformer import CoordsTransformer
from rclpy.node import Node
from os import listdir
from os.path import isfile, join
from robot_interfaces.srv import PoseService
from robot_interfaces.msg import EgoPose
import time
import numpy as np
import cv2
from PIL import Image

BASE_RESOURCE_PATH = get_package_share_directory('webots_ros2_suv') + '/'
# для отладки в режиме редактирования fronend части прописать абсолютный путь, например:

HOME_DIR = os.path.expanduser('~')

BASE_PATH = os.path.join(HOME_DIR, 'ros2_ws/src/webots_ros2_suv/')
#BASE_PATH = BASE_RESOURCE_PATH
STATIC_PATH = BASE_PATH + 'map-server/dist/'
YAML_PATH = BASE_PATH + 'config/ego_states/robocross.yaml'
MAPS_PATH = BASE_PATH + 'config/global_maps/'
ASSETS_PATH = STATIC_PATH + 'assets/'

class MapWebServer(object):
    def __init__(self, log=None):
        try:
            if not log:
                log = print
            self.log = log
            self.world_model = None
            self.init_driving_path()
        except  Exception as err:
            self.log(''.join(traceback.TracebackException.from_exception(err).format()))
    
    def update_model(self, world_model):
        self.world_model = world_model

    def init_driving_path(self):
        self.driving_points = []
        self.driving_points_path = f"{os.path.expanduser('~')}/ros2_ws/data/paths/path_{time.strftime('%Y%m%d-%H%M%S')}.json"


    @cherrypy.expose
    def index(self):
        raise cherrypy.HTTPRedirect("static/index.html")

    @cherrypy.expose
    def save_segment(self):
        self.init_driving_path()

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_driving_points(self):
        try:
            return {'status' : 'ok', 'path': self.driving_points}
        except Exception as e:
            return {'status' : 'error', 'message': ''.join(traceback.TracebackException.from_exception(e).format())}


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_position(self):
        try:
            pos = self.world_model.get_current_position()
            self.driving_points.append([pos[0], pos[1]])
        except Exception as e:
            return {'status' : 'error', 'message': ''.join(traceback.TracebackException.from_exception(e).format())}
        try:
            with open(self.driving_points_path, 'a') as f:
                f.write(f"[{pos[0]},{pos[1]}],\n")

        except Exception as e:
            pass

        if pos:
            return {'status' : 'ok', 'lat': pos[0], 'lon': pos[1], 'orientation': pos[2]}
        else:
            return {'status': 'error', 'message': 'position is None'}


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_point_types(self):
        try:
            with open(YAML_PATH, 'r') as file:
                return {'status': 'ok', 'pointtypes': {"map-elements": yaml.safe_load(file)['map-elements']}}
        except Exception as err:
            return {'status': ''.join(traceback.TracebackException.from_exception(err).format())}
        
    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_maps(self):
        try:
            map_files = [f for f in listdir(MAPS_PATH) if isfile(join(MAPS_PATH, f))]
            return {'status': 'ok', 'maps': map_files}
        except Exception as err:
            return {'status': ''.join(traceback.TracebackException.from_exception(err).format())}

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def save_map(self, filename, map_data):
        try:
            self.log(f'SAVED MAP DATA: {map_data}')
            with open(f'{MAPS_PATH}/{filename}.geojson', 'w') as f:
                json.dump(json.loads(map_data), f)
            with open(f'{MAPS_PATH}/{filename}.geojson') as mapdatafile:            
                self.world_model.load_map(yaml.safe_load(mapdatafile))
            return {'status': 'ok'} 
        except Exception as err:        
            self.log(''.join(traceback.TracebackException.from_exception(err).format()))
            return {'status': 'error', 'message': ''.join(traceback.TracebackException.from_exception(err).format())}

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_params(self):
        return self.world_model.params

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def load_map(self, filename):
        try:
            with open(f'{MAPS_PATH}/{filename}') as f:
                j = json.load(f)
                return {'status': 'ok', 'features' : j} 
        except Exception as err:        
            self.log(''.join(traceback.TracebackException.from_exception(err).format()))
            return {'status': 'error', 'message': ''.join(traceback.TracebackException.from_exception(err).format())}


    @cherrypy.expose
    def get_sign_label(self):
        if self.world_model.found_sign is None:
            return json.dumps({"detected": False, "sign": "знак не обнаружен"})
        return json.dumps({"detected": True, "sign": self.world_model.found_sign[1]})

    @cherrypy.expose
    def get_image(self, img_type, tm):
        if img_type == "obj_detector":
            if self.world_model.img_front_objects_lines_signs is None:
                return None
            data = self.world_model.img_front_objects_lines_signs
        elif img_type == "seg":
            self.world_model.draw_scene()
            if self.world_model.ipm_colorized_lines is None:
                return None
            data = self.world_model.ipm_colorized_lines
        elif img_type == "sign":
            if self.world_model.found_sign is None:
                return None
            data = self.world_model.found_sign[2]
            # use StringIO to stream the image out to the browser direct from RAM
            # output = StringIO.StringIO()
            # format = 'PNG' # JPEG and other formats are available
            # data.save(output, format)
            # contents = output.getvalue()
            # output.close()
        cherrypy.response.headers['Content-Type'] = "image/png"
        contents = cv2.imencode('.png', data)[1].tostring()
        return contents

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_init_point(self):
        config_path = os.path.join(BASE_RESOURCE_PATH,
                                    pathlib.Path(os.path.join(BASE_RESOURCE_PATH, 'config', 'map_config.yaml')))
        if not os.path.exists(config_path):
            print('Global map coords config file file not found. Use default values')
            return
        with open(config_path) as file:
            config = yaml.full_load(file)
        return {'lat': config['lat'], 'lon': config['lon'], 'mapfile': config['mapfile']}


def start_web_server(map_server):
    try:

        cherrypy.config.update({
            'log.screen': False
        })

        # Отключение всех логеров CherryPy
        for log in ('cherrypy.access', 'cherrypy.error'):
            logger = logging.getLogger(log)
            logger.propagate = False
            logger.handlers = []
            logger.addHandler(logging.NullHandler())
        cherrypy.quickstart(map_server, '/', {'global':
                                                   {'server.socket_host': '0.0.0.0',
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
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except  Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()

