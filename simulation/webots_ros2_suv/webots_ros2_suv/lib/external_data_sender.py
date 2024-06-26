from prometheus_client import Enum
from prometheus_client import Gauge
from prometheus_client import Info
from prometheus_client import start_http_server
import time

class ExternalDataSender(object):
    def __init__(self) -> None:
        self.run_id = f'{time.strftime("%Y%m%d-%H%M%S")}'
        self.state = Enum('state', 'Текущее состяоние ВАТС', labelnames=['run_id'], states=['start', 'moving', 'lanefollow', 'gpsfollow', 'stopped', 'turn', 'emergency'])
        self.lat = Gauge('lat', 'Широта', ['run_id'])
        self.lon = Gauge('lon', 'Долгота', ['run_id'])
        self.angle = Gauge('angle', 'Азимут', ['run_id'])
        self.speed = Gauge('speed', 'Скорость', ['run_id'])
        self.steering = Gauge('steering', 'Угол поворота руля', ['run_id'])
        self.point_dist = Gauge('point_dist', 'Расстояние до ближайшей точки', ['run_id'])
        self.cur_path_segment = Gauge('cur_path_segment', 'Текущий сегмент пути', ['run_id'])
        self.cur_point = Gauge('cur_point', 'Текущая точка пути', ['run_id'])
        self.path_len = Gauge('path_len', 'Длина спланированного пути', ['run_id'])
        start_http_server(8010)

    def send_data(self, params):
        if 'state' in params:
            self.state.labels(run_id=self.run_id).state(params['state'].split()[0])
        self.lat.labels(run_id=self.run_id).set(params.get('lat', 0.0))
        self.lon.labels(run_id=self.run_id).set(params.get('lon', 0.0))
        self.angle.labels(run_id=self.run_id).set(params.get('angle', 0.0))
        self.speed.labels(run_id=self.run_id).set(params.get('speed', 0.0))
        self.steering.labels(run_id=self.run_id).set(params.get('steering', 0.0))
        self.point_dist.labels(run_id=self.run_id).set(params.get('point_dist', 0.0))
        self.cur_path_segment.labels(run_id=self.run_id).set(params.get('cur_path_segment', 0))
        self.cur_point.labels(run_id=self.run_id).set(params.get('cur_point', 0))
        self.path_len.labels(run_id=self.run_id).set(params.get('final_path_len', 0))
        