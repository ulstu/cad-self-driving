from AbstractWorker import AbstractWorker
from webots_ros2_suv.lib.linalg import VECTOR2, POINT
from webots_ros2_suv.lib.field_builder import gps_to_rect
import math
from time import time
from webots_ros2_suv.lib.gis import session, FieldChank


class AgronavigatorWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.is_field_chanks = True
        self.is_load_chanks = False
        self.is_spray = False
        self.drawable_chanks = []
        self.chank_generate_time = 0
        self.spray_radius = 1
        self.chank_load_time = time()
        self.chank_update_time = time()
        self.last_chanks_time = time() + 10
        self.sprayers_offsets = [VECTOR2(1, 0),
                                 VECTOR2(1, 1.5),
                                 VECTOR2(1, 3),
                                 VECTOR2(1, -1.5),
                                 VECTOR2(1, -3)]
        self.surround_deepth = 25

    def on_event(self, event, scene=None):
        print("Emergency State")
        return None
    
    def update_surround_chanks(self, world_model, center_chank_position):
        world_model.surround_chanks = session.query(FieldChank).filter((FieldChank.position_x >= center_chank_position.x - self.surround_deepth) & (FieldChank.position_x <= center_chank_position.x + self.surround_deepth)).filter((FieldChank.position_y >= center_chank_position.y - self.surround_deepth) & (FieldChank.position_y <= center_chank_position.y + self.surround_deepth))
        return world_model


    def on_data(self, world_model):
        lat, lon, orientation = world_model.get_current_position() # Текущее месторасположение автомобиля
        orientation -= 1.5
        car_position = gps_to_rect(lat, lon)

        center_chank_position = POINT(int(car_position[0]), int(car_position[1]))
        world_model = self.update_surround_chanks(world_model, center_chank_position)

        # Рассчет положений распрыскивателей
        sprayers_positions = []
        for sprayer_offset in self.sprayers_offsets:
            sprayer_offset = sprayer_offset.rotate(orientation)
            sprayer_position = POINT(car_position[0], car_position[1])
            sprayer_position.x += sprayer_offset.x
            sprayer_position.y += sprayer_offset.y
            sprayers_positions.append(sprayer_position)

        # Симуляция опрыскивания
        # if world_model.is_spray:
        #     self.logi("spray")
        #     for sprayer_position in sprayers_positions:
        #         new_polygon = f'({sprayer_position.x - self.spray_radius} {sprayer_position.y - self.spray_radius},{sprayer_position.x + self.spray_radius} {sprayer_position.y - self.spray_radius},{sprayer_position.x + self.spray_radius} {sprayer_position.y + self.spray_radius},{sprayer_position.x - self.spray_radius} {sprayer_position.y + self.spray_radius},{sprayer_position.x - self.spray_radius} {sprayer_position.y - self.spray_radius})'
        #         chanks = world_model.surround_chanks.filter(FieldChank.polygon.ST_Intersects(f'POLYGON({new_polygon})')).all()
        #         # print(len(chanks), new_polygon)
        #         average_irrigation = 0
        #         for chank in chanks:
        #             average_irrigation += chank.irrigation_degree
        #         if len(chanks) > 0:
        #             average_irrigation /= len(chanks)
        #         irrigation_coefficient = min(math.sqrt(max(0, 30 - average_irrigation)) * 1.3, 10)
        #         for chank in chanks:
        #             if irrigation_coefficient > 0:
        #                 chank.irrigation_degree = min(100, chank.irrigation_degree + 10 / irrigation_coefficient)
        #     if time() - self.chank_update_time > 5:
        #         session.commit()
        #         self.chank_update_time = time()

        return world_model


