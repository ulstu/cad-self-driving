from AbstractWorker import AbstractWorker
import torch
import pathlib
import os
import numpy as np
import traceback
from PIL import Image
from ament_index_python.packages import get_package_share_directory
from fastseg import MobileV3Large
from fastseg.image import colorize, blend
from webots_ros2_suv.lib.timeit import timeit
from webots_ros2_suv.lib.lane_line_model import LaneLineModel
from webots_ros2_suv.lib.lane_line_model_utils import get_label_names, draw_lines, draw_segmentation, LaneLine, LaneMask, default_palette
from webots_ros2_suv.lib.map_builder import MapBuilder
import cv2

PACKAGE_NAME = 'webots_ros2_suv'
local_model_path = 'lane_line_model/lane_line_detection_yolo8s-seg_model.pt'
local_config_path = 'lane_line_model/config.yaml'

class LaneLineDetectionWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        package_dir = get_package_share_directory(PACKAGE_NAME)
        model_path = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', local_model_path)))
        config_path = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', local_config_path)))

        self.labels = get_label_names(config_path)
        self.lane_line_model = LaneLineModel(model_path, use_curve_line=True)
        

    def on_event(self, event, scene=None):
        return None


    def on_data(self, world_model):
        try:
            img = np.array(Image.fromarray(world_model.rgb_image))
            line_batches, mask_batches, results = self.lane_line_model.predict([img])

            world_model.img_front_objects_lines = self.lane_line_model.generate_prediction_plots([world_model.img_front_objects], self.labels, mask_batches, line_batches)[0]
            world_model.ipm_colorized_lines = world_model.ipm_colorized

            lines_bev = []
            
            for line in line_batches[0]:
                points_bev = []
                for point in line.points:
                    x, y = world_model.map_builder.calc_bev_point(point)
                    point_bev = np.array([x, y])

                    global_pos = world_model.coords_transformer.get_global_coordinates_from_ipm_coords(point[0], point[1], world_model.get_current_position())
                    local_pos = world_model.coords_transformer.get_relative_coordinates(global_pos[0], global_pos[1], world_model.get_current_position(), world_model.pov_point)

                    points_bev.append([local_pos[0], local_pos[1]])
                
                points_bev = np.array(points_bev, dtype=np.int32)
                
                lines_bev.append(LaneLine(points_bev, line.label, line.points_n, line.elapsed_time, line.mask_count_points))
            
            line_batches_bev = [lines_bev]

            world_model.ipm_colorized_lines = np.copy(world_model.ipm_colorized)

            image_size = np.array(world_model.ipm_colorized_lines.shape[0:2])[::-1]
            center_point = image_size / 2
            for line in line_batches_bev[0]:
                for point in line.points:
                    cv2.line(world_model.ipm_colorized_lines, center_point.astype(int), point.astype(int), (0, 0, 255), thickness=2)

            draw_lines([world_model.ipm_colorized_lines], line_batches_bev, palette=[(0, 255, 0)], thickness=8)

            
            # count_points = [20, 20]
            # obj_image_size = np.array(world_model.img_front_objects_lines.shape[0:2])[::-1]
            # for x in range(obj_image_size[0] // count_points[0]):
            #     for y in range(obj_image_size[1] // count_points[1]):
            #         point_x = obj_image_size[0] / count_points[0] * x
            #         point_y = obj_image_size[1] / count_points[1] * y

            #         colorR = np.array([255, 0, 0])
            #         colorB = np.array([0, 0, 255])
            #         colorRB = (colorR * (x / count_points[0]) + colorB * (y / count_points[1]))
            #         colorRB = tuple(colorRB.astype(int).tolist())

            #         colorB = np.array([0, 0, 255])
            #         colorG = np.array([0, 255, 0])
            #         colorBG = (colorB * (x / count_points[0]) + colorG * (y / count_points[1]))
            #         colorBG = tuple(colorBG.astype(int).tolist())

            #         point_x_bev, point_y_bev = world_model.map_builder.calc_bev_point((point_x, point_y))
            #         cv2.circle(world_model.img_front_objects_lines, (int(point_x), int(point_y)), 3, colorRB, -1)
            #         cv2.circle(world_model.img_front_objects_lines, (int(point_x_bev), int(point_y_bev)), 3, colorBG, -1)
                
            # cv2.circle(world_model.img_front_objects_lines, (img.shape[0], int(img.shape[1] / 2)), 5, (255, 0, 0), -1)

        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))
        
        return world_model