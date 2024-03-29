from ultralytics import YOLO
from PIL import Image
import cv2
import numpy as np
import os
import yaml
import time
import traceback
from fastseg.image import colorize
from collections import defaultdict
from ament_index_python.packages import get_package_share_directory
# from orientation import local_to_global
# from behavioral_analysis import BehaviourAnalyser
# from ipm_transformer import IPMTransformer
from .orientation import local_to_global
from .behavioral_analysis import BehaviourAnalyser
from .ipm_transformer import IPMTransformer

BASE_RESOURCE_PATH = get_package_share_directory('webots_ros2_suv') + '/'

class MapBuilder(object):
    def __init__(self, model_path, ipm_config):
        self.__model = YOLO(model_path)
        self.load_ipm_config(ipm_config)
        self.__corr_depth_pos = (0, 10)
        self.__track_history = defaultdict(lambda: [])
        self.__track_history_bev = defaultdict(lambda: [])
        model_dir = BASE_RESOURCE_PATH + 'resource/'
        self.__behaviour_analyser = BehaviourAnalyser(model_dir=model_dir)
        self.__behaviour_analyser_bev = BehaviourAnalyser()

    def detect_objects(self, image):
        #results = self.__model.predict(source=image, save=False, save_txt=False)
        results = self.__model.track(source=image, persist=True, verbose=False)
        return results

    def load_ipm_config(self, config_path):
        if not os.path.exists(config_path):
            print('Config file not found. Use default values')
            return
        with open(config_path) as file:
            config = yaml.full_load(file)
        self.__homograpthy_matrix = np.array(config['homography'])
        self.__horizont_line_height = config['horizont']
        self.__img_height = config['height']
        self.__img_width = config['width']

    def get_horizont_line(self):
        return self.__horizont_line_height

    def generate_ipm(self, image, is_mono = False, need_cut=True):
        ipm_transformer = IPMTransformer(homography_matrix=self.__homograpthy_matrix)
        img_ipm = ipm_transformer.get_ipm(image, is_mono=is_mono, horizont=self.__horizont_line_height, need_cut=need_cut)
        return img_ipm

    def get_labels(self):
        labels = {0: u'__background__', 1: u'person', 2: u'bicycle', 3: u'car', 4: u'motorcycle', 5: u'airplane',
                    6: u'bus', 7: u'train', 8: u'truck', 9: u'boat', 10: u'traffic light', 11: u'fire hydrant',
                    12: u'stop sign', 13: u'parking meter', 14: u'bench', 15: u'bird', 16: u'cat', 17: u'dog',
                    18: u'horse', 19: u'sheep', 20: u'cow', 21: u'elephant', 22: u'bear', 23: u'zebra', 24: u'giraffe',
                    25: u'backpack', 26: u'umbrella', 27: u'handbag', 28: u'tie', 29: u'suitcase', 30: u'frisbee',
                    31: u'skis', 32: u'snowboard', 33: u'sports ball', 34: u'kite', 35: u'baseball bat',
                    36: u'baseball glove', 37: u'skateboard', 38: u'surfboard', 39: u'tennis racket', 40: u'bottle',
                    41: u'wine glass', 42: u'cup', 43: u'fork', 44: u'knife', 45: u'spoon', 46: u'bowl', 47: u'banana',
                    48: u'apple', 49: u'sandwich', 50: u'orange', 51: u'broccoli', 52: u'carrot', 53: u'hot dog',
                    54: u'pizza', 55: u'donut', 56: u'cake', 57: u'chair', 58: u'couch', 59: u'potted plant', 60: u'bed',
                    61: u'dining table', 62: u'toilet', 63: u'tv', 64: u'laptop', 65: u'mouse', 66: u'remote',
                    67: u'keyboard', 68: u'cell phone', 69: u'microwave', 70: u'oven', 71: u'toaster', 72: u'sink',
                    73: u'refrigerator', 74: u'book', 75: u'clock', 76: u'vase', 77: u'scissors', 78: u'teddy bear',
                    79: u'hair drier', 80: u'toothbrush'}

        return labels

    def calc_box_distance(self, boxes, depth_map):
        box_depths = []
        for box in boxes:
            depths = []
            for i in range(int(box[0]), int(box[2])):
                for j in range(int(box[1]) - self.__horizont_line_height, int(box[3]) - self.__horizont_line_height):
                    depths.append(depth_map[j + self.__corr_depth_pos[1], i + self.__corr_depth_pos[0]])
            depths.sort(reverse=True)
            box_depths.append(np.percentile(depths, 90))
        return box_depths

    def calc_bev_point(self, p):
        m = self.__homograpthy_matrix
        px = ((m[0][0] * p[0] + m[0][1] * p[1] + m[0][2]) / ((m[2][0] * p[0] + m[2][1] * p[1] + m[2][2])))
        py = ((m[1][0] * p[0] + m[1][1] * p[1] + m[1][2]) / ((m[2][0] * p[0] + m[2][1] * p[1] + m[2][2])))
        return (int(px), int(py))

    def transform_boxes(self, cboxes):
        box_points = []
        widths = []
        for b in cboxes:
            p = np.asarray([
                b[0] + (b[2] - b[0]) / 2,
                b[3] - self.__horizont_line_height
                ], 
                dtype='float32')
            b0 = self.calc_bev_point(np.array([b[0], b[3]]))
            b1 = self.calc_bev_point(np.array([b[2], b[3]]))
            widths.append(b1[0] - b0[0])
            bev_point = self.calc_bev_point(p)
            box_points.append(bev_point)
        return box_points, widths

    def remove_detected_objects(self, image_seg, cboxes):
        image_seg[image_seg == 0] = 100
        excluded_classes = [7, 14]     # !!!!!! этот код здесь из-за ошибочного определения поезда вместо отбойника 
        for b in cboxes:
            if (int(b[-1]) + 1) in excluded_classes:
                continue
            corr = 5
            image_seg[int(b[1])-corr:int(b[3]) + corr, int(b[0]) - corr:int(b[2]) + corr] = 100
        return image_seg


    def put_objects(self, ipm_image, tbs, widths, results):
        excluded_classes = [7, 14]     # !!!!!! этот код здесь из-за ошибочного определения поезда вместо отбойника 
        for i in range(len(tbs)):
            label_num = int(results[0].boxes.data[i][-1]) + 1
            if label_num in excluded_classes:
                continue
            l = self.get_labels()[label_num]
            p1 = (int(tbs[i][0] - widths[i] / 0.45), int(tbs[i][1] - widths[i] / 0.8))
            p2 = (int(tbs[i][0] + widths[i] / 0.45), int(tbs[i][1]))
            ipm_image[p1[1]:p2[1],p1[0]:p2[0]] = label_num
        return ipm_image

    def track_objects(self, results, ipm_image, pos=(0, 0, 0), only_train=False):
        try:
            boxes = results[0].boxes.xywh.cpu()
            if len(results[0].boxes) < 1:
                return ipm_image, []
            track_ids = results[0].boxes.id.int().cpu().tolist()
            classes = [int(results[0].boxes.data[i][-1]) + 1 for i in range(len(results[0].boxes.data))]

            annotated_frame = results[0].plot()
            for box, track_id, obj_class in zip(boxes, track_ids, classes):
                x, y, w, h = box
                track = self.__track_history[track_id]
                track.append((float(x), float(y)))  # x, y center point
                bev_track = self.__track_history_bev[track_id]
                bev_point = self.calc_bev_point((int(x), (int(y + h / 2 - self.__horizont_line_height))))
                if not (0 < bev_point[1] < ipm_image.shape[0] and 0 < bev_point[0] < ipm_image.shape[1]): # Нужно проверить!!!!
                    continue
                if pos and bev_point:
                    x, y = local_to_global(bev_point[0], bev_point[1], pos[0], pos[1], pos[2])
                    bev_track.append((x, y))

                if len(track) > 60:  # retain 90 tracks for 90 frames
                    track.pop(0)
                    bev_track.pop(0)

                points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(annotated_frame, [points], isClosed=False, color=(0, 255, 0), thickness=3)
                if only_train:
                    self.__behaviour_analyser.train(obj_class, track)

                # pred_traj = self.__behaviour_analyser.run_spline(track, obj_class)
                else:
                    pred_traj = self.__behaviour_analyser.run(track, obj_class, need_train=False)
                    if pred_traj:
                        pred_points = np.hstack(pred_traj).astype(np.int32).reshape((-1, 1, 2))
                        cv2.polylines(annotated_frame, [pred_points], isClosed=False, color=(255, 0, 0), thickness=3)
                        pred_points_bev = []
                        for p in pred_traj:
                            p_bev = self.calc_bev_point((int(p[0]), (int(p[1] + h / 2 - self.__horizont_line_height))))
                            pred_points_bev.append(p_bev)

                        pred_points_bev = np.hstack(pred_points_bev).astype(np.int32).reshape((-1, 1, 2))
                        cv2.polylines(ipm_image, [pred_points_bev], isClosed=False, color=(255, 0, 0), thickness=3)

                if len(bev_track) > 0:
                    points_bev = np.hstack(bev_track).astype(np.int32).reshape((-1, 1, 2))
                    cv2.polylines(ipm_image, [points_bev], isClosed=False, color=(0, 255, 0), thickness=3)
                    # pred_traj_bev = self.__behaviour_analyser_bev.run_spline(bev_track, obj_class)
                    # if pred_traj_bev:
                    #     pred_points_bev = np.hstack(pred_traj_bev).astype(np.int32).reshape((-1, 1, 2))
                    #     cv2.polylines(ipm_image, [pred_points_bev], isClosed=False, color=(255, 0, 0), thickness=3)

            cv2.imshow("YOLOv8 Tracking", annotated_frame)
            return ipm_image, track_ids
        except  Exception as err:
            print(''.join(traceback.TracebackException.from_exception(err).format()))
            return ipm_image, []

    def resize_img(self, image):
        return cv2.resize(image, (self.__img_width, self.__img_height))


