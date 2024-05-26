# import numpy as np
# from fastseg import MobileV3Large
# import tensorflow as tf
# from webots_ros2_suv.lib.cnn import CNN
# from sklearn.preprocessing import OneHotEncoder


# def predcit_labels(image) -> np.ndarray:
#     return image


# def predict_traffic_lights(image, labels=None):
#     '''
#         Формат массива найденных квадратов со светофорами:
#         boxes[id][v]
        
#         id - номер найденного на изображении квадрата
#         p1 = [boxes[id][0], boxes[id][1]] - первая точка квадрата
#         p2 = [boxes[id][2], boxes[id][3]] - вторая точка квадрата
#         state = boxes[id][4] - состояние светофора
#     '''
#     return np.zeros((3, 5), dtype=np.int32)


# def predict_signs(image, labels=None):
#     '''
#         Формат массива найденных квадратов со знаками:
#         boxes[id][v]
        
#         id - номер найденного на изображении квадрата
#         p1 = [boxes[id][0], boxes[id][1]] - первая точка квадрата
#         p2 = [boxes[id][2], boxes[id][3]] - вторая точка квадрата
#         cls = boxes[id][4] - номер класса обнаруженного объекта
#     '''
#     return np.zeros((3, 5), dtype=np.int32)

# class SignTrafficDetector():
#     def __init__(self, path_to_fastseg_model, path_to_cnn_model, use_gpu=True):
#         if use_gpu == True:
#             self.seg_model = MobileV3Large.from_pretrained(path_to_fastseg_model).cuda().eval()
#             self.cnn_model = CNN(gpu_load=0.7)
#         else:
#             self.seg_model = MobileV3Large.from_pretrained(path_to_fastseg_model).eval()
#             with tf.device('/CPU:0'):
#                 self.cnn_model = CNN(gpu_load=0.7)
        
#         self.cnn_model.load_model(path_to_cnn_model)
#         self.labels = self.cnn_model.labels
#         self.inverse_labels = {}
#         for idx, label in enumerate(self.labels):
#             self.inverse_labels[label] = idx
        
#         self.init_OneHotEncoder()

#     def init_OneHotEncoder(self):
#         self.enc = OneHotEncoder(handle_unknown='ignore')
#         self.enc.fit(np.array(self.labels).reshape(-1, 1))

#     def encode_labels(self, labels):
#         return self.enc.transform(np.array(labels).reshape(-1, 1)).toarray()

#     def decode_labels(self, labels):
#         return self.enc.inverse_transform(labels)


import os
import cv2
import sys
import socket
import warnings
import traceback
# import pyzed.sl as sl
import tensorflow as tf
from fastseg import MobileV3Large
from webots_ros2_suv.lib.cnn import CNN
from sklearn.preprocessing import OneHotEncoder
import numpy as np
from PIL import Image
import time

warnings.filterwarnings("ignore", category=DeprecationWarning)

HOST = "192.168.1.188"  # Standard loopback interface address (localhost)
PORT = 65432

class ImageAnalyzer:
    def __init__(self, 
                 path_to_cnn_model,
                 path_to_seg_model,
                 path_to_icons,
                 is_red=False,
                 delay=5, 
                 is_viz=True, 
                 is_video=False,
                 is_correct_size=True,
                 correct_width=1000,
                 video_dir='perception/signs_detector/data/road2.mp4', 
                 use_gpu=True):
        self.is_viz = is_viz
        self.is_video = is_video
        self.delay = delay
        self.is_correct_size = is_correct_size
        self.correct_width = correct_width
        self.use_gpu = use_gpu
        if self.is_video:
            self.cap = cv2.VideoCapture(video_dir)
#        self.cap = cv2.VideoCapture(2)
        # if not is_video:
        #     self.init_camera()

        #self.seg_model = MobileV3Large.from_pretrained('mobilev3large-lraspp.pt').cuda().eval()
        if self.use_gpu == False:
            self.seg_model = MobileV3Large.from_pretrained(path_to_seg_model).eval()
        else:
            self.seg_model = MobileV3Large.from_pretrained(path_to_seg_model).cuda().eval()
        
        if self.use_gpu == False:
            with tf.device('/CPU:0'):
                self.cnn_model = CNN(gpu_load=0.7)
        else:
            self.cnn_model = CNN(gpu_load=0.7)

        self.cnn_model.load_model(path_to_cnn_model)
        print('init model')
        self.count = 0
        self.is_red = is_red
        
        self.labels = self.cnn_model.labels
        self.inverse_labels = {}
        for idx, label in enumerate(self.labels):
            self.inverse_labels[label] = idx
        
        icon_filenames = []
        self.icon_labels = []
        labels_set = set(self.labels)
        for icon_filename in os.listdir(path_to_icons):
            if icon_filename.split('.')[0] in labels_set:
                icon_filenames.append(icon_filename)
                self.icon_labels.append(icon_filename.split('.')[0])

        self.inverse_icon_labels = {}
        for idx, label in enumerate(self.icon_labels):
            self.inverse_icon_labels[label] = idx

        icon_paths = [os.path.join(path_to_icons, icon_filename) for icon_filename in icon_filenames]
        self.icons = [cv2.imread(path) for path in icon_paths]
        
        self.init_OneHotEncoder()

    def send_start_cmd(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((HOST, PORT))
                s.sendall(b"greenstart")
        except Exception as err:
            print(f'Socket send error: {traceback.format_exc()}')


    # def init_camera(self):
    #     self.zed = sl.Camera()
    #     input_type = sl.InputType()

    #     # if len(sys.argv) >= 2:
    #     #     input_type.set_from_svo_file(sys.argv[1])
    #     print('init camera')
    #     init = sl.InitParameters(input_t=input_type)
    #     init.camera_resolution = sl.RESOLUTION.HD1080
    #     init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    #     init.coordinate_units = sl.UNIT.MILLIMETER

    #     # Open the camera
    #     err = self.zed.open(init)
    #     if err != sl.ERROR_CODE.SUCCESS:
    #         print(repr(err))
    #         self.zed.close()
    #         exit(1)
    #     self.runtime = sl.RuntimeParameters()
    #     #self.runtime.sensing_mode = sl.SENSING_MODE.STANDARD
    #     self.image_size = self.zed.get_camera_information().camera_configuration.resolution
    #     self.image_size.width = self.image_size.width / 2
    #     self.image_size.height = self.image_size.height / 2

    #     # Declare your sl.Mat matrices
    #     self.image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
    #     self.depth_image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
    #     self.point_cloud = sl.Mat()

    def init_OneHotEncoder(self):
        self.enc = OneHotEncoder(handle_unknown='ignore')
        self.enc.fit(np.array(self.labels).reshape(-1, 1))

    def encode_labels(self, labels):
        return self.enc.transform(np.array(labels).reshape(-1, 1)).toarray()

    def decode_labels(self, labels):
        return self.enc.inverse_transform(labels)


    def cut_image4segments(self, image, mask):
        # print(np.max(mask))
        # if np.max(mask) == 0:
        #     return []
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        boundRect = []
        min_size = 20
        b = 0
        height_image, width_image, channels = image.shape
        for i, c in enumerate(contours):
            contours_poly = cv2.approxPolyDP(c, 3, True)
            x, y, w, h = cv2.boundingRect(contours_poly)
            if w > min_size and h > min_size and x > width_image / 2 and y < height_image / 2 and w / h < 10 \
                    and h / w < 10:
                boundRect += [[x + b, y + b, w - 2 * b, h - 2 * b]]
        images = [image[y:y + h, x:x + w, :] for (x, y, w, h) in boundRect]

        return images, boundRect


    def get_image(self):
        if self.is_video:
            time_start = time.time()
            while True:
                ret, image = self.cap.read()
                if not ret or time.time() - time_start < self.delay:
                    break
            
            if not ret:
                exit()
            return image

        # err = self.zed.grab(self.runtime)
        # if err == sl.ERROR_CODE.SUCCESS:
        #     # Retrieve the left image, depth image in the half-resolution
        #     self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)