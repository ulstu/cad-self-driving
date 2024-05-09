import os
import cv2
import sys
import socket
import warnings
import traceback
# import pyzed.sl as sl
import tensorflow as tf
from cnn import CNN
from fastseg import MobileV3Large
from sklearn.preprocessing import OneHotEncoder
import numpy as np
from PIL import Image
import time

warnings.filterwarnings("ignore", category=DeprecationWarning)

use_cpu = True

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
                 video_dir='perception/signs_detector/data/road2.mp4'):
        self.is_viz = is_viz
        self.is_video = is_video
        self.delay = delay
        self.is_correct_size = is_correct_size
        self.correct_width = correct_width
        if self.is_video:
            self.cap = cv2.VideoCapture(video_dir)
#        self.cap = cv2.VideoCapture(2)
        # if not is_video:
        #     self.init_camera()

        #self.seg_model = MobileV3Large.from_pretrained('mobilev3large-lraspp.pt').cuda().eval()
        if use_cpu == True:
            self.seg_model = MobileV3Large.from_pretrained(path_to_seg_model).eval()
        else:
            self.seg_model = MobileV3Large.from_pretrained(path_to_seg_model).cuda().eval()
        
        if use_cpu == True:
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
        #     self.zed.retrieve_image(self.depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, self.image_size)
        #     self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, self.image_size)
        #     img = self.image_zed.get_data()

        #     # img = cv2.flip(img, 0)
        #     # img = cv2.flip(img, 1)
        #     return img[:, :, :3]
        # else:
        #     return None

    def classify_traffic_light(self, images):
        lights = None
        square = 0
        for image in images:
            height, width, channels = image.shape
            local_square = height * width
            if height / width < 0.55 or local_square < square:
                continue
            square = local_square
            up = image[:int(height / 2), :, :]
            down = image[int(height / 2):, :, :]
            if np.average(up) > np.average(down):
                lights = 'red'
            else:
                lights = 'green'
            #print(f'Светофор: {lights}')
        return lights


    def get_classify_images(self, images):
        sign = None
        square = 0
        #print(f'squa
        # re: {square}')
        finded_signs = []
        for image in images:
            height, width, channels = image.shape
            local_square = height * width
            if local_square < square:
                continue

            image = cv2.resize(image, (64, 64))
            _, y = self.cnn_model.predict([image])
            y = y[0]
            index_y = np.argmax(y)
            finded_signs.append([index_y, y[index_y]])
            print(self.decode_labels([np.round(y)])[0][0], y[index_y])
            # temp_sign = self.decode_labels([np.round(y)])[0][0]
            if y[index_y] > 0.7 or y[index_y] > 0.5 and '3_24_' in self.decode_labels([np.round(y)])[0][0] :
                square = local_square
                sign = self.decode_labels([np.round(y)])[0][0]

        return sign, finded_signs

    def visualize(self, sign):
        if sign is not None:
            # print(sign)
            icon = np.copy(self.icons[self.inverse_labels[sign]])
            image = cv2.resize(icon, (500, 500))
            cv2.imshow('sign', image)
            text_image = np.zeros((60, 230, 3), np.uint8)
            sign = sign.replace('0', '').replace('_', '.')
            cv2.putText(text_image, sign, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
            cv2.imshow('label', text_image)
            # cv2.waitKey(0)


    def run(self):
        size_que = 16
        signs_que = [None] * size_que
        traffic_que = [None] * size_que
        while True:
            image = self.get_image()
            
            if self.is_correct_size:
                correct_height = image.shape[0] / image.shape[1] * self.correct_width
                image = cv2.resize(image, (int(self.correct_width), int(correct_height)))
            
            # cv2.imshow('main', image)
            key = cv2.waitKey(10)
            if key == 113:
                break

            analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            
            if use_cpu == True:
                labels = self.seg_model.predict_one(np.array(analyze_image), device='cpu')
            else:
                labels = self.seg_model.predict_one(np.array(analyze_image))
            traffic_light_mask = np.ones_like(labels)
            traffic_light_mask[labels != 6] = 0
            traffic_sign_mask = np.ones_like(labels)
            traffic_sign_mask[labels != 7] = 0

            # Идёт прибавление по причине того, что маска светофора путается со знаками
            # if self.is_red:
            #     traffic_light_mask += traffic_sign_mask
            
            traffic_light_images, light_rects = self.cut_image4segments(image, np.array(traffic_light_mask, dtype=np.uint8))
            traffic_sign_images, sign_rects = self.cut_image4segments(image, np.array(traffic_sign_mask, dtype=np.uint8))


            #print(self.classify_traffic_light(traffic_light_images), len(traffic_light_images))
            if traffic_light_images and self.is_red:
                traffic_light = self.classify_traffic_light(traffic_light_images)
                print(traffic_light)
                traffic_que[:size_que - 1] = traffic_que[1:]
                traffic_que[-1] = traffic_light
                most_common_tl = max(traffic_que, key=traffic_que.count)
                count_most_common_tl = traffic_que.count(most_common_tl)
                print(f'Текущий сигнал светофора: {most_common_tl}')
                if most_common_tl == 'green':  # and count_most_common_tl > 6:
                    print('Зеленый сигнал светофора обнаружен')
                    self.is_red = False
                    # self.send_start_cmd() # Ошибка
                    #print(f'traffic light detected = {traffic_light}')
            
            
            finded_signs = []
            if traffic_sign_images and not self.is_red:
                sign, finded_signs = self.get_classify_images(traffic_sign_images)
                signs_que[:size_que-1] = signs_que[1:]
                signs_que[-1] = sign
            
            
            # print(signs_que)
            most_common_item = "..."
            most_common_item = max(signs_que, key=signs_que.count)
            count_most_common_item = signs_que.count(most_common_item)
            # if most_common_item: #and count_most_common_item > 9:
            #     self.visualize(most_common_item)
            
            if self.is_viz:
                image_to_draw = np.copy(image)
                self.draw_mask(image_to_draw, traffic_light_mask, [255, 0, 0])
                self.draw_mask(image_to_draw, traffic_sign_mask, [0, 0, 255])
                self.draw_signs(image_to_draw, sign_rects, finded_signs)
                self.draw_traffic_light(image_to_draw, 2, [self.is_red, not self.is_red])
                self.draw_finded_sign(image_to_draw, most_common_item)
                cv2.imshow("visualisation", image_to_draw)
    
    
    def draw_finded_sign(self, 
                         image, 
                         sign_name, 
                         icon_width=60,
                         margin=5, 
                         padding=5, 
                         offset=5,
                         alpha=0.6,
                         color=(255, 255, 255),
                         font_scale=0.8,
                         thickness=2,
                         default_text="Sign not found"):
        
        text = default_text
        if sign_name in self.labels:
            text = sign_name
        
        text_background_position = np.array([margin, margin])
        text_position = text_background_position + [padding, padding]
        
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness=thickness)[0]
        text_background_size = np.array([padding, padding]) * 2 + text_size
        
        mask_image = np.zeros_like(image)
        cv2.rectangle(mask_image, text_background_position, text_background_position + text_background_size, color=(25, 25, 25), thickness=-1)
        indices = np.any(mask_image != np.array([0, 0, 0], dtype=np.uint8), axis=-1)
        image[indices] = cv2.addWeighted(image, 1 - alpha, mask_image, alpha, 0)[indices]
        
        cv2.putText(image, text, text_position + [0, text_size[1]], cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale, color=color, thickness=thickness)
        
        icon_pos = text_background_position + [0, text_background_size[1] + offset]
        
        icon = np.copy(self.get_icon(text))
        icon_height = int(icon.shape[0] / icon.shape[1] * icon_width)
        resized_icon = cv2.resize(icon, (int(icon_width), int(icon_height)))
        image[icon_pos[1]:icon_pos[1] + icon_height, icon_pos[0]:icon_pos[0] + icon_width] = resized_icon
        
    
    def draw_traffic_light(self, 
                           image, 
                           count_circles=2, 
                           circle_states=[True, False],
                           colors=[(235, 64, 52), (52, 235, 92)],
                           dark_factor = 0.6,
                           background_color=(25, 25, 25),
                           background_alpha=0.6, 
                           margin=5, 
                           padding=5, 
                           radius=20, 
                           offset=5):
        lb_traffic_light_possition = np.array([margin, image.shape[0] - margin])
        traffic_light_height = (radius * 2 + offset) * count_circles - offset + padding * 2
        traffic_light_width = padding * 2 + radius * 2
        traffic_light_size = [traffic_light_width, traffic_light_height]
        
        lt_traffic_light_position = lb_traffic_light_possition - np.array([0, traffic_light_height])
        
        image_mask = np.zeros_like(image)
        cv2.rectangle(image_mask, lt_traffic_light_position, lt_traffic_light_position + traffic_light_size, background_color, thickness=-1)
        indices = np.any(image_mask != np.array([0, 0, 0], dtype=np.uint8), axis=-1)
        image[indices] = cv2.addWeighted(image, 1 - background_alpha, image_mask, background_alpha, 0, image_mask)[indices]
        
        circle_position = lt_traffic_light_position + np.array([traffic_light_size[0] / 2, padding + radius], dtype=np.int32)
        for color, state in zip(colors, circle_states):
            if not state:
                color = tuple((np.array(list(color)) * (1 - dark_factor)).tolist())
            
            color = (color[2], color[1], color[0])
            cv2.circle(image, circle_position.astype(int), radius, color, thickness=-1)
            circle_position[1] += radius * 2 + offset 
        
    
        
    def draw_mask(self, image, numerical_mask, color):
        color = np.array(color, dtype=np.uint8)
        mask_image = np.zeros_like(image)
        mask_image[numerical_mask != 0] = color
        indices = np.any(mask_image != np.array([0, 0, 0], dtype=np.uint8), axis=-1)
        if np.all(indices):
            pass
        image[indices] = cv2.addWeighted(image, 0.6, mask_image, 0.4, 0)[indices]
    
    
    def get_icon(self, icon_name):
        if icon_name in self.inverse_icon_labels:
            return self.icons[self.inverse_icon_labels[icon_name]]
        else:
            return np.zeros(shape=(64, 64, 3), dtype=np.uint8) # Заглушка
    

    def predict_labels(self, image):
        if use_cpu == True:
            labels = detector.seg_model.predict_one(np.array(analyze_image), device='cpu')
        else:
            labels = detector.seg_model.predict_one(np.array(analyze_image))
        return labels


    def predict_traffic_lights(self, image, labels=None):
        if labels is None:
            labels = self.predict_labels(image)

        traffic_light_mask = np.ones_like(labels)
        traffic_light_mask[labels != 6] = 0

        traffic_light_images, light_rects = detector.cut_image4segments(image, np.array(traffic_light_mask, dtype=np.uint8))

        return traffic_light_mask, traffic_light_images, light_rects


    def predict_signs(self, image, labels=None):
        if labels is None:
            labels = self.predict_labels(image)

        traffic_sign_mask = np.ones_like(labels)
        traffic_sign_mask[labels != 7] = 0

        traffic_sign_images, sign_rects = detector.cut_image4segments(image, np.array(traffic_sign_mask, dtype=np.uint8))

        found_signs = []
        if traffic_sign_images and not detector.is_red:
            sign, found_signs = detector.get_classify_images(traffic_sign_images)
        
        return traffic_sign_mask, traffic_sign_images, sign_rects, found_signs


    def plot_predictions(self, image, image_to_plot_on=None):
        analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        
        if use_cpu == True:
            labels = detector.seg_model.predict_one(np.array(analyze_image), device='cpu')
        else:
            labels = detector.seg_model.predict_one(np.array(analyze_image))

        traffic_light_mask, traffic_light_images, light_rects = detector.predict_traffic_lights(image, labels)
        traffic_sign_mask, traffic_sign_images, sign_rects, found_signs = detector.predict_signs(image, labels)

        if image_to_plot_on is not None:
            image = image_to_plot_on

        detector.draw_mask(image, traffic_light_mask, [255, 0, 0])
        detector.draw_mask(image, traffic_sign_mask, [0, 0, 255])
        detector.draw_signs(image, sign_rects, found_signs)
        detector.draw_traffic_light(image, 2, [detector.is_red, not detector.is_red])
        return image
        

    
    # def predcit_

        
    def draw_signs(self, image, sign_rects, sign_values, color=(0, 255, 0), icon_color=(255, 0, 0), thickness=2, min_value=0.8, icon_ratio=0.7, min_icon_size=30):
        for idx, (label, value) in enumerate(sign_values):
            if value >= min_value:
                x, y, w, h = sign_rects[idx]
                
                p1 = np.array([x, y], dtype=np.int32)
                p2 = p1 + np.array([w, h], dtype=np.int32)
                
                icon_width = w * icon_ratio
                if icon_width < min_icon_size:
                    icon_width = min_icon_size
                
                if w < min_icon_size:
                    delta_w = min_icon_size - w
                    p1[0] -= delta_w / 2
                    p2[0] += delta_w / 2
                
                icon = np.copy(self.get_icon(self.labels[label]))
                icon_height = int(icon.shape[0] / icon.shape[1] * icon_width)
                icon_width = int(icon_width)
                
                icon_pos = p1 - np.array([0, icon_height], dtype=np.int32)
                if icon_pos[1] < 0:
                    icon_pos[1] = 0
                    
                icon_pos[0] = min(max(0, icon_pos[0]), image.shape[1] - icon_width)
                icon_pos[1] = min(max(0, icon_pos[1]), image.shape[0] - icon_height)

                
                cv2.rectangle(image, (x, y), (x + w, y + h), color=color, thickness=thickness)
                
                resized_icon = cv2.resize(icon, (int(icon_width), int(icon_height)))
                image[int(icon_pos[1]):int(icon_pos[1] + icon_height), int(icon_pos[0]):int(icon_pos[0] + icon_width)] = resized_icon
                
                cv2.rectangle(image, (icon_pos[0], icon_pos[1]), (icon_pos[0] + icon_width, icon_pos[1] + icon_height), color=icon_color, thickness=thickness)
        
        
    cv2.destroyAllWindows()
    #self.zed.close() #################
    

if __name__ == '__main__':
    with tf.device('/CPU:0'):
        # detector = ImageAnalyzer(path_to_cnn_model="perception/signs_detector/weights/model-ep50-signs16/",
        #                          path_to_seg_model="perception/signs_detector/mobilev3large-lraspp.pt",
        #                          path_to_icons="perception/signs_detector/signs_icon/",
        #                          video_dir='perception/signs_detector/data/road.avi',
        #                          is_video=True,
        #                          is_red=False,
        #                          is_correct_size=True,
        #                          correct_width=1000)

        # detector.run()

        detector = ImageAnalyzer(path_to_cnn_model="perception/signs_detector/weights/model-ep50-signs16/",
                                path_to_seg_model="perception/signs_detector/mobilev3large-lraspp.pt",
                                path_to_icons="perception/signs_detector/signs_icon/",
                                is_video=False,
                                is_red=False,
                                is_correct_size=True,
                                correct_width=1000)
        
        image = cv2.imread("perception/signs_detector/data/annotations/images/Screenshot 2024-04-28 212510.png")
        image2 = cv2.imread("perception/signs_detector/data/annotations/images/Screenshot 2024-04-28 212510.png")

        detector.plot_predictions(image, image2)

        cv2.imshow("Image", image2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
