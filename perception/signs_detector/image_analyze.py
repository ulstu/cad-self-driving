import os
import cv2
import sys
import socket
import warnings
import traceback
import pyzed.sl as sl
from cnn import CNN
from fastseg import MobileV3Large
from sklearn.preprocessing import OneHotEncoder
import numpy as np
from PIL import Image

warnings.filterwarnings("ignore", category=DeprecationWarning)


HOST = "192.168.1.188"  # Standard loopback interface address (localhost)
PORT = 65432

class ImageAnalyzer:
    def __init__(self, path_cnn_weights, is_viz=True, is_video=False, is_delay=5, video_dir='data/road.avi', is_red=False):
        self.is_viz = is_viz
        self.is_video = is_video
        self.is_delay = is_delay
        if self.is_video:
            self.cap = cv2.VideoCapture(video_dir)
#        self.cap = cv2.VideoCapture(2)
        if not is_video:
            self.init_camera()

        self.seg_model = MobileV3Large.from_pretrained('mobilev3large-lraspp.pt').cuda().eval()
        self.cnn_model = CNN(gpu_load=0.7)
        self.cnn_model.init_model()
        self.cnn_model.load_model(path_cnn_weights)
        print('init model')
        self.count = 0
        self.is_red = is_red
        self.init_OneHotEncoder()


    def send_start_cmd(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((HOST, PORT))
                s.sendall(b"greenstart")
        except Exception as err:
            print(f'Socket send error: {traceback.format_exc()}')


    def init_camera(self):
        self.zed = sl.Camera()
        input_type = sl.InputType()

        # if len(sys.argv) >= 2:
        #     input_type.set_from_svo_file(sys.argv[1])
        print('init camera')
        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = sl.RESOLUTION.HD1080
        init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init.coordinate_units = sl.UNIT.MILLIMETER

        # Open the camera
        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            self.zed.close()
            exit(1)
        self.runtime = sl.RuntimeParameters()
        #self.runtime.sensing_mode = sl.SENSING_MODE.STANDARD
        self.image_size = self.zed.get_camera_information().camera_configuration.resolution
        self.image_size.width = self.image_size.width / 2
        self.image_size.height = self.image_size.height / 2

        # Declare your sl.Mat matrices
        self.image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.depth_image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.point_cloud = sl.Mat()

    def init_OneHotEncoder(self):
        self.labels = os.listdir('signs_data_classify')
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

        return images


    def get_image(self):
        if self.is_video:
            for i in range(self.is_delay):
                ret, image = self.cap.read()
                #print(image.shape)
    #            if not self.is_video:
    #                image = image[:,:int(image.shape[1] / 2),:]
            # self.count += 1
            if image is None:
                exit()
            # image = cv2.resize(image, (960, 540), interpolation = cv2.INTER_AREA)
            return image

        err = self.zed.grab(self.runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image, depth image in the half-resolution
            self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
            self.zed.retrieve_image(self.depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, self.image_size)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, self.image_size)
            img = self.image_zed.get_data()

            # img = cv2.flip(img, 0)
            # img = cv2.flip(img, 1)
            return img[:, :, :3]
        else:
            return None

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

    def get_segments_from_image(self, image):
        analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        labels = self.seg_model.predict_one(analyze_image)
        traffic_light = np.ones_like(labels) * 255
        traffic_light[labels != 6] = 0
        traffic_sign = np.ones_like(labels) * 255
        traffic_sign[labels != 7] = 0
        if self.is_red:
            traffic_light += traffic_sign
        if self.is_viz:
            tl = np.stack((traffic_light, traffic_light, traffic_light), axis=-1)
            ts = np.stack((traffic_sign, traffic_sign, traffic_sign), axis=-1)
            # print(traffic_sign.shape)
            composited_traffic_light = cv2.addWeighted(image, 0.5, np.array(tl, dtype=np.uint8), 0.5,
                                                     0)
            composited_traffic_sign = cv2.addWeighted(image, 0.5, np.array(ts, dtype=np.uint8), 0.5,
                                                       0)
            cv2.imshow('composited_traffic_light', composited_traffic_light)
            cv2.imshow('composited_traffic_sign', composited_traffic_sign)
        traffic_light_images = self.cut_image4segments(image, np.array(traffic_light, dtype=np.uint8))
        traffic_sign_images = self.cut_image4segments(image, np.array(traffic_sign, dtype=np.uint8))

        return traffic_light_images, traffic_sign_images

    def get_classify_images(self, images):
        sign = None
        square = 0
        #print(f'squa
        # re: {square}')
        for image in images:
            height, width, channels = image.shape
            local_square = height * width
            if local_square < square:
                continue

            image = cv2.resize(image, (64, 64))
            _, y = self.cnn_model.predict([image])
            y = y[0]
            index_y = np.argmax(y)
            print(self.decode_labels([np.round(y)])[0][0], y[index_y])
            # temp_sign = self.decode_labels([np.round(y)])[0][0]
            if y[index_y] > 0.7 or y[index_y] > 0.5 and '3_24_' in self.decode_labels([np.round(y)])[0][0] :
                square = local_square
                sign = self.decode_labels([np.round(y)])[0][0]

        return sign

    def visualize(self, sign):
        if sign is not None:
            # print(sign)
            image = cv2.resize(cv2.imread(os.path.join('signs_icon', f'{sign}.jpg')), (500, 500))
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
            cv2.imshow('main', image)
            key = cv2.waitKey(10)
            if key == 113:
                break

            traffic_light_images, traffic_sign_images = self.get_segments_from_image(image)
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
                    self.send_start_cmd()
                    #print(f'traffic light detected = {traffic_light}')
            if traffic_sign_images and not self.is_red:
                sign = self.get_classify_images(traffic_sign_images)
                signs_que[:size_que-1] = signs_que[1:]
                signs_que[-1] = sign
            # print(signs_que)
            most_common_item = max(signs_que, key=signs_que.count)
            count_most_common_item = signs_que.count(most_common_item)
            if most_common_item: #and count_most_common_item > 9:
                self.visualize(most_common_item)
        cv2.destroyAllWindows()
        self.zed.close()
if __name__ == '__main__':
    try:
        detector = ImageAnalyzer('weights/saved-model-20-0.99815.hdf5', is_red=True)
        detector.run()
    except Exception as err:
        print(f'{str(err)}')
        print("Shutting down")
    cv2.destroyAllWindows()
