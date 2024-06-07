import cv2
import random
import time
import numpy as np
from sklearn.preprocessing import OneHotEncoder
import os
import sys
import albumentations as A
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

# list_signs = ['1_1', '1_6', '1_8', '1_22', '1_31', '1_33', '2_1', '2_2', '2_3', '2_4', '2_5', '3_1', '3_18', '3_20',
#               '3_21', '3_24', '3_25', '3_27', '3_28', '3_31', '4_1_1', '4_3',
#               '5_5', '5_6', '5_16', '5_19_1', '5_20', '6_4', '7_3', '7_4']
# a = np.array([i for i in range(30)]).reshape(-1, 1)
# enc = OneHotEncoder(handle_unknown='ignore')
# enc.fit(a)

class DatasetTrain():
    def __init__(self, path_data, is_train=False):
        self.list_signs = os.listdir(path_data)
        self.enc = OneHotEncoder(handle_unknown='ignore')

        self.enc.fit(np.array(self.list_signs).reshape(-1, 1))
        self.path_data = path_data
        self.data_x = []
        self.data_y = []
        self.last_data_index = 0
        self.class_count = 1
        self.image_size = [64, 64, 3]
        self.seed = 100
        self.filenames = []
        self.boxes = []
        self.is_load = 0
        self.size_data = 1000
        self.current_start = 0
        self.is_train = is_train
        self.transform = A.Compose([
                                # A.HorizontalFlip(p=0.5),
                                A.ShiftScaleRotate(rotate_limit= 20,  p=0.5),
                                A.RandomBrightnessContrast(p=0.3),
                                A.RGBShift(r_shift_limit=30, g_shift_limit=30, b_shift_limit=30, p=0.3),
                                A.Blur(p=0.5),
                            ]
                        )

    def load(self, data_limit=200, transofrm=True, is_train=True, coef=0.9):
        for sign in self.list_signs:
            path2signs = os.path.join(self.path_data, sign)
            count = 0
            files = [cv2.resize(cv2.imread(os.path.join(path2signs, sign_path)), (self.image_size[0], self.image_size[1]))
                     for sign_path in os.listdir(path2signs)]
            if is_train:
                files = files[:int(coef*len(files))]
            else:
                files = files[int(coef*len(files)):]
            while count < data_limit:
                if transofrm:
                    self.data_x += [self.transform(image=files[count%len(files)])['image']]
                else:
                    self.data_x += [files[count%len(files)]]
                self.data_y += [sign]
                count += 1

        random.seed(10)
        # self.data_y = list(self.data_y)
        random.shuffle(self.data_y)
        print(self.data_y)
        random.seed(10)
        random.shuffle(self.data_x)
        self.data_y = np.array(self.data_y).reshape(-1, 1)
        self.data_y = self.enc.transform(self.data_y).toarray()
        print(f'количество изображений в загруженной выборке = {len(self.data_x)}')

    def next_batch(self, batch_size):
        start = self.last_data_index
        stop = start + batch_size
        if stop >= len(self.data_x):
            print('Данные закончились. Обновление данных.')
            start = 0
            stop = start + batch_size
            self.last_data_index = 0

        batch_x = self.data_x[start: stop]
        batch_y = self.data_y[start: stop]
        # batch_y = enc.transform(np.array(batch_y).reshape(-1, 1)).toarray()
        self.last_data_index = stop
        return batch_x, batch_y
    
    def get_num_classes(self):
        return self.data_y[0].shape[0]




