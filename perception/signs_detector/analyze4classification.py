import csv
import os
import numpy as np
import random
import cv2
import albumentations as A
from dataset_cnn import DatasetTrain
from cnn import CNN
from sklearn.preprocessing import OneHotEncoder
import warnings

warnings.filterwarnings("ignore", category=DeprecationWarning)



signs = {}
# list_signs = ['1_1', '1_6', '1_8', '1_22', '1_31', '1_33', '2_1', '2_2', '2_3', '2_4', '2_5', '3_1', '3_18', '3_20',
#               '3_21', '3_24', '3_25', '3_27', '3_28', '3_31', '4_1_1', '4_3',
#               '5_5', '5_6', '5_16', '5_19_1', '5_20', '6_4', '7_3', '7_4']
# list_signs = ['7_4', '5_5', '6_3_2', '5_19_1', '2_1', '6_4', '1_22', '2_4', '1_31', '1_1', '3_18',
#               '3_1', '3_24_n60', '4_1_1', '3_24_n90', '3_24_n30', '3_24_n40', '3_28']
list_signs = os.listdir('signs_data_classify')

# a = '7_3'
# print(a in list_signs)
# exit()
lost_signs = ['6_3_2', '3_22', '3_23']
#3_18_1
with open("full-gt.csv", "r") as f:
    reader = csv.reader(f, delimiter=",")
    for i, line in enumerate(reader):
        if i == 0:
            continue
        filename, x_from, y_from, width, height, sign_class, sign_id = line
        x_from, y_from, width, height = int(x_from), int(y_from), int(width), int(height)


        # if '3_24' in sign_class:
        #     if '3_24' in signs:
        #         signs['3_24'] += [[filename, x_from, y_from, width, height]]
        #     else:
        #         signs['3_24'] = [[filename, x_from, y_from, width, height]]
        # elif '3_25' in sign_class:
        #     if '3_25' in signs:
        #         signs['3_25'] += [[filename, x_from, y_from, width, height]]
        #     else:
        #         signs['3_25'] = [[filename, x_from, y_from, width, height]]

        if sign_class in list_signs:

            if sign_class in signs:
                signs[sign_class] += [[filename, x_from, y_from, width, height]]
            else:
                signs[sign_class] = [[filename, x_from, y_from, width, height]]

signs = dict(sorted(signs.items(), key=lambda item: item[1]))
signs = dict(sorted(signs.items()))

for i, sign in enumerate(list_signs):
    if sign in signs:
        print(f'{i}. count of {sign} = {len(signs[sign])}')
        # random.shuffle(signs[sign])
        # for i, data in enumerate(signs[sign]):
        #     if sign == '3_18':
        #         sign = '3_18_1'
        #     filename, x_from, y_from, width, height = data
        #     img = cv2.imread(os.path.join('rtsd-frames', filename))[y_from:y_from+height, x_from:x_from+width, :]
        #     cv2.imwrite(os.path.join(sign, f'{i}.jpg'), img)
        #     if i > 2000:
        #         break
    else:
        print(f'{i}. count of {sign} = 0')
# random.shuffle(signs['5_19_1'])
# for data in signs['5_19_1']:
#     filename, x_from, y_from, width, height = data
#     image = cv2.imread(os.path.join('rtsd-frames', filename))[y_from:y_from+height, x_from:x_from+width, :]
#     cv2.imshow('image', image)
#     cv2.waitKey(0)

# for sign in signs:
#     print(f'count of {sign} = {len(signs[sign])}')
# print(signs)
# exit()
for sign in signs:
    if len(signs[sign]) > 200:
        signs[sign] = random.choices(signs[sign], k=200)
transform = A.Compose([
                                # A.HorizontalFlip(p=0.5),
                                A.ShiftScaleRotate(rotate_limit= 20,  p=0.5),
                                A.RandomBrightnessContrast(p=0.3),
                                A.RGBShift(r_shift_limit=30, g_shift_limit=30, b_shift_limit=30, p=0.3),
                                A.Blur(p=0.5),
                            ])

train_dataset = DatasetTrain('signs_data_classify')
train_dataset.load(data_limit=400)
valid_dataset = DatasetTrain('signs_data_classify')
valid_dataset.load(data_limit=30, transofrm=False, is_train=False)
model = CNN(load=False)
model.init_model()
# model.load_model('weights/12_0_0.9800000190734863/model.ckpt')
# files = os.listdir('rtsd-frames')
# for file in files:
#     path = os.path.join('rtsd-frames', file)
#     image = cv2.resize(cv2.imread(path)[200:400, 500:700, :], (64, 64))
#     logits, y = model.predict([image])
#     print(logits)
#     print(y)
#     print(f'argmax={np.argmax(y)}')
#     y = np.round(y[0])
#     print(y)
#     print(valid_dataset.enc.inverse_transform([y])[0][0])
#     print('----------')
#     cv2.imshow('image', image)
#     cv2.waitKey(0)

# for image, label in zip(valid_dataset.data_x, valid_dataset.data_y):
#     label = valid_dataset.enc.inverse_transform([label])[0][0]
#     logits, y = model.predict([image])
#
#
#     print(label)
#     print(logits)
#     print(y)
#
#     y = np.round(y[0])
#     print(y)
#     print(valid_dataset.enc.inverse_transform([y])[0][0])
#     print('----------')
#     cv2.imshow('image', image)
#     cv2.waitKey(0)
model.train(train_dataset, valid_dataset, valid_dataset, valid_dataset)

# print(len(signs))