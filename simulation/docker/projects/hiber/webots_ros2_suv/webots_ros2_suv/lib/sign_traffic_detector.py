import numpy as np


def predcit_labels(image) -> np.ndarray:
    return image


def predict_traffic_lights(image, labels=None):
    '''
        Формат массива найденных квадратов со светофорами:
        boxes[id][v]
        
        id - номер найденного на изображении квадрата
        p1 = [boxes[id][0], boxes[id][1]] - первая точка квадрата
        p2 = [boxes[id][2], boxes[id][3]] - вторая точка квадрата
        state = boxes[id][4] - состояние светофора
    '''
    return np.zeros((3, 5), dtype=np.int32)


def predict_signs(image, labels=None):
    '''
        Формат массива найденных квадратов со знаками:
        boxes[id][v]
        
        id - номер найденного на изображении квадрата
        p1 = [boxes[id][0], boxes[id][1]] - первая точка квадрата
        p2 = [boxes[id][2], boxes[id][3]] - вторая точка квадрата
        cls = boxes[id][4] - номер класса обнаруженного объекта
    '''
    return np.zeros((3, 5), dtype=np.int32)