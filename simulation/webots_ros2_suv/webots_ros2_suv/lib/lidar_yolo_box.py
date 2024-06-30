import numpy as np


class LidarYoloBox():
    '''
        LidarYoloBox - класс, который хранит информацию о обнаруженном объекте как с 
        поомощью yolo (класс и 2d bounding box), так и информацию с помощью лидара (3d bounding box)  
    '''
    def __init__(self, label_name: str, box: np.ndarray, box3d: np.ndarray, edges: np.ndarray):
        '''
        label_name - строка, содержащая название класса обрнаруженного объекта

        box - numpy массив, содержащий координаты левой верхней и правой нижней точки обнаруженного объекта на изображении
        в следующем формате: [номер точки][координата x или y].  

        box3d - numpy массив, содержащий координаты углов параллепипеда обнаруженного объекта в трёхмерном пространстве
        в следующем формате: [номер точки][координата x, y или z].

        edges - numpy массив, содержащий масимальные и минимальные границы параллепипеда в трёхмерном прространстве
        в следующем формате: [[xmin, xmax], [ymin, ymax], [zmin, zmax]]
        '''
        self.label_name = label_name
        self.box = box
        self.box3d = box3d
        self.edges = edges