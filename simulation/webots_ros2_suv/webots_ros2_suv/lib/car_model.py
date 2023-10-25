import rclpy

class CarModel(object):
    '''
    Класс, моделирующий параметры внешней среды в привязке к локальной карте.
    В случае использования симулятора используются значения из симулятора
    Для реального автомобиля параметры получаются из подсистемы навигации
    '''
    def __init__(self):
        self.__speed = 0
        self.__x = 0
        self.__y = 0
        self.__direction = 0

    def __update(self, speed=None, x=None, y=None, direction=None):
        if speed:
            self.__speed = speed
        if x:
            self.__x = x
        if y:
            self.__y = y
        if direction:
            self.__direction = direction

