import math
import numpy as np
import cv2


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

# Функция для преобразования координат
def local_to_global(x1, y1, x, y, a_rad):
    # Матрица поворота
    rotation_matrix = np.array([[np.cos(a_rad), -np.sin(a_rad)],
                                [np.sin(a_rad), np.cos(a_rad)]])
    
    # Локальные координаты в виде вектора
    local_coordinates = np.array([x1, y1])
    
    # Применение матрицы поворота
    rotated_coordinates = rotation_matrix.dot(local_coordinates)
    
    # Добавление глобальных координат автомобиля
    global_coordinates = rotated_coordinates + np.array([x, y])
    
    return global_coordinates

def draw_absolute_tracks(tracks, width, height, logger):
    image = np.zeros((height, width, 3), dtype=np.uint8)

    # Итерируем по словарю объектов и рисуем линии
    for obj_id, coordinates in tracks.items():
        #logger.info(f'object: {obj_id} {coordinates}')
        # Рисуем линии между точками
        for i in range(len(coordinates) - 1):
            cv2.line(image, (int(coordinates[i][0]), int(coordinates[i][1])), (int(coordinates[i + 1][0]), int(coordinates[i + 1][1])), (255, 255, 255), 2)

        # Добавляем текст с идентификатором объекта
        # Выбираем первую координату для размещения текста
        text_position = (int(coordinates[0][0]), int(coordinates[0][1]))
        cv2.putText(image, str(obj_id), text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    return image

