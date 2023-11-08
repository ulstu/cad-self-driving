import numpy as np
from scipy.interpolate import interp1d

def predict_trajectory(points, num_predictions, time_interval):
    """
    Предсказывает траекторию объекта на основе временной последовательности точек.

    :param points: Список точек, где каждая точка представлена как (x, y)
    :param num_predictions: Количество точек для предсказания
    :param time_interval: Интервал времени между предсказаниями
    :return: Список точек с предсказанными положениями объекта
    """
    if len(points) < 2:
        raise ValueError("Для предсказания необходимо минимум 2 точки")

    x_values, y_values = zip(*points)
    x_values = np.array(x_values)
    y_values = np.array(y_values)

    interp_func = interp1d(x_values, y_values, kind='linear', fill_value='extrapolate')

    # Создаем массив времени для предсказаний
    time_values = np.arange(x_values[-1] + time_interval, x_values[-1] + (num_predictions * time_interval), time_interval)

    # Предсказываем положения объекта
    predicted_y_values = interp_func(time_values)

    # Создаем список предсказанных точек
    predicted_points = [(x, y) for x, y in zip(time_values, predicted_y_values)]

    return predicted_points

# Пример использования
input_points = [(0, 0), (1, 1), (2, 0), (3, 1), (4, 0)]
num_predictions = 5
time_interval = 1  # Интервал времени между предсказаниями

predicted_trajectory = predict_trajectory(input_points, num_predictions, time_interval)
print("Предсказанная траектория:", predicted_trajectory)
