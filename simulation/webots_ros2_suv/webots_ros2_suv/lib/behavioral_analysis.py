import torch
import random
import torch.nn as nn
import torch.optim as optim
from torchsummary import summary
import math
import numpy as np
from scipy.interpolate import interp1d


class BehaviourAnalyser(object):
    def __init__(self):
        self.input_size = 3    # Размер входных данных (класс объекта, x, y координаты)
        self.hidden_size = 64  # Размер скрытого состояния RNN
        self.output_size = 2   # Размер выходных данных (x, y координаты)
        self.num_classes = 81   # Количество классов объектов
        self.batch_size = 32   # Размер батча для обучения
        self.num_points_per_class = 40  # Количество точек данных для каждого класса

        self.predictor = TrajectoryPredictor(self.input_size, self.hidden_size, self.output_size, self.batch_size)
        self.data_loader = DataLoader(self.batch_size)
        self.data = self.data_loader.generate_sample_data(self.num_classes, self.num_points_per_class)
        self.num_epochs = 20
        self.train_counter = {}

    def run_spline(self, points, point_class):
        if len(points) < 3:
            return None
        dist = math.dist((points[0][0], points[0][1]), (points[-1][0], points[-1][1]))

        if dist > 10:
            predicted_trajectory = self.predict_trajectory(points, 30, 1)
            return predicted_trajectory
        return None

    def run(self, points, point_class):

        data = [[point_class, p[0], p[1]] for p in points]
        predicted_trajectory = []
        if not point_class in self.train_counter:
            self.train_counter[point_class] = 0

        dist = math.dist((points[0][0], points[0][1]), (points[-1][0], points[-1][1]))

        if dist > 10:
            self.train_counter[point_class] = self.train_counter[point_class] + 1
            if self.train_counter[point_class] % 10 == 0:
                print(f'run train for class {point_class}')
                for inputs, targets in self.data_loader.get_batch(data):
                    self.predictor.train(inputs, targets, num_epochs=1)

        for epoch in range(self.num_epochs):
            pred = self.predictor.predict(data)
            if int(pred[0][0]) == 0 and int(pred[0][1]) == 0:
                return None
            predicted_trajectory.append([int(pred[0][0]), int(pred[0][1])])
            d = data.pop()
            data.insert(0, [point_class, pred[0][0], pred[0][1]])

        print(f'Pred: {predicted_trajectory[:5]}')
        return predicted_trajectory

    def predict_trajectory(self, points, num_predictions, time_interval):
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
        predicted_points = [[int(x), int(y)] for x, y in zip(time_values, predicted_y_values) if not math.isinf(x) and not math.isinf(y) and not math.isnan(x) and not math.isnan(y)]

        return predicted_points


class TrajectoryPredictionModel(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(TrajectoryPredictionModel, self).__init__()
        self.rnn = nn.LSTM(input_size=input_size, hidden_size=hidden_size, num_layers=2, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        output, _ = self.rnn(x)
        predictions = self.fc(output)
        return predictions

class DataLoader:
    def __init__(self, batch_size):
        self.batch_size = batch_size

    def generate_sample_data(self, num_classes, num_points_per_class):
        data = []
        for class_id in range(num_classes):
            for _ in range(num_points_per_class):
                x = random.uniform(0, 100)
                y = random.uniform(0, 100)
                data.append((class_id, x, y))
        return data

    def get_batch(self, data):
        num_batches = len(data) // self.batch_size
        for i in range(num_batches):
            batch_data = data[i * self.batch_size: (i + 1) * self.batch_size]
            input_sequence = []
            output_sequence = []
            center_idx = int(len(batch_data) / 2) - 1
            for item in batch_data[:center_idx]:
                class_id, x, y = item
                input_sequence.append([class_id, x, y])
            for item in batch_data[center_idx: 2 * center_idx]:
                class_id, x, y = item
                output_sequence.append([x, y])
            yield input_sequence, output_sequence

class TrajectoryPredictor:
    def __init__(self, input_size, hidden_size, output_size, batch_size):
        self.model = TrajectoryPredictionModel(input_size, hidden_size, output_size)
        self.criterion = nn.MSELoss()
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.001)
        self.data_loader = DataLoader(batch_size)

    def train(self, input_sequence, target_sequence, num_epochs):
        for epoch in range(num_epochs):
            self.optimizer.zero_grad()
            inputs = torch.tensor(input_sequence, dtype=torch.float32)
            targets = torch.tensor(target_sequence, dtype=torch.float32)
            outputs = self.model(inputs)
            loss = self.criterion(outputs, targets)
            loss.backward()
            self.optimizer.step()

    def predict(self, input_sequence):
        with torch.no_grad():
            inputs = torch.tensor(input_sequence, dtype=torch.float32)
            predictions = self.model(inputs)
        return predictions.tolist()
    
