import torch
import random
import torch.nn as nn
import torch.optim as optim
import math
import os
import numpy as np
from scipy.interpolate import interp1d

class BehaviourAnalyser(object):
    def __init__(self, model_dir="~/"):
        self.input_size = 3   # Размер входных данных (класс объекта, x, y координаты)
        self.hidden_size = 128 # Размер скрытого состояния RNN
        self.output_size = 2  # Размер выходных данных (x, y координаты)
        self.batch_size = 25  # Размер батча для обучения
        self.num_epochs = 20
        self.train_counter = {}

        # Директория для сохранения и загрузки моделей
        self.model_dir = model_dir
        if not os.path.exists(self.model_dir):
            os.makedirs(self.model_dir)

        # Словарь для хранения моделей для разных классов
        self.models = {}

        # Загрузка ранее сохраненных моделей
        self.load_models()

    def run_spline(self, points, point_class):
        if len(points) < 3:
            return None
        dist = math.dist((points[0][0], points[0][1]), (points[-1][0], points[-1][1]))

        if dist > 10:
            predicted_trajectory = self.predict_trajectory(points, 35, 1)
            return predicted_trajectory
        return None

    def run(self, points, point_class, need_train=False):
        data = [[point_class, p[0], p[1]] for p in points]
        if not self.check_distance(data):
            return None
        model = self.models.get(point_class)
        if model is None:
            print(f"Model for class {point_class} does not exist.")
            return None
        
        predicted_trajectory = []
        if not point_class in self.train_counter:
            self.train_counter[point_class] = 0

        if self.check_distance(data) and need_train:
            self.train_counter[point_class] = self.train_counter[point_class] + 1
            if self.train_counter[point_class] % 10 == 0:
                print(f'run train for class {point_class}')
                model.train_lstm_model(data, num_epochs=self.num_epochs)

        predicted_trajectory = model.predict(data)

        return predicted_trajectory

    def predict_trajectory(self, points, num_predictions, time_interval):
        if len(points) < 2:
            raise ValueError("Для предсказания необходимо минимум 2 точки")

        x_values, y_values = zip(*points)
        x_values = np.array(x_values)
        y_values = np.array(y_values)

        interp_func = interp1d(x_values, y_values, kind='linear', fill_value='extrapolate')

        time_values = np.arange(x_values[-1] + time_interval, x_values[-1] + (num_predictions * time_interval), time_interval)

        predicted_y_values = interp_func(time_values)

        predicted_points = [[int(x), int(y)] for x, y in zip(time_values, predicted_y_values) if not math.isinf(x) and not math.isinf(y) and not math.isnan(x) and not math.isnan(y)]

        return predicted_points

    def train(self, class_id, data):
        if class_id not in self.models:
            self.models[class_id] = TrajectoryPredictor(self, self.input_size, self.hidden_size, self.output_size, self.batch_size)
        data = [[class_id, d[0], d[1]] for d in data]

        # Проверяем расстояние между точками
        if self.check_distance(data):
            self.models[class_id].train_lstm_model(data, num_epochs=self.num_epochs)

            model_path = os.path.join(self.model_dir, f"model_{class_id}.pt")
            torch.save(self.models[class_id].model.state_dict(), model_path)
        else:
            print(f"Distance between points for class {class_id} is less than 10, skipping training.")

    def check_distance(self, data):
        if len(data) < 25:
            return False

        dist = math.dist((data[0][1], data[0][2]), (data[-1][1], data[-1][2]))
        return dist > 20


    def load_models(self):
        # Получаем список файлов моделей в директории
        model_files = [f for f in os.listdir(self.model_dir) if f.startswith("model_") and f.endswith(".pt")]

        for model_file in model_files:
            class_id = int(model_file.split("_")[1].split(".")[0])
            model_path = os.path.join(self.model_dir, model_file)

            # Проверяем, существует ли файл модели
            if os.path.isfile(model_path):
                model = TrajectoryPredictor(self, self.input_size, self.hidden_size, self.output_size, self.batch_size)
                model.model.load_state_dict(torch.load(model_path))
                self.models[class_id] = model
            else:
                print(f"Model file for class {class_id} does not exist.")

class TrajectoryPredictionModel(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(TrajectoryPredictionModel, self).__init__()
        self.rnn = nn.LSTM(input_size=input_size, hidden_size=hidden_size, num_layers=3, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        output, _ = self.rnn(x)
        predictions = self.fc(output)
        return predictions

class TrajectoryPredictor:
    def __init__(self, behaviour_analyser, input_size, hidden_size, output_size, batch_size):
        self.behaviour_analyser = behaviour_analyser
        self.model = TrajectoryPredictionModel(input_size, hidden_size, output_size)
        self.criterion = nn.HuberLoss()#nn.MSELoss()
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.001)
        self.batch_size = batch_size

    def train_lstm_model(self, data, num_epochs):
        split_ratio = 0.8
        split_index = int(len(data) * split_ratio)
        train_data = data[:split_index]
        test_data = data[split_index:]

        for epoch in range(num_epochs):
            total_loss = 0

            for i in range(0, len(train_data), self.batch_size):
                batch = train_data[i:i + self.batch_size]
                inputs, targets = self.batch_to_tensors(batch)
                self.optimizer.zero_grad()
                outputs = self.model(inputs)
                loss = self.criterion(outputs, targets)
                loss.backward()
                self.optimizer.step()
                total_loss += loss.item()

            test_loss = self.evaluate(test_data)
            print(f"Epoch {epoch + 1}/{num_epochs},train loss: {total_loss:.4f} Test Loss: {test_loss:.4f}")

    def batch_to_tensors(self, batch):
        input_sequence = []
        target_sequence = []

        for item in batch:
            class_id, x, y = item
            input_sequence.append([class_id, x, y])
            target_sequence.append([x, y])

        inputs = torch.tensor(input_sequence, dtype=torch.float32)
        targets = torch.tensor(target_sequence, dtype=torch.float32)

        return inputs, targets  # Возвращаем тензоры как кортеж

    def evaluate(self, test_data):
        total_loss = 0
        self.model.eval()

        with torch.no_grad():
            for i in range(0, len(test_data), self.batch_size):
                batch = test_data[i:i + self.batch_size]
                inputs, targets = self.batch_to_tensors(batch)
                outputs = self.model(inputs)
                loss = self.criterion(outputs, targets)
                total_loss += loss.item()

        self.model.train()
        return total_loss

    def predict(self, input_sequence):
        with torch.no_grad():
            inputs = torch.tensor(input_sequence, dtype=torch.float32)
            predictions = self.model(inputs)
        return predictions.tolist()