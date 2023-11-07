import torch
import random
import torch.nn as nn
import torch.optim as optim

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
        random.shuffle(data)
        num_batches = len(data) // self.batch_size
        for i in range(num_batches):
            batch_data = data[i * self.batch_size: (i + 1) * self.batch_size]
            input_sequence = []
            output_sequence = []
            for item in batch_data:
                class_id, x, y = item
                input_sequence.append([class_id, x, y])
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

if __name__ == "__main__":
    input_size = 3  # Размер входных данных (класс объекта, x, y координаты)
    hidden_size = 64  # Размер скрытого состояния RNN
    output_size = 2  # Размер выходных данных (x, y координаты)
    num_classes = 7  # Количество классов объектов
    batch_size = 32  # Размер батча для обучения
    num_points_per_class = 40  # Количество точек данных для каждого класса

    predictor = TrajectoryPredictor(input_size, hidden_size, output_size, batch_size)
    data_loader = DataLoader(batch_size)
    data = data_loader.generate_sample_data(num_classes, num_points_per_class)

    num_epochs = 10
    for epoch in range(num_epochs):
        for inputs, targets in data_loader.get_batch(data):
            predictor.train(inputs, targets, num_epochs=1)

        # Пример использования модели для предсказания траектории
        class_id = random.randint(0, num_classes - 1)
        x_coord = random.uniform(0, 100)
        y_coord = random.uniform(0, 100)
        input_sequence = [[class_id, x_coord, y_coord]]
        predicted_trajectory = predictor.predict(input_sequence)
        print(f"Epoch {epoch + 1}: Predicted Trajectory: {predicted_trajectory}")

    print("Training completed.")
