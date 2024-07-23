import pygame
import random
import math
import numpy as np
from functools import wraps
import time

def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        message = f'Function {func.__name__} took {total_time:.4f} seconds'
        print(message)
        return result
    return timeit_wrapper

# Инициализация Pygame
pygame.init()

# Константы
WIDTH, HEIGHT = 800, 600
BACKGROUND_COLOR = (255, 255, 255)
STATIC_OBSTACLE_COLOR = (255, 0, 0)
DYNAMIC_OBSTACLE_COLOR = (0, 255, 0)
CAR_COLOR = (0, 0, 255)
TEXT_COLOR = (0, 0, 0)
FPS = 60
CAR_SPEED = 0.1  # Скорость автомобиля
OBSTACLE_SPEED = 0.5  # Скорость динамических препятствий
PREDICTION_INTERVAL = 10  # Интервал предсказания в секундах
EGO_WIDTH = 2  # Ширина эго-автомобиля в метрах
EGO_HEIGHT = 4  # Длина эго-автомобиля в метрах

# Инициализация окна
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Global Map Visualization")

# Текущая позиция автомобиля (широта, долгота, курс в радианах)
car_pos = [0, 0, 0]  # lat, lon, angle
camera_offset = [0, 0]  # Смещение камеры для прокрутки сцены
zoom_level = 1.0  # Масштабирование сцены
dragging = False  # Флаг перетаскивания
last_mouse_pos = [0, 0]  # Последняя позиция мыши
tracked_obstacles = {}  # Словарь для отслеживания препятствий
font = pygame.font.SysFont(None, 24)  # Шрифт для отображения текста

# Калмановский фильтр
class KalmanFilter:
    @timeit
    def __init__(self, dt, u_x, u_y, std_acc, std_meas):
        # Инициализация матриц
        self.dt = dt
        self.u = np.matrix([[u_x], [u_y]])
        self.A = np.matrix([[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.B = np.matrix([[(self.dt**2)/2, 0], [0, (self.dt**2)/2], [self.dt, 0], [0, self.dt]])
        self.H = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0], [0, (self.dt**4)/4, 0, (self.dt**3)/2], [(self.dt**3)/2, 0, self.dt**2, 0], [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2
        self.R = np.matrix([[std_meas**2, 0], [0, std_meas**2]])
        self.P = np.eye(self.A.shape[1])
        self.x = np.matrix([[0], [0], [0], [0]])

    def predict(self):
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))
        I = np.eye(self.H.shape[1])
        self.P = (I - np.dot(K, self.H)) * self.P
        return self.x

@timeit
def generate_test_data():
    # Генерация случайных статических препятствий
    static_obstacles = []
    for _ in range(60):  
        lat = random.uniform(-100, 100)
        lon = random.uniform(-100, 100)
        width = random.uniform(1, 3)
        depth = random.uniform(1, 3)
        static_obstacles.append([lat, lon, width, depth])

    # Генерация случайных динамических препятствий
    dynamic_obstacles = []
    for _ in range(40):  
        obstacle_id = _  # Уникальный идентификатор для каждого препятствия
        lat = random.uniform(-100, 100)
        lon = random.uniform(-100, 100)
        width = random.uniform(1, 3)
        depth = random.uniform(1, 3)
        angle = random.uniform(0, 2 * math.pi)  # Случайный угол движения
        dynamic_obstacles.append([obstacle_id, lat, lon, width, depth, angle])

    return static_obstacles, dynamic_obstacles

@timeit
def update_map(static_obstacles, dynamic_obstacles):
    global tracked_obstacles
    screen.fill(BACKGROUND_COLOR)
    # Обновление положения динамических препятствий
    for obstacle in dynamic_obstacles:
        obstacle_id, lat, lon, width, depth, angle = obstacle
        obstacle[1] += OBSTACLE_SPEED * math.cos(angle)  # Обновление lat
        obstacle[2] += OBSTACLE_SPEED * math.sin(angle)  # Обновление lon
        if obstacle_id not in tracked_obstacles:
            tracked_obstacles[obstacle_id] = {
                'kf': KalmanFilter(1/FPS, OBSTACLE_SPEED * math.cos(angle), OBSTACLE_SPEED * math.sin(angle), 0.1, 0.1),
                'pos': [lat, lon],
                'width': width,
                'depth': depth,
                'angle': angle
            }
        tracked_obstacles[obstacle_id]['pos'] = [obstacle[1], obstacle[2]]
        tracked_obstacles[obstacle_id]['kf'].update(np.matrix([[obstacle[1]], [obstacle[2]]]))

    # Отображение статических препятствий
    for obstacle in static_obstacles:
        draw_obstacle(obstacle, STATIC_OBSTACLE_COLOR)
        collision_prob = estimate_collision_probability_static(obstacle, car_pos)
        display_text(collision_prob, obstacle[:2])

    # Отображение динамических препятствий
    for obstacle_id, data in tracked_obstacles.items():
        if is_visible(data['pos']):
            draw_obstacle(data['pos'] + [data['width'], data['depth']], DYNAMIC_OBSTACLE_COLOR)
            predict_pos = predict_movement(data['kf'])
            collision_prob = estimate_collision_probability(predict_pos, data['width'], data['depth'], data['angle'], car_pos)
            display_text(collision_prob, data['pos'])

    # Отображение автомобиля
    draw_car()
    pygame.display.flip()

def draw_obstacle(obstacle, color):
    lat, lon, width, depth = obstacle[:4]
    x = int((WIDTH / 2 + (lon - car_pos[1]) * 10 - camera_offset[0]) * zoom_level)
    y = int((HEIGHT / 2 - (lat - car_pos[0]) * 10 - camera_offset[1]) * zoom_level)
    pygame.draw.rect(screen, color, (x, y, width * 10 * zoom_level, depth * 10 * zoom_level))

def predict_movement(kf):
    predictions = []
    for _ in range(PREDICTION_INTERVAL * FPS):
        predict_pos = kf.predict()
        predictions.append((predict_pos[0, 0], predict_pos[1, 0]))
    return predictions

@timeit
def estimate_collision_probability(predictions, obs_width, obs_depth, obs_angle, car_pos):
    car_future_positions = []
    for t in range(PREDICTION_INTERVAL * FPS):
        future_lat = car_pos[0] + CAR_SPEED * t / FPS * math.cos(car_pos[2])
        future_lon = car_pos[1] + CAR_SPEED * t / FPS * math.sin(car_pos[2])
        car_future_positions.append((future_lat, future_lon))
    
    collision_count = 0
    for car_fut_pos, obs_pos in zip(car_future_positions, predictions):
        if is_collision(car_fut_pos[0], car_fut_pos[1], obs_pos[0], obs_pos[1], obs_width, obs_depth):
            collision_count += 1
    
    if collision_count == len(car_future_positions):
        return 1.0
    return collision_count / len(car_future_positions)

@timeit
def estimate_collision_probability_static(obstacle, car_pos):
    lat, lon, width, depth = obstacle
    predictions = [(lat, lon)] * (PREDICTION_INTERVAL * FPS)
    return estimate_collision_probability(predictions, width, depth, 0, car_pos)

def is_collision(car_lat, car_lon, obs_lat, obs_lon, obs_width, obs_depth):
    car_half_width = EGO_WIDTH / 2
    car_half_height = EGO_HEIGHT / 2
    obs_half_width = obs_width / 2
    obs_half_height = obs_depth / 2

    car_min_x = car_lon - car_half_width
    car_max_x = car_lon + car_half_width
    car_min_y = car_lat - car_half_height
    car_max_y = car_lat + car_half_height

    obs_min_x = obs_lon - obs_half_width
    obs_max_x = obs_lon + obs_half_width
    obs_min_y = obs_lat - obs_half_height
    obs_max_y = obs_lat + obs_half_height

    return (car_min_x < obs_max_x and car_max_x > obs_min_x and
            car_min_y < obs_max_y and car_max_y > obs_min_y)

def display_text(probability, position):
    text = font.render(f'{probability:.2f}', True, TEXT_COLOR)
    x = int((WIDTH / 2 + (position[1] - car_pos[1]) * 10 - camera_offset[0]) * zoom_level)
    y = int((HEIGHT / 2 - (position[0] - car_pos[0]) * 10 - camera_offset[1]) * zoom_level)
    screen.blit(text, (x, y))

def draw_car():
    lat, lon, angle = car_pos
    x = int((WIDTH / 2 - camera_offset[0]) * zoom_level)
    y = int((HEIGHT / 2 - camera_offset[1]) * zoom_level)
    car_rect = pygame.Rect(x - EGO_WIDTH * 5 * zoom_level, y - EGO_HEIGHT * 5 * zoom_level, EGO_WIDTH * 10 * zoom_level, EGO_HEIGHT * 10 * zoom_level)
    pygame.draw.rect(screen, CAR_COLOR, car_rect)
    pygame.draw.line(screen, CAR_COLOR, (x, y), (x + int(10 * math.cos(angle) * zoom_level), y - int(10 * math.sin(angle) * zoom_level)), 2)

def is_visible(position):
    x = (position[1] - car_pos[1]) * 10
    y = (position[0] - car_pos[0]) * 10
    return -WIDTH / 2 / zoom_level < x < WIDTH / 2 / zoom_level and -HEIGHT / 2 / zoom_level < y < HEIGHT / 2 / zoom_level

def main():
    global dragging, last_mouse_pos, camera_offset, zoom_level, car_pos
    clock = pygame.time.Clock()
    static_obstacles, dynamic_obstacles = generate_test_data()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # ЛКМ для перетаскивания
                    dragging = True
                    last_mouse_pos = list(event.pos)
                elif event.button == 4:  # Прокрутка вверх для увеличения масштаба
                    zoom_level = min(zoom_level * 1.1, 5.0)
                elif event.button == 5:  # Прокрутка вниз для уменьшения масштаба
                    zoom_level = max(zoom_level * 0.9, 0.5)
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:  # Остановка перетаскивания
                    dragging = False
            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    mouse_x, mouse_y = event.pos
                    dx = mouse_x - last_mouse_pos[0]
                    dy = mouse_y - last_mouse_pos[1]
                    camera_offset[0] -= dx / zoom_level
                    camera_offset[1] -= dy / zoom_level
                    last_mouse_pos = [mouse_x, mouse_y]

        # Обновление позиции автомобиля (движение вперед)
        car_pos[0] += CAR_SPEED * math.cos(car_pos[2])  # Обновление широты
        car_pos[1] += CAR_SPEED * math.sin(car_pos[2])  # Обновление долготы

        update_map(static_obstacles, dynamic_obstacles)
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()
