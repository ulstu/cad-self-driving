import pygame
import yaml
import os

class Pattern:
    def __init__(self):
        self.car_size = (2, 6)
        self.meter_on_pixel = 15
        self.data = {"parking": [0, 0], "points": [], "parallel": True}
        self.move_parking = True
        
    def start_editor(self):
        pygame.init()
        pygame.font.init()
        sysfont = pygame.font.SysFont("Arial", 20)

        pygame.display.set_caption('Quick Start')
        window_surface = pygame.display.set_mode((800, 800))

        background = pygame.Surface((800, 800))

        is_running = True

        while is_running:
            background.fill(pygame.Color('#000000'))
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    is_running = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    x, y = self.move_relative(*pygame.mouse.get_pos())
                    if event.button == 1:
                        self.data["points"].append([x, y, 1])
                        print(self.data["points"])
                    elif event.button == 3:
                        self.data["points"].append([x, y, -1])
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_z:
                        self.data["points"].pop()
                    if event.key == pygame.K_p:
                        self.data["parallel"] = not self.data["parallel"]
                    if event.key == pygame.K_m:
                        self.move_parking = not self.move_parking
                    if event.key == pygame.K_s:
                        with open(f"{os.path.expanduser('~')}/ros2_ws/data/output.yaml", "w") as file:
                            doc = yaml.dump([self.data], file, default_flow_style=False)
                    if event.key == pygame.K_LEFT:
                        self.data["parking"][0] -= 0.5
                    if event.key == pygame.K_RIGHT:
                        self.data["parking"][0] += 0.5
                    if event.key == pygame.K_UP:
                        self.data["parking"][1] -= 0.5
                    if event.key == pygame.K_DOWN:
                        self.data["parking"][1] += 0.5


            pygame.draw.rect(background, (255, 0, 0), pygame.Rect(
                *self.move_screen(int(- self.car_size[0] / 2), int(- self.car_size[1] / 2)),
                *(self.car_size[0] * 15, self.car_size[1] * 15)),
                1)
            
            # Отрисовка точек
            for i in range(len(self.data["points"])):
                x, y, d = self.data["points"][i]
                if d == 1:
                    pygame.draw.circle(background, (255, 0, 0), self.move_screen(x, y), 4)
                else:
                    pygame.draw.circle(background, (0, 255, 0), self.move_screen(x, y), 4)
            
            # Отрисовка парковочной зоны
            if self.data["parallel"]:
                pygame.draw.rect(background, (0, 0, 255), pygame.Rect(
                    *self.move_screen(int(self.data["parking"][0] - self.car_size[0] / 2), int(self.data["parking"][1] - self.car_size[1] / 2)),
                    *(self.car_size[0] * 15, self.car_size[1] * 15)), 1)
            else:
                pygame.draw.rect(background, (0, 0, 255), pygame.Rect(
                    *self.move_screen(int(self.data["parking"][0] - self.car_size[1] / 2), int(self.data["parking"][1] - self.car_size[0] / 2)),
                    *(self.car_size[1] * 15, self.car_size[0] * 15)), 1)

            window_surface.blit(background, (0, 0))
            pygame.display.update()
    
    def move_screen(self, x, y):
        return [int(400 + x * self.meter_on_pixel), int(400 + y * self.meter_on_pixel)]
    
    def move_relative(self, x, y):
        return (x - 400) / self.meter_on_pixel, (y - 400) / self.meter_on_pixel

if __name__ == "__main__":
    pattern = Pattern()
    pattern.start_editor()
