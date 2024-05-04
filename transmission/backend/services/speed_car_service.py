from dataclasses import dataclass
from pandas import DataFrame
import matplotlib.pyplot as plt

from utils.graphic_helper import GraphicHelper


@dataclass
class SpeedCarService:
    """Рассчёт скорости от оборотов двигателя км/ч

        Parameters
            ----------
            profile_width : float
                Ширина колеса
            profile_height : float
                Высота колеса
            diameter : float
               Диаметр колеса
            turnovers_wheel_data_frame : DataFrame
                Обороты колеса
            frequency_turns_per_min : list
               Список оборотов, для которых нужно расчитать данные
    """
    profile_width: float
    profile_height: float
    diameter: float
    turnovers_wheel_data_frame: DataFrame
    frequency_turns_per_min: list

    def __post_init__(self):
        self.nominal_radius = 0.0254 * (self.diameter / 2) + (self.profile_width / 1000) * (self.profile_height / 100)
        self.static_radius = self.nominal_radius * self.__tire_crumpling_ratio()
        self.dynamic_radius = self.nominal_radius - ((self.nominal_radius - self.static_radius) / 3)
        self.turnovers_hub1 = self.turnovers_wheel_data_frame.iloc[:,
                              1].to_numpy()  # массив кол-ва оборотов колеса на 1 скорости
        self.turnovers_hub2 = self.turnovers_wheel_data_frame.iloc[:,
                              2].to_numpy()  # массив кол-ва оборотов колеса на 2 скорости
        self.turnovers_hub3 = self.turnovers_wheel_data_frame.iloc[:,
                              3].to_numpy()  # массив кол-ва оборотов колеса на 3 скорости
        self.turnovers_hub4 = self.turnovers_wheel_data_frame.iloc[:,
                              4].to_numpy()  # массив кол-ва оборотов колеса на 4 скорости
        self.turnovers_hub5 = self.turnovers_wheel_data_frame.iloc[:,
                              5].to_numpy()  # массив кол-ва оборотов колеса на 5 скорости
        self.show_graphic()

    @property
    def speed_hub1(self) -> list:
        """Массив скорости для 1 передачи"""
        return self.__calculate_speed_array(self.turnovers_hub1)

    @property
    def speed_hub2(self) -> list:
        """Массив скорости для 2 передачи"""
        return self.__calculate_speed_array(self.turnovers_hub2)

    @property
    def speed_hub3(self) -> list:
        """Массив скорости для 3 передачи"""
        return self.__calculate_speed_array(self.turnovers_hub3)

    @property
    def speed_hub4(self) -> list:
        """Массив скорости для 4 передачи"""
        return self.__calculate_speed_array(self.turnovers_hub4)

    @property
    def speed_hub5(self) -> list:
        """Массив скорости для 5 передачи"""
        return self.__calculate_speed_array(self.turnovers_hub5)

    def __tire_crumpling_ratio(self) -> float:
        """
            Возвращает коефициент смятия шин относительно высоты профиля колеса
        """
        if self.profile_height >= 90:
            return 0.8
        elif self.profile_height <= 50:
            return 0.85
        else:
            return 0.814285714

    def __calculate_speed_array(self, turnovers_hub) -> list:
        """
        Возвращает массив скорости автомобиля, относительно парамтеров колёс, кол-ва оборотов двигателя и кол-ва оборотов колеса для определённого номера скорости
        :param turnoverse_hub: массив оборотов колеса в минуту конкретной скорости
        :return:
        """
        speed_array = []
        for turns_while in turnovers_hub:
            speed = (self.dynamic_radius * 2 * 3.1415926534) * turns_while / 1000 * 60
            speed_array.append(speed)
        return speed_array

    def show_graphic(self):
        """Построение графика скорости км/ч от оборотов двигателя"""
        plt.clf()
        plt.ylabel("Скорость км/ч")
        plt.xlabel("Частота, об/мин")
        plt.title('Скорость км/ч от оборотов двигетеля')
        plt.plot(self.frequency_turns_per_min, self.speed_hub1, label='1 передача')
        plt.plot(self.frequency_turns_per_min, self.speed_hub2, label='2 передача')
        plt.plot(self.frequency_turns_per_min, self.speed_hub3, label='3 передача')
        plt.plot(self.frequency_turns_per_min, self.speed_hub4, label='4 передача')
        plt.plot(self.frequency_turns_per_min, self.speed_hub5, label='5 передача')
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)
