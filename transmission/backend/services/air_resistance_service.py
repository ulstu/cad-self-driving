from dataclasses import dataclass, field

from matplotlib import pyplot as plt
from pandas import DataFrame

from utils.graphic_helper import GraphicHelper


@dataclass
class AirResistanceService:
    """Рассчёт габаритных размеров и сопротивления воздуха

       Parameters
            ----------
            width : float
                Ширина профиля колеса (мм)
            height : float
                Высота профиля колеса (в процентах от ширины)
            streamline_coefficient : float
                Коеффициент смятия шины
            speed_car_data_frame : float
                Датасет со скоростями автомобиля относительно кол-ва оборотов двигателя, номера передачи
            frequency_turns_per_min : list
                Список оборотов, для которых нужно расчитать данные
    """
    width: float
    height: float
    streamline_coefficient: float
    speed_car_data_frame: DataFrame
    frequency_turns_per_min: list

    def __post_init__(self):
        self.midelev_cross_sectional_area = self.width * self.height * 0.79
        self.speed_hub1 = self.speed_car_data_frame.iloc[:,
                              1].to_numpy()  # массив кол-ва оборотов колеса на 1 скорости
        self.speed_hub2 = self.speed_car_data_frame.iloc[:,
                              2].to_numpy()  # массив кол-ва оборотов колеса на 2 скорости
        self.speed_hub3 = self.speed_car_data_frame.iloc[:,
                              3].to_numpy()  # массив кол-ва оборотов колеса на 3 скорости
        self.speed_hub4 = self.speed_car_data_frame.iloc[:,
                              4].to_numpy()  # массив кол-ва оборотов колеса на 4 скорости
        self.speed_hub5 = self.speed_car_data_frame.iloc[:, 5].to_numpy()
        self.show_graphic()

    @property
    def air_resistance_hub1(self):
        resistance_list = []
        for speed in self.speed_hub1:
            resistance = 0.5*self.streamline_coefficient*self.midelev_cross_sectional_area*1.22*(
                        (speed/3.6)**2)
            resistance_list.append(resistance)
        return resistance_list
    @property
    def air_resistance_hub2(self):
        resistance_list = []
        for speed in self.speed_hub2:
            resistance = 0.5 * self.streamline_coefficient * self.midelev_cross_sectional_area * 1.22 * (
                        (speed / 3.6) ** 2)
            resistance_list.append(resistance)
        return resistance_list

    @property
    def air_resistance_hub3(self):
        resistance_list = []
        for speed in self.speed_hub3:
            resistance = 0.5 * self.streamline_coefficient * self.midelev_cross_sectional_area * 1.22 * (
                        (speed / 3.6) ** 2)
            resistance_list.append(resistance)
        return resistance_list

    @property
    def air_resistance_hub4(self):
        resistance_list = []
        for speed in self.speed_hub4:
            resistance = 0.5 * self.streamline_coefficient * self.midelev_cross_sectional_area * 1.22 * (
                        (speed / 3.6) ** 2)
            resistance_list.append(resistance)
        return resistance_list

    @property
    def air_resistance_hub5(self):
        resistance_list = []
        for speed in self.speed_hub5:
            resistance = 0.5 * self.streamline_coefficient * self.midelev_cross_sectional_area * 1.22 * (
                    (speed / 3.6) ** 2)
            resistance_list.append(resistance)
        return resistance_list

    def show_graphic(self):
        """Построение графика сопротивления воздуха"""
        plt.clf()
        plt.xlabel("Частота, об/мин")
        plt.title('Сопротивление воздуха от оборотов двигателя')
        plt.plot(self.frequency_turns_per_min, self.air_resistance_hub1, label='1 передача')
        plt.plot(self.frequency_turns_per_min, self.air_resistance_hub2, label='2 передача')
        plt.plot(self.frequency_turns_per_min, self.air_resistance_hub3, label='3 передача')
        plt.plot(self.frequency_turns_per_min, self.air_resistance_hub4, label='4 передача')
        plt.plot(self.frequency_turns_per_min, self.air_resistance_hub5, label='5 передача')
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)
