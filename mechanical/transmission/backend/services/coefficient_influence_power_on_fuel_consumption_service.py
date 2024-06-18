from dataclasses import dataclass

import matplotlib.pyplot as plt
from pandas import DataFrame

from utils.graphic_helper import GraphicHelper


@dataclass
class CoefficientInfluencePowerOnFuelConsumptionService:
    """ Сервис по расчёту коэффициентов влияния мощности на расход топлива

        Parameters
            ----------
            torque_on_wheel_dataset : DataFrame
                Датасет крутящего момента на колесе
            air_resistance_dataset : DataFrame
                Датасет сопротивления воздуха
    """
    torque_on_wheel_dataset: DataFrame
    air_resistance_dataset: DataFrame

    def __post_init__(self):
        self.frequency_array = self.torque_on_wheel_dataset['Частота оборотов двигателя'].to_numpy()
        self.torque_on_wheel_hub1 = self.torque_on_wheel_dataset['Передача 1'].to_numpy()
        self.torque_on_wheel_hub2 = self.torque_on_wheel_dataset['Передача 2'].to_numpy()
        self.torque_on_wheel_hub3 = self.torque_on_wheel_dataset['Передача 3'].to_numpy()
        self.torque_on_wheel_hub4 = self.torque_on_wheel_dataset['Передача 4'].to_numpy()
        self.torque_on_wheel_hub5 = self.torque_on_wheel_dataset['Передача 5'].to_numpy()
        self.air_resistance_hub1 = self.air_resistance_dataset['Передача 1'].to_numpy()
        self.air_resistance_hub2 = self.air_resistance_dataset['Передача 2'].to_numpy()
        self.air_resistance_hub3 = self.air_resistance_dataset['Передача 3'].to_numpy()
        self.air_resistance_hub4 = self.air_resistance_dataset['Передача 4'].to_numpy()
        self.air_resistance_hub5 = self.air_resistance_dataset['Передача 5'].to_numpy()
        self.show_graphic()

    @property
    def coefs_hub1(self):
        return self.__calculate_coefs_hub(self.torque_on_wheel_hub1, self.air_resistance_hub1)

    @property
    def coefs_hub2(self):
        return self.__calculate_coefs_hub(self.torque_on_wheel_hub2, self.air_resistance_hub2)

    @property
    def coefs_hub3(self):
        return self.__calculate_coefs_hub(self.torque_on_wheel_hub3, self.air_resistance_hub3)

    @property
    def coefs_hub4(self):
        return self.__calculate_coefs_hub(self.torque_on_wheel_hub4, self.air_resistance_hub4)

    @property
    def coefs_hub5(self):
        return self.__calculate_coefs_hub(self.torque_on_wheel_hub5, self.air_resistance_hub5)

    def __calculate_coefs_hub(self, torque_on_wheel_hub, air_resistance_hub):
        """
        :param torque_on_wheel_hub: список крутящих моментов при определённых оборотов
        :param air_resistance_hub: список сопротивления воздуха при определённых оборотах
        :return: список коэффициентов для конкретной передачи передачи
        """
        coefs = []
        for torque, air_resistance in zip(torque_on_wheel_hub, air_resistance_hub):
            coef = 3.27 - 8.22 * (air_resistance / torque) + 9.13 * ((air_resistance / torque) ** 2) - 3.18 * (
                    (air_resistance / torque) ** 3)
            coefs.append(coef)
        return coefs

    def show_graphic(self):
        """График коэффициента влияния мощности на расход топлива"""
        plt.clf()
        plt.xlabel("Частота, об/мин")
        plt.title('Коэффициент влияния мощности на расход топлива')
        plt.plot(self.frequency_array, self.coefs_hub1, label='1 Передача')
        plt.plot(self.frequency_array, self.coefs_hub2, label='2 Передача')
        plt.plot(self.frequency_array, self.coefs_hub3, label='3 Передача')
        plt.plot(self.frequency_array, self.coefs_hub4, label='4 Передача')
        plt.plot(self.frequency_array, self.coefs_hub5, label='5 Передача')
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)
