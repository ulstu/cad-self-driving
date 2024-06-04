from dataclasses import dataclass

import numpy
from matplotlib import pyplot as plt
from pandas import DataFrame

from utils.graphic_helper import GraphicHelper


@dataclass
class FuelConsumptionService:
    """Сервис по расчёту расхода топлива автомобиля в час"""
    frequency_turns_per_min: numpy.array
    coefficient_turnovers_to_fuel_dataset: DataFrame
    coefficient_influence_power_on_fuel_consumption_dataset: DataFrame
    total_force_resistance_movement_dataset: DataFrame

    def __post_init__(self):
        self.coefs_turnovers_to_fuel = self.coefficient_turnovers_to_fuel_dataset['Коэффициенты']
        self.coefficient_influence_power_on_fuel_consumption_hub1 = \
            self.coefficient_influence_power_on_fuel_consumption_dataset['Передача 1']
        self.coefficient_influence_power_on_fuel_consumption_hub2 = \
            self.coefficient_influence_power_on_fuel_consumption_dataset['Передача 2']
        self.coefficient_influence_power_on_fuel_consumption_hub3 = \
            self.coefficient_influence_power_on_fuel_consumption_dataset['Передача 3']
        self.coefficient_influence_power_on_fuel_consumption_hub4 = \
            self.coefficient_influence_power_on_fuel_consumption_dataset['Передача 4']
        self.coefficient_influence_power_on_fuel_consumption_hub5 = \
            self.coefficient_influence_power_on_fuel_consumption_dataset['Передача 5']
        self.total_force_resistance_movement_hub1 = self.total_force_resistance_movement_dataset['1 передача']
        self.total_force_resistance_movement_hub2 = self.total_force_resistance_movement_dataset['2 передача']
        self.total_force_resistance_movement_hub3 = self.total_force_resistance_movement_dataset['3 передача']
        self.total_force_resistance_movement_hub4 = self.total_force_resistance_movement_dataset['4 передача']
        self.total_force_resistance_movement_hub5 = self.total_force_resistance_movement_dataset['5 передача']
        self.show_graphic()

    @property
    def fuel_consumption_hub1(self):
        return self.__calculate_fuel_consumption_hub(self.coefficient_influence_power_on_fuel_consumption_hub1,
                                                     self.total_force_resistance_movement_hub1)

    @property
    def fuel_consumption_hub2(self):
        return self.__calculate_fuel_consumption_hub(self.coefficient_influence_power_on_fuel_consumption_hub2,
                                                     self.total_force_resistance_movement_hub2)

    @property
    def fuel_consumption_hub3(self):
        return self.__calculate_fuel_consumption_hub(self.coefficient_influence_power_on_fuel_consumption_hub3,
                                                     self.total_force_resistance_movement_hub3)

    @property
    def fuel_consumption_hub4(self):
        return self.__calculate_fuel_consumption_hub(self.coefficient_influence_power_on_fuel_consumption_hub4,
                                                     self.total_force_resistance_movement_hub4)

    @property
    def fuel_consumption_hub5(self):
        return self.__calculate_fuel_consumption_hub(self.coefficient_influence_power_on_fuel_consumption_hub5,
                                                     self.total_force_resistance_movement_hub5)

    def __calculate_fuel_consumption_hub(self, coefficient_influence_power_on_fuel_consumption_hub,
                                         total_force_resistance_movement_hub):
        consumptions = []
        for coefficient, total, coef_turnovers_to_fuel in zip(coefficient_influence_power_on_fuel_consumption_hub,
                                                              total_force_resistance_movement_hub,
                                                              self.coefs_turnovers_to_fuel):
            consumptions.append(coef_turnovers_to_fuel * coefficient * 370 / (36000 * 0.73) * total)
        return consumptions

    def show_graphic(self):
        """Отрисовывает график расхода топлива автомобиля"""
        plt.clf()
        plt.xlabel("Частота, об/мин")
        plt.title('Расход топлива автомобиля')
        plt.plot(self.frequency_turns_per_min, self.fuel_consumption_hub1, label='1 передача')
        plt.plot(self.frequency_turns_per_min, self.fuel_consumption_hub2, label='2 передача')
        plt.plot(self.frequency_turns_per_min, self.fuel_consumption_hub3, label='3 передача')
        plt.plot(self.frequency_turns_per_min, self.fuel_consumption_hub4, label='4 передача')
        plt.plot(self.frequency_turns_per_min, self.fuel_consumption_hub5, label='5 передача')
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)
