from dataclasses import dataclass

import numpy
from matplotlib import pyplot as plt

from utils.graphic_helper import GraphicHelper


@dataclass
class CoefficientTurnoversToFuelService:
    """Сервис для формирования таблицы коэффициентов влияния оборотов двигателя на расход топлива

       Parameters
            ----------
            frequency_turns_per_min : array
                Массив оборотов, для которых будут осуществляться вычисления
    """
    frequency_turns_per_min: numpy.array

    def __post_init__(self):
        self.show_graphic()

    @property
    def coefs(self):
        return self.__calculate_coefs_effect_engine_speed_on_fuel_consumption()

    def __calculate_coefs_effect_engine_speed_on_fuel_consumption(self):
        """
        :return: список коэффициентов влияния оборотов двигателя на расход топлива
        """
        coefs = []
        max_turn = self.frequency_turns_per_min[-1]
        for turn in self.frequency_turns_per_min:
            coef = 1.25 - 0.99 * (turn / max_turn) + 0.98 * ((turn / max_turn) ** 2) - 0.24 * ((turn / max_turn) ** 3)
            coefs.append(coef)
        return coefs

    def show_graphic(self):
        """График коэффициента влияния оборотов двигателя на расход топлива"""
        plt.clf()
        plt.xlabel("Частота, об/мин")
        plt.title('Коэффициенты влияния оборотов двигателя на расход топлива')
        plt.plot(self.frequency_turns_per_min, self.coefs)
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)
