from dataclasses import dataclass

import numpy
from matplotlib import pyplot as plt
from pandas import DataFrame

from utils.graphic_helper import GraphicHelper


@dataclass
class RollingResistanceService:
    """Рассчитывает Коэффициенты сопротивления качению"""
    km_per_hour_array: numpy.array
    coefficient_rolling_resistance_wheel_dataset: DataFrame
    mass_car_dataset: DataFrame
    coefficient_influence_speed: DataFrame

    def __post_init__(self):
        self.coef_rolling_resistance_wheel_type1 = self.coefficient_rolling_resistance_wheel_dataset['Минимум'][0]
        self.coef_rolling_resistance_wheel_type2 = self.coefficient_rolling_resistance_wheel_dataset['Минимум'][1]
        self.coef_rolling_resistance_wheel_type3 = self.coefficient_rolling_resistance_wheel_dataset['Минимум'][2]
        self.coef_rolling_resistance_wheel_type4 = self.coefficient_rolling_resistance_wheel_dataset['Минимум'][3]
        self.coef_rolling_resistance_wheel_type5 = self.coefficient_rolling_resistance_wheel_dataset['Минимум'][4]
        self.coef_rolling_resistance_wheel_type6 = self.coefficient_rolling_resistance_wheel_dataset['Минимум'][5]
        self.coef_rolling_resistance_wheel_type7 = self.coefficient_rolling_resistance_wheel_dataset['Минимум'][6]
        self.coef_rolling_resistance_wheel_type8 = self.coefficient_rolling_resistance_wheel_dataset['Минимум'][7]
        self.coefficient_influence_speed_truck = self.coefficient_influence_speed['Км/час минимум'][1]
        self.full_mass = self.mass_car_dataset['Полная масса'][0]
        self.show_graphic()

    @property
    def coef_rolling_resistance_type1(self):
        return self.__calculate_coef_rolling_resistance(self.coef_rolling_resistance_wheel_type1)

    @property
    def coef_rolling_resistance_type2(self):
        return self.__calculate_coef_rolling_resistance(self.coef_rolling_resistance_wheel_type2)

    @property
    def coef_rolling_resistance_type3(self):
        return self.__calculate_coef_rolling_resistance(self.coef_rolling_resistance_wheel_type3)

    @property
    def coef_rolling_resistance_type4(self):
        return self.__calculate_coef_rolling_resistance(self.coef_rolling_resistance_wheel_type4)

    @property
    def coef_rolling_resistance_type5(self):
        return self.__calculate_coef_rolling_resistance(self.coef_rolling_resistance_wheel_type5)

    @property
    def coef_rolling_resistance_type6(self):
        return self.__calculate_coef_rolling_resistance(self.coef_rolling_resistance_wheel_type6)

    @property
    def coef_rolling_resistance_type7(self):
        return self.__calculate_coef_rolling_resistance(self.coef_rolling_resistance_wheel_type7)

    @property
    def coef_rolling_resistance_type8(self):
        return self.__calculate_coef_rolling_resistance(self.coef_rolling_resistance_wheel_type8)

    def __calculate_coef_rolling_resistance(self, coef_rolling_resistance_wheel):
        coefs = []
        for speed in self.km_per_hour_array:
            coef = coef_rolling_resistance_wheel * (
                        1 + self.coefficient_influence_speed_truck * (speed ** 2)) * self.full_mass

            coefs.append(coef)
        return coefs

    def show_graphic(self):
        """График сопротивления качению от скорости"""
        plt.clf()
        plt.xlabel("км/ч")
        plt.title('Сопротивления качению от скорости')
        plt.plot(self.km_per_hour_array, self.coef_rolling_resistance_type1, label='1')
        plt.plot(self.km_per_hour_array, self.coef_rolling_resistance_type2, label='2')
        plt.plot(self.km_per_hour_array, self.coef_rolling_resistance_type3, label='3')
        plt.plot(self.km_per_hour_array, self.coef_rolling_resistance_type4, label='4')
        plt.plot(self.km_per_hour_array, self.coef_rolling_resistance_type5, label='5')
        plt.plot(self.km_per_hour_array, self.coef_rolling_resistance_type6, label='6')
        plt.plot(self.km_per_hour_array, self.coef_rolling_resistance_type7, label='7')
        plt.plot(self.km_per_hour_array, self.coef_rolling_resistance_type8, label='8')
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)
