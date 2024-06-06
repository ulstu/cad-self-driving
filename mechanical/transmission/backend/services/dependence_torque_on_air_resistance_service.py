from dataclasses import dataclass

import numpy
from matplotlib import pyplot as plt
from pandas import DataFrame

from utils.graphic_helper import GraphicHelper
from utils.table_helper import TableHelper


@dataclass
class DependenceOfTorqueOnAirResistanceService:
    """ Сервис для формирования таблицы совмещённой мощности на колесе для каждой передачи и сопротивление воздуха

        Parameters
            ----------
            km_per_hour_array : array
                Массив величин скорости, для которых нужно сформировать расчёты
            dimensions_dataset : DataFrame
                Датасет габаритов автомобиля
            speed_car_dataset : DataFrame
                Датасет с скоростью автомобиля относительно кол-ва оборотов двигателя, номера передачи, данных о передаточных числах каждой скорости и параметрах колёс
            polynom_dataset : DataFrame
                Датасет коэффициентов полинома
            gear_ratio_dataset : DataFrame
                Датасет полных передаточных чисел для каждой передачи
            kpd_dataset : DataFrame
                Датасет кпд трансмиссии

    """
    km_per_hour_array: numpy.array
    dimensions_dataset: DataFrame
    speed_car_dataset: DataFrame
    polynom_dataset: DataFrame
    gear_ratio_dataset: DataFrame
    kpd_dataset: DataFrame

    def __post_init__(self):
        self.midelev_section = self.dimensions_dataset['площадь Миделева сечения'][1]
        self.coef_streamlining = self.dimensions_dataset['коэфициент обтекаемости'][1]
        self.min_frequency_turns = self.speed_car_dataset['Частота оборотов двигателя'][0]
        self.min_speed_hub1 = self.speed_car_dataset['Передача 1'][0]
        self.min_speed_hub2 = self.speed_car_dataset['Передача 2'][0]
        self.min_speed_hub3 = self.speed_car_dataset['Передача 3'][0]
        self.min_speed_hub4 = self.speed_car_dataset['Передача 4'][0]
        self.min_speed_hub5 = self.speed_car_dataset['Передача 5'][0]
        self.coef_moment_5 = self.polynom_dataset['X/5'][0]
        self.coef_moment_4 = self.polynom_dataset['X/4'][0]
        self.coef_moment_3 = self.polynom_dataset['X/3'][0]
        self.coef_moment_2 = self.polynom_dataset['X/2'][0]
        self.coef_moment_1 = self.polynom_dataset['X'][0]
        self.coef_moment_1 = self.polynom_dataset['X'][0]
        self.coef_self = self.polynom_dataset['Собственный коэффициент'][0]
        self.full_gear_ratio_hub1 = self.gear_ratio_dataset['Полное передаточное число'][0]
        self.full_gear_ratio_hub2 = self.gear_ratio_dataset['Полное передаточное число'][1]
        self.full_gear_ratio_hub3 = self.gear_ratio_dataset['Полное передаточное число'][2]
        self.full_gear_ratio_hub4 = self.gear_ratio_dataset['Полное передаточное число'][3]
        self.full_gear_ratio_hub5 = self.gear_ratio_dataset['Полное передаточное число'][4]
        self.full_gear_ratio_reverse = self.gear_ratio_dataset['Полное передаточное число'][5]
        self.kpd_hub1 = self.kpd_dataset['КПД'][0]
        self.kpd_hub2 = self.kpd_dataset['КПД'][1]
        self.kpd_hub3 = self.kpd_dataset['КПД'][2]
        self.kpd_hub4 = self.kpd_dataset['КПД'][3]
        self.kpd_hub5 = self.kpd_dataset['КПД'][4]
        self.show_graphic()

    @property
    def air_resistance(self):
        return self.__calculate_air_resistance()

    @property
    def turnovers_hub1(self):
        return self.__calculate_turnovers_hub(1, 30, self.min_speed_hub1)

    @property
    def turnovers_hub2(self):
        return self.__calculate_turnovers_hub(10, 55, self.min_speed_hub2)

    @property
    def turnovers_hub3(self):
        return self.__calculate_turnovers_hub(15, 90, self.min_speed_hub3)

    @property
    def turnovers_hub4(self):
        return self.__calculate_turnovers_hub(20, 130, self.min_speed_hub4)

    @property
    def turnovers_hub5(self):
        return self.__calculate_turnovers_hub(25, 150, self.min_speed_hub5)

    @property
    def torque_hub1(self):
        return self.__calculate_torque_hub(self.turnovers_hub1, self.full_gear_ratio_hub1, self.kpd_hub1)

    @property
    def torque_hub2(self):
        return self.__calculate_torque_hub(self.turnovers_hub2, self.full_gear_ratio_hub2, self.kpd_hub2)

    @property
    def torque_hub3(self):
        return self.__calculate_torque_hub(self.turnovers_hub3, self.full_gear_ratio_hub3, self.kpd_hub3)

    @property
    def torque_hub4(self):
        return self.__calculate_torque_hub(self.turnovers_hub4, self.full_gear_ratio_hub4, self.kpd_hub4)

    @property
    def torque_hub5(self):
        return self.__calculate_torque_hub(self.turnovers_hub5, self.full_gear_ratio_hub5, self.kpd_hub5)

    def __calculate_air_resistance(self):
        """
        :return: сопротивление воздуха
        """
        air_resistance = []
        for speed in self.km_per_hour_array:
            air_resistance.append(0.5 * self.midelev_section * self.coef_streamlining * 1.22 * ((speed / 3.6) ** 2))
        return air_resistance

    def __calculate_turnovers_hub(self, min_speed, max_speed, min_speed_hub):
        """

        :param min_speed: Минимальная скорость диапазона
        :param max_speed: Максимальная скорость диапазона
        :param min_speed_hub: Минимальная скорость на конкретной передаче
        :return: список оборотов для определённой передачи в определённом скоростном диапазоне
        """
        turnovers_hub = []
        for speed in self.km_per_hour_array:
            if min_speed <= speed <= max_speed:
                turnovers_hub.append((self.min_frequency_turns / min_speed_hub) * speed)
            else:
                turnovers_hub.append('-')
        return turnovers_hub

    def __calculate_torque_hub(self, turnovers_hub_list, full_gear_ratio_hub, kpd_hub):
        torques = []
        for turnover in turnovers_hub_list:
            if turnover == '-':
                torques.append('-')
            else:
                torque = ((self.coef_moment_5 * (turnover ** 5)) + (self.coef_moment_4 * (turnover ** 4)) + (
                        self.coef_moment_3 * (turnover ** 3)) + (self.coef_moment_2 * (turnover ** 2)) + (
                                  self.coef_moment_1 * turnover) + self.coef_self) * full_gear_ratio_hub * kpd_hub
                torques.append(torque)
        return torques

    def show_graphic(self):
        """Построение графика МКР и сопротивление воздуха от скорости"""
        plt.clf()
        plt.xlabel("Скорость км/ч")
        plt.ylabel("Н/м")
        plt.title('МКР и сопротивление воздуха от скорости')

        torque_hub1, km_per_hour_for_1_tier = TableHelper().prepare_data_y_for_x(self.torque_hub1, self.km_per_hour_array)
        torque_hub2, km_per_hour_for_2_tier = TableHelper().prepare_data_y_for_x(self.torque_hub2, self.km_per_hour_array)
        torque_hub3, km_per_hour_for_3_tier = TableHelper().prepare_data_y_for_x(self.torque_hub3, self.km_per_hour_array)
        torque_hub4, km_per_hour_for_4_tier = TableHelper().prepare_data_y_for_x(self.torque_hub4, self.km_per_hour_array)
        torque_hub5, km_per_hour_for_5_tier = TableHelper().prepare_data_y_for_x(self.torque_hub5, self.km_per_hour_array)

        plt.plot(km_per_hour_for_1_tier, torque_hub1, label='МКР 1 передачи')
        plt.plot(km_per_hour_for_2_tier, torque_hub2, label='МКР 2 передачи')
        plt.plot(km_per_hour_for_3_tier, torque_hub3, label='МКР 3 передачи')
        plt.plot(km_per_hour_for_4_tier, torque_hub4, label='МКР 4 передачи')
        plt.plot(km_per_hour_for_5_tier, torque_hub5, label='МКР 5 передачи')
        plt.plot(self.km_per_hour_array, self.air_resistance, label='Сопр. воздуха')
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)

