from dataclasses import dataclass

import numpy
from pandas import DataFrame


@dataclass
class DynamicFactorService:
    """Рассчитывает Динамический фактор"""
    km_per_hour_array: numpy.array
    speed_car_dataset: DataFrame
    polynom_dataset: DataFrame
    wheel_info_dataset:DataFrame
    dependence_torque_on_air_resistance_dataset:DataFrame
    gear_ratio_dataset:DataFrame
    kpd_dataset: DataFrame


    def __post_init__(self):
        self.min_frequency = self.speed_car_dataset['Частота оборотов двигателя'][0]
        self.max_frequency = self.speed_car_dataset['Частота оборотов двигателя'].iat[-1]
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
        self.dynamic_radius = self.wheel_info_dataset['Ширина профиля'][3]
        self.air_resistance_array = self.dependence_torque_on_air_resistance_dataset['Сопротивление воздуха']
        self.dependence_torque_hub1_array=self.dependence_torque_on_air_resistance_dataset['Крутящий момент 1']
        self.dependence_torque_hub2_array=self.dependence_torque_on_air_resistance_dataset['Крутящий момент 2']
        self.dependence_torque_hub3_array=self.dependence_torque_on_air_resistance_dataset['Крутящий момент 3']
        self.dependence_torque_hub4_array=self.dependence_torque_on_air_resistance_dataset['Крутящий момент 4']
        self.dependence_torque_hub5_array=self.dependence_torque_on_air_resistance_dataset['Крутящий момент 5']
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
        return self.__calculate_torque_hub(self.turnovers_hub5, self.full_gear_ratio_hub5, self.kpd_hub1)

    @property
    def fuel_hub1(self):
        return self.__calculate_fuel(self.turnovers_hub1, self.torque_hub1)

    @property
    def fuel_hub2(self):
        return self.__calculate_fuel(self.turnovers_hub2, self.torque_hub2)

    @property
    def fuel_hub3(self):
        return self.__calculate_fuel(self.turnovers_hub3, self.torque_hub3)

    @property
    def fuel_hub4(self):
        return self.__calculate_fuel(self.turnovers_hub4, self.torque_hub4)

    @property
    def fuel_hub5(self):
        return self.__calculate_fuel(self.turnovers_hub5, self.torque_hub5)

    def __calculate_turnovers_hub(self, min_speed, max_speed, min_speed_hub):
        turnovers=[]
        for speed in self.km_per_hour_array:
            if min_speed<=speed<=max_speed:
                turnovers.append((self.min_frequency/min_speed_hub)*speed)
            else:
                turnovers.append('-')
        return turnovers

    def __calculate_torque_hub(self, turnovers_hub, full_gear_ratio_hub, kpd_hub):
        torques=[]
        for turnover, air_resistance in zip(turnovers_hub,self.air_resistance_array):
            if turnover=='-':
                torques.append('-')
            else:
                torque = (((self.coef_moment_5*(turnover**5)+self.coef_moment_4*(turnover**4)+self.coef_moment_3*(turnover**3)+self.coef_moment_2*(turnover**2)+self.coef_moment_1*turnover+self.coef_self)*full_gear_ratio_hub*kpd_hub)/self.dynamic_radius)-air_resistance
                torques.append(torque)
        return torques

    def __calculate_fuel(self, turnovers_hub, torque_hub):
        fuels = []
        for turnover, air_resistance, torque in zip(turnovers_hub, self.air_resistance_array, torque_hub):
            if turnover=='-':
                fuels.append('-')
            else:
                fuel = ((1.25-0.99*(turnover/self.max_frequency)+0.98*((turnover/self.max_frequency)**2)-0.24*((turnover/self.max_frequency)**3))*(3.27-8.22*(air_resistance/torque)+9.13*((air_resistance/torque)**2)-3.18*((air_resistance/torque)**3))*270/(36000*0.73)*air_resistance)
                fuels.append(fuel)
        return fuels