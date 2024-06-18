from dataclasses import dataclass

import numpy
from matplotlib import pyplot as plt
from pandas import DataFrame

from utils.graphic_helper import GraphicHelper
from utils.table_helper import TableHelper


@dataclass
class TotalForceWheelIdealConditionsService:
    """Суммарная сила на колесе в идеальных условиях
        Parameters
            ----------
            km_per_hour_array : numpy.array
                Скорости автомобиля, км в час
            total_resistance_force_movement_dataset : DataFrame
                Коэффиценты сопротивлению качению
            speed_car_dataset : DataFrame
                Скорости автомобиля
            polynom_dataset : DataFrame
                Коэффиценты полинома
            dependence_torque_on_air_resistance_dataset : DataFrame
                Зависимость крутящего момента от сопротивления воздуха
            gear_ratio_dataset : DataFrame
                Передаточные числа
            kpd_dataset : DataFrame
                КПД
    """
    km_per_hour_array: numpy.array
    total_resistance_force_movement_dataset: DataFrame
    #TODO: доделать сервис

    speed_car_dataset: DataFrame
    polynom_dataset: DataFrame
    dependence_torque_on_air_resistance_dataset: DataFrame
    gear_ratio_dataset: DataFrame
    kpd_dataset: DataFrame

    def __post_init__(self):
        #self.total_resistance_force_movement = self.total_resistance_force_movement_dataset['Сила подъёма'][4]


        # self.rolling_resistance_array = self.rolling_resistance_dataset[
        #     '1.Хорошее состояние сухого асфальта'].to_numpy()
        self.min_frequency = self.speed_car_dataset['Частота оборотов двигателя'][0]
        self.min_speed_hub1 = self.speed_car_dataset['Передача 1'][0]
        self.min_speed_hub2 = self.speed_car_dataset['Передача 2'][0]
        self.min_speed_hub3 = self.speed_car_dataset['Передача 3'][0]
        self.min_speed_hub4 = self.speed_car_dataset['Передача 4'][0]
        self.min_speed_hub5 = self.speed_car_dataset['Передача 5'][0]
        self.frequency_array = self.speed_car_dataset['Частота оборотов двигателя'].to_numpy()
        self.speed_hub1_array = self.speed_car_dataset['Передача 1'].to_numpy()
        self.speed_hub2_array = self.speed_car_dataset['Передача 2'].to_numpy()
        self.speed_hub3_array = self.speed_car_dataset['Передача 3'].to_numpy()
        self.speed_hub4_array = self.speed_car_dataset['Передача 4'].to_numpy()
        self.speed_hub5_array = self.speed_car_dataset['Передача 5'].to_numpy()
        self.coef_moment_5 = self.polynom_dataset['X/5'][0]
        self.coef_moment_4 = self.polynom_dataset['X/4'][0]
        self.coef_moment_3 = self.polynom_dataset['X/3'][0]
        self.coef_moment_2 = self.polynom_dataset['X/2'][0]
        self.coef_moment_1 = self.polynom_dataset['X'][0]
        self.coef_moment_1 = self.polynom_dataset['X'][0]
        self.full_gear_ratio_hub1 = self.gear_ratio_dataset['Полное передаточное число'][0]
        self.full_gear_ratio_hub2 = self.gear_ratio_dataset['Полное передаточное число'][1]
        self.full_gear_ratio_hub3 = self.gear_ratio_dataset['Полное передаточное число'][2]
        self.full_gear_ratio_hub4 = self.gear_ratio_dataset['Полное передаточное число'][3]
        self.full_gear_ratio_hub5 = self.gear_ratio_dataset['Полное передаточное число'][4]
        self.kpd_hub1 = self.kpd_dataset['КПД'][0]
        self.kpd_hub2 = self.kpd_dataset['КПД'][1]
        self.kpd_hub3 = self.kpd_dataset['КПД'][2]
        self.kpd_hub4 = self.kpd_dataset['КПД'][3]
        self.kpd_hub5 = self.kpd_dataset['КПД'][4]
        self.air_resistance_array = self.dependence_torque_on_air_resistance_dataset['Сопротивление воздуха'].to_numpy()
        self.coef_self = self.polynom_dataset['Собственный коэффициент'][0]
        self.show_graphic()

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
        return self.__calculate_turnovers_hub(20, 125, self.min_speed_hub4)

    @property
    def turnovers_hub5(self):
        return self.__calculate_turnovers_hub(25, 150, self.min_speed_hub5)

    @property
    def force_on_wheel_hub1(self):
        return self.__calculate_force_on_wheel(self.turnovers_hub1, self.full_gear_ratio_hub1, self.kpd_hub1)

    @property
    def force_on_wheel_hub2(self):
        return self.__calculate_force_on_wheel(self.turnovers_hub2, self.full_gear_ratio_hub2, self.kpd_hub2)

    @property
    def force_on_wheel_hub3(self):
        return self.__calculate_force_on_wheel(self.turnovers_hub3, self.full_gear_ratio_hub3, self.kpd_hub3)

    @property
    def force_on_wheel_hub4(self):
        return self.__calculate_force_on_wheel(self.turnovers_hub4, self.full_gear_ratio_hub4, self.kpd_hub4)

    @property
    def force_on_wheel_hub5(self):
        return self.__calculate_force_on_wheel(self.turnovers_hub5, self.full_gear_ratio_hub5, self.kpd_hub5)

    def __calculate_turnovers_hub(self, min_speed, max_speed, min_speed_hub):
        turnovers = []
        for speed in self.km_per_hour_array:
            if min_speed <= speed <= max_speed:
                turnovers.append((self.min_frequency / min_speed_hub) * speed)
            else:
                turnovers.append('-')
        return turnovers

    def __calculate_force_on_wheel(self, turnovers_hub, full_gear_ratio_hub, kpd_hub):
        force_on_wheel = []
        for turnovers, air_resistance in zip(turnovers_hub, self.air_resistance_array):
            if turnovers == '-':
                force_on_wheel.append('-')
            else:
                force = ((self.coef_moment_5 * (turnovers ** 5) + self.coef_moment_4 * (
                        turnovers ** 4) + self.coef_moment_3 * (turnovers ** 3) + self.coef_moment_2 * (
                                  turnovers ** 2) + self.coef_moment_1 * turnovers + self.coef_self) * full_gear_ratio_hub * kpd_hub) - air_resistance
                force_on_wheel.append(force)
        return force_on_wheel

    def show_graphic(self):
        """График Суммарной силы на колесе в идиальных условиях"""
        plt.clf()
        plt.xlabel("км/ч")
        plt.title('Сопротивления качению от скорости')

        power_hub1, km_per_hour_for_1_tier = TableHelper().prepare_data_y_for_x(self.force_on_wheel_hub1,
                                                                                self.km_per_hour_array)
        power_hub2, km_per_hour_for_2_tier = TableHelper().prepare_data_y_for_x(self.force_on_wheel_hub2,
                                                                                self.km_per_hour_array)
        power_hub3, km_per_hour_for_3_tier = TableHelper().prepare_data_y_for_x(self.force_on_wheel_hub3,
                                                                                self.km_per_hour_array)
        power_hub4, km_per_hour_for_4_tier = TableHelper().prepare_data_y_for_x(self.force_on_wheel_hub4,
                                                                                self.km_per_hour_array)
        power_hub5, km_per_hour_for_5_tier = TableHelper().prepare_data_y_for_x(self.force_on_wheel_hub5,
                                                                                self.km_per_hour_array)

        plt.plot(km_per_hour_for_1_tier, power_hub1, label='1')
        plt.plot(km_per_hour_for_2_tier, power_hub2, label='2')
        plt.plot(km_per_hour_for_3_tier, power_hub3, label='3')
        plt.plot(km_per_hour_for_4_tier, power_hub4, label='4')
        plt.plot(km_per_hour_for_5_tier, power_hub5, label='5')
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)
