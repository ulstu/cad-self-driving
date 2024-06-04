from dataclasses import dataclass

import numpy
from pandas import DataFrame


@dataclass
class TotalPowerCarEachHubService:
    """Суммарная мощность автомобиля на каждой передаче

        Parameters
            ----------
            frequency_turns_per_min : numpy.array
                Список оборотов, для которых нужно расчитать данные
            torque_on_wheel_dataset : DataFrame
                Крутящий момент на колесе
            total_force_resistance_movement_dataset : DataFrame
                Суммарная сила сопротивления движения
    """
    frequency_turns_per_min: numpy.array
    torque_on_wheel_dataset: DataFrame
    total_force_resistance_movement_dataset: DataFrame

    def __post_init__(self):
        self.torque_on_wheel_hub1 = self.torque_on_wheel_dataset['Передача 1']
        self.torque_on_wheel_hub2 = self.torque_on_wheel_dataset['Передача 2']
        self.torque_on_wheel_hub3 = self.torque_on_wheel_dataset['Передача 3']
        self.torque_on_wheel_hub4 = self.torque_on_wheel_dataset['Передача 4']
        self.torque_on_wheel_hub5 = self.torque_on_wheel_dataset['Передача 5']
        self.total_force_resistance_movement_hub1 = self.total_force_resistance_movement_dataset['1 передача']
        self.total_force_resistance_movement_hub2 = self.total_force_resistance_movement_dataset['2 передача']
        self.total_force_resistance_movement_hub3 = self.total_force_resistance_movement_dataset['3 передача']
        self.total_force_resistance_movement_hub4 = self.total_force_resistance_movement_dataset['4 передача']
        self.total_force_resistance_movement_hub5 = self.total_force_resistance_movement_dataset['5 передача']

    @property
    def total_power_car_each_hub1(self):
        """Мощность на 1 передаче"""
        return self.__calculate_total_power_car_each_hub(self.torque_on_wheel_hub1,
                                                         self.total_force_resistance_movement_hub1)

    @property
    def total_power_car_each_hub2(self):
        """Мощность на 2 передаче"""
        return self.__calculate_total_power_car_each_hub(self.torque_on_wheel_hub2,
                                                         self.total_force_resistance_movement_hub3)

    @property
    def total_power_car_each_hub3(self):
        """Мощность на 3 передаче"""
        return self.__calculate_total_power_car_each_hub(self.torque_on_wheel_hub3,
                                                         self.total_force_resistance_movement_hub3)

    @property
    def total_power_car_each_hub4(self):
        """Мощность на 4 передаче"""
        return self.__calculate_total_power_car_each_hub(self.torque_on_wheel_hub4,
                                                         self.total_force_resistance_movement_hub4)

    @property
    def total_power_car_each_hub5(self):
        """Мощность на 5 передаче"""
        return self.__calculate_total_power_car_each_hub(self.torque_on_wheel_hub5,
                                                         self.total_force_resistance_movement_hub5)

    def __calculate_total_power_car_each_hub(self, torque_on_wheel_hub: list, total_force_resistance_movement_hub: list):
        """
        Рассчитывает для скорости суммарную мощность
        :param torque_on_wheel_hub:
        :param total_force_resistance_movement_hub:
        :return:
        """
        totals = []
        for torque_on_wheel, total_force_resistance_movement in zip(torque_on_wheel_hub,
                                                                    total_force_resistance_movement_hub):
            totals.append(torque_on_wheel - total_force_resistance_movement)
        return totals
