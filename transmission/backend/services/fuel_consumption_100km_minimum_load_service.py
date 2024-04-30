from dataclasses import dataclass

import numpy
from pandas import DataFrame


@dataclass
class FuelConsumption100kmMinimumLoadService:
    """Расход топлива автомобиля на 100км  для минимальной нагрузки"""
    frequency_turns_per_min: numpy.array
    fuel_consumption_dataset: DataFrame
    speed_car_dataset: DataFrame

    def __post_init__(self):
        self.fuel_consumption_hub1 = self.fuel_consumption_dataset['Передача 1']
        self.fuel_consumption_hub2 = self.fuel_consumption_dataset['Передача 2']
        self.fuel_consumption_hub3 = self.fuel_consumption_dataset['Передача 3']
        self.fuel_consumption_hub4 = self.fuel_consumption_dataset['Передача 4']
        self.fuel_consumption_hub5 = self.fuel_consumption_dataset['Передача 5']
        self.speed_car_hub1 = self.speed_car_dataset['Передача 1']
        self.speed_car_hub2 = self.speed_car_dataset['Передача 2']
        self.speed_car_hub3 = self.speed_car_dataset['Передача 3']
        self.speed_car_hub4 = self.speed_car_dataset['Передача 4']
        self.speed_car_hub5 = self.speed_car_dataset['Передача 5']

    @property
    def calculate_fuel_consumption_hub1(self):
        return self.__calculate_fuel_consumption(self.fuel_consumption_hub1, self.speed_car_hub1)

    @property
    def calculate_fuel_consumption_hub2(self):
        return self.__calculate_fuel_consumption(self.fuel_consumption_hub2, self.speed_car_hub2)

    @property
    def calculate_fuel_consumption_hub3(self):
        return self.__calculate_fuel_consumption(self.fuel_consumption_hub3, self.speed_car_hub3)

    @property
    def calculate_fuel_consumption_hub4(self):
        return self.__calculate_fuel_consumption(self.fuel_consumption_hub4, self.speed_car_hub4)

    @property
    def calculate_fuel_consumption_hub5(self):
        return self.__calculate_fuel_consumption(self.fuel_consumption_hub5, self.speed_car_hub5)

    def __calculate_fuel_consumption(self, fuel_consumption_hub, speed_car_hub):
        consumptions = []
        for fuel, speed in zip(fuel_consumption_hub, speed_car_hub):
            consumptions.append(fuel * 100 / speed)
        return consumptions
