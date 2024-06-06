from dataclasses import dataclass

import numpy
from pandas import DataFrame


@dataclass
class RollingResistanceCoefficientDryAsphaltService:
    """Сервис для формирования таблицы коэффициент сопротивления качению при хорошее состояние сухого асфальта"""
    frequency_turns_per_min: numpy.array
    rolling_resistance_data: DataFrame
    coefficient_influence_speed_data: DataFrame
    speed_car_data: DataFrame
    mass_data: DataFrame

    def __post_init__(self):
        self.speed_hub1 = self.speed_car_data['Передача 1']
        self.speed_hub2 = self.speed_car_data['Передача 2']
        self.speed_hub3 = self.speed_car_data['Передача 3']
        self.speed_hub4 = self.speed_car_data['Передача 4']
        self.speed_hub5 = self.speed_car_data['Передача 5']
        self.coef_rolling_resistance = self.rolling_resistance_data['Минимум'][0]
        self.coefficient_influence_speed = self.coefficient_influence_speed_data['Км/час минимум'][1]
        self.full_mass=self.mass_data['Полная масса'][0]

    @property
    def rolling_resistance_coefficient_dry_asphalt_hub1(self):
        return self.__calculate_rolling_resistance_coefficient_dry_asphalt_hub( self.speed_hub1)

    @property
    def rolling_resistance_coefficient_dry_asphalt_hub2(self):
        return self.__calculate_rolling_resistance_coefficient_dry_asphalt_hub( self.speed_hub2)

    @property
    def rolling_resistance_coefficient_dry_asphalt_hub3(self):
        return self.__calculate_rolling_resistance_coefficient_dry_asphalt_hub( self.speed_hub3)

    @property
    def rolling_resistance_coefficient_dry_asphalt_hub4(self):
        return self.__calculate_rolling_resistance_coefficient_dry_asphalt_hub( self.speed_hub4)

    @property
    def rolling_resistance_coefficient_dry_asphalt_hub5(self):
        return self.__calculate_rolling_resistance_coefficient_dry_asphalt_hub( self.speed_hub5)


    def __calculate_rolling_resistance_coefficient_dry_asphalt_hub(self, speed_hub):
        coefs = []
        for speed in speed_hub:
            coef = self.coef_rolling_resistance*(1+self.coefficient_influence_speed*(speed**2))*self.full_mass
            coefs.append(coef)
        return coefs
