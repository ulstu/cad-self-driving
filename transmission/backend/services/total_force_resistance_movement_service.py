from dataclasses import dataclass

import numpy
from pandas import DataFrame


@dataclass
class TotalForceResistanceMovement:
    """Сервис по рассчёту суммарной силы сопротивления движению
        Parameters
            ----------
            frequency_turns_per_min : numpy.array
                Список оборотов, для которых нужно расчитать данные
            rolling_resistance_coefficient_dry_asphalt_dataset : DataFrame
                Коэффициенты сопротивления качению при хорошем состояние сухого асфальта
            air_resistance_dataset : DataFrame
               Сопротивление воздуха

    """
    frequency_turns_per_min: numpy.array
    rolling_resistance_coefficient_dry_asphalt_dataset: DataFrame
    air_resistance_dataset: DataFrame

    def __post_init__(self):
        self.rolling_resistance_coefficient_hub1 = self.rolling_resistance_coefficient_dry_asphalt_dataset['1 передача']
        self.rolling_resistance_coefficient_hub2 = self.rolling_resistance_coefficient_dry_asphalt_dataset['2 передача']
        self.rolling_resistance_coefficient_hub3 = self.rolling_resistance_coefficient_dry_asphalt_dataset['3 передача']
        self.rolling_resistance_coefficient_hub4 = self.rolling_resistance_coefficient_dry_asphalt_dataset['4 передача']
        self.rolling_resistance_coefficient_hub5 = self.rolling_resistance_coefficient_dry_asphalt_dataset['5 передача']
        self.air_resistance_hub1 = self.air_resistance_dataset['Передача 1']
        self.air_resistance_hub2 = self.air_resistance_dataset['Передача 2']
        self.air_resistance_hub3 = self.air_resistance_dataset['Передача 3']
        self.air_resistance_hub4 = self.air_resistance_dataset['Передача 4']
        self.air_resistance_hub5 = self.air_resistance_dataset['Передача 5']

    @property
    def total_force_resistance_movement_hub1(self):
        return self.__calculate_total_force_resistance_movement_hub(self.rolling_resistance_coefficient_hub1,
                                                                    self.air_resistance_hub1)

    @property
    def total_force_resistance_movement_hub2(self):
        return self.__calculate_total_force_resistance_movement_hub(self.rolling_resistance_coefficient_hub2,
                                                                    self.air_resistance_hub2)

    @property
    def total_force_resistance_movement_hub3(self):
        return self.__calculate_total_force_resistance_movement_hub(self.rolling_resistance_coefficient_hub3,
                                                                    self.air_resistance_hub3)

    @property
    def total_force_resistance_movement_hub4(self):
        return self.__calculate_total_force_resistance_movement_hub(self.rolling_resistance_coefficient_hub4,
                                                                    self.air_resistance_hub4)

    @property
    def total_force_resistance_movement_hub5(self):
        return self.__calculate_total_force_resistance_movement_hub(self.rolling_resistance_coefficient_hub5,
                                                                    self.air_resistance_hub5)

    def __calculate_total_force_resistance_movement_hub(self, rolling_resistance_coefficient_hub, air_resistance_hub):
        totals = []
        for rolling_resistance, air_resistance in zip(rolling_resistance_coefficient_hub, air_resistance_hub):
            totals.append(rolling_resistance + air_resistance)
        return totals
