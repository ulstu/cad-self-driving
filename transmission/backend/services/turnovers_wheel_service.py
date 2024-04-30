from dataclasses import dataclass


@dataclass
class TurnoversWheelsService:
    """Таблица - обороты колеса от частоты оборотов двигателя

       Parameters
            ----------
            full_gear_ratio_hub1 : float
                Передаточное число для 1 передачи
            full_gear_ratio_hub2 : float
                Передаточное число для 2 передачи
            full_gear_ratio_hub3 : float
                Передаточное число для 3 передачи
            full_gear_ratio_hub4 : float
                Передаточное число для 4 передачи
            full_gear_ratio_hub5 : float
                Передаточное число для 5 передачи
            frequency_turns_per_min : list
                Список оборотов, для которых нужно расчитать данные
        """
    full_gear_ratio_hub1: float
    full_gear_ratio_hub2: float
    full_gear_ratio_hub3: float
    full_gear_ratio_hub4: float
    full_gear_ratio_hub5: float
    full_gear_ratio_reverse: float
    frequency_turns_per_min: list

    @property
    def turnovers_wheels_hub1(self) -> list:
        """Частота вращения колеса об/мин на 1 скорости"""
        return self.__calculate_turnovers_wheels_array(self.full_gear_ratio_hub1)

    @property
    def turnovers_wheels_hub2(self) -> list:
        """Частота вращения колеса об/мин на 2 скорости"""
        return self.__calculate_turnovers_wheels_array(self.full_gear_ratio_hub2)

    @property
    def turnovers_wheels_hub3(self) -> list:
        """Частота вращения колеса об/мин на 3 скорости"""
        return self.__calculate_turnovers_wheels_array(self.full_gear_ratio_hub3)

    @property
    def turnovers_wheels_hub4(self) -> list:
        """Частота вращения колеса об/мин на 4 скорости"""
        return self.__calculate_turnovers_wheels_array(self.full_gear_ratio_hub4)

    @property
    def turnovers_wheels_hub5(self) -> list:
        """Частота вращения колеса об/мин на 5 скорости"""
        return self.__calculate_turnovers_wheels_array(self.full_gear_ratio_hub5)

    @property
    def turnovers_wheels_reverse(self) -> list:
        """Частота вращения колеса об/мин на R скорости"""
        return self.__calculate_turnovers_wheels_array(self.full_gear_ratio_reverse)

    def __calculate_turnovers_wheels_array(self, full_gear_ratio: float) -> list:
        """"Возвращает массив оборотов колеса в минуту относительно частоты оборотов двигателя и номера передачи"""
        gear_ratio_array = []
        for frequency in self.frequency_turns_per_min:
            gear_ratio = frequency / full_gear_ratio
            gear_ratio_array.append(gear_ratio)
        return gear_ratio_array
