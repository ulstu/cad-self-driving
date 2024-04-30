from dataclasses import dataclass


@dataclass
class GearRatioService:
    """Сервис по высчитыванию полного передаточного числа для каждой передачи

        Parameters
            ----------
            gear_ratio_hub(number) : float
                Передаточное число определённой передачи (из config)
            gear_ratio_reverse : float
                Передаточное число задней передачи
            transfer_case : float
                Передаточное число раздаточной коробки (если нет информации, то в config должно быть 1.0)
            on_board_gearbox : float
                Передаточное число бортового редуктора (если нет информации, то в config должно быть 1.0)
            main_pair : float
                Передаточное число главной пары
    """
    gear_ratio_hub1: float
    gear_ratio_hub2: float
    gear_ratio_hub3: float
    gear_ratio_hub4: float
    gear_ratio_hub5: float
    gear_ratio_reverse: float
    transfer_case: float
    on_board_gearbox: float
    main_pair: float

    @property
    def full_gear_ratio_hub1(self):
        return self.gear_ratio_hub1 * self.main_pair

    @property
    def full_gear_ratio_hub2(self):
        return self.gear_ratio_hub2 * self.main_pair

    @property
    def full_gear_ratio_hub3(self):
        return self.gear_ratio_hub3 * self.main_pair

    @property
    def full_gear_ratio_hub4(self):
        return self.gear_ratio_hub4 * self.main_pair

    @property
    def full_gear_ratio_hub5(self):
        return self.gear_ratio_hub5 * self.main_pair

    @property
    def full_gear_ratio_reverse(self):
        return self.gear_ratio_reverse * self.main_pair
