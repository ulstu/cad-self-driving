from dataclasses import dataclass, field


@dataclass
class CalculateKPDService:
    """Сервис для рассчёта КПД трансмиссии

       Parameters
            ----------
            number_of_spur_gears : list
                Список числа цилиндров каждой передачи
            number_of_bevel_gears : list
                Список конических передач каждой передачи
            number_of_cardan_gears : list
                Число крестовин кардана каждой передачи
    """
    number_of_spur_gears: list = field(default_factory=list)
    number_of_bevel_gears: list = field(default_factory=list)
    number_of_cardan_gears: list = field(default_factory=list)

    @property
    def KPD(self):
        """Атрибут КПД"""
        return [
            (0.98 ** self.number_of_spur_gears[i]) * (0.97 ** self.number_of_bevel_gears[i]) * (0.995 ** self.number_of_cardan_gears[i])
                for i in range(5)
        ]
