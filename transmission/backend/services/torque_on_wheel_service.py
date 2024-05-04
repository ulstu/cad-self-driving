from dataclasses import dataclass

from matplotlib import pyplot as plt
from pandas import DataFrame

from utils.graphic_helper import GraphicHelper


@dataclass
class TorqueOnWheelService:
    """ Сервис для формирования данных о крутящем момменте на колесе относиельно передачи и кол-ва оборотов двигателя

        Parameters
            ----------
            gear_ratios_dataset : DataFrame
                Передаточные числа
            power_and_torque_dataset : DataFrame
                Мощность и крутящий момент
            kpd_dataset : DataFrame
               КПД
            frequency_turns_per_min : list
               Список оборотов, для которых нужно расчитать данные
    """
    gear_ratios_dataset: DataFrame
    power_and_torque_dataset: DataFrame
    kpd_dataset: DataFrame
    frequency_turns_per_min: list

    def __post_init__(self):
        self.full_gear_ratio_hub1 = self.gear_ratios_dataset['Полное передаточное число'][0]
        self.full_gear_ratio_hub2 = self.gear_ratios_dataset['Полное передаточное число'][1]
        self.full_gear_ratio_hub3 = self.gear_ratios_dataset['Полное передаточное число'][2]
        self.full_gear_ratio_hub4 = self.gear_ratios_dataset['Полное передаточное число'][3]
        self.full_gear_ratio_hub5 = self.gear_ratios_dataset['Полное передаточное число'][4]
        self.full_gear_ratio_reverse = self.gear_ratios_dataset['Полное передаточное число'][5]
        self.hm_per_turns_engine = self.power_and_torque_dataset['Крутящий момент'].to_numpy()
        self.kpd_hub1 = self.kpd_dataset['КПД'][0]
        self.kpd_hub2 = self.kpd_dataset['КПД'][1]
        self.kpd_hub3 = self.kpd_dataset['КПД'][2]
        self.kpd_hub4 = self.kpd_dataset['КПД'][3]
        self.kpd_hub5 = self.kpd_dataset['КПД'][4]
        self.show_graphic()

    @property
    def torque_on_wheel_hub1(self):
        return self.__calculate_torque_on_wheel(self.full_gear_ratio_hub1, self.kpd_hub1)

    @property
    def torque_on_wheel_hub2(self):
        return self.__calculate_torque_on_wheel(self.full_gear_ratio_hub2, self.kpd_hub2)

    @property
    def torque_on_wheel_hub3(self):
        return self.__calculate_torque_on_wheel(self.full_gear_ratio_hub3, self.kpd_hub3)

    @property
    def torque_on_wheel_hub4(self):
        return self.__calculate_torque_on_wheel(self.full_gear_ratio_hub4, self.kpd_hub4)

    @property
    def torque_on_wheel_hub5(self):
        return self.__calculate_torque_on_wheel(self.full_gear_ratio_hub5, self.kpd_hub5)

    def __calculate_torque_on_wheel(self, full_gear_ratio_hub, kpd_hub):
        """Возвращает список крутящего момента на колесе для определённой передачи при определённом кол-ве оборотов двигателя"""
        torque_on_wheel = []
        for hm in self.hm_per_turns_engine:
            torque_on_wheel.append(hm * full_gear_ratio_hub * kpd_hub)
        return torque_on_wheel

    def show_graphic(self):
        """Построение графика крутящего момента на колесе"""
        plt.clf()
        plt.ylabel("об/мин")
        plt.xlabel("Частота, об/мин")
        plt.title('Крутящий момент на колесе от оборотов двигателя')
        plt.plot(self.frequency_turns_per_min, self.torque_on_wheel_hub1, label='1 передача')
        plt.plot(self.frequency_turns_per_min, self.torque_on_wheel_hub2, label='2 передача')
        plt.plot(self.frequency_turns_per_min, self.torque_on_wheel_hub3, label='3 передача')
        plt.plot(self.frequency_turns_per_min, self.torque_on_wheel_hub4, label='4 передача')
        plt.plot(self.frequency_turns_per_min, self.torque_on_wheel_hub5, label='5 передача')
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)
