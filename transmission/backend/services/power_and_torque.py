from dataclasses import dataclass, field

from matplotlib import pyplot as plt

from utils.graphic_helper import GraphicHelper


@dataclass
class PowerAndTorqueService:
    """Сервис рассчёта мощности и крутящего момента"""
    frequency_turns_per_min: list = field(default_factory=list)
    POLYNOMIAL_COEFFICIENTS_POWER: list = field(default_factory=list)
    POLYNOMIAL_COEFFICIENTS_TORQUE: list = field(default_factory=list)

    def __post_init__(self):
        self.POLYNOMIAL_COEFFICIENTS_POWER = [0, -1.75E-13, -1.98E-09, 1.55373E-05, 0.005991066, 7.459965753]
        self.POLYNOMIAL_COEFFICIENTS_TORQUE = [0, 6.51774E-13, -0.0000000104, 3.44758E-05, -0.010124117, 175.1449366]
        self.torques = self.__build_torques()
        self.powers = self.__build_powers()

    def __build_torques(self) -> list:
        """Рассчитывает мощность крутящего момента"""
        torques = []
        pol_coeffs = self.POLYNOMIAL_COEFFICIENTS_TORQUE
        for frequency in self.frequency_turns_per_min:
            torque = pol_coeffs[0] * (frequency ** 5) + pol_coeffs[1] * (frequency ** 4) + pol_coeffs[2] * (frequency ** 3) \
                     + pol_coeffs[3] * (frequency ** 2) + pol_coeffs[4] * frequency + pol_coeffs[5]
            torques.append(torque)
        return torques

    def __build_powers(self) -> list:
        """Рассчитывает мощность в лошадиных силах"""
        powers = []
        pol_coeffs = self.POLYNOMIAL_COEFFICIENTS_POWER
        for frequency in self.frequency_turns_per_min:
            torque = pol_coeffs[0] * (frequency ** 5) + pol_coeffs[1] * (frequency ** 4) + pol_coeffs[2] * (frequency ** 3) \
                     + pol_coeffs[3] * (frequency ** 2) + pol_coeffs[4] * frequency + pol_coeffs[5]
            powers.append(torque)
        return powers
