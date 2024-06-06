from dataclasses import dataclass

import numpy as np
from matplotlib import pyplot as plt
from pandas import DataFrame
from sklearn import linear_model
from sklearn.preprocessing import PolynomialFeatures

from utils.graphic_helper import GraphicHelper


@dataclass
class TrendLinesService:
    '''Сервис для построения линий трендов

        Parameters
            ----------
            power_and_torque_info_dataset : array
                Датасет мощности и крутящего момента (добытых путём замеров, данные хранятся в config)
    '''
    power_and_torque_info_dataset: DataFrame

    def __post_init__(self):
        self.predicate_freq_array = np.linspace(0, 4800, 101)
        self.show_graphic_torques()
        self.show_graphic_powers()
        self.show_graphic_increased_values()
        self.show_graphic_original_values()

    @property
    def polynom_coefs_hm(self):
        x_data, y_data = self._get_linear_regression(self.power_and_torque_info_dataset[['power_and_torque_hms']])
        return self._get_polynom_coefs(x_data, y_data)

    @property
    def polynom_coefs_hp(self):
        x_data, y_data = self._get_linear_regression(
            self.power_and_torque_info_dataset[['power_and_torque_horse_powers']])
        return self._get_polynom_coefs(x_data, y_data)

    def _get_linear_regression(self, y_column):
        """
        :param y_column: Массив данных для обучения (в частности массив значений момента или л.с.)
        :return: Возвращает 2 массива X и Y (X-увеличенный массив оборотов, Y-спрогнозируемые значения момента или л.с.)
        """
        polynomial_features = PolynomialFeatures(degree=4, include_bias=False)
        x_data = polynomial_features.fit_transform(self.power_and_torque_info_dataset[['power_and_torque_turns']])
        predicate_data = polynomial_features.fit_transform(self.predicate_freq_array.reshape(-1, 1))
        model = linear_model.LinearRegression()
        model.fit(x_data, y_column)
        predicate_result = model.predict(predicate_data)
        return (self.predicate_freq_array, predicate_result.reshape(-1, 1))

    def _get_polynom_coefs(self, x_array, y_aaray):
        """
        :param x_array: Выборка X
        :param y_aaray: Выборка Y
        :return: Список коэффициентов поолиномомов (от 1 до 4)
        """
        return np.polyfit(x_array, y_aaray, 4)

    def show_graphic_torques(self):
        """Построение графика крутящего момента от оборотов двигателя"""
        plt.clf()
        plt.ylabel("МКР, Нм")
        plt.xlabel("Частота, об/мин")
        plt.title('Крутящий момент от оборотов двигателя')
        plt.xlim(0, 5000)
        plt.ylim(0, 250)
        plt.plot(np.array(self.power_and_torque_info_dataset['power_and_torque_turns']),
                 np.array(self.power_and_torque_info_dataset['power_and_torque_hms']), label='Крутящий момент МКР, Нм')
        polynom_coefs = self.polynom_coefs_hm
        text = (f"y = {polynom_coefs[0][0]}x^4 - \n"
                f"{polynom_coefs[1][0]}x^3 + \n"
                f"{polynom_coefs[2][0]}x^2 - \n"
                f"{polynom_coefs[3][0]}x + \n"
                f"{polynom_coefs[4][0]}")
        plt.text(140, 80, text, fontsize=10)
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)

    def show_graphic_powers(self):
        """Построение графика мощности от оборотов двигателя"""
        plt.clf()
        plt.ylabel("Мощность, л.с.")
        plt.xlabel("Частота, об/мин")
        plt.title('Мощность от оборотов двигателя')
        plt.xlim(0, 5000)
        plt.ylim(0, 120)
        plt.plot(np.array(self.power_and_torque_info_dataset['power_and_torque_turns']),
                 np.array(self.power_and_torque_info_dataset['power_and_torque_horse_powers']),
                 label='Крутящий момент МКР, Нм')
        polynom_coefs = self.polynom_coefs_hp
        text = (f"y = {polynom_coefs[0][0]}x^4 - \n"
                f"{polynom_coefs[1][0]}x^3 + \n"
                f"{polynom_coefs[2][0]}x^2 - \n"
                f"{polynom_coefs[3][0]}x + \n"
                f"{polynom_coefs[4][0]}")
        plt.text(2300, 12, text, fontsize=10)
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)

    def show_graphic_increased_values(self):
        """Построение графика с увеличенным числом значений"""
        plt.clf()
        plt.ylabel("Мощность, л.с.")
        plt.xlabel("Частота, об/мин")
        plt.title('График с увеличенным числом значений')
        plt.xlim(0, 4500)
        plt.ylim(0, 250)
        x, y = self._get_linear_regression(self.power_and_torque_info_dataset[['power_and_torque_hms']])
        plt.plot(x, y, label='Крутящий момент МКР, Нм')
        x, y = self._get_linear_regression(self.power_and_torque_info_dataset[['power_and_torque_horse_powers']])
        plt.plot(x, y, label='Мощность, л.с.')
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)

    def show_graphic_original_values(self):
        """Построение графика с увеличенным числом значений"""
        plt.clf()
        plt.ylabel("Мощность, л.с.")
        plt.xlabel("Частота, об/мин")
        plt.title('График с оригинальными значениями')
        plt.xlim(0, 4500)
        plt.ylim(0, 250)
        plt.plot(
            np.array(self.power_and_torque_info_dataset['power_and_torque_turns']),
            np.array(self.power_and_torque_info_dataset['power_and_torque_hms']),
            label='Крутящий момент МКР, Нм')
        plt.plot(
            np.array(self.power_and_torque_info_dataset['power_and_torque_turns']),
            np.array(self.power_and_torque_info_dataset['power_and_torque_horse_powers']),
            label='Мощность, л.с.'
        )
        plt.legend()
        plt.grid(axis='y')
        GraphicHelper().save_graphic(plt)
