import json
import pandas as pd
from pandas import DataFrame

from services.calculate_KPD_service import CalculateKPDService
from services.air_resistance_service import AirResistanceService
from services.coefficient_influence_power_on_fuel_consumption_service import \
    CoefficientInfluencePowerOnFuelConsumptionService
from services.coefficient_turnovers_to_fuel_service import CoefficientTurnoversToFuelService
from services.dependence_torque_on_air_resistance_service import DependenceOfTorqueOnAirResistanceService
from services.dynamic_factor_service import DynamicFactorService
from services.fuel_consumption_100km_minimum_load_service import FuelConsumption100kmMinimumLoadService
from services.fuel_consumption_service import FuelConsumptionService
from services.gear_ratio_service import GearRatioService
from services.power_and_torque import PowerAndTorqueService
from services.rolling_resistance_coefficient_dry_asphalt_service import RollingResistanceCoefficientDryAsphaltService
from services.rolling_resistance_service import RollingResistanceService
from services.speed_car_service import SpeedCarService
from services.torque_on_wheel_service import TorqueOnWheelService
from services.total_force_resistance_movement_service import TotalForceResistanceMovement
from services.total_power_car_each_hub_service import TotalPowerCarEachHubService
from services.total_resistance_force_movement_service import TotalResistanceForceMovementService
from services.trend_lines_service import TrendLinesService
from services.turnovers_wheel_service import TurnoversWheelsService
from utils.graphic_helper import GraphicHelper

# загружаем данные с файла конфига
from utils.json_helper import JSONHelper


class Main:

    def __init__(self, config: dict = None):
        if config is None:
            file_json = open('source/config.json')
            config = json.load(file_json)

        frequency_turns_per_min = config['data']['frequency_turns_per_min']
        all_dataframes: list[DataFrame] = []
        GraphicHelper.reset_counter()

        # таблица с расчётом полного передаточного числа для каждой передачи/таб. 1. Передаточного числа
        gear_ratio_service = GearRatioService(config['data']['gear_ratio']['hub_1'],
                                              config['data']['gear_ratio']['hub_2'],
                                              config['data']['gear_ratio']['hub_3'],
                                              config['data']['gear_ratio']['hub_4'],
                                              config['data']['gear_ratio']['hub_5'],
                                              config['data']['gear_ratio']['hub_reverse'],
                                              config['data']['gear_ratio']['transfer_case'],
                                              config['data']['gear_ratio']['on_board_gearbox'],
                                              config['data']['gear_ratio']['main_pair'])
        gear_ratio_info = pd.DataFrame()
        gear_ratio_info['Параметры'] = ['Передача 1', 'Передача 2', 'Передача 3', 'Передача 4', 'Передача 5',
                                        'Передача R',
                                        'Раздаточная коробка', 'Бортовой редуктор', 'Главная пара']
        gear_ratio_info['Передаточное число'] = [gear_ratio_service.gear_ratio_hub1, gear_ratio_service.gear_ratio_hub2,
                                                 gear_ratio_service.gear_ratio_hub3, gear_ratio_service.gear_ratio_hub4,
                                                 gear_ratio_service.gear_ratio_hub5,
                                                 gear_ratio_service.gear_ratio_reverse,
                                                 gear_ratio_service.transfer_case, gear_ratio_service.on_board_gearbox,
                                                 gear_ratio_service.main_pair]
        gear_ratio_info['Полное передаточное число'] = [gear_ratio_service.full_gear_ratio_hub1,
                                                        gear_ratio_service.full_gear_ratio_hub2,
                                                        gear_ratio_service.full_gear_ratio_hub3,
                                                        gear_ratio_service.full_gear_ratio_hub4,
                                                        gear_ratio_service.full_gear_ratio_hub5,
                                                        gear_ratio_service.full_gear_ratio_reverse, '-', '-', '-']
        gear_ratio_info.name = 'Передаточные числа'
        all_dataframes.append(gear_ratio_info)

        # формирование данных, где вычислются обороты колёс в минуту относительно кол-ва оборотов двигателя, номера передачи,
        # данных о передаточных числах каждой скорости (таблица из ecxel №2)/таб. 2. Оборотов колеса
        turns_wheels_service = TurnoversWheelsService(gear_ratio_service.full_gear_ratio_hub1,
                                                      gear_ratio_service.full_gear_ratio_hub2,
                                                      gear_ratio_service.full_gear_ratio_hub3,
                                                      gear_ratio_service.full_gear_ratio_hub4,
                                                      gear_ratio_service.full_gear_ratio_hub5,
                                                      gear_ratio_service.full_gear_ratio_reverse,
                                                      list(frequency_turns_per_min))

        turnovers_wheel = pd.DataFrame()
        turnovers_wheel['Частота оборотов двигателя'] = frequency_turns_per_min
        turnovers_wheel['Передача 1'] = turns_wheels_service.turnovers_wheels_hub1
        turnovers_wheel['Передача 2'] = turns_wheels_service.turnovers_wheels_hub2
        turnovers_wheel['Передача 3'] = turns_wheels_service.turnovers_wheels_hub3
        turnovers_wheel['Передача 4'] = turns_wheels_service.turnovers_wheels_hub4
        turnovers_wheel['Передача 5'] = turns_wheels_service.turnovers_wheels_hub5
        turnovers_wheel.name = 'Обороты колеса'
        all_dataframes.append(turnovers_wheel)

        # таблица размерности шин/таб.3. Размерности шин
        width_wheel = config['data']['wheel_info']['profile_width']
        height_wheel = config['data']['wheel_info']['profile_height']
        diameter_wheel = config['data']['wheel_info']['diameter']
        wheel_info_table = pd.DataFrame()
        wheel_info_table['Параметр'] = ['Размер колес', 'Номинальный радиус (м)', 'Статический радиус',
                                        'Динамический радиус']
        nom_radius = 0.0254 * (diameter_wheel / 2) + (width_wheel / 1000) * (height_wheel / 100)
        if height_wheel >= 90:
            tire_crumpling_ratio = 0.8
        elif height_wheel <= 50:
            tire_crumpling_ratio = 0.85
        else:
            tire_crumpling_ratio = 0.814285714
        stat_radius = 0.0254 * (diameter_wheel / 2) + (width_wheel / 1000) * (height_wheel / 100) * tire_crumpling_ratio
        dynamic_radius = nom_radius - ((nom_radius - stat_radius) / 3)
        wheel_info_table['Ширина профиля'] = [width_wheel, nom_radius, stat_radius, dynamic_radius]
        wheel_info_table['Профиль шины'] = [height_wheel, '-', '-', '-']
        wheel_info_table['Диаметр шины'] = [diameter_wheel, '-', '-', '-']
        wheel_info_table.name = 'Размерность шин'
        all_dataframes.append(wheel_info_table)

        # формирование данных, где вычисляется скорость автомобиля относительно кол-ва оборотов двигателя, номера передачи,
        # данных о передаточных числах каждой скорости и параметрах колёс(таблица из ecxel №5) / табл.5. Скорости  от оборотов двигателя км/ч
        wheel_info = config['data']['wheel_info']
        speed_car_service = SpeedCarService(
            wheel_info['profile_width'],
            wheel_info['profile_height'],
            wheel_info['diameter'],
            turnovers_wheel,
            frequency_turns_per_min
        )

        speed_car = pd.DataFrame()
        speed_car['Частота оборотов двигателя'] = frequency_turns_per_min
        speed_car['Передача 1'] = speed_car_service.speed_hub1
        speed_car['Передача 2'] = speed_car_service.speed_hub2
        speed_car['Передача 3'] = speed_car_service.speed_hub3
        speed_car['Передача 4'] = speed_car_service.speed_hub4
        speed_car['Передача 5'] = speed_car_service.speed_hub5
        speed_car.name = 'Скорость автомобиля'
        all_dataframes.append(speed_car)

        # Рассчёт мощности и крутящего момента / таб. Мощьности и крутящего момента
        power_and_torque_info = pd.DataFrame()
        power_and_torque_info['power_and_torque_turns'] = [data['freq_turns_per_min'] for data in config['data']['engine_performance']['measurements']]
        power_and_torque_info['power_and_torque_hms'] = [data['Hm'] for data in config['data']['engine_performance']['measurements']]
        power_and_torque_info['power_and_torque_horse_powers'] = [data['horse_power'] for data in config['data']['engine_performance']['measurements']]

        trend_lines_service = TrendLinesService(power_and_torque_info)
        coefficient_polynom_hm = trend_lines_service.polynom_coefs_hm
        coefficient_polynom_hp = trend_lines_service.polynom_coefs_hp

        # таблица коэффициентов полинома / таблица коэффициентов полинома
        coefficient_polynom = pd.DataFrame()
        coefficient_polynom['Данные'] = ['Коэффициент момент', 'Коэффициент мощность']
        coefficient_polynom['X/5'] = [0.0, 0.0]
        coefficient_polynom['X/4'] = [coefficient_polynom_hm[0][0], coefficient_polynom_hp[0][0]]
        coefficient_polynom['X/3'] = [coefficient_polynom_hm[1][0], coefficient_polynom_hp[1][0]]
        coefficient_polynom['X/2'] = [coefficient_polynom_hm[2][0], coefficient_polynom_hp[2][0]]
        coefficient_polynom['X'] = [coefficient_polynom_hm[3][0], coefficient_polynom_hp[3][0]]
        coefficient_polynom['Собственный коэффициент'] = [coefficient_polynom_hm[4][0], coefficient_polynom_hp[4][0]]

        coefficient_polynom.name = 'Коэффициенты полинома'
        all_dataframes.append(coefficient_polynom)

        power_and_torque_data = PowerAndTorqueService(
            frequency_turns_per_min=list(frequency_turns_per_min)
        )
        power_and_torque = pd.DataFrame()
        power_and_torque['Частота оборотов двигателя'] = power_and_torque_data.frequency_turns_per_min
        power_and_torque['Крутящий момент'] = power_and_torque_data.torques
        power_and_torque['Мощность'] = power_and_torque_data.powers

        power_and_torque.name = 'Мощность и крутящий момент'
        all_dataframes.append(power_and_torque)

        # Рассчёт КПД трансмиссии / таб. Расчета КПД трансмиссии
        number_of_spur_gears = config['data']['number_of_spur_gears']
        number_of_bevel_gears = config['data']['number_of_bevel_gears']
        number_of_cardan_gears = config['data']['number_of_cardan_gears']

        kpd = pd.DataFrame()
        kpd_data = CalculateKPDService(
            number_of_spur_gears=number_of_spur_gears,
            number_of_bevel_gears=number_of_bevel_gears,
            number_of_cardan_gears=number_of_cardan_gears
        )
        kpd['число цилиндрических передач'] = kpd_data.number_of_spur_gears
        kpd['число конических передач'] = kpd_data.number_of_bevel_gears
        kpd['число крестовин кардана'] = kpd_data.number_of_cardan_gears
        kpd['КПД'] = kpd_data.KPD

        kpd.name = 'КПД'
        all_dataframes.append(kpd)

        # Таблица габаритных размеров /таб. Вводных параметров для расчета сопротивления воздуха
        air_resistance_service = AirResistanceService(
            config['data']['dimensions']['car_width'],
            config['data']['dimensions']['car_height'],
            config['data']['dimensions']['streamline_coefficient'],
            speed_car,
            frequency_turns_per_min
        )

        dimensions = pd.DataFrame()
        dimensions['габаритная ширина автомобиля'] = ['габаритная ширина автомобиля', air_resistance_service.width]
        dimensions['габаритная высота автомобиля'] = ['габаритная высота автомобиля', air_resistance_service.height]
        dimensions['коэфициент обтекаемости'] = ['коэфициент обтекаемости',
                                                 air_resistance_service.streamline_coefficient]
        dimensions['площадь Миделева сечения'] = ['площадь Миделева сечения',
                                                  air_resistance_service.midelev_cross_sectional_area]

        dimensions.name = 'Габариты'
        all_dataframes.append(dimensions)

        # таблица сопротивления воздуха / сопротивление воздуха при двидении автомобиля
        air_resistance = pd.DataFrame()
        air_resistance['Частота оборотов двигателя'] = frequency_turns_per_min
        air_resistance['Передача 1'] = air_resistance_service.air_resistance_hub1
        air_resistance['Передача 2'] = air_resistance_service.air_resistance_hub2
        air_resistance['Передача 3'] = air_resistance_service.air_resistance_hub3
        air_resistance['Передача 4'] = air_resistance_service.air_resistance_hub4
        air_resistance['Передача 5'] = air_resistance_service.air_resistance_hub5

        air_resistance.name = 'Сопротивление воздуха'
        all_dataframes.append(air_resistance)

        # таблица крутящего момента на колесе / крутящий момент на колесе
        torque_on_wheel_service = TorqueOnWheelService(gear_ratio_info, power_and_torque, kpd, frequency_turns_per_min)
        torque_on_wheel = pd.DataFrame()
        torque_on_wheel['Частота оборотов двигателя'] = frequency_turns_per_min
        torque_on_wheel['Передача 1'] = torque_on_wheel_service.torque_on_wheel_hub1
        torque_on_wheel['Передача 2'] = torque_on_wheel_service.torque_on_wheel_hub2
        torque_on_wheel['Передача 3'] = torque_on_wheel_service.torque_on_wheel_hub3
        torque_on_wheel['Передача 4'] = torque_on_wheel_service.torque_on_wheel_hub4
        torque_on_wheel['Передача 5'] = torque_on_wheel_service.torque_on_wheel_hub5

        torque_on_wheel.name = 'Крутящий момент на колесе'
        all_dataframes.append(torque_on_wheel)

        # таблица совмещенной мощности на колесе для каждой передачи и сопротивление воздуха
        km_per_hour = config['data']['km_per_hour']
        dependence_torque_on_air_resistance_service = DependenceOfTorqueOnAirResistanceService(km_per_hour, dimensions,
                                                                                               speed_car,
                                                                                               coefficient_polynom,
                                                                                               gear_ratio_info,
                                                                                               kpd)
        dependence_torque_on_air_resistance = pd.DataFrame()
        dependence_torque_on_air_resistance['Км/ч'] = km_per_hour
        dependence_torque_on_air_resistance[
            'Сопротивление воздуха'] = dependence_torque_on_air_resistance_service.air_resistance
        dependence_torque_on_air_resistance[
            'Обороты 1 передачи'] = dependence_torque_on_air_resistance_service.turnovers_hub1
        dependence_torque_on_air_resistance[
            'Крутящий момент 1'] = dependence_torque_on_air_resistance_service.torque_hub1
        dependence_torque_on_air_resistance[
            'Обороты 2 передачи'] = dependence_torque_on_air_resistance_service.turnovers_hub2
        dependence_torque_on_air_resistance[
            'Крутящий момент 2'] = dependence_torque_on_air_resistance_service.torque_hub2
        dependence_torque_on_air_resistance[
            'Обороты 3 передачи'] = dependence_torque_on_air_resistance_service.turnovers_hub3
        dependence_torque_on_air_resistance[
            'Крутящий момент 3'] = dependence_torque_on_air_resistance_service.torque_hub3
        dependence_torque_on_air_resistance[
            'Обороты 4 передачи'] = dependence_torque_on_air_resistance_service.turnovers_hub4
        dependence_torque_on_air_resistance[
            'Крутящий момент 4'] = dependence_torque_on_air_resistance_service.torque_hub4
        dependence_torque_on_air_resistance[
            'Обороты 5 передачи'] = dependence_torque_on_air_resistance_service.turnovers_hub5
        dependence_torque_on_air_resistance[
            'Крутящий момент 5'] = dependence_torque_on_air_resistance_service.torque_hub5

        dependence_torque_on_air_resistance.name = 'Зависимость крутящего момента от сопротивления воздуха'
        all_dataframes.append(dependence_torque_on_air_resistance)

        # таблица масс автомобиля / таблица масс автомобиля
        car_weight = config['data']['weights']['car_weight']
        full_mass = config['data']['weights']['full_mass']
        passenger_seats = config['data']['weights']['passenger_seats']
        mass_table = pd.DataFrame()
        mass_table['Масса автомобиля'] = [car_weight]
        mass_table['Снаряжённая масса'] = [car_weight + 70]
        mass_table['Полная масса'] = [full_mass]
        mass_table['Масса полезного груза'] = [full_mass - (car_weight + 70) - (70 * (passenger_seats - 1))]
        mass_table['Число посадочных мест'] = [passenger_seats]

        mass_table.name = 'Массы автомобиля'
        all_dataframes.append(mass_table)

        # таблица коэффициент сопротивления кочению колеса / табл коэффициент сопротивления качению колеса
        coefficient_rolling_resistance_wheel = pd.DataFrame()
        coefficient_rolling_resistance_wheel['Погодные условия'] = ['1.Хорошее состояние сухого асфальта',
                                                                    '2.Удовлетворительное состояние сухого асфальта',
                                                                    '3.Обледенелелая асфальтная дорога',
                                                                    '4.Гравийая укатаная дорога',
                                                                    '5.Хорошее состояние булыжника',
                                                                    '6.Удовлетворительное состояние булыжника',
                                                                    '7.Сухая укатанная грунтовая дорога',
                                                                    '8.Мокрая укатанная грунтовая дорога']
        coefficient_rolling_resistance_wheel['Минимум'] = [0.008, 0.015, 0.015, 0.02, 0.025, 0.035, 0.025, 0.05]
        coefficient_rolling_resistance_wheel['Максимум'] = [0.015, 0.03, 0.02, 0.025, 0.03, 0.05, 0.035, 0.15]

        coefficient_rolling_resistance_wheel.name = 'Коэффиценты сопротивления кочению колеса'
        all_dataframes.append(coefficient_rolling_resistance_wheel)

        # табл коэффициента влияния скорости в км/ч / табл коэффициента влияния скорости в км/ч
        coefficient_influence_speed = pd.DataFrame()
        coefficient_influence_speed['Тип автомобиля'] = ['Легковой', 'Грузовой']
        coefficient_influence_speed['Км/час минимум'] = [0.00004, 0.00002]
        coefficient_influence_speed['Км/час максимум'] = [0.00005, 0.00003]
        coefficient_influence_speed['М/с минимум'] = [0.00051, 0.00026]
        coefficient_influence_speed['М/с максимум'] = [0.00065, 0.00039]

        coefficient_influence_speed.name = 'Коэффициенты влияния скорости'
        all_dataframes.append(coefficient_influence_speed)

        # таблица коэффициент сопротивления качению / коэффициент сопротивления качению
        rolling_resistance_service = RollingResistanceService(km_per_hour, coefficient_rolling_resistance_wheel,
                                                              mass_table,
                                                              coefficient_influence_speed)
        rolling_resistance = pd.DataFrame()
        rolling_resistance['Км/ч'] = km_per_hour
        rolling_resistance[
            '1.Хорошее состояние сухого асфальта'] = rolling_resistance_service.coef_rolling_resistance_type1
        rolling_resistance[
            '2.Удовлетворительное состояние сухого асфальта'] = rolling_resistance_service.coef_rolling_resistance_type2
        rolling_resistance[
            '3.Обледенелелая асфальтная дорога'] = rolling_resistance_service.coef_rolling_resistance_type3
        rolling_resistance['4.Гравийая укатаная дорога'] = rolling_resistance_service.coef_rolling_resistance_type4
        rolling_resistance['5.Хорошее состояние булыжника'] = rolling_resistance_service.coef_rolling_resistance_type5
        rolling_resistance[
            '6.Удовлетворительное состояние булыжника'] = rolling_resistance_service.coef_rolling_resistance_type6
        rolling_resistance[
            '7.Сухая укатанная грунтовая дорога'] = rolling_resistance_service.coef_rolling_resistance_type7
        rolling_resistance[
            '8.Мокрая укатанная грунтовая дорога'] = rolling_resistance_service.coef_rolling_resistance_type8

        rolling_resistance.name = 'Коэффициенты сопротивления качению'
        all_dataframes.append(rolling_resistance)

        # таблица силы сопротивлению движения / таблица в екселе без названия
        total_resistance_force_movement_service = TotalResistanceForceMovementService(
            [-20, -15, -10, -5, 0, 5, 10, 15, 20],
            full_mass)
        total_resistance_force_movement = pd.DataFrame()
        total_resistance_force_movement['Угол %'] = total_resistance_force_movement_service.angle_array
        total_resistance_force_movement['Сила подъёма'] = total_resistance_force_movement_service.lifting_force
        total_resistance_force_movement.name = 'Суммарные силы сопротивлению движения'
        all_dataframes.append(total_resistance_force_movement)

        # коэффициент сопротивления качению при хорошее состояние сухого асфальта / коэффициент сопротивления качению при хорошее состояние сухого асфальта
        rolling_resistance_coefficient_dry_asphalt_service = RollingResistanceCoefficientDryAsphaltService(
            frequency_turns_per_min, coefficient_rolling_resistance_wheel, coefficient_influence_speed,
            speed_car, mass_table)
        rolling_resistance_coefficient_dry_asphalt = pd.DataFrame()

        rolling_resistance_coefficient_dry_asphalt[
            'Частота об/мин'] = rolling_resistance_coefficient_dry_asphalt_service.frequency_turns_per_min
        rolling_resistance_coefficient_dry_asphalt[
            '1 передача'] = rolling_resistance_coefficient_dry_asphalt_service.rolling_resistance_coefficient_dry_asphalt_hub1
        rolling_resistance_coefficient_dry_asphalt[
            '2 передача'] = rolling_resistance_coefficient_dry_asphalt_service.rolling_resistance_coefficient_dry_asphalt_hub2
        rolling_resistance_coefficient_dry_asphalt[
            '3 передача'] = rolling_resistance_coefficient_dry_asphalt_service.rolling_resistance_coefficient_dry_asphalt_hub3
        rolling_resistance_coefficient_dry_asphalt[
            '4 передача'] = rolling_resistance_coefficient_dry_asphalt_service.rolling_resistance_coefficient_dry_asphalt_hub4
        rolling_resistance_coefficient_dry_asphalt[
            '5 передача'] = rolling_resistance_coefficient_dry_asphalt_service.rolling_resistance_coefficient_dry_asphalt_hub5
        rolling_resistance_coefficient_dry_asphalt.name = 'Коэффициент сопротивления качению при хорошем состояние сухого асфальта'
        all_dataframes.append(rolling_resistance_coefficient_dry_asphalt)

        # суммарная сила сопротивления движению / найдем суммарную силу сопротивления движению
        total_force_resistance_movement_service = TotalForceResistanceMovement(frequency_turns_per_min,
                                                                               rolling_resistance_coefficient_dry_asphalt,
                                                                               air_resistance)
        total_force_resistance_movement = pd.DataFrame()

        total_force_resistance_movement[
            'Частота об/мин'] = total_force_resistance_movement_service.frequency_turns_per_min
        total_force_resistance_movement[
            '1 передача'] = total_force_resistance_movement_service.total_force_resistance_movement_hub1
        total_force_resistance_movement[
            '2 передача'] = total_force_resistance_movement_service.total_force_resistance_movement_hub2
        total_force_resistance_movement[
            '3 передача'] = total_force_resistance_movement_service.total_force_resistance_movement_hub3
        total_force_resistance_movement[
            '4 передача'] = total_force_resistance_movement_service.total_force_resistance_movement_hub4
        total_force_resistance_movement[
            '5 передача'] = total_force_resistance_movement_service.total_force_resistance_movement_hub5

        total_force_resistance_movement.name = 'Суммарная сила сопротивления движения'
        all_dataframes.append(total_force_resistance_movement)

        # суммарная мощьность автомобиля на каждой передаче / суммарная мощьность автомобиля на каждой передаче
        total_power_car_each_hub_service = TotalPowerCarEachHubService(frequency_turns_per_min, torque_on_wheel,
                                                                       total_force_resistance_movement)
        total_power_car_each_hub = pd.DataFrame()

        total_power_car_each_hub['Частота об/мин'] = total_power_car_each_hub_service.frequency_turns_per_min
        total_power_car_each_hub['1 передача'] = total_power_car_each_hub_service.total_power_car_each_hub1
        total_power_car_each_hub['2 передача'] = total_power_car_each_hub_service.total_power_car_each_hub2
        total_power_car_each_hub['3 передача'] = total_power_car_each_hub_service.total_power_car_each_hub3
        total_power_car_each_hub['4 передача'] = total_power_car_each_hub_service.total_power_car_each_hub4
        total_power_car_each_hub['5 передача'] = total_power_car_each_hub_service.total_power_car_each_hub5

        total_power_car_each_hub.name = 'Суммарная мощьность автомобиля на каждой передаче'
        all_dataframes.append(total_power_car_each_hub)

        # # суммарная сила на колесе в идеальных условиях
        # total_force_wheel_ideal_conditions_service = TotalForceWheelIdealConditionsService(km_per_hour,
        #                                                                                    rolling_resistance,
        #                                                                                    speed_car,
        #                                                                                    coefficient_polynom,
        #                                                                                    dependence_torque_on_air_resistance,
        #                                                                                    gear_ratio_info, kpd)
        # total_force_wheel_ideal_conditions = pd.DataFrame()
        # total_force_wheel_ideal_conditions['Км/ч'] = total_force_wheel_ideal_conditions_service.km_per_hour_array
        # total_force_wheel_ideal_conditions[
        #     'Сумма сопротивления'] = total_force_wheel_ideal_conditions_service.air_resistance_array
        # total_force_wheel_ideal_conditions[
        #     'Обороты 1 передача'] = total_force_wheel_ideal_conditions_service.turnovers_hub1
        # total_force_wheel_ideal_conditions[
        #     'Крутящий момент 1 передача'] = total_force_wheel_ideal_conditions_service.force_on_wheel_hub1
        # total_force_wheel_ideal_conditions[
        #     'Обороты 2 передача'] = total_force_wheel_ideal_conditions_service.turnovers_hub2
        # total_force_wheel_ideal_conditions[
        #     'Крутящий момент 2 передача'] = total_force_wheel_ideal_conditions_service.force_on_wheel_hub2
        # total_force_wheel_ideal_conditions[
        #     'Обороты 3 передача'] = total_force_wheel_ideal_conditions_service.turnovers_hub3
        # total_force_wheel_ideal_conditions[
        #     'Крутящий момент 3 передача'] = total_force_wheel_ideal_conditions_service.force_on_wheel_hub3
        # total_force_wheel_ideal_conditions[
        #     'Обороты 4 передача'] = total_force_wheel_ideal_conditions_service.turnovers_hub4
        # total_force_wheel_ideal_conditions[
        #     'Крутящий момент 4 передача'] = total_force_wheel_ideal_conditions_service.force_on_wheel_hub4
        # total_force_wheel_ideal_conditions[
        #     'Обороты 5 передача'] = total_force_wheel_ideal_conditions_service.turnovers_hub5
        # total_force_wheel_ideal_conditions[
        #     'Крутящий момент 5 передача'] = total_force_wheel_ideal_conditions_service.force_on_wheel_hub5
        # total_force_wheel_ideal_conditions.name = 'Суммарная сила на колесе в идеальных условиях'
        # all_dataframes.append(total_force_wheel_ideal_conditions)

        # таблица коэффициентов влияния оборотов двигателя на расход топлива / коэффициент влияния оборотов двигателя на расход топлива
        coefficient_turnovers_to_fuel_service = CoefficientTurnoversToFuelService(frequency_turns_per_min)
        coefficient_turnovers_to_fuel = DataFrame()

        coefficient_turnovers_to_fuel[
            'Частота об/мин'] = coefficient_turnovers_to_fuel_service.frequency_turns_per_min
        coefficient_turnovers_to_fuel[
            'Коэффициенты'] = coefficient_turnovers_to_fuel_service.coefs
        coefficient_turnovers_to_fuel.name = 'Таблица коэффициентов влияния оборотов двигателя на расход топлива'
        all_dataframes.append(coefficient_turnovers_to_fuel)

        # таблица коэффициентов влияния мощьности на расход топлива / коэффициент влияния мощьности на расход топлива
        coefficient_influence_power_on_fuel_consumption_service = CoefficientInfluencePowerOnFuelConsumptionService(
            torque_on_wheel, air_resistance)
        influence_power_on_fuel_consumption = pd.DataFrame()

        influence_power_on_fuel_consumption[
            'Частота об/м'] = coefficient_influence_power_on_fuel_consumption_service.frequency_array
        influence_power_on_fuel_consumption[
            'Передача 1'] = coefficient_influence_power_on_fuel_consumption_service.coefs_hub1
        influence_power_on_fuel_consumption[
            'Передача 2'] = coefficient_influence_power_on_fuel_consumption_service.coefs_hub2
        influence_power_on_fuel_consumption[
            'Передача 3'] = coefficient_influence_power_on_fuel_consumption_service.coefs_hub3
        influence_power_on_fuel_consumption[
            'Передача 4'] = coefficient_influence_power_on_fuel_consumption_service.coefs_hub4
        influence_power_on_fuel_consumption[
            'Передача 5'] = coefficient_influence_power_on_fuel_consumption_service.coefs_hub5

        influence_power_on_fuel_consumption.name = 'Коэффициенты влияния мощности на расход топлива'
        all_dataframes.append(influence_power_on_fuel_consumption)

        # таблица расхода топлива в час / расход топлива автомобиля в час
        fuel_consumption_service = FuelConsumptionService(frequency_turns_per_min,
                                                          coefficient_turnovers_to_fuel,
                                                          influence_power_on_fuel_consumption,
                                                          total_force_resistance_movement)
        fuel_consumption = pd.DataFrame()

        fuel_consumption['Частота об/м'] = fuel_consumption_service.frequency_turns_per_min
        fuel_consumption['Передача 1'] = fuel_consumption_service.fuel_consumption_hub1
        fuel_consumption['Передача 2'] = fuel_consumption_service.fuel_consumption_hub2
        fuel_consumption['Передача 3'] = fuel_consumption_service.fuel_consumption_hub3
        fuel_consumption['Передача 4'] = fuel_consumption_service.fuel_consumption_hub4
        fuel_consumption['Передача 5'] = fuel_consumption_service.fuel_consumption_hub5

        fuel_consumption.name = 'Расход топлива автомобиля в час'
        all_dataframes.append(fuel_consumption)

        # таблица расхода топлива на 100 км при минимальной нагрузке / расход топлива автомобиля на 100км  для минимальной нагрузки
        fuel_consumption_100km_minimum_load_service = FuelConsumption100kmMinimumLoadService(frequency_turns_per_min,
                                                                                             fuel_consumption,
                                                                                             speed_car)
        fuel_consumption_100km_minimum_load = DataFrame()
        fuel_consumption_100km_minimum_load[
            'Частота об/м'] = fuel_consumption_100km_minimum_load_service.frequency_turns_per_min
        fuel_consumption_100km_minimum_load[
            'Передача 1'] = fuel_consumption_100km_minimum_load_service.calculate_fuel_consumption_hub1
        fuel_consumption_100km_minimum_load[
            'Передача 2'] = fuel_consumption_100km_minimum_load_service.calculate_fuel_consumption_hub2
        fuel_consumption_100km_minimum_load[
            'Передача 3'] = fuel_consumption_100km_minimum_load_service.calculate_fuel_consumption_hub3
        fuel_consumption_100km_minimum_load[
            'Передача 4'] = fuel_consumption_100km_minimum_load_service.calculate_fuel_consumption_hub4
        fuel_consumption_100km_minimum_load[
            'Передача 5'] = fuel_consumption_100km_minimum_load_service.calculate_fuel_consumption_hub5

        fuel_consumption_100km_minimum_load.name = 'Расход топлива автомобиля на 100км  для минимальной нагрузки'
        all_dataframes.append(fuel_consumption_100km_minimum_load)

        # таблица расчёта динамического фактора
        dynamic_factor_service = DynamicFactorService(km_per_hour, speed_car, coefficient_polynom, wheel_info_table,
                                                      dependence_torque_on_air_resistance, gear_ratio_info, kpd)
        dynamic_factor = pd.DataFrame()
        dynamic_factor['Км/ч'] = dynamic_factor_service.km_per_hour_array
        dynamic_factor['Обороты 1 передача'] = dynamic_factor_service.turnovers_hub1
        dynamic_factor['Крутящий момент 1 передача'] = dynamic_factor_service.torque_hub1
        dynamic_factor['Топливо 1 передача'] = dynamic_factor_service.fuel_hub1
        dynamic_factor['Обороты 2 передача'] = dynamic_factor_service.turnovers_hub2
        dynamic_factor['Крутящий момент 2 передача'] = dynamic_factor_service.torque_hub2
        dynamic_factor['Топливо 2 передача'] = dynamic_factor_service.fuel_hub2
        dynamic_factor['Обороты 3 передача'] = dynamic_factor_service.turnovers_hub3
        dynamic_factor['Крутящий момент 3 передача'] = dynamic_factor_service.torque_hub3
        dynamic_factor['Топливо 3 передача'] = dynamic_factor_service.fuel_hub3
        dynamic_factor['Обороты 4 передача'] = dynamic_factor_service.turnovers_hub4
        dynamic_factor['Крутящий момент 4 передача'] = dynamic_factor_service.torque_hub4
        dynamic_factor['Топливо 4 передача'] = dynamic_factor_service.fuel_hub4
        dynamic_factor['Обороты 5 передача'] = dynamic_factor_service.turnovers_hub5
        dynamic_factor['Крутящий момент 5 передача'] = dynamic_factor_service.torque_hub5
        dynamic_factor['Топливо 5 передача'] = dynamic_factor_service.fuel_hub5
        dynamic_factor.name = 'Динамический фактор'
        all_dataframes.append(dynamic_factor)

        self.response = JSONHelper().dataframes_to_dict(all_dataframes)


if __name__ == '__main__':
    Main()
