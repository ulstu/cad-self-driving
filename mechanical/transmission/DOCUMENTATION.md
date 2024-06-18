# БЭКЭНД

Вызовы сервисов происходят в файле [main.py](backend/main.py)

Сервер написан на cherrypy и запускается в файле [app.py](backend/app.py)

Конфиг для сервера [config.py](backend/config.py)

Сервер принимает GET и POST запрос, на GET возвращает все вычисления в виде JSON по шаблонным данным
и сохраняет графики в static/graphics. Графики строятся с использованием matplotlib, 
для их сохранения используется готовый метод savefig.
На POST запрос - достаёт тело запроса(json) и рассчитывает все параметры относительно переданных данных, 
отдает JSON ответ. 
Пример конфига лежит в папке source.

Все сервисы по рассчёту таблиц содержатся в папке services
- air_resistance_service (Рассчёт габаритных размеров и сопротивления воздуха)
- calculate_KPD_service (Сервис для рассчёта КПД трансмиссии)
- coefficient_influence_power_on_fuel_consumption_service (Сервис по расчёту коэффициентов влияния мощности на расход топлива)
- coefficient_turnovers_to_fuel_service (Сервис для формирования таблицы коэффициентов влияния оборотов двигателя на расход топлива)
- dependence_torque_on_air_resistance_service (Рассчитывает зависимость крутящего момента от сопротивления воздуха)
- dynamic_factor_service (Рассчитывает Динамический фактор)
- fuel_consumption_100km_minimum_load_service (Расход топлива автомобиля на 100км  для минимальной нагрузки)
- fuel_consumption_service (Сервис по расчёту расхода топлива автомобиля в час)
- gear_ratio_service (Сервис по высчитыванию полного передаточного числа для каждой передачи)
- power_and_torque (Сервис рассчёта мощности и крутящего момента)
- rolling_resistance_coefficient_dry_asphalt_service (Сервис для формирования таблицы коэффициент сопротивления качению при хорошее состояние сухого асфальта)
- rolling_resistance_service (Рассчитывает Коэффициенты сопротивления качению)
- speed_car_service (Рассчёт скорости от оборотов двигателя км/ч)
- torque_on_wheel_service (Сервис для формирования данных о крутящем момменте на колесе относиельно передачи и кол-ва оборотов двигателя)
- total_force_resistance_movement_service (Сервис по рассчёту суммарной силы сопротивления движению)
- total_force_wheel_ideal_conditions_service (Суммарная сила на колесе в идеальных условиях)
- total_power_car_each_hub_service (Суммарная мощность автомобиля на каждой передаче)
- total_resistance_force_movement_service (Сервис силы сопротивлению движения)
- trend_lines_service (Класс для построения линий трендов)
- turnovers_wheel_service (Обороты колеса от частоты оборотов двигателя)
