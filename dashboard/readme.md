# Панель с метриками для беспилотного автомобиля

## Список метрик
Панель с метриками отображает основые параметры проезда ВАТС:
* state - Текущее состяоние ВАТС
* lat - Широта
* lon - Долгота
* angle - Азимут
* speed - Скорость
* steering - Угол поворота руля
* point_dist - Расстояние до ближайшей точки
* cur_path_segment - Текущий сегмент пути
* cur_point - Текущая точка пути
* path_len - Длина спланированного пути

## Установка
* Для запуска панели управления необходимо установить [Prometheus](https://prometheus.io/docs/prometheus/latest/installation/). 
* В рабочую папку необходимо скопировать конфигурационный файл `prometheus.yml`
* Установить [Graphana](https://grafana.com/docs/grafana/latest/setup-grafana/installation/)
* Импортировать дашборд из файла graphana.json по [инструкции](https://grafana.com/docs/grafana/latest/dashboards/build-dashboards/import-dashboards/)
* Запустите Prometheus
* Запустите Graphana по адресу http://localhost:3000
Метрики собираются автоматически при каждом запуске решения
