# Открытые библиотеки для проектирования беспилотных наземных автомобилей

## Структура проекта со ссылками на руководства и исходные коды
* **[simulation/webots_ros2_suv](simulation/webots_ros2_suv)** - программный пакет ROS2 основное решение для системы управления как виртуальным автомобилем на виртуальном полигоне webots, так и реальным автомобилем
* **[simulation/webots_ros2_suv/map-server](simulation/webots_ros2_suv/map-server)** - описание компиляции и запуска web-панели управления роботом
* **[simulation/robot_interfaces](simulation/robot_interfaces)** - интерфейсы (типы данных) для решения ROS2
* **[simulation/pcl_maps](simulation/pcl_maps)** - программный пакет ROS2 для кластеризации облака точек
* **[simulation/web_platform/docker](simulation/web_platform/docker)** - средства развертывания симулятора в веб-среде на основе контейнеризации docker
* **[simulation/web_platform/robot_benchmark](simulation/web_platform/robot_benchmark)** - онлайн платформа для проведения соревнований
* **[control](control)** - программный код для управления электронными компонентами
* **[drivers](drivers)** - драйвера лидара, камеры ZED, GNSS системы
* **[mechanical/3d_models](mechanical/3d_models)** - 3d модели узлов и агрегатов для переоборудования типового автомобиля ГАЗель Next
* **[mechanical/schema](mechanical/schema)** - чертежи и документы узлов и агергатов для переоборудования типового автомобиля ГАЗель Next
* **[mechanical/transmission](mechanical/transmission)** - программные средства САПР для расчета момента переключения передач для роботизированной АКПП
* **[navigation/sdgazelle](navigation/sdgazelle)** - программный код управления беспилотным автомобилем на примере задачи движения по пересеченной местности
* **[navigation/map_utils](navigation/map_utils)** - утилиты понижения дискретизации точек GPS планируемого пути
* **[network](network)** - программные средства связи беспилотных автомобилей 
* **[perception](perception)** - компоненты сенсорики беспилотного автомобиля в части распознавания изображений и сигналов лидара, утилита настройки матрицы гомографии
* **[perception/train_doc](perception/train_doc/yolo_train/)** - описание процесса обучения модели YOLOv8 новым объектам с разметкой данных 
* **[dashboard](dashboard)** - панель мониторинга метрик вождения, построенная на базе prometheus и graphana


# Лицензия
Программное обеспечение предоставляется по [лицензии](LICENSE) MIT