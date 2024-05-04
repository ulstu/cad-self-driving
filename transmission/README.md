# KPP_calculation
Разворачивание проекта в докере
~~~
cd .deploy
docker compose build
docker compose up -d
~~~

После запуска контейнеров перейти на http://localhost:3001/

Бэкэнд запускается на 80 порту.

# Для локальной разработки 
Backend (Python 3.10)
~~~
cd backend
pip install -r requirements.txt
python app.py
~~~
# Документация к проекту
[DOCUMENTATION.md](DOCUMENTATION.md)

# Troubleshooting:
Если добавили какую-то питон библиотеку, обновите список библиотек, используемых в проекте

- pip freeze > requirements.txt

Если при разработке в backend проблема с импортами своих модулей

- Mark directory backend as source root and resource root 