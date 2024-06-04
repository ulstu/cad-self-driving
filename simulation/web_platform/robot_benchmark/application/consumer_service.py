import json
import os
import time
import urllib.request as req
from urllib.error import URLError

print("Служба запущена!")
os.chdir("../docker/webots")
while True:
    try:
        url = 'http://localhost:8000/api/commands/'
        response = req.urlopen(url)
    except URLError:
        print("Сервер недоступен!")
    else:
        data = response.read()
        data_dict = json.loads(data)
        if data_dict:
            print(data_dict['command'])
            os.system(data_dict['command'])
    finally:
        time.sleep(5)
