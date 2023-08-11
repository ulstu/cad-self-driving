FROM deephdc/tensorflow:1.12.0-gpu-py36
# set work directory
WORKDIR /usr/src/app
# set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1
# install psycopg2 dependencies
RUN rm /etc/apt/sources.list.d/cuda.list
RUN rm /etc/apt/sources.list.d/nvidia-ml.list
RUN apt-get update \
    && apt-get install ffmpeg libsm6 libxext6  -y
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata
RUN apt install python3-opencv -y

RUN pip3 install cmake
#RUN pip3 install --only-binary opencv-python
#RUN pip3 install --only-binary opencv-python-headless
RUN pip3 install nvidia-ml-py3
RUN pip3 install scikit-image scikit-learn
#RUN pip3 install --upgrade pip
COPY ./torch-1.10.2-cp36-cp36m-manylinux1_x86_64.whl ./torch-1.10.2-cp36-cp36m-manylinux1_x86_64.whl
RUN pip3 install ./torch-1.10.2-cp36-cp36m-manylinux1_x86_64.whl
RUN pip3 install --default-timeout=900 fastseg
COPY ./pyzed-3.5-cp36-cp36m-linux_x86_64.whl ./pyzed-3.5-cp36-cp36m-linux_x86_64.whl
RUN pip3 install ./pyzed-3.5-cp36-cp36m-linux_x86_64.whl
#COPY . .
#RUN cd "/usr/local/zed/" && python get_python_api.py
#RUN pip install -r requirements.txt
# copy project
#RUN mkdir /root/.config/ \
#    && mkdir /root/.config/gspread
#COPY service_account.json /root/.config/gspread/service_account.json
#RUN python manage.py migrate
#EXPOSE 8008
