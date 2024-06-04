FROM python:3.10

ENV WORKDIR=/usr/src/app
WORKDIR ${WORKDIR}

COPY . ${WORKDIR}

COPY /backend/requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir --upgrade -r /app/requirements.txt

EXPOSE 8080
