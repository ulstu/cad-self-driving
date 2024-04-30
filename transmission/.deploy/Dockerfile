FROM ubuntu:latest

ENV WORKDIR=/usr/src/app
WORKDIR ${WORKDIR}

COPY . ${WORKDIR}

RUN apt update
RUN apt install python3 python3-pip -y

EXPOSE 8080
