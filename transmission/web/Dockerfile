FROM node:lts-alpine

ENV WORKDIR=/usr/src/app
WORKDIR ${WORKDIR}

COPY web/package.json .

RUN yarn 

COPY . ${WORKDIR}

EXPOSE 5173