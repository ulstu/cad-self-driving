FROM php:7.4-fpm
WORKDIR "/application"

ARG DOCKER_GID=1000
ARG DOCKER_UID=1000

# Fix debconf warnings upon build
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y \
        libpq-dev \
        libfreetype6-dev \
        libjpeg62-turbo-dev \
        libmcrypt-dev \
        libzip-dev \
        libpng-dev

RUN docker-php-ext-install pgsql \
    && docker-php-ext-install pdo_pgsql \
    && docker-php-ext-install bcmath zip

RUN docker-php-ext-configure gd --with-freetype --with-jpeg \
    && docker-php-ext-install -j$(nproc) gd

RUN groupadd -g $DOCKER_GID dev
RUN useradd -g $DOCKER_GID -u $DOCKER_UID dev

RUN mkdir /home/dev
RUN chown -R dev:dev /home/dev

COPY "docker/php-fpm/php-ini-overrides.ini" "/etc/php/7.4/fpm/conf.d/99-overrides.ini"

RUN chown -R dev:dev /application

RUN curl -sS https://getcomposer.org/installer | php -- --install-dir=/usr/local/bin --filename=composer

USER root
