#pragma once
#include <string>
#include "esp_http_server.h"

std::string webSystemJson();
void webSystemSet(httpd_req_t *req);

std::string webStatusJson();

std::string webClutchJson();
void webClutchSet(httpd_req_t *req);

std::string webBrakeJson();
void webBrakeSet(httpd_req_t *req);

void disableMotors();