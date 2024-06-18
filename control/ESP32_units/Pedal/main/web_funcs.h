#pragma once
#include <string>
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvstorage.h"

std::string getWheelStateJSON();
std::string webWheelCalibrationJSON();
void webSetWheelCalibration(httpd_req_t *req);

std::string webSystemJson();
void webSystemSet(httpd_req_t *req);

std::string webGearboxJson();
void webGearboxSet(httpd_req_t *req);

std::string webPedalJson();
void webPedalParamsSet(httpd_req_t *req);

std::string webStatusJson();

std::string getCANStateJSON();