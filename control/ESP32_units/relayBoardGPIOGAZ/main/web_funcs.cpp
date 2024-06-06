#include "web_funcs.h"
#include "WheelFollower.h"
#include "Can.h"

static const char *TAG = "WebFuncs";


std::string webSystemJson() {
    std::string json = "{";
    json += "\"dhcp\":\"";
    json += (systemParams.dhcp == true ? "checked" : "0");

    json += "\",\"ip1\":\"";
    json += std::to_string(systemParams.ip[0]);
    json += "\",\"ip2\":\"";
    json += std::to_string(systemParams.ip[1]);
    json += "\",\"ip3\":\"";
    json += std::to_string(systemParams.ip[2]);
    json += "\",\"ip4\":\"";
    json += std::to_string(systemParams.ip[3]);

    json += "\",\"ma1\":\"";
    json += std::to_string(systemParams.mask[0]);
    json += "\",\"ma2\":\"";
    json += std::to_string(systemParams.mask[1]);
    json += "\",\"ma3\":\"";
    json += std::to_string(systemParams.mask[2]);
    json += "\",\"ma4\":\"";
    json += std::to_string(systemParams.mask[3]);

    json += "\",\"gw1\":\"";
    json += std::to_string(systemParams.gateway[0]);
    json += "\",\"gw2\":\"";
    json += std::to_string(systemParams.gateway[1]);
    json += "\",\"gw3\":\"";
    json += std::to_string(systemParams.gateway[2]);
    json += "\",\"gw4\":\"";
    json += std::to_string(systemParams.gateway[3]);

    json += "\"}";
    return json;
}

void webSystemSet(httpd_req_t *req) {
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        char*  buf;
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[50];
            if (httpd_query_key_value(buf, "dhcp", param, sizeof(param)) == ESP_OK) {
                systemParams.dhcp = std::stoi(param);
                if (!systemParams.dhcp) {
                    if (httpd_query_key_value(buf, "ip1", param, sizeof(param)) == ESP_OK) {
                        systemParams.ip[0] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ip2", param, sizeof(param)) == ESP_OK) {
                        systemParams.ip[1] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ip3", param, sizeof(param)) == ESP_OK) {
                        systemParams.ip[2] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ip4", param, sizeof(param)) == ESP_OK) {
                        systemParams.ip[3] = std::stoi(param);
                    }

                    if (httpd_query_key_value(buf, "ma1", param, sizeof(param)) == ESP_OK) {
                        systemParams.mask[0] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ma2", param, sizeof(param)) == ESP_OK) {
                        systemParams.mask[1] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ma3", param, sizeof(param)) == ESP_OK) {
                        systemParams.mask[2] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ma4", param, sizeof(param)) == ESP_OK) {
                        systemParams.mask[3] = std::stoi(param);
                    }

                    if (httpd_query_key_value(buf, "gw1", param, sizeof(param)) == ESP_OK) {
                        systemParams.gateway[0] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "gw2", param, sizeof(param)) == ESP_OK) {
                        systemParams.gateway[1] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "gw3", param, sizeof(param)) == ESP_OK) {
                        systemParams.gateway[2] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "gw4", param, sizeof(param)) == ESP_OK) {
                        systemParams.gateway[3] = std::stoi(param);
                    }
                }
            }
            saveSystemParams();
        }
    }
}

std::string webPedalJson() {
    std::string json = "{";

    json += "\"lowPosition\":\"";
    json += std::to_string(pedalParams.lowPosition);
    json += "\",\"highPostition\":\"";
    json += std::to_string(pedalParams.highPostition);
    json += "\",\"DACVoltage\":\"";
    json += std::to_string(pedalParams.DACVoltage);

    json += "\"}";
    return json;
}

void webPedalParamsSet(httpd_req_t *req) {
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        char*  buf;
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[50];
            if (httpd_query_key_value(buf, "lowPosition", param, sizeof(param)) == ESP_OK) {
                pedalParams.lowPosition = std::stof(param);
            }
            if (httpd_query_key_value(buf, "highPostition", param, sizeof(param)) == ESP_OK) {
                pedalParams.highPostition = std::stof(param);
            }
            if (httpd_query_key_value(buf, "DACVoltage", param, sizeof(param)) == ESP_OK) {
                pedalParams.DACVoltage = std::stof(param);
            }
            savePedalParams();
        }
    }
}

std::string getWheelStateJSON() {
    std::string json = "{";

    json += "\"resisorPosition\":\"";
    json += std::to_string(WheelFollower::resistor_position);
    json += "\",\"wheelRotation\":\"";
    json += std::to_string(WheelFollower::absolute_angle);
    json += "\",\"wheelTarget\":\"";
    json += std::to_string(WheelFollower::target_position);

    json += "\"}";
    return json;
}

void webSetWheelCalibration(httpd_req_t *req) {
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        char*  buf;
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[50];
            if (httpd_query_key_value(buf, "wheelCalibrationLeft", param, sizeof(param)) == ESP_OK) {
                wheelFollowerParams.leftPosition = std::stoi(param);
            }
            if (httpd_query_key_value(buf, "wheelCalibrationCenter", param, sizeof(param)) == ESP_OK) {
                wheelFollowerParams.centerPosition = std::stoi(param);
            }
            if (httpd_query_key_value(buf, "wheelCalibrationRight", param, sizeof(param)) == ESP_OK) {
                wheelFollowerParams.rightPosition = std::stoi(param);
            }
            if (httpd_query_key_value(buf, "wheelCalibrationRange", param, sizeof(param)) == ESP_OK) {
                wheelFollowerParams.angleRange = std::stoi(param);
            }
            if (httpd_query_key_value(buf, "position_P", param, sizeof(param)) == ESP_OK) {
                wheelFollowerParams.position_P = std::stof(param);
            }
            if (httpd_query_key_value(buf, "position_I", param, sizeof(param)) == ESP_OK) {
                wheelFollowerParams.position_I = std::stof(param);
            }
            if (httpd_query_key_value(buf, "position_D", param, sizeof(param)) == ESP_OK) {
                wheelFollowerParams.position_D = std::stof(param);
            }
            if (httpd_query_key_value(buf, "ramp", param, sizeof(param)) == ESP_OK) {
                wheelFollowerParams.ramp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "limit", param, sizeof(param)) == ESP_OK) {
                wheelFollowerParams.limit = std::stof(param);
            }
            if (httpd_query_key_value(buf, "filter", param, sizeof(param)) == ESP_OK) {
                wheelFollowerParams.filter = std::stof(param);
            }

            saveWheelCalibrationParams();

            WheelFollower::set_params(
                wheelFollowerParams.position_P, 
                wheelFollowerParams.position_I, 
                wheelFollowerParams.position_D, 
                wheelFollowerParams.ramp, 
                wheelFollowerParams.limit, 
                wheelFollowerParams.filter
            );
        }
    }
}

std::string webWheelCalibrationJSON() {
    std::string json = "{";

    json += "\"wheelCalibrationLeft\":\"";
    json += std::to_string(wheelFollowerParams.leftPosition);
    json += "\",\"wheelCalibrationCenter\":\"";
    json += std::to_string(wheelFollowerParams.centerPosition);
    json += "\",\"wheelCalibrationRight\":\"";
    json += std::to_string(wheelFollowerParams.rightPosition);
    json += "\",\"wheelCalibrationRange\":\"";
    json += std::to_string(wheelFollowerParams.angleRange);
    json += "\",\"position_P\":\"";
    json += std::to_string(wheelFollowerParams.position_P);
    json += "\",\"position_I\":\"";
    json += std::to_string(wheelFollowerParams.position_I);
    json += "\",\"position_D\":\"";
    json += std::to_string(wheelFollowerParams.position_D);
    json += "\",\"ramp\":\"";
    json += std::to_string(wheelFollowerParams.ramp);
    json += "\",\"limit\":\"";
    json += std::to_string(wheelFollowerParams.limit);
    json += "\",\"filter\":\"";
    json += std::to_string(wheelFollowerParams.filter);

    json += "\"}";
    return json;
}

std::string getCANStateJSON() {
    std::string json = "{";

    json += "\"carTacho\":\"";
    json += std::to_string(CAN::tacho);
    json += "\",\"carSpeed\":\"";
    json += std::to_string(CAN::speed);

    json += "\"}";
    return json;
}
