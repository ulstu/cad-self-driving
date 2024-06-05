#include "web_funcs.h"
#include "esp_log.h"
#include "NVStorage.h"
#include "FlySkyIBus.h"
#include "Estop.h"
#include "StateMachine.h"
#include "Engine.h"
#include "Motors.h"
#include "Brake.h"
#include "Clutch.h"

static const char *TAG = "WebFuncs";


std::string webSystemJson() {
    std::string json = "{";
    json += "\"dhcp\":\"";
    json += (NVStorage::systemParams.dhcp == true ? "checked" : "0");

    json += "\",\"ip1\":\"";
    json += std::to_string(NVStorage::systemParams.ip[0]);
    json += "\",\"ip2\":\"";
    json += std::to_string(NVStorage::systemParams.ip[1]);
    json += "\",\"ip3\":\"";
    json += std::to_string(NVStorage::systemParams.ip[2]);
    json += "\",\"ip4\":\"";
    json += std::to_string(NVStorage::systemParams.ip[3]);

    json += "\",\"ma1\":\"";
    json += std::to_string(NVStorage::systemParams.mask[0]);
    json += "\",\"ma2\":\"";
    json += std::to_string(NVStorage::systemParams.mask[1]);
    json += "\",\"ma3\":\"";
    json += std::to_string(NVStorage::systemParams.mask[2]);
    json += "\",\"ma4\":\"";
    json += std::to_string(NVStorage::systemParams.mask[3]);

    json += "\",\"gw1\":\"";
    json += std::to_string(NVStorage::systemParams.gateway[0]);
    json += "\",\"gw2\":\"";
    json += std::to_string(NVStorage::systemParams.gateway[1]);
    json += "\",\"gw3\":\"";
    json += std::to_string(NVStorage::systemParams.gateway[2]);
    json += "\",\"gw4\":\"";
    json += std::to_string(NVStorage::systemParams.gateway[3]);

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
                NVStorage::systemParams.dhcp = std::stoi(param);
                if (!NVStorage::systemParams.dhcp) {
                    if (httpd_query_key_value(buf, "ip1", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.ip[0] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ip2", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.ip[1] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ip3", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.ip[2] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ip4", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.ip[3] = std::stoi(param);
                    }

                    if (httpd_query_key_value(buf, "ma1", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.mask[0] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ma2", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.mask[1] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ma3", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.mask[2] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "ma4", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.mask[3] = std::stoi(param);
                    }

                    if (httpd_query_key_value(buf, "gw1", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.gateway[0] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "gw2", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.gateway[1] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "gw3", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.gateway[2] = std::stoi(param);
                    }
                    if (httpd_query_key_value(buf, "gw4", param, sizeof(param)) == ESP_OK) {
                        NVStorage::systemParams.gateway[3] = std::stoi(param);
                    }
                }
            }
            NVStorage::saveSystemParams();
        }
    }
}

std::string webBrakeJson() {
    std::string json = "{";
    json += "\"disabledPos\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.disabledPos);

    json += "\",\"zeroPos\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.zeroPos);

    json += "\",\"pressedPos\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.pressedPos);
    
    json += "\",\"curProp\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.curProp);
    
    json += "\",\"curIntegral\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.curIntegral);
    
    json += "\",\"curDiff\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.curDiff);
    
    json += "\",\"curRamp\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.curRamp);
    
    json += "\",\"curLimit\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.curLimit);
    
    json += "\",\"curFilter\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.curFilter);
    
    json += "\",\"velProp\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.velProp);
    
    json += "\",\"velIntegral\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.velIntegral);
    
    json += "\",\"velDiff\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.velDiff);

    json += "\",\"velRamp\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.velRamp);

    json += "\",\"velLimit\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.velLimit);

    json += "\",\"velFilter\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.velFilter);

    json += "\",\"posProp\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.posProp);

    json += "\",\"posLimit\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.posLimit);

    json += "\",\"curOffset\":\"";
    json += std::to_string(NVStorage::brakeMotorParams.curOffset);

    json += "\"}";
    return json;
}

void webBrakeSet(httpd_req_t *req) {
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        char*  buf;
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[50];
            if (httpd_query_key_value(buf, "disabledPos", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.disabledPos = std::stof(param);
            }
            if (httpd_query_key_value(buf, "zeroPos", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.zeroPos = std::stof(param);
            }
            if (httpd_query_key_value(buf, "pressedPos", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.pressedPos = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curProp", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.curProp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curIntegral", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.curIntegral = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curDiff", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.curDiff = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curRamp", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.curRamp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curLimit", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.curLimit = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curFilter", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.curFilter = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velProp", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.velProp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velIntegral", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.velIntegral = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velDiff", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.velDiff = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velRamp", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.velRamp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velLimit", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.velLimit = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velFilter", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.velFilter = std::stof(param);
            }
            if (httpd_query_key_value(buf, "posProp", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.posProp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "posLimit", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.posLimit = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curOffset", param, sizeof(param)) == ESP_OK) {
                NVStorage::brakeMotorParams.curOffset = std::stof(param);
            }

            NVStorage::saveBrakeMotorParams();
            set_brake_motor_params();
            brake_motor.pidReset();
            Brake::setParams(NVStorage::brakeMotorParams.velLimit, NVStorage::brakeMotorParams.zeroPos);
        }
    }
}


std::string webClutchJson() {
    std::string json = "{";
    json += "\"disabledPos\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.disabledPos);

    json += "\",\"zeroPos\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.zeroPos);

    json += "\",\"fullEngPos\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.fullEngPos);

    json += "\",\"minEngPos\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.minEngPos);

    json += "\",\"pressedPos\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.pressedPos);
    
    json += "\",\"curProp\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.curProp);
    
    json += "\",\"curIntegral\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.curIntegral);
    
    json += "\",\"curDiff\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.curDiff);
    
    json += "\",\"curRamp\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.curRamp);
    
    json += "\",\"curLimit\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.curLimit);
    
    json += "\",\"curFilter\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.curFilter);
    
    json += "\",\"velProp\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.velProp);
    
    json += "\",\"velIntegral\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.velIntegral);
    
    json += "\",\"velDiff\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.velDiff);

    json += "\",\"velRamp\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.velRamp);

    json += "\",\"velLimit\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.velLimit);

    json += "\",\"velFilter\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.velFilter);

    json += "\",\"posProp\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.posProp);

    json += "\",\"posLimit\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.posLimit);

    json += "\",\"curOffset\":\"";
    json += std::to_string(NVStorage::clutchMotorParams.curOffset);

    json += "\"}";
    return json;
}

void webClutchSet(httpd_req_t *req) {
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        char*  buf;
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[50];
            if (httpd_query_key_value(buf, "disabledPos", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.disabledPos = std::stof(param);
            }
            if (httpd_query_key_value(buf, "zeroPos", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.zeroPos = std::stof(param);
            }
            if (httpd_query_key_value(buf, "fullEngPos", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.fullEngPos = std::stof(param);
            }
            if (httpd_query_key_value(buf, "minEngPos", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.minEngPos = std::stof(param);
            }
            if (httpd_query_key_value(buf, "pressedPos", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.pressedPos = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curProp", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.curProp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curIntegral", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.curIntegral = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curDiff", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.curDiff = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curRamp", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.curRamp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curLimit", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.curLimit = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curFilter", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.curFilter = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velProp", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.velProp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velIntegral", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.velIntegral = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velDiff", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.velDiff = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velRamp", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.velRamp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velLimit", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.velLimit = std::stof(param);
            }
            if (httpd_query_key_value(buf, "velFilter", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.velFilter = std::stof(param);
            }
            if (httpd_query_key_value(buf, "posProp", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.posProp = std::stof(param);
            }
            if (httpd_query_key_value(buf, "posLimit", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.posLimit = std::stof(param);
            }
            if (httpd_query_key_value(buf, "curOffset", param, sizeof(param)) == ESP_OK) {
                NVStorage::clutchMotorParams.curOffset = std::stof(param);
            }

            NVStorage::saveClutchMotorParams();
            set_clutch_motor_params();
            clutch_motor.pidReset();
            Clutch::setParams(NVStorage::clutchMotorParams.disabledPos, NVStorage::clutchMotorParams.zeroPos, NVStorage::clutchMotorParams.fullEngPos, NVStorage::clutchMotorParams.minEngPos, NVStorage::clutchMotorParams.pressedPos);
        }
    }
}

std::string webStatusJson() {
    std::string json = "{";
    json += "\"rcConnection\":\"";
    json += (FlySkyIBus::available() ? "checked" : "0");

    json += "\",\"eStop\":\"";
    json += (Estop::normal() ? "checked" : "0");

    json += "\",\"driveMode\":\"";
    switch (StateMachine::get_current_control_mode()) {
        case StateMachine::EStop:
            json += "E-Stop";
            break;
        case StateMachine::Manual:
            json += "С пульта";
            break;
        case StateMachine::Auto:
            json += "С компьютера";
            break;
        default:
            json += "Неизвестен";
            break;
    }

    json += "\",\"engineState\":\"";
    switch (Engine::getStatus()) {
        case Engine::disabled:
            json += "Отключен";
            break;
        case Engine::preparing:
            json += "Подготовка";
            break;
        case Engine::starting:
            json += "Запускается";
            break;
        case Engine::engaged:
            json += "Запущен";
            break;
        default:
            json += "Неизвестно";
            break;
    }

    json += "\",\"brakeMotorEnable\":\"";
    json += (brake_motor_driver.getState() ? "checked" : "0");

    json += "\",\"brakeCurrent\":\"";
    json += std::to_string(brake_motor.getCurrent());

    json += "\",\"brakePosition\":\"";
    json += std::to_string(brake_motor.getPosition());

    json += "\",\"brakeTarget\":\"";
    json += std::to_string(brake_motor.getTarget());


    json += "\",\"clutchMotorEnable\":\"";
    json += (clutch_motor_driver.getState() ? "checked" : "0");

    json += "\",\"clutchCurrent\":\"";
    json += std::to_string(clutch_motor.getCurrent());

    json += "\",\"clutchPosition\":\"";
    json += std::to_string(clutch_motor.getPosition());

    json += "\",\"clutchTarget\":\"";
    json += std::to_string(clutch_motor.getTarget());

    json += "\"}";

    return json;
}

void disableMotors() {
    brake_motor_driver.disable();
    clutch_motor_driver.disable();
}