#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_log.h"

class Storage {
public:
    Storage();
    void init();
    void removeLog();
    void append(std::string str);
    const char *events_filename = "/spiffs/events.csv";
};

extern Storage storage;

extern std::string getTimeStr();