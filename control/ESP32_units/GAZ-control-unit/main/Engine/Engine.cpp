#include "Engine.h"
#include <esp_log.h>
#include <esp_timer.h>

#define OIL_PUMP_TIME   1000
#define STARTER_TIME    1000

static const char *TAG = "Engine";

gpio_num_t Engine::ignitionPin = GPIO_NUM_NC;
gpio_num_t Engine::starterPin = GPIO_NUM_NC;

int64_t Engine::engine_timer = 0;

SemaphoreHandle_t Engine::engine_mutex;

Engine::EngineState Engine::engine_state = disabled;

void Engine::init(gpio_num_t ignitionPin, gpio_num_t starterPin) {
    Engine::ignitionPin = ignitionPin;
    Engine::starterPin = starterPin;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << ignitionPin | 1ULL << starterPin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(ignitionPin, 0);
    gpio_set_level(starterPin, 0);

    engine_mutex = xSemaphoreCreateMutex();

    xTaskCreate(Engine::engineTask, "Engine_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Engine initialized");
}

void Engine::engineTask(void* args) {
    while (true) {
        xSemaphoreTake(engine_mutex, portMAX_DELAY);
        if (engine_state == starting) {
            if (esp_timer_get_time() > engine_timer + OIL_PUMP_TIME * 1000 + STARTER_TIME * 1000) {
                gpio_set_level(starterPin, 0);
                engine_state = engaged;
                ESP_LOGI(TAG, "Engine started");
            }
        }
        else if (engine_state == preparing) {
            if (esp_timer_get_time() > engine_timer + OIL_PUMP_TIME * 1000) {
                gpio_set_level(starterPin, 1);
                engine_state = starting;
                ESP_LOGI(TAG, "Engine starting");
            }
        }
        xSemaphoreGive(engine_mutex);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void Engine::start() {
    if (xSemaphoreTake(engine_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (engine_state == engaged) {
            xSemaphoreGive(engine_mutex);
            ESP_LOGW(TAG, "Engine already engaged");
            return;
        }
        if (engine_state == preparing || engine_state == starting) {
            xSemaphoreGive(engine_mutex);
            ESP_LOGW(TAG, "Engine already engaging");
            return;
        }
        engine_timer = esp_timer_get_time();
        engine_state = preparing;
        gpio_set_level(ignitionPin, 1);
        xSemaphoreGive(engine_mutex);
        ESP_LOGI(TAG, "Engine preparing to start");
    } else {
        ESP_LOGE(TAG, "Engine start failed: mutex timeout");
    }
}

void Engine::stop() {
    if (xSemaphoreTake(engine_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (engine_state == disabled) {
            xSemaphoreGive(engine_mutex);
            ESP_LOGW(TAG, "Engine already stopped");
            return;
        }
        gpio_set_level(ignitionPin, 0);
        gpio_set_level(starterPin, 0);
        engine_state = disabled;
        xSemaphoreGive(engine_mutex);
        ESP_LOGI(TAG, "Engine stopped");
    } else {
        ESP_LOGE(TAG, "Engine stop failed: mutex timeout");
    }
}

void Engine::set_state(bool enabled) {
    EngineState target_state = enabled ? engaged : disabled;

    if (engine_state == target_state || (target_state == engaged && engine_state > disabled))
        return;

    if (enabled) {
        start();
    } else {
        stop();
    }
}

Engine::EngineState Engine::getStatus() {
    return engine_state;
}