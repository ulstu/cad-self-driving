#pragma once
#include "sounds.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"


class Sound {
public:
    static void init();
    static void play(pcm_file_t* soundFile, uint16_t repeatCount = 1);
private:
    static i2s_chan_handle_t tx_chan;
    static uint16_t i2s_buff[2048];
    static void playerTask(void* args);
    static QueueHandle_t playSoundQueue;

    enum filePriority {
        normal = 0,
        high,
        cancel
    };

    struct fileToPlay_t {
        pcm_file_t* soundFile;
        uint16_t repeatCount = 0;
        filePriority priority = filePriority::normal;
    };
};