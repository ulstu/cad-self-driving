#include "Sound.h"
#include "esp_log.h"

#define I2S_BUFF_SIZE               2048
#define SAMPLE_RATE                 44100

#define I2S_PIN_LRC                 GPIO_NUM_45
#define I2S_PIN_BCLK                GPIO_NUM_39
#define I2S_PIN_DOUT                GPIO_NUM_40

static const char* TAG = "Sound";

i2s_chan_handle_t Sound::tx_chan;
QueueHandle_t Sound::playSoundQueue;
uint16_t Sound::i2s_buff[2048];

void Sound::init() {
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

    i2s_std_config_t tx_std_cfg = {
            .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
            .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_MONO),

            .gpio_cfg = {
                    .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
                    .bclk = I2S_PIN_BCLK,
                    .ws   = I2S_PIN_LRC,
                    .dout = I2S_PIN_DOUT,
                    .din  = I2S_GPIO_UNUSED,
                    .invert_flags = {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv   = false,
                    },
            },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));

    playSoundQueue = xQueueCreate(16, sizeof(fileToPlay_t));

    xTaskCreate(playerTask, "sound_task", 4096, NULL, 3, NULL);
}

void Sound::play(pcm_file_t* soundFile, uint16_t repeatCount) {
    fileToPlay_t sound = {
        .soundFile = soundFile,
        .repeatCount = repeatCount,
        .priority = filePriority::normal
    };
    xQueueSendToBack(playSoundQueue, &sound, 0);
}

void Sound::playerTask(void* args) {
    fileToPlay_t sound;
    pcm_file_t soundFile;
    while (true) {
        xQueueReceive(playSoundQueue, &sound, portMAX_DELAY);
        soundFile = *sound.soundFile;

        uint32_t samples = 0;
        size_t bytes_written;

        ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));

        while (samples < soundFile.length) {
            uint32_t samplesToWrite = samples + I2S_BUFF_SIZE <= soundFile.length ? I2S_BUFF_SIZE : soundFile.length - samples;
            for (uint32_t sample = 0; sample < samplesToWrite; sample++) {
                i2s_buff[sample] = soundFile.pcm_start[samples] << soundFile.volume;
                samples++;
            }
            if (i2s_channel_write(tx_chan, i2s_buff, samplesToWrite * 2, &bytes_written, pdMS_TO_TICKS(1000)) != ESP_OK) {
                ESP_LOGE(TAG, "Error while writing data to i2s channel");
            }
        }

        ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));

        ESP_LOGI(TAG, "Playing finished");
    }
    
}