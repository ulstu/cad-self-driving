#pragma once
#include <stdint.h>
#include <stdlib.h>

struct pcm_file_t {
    uint8_t* pcm_start;
    uint32_t length;
    uint8_t volume = 0;
};

extern pcm_file_t soundPullup;
// extern pcm_file_t soundMusic;