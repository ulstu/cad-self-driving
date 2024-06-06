#include "sounds.h"

extern uint8_t pullup_pcm_start[] asm("_binary_pull_pcm_start");
extern uint8_t pullup_pcm_end[]   asm("_binary_pull_pcm_end");

pcm_file_t soundPullup = {
    .pcm_start = pullup_pcm_start,
    .length = (uint32_t)(pullup_pcm_end - pullup_pcm_start),
    .volume = 8
};

// extern uint8_t music_pcm_start[] asm("_binary_o_pcm_start");
// extern uint8_t music_pcm_end[]   asm("_binary_o_pcm_end");

// pcm_file_t soundMusic = {
//     .pcm_start = music_pcm_start,
//     .length = (uint32_t)(music_pcm_end - music_pcm_start),
//     .volume = 0
// };