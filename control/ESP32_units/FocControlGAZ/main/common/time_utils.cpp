#include "time_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){
  vTaskDelay(ms / portTICK_PERIOD_MS);
}


// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){
  return esp_timer_get_time();
}
