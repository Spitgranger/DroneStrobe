#include "driver/gpio.h"
#include <stdio.h>

void app_main(void){
    gpio_reset_pin(8);
    gpio_reset_pin(4);
    gpio_set_direction(8, GPIO_MODE_OUTPUT);
    gpio_set_direction(4, GPIO_MODE_OUTPUT);
    gpio_set_level(8, 0);
    gpio_set_level(4, 0);
}
