#ifndef GPIO_H
#define GPIO_H
#include <stdbool.h>
#include <zephyr/drivers/gpio.h>
//function to switch onboard led on or off using true/false
void set_board_led(bool state);
//initialises GPIO inputs and outputs so they can be used
int initialise_gpio();

#endif