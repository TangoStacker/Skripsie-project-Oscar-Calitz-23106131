//initialize led
#include "GPIO.h"
static const struct gpio_dt_spec led_onboard = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
void set_board_led(bool state) {
	gpio_pin_set_dt(&led_onboard, state);
}
int initialise_gpio(){
    //check that gpio is ready
    if (!gpio_is_ready_dt(&led_onboard)) {
		return 0;
	}
    //configure it as output and set it active
	int ret = gpio_pin_configure_dt(&led_onboard, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
    return 1;
}
