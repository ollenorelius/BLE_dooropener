#include <zephyr/types.h>
#include <zephyr.h>
#include <uart.h>
#include <stdlib.h>
#include <stdio.h>

#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <drivers/pwm.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <stdio.h>

#include "board_functions.h"

struct device  *led_port;


void set_led_state(int led, bool state)
{
	if (led_port) {
		gpio_pin_write(led_port, led, state);
	}
}

int init_gpio(void)
{
	int err = 0;

	led_port = device_get_binding(LED_PORT);

	if (!led_port) {
		printk("Could not bind to LED port\n");
		return -ENXIO;
	}

	err = gpio_pin_configure(led_port, RUN_STATUS_LED,
			   GPIO_DIR_OUT);
	if (!err) {
		err = gpio_pin_configure(led_port, CON_STATUS_LED,
			   GPIO_DIR_OUT);
	}
        if (!err) {
		err = gpio_pin_configure(led_port, FIVE_V_EN,
			   GPIO_DIR_OUT);
	}
        if (!err) {
		err = gpio_pin_configure(led_port, SERVO_OUT,
			   GPIO_DIR_OUT);
	}

	if (!err) {
		err = gpio_pin_write(led_port, RUN_STATUS_LED, LED_OFF);
	}

	if (!err) {
		err = gpio_pin_write(led_port, CON_STATUS_LED, LED_OFF);
	}
        if (!err) {
		err = gpio_pin_write(led_port, FIVE_V_EN, LED_OFF);
	}
        if (!err) {
		err = gpio_pin_write(led_port, SERVO_OUT, LED_OFF);
	}

	if (err) {
		printk("Not able to correctly initialize LED pins (err:%d)",
			err);
		led_port = NULL;
	}

	return err;
}