
#ifndef BOARD_FUNCTIONS
#define  BOARD_FUNCTIONS 1

/* Change this if you have an LED connected to a custom port */
#define LED_PORT                DT_ALIAS_LED0_GPIOS_CONTROLLER

#define RUN_STATUS_LED          DT_ALIAS_LED0_GPIOS_PIN
#define RUN_LED_BLINK_INTERVAL  5000

#define CON_STATUS_LED          DT_ALIAS_LED1_GPIOS_PIN
#define FIVE_V_EN               DT_GPIO_LEDS_FIVE_VOLT_GPIOS_PIN
#define SERVO_OUT               DT_GPIO_LEDS_SERVO_OUT_GPIOS_PIN

#define LED_ON                  1
#define LED_OFF                 0


int init_gpio(void);
void set_led_state(int led, bool state);

#endif