/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */

#include <zephyr/types.h>
#include <zephyr.h>
#include <uart.h>
#include <stdlib.h>
#include <stdio.h>

#include <device.h>
#include <soc.h>
#include <nrf.h>
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
#include "adc.h"
//#include "ble.h"

#define STACKSIZE               CONFIG_BT_GATT_NUS_THREAD_STACK_SIZE
#define PRIORITY                7

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	        (sizeof(DEVICE_NAME) - 1)



#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE           CONFIG_BT_GATT_NUS_UART_BUFFER_SIZE


#if defined(DT_ALIAS_PWM_LED0_PWMS_CONTROLLER)
/* get the defines from dt (based on alias 'pwm-led0') */
#define PWM_DRIVER	DT_ALIAS_PWM_LED0_PWMS_CONTROLLER
#define PWM_CHANNEL	11
#else
//#error "Choose supported PWM driver"
#endif

/*
 * 50 is flicker fusion threshold. Modulated light will be perceived
 * as steady by our eyes when blinking rate is at least 50.
 */


#define PERIOD (USEC_PER_SEC / 50U)

#define UART_BUF_SIZE           CONFIG_BT_GATT_NUS_UART_BUFFER_SIZE
#define OPEN 1000
#define CLOSED 2000

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	        (sizeof(DEVICE_NAME) - 1)


/* in micro second */
static int FADESTEP = 500;

static uint16_t vcc = 0;

static int led_blink_state = 1;

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static int pulse_width = OPEN;


struct ble_message_t {
  volatile uint16_t battery_level;
  volatile int16_t servo_pos;
  volatile int8_t nonce;
  volatile uint16_t check;
};

struct ecb_t {
  volatile char key[16];
  volatile char cleartext[16];
  volatile char ciphertext[16];
};

struct ble_message_t msg;
struct ecb_t to_encrypt;

static K_SEM_DEFINE(ble_init_ok, 0, 2);

void TIMER0_IRQHandler(void);
K_TIMER_DEFINE(my_timer, TIMER0_IRQHandler, NULL);
void TIMER0_IRQHandler(void)
{
  set_led_state(RUN_STATUS_LED, 0);
  k_timer_stop(&my_timer);
}

void turn_off_five_v(void);
K_TIMER_DEFINE(my_timer2, turn_off_five_v, NULL);
void turn_off_five_v(void)
{
  set_led_state(FIVE_V_EN, 0);
  k_timer_stop(&my_timer2);
}


//extern void connected(struct bt_conn*conn, u8_t err);



/*struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
	.pairing_confirm = auth_done,
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};*/
extern struct bt_conn_auth_cb conn_auth_callbacks;

static struct device  *uart;

static u32_t led_pins[] = {DT_ALIAS_LED0_GPIOS_PIN,
			   DT_ALIAS_LED1_GPIOS_PIN,
			   DT_ALIAS_LED2_GPIOS_PIN,
			   DT_ALIAS_LED3_GPIOS_PIN};

/*struct uart_data_t {
	void  *fifo_reserved;
	u8_t    data[UART_BUF_SIZE];
	u16_t   len;
};*/



static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static K_FIFO_DEFINE(fifo_ble_tx_data);



 static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, NUS_UUID_SERVICE),
};

struct uart_data_t {
	void  *fifo_reserved;
	u8_t    data[UART_BUF_SIZE];
	u16_t   len;
};

void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");
	current_conn = bt_conn_ref(conn);

	set_led_state(CON_STATUS_LED, LED_ON);
}

void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		set_led_state(CON_STATUS_LED, LED_OFF);
	}
}

#ifdef CONFIG_BT_GATT_NUS_SECURITY_ENABLED
void security_changed(struct bt_conn *conn, bt_security_t level)
{
	printk("Security level was raised to %d\n", level);
}
#endif



#if defined(CONFIG_BT_GATT_NUS_SECURITY_ENABLED)
void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
	printk("Press Button 1 to confirm, Button 2 to reject.\n");
}


void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}


void auth_done(struct bt_conn *conn)
{
	printk("%s()\n", __func__);
	bt_conn_auth_pairing_confirm(conn);
}


void pairing_complete(struct bt_conn *conn, bool bonded)
{
	printk("Paired conn: %p, bonded: %d\n", conn, bonded);
}


void pairing_failed(struct bt_conn *conn)
{
	printk("Pairing failed conn: %p\n", conn);
}


struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
	.pairing_confirm = auth_done,
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
struct bt_conn_auth_cb conn_auth_callbacks;
#endif
extern int led_blink_state;
extern int pulse_width;
void bt_receive_cb(struct bt_conn *conn, const u8_t *const data,
			  u16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	printk("Received data from: %s\n", addr);

	for (u16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			printk("Not able to allocate UART send data buffer\n");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

                if (data[pos] == 'd')
                {
                  set_led_state(FIVE_V_EN, 1);
                  k_timer_start(&my_timer, 0, 2000);
                  pulse_width = OPEN;
                }
                else if (data[pos] == 'e')
                {
                  set_led_state(FIVE_V_EN, 1);
                  k_timer_start(&my_timer, 0, 2000);
                  pulse_width = CLOSED;
                }
		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		k_fifo_put(&fifo_uart_tx_data, tx);
                char* buf = k_malloc(5);
                //sprintf(buf, "%d", vcc);
                k_fifo_put(&fifo_ble_tx_data, buf);
                }

	/* Start the UART transfer by enabling the TX ready interrupt */
	uart_irq_tx_enable(uart);
}

struct bt_gatt_nus_cb nus_cb = {
	.received_cb = bt_receive_cb,
};

void bt_ready(int err)
{
	if (err) {
		printk("BLE init failed with error code %d\n", err);
		return;
	}

	err = bt_gatt_nus_init(&nus_cb);
	if (err) {
		printk("Failed to initialize UART service (err: %d)\n", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
	}

	/* Give two semaphores to signal both the led_blink_thread, and
	 * and the ble_write_thread that ble initialized successfully
	 */
	k_sem_give(&ble_init_ok);
	k_sem_give(&ble_init_ok);
}

struct bt_conn_cb conn_callbacks = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_GATT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

static void uart_cb(struct device *uart)
{
	static struct uart_data_t *rx;

	uart_irq_update(uart);

	if (uart_irq_rx_ready(uart)) {
		int data_length;

		if (!rx) {
			rx = k_malloc(sizeof(*rx));
			if (rx) {
				rx->len = 0;
			} else {
				char dummy;

				printk("Not able to allocate UART receive buffer\n");

				/* Drop one byte to avoid spinning in a
				 * eternal loop.
				 */
				uart_fifo_read(uart, &dummy, 1);

				return;
			}
		}

		data_length = uart_fifo_read(uart, &rx->data[rx->len],
					     UART_BUF_SIZE-rx->len);
		rx->len += data_length;

		if (rx->len > 0) {
			/* Send buffer to bluetooth unit if either buffer size
			 * is reached or the char \n or \r is received, which
			 * ever comes first
			 */
			if ((rx->len == UART_BUF_SIZE) ||
			   (rx->data[rx->len - 1] == '\n') ||
			   (rx->data[rx->len - 1] == '\r')) {
				k_fifo_put(&fifo_uart_rx_data, rx);
				rx = NULL;
			}
		}
	}

	if (uart_irq_tx_ready(uart)) {
		struct uart_data_t *buf =
			k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		u16_t written = 0;

		/* Nothing in the FIFO, nothing to send */
		if (!buf) {
			uart_irq_tx_disable(uart);
			return;
		}

		while (buf->len > written) {
			written += uart_fifo_fill(uart,
						  &buf->data[written],
						  buf->len - written);
		}

		while (!uart_irq_tx_complete(uart)) {
			/* Wait for the last byte to get
			 * shifted out of the module
			 */
		}

		if (k_fifo_is_empty(&fifo_uart_tx_data)) {
			uart_irq_tx_disable(uart);
		}

		k_free(buf);
	}
}

static int init_uart(void)
{
	uart = device_get_binding("UART_0");
	if (!uart) {
		return -ENXIO;
	}

	uart_irq_callback_set(uart, uart_cb);
	uart_irq_rx_enable(uart);

	return 0;
}





void error(void)
{
	int err = -1;

	struct device* led_port = device_get_binding(LED_PORT);
	if (led_port) {
		for (size_t i = 0; i < ARRAY_SIZE(led_pins); i++) {
			err = gpio_pin_configure(led_port, led_pins[i],
						 GPIO_DIR_OUT);
			if (err) {
				break;
			}
		}
	}

	if (!err) {
		for (size_t i = 0; i < ARRAY_SIZE(led_pins); i++) {
			err = gpio_pin_write(led_port, led_pins[i], LED_ON);
			if (err) {
				break;
			}
		}
	}

	while (true) {
		/* Spin for ever */
		k_sleep(1000);
	}
}

static void num_comp_reply(bool accept)
{
        struct bt_conn *auth_conn;
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		printk("Numeric Match, conn %p\n", auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		printk("Numeric Reject, conn %p\n", auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(u32_t button_state, u32_t has_changed)
{
	u32_t buttons = button_state & has_changed;
        struct bt_conn *auth_conn;
	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}

void configure_buttons(void)
{
	int err = dk_buttons_init(button_changed);

	if (err) {
		printk("Cannot init buttons (err: %d)\n", err);
	}
}


static void led_blink_thread(void)
{
	int    blink_status       = 0;
	int    err                = 0;
        struct device *pwm_dev;

        pwm_dev = device_get_binding(PWM_DRIVER);
	if (!pwm_dev) {
		printk("Cannot find %s!\n", PWM_DRIVER);
		return;
	}

	printk("Starting Nordic UART service example\n");

	err = init_uart();
	if (!err) {
		err = bt_enable(bt_ready);
	}

	configure_buttons();

	if (!err) {
		bt_conn_cb_register(&conn_callbacks);

		if (IS_ENABLED(CONFIG_BT_GATT_NUS_SECURITY_ENABLED)) {
			bt_conn_auth_cb_register(&conn_auth_callbacks);
		}

		err = k_sem_take(&ble_init_ok, K_MSEC(100));

		if (!err) {
			printk("Bluetooth initialized\n");
		} else {
			printk("BLE initialization \
				did not complete in time\n");
		}
	}

	if (err) {
		error();
	}

	init_gpio();

	for (;;) {
                vcc = 5.0f/3.0f*sample_adc();
                encrypt();
                
		set_led_state(RUN_STATUS_LED, 1);
                k_timer_start(&my_timer, 0, 2);

                //pwm_pin_set_usec(pwm_dev, 23,
		//			PERIOD, (blink_status % 2) * 1000 + 1000);

	}
}


void ble_write_thread(void)
{
	/* Don't go any further until BLE is initailized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		char* buf = k_fifo_get(&fifo_ble_tx_data,
                                                     K_FOREVER);
                char test[16]; 
                //sprintf(test, "%d", vcc);
                memcpy(test, to_encrypt.ciphertext, 16);
		if (bt_gatt_nus_send(NULL, test, 16)) {
			printk("Failed to send data over BLE connection\n");
		}

		k_free(buf);
	}
}

void pwm_thread(void)
{
	struct device *pwm_dev;
	u32_t pulse_width = 0U;
	u8_t dir = 0U;

	printk("PWM demo app-fade LED\n");

	pwm_dev = device_get_binding(PWM_DRIVER);
	if (!pwm_dev) {
		printk("Cannot find %s!\n", PWM_DRIVER);
		return;
	}

	while (1) {
		if (pwm_pin_set_usec(pwm_dev, PWM_CHANNEL,
					PERIOD, pulse_width)) {
			printk("pwm pin set fails\n");
			return;
		}
                

		if (dir) {
			if (pulse_width < FADESTEP) {
				dir = 0U;
				pulse_width = 0U;
			} else {
				pulse_width -= FADESTEP * (led_blink_state+1);
			}
		} else {
			pulse_width += FADESTEP * (led_blink_state+1);

			if (pulse_width >= PERIOD) {
				dir = 1U;
				pulse_width = PERIOD;
			}
		}
                
		k_sleep(50);
	}
}

void bitbang_pwm_thread()
{
  
  const int period = 20000;
  while(1)
  {
    if (pulse_width < 1500)
    {
      set_led_state(SERVO_OUT, 1);
      k_busy_wait(pulse_width);
      set_led_state(SERVO_OUT, 0);
    }
    k_sleep(20);
  }
}




uint16_t pwm_seq[4] = {30000, 30000, 15000, 15000};
void setup_pwm()
{


    NRF_PWM0->COUNTERTOP = 20000 << PWM_COUNTERTOP_COUNTERTOP_Pos;
    NRF_PWM0->PSEL.OUT[0] = 23;
    NRF_PWM0->ENABLE = 1;
    NRF_PWM0->PRESCALER   = (PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos);
    NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    NRF_PWM0->SEQ[0].PTR = (uint32_t)pwm_seq << PWM_SEQ_PTR_PTR_Pos;
    NRF_PWM0->SEQ[0].CNT = (sizeof(pwm_seq)/sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos;
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;
    NRF_PWM0->TASKS_SEQSTART[0] = 1;

}



char random_char()
{
  NRF_RNG->TASKS_START = 1;
  NRF_RNG->SHORTS = RNG_SHORTS_VALRDY_STOP_Enabled;
  while (!NRF_RNG->EVENTS_VALRDY);
  return  (char)NRF_RNG->VALUE;
}

void encrypt()
{
  
  msg.battery_level = vcc;
  msg.servo_pos = 1234;
  msg.nonce = random_char();
  msg.check = 0xDEAD;

  for (int i = 0; i < 16; i++) 
  {
    to_encrypt.key[i] = 48;
    to_encrypt.cleartext[i] = 0;
    to_encrypt.ciphertext[i] = 0;
  }
  to_encrypt.key[1] = 120;
  to_encrypt.key[2] = 120;

  memcpy(to_encrypt.cleartext, &msg, 8);
  NRF_ECB->ECBDATAPTR = (int)&to_encrypt;
  NRF_ECB->TASKS_STARTECB = 1;
  while (!NRF_ECB->EVENTS_ENDECB && !NRF_ECB->EVENTS_ERRORECB);


}





void set_pwm_value(uint16_t value)
{
    for(int i = 0; i < 4; i++)
    {
        pwm_seq[0] = NRF_PWM0->COUNTERTOP - value;
    }
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

void manual_pwm_thread()
{
  while(1)
  {
    set_pwm_value(pulse_width);
    k_sleep(500);
  }
}

void main()
{
  //setup_adc();
  setup_pwm();
  //volatile int adc_result = sample_adc();
  //volatile int adc_result2 = sample_adc();
  k_sleep(100);
  set_led_state(FIVE_V_EN, 1);

  volatile char random = random_char();
  volatile int vcc2 = sample_adc();
  encrypt();

}

K_THREAD_DEFINE(led_blink_thread_id, STACKSIZE, led_blink_thread, NULL, NULL,
		NULL, PRIORITY, 0, K_NO_WAIT);

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, K_NO_WAIT);

//K_THREAD_DEFINE(pwm_thread_id, STACKSIZE/2, pwm_thread, NULL, NULL,
//		NULL, PRIORITY, 0, K_NO_WAIT);

//K_THREAD_DEFINE(bb_pwm_thread_id, STACKSIZE/8, bitbang_pwm_thread, NULL, NULL,
//		NULL, PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(man_pwm_thread_id, STACKSIZE/8, manual_pwm_thread, NULL, NULL,
		NULL, PRIORITY, 0, K_NO_WAIT);


