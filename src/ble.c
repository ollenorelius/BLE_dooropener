#include "ble.h"
#include "vars.c"

#include "board_functions.h"

struct bt_conn *current_conn;
struct bt_conn *auth_conn;

extern struct k_fifo fifo_uart_tx_data;
extern struct k_fifo fifo_ble_tx_data;
extern struct device *uart;

extern struct k_sem ble_init_ok;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, NUS_UUID_SERVICE),
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
                  led_blink_state = 1;
                  set_led_state(CON_STATUS_LED, 0);
                  pulse_width = OPEN;
                }
                else if (data[pos] == 'e')
                {
                  led_blink_state = 0;
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