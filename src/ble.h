#ifdef __BLE__
#define __BLE__


#include <zephyr/types.h>
#include <zephyr.h>
#include <uart.h>
#include <stdlib.h>
#include <stdio.h>


#include <device.h>
#include <soc.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#define UART_BUF_SIZE           CONFIG_BT_GATT_NUS_UART_BUFFER_SIZE
#define OPEN 1000
#define CLOSED 2000

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	        (sizeof(DEVICE_NAME) - 1)

struct bt_conn;



struct uart_data_t {
	void  *fifo_reserved;
	u8_t    data[UART_BUF_SIZE];
	u16_t   len;
};

void connected(struct bt_conn*conn, u8_t err);
void disconnected(struct bt_conn *conn, u8_t reason);
void bt_receive_cb(struct bt_conn *conn, const u8_t *const data, u16_t len);
void bt_ready(int err);
void security_changed(struct bt_conn *conn, bt_security_t level);
void auth_passkey_display(struct bt_conn *conn, unsigned int passkey);
void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey);
void auth_cancel(struct bt_conn *conn);
void auth_done(struct bt_conn *conn);
void pairing_complete(struct bt_conn *conn, bool bonded);
void pairing_failed(struct bt_conn *conn);

#endif