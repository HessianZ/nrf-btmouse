/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <assert.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/bas.h>
#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/services/dis.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>

#include "kbd.h"
#include "nfc.h"

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION   0x0101

/* Number of pixels by which the cursor is moved when a button is pushed. */
#define MOVEMENT_SPEED              5
/* Number of input reports in this application. */
#define INPUT_REPORT_COUNT          3
/* Length of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_LEN       3
/* Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_LEN      3
/* Length of Mouse Input Report containing media player data. */
#define INPUT_REP_MEDIA_PLAYER_LEN  1
/* Index of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_INDEX     0
/* Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_INDEX    1
/* Index of Mouse Input Report containing media player data. */
#define INPUT_REP_MPLAYER_INDEX     2
/* Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_BUTTONS_ID    1
/* Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MOVEMENT_ID   2
/* Id of reference to Mouse Input Report containing media player data. */
#define INPUT_REP_REF_MPLAYER_ID    3

/* HIDs queue size. */
#define HIDS_QUEUE_SIZE 10


static const struct gpio_dt_spec button_up = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(button_up), gpios, {0});
static const struct gpio_dt_spec button_down = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(button_down), gpios, {0});
static const struct gpio_dt_spec button_left = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(button_left), gpios, {0});
static const struct gpio_dt_spec button_right = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(button_right), gpios, {0});
static const struct gpio_dt_spec button_mid = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(button_mid), gpios, {0});
static const struct gpio_dt_spec button_set = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(button_set), gpios, {0});
static const struct gpio_dt_spec button_rst = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(button_rst), gpios, {0});
static struct gpio_callback mouse_button_callback;
static struct gpio_callback mouse_move_callback;

static void num_comp_reply(bool accept);


/// Standard Mouse Buttons Bitmap
#define MOUSE_BUTTON_LEFT 1 ///< Left button
#define  MOUSE_BUTTON_RIGHT 2 ///< Right button
#define  MOUSE_BUTTON_MIDDLE 4 ///< Middle button


/* Key used to move cursor left */
#define KEY_LEFT_MASK   BIT(button_left.pin)
/* Key used to move cursor up */
#define KEY_UP_MASK     BIT(button_up.pin)
/* Key used to move cursor right */
#define KEY_RIGHT_MASK  BIT(button_right.pin)
/* Key used to move cursor down */
#define KEY_DOWN_MASK   BIT(button_down.pin)
/* Key used to move cursor mid */
#define KEY_MID_MASK    BIT(button_mid.pin)
/* Key used to move cursor set */
#define KEY_SET_MASK    BIT(button_set.pin)
/* Key used to move cursor rst */
#define KEY_RST_MASK    BIT(button_rst.pin)


/* Key used to accept or reject passkey value */
#define KEY_PAIRING_ACCEPT DK_BTN1_MSK
#define KEY_PAIRING_REJECT DK_BTN2_MSK

/* HIDS instance. */
BT_HIDS_DEF(hids_obj,
	    INPUT_REP_BUTTONS_LEN,
	    INPUT_REP_MOVEMENT_LEN,
	    INPUT_REP_MEDIA_PLAYER_LEN,
        OUTPUT_REPORT_MAX_LEN,
        INPUT_REPORT_KEYS_MAX_LEN);

static struct k_work hids_work;

enum InputEventType {
    INPUT_EVENT_TYPE_NONE = 0,
    INPUT_EVENT_TYPE_MOUSE = 1,
    INPUT_EVENT_TYPE_KEYBOARD = 2,
} ;

struct InputEvent {
    uint8_t type;
    uint8_t buttons;
	int16_t x_val;
	int16_t y_val;
};

/* Mouse movement queue. */
K_MSGQ_DEFINE(hids_queue,
	      sizeof(struct InputEvent),
	      HIDS_QUEUE_SIZE,
	      4);

#if CONFIG_BT_DIRECTED_ADVERTISING
/* Bonded address queue. */
K_MSGQ_DEFINE(bonds_queue,
	      sizeof(bt_addr_le_t),
	      CONFIG_BT_MAX_PAIRED,
	      4);
#endif

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct conn_mode {
	struct bt_conn *conn;
	bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

static struct k_work adv_work;

static struct k_work pairing_work;
struct pairing_data_mitm {
	struct bt_conn *conn;
	unsigned int passkey;
};

K_MSGQ_DEFINE(mitm_queue,
	      sizeof(struct pairing_data_mitm),
	      CONFIG_BT_HIDS_MAX_CLIENT_COUNT,
	      4);


/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

K_THREAD_DEFINE(blink0_id, STACKSIZE, nfc_task, NULL, NULL, NULL,
                PRIORITY, 0, 0);

extern struct keyboard_state hid_keyboard_state;

#if CONFIG_BT_DIRECTED_ADVERTISING
static void bond_find(const struct bt_bond_info *info, void *user_data)
{
	int err;

	/* Filter already connected peers. */
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn) {
			const bt_addr_le_t *dst =
				bt_conn_get_dst(conn_mode[i].conn);

			if (!bt_addr_le_cmp(&info->addr, dst)) {
				return;
			}
		}
	}

	err = k_msgq_put(&bonds_queue, (void *) &info->addr, K_NO_WAIT);
	if (err) {
		printk("No space in the queue for the bond.\n");
	}
}
#endif

static void advertising_continue(void)
{
	struct bt_le_adv_param adv_param;

#if CONFIG_BT_DIRECTED_ADVERTISING
	bt_addr_le_t addr;

	if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) {
		char addr_buf[BT_ADDR_LE_STR_LEN];

		adv_param = *BT_LE_ADV_CONN_DIR(&addr);
		adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

		int err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);

		if (err) {
			printk("Directed advertising failed to start\n");
			return;
		}

		bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN);
		printk("Direct advertising to %s started\n", addr_buf);
	} else
#endif
	{
		int err;

		adv_param = *BT_LE_ADV_CONN;
		adv_param.options |= BT_LE_ADV_OPT_ONE_TIME;
		err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
		if (err) {
			printk("Advertising failed to start (err %d)\n", err);
			return;
		}

		printk("Regular advertising started\n");
	}
}

static void advertising_start(void)
{
#if CONFIG_BT_DIRECTED_ADVERTISING
	k_msgq_purge(&bonds_queue);
	bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL);
#endif

	k_work_submit(&adv_work);
}

static void advertising_process(struct k_work *work)
{
	advertising_continue();
}

static void pairing_process(struct k_work *work)
{
	int err;
	struct pairing_data_mitm pairing_data;

	char addr[BT_ADDR_LE_STR_LEN];

	err = k_msgq_peek(&mitm_queue, &pairing_data);
	if (err) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(pairing_data.conn),
			  addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, pairing_data.passkey);
//	printk("Press Button 1 to confirm, Button 2 to reject.\n");

//     Auto confirm
    if (k_msgq_num_used_get(&mitm_queue)) {
        printk("Auto confirmed passkey.\n");
        num_comp_reply(true);
    }
}


static void insert_conn_object(struct bt_conn *conn)
{
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (!conn_mode[i].conn) {
			conn_mode[i].conn = conn;
			conn_mode[i].in_boot_mode = false;

			return;
		}
	}

	printk("Connection object could not be inserted %p\n", conn);
}


static bool is_conn_slot_free(void)
{
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (!conn_mode[i].conn) {
			return true;
		}
	}

	return false;
}


static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		if (err == BT_HCI_ERR_ADV_TIMEOUT) {
			printk("Direct advertising to %s timed out\n", addr);
			k_work_submit(&adv_work);
		} else {
			printk("Failed to connect to %s (%u)\n", addr, err);
		}
		return;
	}

	printk("Connected %s\n", addr);

	err = bt_hids_connected(&hids_obj, conn);

	if (err) {
		printk("Failed to notify HID service about connection\n");
		return;
	}

	insert_conn_object(conn);

	if (is_conn_slot_free()) {
		advertising_start();
	}
}


static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected from %s (reason %u)\n", addr, reason);

	err = bt_hids_disconnected(&hids_obj, conn);

	if (err) {
		printk("Failed to notify HID service about disconnection\n");
	}

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn == conn) {
			conn_mode[i].conn = NULL;
			break;
		}
	}

	advertising_start();
}


#ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level,
			err);
	}
}
#endif


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};


static void hids_pm_evt_handler(enum bt_hids_pm_evt evt,
				struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	size_t i;

	for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn == conn) {
			break;
		}
	}

	if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	switch (evt) {
	case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
		printk("Boot mode entered %s\n", addr);
		conn_mode[i].in_boot_mode = true;
		break;

	case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
		printk("Report mode entered %s\n", addr);
		conn_mode[i].in_boot_mode = false;
		break;

	default:
		break;
	}
}

static void hids_outp_rep_handler(struct bt_hids_rep *rep,
                                  struct bt_conn *conn,
                                  bool write)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (!write) {
        printk("Output report read\n");
        return;
    };

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Output report has been received %s\n", addr);
    caps_lock_handler(rep);
}


static void hid_init(void)
{
	int err;
	struct bt_hids_init_param hids_init_param = { 0 };
	struct bt_hids_inp_rep *hids_inp_rep;
    struct bt_hids_outp_feat_rep *hids_outp_rep;
	static const uint8_t mouse_movement_mask[DIV_ROUND_UP(INPUT_REP_MOVEMENT_LEN, 8)] = {0};

	static const uint8_t report_map[] = {
		0x05, 0x01,     /* Usage Page (Generic Desktop) */
		0x09, 0x02,     /* Usage (Mouse) */

		0xA1, 0x01,     /* Collection (Application) */

		/* Report ID 1: Mouse buttons + scroll/pan */
		0x85, 0x01,       /* Report Id 1 */
		0x09, 0x01,       /* Usage (Pointer) */
		0xA1, 0x00,       /* Collection (Physical) */
		0x95, 0x05,       /* Report Count (3) */
		0x75, 0x01,       /* Report Size (1) */
		0x05, 0x09,       /* Usage Page (Buttons) */
		0x19, 0x01,       /* Usage Minimum (01) */
		0x29, 0x05,       /* Usage Maximum (05) */
		0x15, 0x00,       /* Logical Minimum (0) */
		0x25, 0x01,       /* Logical Maximum (1) */
		0x81, 0x02,       /* Input (Data, Variable, Absolute) */
		0x95, 0x01,       /* Report Count (1) */
		0x75, 0x03,       /* Report Size (3) */
		0x81, 0x01,       /* Input (Constant) for padding */
		0x75, 0x08,       /* Report Size (8) */
		0x95, 0x01,       /* Report Count (1) */
		0x05, 0x01,       /* Usage Page (Generic Desktop) */
		0x09, 0x38,       /* Usage (Wheel) */
		0x15, 0x81,       /* Logical Minimum (-127) */
		0x25, 0x7F,       /* Logical Maximum (127) */
		0x81, 0x06,       /* Input (Data, Variable, Relative) */
		0x05, 0x0C,       /* Usage Page (Consumer) */
		0x0A, 0x38, 0x02, /* Usage (AC Pan) */
		0x95, 0x01,       /* Report Count (1) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0xC0,             /* End Collection (Physical) */

		/* Report ID 2: Mouse motion */
		0x85, 0x02,       /* Report Id 2 */
		0x09, 0x01,       /* Usage (Pointer) */
		0xA1, 0x00,       /* Collection (Physical) */
		0x75, 0x0C,       /* Report Size (12) */
		0x95, 0x02,       /* Report Count (2) */
		0x05, 0x01,       /* Usage Page (Generic Desktop) */
		0x09, 0x30,       /* Usage (X) */
		0x09, 0x31,       /* Usage (Y) */
		0x16, 0x01, 0xF8, /* Logical maximum (2047) */
		0x26, 0xFF, 0x07, /* Logical minimum (-2047) */
		0x81, 0x06,       /* Input (Data, Variable, Relative) */
		0xC0,             /* End Collection (Physical) */
		0xC0,             /* End Collection (Application) */

		/* Report ID 3: Advanced buttons */
		0x05, 0x0C,       /* Usage Page (Consumer) */
		0x09, 0x01,       /* Usage (Consumer Control) */
		0xA1, 0x01,       /* Collection (Application) */
		0x85, 0x03,       /* Report Id (3) */
		0x15, 0x00,       /* Logical minimum (0) */
		0x25, 0x01,       /* Logical maximum (1) */
		0x75, 0x01,       /* Report Size (1) */
		0x95, 0x01,       /* Report Count (1) */

		0x09, 0xCD,       /* Usage (Play/Pause) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x83, 0x01, /* Usage (Consumer Control Configuration) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xB5,       /* Usage (Scan Next Track) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xB6,       /* Usage (Scan Previous Track) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */

		0x09, 0xEA,       /* Usage (Volume Down) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xE9,       /* Usage (Volume Up) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x25, 0x02, /* Usage (AC Forward) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x24, 0x02, /* Usage (AC Back) */
		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
		0xC0,              /* End Collection */


        0x05, 0x01,       /* Usage Page (Generic Desktop) */
        0x09, 0x06,       /* Usage (Keyboard) */
        0xA1, 0x01,       /* Collection (Application) */

            /* Keys */
        0x85, 0x04,       /* Report Id (4) */
        0x05, 0x07,       /* Usage Page (Key Codes) */
        0x19, 0xe0,       /* Usage Minimum (224) */
        0x29, 0xe7,       /* Usage Maximum (231) */
        0x15, 0x00,       /* Logical Minimum (0) */
        0x25, 0x01,       /* Logical Maximum (1) */
        0x75, 0x01,       /* Report Size (1) */
        0x95, 0x08,       /* Report Count (8) */
        0x81, 0x02,       /* Input (Data, Variable, Absolute) */

        0x95, 0x01,       /* Report Count (1) */
        0x75, 0x08,       /* Report Size (8) */
        0x81, 0x01,       /* Input (Constant) reserved byte(1) */

        0x95, 0x06,       /* Report Count (6) */
        0x75, 0x08,       /* Report Size (8) */
        0x15, 0x00,       /* Logical Minimum (0) */
        0x25, 0x65,       /* Logical Maximum (101) */
        0x05, 0x07,       /* Usage Page (Key codes) */
        0x19, 0x00,       /* Usage Minimum (0) */
        0x29, 0x65,       /* Usage Maximum (101) */
        0x81, 0x00,       /* Input (Data, Array) Key array(6 bytes) */

            /* LED */
        0x85, 0x05,       /* Report Id (5) */
        0x95, 0x05,       /* Report Count (5) */
        0x75, 0x01,       /* Report Size (1) */
        0x05, 0x08,       /* Usage Page (Page# for LEDs) */
        0x19, 0x01,       /* Usage Minimum (1) */
        0x29, 0x05,       /* Usage Maximum (5) */
        0x91, 0x02,       /* Output (Data, Variable, Absolute), */
            /* Led report */
        0x95, 0x01,       /* Report Count (1) */
        0x75, 0x03,       /* Report Size (3) */
        0x91, 0x01,       /* Output (Data, Variable, Absolute), */
            /* Led report padding */

        0xC0              /* End Collection (Application) */
    };

	hids_init_param.rep_map.data = report_map;
	hids_init_param.rep_map.size = sizeof(report_map);

	hids_init_param.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	hids_init_param.info.b_country_code = 0x00;
	hids_init_param.info.flags = (BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE);

	hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[0];
	hids_inp_rep->size = INPUT_REP_BUTTONS_LEN;
	hids_inp_rep->id = INPUT_REP_REF_BUTTONS_ID;
	hids_init_param.inp_rep_group_init.cnt++;

	hids_inp_rep++;
	hids_inp_rep->size = INPUT_REP_MOVEMENT_LEN;
	hids_inp_rep->id = INPUT_REP_REF_MOVEMENT_ID;
	hids_inp_rep->rep_mask = mouse_movement_mask;
	hids_init_param.inp_rep_group_init.cnt++;

	hids_inp_rep++;
	hids_inp_rep->size = INPUT_REP_MEDIA_PLAYER_LEN;
	hids_inp_rep->id = INPUT_REP_REF_MPLAYER_ID;
	hids_init_param.inp_rep_group_init.cnt++;

    hids_inp_rep++;
    hids_inp_rep->size = INPUT_REPORT_KEYS_MAX_LEN;
    hids_inp_rep->id = INPUT_REP_KEYS_REF_ID;
    hids_init_param.inp_rep_group_init.cnt++;


    hids_init_param.is_mouse = true;
	hids_init_param.pm_evt_handler = hids_pm_evt_handler;

    hids_outp_rep = &hids_init_param.outp_rep_group_init.reports[OUTPUT_REP_KEYS_IDX];
    hids_outp_rep->size = OUTPUT_REPORT_MAX_LEN;
    hids_outp_rep->id = OUTPUT_REP_KEYS_REF_ID;
    hids_outp_rep->handler = hids_outp_rep_handler;
    hids_init_param.outp_rep_group_init.cnt++;

    hids_init_param.is_kb = true;

	err = bt_hids_init(&hids_obj, &hids_init_param);
	__ASSERT(err == 0, "HIDS initialization failed\n");
}


static void mouse_movement_send(int16_t x_delta, int16_t y_delta)
{
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {

		if (!conn_mode[i].conn) {
			continue;
		}

		if (conn_mode[i].in_boot_mode) {
			x_delta = MAX(MIN(x_delta, SCHAR_MAX), SCHAR_MIN);
			y_delta = MAX(MIN(y_delta, SCHAR_MAX), SCHAR_MIN);

			bt_hids_boot_mouse_inp_rep_send(&hids_obj,
							     conn_mode[i].conn,
							     NULL,
							     (int8_t) x_delta,
							     (int8_t) y_delta,
							     NULL);
		} else {
			uint8_t x_buff[2];
			uint8_t y_buff[2];
			uint8_t buffer[INPUT_REP_MOVEMENT_LEN];

			int16_t x = MAX(MIN(x_delta, 0x07ff), -0x07ff);
			int16_t y = MAX(MIN(y_delta, 0x07ff), -0x07ff);

			/* Convert to little-endian. */
			sys_put_le16(x, x_buff);
			sys_put_le16(y, y_buff);

			/* Encode report. */
			BUILD_ASSERT(sizeof(buffer) == 3,
					 "Only 2 axis, 12-bit each, are supported");

			buffer[0] = x_buff[0];
			buffer[1] = (y_buff[0] << 4) | (x_buff[1] & 0x0f);
			buffer[2] = (y_buff[1] << 4) | (y_buff[0] >> 4);


			bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn,
						  INPUT_REP_MOVEMENT_INDEX,
						  buffer, sizeof(buffer), NULL);
		}
	}
}

static void mouse_click_send(const uint8_t buttons)
{
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {

		if (!conn_mode[i].conn) {
			continue;
		}

		if (conn_mode[i].in_boot_mode) {
			bt_hids_boot_mouse_inp_rep_send(&hids_obj,
							     conn_mode[i].conn,
                                 &buttons,
							     0,
							     0,
							     NULL);
		} else {

            // 0. Buttons
            // 1. Wheel
            // 2. AC Pan
            uint8_t buffer[INPUT_REP_BUTTONS_LEN];


            // press
            buffer[0] = buttons;
			bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn,
                         INPUT_REP_BUTTONS_INDEX,
                         &buffer, sizeof(buffer), NULL);

            // release
            buffer[0] = 0;
            bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn,
                                 INPUT_REP_BUTTONS_INDEX,
                                 &buffer, sizeof(buffer), NULL);
		}
	}
}

static void input_event_queue_handler(struct k_work *work)
{
	struct InputEvent event;


	while (!k_msgq_get(&hids_queue, &event, K_NO_WAIT)) {
        if (event.type == INPUT_EVENT_TYPE_MOUSE) {
            if (event.buttons > 0) {
                mouse_click_send(event.buttons);
            } else {
                mouse_movement_send(event.x_val, event.y_val);
            }
        } else if (event.type == INPUT_EVENT_TYPE_KEYBOARD) {
            type_demo_chars();
        }
	}
}

#if defined(CONFIG_BT_HIDS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}


static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	int err;

	struct pairing_data_mitm pairing_data;

	pairing_data.conn    = bt_conn_ref(conn);
	pairing_data.passkey = passkey;

	err = k_msgq_put(&mitm_queue, &pairing_data, K_NO_WAIT);
	if (err) {
		printk("Pairing queue is full. Purge previous data.\n");
	}

	/* In the case of multiple pairing requests, trigger
	 * pairing confirmation which needed user interaction only
	 * once to avoid display information about all devices at
	 * the same time. Passkey confirmation for next devices will
	 * be proccess from queue after handling the earlier ones.
	 */
	if (k_msgq_num_used_get(&mitm_queue) == 1) {
		k_work_submit(&pairing_work);
	}
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	struct pairing_data_mitm pairing_data;

	if (k_msgq_peek(&mitm_queue, &pairing_data) != 0) {
		return;
	}

	if (pairing_data.conn == conn) {
		bt_conn_unref(pairing_data.conn);
		k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT);
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif /* defined(CONFIG_BT_HIDS_SECURITY_ENABLED) */


static void num_comp_reply(bool accept)
{
	struct pairing_data_mitm pairing_data;
	struct bt_conn *conn;

	if (k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT) != 0) {
		return;
	}

	conn = pairing_data.conn;

	if (accept) {
		bt_conn_auth_passkey_confirm(conn);
		printk("Numeric Match, conn %p\n", conn);
	} else {
		bt_conn_auth_cancel(conn);
		printk("Numeric Reject, conn %p\n", conn);
	}

	bt_conn_unref(pairing_data.conn);

	if (k_msgq_num_used_get(&mitm_queue)) {
		k_work_submit(&pairing_work);
	}
}

typedef enum {
    MOUSE_MOVE_NONE,
    MOUSE_MOVE_UP,
    MOUSE_MOVE_DOWN,
    MOUSE_MOVE_LEFT,
    MOUSE_MOVE_RIGHT,
} MouseMoveDir;

struct MouseMovement {
    MouseMoveDir dir;
} mouse_movement;


void mouse_button_pressed(const struct device *dev, struct gpio_callback *cb,
                          uint32_t pins)
{
    struct InputEvent event;
    memset(&event, 0, sizeof(struct InputEvent));

    uint8_t pin_no = find_msb_set(pins) - 1;
    int level = gpio_pin_get(dev, pin_no);

    printk("Button pressed at %" PRIu32 " PIN %s %" PRIu32 " PIN_MASK %" PRIu32 " LEVEL %d\n", k_cycle_get_32(), dev->name, pin_no, cb->pin_mask, level);

    event.type = INPUT_EVENT_TYPE_NONE;

    if (pins & KEY_SET_MASK) {
        printk("%s(): set\n", __func__);
        event.buttons = MOUSE_BUTTON_LEFT;
        event.type = INPUT_EVENT_TYPE_MOUSE;
    }

    if (pins & KEY_RST_MASK) {
        printk("%s(): rst\n", __func__);
        event.buttons = MOUSE_BUTTON_RIGHT;
        event.type = INPUT_EVENT_TYPE_MOUSE;
    }

    if (event.type != INPUT_EVENT_TYPE_NONE) {
        int err;

        err = k_msgq_put(&hids_queue, &event, K_NO_WAIT);
        if (err) {
            printk("No space in the queue for button pressed\n");
            return;
        }
        if (k_msgq_num_used_get(&hids_queue) == 1) {
            k_work_submit(&hids_work);
        }
    }
}


void mouse_move_pressed(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins)
{
    uint8_t pin_no = find_msb_set(pins) - 1;
    // logical level (gpio configured as low active)
    bool is_down = gpio_pin_get(dev, pin_no) == 1;

    printk("Button pressed at %" PRIu32 " PIN %s %" PRIu32 " PIN_MASK %" PRIu32 " LEVEL %d\n", k_cycle_get_32(), dev->name, pin_no, cb->pin_mask, is_down);

    if (is_down) {
        if (pins & KEY_LEFT_MASK) {
            printk("%s(): left\n", __func__);
            mouse_movement.dir = MOUSE_MOVE_LEFT;
        }
        if (pins & KEY_UP_MASK) {
            printk("%s(): up\n", __func__);
            mouse_movement.dir = MOUSE_MOVE_UP;
        }
        if (pins & KEY_RIGHT_MASK) {
            printk("%s(): right\n", __func__);
            mouse_movement.dir = MOUSE_MOVE_RIGHT;
        }
        if (pins & KEY_DOWN_MASK) {
            printk("%s(): down\n", __func__);
            mouse_movement.dir = MOUSE_MOVE_DOWN;
        }
    } else {
        mouse_movement.dir = MOUSE_MOVE_NONE;
    }
}

void init_button(const struct gpio_dt_spec *button)
{

    if (!gpio_is_ready_dt(button)) {
        printk("Error: button device %s is not ready\n",
               button->port->name);
        return;
    }

    int ret = gpio_pin_configure_dt(button, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure %s pin %d\n",
               ret, button->port->name, button->pin);
        return;
    }

    ret = gpio_pin_interrupt_configure_dt(button, GPIO_INT_EDGE_TO_INACTIVE);
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on %s pin %d\n",
               ret, button->port->name, button->pin);
        return;
    }

    printk("Set up button at %s pin %d\n", button->port->name, button->pin);
}

int init_buttons()
{
    const struct gpio_dt_spec *move_buttons[] = {
        &button_up,
        &button_down,
        &button_left,
        &button_right,
    };
    const struct gpio_dt_spec *cmd_buttons[] = {
        &button_mid,
        &button_set,
        &button_rst,
    };

    int button_count = ARRAY_SIZE(cmd_buttons);
    uint32_t pin_mask = 0;

    for (int i = 0; i < button_count; ++i) {
        const struct gpio_dt_spec *button = cmd_buttons[i];
        init_button(button);

        pin_mask |= BIT(cmd_buttons[i]->pin);
    }
    gpio_init_callback(&mouse_button_callback, mouse_button_pressed, pin_mask);

    for (int i = 0; i < button_count; ++i) {
        int err = gpio_add_callback(cmd_buttons[i]->port, &mouse_button_callback);
        if (err) {
            printk("[%s] Cannot add callback", __func__);
            return err;
        }
    }

    button_count = ARRAY_SIZE(move_buttons);
    pin_mask = 0;

    for (int i = 0; i < button_count; ++i) {
        const struct gpio_dt_spec *button = move_buttons[i];
        init_button(button);

        pin_mask |= BIT(move_buttons[i]->pin);
    }
    gpio_init_callback(&mouse_move_callback, mouse_move_pressed, pin_mask);

    for (int i = 0; i < button_count; ++i) {
        int err = gpio_add_callback(move_buttons[i]->port, &mouse_move_callback);
        if (err) {
            printk("[%s] Cannot add callback", __func__);
            return err;
        }
    }

    return 0;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	struct InputEvent event;
	uint32_t buttons = button_state & has_changed;

	memset(&event, 0, sizeof(struct InputEvent));

    event.type = INPUT_EVENT_TYPE_NONE;

	if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
		if (k_msgq_num_used_get(&mitm_queue)) {
			if (buttons & KEY_PAIRING_ACCEPT) {
				num_comp_reply(true);

				return;
			}

			if (buttons & KEY_PAIRING_REJECT) {
				num_comp_reply(false);

				return;
			}
		}
	}

	if (buttons & DK_BTN1_MSK) {
        printk("%s(): button1 click\n", __func__);
        event.buttons = MOUSE_BUTTON_LEFT;
        event.type = INPUT_EVENT_TYPE_MOUSE;
	}

	if (buttons & KEY_TEXT_MASK) {
        printk("%s(): button2 click\n", __func__);
        if ((button_state & KEY_TEXT_MASK) != 0) {
            event.type = INPUT_EVENT_TYPE_KEYBOARD;
        }
	}

	if (event.type != INPUT_EVENT_TYPE_NONE) {
		int err;

		err = k_msgq_put(&hids_queue, &event, K_NO_WAIT);
		if (err) {
			printk("No space in the queue for button pressed\n");
			return;
		}
		if (k_msgq_num_used_get(&hids_queue) == 1) {
			k_work_submit(&hids_work);
		}
	}
}

void configure_buttons(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		printk("Cannot init buttons (err: %d)\n", err);
	}

    err = init_buttons();

    if (err) {
        printk("Cannot init buttons (err: %d)\n", err);
    }
}


static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}

void mouse_event_sender(struct k_timer * timer)
{
    struct InputEvent event;
    memset(&event, 0, sizeof(struct InputEvent));
    static uint8_t acc = 0;

    if (mouse_movement.dir == MOUSE_MOVE_NONE) {
        // reset acc timer
        acc = 0;
    } else {

        event.type = INPUT_EVENT_TYPE_MOUSE;
        switch (mouse_movement.dir) {
            case MOUSE_MOVE_UP:
                event.y_val = -acc;
                break;
            case MOUSE_MOVE_DOWN:
                event.y_val = acc;
                break;
            case MOUSE_MOVE_LEFT:
                event.x_val = -acc;
                break;
            case MOUSE_MOVE_RIGHT:
                event.x_val = acc;
                break;
            default:
                event.type = INPUT_EVENT_TYPE_NONE;
                break;
        }

        if (event.type != INPUT_EVENT_TYPE_NONE) {
            int err;

            err = k_msgq_put(&hids_queue, &event, K_NO_WAIT);
            if (err) {
                printk("No space in the queue for button pressed\n");
                return;
            }
            if (k_msgq_num_used_get(&hids_queue) == 1) {
                k_work_submit(&hids_work);
            }
        }

        if (acc < 10) {
            acc++;
        }
    }
}

K_TIMER_DEFINE(hid_report_timer, mouse_event_sender, NULL);

int main(void)
{
	int err;

	printk("Starting Bluetooth Peripheral HIDS mouse example\n");

	if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	/* DIS initialized at system boot with SYS_INIT macro. */
	hid_init();

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

    k_work_init(&hids_work, input_event_queue_handler);
	k_work_init(&adv_work, advertising_process);
	if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
		k_work_init(&pairing_work, pairing_process);
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	advertising_start();

	configure_buttons();


    /* start periodic timer that expires once every second */
    k_timer_start(&hid_report_timer, K_MSEC(20), K_MSEC(20));

	while (1) {
		k_sleep(K_SECONDS(1));
		/* Battery level simulation */
		bas_notify();
	}
}



/** @brief Function process and send keyboard state to all active connections
 *
 * Function process global keyboard state and send it to all connected
 * clients.
 *
 * @return 0 on success or negative error code.
 */
int key_report_send(void)
{
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn) {
            int err;

            err = key_report_con_send(&hid_keyboard_state,
                                      conn_mode[i].in_boot_mode,
                                      conn_mode[i].conn);
            if (err) {
                printk("Key report send error: %d\n", err);
                return err;
            }
        }
    }
    return 0;
}


/** @brief Function process keyboard state and sends it
 *
 *  @param pstate     The state to be sent
 *  @param boot_mode  Information if boot mode protocol is selected.
 *  @param conn       Connection handler
 *
 *  @return 0 on success or negative error code.
 */
int key_report_con_send(const struct keyboard_state *state,
                        bool boot_mode,
                        struct bt_conn *conn)
{
    int err = 0;
    uint8_t  data[INPUT_REPORT_KEYS_MAX_LEN];
    uint8_t *key_data;
    const uint8_t *key_state;
    size_t n;

    data[0] = state->ctrl_keys_state;
    data[1] = 0;
    key_data = &data[2];
    key_state = state->keys_state;

    for (n = 0; n < KEY_PRESS_MAX; ++n) {
        *key_data++ = *key_state++;
    }
    if (boot_mode) {
        err = bt_hids_boot_kb_inp_rep_send(&hids_obj, conn, data,
                                           sizeof(data), NULL);
    } else {
        err = bt_hids_inp_rep_send(&hids_obj, conn,
                                   INPUT_REP_KEYS_IDX, data,
                                   sizeof(data), NULL);
    }
    return err;
}