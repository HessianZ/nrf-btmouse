/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#include <nfc_t2t_lib.h>
#include <nfc/ndef/launchapp_msg.h>

#include <dk_buttons_and_leds.h>
#include <nfc/ndef/uri_msg.h>

#define NDEF_MSG_BUF_SIZE	256
#define NFC_FIELD_LED		DK_LED1

/** .. include_startingpoint_pkg_def_launchapp_rst */
/* Package: no.nordicsemi.android.nrftoolbox */
static const uint8_t android_pkg_name[] = "com.tencent.mm";

/* URI nrf-toolbox://main/ */
static const uint8_t universal_link[] = "weixin://dl/scan";
/** .. include_endpoint_pkg_def_launchapp_rst */

/* Buffer used to hold an NFC NDEF message. */
static uint8_t ndef_msg_buf[NDEF_MSG_BUF_SIZE];


static void nfc_callback(void *context,
			 nfc_t2t_event_t event,
			 const uint8_t *data,
			 size_t data_length)
{
	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(data_length);

	switch (event) {
	case NFC_T2T_EVENT_FIELD_ON:
		dk_set_led_on(NFC_FIELD_LED);
		break;
	case NFC_T2T_EVENT_FIELD_OFF:
		dk_set_led_off(NFC_FIELD_LED);
		break;
	default:
		break;
	}
}

void nfc_task(void)
{
	int err;
	size_t len = sizeof(ndef_msg_buf);

	printk("Starting NFC Launch app example\n");

	/* Configure LED-pins as outputs */
	err = dk_leds_init();
	if (err) {
		printk("Cannot init LEDs!\n");
		goto fail;
	}

	/* Set up NFC */
	err = nfc_t2t_setup(nfc_callback, NULL);
	if (err) {
		printk("Cannot setup NFC T2T library!\n");
		goto fail;
	}

	/* Encode launch app data  */
//	err = nfc_launchapp_msg_encode(android_pkg_name,
//				       sizeof(android_pkg_name)-1,
//				       universal_link,
//				       sizeof(universal_link)-1,
//				       ndef_msg_buf,
//				       &len);

    const uint8_t url[] = "//hessian.cn/";
	/* Encode launch URI  */
	err = nfc_ndef_uri_msg_encode(NFC_URI_HTTP,
                                  url,
				       sizeof(url)-1,
				       ndef_msg_buf,
				       &len);
	if (err) {
		printk("Cannot encode message!\n");
		goto fail;
	}

	/* Set created message as the NFC payload */
	err = nfc_t2t_payload_set(ndef_msg_buf, len);
	if (err) {
		printk("Cannot set payload!\n");
		goto fail;
	}

	/* Start sensing NFC field */
	err = nfc_t2t_emulation_start();
	if (err) {
		printk("Cannot start emulation!\n");
		goto fail;
	}

	printk("NFC configuration done\n");
	return;

fail:
#if CONFIG_REBOOT
	sys_reboot(SYS_REBOOT_COLD);
#endif /* CONFIG_REBOOT */

}
