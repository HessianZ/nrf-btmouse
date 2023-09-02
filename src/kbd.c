//
// Created by Hessian on 2023/8/31.
//

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
#include <bluetooth/services/hids.h>
#include <dk_buttons_and_leds.h>

#include "kbd.h"


extern struct bt_hids hids_obj;

static const uint8_t INPUT_DEMO_STR[] = {
        HID_KEY_E,	/* Key h */
        HID_KEY_E,	/* Key e */
        HID_KEY_T,	/* Key l */
        HID_KEY_R,	/* Key l */
        HID_KEY_E,	/* Key o */
        HID_KEY_E,	/* Key o */
        HID_KEY_ENTER,	/* Key Return */
};

static const uint8_t shift_key[] = { 225 };

/* Current report status
 */
struct keyboard_state hid_keyboard_state;



/** @brief Change key code to ctrl code mask
 *
 *  Function changes the key code to the mask in the control code
 *  field inside the raport.
 *  Returns 0 if key code is not a control key.
 *
 *  @param key Key code
 *
 *  @return Mask of the control key or 0.
 */
static uint8_t button_ctrl_code(uint8_t key)
{
    if (KEY_CTRL_CODE_MIN <= key && key <= KEY_CTRL_CODE_MAX) {
        return (uint8_t)(1U << (key - KEY_CTRL_CODE_MIN));
    }
    return 0;
}


static int hid_kbd_state_key_set(uint8_t key)
{
    uint8_t ctrl_mask = button_ctrl_code(key);

    if (ctrl_mask) {
        hid_keyboard_state.ctrl_keys_state |= ctrl_mask;
        return 0;
    }
    for (size_t i = 0; i < KEY_PRESS_MAX; ++i) {
        if (hid_keyboard_state.keys_state[i] == 0) {
            hid_keyboard_state.keys_state[i] = key;
            return 0;
        }
    }
    /* All slots busy */
    return -EBUSY;
}


static int hid_kbd_state_key_clear(uint8_t key)
{
    uint8_t ctrl_mask = button_ctrl_code(key);

    if (ctrl_mask) {
        hid_keyboard_state.ctrl_keys_state &= ~ctrl_mask;
        return 0;
    }
    for (size_t i = 0; i < KEY_PRESS_MAX; ++i) {
        if (hid_keyboard_state.keys_state[i] == key) {
            hid_keyboard_state.keys_state[i] = 0;
            return 0;
        }
    }
    /* Key not found */
    return -EINVAL;
}

/** @brief Press a button and send report
 *
 *  @note Functions to manipulate hid state are not reentrant
 *  @param keys
 *  @param cnt
 *
 *  @return 0 on success or negative error code.
 */
static int hid_buttons_press(const uint8_t *keys, size_t cnt)
{
    while (cnt--) {
        int err;

        err = hid_kbd_state_key_set(*keys++);
        if (err) {
            printk("Cannot set selected key.\n");
            return err;
        }
    }

    return key_report_send();
}

/** @brief Release the button and send report
 *
 *  @note Functions to manipulate hid state are not reentrant
 *  @param keys
 *  @param cnt
 *
 *  @return 0 on success or negative error code.
 */
static int hid_buttons_release(const uint8_t *keys, size_t cnt)
{
    while (cnt--) {
        int err;

        err = hid_kbd_state_key_clear(*keys++);
        if (err) {
            printk("Cannot clear selected key.\n");
            return err;
        }
    }

    return key_report_send();
}


void button_text_changed(bool down)
{
    static const uint8_t *chr = INPUT_DEMO_STR;

    if (down) {
        hid_buttons_press(chr, 1);
    } else {
        hid_buttons_release(chr, 1);
        if (++chr == (INPUT_DEMO_STR + sizeof(INPUT_DEMO_STR))) {
            chr = INPUT_DEMO_STR;
        }
    }
}

void type_demo_chars()
{
    for (int i = 0; i < ARRAY_SIZE(INPUT_DEMO_STR); ++i) {
        hid_buttons_press(&INPUT_DEMO_STR[i], 1);
        k_sleep(K_MSEC(5));
        hid_buttons_release(&INPUT_DEMO_STR[i], 1);
        k_sleep(K_MSEC(200));
    }
}


static void button_shift_changed(bool down)
{
    if (down) {
        hid_buttons_press(shift_key, 1);
    } else {
        hid_buttons_release(shift_key, 1);
    }
}


void caps_lock_handler(const struct bt_hids_rep *rep)
{
    uint8_t report_val = ((*rep->data) & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) ?
                         1 : 0;
    dk_set_led(DK_LED1, report_val);
}
