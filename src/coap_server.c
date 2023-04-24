/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <dk_buttons_and_leds.h>
#include <openthread/thread.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/openthread.h>
// Channel management
#include <nrf_802154.h>
#include <openthread/channel_manager.h>
#include <openthread/channel_monitor.h>

#include "coap_client_utils.h"
#include "ot_coap_utils.h"

// Control modules
#include "AEDB_9140.h"
#include "imu.h"

/* 1000 nsec = 1 usec */
#define MIN_PER 200000
#define MAX_PER 100000000
#define full_length_in_steps 3000
#define pi 3.14159265
//#define delta_phi = pi/2/100;

LOG_MODULE_REGISTER(coap_server, CONFIG_COAP_SERVER_LOG_LEVEL);

const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

#define OT_CONNECTION_LED DK_LED1
#define PROVISIONING_LED DK_LED3
#define LIGHT_LED DK_LED4

static struct k_work provisioning_work;

static struct k_timer led_timer;
static struct k_timer provisioning_timer;

static void on_light_request(uint8_t command) {
  static uint8_t val;

  switch (command) {
  case THREAD_COAP_UTILS_LIGHT_CMD_ON:
    dk_set_led_on(LIGHT_LED);
    val = 1;
    break;

  case THREAD_COAP_UTILS_LIGHT_CMD_OFF:
    dk_set_led_off(LIGHT_LED);
    val = 0;
    break;

  case THREAD_COAP_UTILS_LIGHT_CMD_TOGGLE:
    val = !val;
    dk_set_led(LIGHT_LED, val);
    break;

  default:
    break;
  }
}

static void activate_provisioning(struct k_work *item) {
  ARG_UNUSED(item);

  ot_coap_activate_provisioning();

  k_timer_start(&led_timer, K_MSEC(100), K_MSEC(100));
  k_timer_start(&provisioning_timer, K_SECONDS(5), K_NO_WAIT);

  LOG_INF("Provisioning activated");
}

static void deactivate_provisionig(void) {
  k_timer_stop(&led_timer);
  k_timer_stop(&provisioning_timer);

  if (ot_coap_is_provisioning_active()) {
    ot_coap_deactivate_provisioning();
    LOG_INF("Provisioning deactivated");
  }
}

static void on_provisioning_timer_expiry(struct k_timer *timer_id) {
  ARG_UNUSED(timer_id);

  deactivate_provisionig();
}

static void on_led_timer_expiry(struct k_timer *timer_id) {
  static uint8_t val = 1;

  ARG_UNUSED(timer_id);

  dk_set_led(PROVISIONING_LED, val);
  val = !val;
}

static void on_led_timer_stop(struct k_timer *timer_id) {
  ARG_UNUSED(timer_id);

  dk_set_led_off(PROVISIONING_LED);
}

static void on_button_changed(uint32_t button_state, uint32_t has_changed) {
  uint32_t buttons = button_state & has_changed;

  if (buttons & DK_BTN4_MSK) {
    k_work_submit(&provisioning_work);
  }
  if (buttons & DK_BTN3_MSK) {
    // Should toggle through server selected.
    serverScroll();
  }
  if (buttons & DK_BTN2_MSK) {
    // Should send a provisioning request.
    coap_client_send_provisioning_request();
  }
  if (buttons & DK_BTN1_MSK) {
    // coap_client_toggle_one_light();
    coap_client_floatSend(10.768);
  }
}

static void on_thread_state_changed(otChangedFlags flags,
                                    struct openthread_context *ot_context,
                                    void *user_data) {
  if (flags & OT_CHANGED_THREAD_ROLE) {
    switch (otThreadGetDeviceRole(ot_context->instance)) {
    case OT_DEVICE_ROLE_CHILD:
    case OT_DEVICE_ROLE_ROUTER:
    case OT_DEVICE_ROLE_LEADER:
      dk_set_led_on(OT_CONNECTION_LED);
      break;

    case OT_DEVICE_ROLE_DISABLED:
    case OT_DEVICE_ROLE_DETACHED:
    default:
      dk_set_led_off(OT_CONNECTION_LED);
      deactivate_provisionig();
      break;
    }
  }
}

static void
on_generic_request( // otChangedFlags flags, struct openthread_context
                    // *ot_context, void *user_data
    char *msg) {
  // Something to deal with message would normally go here. However, message is
  // just character string so it doesn't matter.
  LOG_INF("Generic Request event execution!");
}

static void on_float_request(double num) { LOG_INF("Number is: %f", num); }

static struct openthread_state_changed_cb ot_state_chaged_cb = {
    .state_changed_cb = on_thread_state_changed};

void main(void) {
  // Need to sleep at start for logs to display correctly.
  k_msleep(1000);
uint32_t period = 1U * 1000U * 1000U ; //ms * to_us * to_ns
	int ySteps = 0;
	int32_t encpos = 0;
	int ret;
	int dir = 1;

	Setup_interrupt();

	if (!device_is_ready(P0)) {
		return;
	}

	ret = gpio_pin_configure(P0, 3, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(P0, 4, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(P0, 11, GPIO_INPUT);
	if (ret < 0) {
		return;
	}

	printk("Accelerating \n");
	k_sleep(K_NSEC(4000U*1000U*1000U));

	while (1) {
		encpos = getPosition();

		gpio_pin_set(P0, 3, 1);

		k_sleep(K_NSEC(period/2U));

		gpio_pin_set(P0, 3, 0);

		k_sleep(K_NSEC(period/2U));

		ySteps++;

		printk("ySteps = %u, encoder position = %i",ySteps,encpos); //test

		if(gpio_pin_get(P0,11) == 1)
			dir = -dir;

		if(dir == 1)
			gpio_pin_set(P0, 4, 1);
		if(dir == -1)
			gpio_pin_set(P0, 4, 0);

		if(period < MIN_PER)
			period = MIN_PER;
		if(period > MAX_PER)
			period = MAX_PER;
	}
}
