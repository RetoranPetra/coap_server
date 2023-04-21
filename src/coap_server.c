/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/openthread.h>
#include <openthread/thread.h>
// Channel management
#include <openthread/channel_manager.h>
#include <openthread/channel_monitor.h>
#include <nrf_802154.h>

#include "ot_coap_utils.h"
#include "coap_client_utils.h"

// Control modules
#include "imu.h"
#include "AEDB_9140.h"

LOG_MODULE_REGISTER(coap_server, CONFIG_COAP_SERVER_LOG_LEVEL);

#define OT_CONNECTION_LED DK_LED1
#define PROVISIONING_LED DK_LED3
#define LIGHT_LED DK_LED4

static struct k_work provisioning_work;

static struct k_timer led_timer;
static struct k_timer provisioning_timer;

static void on_light_request(uint8_t command)
{
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

static void activate_provisioning(struct k_work *item)
{
	ARG_UNUSED(item);

	ot_coap_activate_provisioning();

	k_timer_start(&led_timer, K_MSEC(100), K_MSEC(100));
	k_timer_start(&provisioning_timer, K_SECONDS(5), K_NO_WAIT);

	LOG_INF("Provisioning activated");
}

static void deactivate_provisionig(void)
{
	k_timer_stop(&led_timer);
	k_timer_stop(&provisioning_timer);

	if (ot_coap_is_provisioning_active()) {
		ot_coap_deactivate_provisioning();
		LOG_INF("Provisioning deactivated");
	}
}

static void on_provisioning_timer_expiry(struct k_timer *timer_id)
{
	ARG_UNUSED(timer_id);

	deactivate_provisionig();
}

static void on_led_timer_expiry(struct k_timer *timer_id)
{
	static uint8_t val = 1;

	ARG_UNUSED(timer_id);

	dk_set_led(PROVISIONING_LED, val);
	val = !val;
}

static void on_led_timer_stop(struct k_timer *timer_id)
{
	ARG_UNUSED(timer_id);

	dk_set_led_off(PROVISIONING_LED);
}

static void on_button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (buttons & DK_BTN4_MSK) {
		k_work_submit(&provisioning_work);
	}
	if (buttons & DK_BTN3_MSK) {
		//Should toggle through server selected.
		serverScroll();
	}
	if (buttons & DK_BTN2_MSK) {
		//Should send a provisioning request.
		coap_client_send_provisioning_request();
	}
	if (buttons & DK_BTN1_MSK) {
		//coap_client_toggle_one_light();
    coap_client_floatSend(10.768);
	}
}

static void on_thread_state_changed(otChangedFlags flags, struct openthread_context *ot_context,
				    void *user_data)
{
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

static void on_generic_request(//otChangedFlags flags, struct openthread_context *ot_context, void *user_data
char* msg) {
	//Something to deal with message would normally go here. However, message is just character string so it doesn't matter.
	LOG_INF("Generic Request event execution!");
}

static void on_float_request(double num) {
  LOG_INF("Number is: %f",num);
}

static struct openthread_state_changed_cb ot_state_chaged_cb = { .state_changed_cb =
									 on_thread_state_changed };

void main(void)
{
	//Need to sleep at start for logs to display correctly.
	k_msleep(1000);

	int ret;

	LOG_INF("Start CoAP-server sample");

	k_timer_init(&led_timer, on_led_timer_expiry, on_led_timer_stop);
	k_timer_init(&provisioning_timer, on_provisioning_timer_expiry, NULL);

	k_work_init(&provisioning_work, activate_provisioning);

	ret = ot_coap_init(&deactivate_provisionig, &on_light_request, &on_generic_request, &on_float_request);
	if (ret) {
		LOG_ERR("Could not initialize OpenThread CoAP");
		goto end;
	}

	ret = dk_leds_init();
	if (ret) {
		LOG_ERR("Could not initialize leds, err code: %d", ret);
		goto end;
	}

	ret = dk_buttons_init(on_button_changed);
	if (ret) {
		LOG_ERR("Cannot init buttons (error: %d)", ret);
		goto end;
	}

	openthread_state_changed_cb_register(openthread_get_default_context(), &ot_state_chaged_cb);
	openthread_start(openthread_get_default_context());

	LOG_DBG("Passed openthread_start in main!");

	coap_client_utils_init();

  Setup_interrupt();
  for (int i=1;;i++) {
    printf("Pos: %d, Vel: %.3f, Acc: %3f\n", getPosition(), getFloatVel(), getFloatAcc());
    k_msleep(ENCODER_SAMPLE_PERIOD*10);
    setVelocity();
    if (i%3 == 0) {
      setAcceleration();
      i = 0;
    }
  }



  // See https://openthread.io/reference/group/api-channel-manager
  otInstance *inst = openthread_get_default_instance();
  otChannelManagerSetAutoChannelSelectionEnabled(inst, false);

  // Seems to not work.

  //otChannelManagerSetFavoredChannels(inst, 5); // Doesn't set channel, just
  // sets a preferred one so auto selector chooses it more often.
  //otChannelManagerSetDelay(inst, 1);
  //otChannelManagerRequestChannelSelect(inst, 7); // Request channel change to
  // 7, doesn't work it seems.
  /*
  LOG_DBG("Favoured channel: %d", otChannelManagerGetFavoredChannels(inst));
  while (1) {
    k_msleep(1000);
    LOG_DBG("Channel is: %d", nrf_802154_channel_get());
  }
  */
  /*
  ICM20600_startup();
  imuTestLoop();
  */
end:
	return;
}
