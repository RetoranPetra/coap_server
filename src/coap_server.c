/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include "AEDB_9140.h"
#include "imu.h"

LOG_MODULE_REGISTER(coap_server, CONFIG_COAP_SERVER_LOG_LEVEL);

/* The devicetree node identifier for the "led0" alias. */

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));


void main(void)
{
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

		printk("ySteps = %u, encoder position = %i",ySteps,encpos); //should work

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

  int ret;

  LOG_INF("Start CoAP-server sample");

  k_timer_init(&led_timer, on_led_timer_expiry, on_led_timer_stop);
  k_timer_init(&provisioning_timer, on_provisioning_timer_expiry, NULL);

  k_work_init(&provisioning_work, activate_provisioning);

  ret = ot_coap_init(&deactivate_provisionig, &on_light_request,
                     &on_generic_request, &on_float_request);
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

  openthread_state_changed_cb_register(openthread_get_default_context(),
                                       &ot_state_chaged_cb);
  openthread_start(openthread_get_default_context());

  LOG_DBG("Passed openthread_start in main!");

  coap_client_utils_init();

  LOG_DBG("Passed client start in main!");
  Setup_interrupt();
  encoderTestLoop();
  // See https://openthread.io/reference/group/api-channel-manager
  otInstance *inst = openthread_get_default_instance();
  otChannelManagerSetAutoChannelSelectionEnabled(inst, false);

  // Seems to not work.

  // otChannelManagerSetFavoredChannels(inst, 5); // Doesn't set channel, just
  //  sets a preferred one so auto selector chooses it more often.
  // otChannelManagerSetDelay(inst, 1);
  // otChannelManagerRequestChannelSelect(inst, 7); // Request channel change to
  //  7, doesn't work it seems.
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
