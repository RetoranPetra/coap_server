/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <dk_buttons_and_leds.h>
#include <openthread/thread.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
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

const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

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

		printk("ySteps = %u, encoder position = %i",ySteps,encpos);

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
