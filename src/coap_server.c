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
//#define step_pin 29
//#define dir_pin 30
//#define change_dir_pin 11
//#define print_pin 12
//#define delta_phi = pi/2/100;

LOG_MODULE_REGISTER(coap_server, CONFIG_COAP_SERVER_LOG_LEVEL);

const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

void main(void) {
  // Need to sleep at start for logs to display correctly.
  k_msleep(2000);
  LOG_DBG("Start Graph sample");
	uint32_t period = 1U * 1000U * 1000U ; //ms * to_us * to_ns
	int ySteps = 0;
	int32_t encpos = 0;
	int ret;
	int dir = 1;
  int yStepsGraph[3000];
  int32_t encposGraph[3000];
  int k = 0;
  int should_stop = 0;

	Setup_interrupt();

	if (!device_is_ready(P0)) {
		return;
	}

	ret = gpio_pin_configure(P0, 29, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(P0, 30, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(P0, 11, GPIO_INPUT);
	if (ret < 0) {
		return;
	}

  ret = gpio_pin_configure(P0, 12, GPIO_INPUT);
	if (ret < 0) {
		return;
	}

	LOG_DBG("Start main sample");

	while (1) {
		if(!should_stop){
		encpos = getPosition();

		gpio_pin_set(P0, 29, 1);

		k_sleep(K_NSEC(period/2U));

		gpio_pin_set(P0, 29, 0);

		k_sleep(K_NSEC(period/2U));

		ySteps++;

    yStepsGraph[k] = ySteps;
    encposGraph[k] = encpos;
    k++;
    if(k>2000){
		k_sleep(K_NSEC(1U*1000U*1000U*1000U));
      for(int i=0; i<k; i++){
        printk("%u,%i\n",yStepsGraph[i],encposGraph[i]);
		k_sleep(K_NSEC(10U*1000U*1000U));
      }
      should_stop = 1;
    }
		}

		if(gpio_pin_get(P0,11) == 1)
			dir = -dir;

		if(dir == 1)
			gpio_pin_set(P0, 30, 1);
		if(dir == -1)
			gpio_pin_set(P0, 30, 0);

    if(gpio_pin_get(P0, 12) == 1){
		k_sleep(K_NSEC(2U*1000U*1000U*1000U));
      for(int i=0; i<k; i++){
        LOG_DBG("%u,%i\n",yStepsGraph[i],encposGraph[i]);
		k_sleep(K_NSEC(100U*1000U*1000U));
      }
    }

		if(period < MIN_PER)
			period = MIN_PER;
		if(period > MAX_PER)
			period = MAX_PER;
	}
}
