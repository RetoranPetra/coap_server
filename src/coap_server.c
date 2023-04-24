/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include "AEDB_9140.h"

/* 1000 nsec = 1 usec */
#define MIN_PER 200000
#define MAX_PER 100000000
#define full_length_in_steps 3000
#define pi 3.14159265
//#define delta_phi = pi/2/100;

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
