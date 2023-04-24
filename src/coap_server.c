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
#define step_pin 29
#define dir_pin 30
#define change_dir_pin 11
#define print_pin 12
//#define delta_phi = pi/2/100;

/* The devicetree node identifier for the "led0" alias. */

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));


void main(void)
{
  k_sleep(K_NSEC(2000U*1000U*1000U));
	uint32_t period = 1U * 1000U * 1000U ; //ms * to_us * to_ns
	int ySteps = 0;
	int32_t encpos = 0;
	int ret;
	int dir = 1;
  int yStepsGraph[300000];
  int32_t encposGraph[300000];
  int k = 0;

	Setup_interrupt();

	if (!device_is_ready(P0)) {
		return;
	}

	ret = gpio_pin_configure(P0, step_pin, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(P0, dir_pin, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(P0, change_dir_pin, GPIO_INPUT);
	if (ret < 0) {
		return;
	}

  ret = gpio_pin_configure(P0, print_pin, GPIO_INPUT);
	if (ret < 0) {
		return;
	}

	printk("Accelerating \n");

	while (1) {
		encpos = getPosition();

		gpio_pin_set(P0, step_pin, 1);

		k_sleep(K_NSEC(period/2U));

		gpio_pin_set(P0, step_pin, 0);

		k_sleep(K_NSEC(period/2U));

		ySteps++;

    yStepsGraph[k] = ySteps;
    encposGraph[k] = encpos;
    k++;
		

		if(gpio_pin_get(P0,change_dir_pin) == 1)
			dir = -dir;

		if(dir == 1)
			gpio_pin_set(P0, dir_pin, 1);
		if(dir == -1)
			gpio_pin_set(P0, dir_pin, 0);

    if(gpio_pin_get(P0,print_pin) == 1){
			for(int i=0; i<k; i++){
        printk("%u,%i\n",yStepsGraph[k],encposGraph[k]);
      }
    }

		if(period < MIN_PER)
			period = MIN_PER;
		if(period > MAX_PER)
			period = MAX_PER;
	}
}
