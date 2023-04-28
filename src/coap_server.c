/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include "AEDB_9140.h"

/* 1000 nsec = 1 usec */
#define MIN_PER 400000
#define GEAR_PER 2000000//10000000
#define GEAR_GUARD 400000
#define MAX_PER 1000000000
#define full_length_in_steps 3000
#define pi 3.14159265
#define step_pin 29
#define dir_pin 30
#define mode2_pin 0
#define mode1_pin 1
#define mode0_pin 5
#define delta_phi_start 0.01570796326//pi/2/100;

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

void main(void)
{
	uint32_t period = 10U * 1000U * 1000U ; //ms * to_us * to_ns
	uint32_t scalar = 1U;
	double per_c = period/1000000000.0;  //ns to s
	float ySteps = 0;
	float yTargetSteps = 2000;
	int ret;
	int dir = 1;
	double a = 0;
	double accel = 0;
	double delta_phi = delta_phi_start;
	double placeholder = period;
  int32_t encpos = 0;

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

	ret = gpio_pin_configure(P0, mode2_pin, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(P0, mode1_pin, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(P0, mode0_pin, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	gpio_pin_set(P0, mode2_pin, 1);
	gpio_pin_set(P0, mode1_pin, 1);
	gpio_pin_set(P0, mode0_pin, 0);

	printk("Control \n");
	k_sleep(K_NSEC(8000U*1000U*1000U));

	while (1) {
		delta_phi = delta_phi_start/scalar;
    encpos = getPosition();

		while( yTargetSteps <= ySteps && ySteps < yTargetSteps+1){
			printk("Target Reached at %u\n\n",encpos);
			//period = 2U * 1000U * 1000U;
			//per_c = period/1000000000.0;
			yTargetSteps = 3000 - yTargetSteps;
		}

		gpio_pin_set(P0, step_pin, 1);

		k_sleep(K_NSEC(period/scalar/2U));

		gpio_pin_set(P0, step_pin, 0);

		k_sleep(K_NSEC(period/scalar/2U));

		ySteps = ySteps + 1.0/scalar*dir;

		//printf("period = %u, per_c = %f, ySteps = %f, accel = %f, scalar = %u\n",period,per_c,ySteps,accel,scalar);

		if(yTargetSteps-ySteps < 0){
			if(dir > 0)
				a = -20*pi;
			if(dir < 0)
				a = 15*pi*(yTargetSteps-ySteps)/3000+1;
			
		}
		if(yTargetSteps-ySteps > 0){
			if(dir > 0)
				a = 15*pi*(yTargetSteps-ySteps)/3000+1;
			if(dir<0)
				a = 20*pi;
		}

		accel = a*dir;

		if(dir == 1)
			gpio_pin_set(P0, dir_pin, 0); //Away from motor

		if(dir == -1)
			gpio_pin_set(P0, dir_pin, 1); //Towards motor
    
    if( (delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel < 0)
    {
		per_c = sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) - delta_phi/accel) - delta_phi/(accel*per_c*2);
		accel = -accel;
		dir = -dir;
    }
    else {
        if(accel > 0.1) 
        {
    		    per_c = sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel) - delta_phi/(accel*per_c*2);
    	}
    	else
            if(accel < -0.1)
    		{
    		        per_c = -sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel) - delta_phi/(accel*per_c*2);
    		}
	}

		placeholder = per_c*1000000000;
		period = placeholder;

		if(period*scalar < MIN_PER){
			period = MIN_PER;
			per_c = period/1000000000.0;
		}
		if(period*scalar > MAX_PER){
			period = MAX_PER;
			per_c = period/1000000000.0;
			//printf("Should have theoretically stopped, per_c = %f\n",per_c);
		}

		if(period/scalar > GEAR_PER+GEAR_GUARD){ //if going slower than a predefined speed
			scalar = scalar*2U;  //gear down
		}
		else{
			if(period/scalar*2 < GEAR_PER-GEAR_GUARD) //if going faster than said speed
				scalar = scalar/2U; //gear up
				if(scalar < 1U) scalar = 1U;
		}

	}
}