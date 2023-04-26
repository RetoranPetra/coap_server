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
#define step_pin 29
#define dir_pin 30
//#define change_dir_pin 11
//#define print_pin 12
//#define delta_phi = pi/2/100;

LOG_MODULE_REGISTER(coap_server, CONFIG_COAP_SERVER_LOG_LEVEL);

int yTargetSteps = 0;

const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
//L
//int position = 0; //Changed from Flag
float fixed_position = 0; //Placeholder for position
uint8_t led_toggle = 0;	//To toggle the led GPIO when message received
int count = 0; //To count message dropout rate

#define OT_CONNECTION_LED DK_LED1
#define PROVISIONING_LED DK_LED3
#define LIGHT_LED DK_LED4

static struct k_work provisioning_work;

static struct k_timer led_timer;
static struct k_timer provisioning_timer;

static void on_light_request(uint8_t command)
{
	static uint8_t val;
/*
	switch (command) {

	case LIGHT_URI_PATH: //Code to stop motors
		LOG_DBG("Toggling LED\n");
		dk_set_led(LIGHT_LED, val);
		val = !val;
		break;

	default:
		break;
	}
	*/
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
	LOG_INF("Timer Expired. Provisioning deactivated!\n");
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
		LOG_INF("Provisioning request sent\n");
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

static void on_generic_request(otChangedFlags flags, struct openthread_context *ot_context, void *user_data) {
	//Something to deal with message would normally go here. However, message is just character string so it doesn't matter.
	static uint8_t val;
	LOG_INF("Generic Request event execution!");
	dk_set_led(LIGHT_LED, val);
	val = !val;
}

static void on_float_request(otChangedFlags flags, struct openthread_context *ot_context, void *user_data) {
	yTargetSteps = get_float() *3000;
}

static struct openthread_state_changed_cb ot_state_chaged_cb = { .state_changed_cb =
									 on_thread_state_changed };


void main(void) {
	// Need to sleep at start for logs to display correctly.
	k_msleep(2000);
	int ret; 
	k_timer_init(&led_timer, on_led_timer_expiry, on_led_timer_stop);
	k_timer_init(&provisioning_timer, on_provisioning_timer_expiry, NULL);
	
	k_work_init(&provisioning_work, activate_provisioning);

	ret = ot_coap_init(&deactivate_provisionig, &on_light_request, &on_generic_request, &on_float_request);
	if (ret) {
		LOG_ERR("Could not initialize OpenThread CoAP");
	}

	ret = dk_leds_init();
	if (ret) {
		LOG_ERR("Could not initialize leds, err code: %d", ret);
	}

	ret = dk_buttons_init(on_button_changed);
	if (ret) {
		LOG_ERR("Cannot init buttons (error: %d)", ret);
	}

	openthread_state_changed_cb_register(openthread_get_default_context(), &ot_state_chaged_cb);
	openthread_start(openthread_get_default_context());

	uint32_t period = 1U * 800U * 1000U ; //ms * to_us * to_ns
	double per_c = period/1000000000.0;  //ns to s
	int ySteps = 0;

	int dir = 1;
	double a = -20;
	double accel = -20;
	double placeholder = period;

	if (!device_is_ready(P0)) {
		return;
	}

	ret = gpio_pin_configure(P0, step_pin, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	// ret = gpio_pin_configure(P0, mode2_pin, GPIO_OUTPUT_INACTIVE);
	// if (ret < 0) {
	// 	return;
	// }

	ret = gpio_pin_configure(P0, dir_pin, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	//gpio_pin_set(P0, mode2_pin, 1);

	printk("Accelerating \n");
	k_sleep(K_NSEC(8000U*1000U*1000U));

	
	while (1) {
		gpio_pin_set(P0, step_pin, 1);

		k_sleep(K_NSEC(period));

		gpio_pin_set(P0, step_pin, 0);

		k_sleep(K_NSEC(period));

		//printk("Period: %u\n", period*2);
		//printk("yTargetSteps: %i\n", yTargetSteps);

		ySteps = ySteps + dir;

		if(ySteps < yTargetSteps){
			a = 15*pi*(yTargetSteps-ySteps)/3000;
			accel = a*dir;
		}
		if(ySteps > yTargetSteps){
			a = -15*pi*(yTargetSteps-ySteps)/3000;
			accel = a*dir;
		}
		while(ySteps == yTargetSteps){
			printk("Target Reached");
		}
		if(dir == 1)
			gpio_pin_set(P0, dir_pin, 0); //Away from motor

		if(dir == -1)
			gpio_pin_set(P0, dir_pin, 1); //Towards motor
    
    	if( (pi/2/100)*(pi/2/100)/(accel*accel*per_c*per_c*4) + pi/2/100/accel < 0)
    	{
			per_c = sqrt((pi/2/100)*(pi/2/100)/(accel*accel*per_c*per_c*4) - pi/2/100/accel) - pi/2/100/(accel*per_c*2);
			accel = -accel;
			dir = -dir;
   		}
    	else {
        	if(accel > 0.1) 
        	{
    		    per_c = sqrt((pi/2/100)*(pi/2/100)/(accel*accel*per_c*per_c*4) + pi/2/100/accel) - pi/2/100/(accel*per_c*2);
    		}
    	else {
            if(accel < -0.1)
    		{
    		        per_c = -sqrt((pi/2/100)*(pi/2/100)/(accel*accel*per_c*per_c*4) + pi/2/100/accel) - pi/2/100/(accel*per_c*2);
    		}
		}
	}


	placeholder = per_c*1000000000/2;
	period = placeholder;

	if(period < MIN_PER){
		period = MIN_PER;
		per_c = period/1000000000.0*2;
	}
	if(period > MAX_PER){
		period = MAX_PER;
		per_c = period/1000000000.0*2;
		printf("Should have theoretically stopped, per_c = %f\n",per_c);
	}
}
