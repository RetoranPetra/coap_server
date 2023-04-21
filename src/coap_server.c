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

#include "ot_coap_utils.h"
//L
#include <zephyr/drivers/gpio.h>
#include <stdlib.h> //For string to float conversion
//L
#include <math.h>
/* 1000 nsec = 1 usec */
#define MIN_PER 200000
#define MAX_PER 100000000
#define full_length_in_steps 3000
#define pi 3.14159265
//#define delta_phi = pi/2/100;

/*Once the client and server are paired. Buttons 1 and 2 on the client are used to set the desired position.
Button 3 is then pressed to move the motors to that specified position.
The setting of the position is shown in the log*/

LOG_MODULE_REGISTER(coap_server, CONFIG_COAP_SERVER_LOG_LEVEL);
//L
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

	switch (command) {

	case THREAD_STOP_MOTORS: //Code to stop motors
		LOG_DBG("Toggling LED\n");
		dk_set_led(LIGHT_LED, val);
		val = !val;
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
	gpio_pin_toggle(P0, 3);
	dk_set_led(LIGHT_LED, led_toggle); //Toggles LED and pin P0.16	
	LOG_INF("Float Request event execution!");
	//Change position and fixed position here to floats and set them accordingly
	fixed_position = get_float();
	LOG_DBG("GPIO Toggled!\nOn Float Request float to 2dp: %.2f\nMessage Count: %d\n", fixed_position, ++count);

}

static struct openthread_state_changed_cb ot_state_chaged_cb = { .state_changed_cb =
									 on_thread_state_changed };

void main(void)
{
	//Need to sleep at start for logs to display correctly.
	k_msleep(1000);
	/*uint32_t period = 1U * 1000U * 1000U ; //ms * to_us * to_ns
	int ySteps = 0;
	int yTargetSteps = 0;
	fixed_position = 0.0;
	int ret;
	int dir = 1;
	*/
	int ret;
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

	LOG_INF("Everything initialised correctly\n");
	//k_sleep(K_NSEC(4000U*1000U*1000U));

	/*while (1) {
		yTargetSteps = fixed_position*full_length_in_steps/100; //Only updates the target position when corresponding message received
		//Doesn't work when position set once
		if(yTargetSteps > ySteps){
			dir = 1;
			LOG_INF("Moving Forwards");
		}
		else if(yTargetSteps < ySteps){
			LOG_INF("Moving Back");
			dir = -1;
		}
		else{
			while(ySteps == yTargetSteps){ // Sets target steps when new position set
				yTargetSteps = fixed_position*full_length_in_steps/100;
				LOG_INF("Position reached!\nAwaiting Command");
			}
		}
 
		gpio_pin_set(P0, 3, 1); //Sets step input

		k_sleep(K_NSEC(period/2U));

		gpio_pin_set(P0, 3, 0); //Sets step input

		k_sleep(K_NSEC(period/2U));

		ySteps = ySteps+dir;

		if(dir == 1)
			gpio_pin_set(P0, 4, 1); //Sets driver input
		if(dir == -1)
			gpio_pin_set(P0, 4, 0); //Sets driver input

		if(period < MIN_PER)
			period = MIN_PER;
		if(period > MAX_PER)
			period = MAX_PER;
		//k_msleep(1000); //Sleep so inifinite loop does not disturb threads
	}
	*/
end:
	return;
}
