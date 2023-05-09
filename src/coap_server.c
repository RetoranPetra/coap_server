/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

// Toggles to disable or enable functionality.
#include "coap_server_client_interface.h"
#define CLIENT
#define SERVER
#define IMU
//#define ENCODER

#include <dk_buttons_and_leds.h>
#include <openthread/thread.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/openthread.h>
#include <zephyr/drivers/gpio.h>
//#include <zephyr/drivers/flash.h>
// Channel management
#include <nrf_802154.h>
#include <openthread/channel_manager.h>
#include <openthread/channel_monitor.h>
#ifdef CLIENT
#include "coap_client_utils.h"
#endif
#ifdef SERVER
#include "ot_coap_utils.h"
#endif

// Control modules
#ifdef ENCODER
#include "AEDB_9140.h"
#endif
#ifdef IMU
//#include "ICM20600.h"
#endif

#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
/* 1000 nsec = 1 usec */
#define MIN_PER 600000
#define GEAR_PER 2000000//10000000
#define Sampling_period 10 // in milli  secs
#define pi 3.14159265359
#define radius 8.5E-3
#define distance_per_step 2.0*3.14*8.5E-3/200.0

#define Stepper_Pin DT_ALIAS(stepper)
#define Direction_pin DT_ALIAS(stepperdir)
#define M0_Pin DT_ALIAS(m0)
#define M1_Pin DT_ALIAS(m1)
#define M2_Pin DT_ALIAS(m2)

#define GEAR_PER 2000000//10000000
#define GEAR_GUARD 400000

static const struct gpio_dt_spec STEP = GPIO_DT_SPEC_GET(Stepper_Pin, gpios);
static const struct gpio_dt_spec DIR = GPIO_DT_SPEC_GET(Direction_pin, gpios);


double Step_time_interval;

double   Vm = 0;

double phi = 0;

bool mainloop = false;

struct encoderMessage currentEncode = {};

void my_work_handler(struct k_work *work)
{
	double   xddot = 0;
	double   t = 0;
	double   Theta_reference = 0;
	double   error = 0;
	double 	 prev_error = 0;
	double 	 integral_error = 0;
	double 	 devirative_error =0;
	double   input =0 ;
	double   input_prev = 0;
	double   kp = 4;
	double   ki = 0;
	double   kd = 0.8;
	phi = currentEncode.position;
	//SimpleComplementaryFilter(getRawAccelerationX()*9.81/1000,getRawAccelerationY()*9.81/1000,getRawAccelerationZ()*9.81/1000,getRawGyroscopeX()*2*pi/360,getRawGyroscopeY()*2*pi/360,getRawGyroscopeZ()*2*pi/360,Sampling_period*pow(10,-3));
	
	//printf("Start t = %f\n", t);
	//printf("Demand Velocity is: %f\n",Vm);
	input_prev = input;
	prev_error = error; 
	//printf("Prev input: %f",input_prev);
	error = Theta_reference - (-phi); 
	///printf("Error: %f",error);
	integral_error = integral_error + error*Sampling_period;
	devirative_error = (error - prev_error)/Sampling_period;
	input = (kp*error + ki*integral_error + kd*devirative_error); // PID Control
	if(input < 0)
	{
		gpio_pin_set_dt(&DIR,1);
	}
	else
	{
		gpio_pin_set_dt(&DIR,0);
	}
	if(Step_time_interval <= 0)
	{
		Step_time_interval = -Step_time_interval;
	}
	if(input <= 0)
	{
		input = -input;
	}
	if(input < 66.75 && input > 0.134){
		Step_time_interval = (distance_per_step/(input*66.75/(0.10)))*pow(10,6);
		
	}
	else if(input > 66.75)
	{
		Step_time_interval = 4; 
	}
	else if(input < 0.134)
	{
		Step_time_interval = 2000; 
	}
	//printf("Step Period: %f,Error: %f,Input: %f,Theta: %f,Phi:%f,Psi:%f\n",Step_time_interval,error,input,theta,phi,psi);
	//printf("Input: %f,Theta:%f,\n",input,phi);
	
	
	//printf("Step Period: %f,Theta: %f,Phi:%f,Psi:%f\n",Step_time_interval,theta,phi,psi);
	//t++;
}
K_WORK_DEFINE(my_work, my_work_handler);

void my_timer_handler(struct k_timer *timer_id)
{
	k_work_submit(&my_work);
}
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);


LOG_MODULE_REGISTER(coap_server, CONFIG_COAP_SERVER_LOG_LEVEL);


#define OT_CONNECTION_LED DK_LED1
#define PROVISIONING_LED DK_LED3
#define LIGHT_LED DK_LED4
#ifdef SERVER
static struct k_work provisioning_work;

static struct k_timer led_timer;
static struct k_timer provisioning_timer;

static void on_light_request(uint8_t command) {
  static uint8_t val;

  // switch (command) {
  // case THREAD_COAP_UTILS_LIGHT_CMD_ON:
  //   dk_set_led_on(LIGHT_LED);
  //   val = 1;
  //   break;

  // case THREAD_COAP_UTILS_LIGHT_CMD_OFF:
  //   dk_set_led_off(LIGHT_LED);
  //   val = 0;
  //   break;

  // case THREAD_COAP_UTILS_LIGHT_CMD_TOGGLE:
  //   val = !val;
  //   dk_set_led(LIGHT_LED, val);
  //   break;

  // default:
  //   break;
  // }
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
#endif


#ifdef SERVER
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
static void on_percentage_request(struct percentageStruct percent) {
  ARG_UNUSED(percent);
  LOG_INF("Percentage request callback");
}

static void on_encoder_request(struct encoderMessage encode) {
  //LOG_DBG("Message Number: %i\nPosition:%i,Velocity:%i",encode.messageNum,encode.position,encode.velocity);
  //LOG_DBG("Encoder request callback!");
  if (currentEncode.messageNum + 1 != encode.messageNum) {
    LOG_INF("Dropped 1!");
  }
  currentEncode = encode;
}

static struct openthread_state_changed_cb ot_state_chaged_cb = {
    .state_changed_cb = on_thread_state_changed};
#endif

const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

static void on_button_changed(uint32_t button_state, uint32_t has_changed) {
  uint32_t buttons = button_state & has_changed;
#ifdef SERVER
  if (buttons & DK_BTN4_MSK) {
    k_work_submit(&provisioning_work);
  }
#endif
#ifdef CLIENT
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
    //coap_client_floatSend(10.768);
    /*
    struct percentageStruct example = {.percentages = {1.0,1.0,1.0},
      .identifier = "Hello!"};
    coap_client_percentageSend(example);
    */
   //printf("per_c = %f, ySteps = %f, accel = %f, ySpeed = %f, dir = %d, encpos = %i\n",per_c,ySteps,accel,ySpeed,dir,currentEncode.position);
   mainloop = true;
    // struct encoderMessage example = {.position = 3000,
    //   .messageNum=0,.velocity=20};
    // coap_client_encoderSend(example);
  }
#endif
}

// static void on_button_changed(uint32_t button_state, uint32_t has_changed) {
//   uint32_t buttons = button_state & has_changed;
//   if (buttons & DK_BTN4_MSK) {
// 	printf("time is like %u and %u when %u\n", uptime, oldtime, k_uptime_ticks());
//   }
//   if (buttons & DK_BTN3_MSK) {
// 	printf("P = %f, I = %f, D = %f, a = %f, error = %f\n",kP*(yTargetSteps-ySteps),kI*ierr,kD*ySpeed,a,yTargetSteps - ySteps);
//   }
//   if (buttons & DK_BTN2_MSK) {
// 	printf("per_c = %f, ySteps = %f, accel = %f, ySpeed = %f, dir = %d\n",per_c,ySteps,accel,ySpeed,dir);
//   }
//   if (buttons & DK_BTN1_MSK) {
// 	printf("Encpos = %i\n", getPosition());
//   }
// }

//const struct device *flashmem = DEVICE_DT_GET(DT_PATH(soc,flash_controller_4001e000));


void main(void)
{
    // Need to sleep at start for logs to display correctly.
  k_msleep(1000);
#ifdef SERVER
  int ret;

  LOG_INF("Start CoAP-server sample");

  k_timer_init(&led_timer, on_led_timer_expiry, on_led_timer_stop);
  k_timer_init(&provisioning_timer, on_provisioning_timer_expiry, NULL);

  k_work_init(&provisioning_work, activate_provisioning);

  ret = ot_coap_init(&deactivate_provisionig, &on_light_request,
                     &on_generic_request, &on_float_request, &on_percentage_request, &on_encoder_request);
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
#endif
#ifdef CLIENT
  coap_client_utils_init();
  LOG_DBG("Passed client start in main!");
#endif /* ifdef CLIENT */
#ifdef IMU
  //ICM20600_startup();
#endif /* ifdef IMU */
#ifdef ENCODER
  Setup_interrupt();
#endif /* ifdef ENCODER */

	if (!device_is_ready(P0)) {
		return;
	}

	// ret = flash_get_page_info_by_offs(flashmem, addrs, &pginf);
	
	// printf("getinfo ret %d and offset %u, size %u and i %u with max size = %u with doubles per page = %u\n",ret, pginf.start_offset,pginf.size,pginf.index,flash_get_page_count(flashmem), pginf.size/sizeof(per_c));
	// ret = flash_erase(flashmem, addrs, pginf.size*maxPages);
	// printf("erase ret %d\n",ret);

	printk("IMU Wirelessly Correct\n");
	k_sleep(K_NSEC(2000U*1000U*1000U));

	while(!mainloop){
		k_sleep(K_NSEC(20000U));
	}

	k_timer_start(&my_timer, K_SECONDS(0), K_MSEC(Sampling_period));
	while (1) {		
		if(Step_time_interval<10000 && Step_time_interval>4)
		{
			gpio_pin_set_dt(&STEP,1);
			k_sleep(K_USEC(Step_time_interval));
			gpio_pin_set_dt(&STEP,0);
			k_sleep(K_USEC(Step_time_interval));
		}
		else if(Step_time_interval<=4)
		{
			gpio_pin_set_dt(&STEP,1);
			k_sleep(K_USEC(2));
			gpio_pin_set_dt(&STEP,0);
			k_sleep(K_USEC(2));
		}
	}

  end: return;
}
