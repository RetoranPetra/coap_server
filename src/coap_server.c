/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

// Toggles to disable or enable functionality.
#include "coap_server_client_interface.h"
#define CLIENT
#define SERVER
#define CLIENT
#define SERVER
//#define IMU
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
#include "imu.h"
#endif

#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
/* 1000 nsec = 1 usec */
#define MIN_PER 600000
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
#define MAXENCODER 30000.0

#define addrs 0x0007A000//0x00070000//
#define notusable 0x9C7B
#define maxPages 6//16//
#define arraySize 20
#define currentOffset -0.00041

bool mainloop = false;
bool newMessage = false;
bool manualStop = false;;


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

struct encoderMessage currentEncode = {};
static void on_encoder_request(struct encoderMessage encode) {
  //LOG_DBG("Message Number: %i\nPosition:%i,Velocity:%i",encode.messageNum,encode.position,encode.velocity);
  //LOG_DBG("Encoder request callback!");
  if (currentEncode.messageNum + 1 != encode.messageNum) {
    LOG_INF("Dropped 1!");
  }
  currentEncode = encode;
  newMessage = true;
}

static struct openthread_state_changed_cb ot_state_chaged_cb = {
    .state_changed_cb = on_thread_state_changed};
#endif

const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

uint32_t scalar = 1U;
double per_c = 0;
float oldySteps = 0;
float yTargetSteps = 2000;
float ySpeed = 0;
float ierr = 0;
float dererr = 0;
int dir = 1;
double a = 0;
double accel = 0;
double delta_phi = delta_phi_start;
double placeholder = 0;
double flip = 0;
int notMovingCounter = 0;
uint32_t uptime = 0;
uint32_t oldtime = 0;
uint32_t collectTimeDone = 0;
float ySteps = 0;
bool firstTimeAchieve = true;
bool printNow = false;

float rxth = 0;
float angle = 0;
float kp = 1;
float ki = 15;
float kd = -0.15;
float oldrx = 0;
float slow = 1;
int angleCount = 0;
bool shouldStep = false;

// float kP = 10.0*pi/3000.0;
// float kD = -900;
// float kI = 10.0/1000.0;

float Poss[300];
int possi = 0;

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
	manualStop = true;
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
   printf("per_c = %f, ySteps = %f,p %f, i %f, d %f, dir %d, scalar %u, encpos = %i\n",per_c,angle,kp*ySteps, ki*ierr, kd*dererr,dir, scalar,currentEncode.position);
   mainloop = true;
    // struct encoderMessage example = {.position = 3000,
    //   .messageNum=0,.velocity=20};
    // coap_client_encoderSend(example);
  }
#endif
}

void my_work_handler(struct k_work *work)
{
	if(possi < 300){
		Poss[possi] = ySteps;
	}
	possi++;
	if(0*possi > 30){
		//yTargetSteps = 3000 - yTargetSteps;
		//firstTimeAchieve = true;
		printNow = true;
		possi = 0;
		printf("Changing to target %f\n",yTargetSteps);
		ierr = 0;
		ySpeed = 0;
		per_c = 0.1;
	}
}
K_WORK_DEFINE(my_work, my_work_handler);

void my_timer_handler(struct k_timer *timer_id)
{
	k_work_submit(&my_work);
}
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

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
  ICM20600_startup();
#endif /* ifdef IMU */
#ifdef ENCODER
  Setup_interrupt();
#endif /* ifdef ENCODER */

	uint32_t period = 4U * 1000U * 1000U ; //ms * to_us * to_ns
	per_c = period/1000000000.0;  //ns to s
	int olddir = dir;
	//float ySteps = 0;
	// float oldySteps = 0;
	// float yTargetSteps = 1500;
	// float ySpeed = 0;
	// float ierr = 0;
	//int ret;
	// int dir = 1;
	// double a = 0;
	// double accel = 0;
	// double delta_phi = delta_phi_start;
	// double placeholder = period;
	// double flip = 0;
	// int notMovingCounter = 0;
	// uint32_t uptime = k_uptime_ticks();
	// uint32_t oldtime = 0;
	printk("Uptime is %u\n",uptime);

	// ret = dk_buttons_init(on_button_changed);
	// if (ret) {
	// 	LOG_ERR("Cannot init buttons (error: %d)", ret);
	// 	goto end;
	// }
	// float buf[arraySize];
	// int bufindex = 0;
	// size_t timesFull = 0;
	// struct flash_pages_info pginf;

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

	gpio_pin_set(P0, mode2_pin, 0);
	gpio_pin_set(P0, mode1_pin, 0);
	gpio_pin_set(P0, mode0_pin, 0);

	// ret = flash_get_page_info_by_offs(flashmem, addrs, &pginf);
	
	// printf("getinfo ret %d and offset %u, size %u and i %u with max size = %u with doubles per page = %u\n",ret, pginf.start_offset,pginf.size,pginf.index,flash_get_page_count(flashmem), pginf.size/sizeof(per_c));
	// ret = flash_erase(flashmem, addrs, pginf.size*maxPages);
	// printf("erase ret %d\n",ret);

	printk("Control Wirelessly Correct\n");
	k_sleep(K_NSEC(4000U*1000U*1000U));

	while(!mainloop){
		k_sleep(K_NSEC(20000U));
	}

	k_timer_start(&my_timer, K_MSEC(0), K_MSEC(100));
	uptime = k_uptime_ticks();
	printk("Uptime is %u\n",uptime);

	while (1) {	

		if(manualStop){
			goto toend;
		}

		//delta_phi = delta_phi_start/scalar;

		if(shouldStep){

			gpio_pin_set(P0, step_pin, 1);

			k_sleep(K_NSEC(period/scalar/2U));

			gpio_pin_set(P0, step_pin, 0);

			k_sleep(K_NSEC(period/scalar/2U));
		}
		// if(uptime - oldtime > 0){
		// 	ySpeed = (ySteps - oldySteps)/(uptime-oldtime);
		// }
		// else{
		// 	ySpeed = ySteps - oldySteps;
		// }
		// oldtime = uptime;

		// ierr = ierr + (yTargetSteps - ySteps)/3000.0*(uptime-oldtime)/32786.0/0.1;

		rxth = (float) currentEncode.position/10000.0 - currentOffset;

		ySteps = rxth;
		

		if(newMessage){
		uptime = k_uptime_ticks();
		if(uptime - oldtime > 0){
			dererr = (rxth - oldrx)/(uptime-oldtime)*32786;
		}
		else{
			dererr = (rxth - oldrx)/0.1;
		}
		ierr = ierr*0.85 + rxth*(uptime-oldtime)/32786.0;

		if(ierr > 20)
			ierr = 20;
		if(ierr < -20)
			ierr = -20;
		if(dererr > 80)
			dererr = 80;
		if(dererr < -80)
			dererr = -80;

		oldtime = uptime;
		oldrx = rxth;
		newMessage = false;

		if(fabs(angle) < 9.9){//fabs(rxth) < 1 ){
			angleCount++;
			if(angleCount > 100){
				goto toend;
			}
		}
		else{
			angleCount = 0;
		}
		}

		angle = kp*rxth + ki*ierr + kd*dererr;


		olddir = dir;
		if(fabs(angle) > 5){
			shouldStep = true;
		}
		else{
			shouldStep = false;
		}

		if(rxth > 0){
			dir = 1;
		}
		if(rxth < 0){
			dir = -1;
		}

		if(fabs(angle) < 12)
			slow = -1;
		else
			slow = 1;


		per_c = delta_phi_start/(delta_phi_start/per_c + slow*fabs(angle)/10.0);
		if(olddir != dir){
			per_c = 2*per_c;
			// for(int i=0; i<possi && i<100; i++){
			// 	printf("%f\n",Poss[i]);
			// }
			// possi = 0;
		}

		placeholder = per_c*1000000000;
		period = placeholder;

		if(dir == 1)
			gpio_pin_set(P0, dir_pin, 0); //Away from motor

		if(dir == -1)
			gpio_pin_set(P0, dir_pin, 1); //Towards motor

	//printf("per_c = %f, ySteps = %f, accel = %f, ySpeed = %f, dir = %d\n",per_c,ySteps,accel,ySpeed,dir);
	
	// if(bufindex < arraySize){
	// 	buf[bufindex] = ySteps;
	// 	bufindex++;
	// }
	// else
	// {
	// 	if(timesFull*sizeof(buf)<maxPages*pginf.size){
	// 		ret = flash_write(flashmem, addrs+sizeof(buf)*timesFull, buf, sizeof(buf));
	// 		if(ret < 0)
	// 			printf("write ret %d\n",ret);
	// 		timesFull++;
	// 		bufindex = 0;
	// 		buf[bufindex] = ySteps;
	// 		bufindex++;
	// 	}
	// }
	 
    // if( (delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel < 0)
    // {
	// 	flip = 1;
	// 	per_c = sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) - delta_phi/accel) - delta_phi/(accel*per_c*2);
	// 	accel = -accel;
	// 	dir = -dir;
    // }
    // else {
    //     if(accel > 0.01) 
    //     {
    // 		    per_c = sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel) - delta_phi/(accel*per_c*2);
    // 	}
    // 	else
    //         if(accel < -0.01)
    // 		{
    // 		        per_c = -sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel) - delta_phi/(accel*per_c*2);
    // 		}
	// }

	// 	placeholder = per_c*1000000000;
	// 	period = placeholder;

		if(period*scalar < MIN_PER){
			period = MIN_PER;
			per_c = period/1000000000.0;
		}
		if(period*scalar > MAX_PER){
			period = MAX_PER;
			per_c = period/1000000000.0;
			//printf("Should have theoretically stopped, per_c = %f\n",per_c);
		}

		if(period/scalar > GEAR_PER+GEAR_GUARD && scalar < 20U){ //if going slower than a predefined speed
			scalar = scalar*2U;  //gear down
		}
		else{
			if(period/scalar*2 < GEAR_PER-GEAR_GUARD) //if going faster than said speed
				scalar = scalar/2U;
				if(scalar < 1U) scalar = 1U;
		}
		switch(scalar){
			case 1U:
				gpio_pin_set(P0, mode2_pin, 0);
				gpio_pin_set(P0, mode1_pin, 0);
				gpio_pin_set(P0, mode0_pin, 0);
				break;
			case 2U:
				gpio_pin_set(P0, mode2_pin, 0);
				gpio_pin_set(P0, mode1_pin, 0);
				gpio_pin_set(P0, mode0_pin, 1);
				break;
			case 4U:
				gpio_pin_set(P0, mode2_pin, 0);
				gpio_pin_set(P0, mode1_pin, 1);
				gpio_pin_set(P0, mode0_pin, 0);
				break;
			case 8U:
				gpio_pin_set(P0, mode2_pin, 0);
				gpio_pin_set(P0, mode1_pin, 1);
				gpio_pin_set(P0, mode0_pin, 1);
				break;
			case 16U:
				gpio_pin_set(P0, mode2_pin, 1);
				gpio_pin_set(P0, mode1_pin, 0);
				gpio_pin_set(P0, mode0_pin, 0);
				break;
			case 32U:
				gpio_pin_set(P0, mode2_pin, 1);
				gpio_pin_set(P0, mode1_pin, 0);
				gpio_pin_set(P0, mode0_pin, 1);
				break;
			default:
				//printk("Scalar is wrong\n");
			break;
		}
	}
	//}

  toend:
  	printf("Target Reached in %u\n", uptime);

	for(int i=0; i<300; i++){
		printf("%f\n",Poss[i]);
	}
	printf("Might require up to %d, rn with every 0.1\n",possi); 

  end: return;
}
