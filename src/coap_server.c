/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

// Toggles to disable or enable functionality.
#include "coap_server_client_interface.h"
#define CLIENT
#define SERVER
//#define IMU
#define ENCODER

#include <dk_buttons_and_leds.h>
#include <openthread/thread.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/openthread.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h> //uart
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

#include <math.h>
/* 1000 nsec = 1 usec */
//#define MIN_PER 1000000 //1 ms
#define GEAR_PER 10000000//2000000//
#define GEAR_GUARD 400000
#define MAX_PER 10000000000
#define full_length_in_steps 3000
#define pi 3.14159265
#define step_pin 29
#define dir_pin 30
#define mode2_pin 5
#define mode1_pin 1
#define mode0_pin 0
#define delta_phi_start 0.01570796326//pi/2/100;
#define MAXENCODER 30000.0
#define maxRecord 300

#define RECEIVE_TIMEOUT 100

#define invPolarity 1
#define readPolarity -1

bool newMessage = false;
bool mainloop = false;
bool inPrinting = false;
bool allBoardsTarget[5] = {false, false, false, false, false}; 


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

static void on_generic_request(char *msg) {
  // Something to deal with message would normally go here. However, message is
  // just character string so it doesn't matter.
  LOG_INF("Generic Request event execution!");
}

static void on_float_request(double num) { LOG_INF("Number is: %f", num); }
static void on_percentage_request(struct percentageStruct percent) {
  ARG_UNUSED(percent);
  LOG_INF("Percentage request callback");
}
struct encoderMessage example = {
      .payload = 3000, .messageNum = 0, .command = 20};

const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

//UART device obtained
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

//Define transmission buffer (changed from 8 to 32)
static uint8_t tx_buf[] =   {"nRF Connect SDK Fundamentals Course\n\r"
                             "Press 1-3 on your keyboard to toggle LEDS 1-3 on your development kit\n\r"}; //Transmission message displayed here
//Define receive buffer and initialise it
static uint8_t rx_buf[10] = {0}; //Buffer size set to 10

int posindex = 0;
int yTarget[] = {0,1500,2500};
int xTarget[] = {0,2500,0};

uint32_t period = 4U * 1000U * 1000U ; //ms * to_us * to_ns
uint32_t MIN_PER = 2500000;
double per_c = 0;
float oldySteps = 0;
float yTargetSteps = 1500;
uint8_t scalar = 1U;
int TEMPORARY = 0;
float ySpeed = 0;
float error = 0;
float olderror = 0;
float ierr = 0;
int dir = 1;
double a = 0;
double accel = 0;
double delta_phi = delta_phi_start;
double placeholder = 0;
double flip = 0;
int notMovingCounter = 0;
uint32_t uptime = 0;
uint32_t oldtime = 0;
float ySteps = 0;
bool firstTimeAchieve = true;
int step_semaphore = -1;

float kP = 0.003875;//2.0*pi/3000.0;
float kD = 270.0;//70;
float kI = 0.072;//24.0/1000.0;

// float kP = 20.0*pi/3000.0;
// float kD = -1050;
// float kI = 1.0/1000.0;

int i = 0;

//UART application callback function
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
		//Only defining the receiving modes
	case UART_RX_RDY:
		if((evt->data.rx.len) == 1){ //If something entered

		//Defining responses:
		if(evt->data.rx.buf[evt->data.rx.offset] == 'w'){
			LOG_DBG("Upwards \n");
			ierr = 0;
			yTargetSteps = 2500;
			// struct encoderMessage example = {.payload = yTargetSteps,
			// .messageNum=0,.command=70};
			// coap_client_encoderSend(1,example);
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 70,
			  .datum2 = yTargetSteps,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//Up code uart
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'a'){
			LOG_DBG("Left \n");
			struct encoderMessage example = {.payload = 3000,
			.messageNum=0,.command=69};
			coap_client_encoderSend(1,example);
			mainloop = true;
			//Left code uart
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 's'){
			LOG_DBG("Downwards \n");
			ierr = 0;
			yTargetSteps = 500;
			// struct encoderMessage example = {.payload = yTargetSteps,
			// .messageNum=0,.command=70};
			// coap_client_encoderSend(1,example);
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 70,
			  .datum2 = yTargetSteps,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//Down code uart					
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'd'){
			LOG_DBG("Right \n");
			//Right code uart
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '='){
			kD = kD + 5;
			LOG_DBG("Kd+ is now %f \n",kD);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '-'){
			kD = kD - 5;
			LOG_DBG("Kd- is now %f \n",kD);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '0'){
			kP = kP + 0.1*pi/3000;
			LOG_DBG("Kp+ is now %f \n",kP);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '9'){
			kP = kP - 0.1*pi/3000;
			LOG_DBG("Kp- is now %f \n",kP);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '8'){
			kI = kI + 2.0/1000.0;
			LOG_DBG("KI+ is now %f \n",kI);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '7'){
			kI = kI - 2.0/1000.0;
			LOG_DBG("KI- is now %f \n",kI);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 't'){
			LOG_DBG("Target 0 \n");
			ierr = 0;
			yTargetSteps = 0;
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'b'){
			LOG_DBG("Target middle \n");
			ierr = 0;
			yTargetSteps = 1500;
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'x'){
			mainloop = false;
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '['){
			MIN_PER -= 10000;
			LOG_DBG("MIN_PER- is now %u \n",MIN_PER);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == ']'){
			MIN_PER += 10000;
			LOG_DBG("MIN_PER+ is now %u \n",MIN_PER);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'r'){
			resetPosition(0);
			ySteps = 0;
			LOG_DBG("Reset position to 0");
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == ' '){
			LOG_DBG("per_c = %f, ySteps = %f, a = %f, ySpeed = %f, dir = %d, scalar = %u, target = %f, ierr = %f, notMoving = %d, temp = %d, semaphore = %d, uptime = %u, oldtime = %u\n",per_c,ySteps,a,ySpeed,dir,scalar,yTargetSteps,ierr, notMovingCounter, TEMPORARY, step_semaphore,uptime, oldtime);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'p'){
			LOG_DBG("per_c = %f, a = %f, P = %f, I = %f, D = %f\n",per_c,a,kP*(yTargetSteps-ySteps),kI*ierr,kD*ySpeed);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '1'){
			LOG_DBG("All start\n");
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 69,
			  .datum2 = yTargetSteps,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			mainloop = true;
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '2'){
			coap_client_send_provisioning_request();
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '3'){
			serverScroll();
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '4'){
			k_work_submit(&provisioning_work);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '5'){
			
		}
		}

        break;
	case UART_RX_DISABLED: //Constant receiving
		uart_rx_enable(dev ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
		break;
		
	default:
		break;
	}
}

struct encoderMessage currentEncode = {};
static void on_encoder_request(struct encoderMessage encode) {
  //LOG_DBG("Message Number: %i\nPosition:%i,Velocity:%i",encode.messageNum,encode.position,encode.velocity);
  //LOG_DBG("Encoder request callback!");
  if (currentEncode.messageNum + 1 != encode.messageNum) {
    LOG_INF("Dropped %d!", encode.messageNum - currentEncode.messageNum + 1);
  }
  currentEncode = encode;
  if(currentEncode.command == 69){
	mainloop = true;
  }
  if(currentEncode.command == 70){
	yTargetSteps = currentEncode.payload;
  }
  newMessage = true;
}

static void on_cmd_request(struct commandMsg cmd) {
	if(cmd.datum1 == NODE){
		yTargetSteps = cmd.datum2;
	}
	switch(cmd.datum1){
		case 69:
			mainloop = true;
			break;
		case 70:
			yTargetSteps = cmd.datum2;
			ierr = 0;
			break;
		case 100:
			if(NODE == CCU){
				allBoardsTarget[cmd.datum3] = true;
				LOG_DBG("Board Number %d has arrived",cmd.datum3);
			}
			if(allBoardsTarget[MIDX] && allBoardsTarget[LEFTY] && allBoardsTarget[RIGHTY]){
				for(i = 0; i<5; i++){
					allBoardsTarget[i] = false;
				}
				posindex = (posindex+1)%3;
				struct commandMsg example = {
				.cmd = 0,
				.datum1 = 2,
				.datum2 = yTarget[posindex],
				.datum3 = 0
				};
				coap_client_cmdSend(-1,example);
				example.datum1 = 1;
				example.datum2 = xTarget[posindex];
				coap_client_cmdSend(MIDX,example);
				LOG_DBG("Moving to next target");
			}
		default:
		break;
}
}

static struct openthread_state_changed_cb ot_state_chaged_cb = {
    .state_changed_cb = on_thread_state_changed};
#endif

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
    //serverScroll();
	 struct encoderMessage example = {.payload = 3000,
      .messageNum=0,.command=69};
    coap_client_encoderSend(1,example);
	mainloop = true;
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
   printf("per_c = %f, ySteps = %f, a = %f, ySpeed = %f, dir = %d, scalar = %u, target = %f, ierr = %f, notMoving = %d, temp = %d, semaphore = %d\n",per_c,ySteps,a,ySpeed,dir,scalar,yTargetSteps,ierr, notMovingCounter, TEMPORARY, step_semaphore);
   mainloop = true;
  }
#endif
}

void step_work_handler(struct k_work *work)
{
	if(step_semaphore < 2*scalar){
		TEMPORARY++;
		//printk("I am stepping for %d\n", TEMPORARY);
		if(step_semaphore%2 == 0){
			gpio_pin_set(P0, step_pin, 1);}
		else{
			gpio_pin_set(P0, step_pin, 0);}
		
		step_semaphore++;
	}
	// for(i = 0; i<scalar; i++){
	// 	gpio_pin_set(P0, step_pin, 1);

	// 	k_sleep(K_NSEC(period/scalar/2U));

	// 	gpio_pin_set(P0, step_pin, 0);

	// 	k_sleep(K_NSEC(period/scalar/2U));
	// }
}
K_WORK_DEFINE(step_work, step_work_handler);

void step_timer_handler(struct k_timer *timer_id)
{
	k_work_submit(&step_work);
}
K_TIMER_DEFINE(step_timer, step_timer_handler, NULL);


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
                     &on_generic_request, &on_float_request,
                     &on_percentage_request, &on_encoder_request, &on_cmd_request);
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

	// uint32_t period = 4U * 1000U * 1000U ; //ms * to_us * to_ns
	per_c = period/1000000000.0;  //ns to s
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

	//UART callback function registered
	ret = uart_callback_set(uart, uart_cb, NULL);
		if (ret) {
			return;
		}
	//Data sent over UART
	ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US);
	if (ret) {
		return;
	}	
	//uart_rx_enable() call to start receiving
	ret = uart_rx_enable(uart ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
	if (ret) {
		return;
	}

	printk("Uart is setup \n");
	gpio_pin_set(P0, mode2_pin, 0);
	gpio_pin_set(P0, mode1_pin, 0);
	gpio_pin_set(P0, mode0_pin, 0);

	printk("Control Wirelessly Correct\n");
	k_sleep(K_NSEC(2000U*1000U*1000U));


	restart:
	while(!mainloop){
		k_sleep(K_NSEC(2000U));
	}

	uptime = k_uptime_ticks();

	while (1) {		
		//delta_phi = delta_phi_start/scalar;
		if(mainloop == false){
			goto restart;
		}
		//Uncomment for utilising Antiblock Measures
		if(notMovingCounter> 100 && ((ySteps + dir*5<yTargetSteps-10) || (ySteps + dir*5>yTargetSteps+10))){
			step_semaphore = 64;
			for(i = 0; i<5; i++){
				gpio_pin_set(P0, step_pin, 1);

				k_sleep(K_MSEC(30));

				gpio_pin_set(P0, step_pin, 0);

				k_sleep(K_MSEC(30));

				//ySteps = ySteps + 1.0*dir/scalar;
			}
			printf("Antiblock measures\n");
			period = period*10;
			per_c = per_c*10;
			notMovingCounter = 0;
		}

		not_done_stepping:
		
		if(step_semaphore < 2*scalar && step_semaphore != -1){
			k_sleep(K_NSEC(5000));
			goto not_done_stepping;
		}
		step_semaphore = 0;
		k_timer_start(&step_timer,K_NSEC(0),K_NSEC(period/scalar/2U));
		if(ySteps - 100 < yTargetSteps && yTargetSteps < ySteps + 100 && per_c > 0.5)
		{
			LOG_DBG("Target Reached \n");
			struct commandMsg cmd = {.datum1 = 100, .datum3 = NODE};
			coap_client_cmdSend(CCU, cmd);
			if(per_c > 0.8) k_sleep(K_MSEC(300));
		}

		//Was moving this to a timer function to not clutter main and to use this time to send messages for synchronisation.
		// for(i = 0; i<scalar; i++){
		// 	gpio_pin_set(P0, step_pin, 1);

		// 	k_sleep(K_NSEC(period/scalar/2U));

		// 	gpio_pin_set(P0, step_pin, 0);

		// 	k_sleep(K_NSEC(period/scalar/2U));
		// }
		//ySteps = ySteps + 1.0/scalar*dir;
		oldySteps = ySteps;
		olderror = error;
    	ySteps = 3000.0*getPosition()/MAXENCODER*readPolarity;//*currentEncode.position/MAXENCODER;//
		error = yTargetSteps - ySteps;

		if( oldySteps == ySteps ){
			notMovingCounter++;
		}
		else{
			notMovingCounter = 0;
		}
		
		oldtime = uptime;
		//all olds have been updated
		uptime = k_uptime_ticks();
		if(uptime - oldtime > 0){
			ySpeed = (error - olderror)/(uptime-oldtime);
		}
		else{
			ySpeed = error - olderror;
		}

		ierr = ierr + error/3000.0*(uptime-oldtime)/32786.0*5;
		if(kI*ierr > 20){
			ierr = 20.0/kI;
		}
		if(kI*ierr < -20){
			ierr = -20.0/kI;
		}
	

		// if(flip!=0){
		// 	ySpeed = ySpeed/20.0;//ySpeed/10.0;
		// 	flip = 0;
		// }

		//yTargetstepsex = 1500 , yStepsex = 500; ySteps2 = 1000
		//then error = 1000, error2 = 500 ySpeed = -500  ierr = 200
		//a = 2*1000 + 200*-500 + 200/1000 = +Number slowed down by D

		//yTargetstepsex = 1500 , yStepsex = 2000; ySteps2 = 2500
		//then error = -500, error2 = -1000 ySpeed = -500  ierr = -200
		//a = 2*-500 + 200*-500 + -200/1000

		//Target = 500, ySteps = 2000 ysteps
		//then error = -1500, error

		a = kP*error + kD*ySpeed + kI*ierr;
		if(a > 20)
			a = 20;
		if(a < -20)
		    a = -20;
 
		accel = a*dir;

		if(dir == 1*invPolarity)
			gpio_pin_set(P0, dir_pin, 0); //Away from motor

		if(dir == -1*invPolarity)
			gpio_pin_set(P0, dir_pin, 1); //Towards motor

    //printf("per_c = %f, ySteps = %f, a = %f, ySpeed = %f, dir = %d, scalar = %u, target = %f, ierr = %f\n",per_c,ySteps,a,ySpeed,dir,scalar,yTargetSteps,ierr);
	 
    if( (delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel < 0)
    {
		flip = 1;
		per_c = sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) - delta_phi/accel) - delta_phi/(accel*per_c*2);
		accel = -accel;
		dir = -dir;
    }
    else {
        if(accel > 0.01) 
        {
    		    per_c = sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel) - delta_phi/(accel*per_c*2);
    	}
    	else
            if(accel < -0.01)
    		{
    		        per_c = -sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel) - delta_phi/(accel*per_c*2);
    		}
	}

		placeholder = per_c*1000000000;
		period = placeholder;

		// if(period*scalar < MIN_PER){
		// 	period = MIN_PER;
		// 	per_c = period/1000000000.0;
		// }
		// if(period*scalar > MAX_PER){
		// 	period = MAX_PER;
		// 	per_c = period/1000000000.0;
		// 	//printf("Should have theoretically stopped, per_c = %f\n",per_c);
		// }

		// if(period/scalar > GEAR_PER+GEAR_GUARD && scalar < 20U){ //if going slower than a predefined speed
		// 	scalar = scalar*2U;  //gear down
		// }
		// else{
		// 	if(period/scalar*2 < GEAR_PER-GEAR_GUARD) //if going faster than said speed
		// 		scalar = scalar/2U;
		// 		if(scalar < 1U) scalar = 1U;
		// }
		// switch(scalar){
		// 	case 1U:
		// 		gpio_pin_set(P0, mode2_pin, 0);
		// 		gpio_pin_set(P0, mode1_pin, 0);
		// 		gpio_pin_set(P0, mode0_pin, 0);
		// 		break;
		// 	case 2U:
		// 		gpio_pin_set(P0, mode2_pin, 0);
		// 		gpio_pin_set(P0, mode1_pin, 0);
		// 		gpio_pin_set(P0, mode0_pin, 1);
		// 		break;
		// 	case 4U:
		// 		gpio_pin_set(P0, mode2_pin, 0);
		// 		gpio_pin_set(P0, mode1_pin, 1);
		// 		gpio_pin_set(P0, mode0_pin, 0);
		// 		break;
		// 	case 8U:
		// 		gpio_pin_set(P0, mode2_pin, 0);
		// 		gpio_pin_set(P0, mode1_pin, 1);
		// 		gpio_pin_set(P0, mode0_pin, 1);
		// 		break;
		// 	case 16U:
		// 		gpio_pin_set(P0, mode2_pin, 1);
		// 		gpio_pin_set(P0, mode1_pin, 0);
		// 		gpio_pin_set(P0, mode0_pin, 0);
		// 		break;
		// 	case 32U:
		// 		gpio_pin_set(P0, mode2_pin, 1);
		// 		gpio_pin_set(P0, mode1_pin, 0);
		// 		gpio_pin_set(P0, mode0_pin, 1);
		// 		break;
		// 	default:
		// 		//printk("Scalar is wrong\n");
		// 	break;
		// }
	}

  end: return;
}
