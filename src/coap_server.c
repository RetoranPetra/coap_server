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
#define GEAR_PER 10000000//2000000// //Used only for using Microstepping Mode. Not necessary if microstepping not used. It determines at which period it should increase the microstepping prescaler (take smaller steps and speed up the steps for more linearity)
#define GEAR_GUARD 400000 //prevents the motors from constantly changing between 'gears', microstepping modes
#define MAX_PER 10000000000 //slowest steps possible to be taken, compiler will transforming into long max, that is 1.41 seconds
#define full_length_in_steps 3000 //1 meter in motors steps is 3000, our gantry is 1 m x 1 m x 1 m
#define pi 3.14159265 //pi
#define step_pin 29 //step input to the motor driver board
#define dir_pin 30 //direction pin for the driver
#define mode2_pin 5 //mode pins
#define mode1_pin 1
#define mode0_pin 0
#define delta_phi_start 0.01570796326//pi/2/100; //used for equations
#define MAXENCODER 30000.0 //1 meter in encoder steps, 30000 encoder steps = 3000 motor steps
#define maxRecord 300 //unused, used to be for using the flash memory

#define RECEIVE_TIMEOUT 500 //used for reading UART inputs, not needed unless uart is used for debugging, currently disabled

#define invPolarity 1 //Depends on how the motors are connected to the motor driver. For node LeftY it should be -1, for the other +1
#define readPolarity -1 //Depends on how the encoders are wired to the nRF, -1 for all boards in our configuration

bool newMessage = false; //unused
bool mainloop = false; //Main does not start until this becomes true
bool shouldMove = false; //Used for manual control
bool manualControl = false; //used to remember whether we are operating in manual mode or in automatic mode
bool forcedStep = false; //makes the manual control take a step if a message is received
bool shouldWait = false; //variable to remember that we have to wait after we send a command


LOG_MODULE_REGISTER(coap_server, CONFIG_COAP_SERVER_LOG_LEVEL); 
////////////////////////
//provisioning is needed to connect one board with another, however, messages can be broadcasted and received without prior pairing
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
/////////////////////////// END OF PROVISIONING STUFF
static void on_generic_request(char *msg) { //unused
  // Something to deal with message would normally go here. However, message is
  // just character string so it doesn't matter.
  LOG_INF("Generic Request event execution!");
}

static void on_float_request(double num) { LOG_INF("Number is: %f", num); } //unused, but can be useful
static void on_percentage_request(struct percentageStruct percent) { //unused, but can be useful
  ARG_UNUSED(percent);
  LOG_INF("Percentage request callback");
}
struct encoderMessage example = { //One way of sending sensor information
      .payload = 3000, .messageNum = 0, .command = 20};

const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0)); //Port for changing pins

//UART device obtained, unused on the deployed nodes
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

uint32_t period = 4U * 1000U * 1000U ; //ms * to_us * to_ns , 4 ms initial stepping period for being safe during demo
uint32_t MIN_PER = 2500000; //minimum period, 250 us for demo for safety
double per_c = 0; //double representation of the period, for doing operations
float oldySteps = 0; //Position at time (k-1)T
float yTargetSteps = 1500; //Target
uint8_t scalar = 1U; //Microstepping mode, can be full step 1, half step 2, .... 1/32th of a step then 32
int TEMPORARY = 0; //used for debugging
float ySpeed = 0; //measured speed
float error = 0; //target - position
float olderror = 0; //error at (k-1)T
float ierr = 0; //integral error
int dir = 1; //direction of movement, 1 or -1
double a = 0; //acceleration with respect to the positive direction of the axis
double accel = 0; //acceleration with respect to our current direction of movement
double delta_phi = delta_phi_start; //used for calculations
double placeholder = 0; //placeholder when converting between variable types, casting seemed to sometimes not function as expected
double flip = 0; //unused in final version
int notMovingCounter = 0; //checks if not the gantry is stuck in place (because of a mechanical imperfection)
uint32_t uptime = 0; //time since main is running
uint32_t oldtime = 0; //time at (k-1)T
float ySteps = 0; //current measured position
bool firstTimeAchieve = true; //first time to arrive at the target, do certain actions
int step_semaphore = -1; //count how many steps have been taken or if they should no longer be taken asynchronously

float kP = 0.003875;//PID variables
float kD = 270.0;
float kI = 0.072;

int i = 0; //counter for 'for loops'

struct encoderMessage currentEncode = {}; //received encoder message
static void on_encoder_request(struct encoderMessage encode) { //Action to be taken when a message of this type is received
  //LOG_DBG("Message Number: %i\nPosition:%i,Velocity:%i",encode.messageNum,encode.position,encode.velocity);
  //LOG_DBG("Encoder request callback!");
  if (currentEncode.messageNum + 1 != encode.messageNum) { //see how many messages have been dropped
    LOG_INF("Dropped %d!", encode.messageNum - currentEncode.messageNum + 1);
  }
  currentEncode = encode; //update the currentEncode variable
  if(currentEncode.command == 69){ //if the message carries certain commands, execute them
	mainloop = true; //command 69 starts the main loop of a node
  }
  if(currentEncode.command == 70){  //command 70 changes the target of a node
	yTargetSteps = currentEncode.payload;
  }
  newMessage = true;
}

static void on_cmd_request(struct commandMsg cmd) { //on message of type command received (a different type from encoder message)
	LOG_DBG("Received message with %d and %d and %d",cmd.datum1,cmd.datum2,cmd.datum3); //for debugging print what was received
	switch(cmd.datum1){ //datum1 decides what should be done with the contents of the message. Caleb named the fields
		case 1:

			break;
		case 2:
			if(cmd.datum3 == NODE){ //if datum3 signals to our node number, we should stop. Case 2 is for telling a node to wait
				shouldWait = true;
			}
			break;
		case 66:
			mainloop = false; //case 66 is for telling a node to stop executing main
			break;
		case 67:
			resetPosition(0); //case 67 is for recalibrating the position, our current position will become the '0'
			break;
		case 68://case 68 is requesting a node to print its information, for debugging
			LOG_DBG("per_c = %f, ySteps = %f, a = %f, ySpeed = %f, dir = %d, scalar = %u, target = %f, ierr = %f, notMoving = %d, temp = %d, semaphore = %d, uptime = %u, oldtime = %u\n",per_c,ySteps,a,ySpeed,dir,scalar,yTargetSteps,ierr, notMovingCounter, TEMPORARY, step_semaphore,uptime, oldtime);
			break;
		case 69: //case 69 is for starting a node
			mainloop = true;
			break;
		case 70: //case 70 is for changing the target of the node to datum2
			yTargetSteps = cmd.datum2;
			ierr = 0;
			break;
		case 71: //case 71 is for moving to a predefined positon
			if(NODE == MIDX)
				yTargetSteps = 800;
			if(NODE == RIGHTY)
				yTargetSteps = 800;
			if(NODE == LEFTY)
				yTargetSteps = 800;

			ierr = 0;
			error = 0;
			break;
		case 72: //case 72 is for moving to a predefined positon
			if(NODE == MIDX)
				yTargetSteps = 1600;
			if(NODE == RIGHTY)
				yTargetSteps = 800;
			if(NODE == LEFTY)
				yTargetSteps = 800;

			ierr = 0;
			error = 0;
			break;
		case 73: //case 73 is for moving to a predefined positon
			if(NODE == MIDX)
				yTargetSteps = 1200;
			if(NODE == RIGHTY)
				yTargetSteps = 1600;
			if(NODE == LEFTY)
				yTargetSteps = 1600;

			ierr = 0;
			error = 0;
			break;
		case 74: //case 74 is for moving to a predefined positon
			if(NODE == MIDX)
				yTargetSteps = 10;
			if(NODE == RIGHTY)
				yTargetSteps = 10;
			if(NODE == LEFTY)
				yTargetSteps = 10;

			ierr = 0;
			error = 0;
			break;
		case 75: //case 75 is for moving to a predefined positon
			if(NODE == MIDX)
				yTargetSteps = 1000;
			if(NODE == RIGHTY)
				yTargetSteps = 1000;
			if(NODE == LEFTY)
				yTargetSteps = 1000;

			ierr = 0;
			error = 0;
			break;
		case 76: //case 76 is for moving to a predefined positon
			if(NODE == MIDX)
				yTargetSteps = 1600;
			if(NODE == RIGHTY)
				yTargetSteps = 1000;
			if(NODE == LEFTY)
				yTargetSteps = 1000;

			ierr = 0;
			error = 0;
			break;
		case 77: //case 77 is for moving to a predefined positon
			if(NODE == MIDX)
				yTargetSteps = 1000;
			if(NODE == RIGHTY)
				yTargetSteps = 1600;
			if(NODE == LEFTY)
				yTargetSteps = 1600;

			ierr = 0;
			error = 0;
			break;
		case 78: //case 78 is for moving to a predefined positon
			if(NODE == MIDX)
				yTargetSteps = 1600;
			if(NODE == RIGHTY)
				yTargetSteps = 1600;
			if(NODE == LEFTY)
				yTargetSteps = 1600;

			ierr = 0;
			error = 0;
			break;
		case 101: //case 101 is for moving in the X direction in manual mode
			if(NODE == MIDX){
				shouldMove = true;
				forcedStep = true;
				dir = 1;
			}
			if(NODE == RIGHTY){
				shouldMove = false;
				dir = 1;
			}
			if(NODE == LEFTY){
				shouldMove = false;
				dir = 1;
			}
			break;
		case 102: //case 102 is for moving in the -X direction in manual mode
			if(NODE == MIDX){
				shouldMove = true;
				forcedStep = true;
				dir = -1;
			}
			if(NODE == RIGHTY){
				shouldMove = false;
				dir = 1;
			}
			if(NODE == LEFTY){
				shouldMove = false;
				dir = 1;
			}
			break;
		case 103: //case 103 is for moving in the Y direction in manual mode
			if(NODE == MIDX){
				shouldMove = false;
				dir = 1;
			}
			if(NODE == RIGHTY){
				shouldMove = true;
				forcedStep = true;
				dir = 1;
			}
			if(NODE == LEFTY){
				shouldMove = true;
				forcedStep = true;
				dir = 1;
			}
			break;
		case 104: //case 104 is for moving in the -Y direction in manual mode
			if(NODE == MIDX){
				shouldMove = false;
				dir = 1;
			}
			if(NODE == RIGHTY){
				shouldMove = true;
				forcedStep = true;
				dir = -1;
			}
			if(NODE == LEFTY){
				shouldMove = true;
				forcedStep = true;
				dir = -1;
			}
			break;
		case 105: //case 105 is for standing still in manual mode
			if(NODE == MIDX){
				shouldMove = false;
				dir = 1;
			}
			if(NODE == RIGHTY){
				shouldMove = false;
				dir = 1;
			}
			if(NODE == LEFTY){
				shouldMove = false;
				dir = 1;
			}
			break;
		case 106: //case 106 is for changing to manual or automatic control
			shouldMove = false;
			dir = 1;
			manualControl = cmd.datum3;
			LOG_DBG("Manual control is now = %d",manualControl);
			break;
		default:
		break;
}
}
/// other subroutines for openthread
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
/// end of subroutines for debugging provisioning

void step_work_handler(struct k_work *work) //thread to take a step
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

void step_timer_handler(struct k_timer *timer_id) //set a timer to step at a every interval
{
	k_work_submit(&step_work);
}
K_TIMER_DEFINE(step_timer, step_timer_handler, NULL);

///////////////// MAIN
void main(void)
{
    // Need to sleep at start for logs to display correctly.
  k_msleep(1000);
#ifdef SERVER
  int ret;

  LOG_INF("Start CoAP-server sample with Node %d",NODE);

  k_timer_init(&led_timer, on_led_timer_expiry, on_led_timer_stop);
  k_timer_init(&provisioning_timer, on_provisioning_timer_expiry, NULL);

  k_work_init(&provisioning_work, activate_provisioning);
//start provisioning and all coap functions
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
  Setup_interrupt(); //start reading data from the encoder
#endif /* ifdef ENCODER */
	
	//in order to print all variables through interrupts, all variables were made GLOBAL

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
	LOG_DBG("fabs(-7) returns %f\n",fabs(-7)); //check that math.h is working as expected


	//check that all peripherals work as expected
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

	LOG_DBG("Uart is setup \n");
	gpio_pin_set(P0, mode2_pin, 0);
	gpio_pin_set(P0, mode1_pin, 0);
	gpio_pin_set(P0, mode0_pin, 0);

	//start the actual program
	LOG_DBG("Control Wirelessly Correct\n");
	k_sleep(K_NSEC(2000U*1000U*1000U));

	
	restart:
	while(!mainloop){ //Until mainloop is not told to start, it is stuck waiting here
		k_sleep(K_NSEC(2000U));
	}

	uptime = k_uptime_ticks(); //read the current time

	while (1) {		
		//delta_phi = delta_phi_start/scalar;
		if(mainloop == false){ //if node is told to not do main anymore, go to restart above
			goto restart;
		}
		if(manualControl){ //if it is switching to manual control
			k_timer_stop(&step_timer); //stop stepping
			step_semaphore = -1; 
			ierr = 0;
			LOG_DBG("Switch to manual mode");
			goto manCon; //go to the next while loop where manual control happens
		}
		//Uncomment for utilising Antiblock Measures
		//if motor is stuck for too long on a stone. Force 5 steps at a slower rate and slow down
		if(notMovingCounter> 100 && ((ySteps + dir*5<yTargetSteps-10) || (ySteps + dir*5>yTargetSteps+10))){
			step_semaphore = 64;
			for(i = 0; i<5; i++){
				gpio_pin_set(P0, step_pin, 1);

				k_sleep(K_MSEC(30));

				gpio_pin_set(P0, step_pin, 0);

				k_sleep(K_MSEC(30));

				//ySteps = ySteps + 1.0*dir/scalar;
			}
			printf("Antiblock measures");
			period = period*10;
			per_c = per_c*10;
			notMovingCounter = 0;
		}

		//used to be used for synchronising RightY and LeftY nodes
		// if( ((NODE == RIGHTY) || (NODE == LEFTY)) && fabs(ySpeed) < 0.1){
		// 	struct commandMsg cmd = {.cmd = (uint8_t) dir, .datum1 = 1, .datum2 = ySteps ,.datum3 = NODE};
		// 	coap_client_cmdSend(-1, cmd);
		// }
		not_done_stepping: 
		//come back here if the steps have not been taken
		if(step_semaphore < 2*scalar && step_semaphore != -1){ //check if the step pin has gone high and low at least twice (that is a step)
			k_sleep(K_NSEC(5000));
			goto not_done_stepping;//if steps are still being taken, wait
		}
		if(shouldWait){ //if told to wait by the CCU, wait
			LOG_DBG("Waiting For Sync");
			k_sleep(K_NSEC(20*period/scalar/2U));
			shouldWait = false;
		}
		step_semaphore = 0; //for taking steps
		k_timer_start(&step_timer,K_NSEC(0),K_NSEC(period/scalar/2U)); //take a step at period "period" asynchronously, if microstepping mode is being used, take multiple steps depeding on the prescaler
		if(ySteps - 150 < yTargetSteps && yTargetSteps < ySteps + 150 && per_c > 0.4) //if we are close to the target and moving slowly, transmit the 'target reached' message to request the next point to move to
		{
			LOG_DBG("Target Reached \n");
			struct commandMsg cmd = {.datum1 = 100, .datum2 = yTargetSteps ,.datum3 = NODE};
			coap_client_cmdSend(-1, cmd);
			//if(per_c > 0.8) 
			k_sleep(K_MSEC(400+NODE*250));
		}

		//Was moving this to a timer function to not clutter main and to use this time to send messages for synchronisation.
		//this is the synchronous of way of taking steps
		// for(i = 0; i<scalar; i++){
		// 	gpio_pin_set(P0, step_pin, 1);

		// 	k_sleep(K_NSEC(period/scalar/2U));

		// 	gpio_pin_set(P0, step_pin, 0);

		// 	k_sleep(K_NSEC(period/scalar/2U));
		// }
		//ySteps = ySteps + 1.0/scalar*dir;
		oldySteps = ySteps; //record distance travelled at (k-1)T
		olderror = error; //record error at (k-1)T
    	ySteps = 3000.0*getPosition()/MAXENCODER*readPolarity;//*currentEncode.position/MAXENCODER;// //Read current position from the encoder and transform it to motor steps
		error = yTargetSteps - ySteps; //update the error for control

		if( oldySteps == ySteps ){ //if position has not changed at all, increase the notMovingCounter because the motor is stuck
			notMovingCounter++;
		}
		else{
			notMovingCounter = 0; //otherwise, reset the counter for time spent without moving
		}
		
		oldtime = uptime; //record previous step time
		//all olds have been updated
		uptime = k_uptime_ticks(); //take a new measurement of time
		if(uptime - oldtime > 0){
			ySpeed = (error - olderror)/(uptime-oldtime); //speed, is the derivative or the positional error for control in this case
		}
		else{
			ySpeed = error - olderror;
		}

		ierr = ierr + error/3000.0*(uptime-oldtime)/32786.0*5; //integral error i(k+1) = i(k) + e(k)*dt
		if(kI*ierr > 5){ //limit the maximum integral value
			ierr = 5.0/kI;
		}
		if(kI*ierr < -5){
			ierr = -5.0/kI;
		}

		a = kP*error + kD*ySpeed + kI*ierr; //PID method of computing absolute acceleration
		if(a > 20) //limit acceleration
			a = 20;
		if(a < -20)
		    a = -20;
 
		accel = a*dir; //acceleration with respect to current movement direction, allows us to only use 3 cases to update stepping period
		
		//set direction of movement accoring to dir
		if(dir == 1*invPolarity) 
			gpio_pin_set(P0, dir_pin, 0); //Away from motor

		if(dir == -1*invPolarity)
			gpio_pin_set(P0, dir_pin, 1); //Towards motor

    //printf("per_c = %f, ySteps = %f, a = %f, ySpeed = %f, dir = %d, scalar = %u, target = %f, ierr = %f\n",per_c,ySteps,a,ySpeed,dir,scalar,yTargetSteps,ierr); //for debugging only

	//update stepping period according to the acceleration profile and the deduced cases. Only 3 cases
	//Acceleration is with respect to the direction of movement, HENCE, speed v is always positive. Only 3 cases instead of 6-7
    if( (delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel < 0)
    {
		flip = 1;
		per_c = sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) - delta_phi/accel) - delta_phi/(accel*per_c*2);
		accel = -accel;
		dir = -dir;
    }
    else {
        if(accel > 0.01) //to avoid diving by very small numbers
        {
    		    per_c = sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel) - delta_phi/(accel*per_c*2);
    	}
    	else
            if(accel < -0.01) //to avoid diving by very small numbers
    		{
    		        per_c = -sqrt((delta_phi)*(delta_phi)/(accel*accel*per_c*per_c*4) + delta_phi/accel) - delta_phi/(accel*per_c*2);
    		}
	}

		placeholder = per_c*1000000000;
		period = placeholder; //transform the obtained period in seconds (per_c) to the period in nanoseconds (period)

		//Security measures to never go above maximum speed or bellow minimum speed
		if(period*scalar < MIN_PER){
			period = MIN_PER;
			per_c = period/1000000000.0;
		}
		if(period*scalar > MAX_PER){
			period = MAX_PER;
			per_c = period/1000000000.0;
			//printf("Should have theoretically stopped, per_c = %f\n",per_c);
		}


		//Uncomment for using microstepping mode
		// if(period/scalar > GEAR_PER+GEAR_GUARD && scalar < 20U){ //if going slower than a predefined speed
		// 	scalar = scalar*2U;  //gear down
		// }
		// else{
		// 	if(period/scalar*2 < GEAR_PER-GEAR_GUARD) //if going faster than said speed
		// 		scalar = scalar/2U; //gear up
		// 		if(scalar < 1U) scalar = 1U;
		// }
		// switch(scalar){ //set the pins to according mode
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

	manCon: //jump to here if manual control was requested
	while(1){
		if(manualControl == false){
			LOG_DBG("Switch to automatic mode");
			goto restart; //go back to automatic mode
		}
		if(mainloop == false){ //go back to waiting if received command to stop main
			LOG_DBG("Restart mode");
			goto restart;
		}
		//set direction pin
		if(dir == 1*invPolarity)
			gpio_pin_set(P0, dir_pin, 0); //Away from motor

		if(dir == -1*invPolarity)
			gpio_pin_set(P0, dir_pin, 1); //Towards motor

		//if told to move or if a forced step should be taken, step at a predefined frequency of 4/MIN_PER
		//forcedStep is used to override protection
		if(shouldMove || forcedStep){
			forcedStep = false;
			gpio_pin_set(P0, step_pin, 1);

			k_sleep(K_NSEC(MIN_PER/4U));

			gpio_pin_set(P0, step_pin, 0);

			k_sleep(K_NSEC(MIN_PER/4U));
		}
    	ySteps = 3000.0*getPosition()/MAXENCODER*readPolarity; //read current position
		if(!shouldMove){ //wait if should not be moving
			k_sleep(K_NSEC(3000));
		}
		if(ySteps < 20){ //if too close to the edge, security measures. Stop moving
			shouldMove = false;
		}
		if(ySteps > 2300){ //if too close to the opposite end, stop moving
			shouldMove = false;
		}
	}

  end: return;
}
