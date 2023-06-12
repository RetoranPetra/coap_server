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
#define full_length_in_steps 3000 //1 meter in motors steps is 3000, our gantry is 1 m x 1 m x 1 m
#define pi 3.14159265 //pi
#define MAXENCODER 30000.0 //1 meter in encoder steps, 30000 encoder steps = 3000 motor steps

#define RECEIVE_TIMEOUT 500 //used for reading UART inputs

bool newMessage = false;
bool mainloop = false; //Main does not start until this becomes true
bool inPrinting = false;
bool allBoardsTarget[5] = {false, false, false, false, false}; //keep track of whether or not all boards have arrived at their required targets


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
static void on_generic_request(char *msg) {//unused
  // Something to deal with message would normally go here. However, message is
  // just character string so it doesn't matter.
  LOG_INF("Generic Request event execution!");
}

static void on_float_request(double num) { LOG_INF("Number is: %f", num); }//unused, but can be useful
static void on_percentage_request(struct percentageStruct percent) {//unused, but can be useful
  ARG_UNUSED(percent);
  LOG_INF("Percentage request callback");
}
struct encoderMessage example = {//One way of sending sensor information
      .payload = 3000, .messageNum = 0, .command = 20};

const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));//Port for changing pins, ununsed on the CCU

//UART device obtained
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

//Define transmission buffer (changed from 8 to 32). Not used, only for checking if uart working
static uint8_t tx_buf[] =   {"nRF Connect SDK Fundamentals Course\n\r"
                             "Press 1-3 on your keyboard to toggle LEDS 1-3 on your development kit\n\r"}; //Transmission message displayed here
//Define receive buffer and initialise it
static uint8_t rx_buf[10] = {0}; //Buffer size set to 10

int posindex = 0; //keep track of current positon

int yTarget[] = {500,1500,2500}; //unused
int xTarget[] = {500,2500,500};//unused
uint32_t RightPos = 696969;//unused, for synchronisation of RightY and LeftY motors
uint32_t LeftPos = 696969;//unused, for synchronisation of RightY and LeftY motors
int rightDir = 1;//unused, for synchronisation of RightY and LeftY motors
int leftDir = 1;//unused, for synchronisation of RightY and LeftY motors
uint8_t target[] = {71,72,73};//{75,76,77,78};// //positions to cycle through in automatic mode
#define MAXSTATES 3; //max number of positions to cycle through in automatic mode. Number of elements of 'target'
bool cycleToNew = false; //ready to move to the next target
bool presetsMode = false; //cycling through the preset positions, or not
bool canCycleToNext = true; 
bool forceCycle = false;
bool fastCycle = false;
bool syncWork = false; 
bool manualControl = false; //switch between manual control and cycling automatically through preset positions

float yTargetSteps = 1500; //target, just a placeholder

int i = 0; //counter for loops

//UART application callback function
//Check this function to see what buttonscan be pressed to do everything form the CCU
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{switch (evt->type) {
		//Only defining the receiving modes
	case UART_RX_RDY:
		if((evt->data.rx.len) == 1){ //If something entered

		//Defining responses:
		//MANUAL MODE INPUTS WASD
		if(evt->data.rx.buf[evt->data.rx.offset] == 'w'){
			LOG_DBG("Upwards \n");
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 101,
			  .datum2 = 101,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//Up code uart (positive X direction in manual mode)
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'a'){
			LOG_DBG("Left \n");
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 103,
			  .datum2 = 103,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			mainloop = true;
			//Left code uart
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 's'){
			LOG_DBG("Downwards \n");
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 102,
			  .datum2 = 102,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'd'){
			LOG_DBG("Right \n");
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 104,
			  .datum2 = 104,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//Right code uart
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'f'){
			LOG_DBG("Stay \n");
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 105,
			  .datum2 = 105,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//Stay Still code uart
		}
		//END OF MANUAL MODE INPUTS WASD
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'm'){
			manualControl = !manualControl;
			LOG_DBG("Switch to manual = %d\n",manualControl);
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 106,
			  .datum2 = 106,
			  .datum3 = manualControl
			};
			coap_client_cmdSend(-1,example);
			//Switch between manual or automatic by inputting m
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'e'){
			LOG_DBG("Target to 500");
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
			//Target a preset point of (500,500)
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'n'){
			fastCycle = !fastCycle;
			LOG_DBG("Fast cycling = %d\n",fastCycle);
			//make the board cycle between the points faster
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'q'){
			LOG_DBG("Target to 2500");
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
			//Target a preset point of (2500,2500)
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 't'){
			LOG_DBG("Target 0 \n");
			yTargetSteps = 0;
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 70,
			  .datum2 = yTargetSteps,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//Target a preset point of (0,0), that is come back to initial position
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'b'){
			LOG_DBG("Target middle \n");
			yTargetSteps = 1500;
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 70,
			  .datum2 = yTargetSteps,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			////Target a preset point of (1500,1500)
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'x'){
			LOG_DBG("No more main");
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 66,
			  .datum2 = 0,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			mainloop = false;
			//Tell all boards to stop operation
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'r'){
			resetPosition(0);
			LOG_DBG("Reset position to 0");
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 67,
			  .datum2 = 0,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//Calibrate all boards to consider current position is the (0,0) position
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == ' '){
			LOG_DBG("Asking to print Details");
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 68,
			  .datum2 = 0,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//ask all nodes to output useful information for debugging
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == 'p'){
			presetsMode = !presetsMode;
			LOG_DBG("Presets mode is now %d",presetsMode);
			//change between just inputing targets and automatic cycling through said targets
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
			//Send the message to start the main loop of every node
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '2'){
			coap_client_send_provisioning_request();
			//send a provisioning request for pairing
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '3'){
			serverScroll();
			//change who to send messages to. Unused
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '4'){
			k_work_submit(&provisioning_work);
			//check if received a provisioning request for pairing
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '5'){
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 71,
			  .datum2 = yTargetSteps,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//move to preset position 71
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '6'){
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 72,
			  .datum2 = yTargetSteps,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//move to preset position 72
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '7'){
			struct commandMsg example = {
			  .cmd = 0,
			  .datum1 = 73,
			  .datum2 = yTargetSteps,
			  .datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			//move to preset position 73
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

void fast_work_handler(struct k_work *work) //cycle quickly through the presets. Only if set to true by inputting 'n' on uart
{
	canCycleToNext = true;
	cycleToNew = true;
}
K_WORK_DEFINE(fast_work, fast_work_handler);

void fast_timer_handler(struct k_timer *timer_id)
{
	k_work_submit(&fast_work);
}
K_TIMER_DEFINE(fast_timer, fast_timer_handler, NULL); //set a timer to cycle between the presets

void wait_work_handler(struct k_work *work) //wait for a certain time after sending a new target before checking if it has been achieved
{
	LOG_DBG("I have allowed the change to the next pattern");
	canCycleToNext = true;
	// if(fastCycle){
	// 	k_timer_start(&fast_timer,K_MSEC(2000),K_FOREVER);
	// }
}
K_WORK_DEFINE(wait_work, wait_work_handler);

void wait_timer_handler(struct k_timer *timer_id)
{
	k_work_submit(&wait_work);
}
K_TIMER_DEFINE(wait_timer, wait_timer_handler, NULL);

struct encoderMessage currentEncode = {}; //encoder information message sending
static void on_encoder_request(struct encoderMessage encode) {
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

static void on_cmd_request(struct commandMsg cmd) {
	switch(cmd.datum1){
		case 1: //case 1 is for synchronisation between Right and Left motors. Unused right now
			LOG_DBG("Received message with %d and %d and %d",cmd.datum1,cmd.datum2,cmd.datum3);
			if(cmd.datum3 == RIGHTY){
				RightPos = cmd.datum2;
				if(cmd.cmd == (uint8_t) -1){
					rightDir = -1;
				}
				else{
					rightDir = 1;
				}
			}
			if(cmd.datum3 == LEFTY){
				LeftPos = cmd.datum2;
				if(cmd.cmd == (uint8_t) -1){
					leftDir = -1;
				}
				else{
					leftDir = 1;
				}
			}
			if((RightPos < 696969) && (LeftPos < 696969)){
				LOG_DBG("Gonna Sync them");
				syncWork = true;
			}
			break;
		case 2:
			
			break;
		case 69: //case 69 is for starting a node. Unused on the CCU
			mainloop = true;
			break;
		case 70: //case 70 is for changing the target of the node to datum2. Unused on the CCU
			yTargetSteps = cmd.datum2;
			break;
		case 100: //if received message from node that it has reached the target, see if all nodes reached the target and move to the next position if yes
			if(NODE == CCU){
				allBoardsTarget[cmd.datum3] = true;
				LOG_DBG("Board Number %d has arrived",cmd.datum3);
				if(allBoardsTarget[MIDX] && allBoardsTarget[LEFTY] && allBoardsTarget[RIGHTY]){
					if(canCycleToNext){
						cycleToNew = true;
						canCycleToNext = false;
						k_timer_start(&wait_timer,K_MSEC(2000),K_FOREVER);
					}
			}
			}
			
		default:
		break;
}
}
//openthread functions
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
    mainloop = true;
  }
#endif
}


void main(void)
{
    // Need to sleep at start for logs to display correctly.
  k_msleep(1000);
#ifdef SERVER
//initialise openthread functions
  int ret;

  LOG_INF("Start CoAP-server sample with Node %d",NODE);

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

	if (!device_is_ready(P0)) {
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

	printk("Control Wirelessly Correct\n");
	k_sleep(K_NSEC(2000U*1000U*1000U));


	//The CCU does not do anything in MAIN besides waiting for data form the motor actuating nodes and sending them targets or receiving their positons.
	while (1) {		
		//delta_phi = delta_phi_start/scalar;
		if(cycleToNew && presetsMode){ //if it shoudl move to the next target, send the message to move to the next target
			k_sleep(K_MSEC(200));
			posindex = (posindex+1)%MAXSTATES; //index of next target
			LOG_DBG("Cycling to pos %d",posindex);
			k_sleep(K_USEC(300));
			struct commandMsg example = {
			.cmd = 0,
			.datum1 = target[posindex], //will be a number like 71, 72 or 73, each corresponding to a agreed position
			.datum2 = 6969,
			.datum3 = 0
			};
			coap_client_cmdSend(-1,example);
			LOG_DBG("Moving to next target");
			k_sleep(K_USEC(2000));
			cycleToNew = false;
			for(i = 0; i<5; i++){
				allBoardsTarget[i] = false; //reset the fact that all boards have achieved their target
			}
			if(fastCycle){ //if fast mode. Cycle every 3 seconds, even if the target is not reached, to Show degradation with control
				k_timer_stop(&wait_timer);
				k_timer_start(&fast_timer,K_MSEC(3000),K_FOREVER);
			}
		}
		//Used to be for Synchronising the motors, sometimes fails, hence, not used in the final product for security
		// if(syncWork){
		// 	//700 - 600 = 100/1
		// 	//600 - 700 = -100/-1
		// 	if(rightDir*(RightPos-LeftPos) > 200){//Right is ahead
		// 	struct commandMsg example = {
		// 	.cmd = 0,
		// 	.datum1 = 2,
		// 	.datum2 = 0,
		// 	.datum3 = RIGHTY
		// 	};
		// 	coap_client_cmdSend(-1,example);
		// 	LOG_DBG("Told Right to wait");
		// 	LeftPos = 696969;
		// 	RightPos = 696969;
		// 	}else if(leftDir*(LeftPos-RightPos) > 200){//Right is ahead
		// 		struct commandMsg example = {
		// 		.cmd = 0,
		// 		.datum1 = 2,
		// 		.datum2 = 0,
		// 		.datum3 = LEFTY
		// 		};
		// 		coap_client_cmdSend(-1,example);
		// 		LOG_DBG("Told Left to wait");
		// 		LeftPos = 696969;
		// 		RightPos = 696969;
		// 		}
		// 	syncWork = false;
		// }
		k_sleep(K_USEC(2000));
	}

  end: return;
}
