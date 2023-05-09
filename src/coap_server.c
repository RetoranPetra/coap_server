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
//#include "AEDB_9140.h"
#endif
#ifdef IMU
#include "ICM20600.h"
#endif

#define Sampling_period 10 // in milli  secs
#define pi 3.14159265359
#define radius 8.5E-3
#define distance_per_step 2.0*3.14*8.5E-3/200.0

#define Stepper_Pin DT_ALIAS(stepper)
#define Direction_pin DT_ALIAS(stepperdir)
#define M0_Pin DT_ALIAS(m0)
#define M1_Pin DT_ALIAS(m1)
#define M2_Pin DT_ALIAS(m2)

float phiToSend = 0;

static const struct gpio_dt_spec STEP = GPIO_DT_SPEC_GET(Stepper_Pin, gpios);
static const struct gpio_dt_spec DIR = GPIO_DT_SPEC_GET(Direction_pin, gpios);
static const struct gpio_dt_spec M0 = GPIO_DT_SPEC_GET(M0_Pin, gpios);
static const struct gpio_dt_spec M1 = GPIO_DT_SPEC_GET(M1_Pin, gpios);
static const struct gpio_dt_spec M2 = GPIO_DT_SPEC_GET(M2_Pin, gpios);

double Step_time_interval;

double   Vm = 0;

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

  switch (command) {
  case THREAD_COAP_UTILS_LIGHT_CMD_ON:
    dk_set_led_on(LIGHT_LED);
    val = 1;
    break;

  case THREAD_COAP_UTILS_LIGHT_CMD_OFF:
    dk_set_led_off(LIGHT_LED);
    val = 0;
    break;

  case THREAD_COAP_UTILS_LIGHT_CMD_TOGGLE:
    val = !val;
    dk_set_led(LIGHT_LED, val);
    break;

  default:
    break;
  }
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

bool mainloop = false;

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
    /*
    struct encoderMessage example = {.position = 3000,
      .messageNum=0,.velocity=20};
    coap_client_encoderSend(example);
    */
    LOG_DBG("Start main loop!");
    mainloop = true;
  }
#endif
}
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
  LOG_DBG("Message Number: %i\nPosition:%i,Velocity:%i",encode.messageNum,encode.position,encode.velocity);
  LOG_DBG("Encoder request callback!");
}

static struct openthread_state_changed_cb ot_state_chaged_cb = {
    .state_changed_cb = on_thread_state_changed};
#endif

void main(void) {
  goto setup;
start:
  LOG_INF("START!");

  while (!mainloop) {
    k_msleep(500);
  }
  while (1) {
    k_msleep(Sampling_period);
	SimpleComplementaryFilter(getRawAccelerationX()*9.81/1000,getRawAccelerationY()*9.81/1000,getRawAccelerationZ()*9.81/1000,getRawGyroscopeX()*2*pi/360,getRawGyroscopeY()*2*pi/360,getRawGyroscopeZ()*2*pi/360,Sampling_period*pow(10,-3));
	phiToSend = phi;
    struct encoderMessage out = {.position = phiToSend,.messageNum=0,.velocity=0};
    //LOG_DBG("Position:%i",out.position);
    coap_client_encoderSend(out);
	
  }


  goto end;
setup:
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
 	gpio_pin_configure_dt(&STEP, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&DIR, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&M0,GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&M1,GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&M2,GPIO_OUTPUT_INACTIVE);
	ICM20600_startup();
    //printf("Temperature: %d",getTemperature());
    //k_timer_start(&my_timer, K_SECONDS(0), K_MSEC(Sampling_period));
	gpio_pin_set_dt(&M0,0);
	gpio_pin_set_dt(&M1,0);
	gpio_pin_set_dt(&M2,0);
#endif /* ifdef IMU */
#ifdef ENCODER
  Setup_interrupt();
#endif /* ifdef ENCODER */
  // See https://openthread.io/reference/group/api-channel-manager

  // Auto channel stuff
  otInstance *inst = openthread_get_default_instance();
  otChannelManagerSetAutoChannelSelectionEnabled(inst, false);
  // Seems to not work.

  // otChannelManagerSetFavoredChannels(inst, 5); // Doesn't set channel, just
  //  sets a preferred one so auto selector chooses it more often.
  // otChannelManagerSetDelay(inst, 1);
  // otChannelManagerRequestChannelSelect(inst, 7); // Request channel change to
  //  7, doesn't work it seems.
  /*
  LOG_DBG("Favoured channel: %d", otChannelManagerGetFavoredChannels(inst));
  while (1) {
    k_msleep(1000);
    LOG_DBG("Channel is: %d", nrf_802154_channel_get());
  }
  */
  goto start;
end:
  return;
}
