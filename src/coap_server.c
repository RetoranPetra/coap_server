/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

// Toggles to disable or enable functionality.
#include "coap_server_client_interface.h"
#define CLIENT
#define SERVER
// #define IMU
// #define ENCODER

#include <dk_buttons_and_leds.h>
#include <openthread/thread.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/openthread.h>
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
    // coap_client_floatSend(10.768);
    /*
    struct percentageStruct example = {.percentages = {1.0,1.0,1.0},
      .identifier = "Hello!"};
    coap_client_percentageSend(example);
    */
    /*
    struct encoderMessage example = {
        .position = 3000, .messageNum = 0, .velocity = 20};
    switch (NODE) {
    case 0:
      LOG_DBG("I'm not initiating, I'm the master!");
      break;
    default:
      LOG_DBG("I'm %d, and I'll send to master!", NODE);
      coap_client_encoderSend(0, example);
      break;
    }
    */
    struct commandMsg example = {
      .cmd = 0,
      .datum1 = 100
    };
    coap_client_cmdSend(0,example,0);
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
      .position = 3000, .messageNum = 0, .velocity = 20};

static void on_encoder_request(struct encoderMessage encode) {
  LOG_DBG("Message Number: %i\nPosition:%i,Velocity:%i,Origin:%i",
          encode.messageNum, encode.position, encode.velocity,
          encode.nodeOrigin);

  // Return to sender after delay.
  k_msleep(500);
  coap_client_encoderSend((int)encode.nodeOrigin, encode);
}
static void on_cmd_request(struct commandMsg cmd) {
  LOG_DBG("CMD CALLBACK!");
}

static struct openthread_state_changed_cb ot_state_chaged_cb = {
    .state_changed_cb = on_thread_state_changed};
#endif

void main(void) {
  goto setup;
start:
  LOG_INF("START NODE %d!", NODE);

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
  // See https://openthread.io/reference/group/api-channel-manager

  // Auto channel stuff
  otInstance *inst = openthread_get_default_instance();
  otChannelManagerSetAutoChannelSelectionEnabled(inst, false);
  // Seems to not work.
  goto start;
end:
  return;
}
