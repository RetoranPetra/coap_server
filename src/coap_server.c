/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

// Toggles to disable or enable functionality.
//#define CLIENT
//#define SERVER
//#define IMU
#define ENCODER

#include <dk_buttons_and_leds.h>
#include <openthread/thread.h>
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

#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
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

const struct device *P0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

LOG_MODULE_REGISTER(coap_server, CONFIG_COAP_SERVER_LOG_LEVEL);

#define OT_CONNECTION_LED DK_LED1
#define PROVISIONING_LED DK_LED3
#define LIGHT_LED DK_LED4

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

static void on_button_changed(uint32_t button_state, uint32_t has_changed) {
  uint32_t buttons = button_state & has_changed;
#ifdef CLIENT

  if (buttons & DK_BTN4_MSK) {
    k_work_submit(&provisioning_work);
  }
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
    coap_client_floatSend(10.768);
  }
#endif /* ifdef CLIENT
   */
}

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

static struct openthread_state_changed_cb ot_state_chaged_cb = {
    .state_changed_cb = on_thread_state_changed};

void main(void) {
  // Need to sleep at start for logs to display correctly.
  k_msleep(1000);
#ifdef SERVER
  int ret;

  LOG_INF("Start CoAP-server sample");

  k_timer_init(&led_timer, on_led_timer_expiry, on_led_timer_stop);
  k_timer_init(&provisioning_timer, on_provisioning_timer_expiry, NULL);

  k_work_init(&provisioning_work, activate_provisioning);

  ret = ot_coap_init(&deactivate_provisionig, &on_light_request,
                     &on_generic_request, &on_float_request);
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
  //otInstance *inst = openthread_get_default_instance();
  //otChannelManagerSetAutoChannelSelectionEnabled(inst, false);
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
    int32_t encpos = 0U;

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
          scalar = scalar/2U;
          if(scalar < 1U) scalar = 1U;
      }

    }
}
