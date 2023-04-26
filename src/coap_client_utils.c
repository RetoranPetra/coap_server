/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "coap_server_client_interface.h"
#include <net/coap_utils.h>
#include <openthread/thread.h>
#include <string.h>
#include <sys/_stdint.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/openthread.h>
#include <zephyr/net/socket.h>

#include "coap_client_utils.h"

LOG_MODULE_REGISTER(coap_client_utils, CONFIG_COAP_CLIENT_UTILS_LOG_LEVEL);

// How often client will poll for response.
#define RESPONSE_POLL_PERIOD 100

static int serverSelector = 0;

static uint32_t poll_period;

static bool is_connected[SERVERS];

static struct k_work unicast_light_work;
static struct k_work multicast_light_work;
static struct k_work toggle_MTD_SED_work;
static struct k_work provisioning_work;
static struct k_work on_connect_work;
static struct k_work on_disconnect_work;

static struct k_work genericSend_work;
static struct k_work floatSend_work;
static struct k_work percentageSend_work;

// Must point to something of size GENERIC_PAYLOAD_SIZE
static char messagePointer[GENERIC_PAYLOAD_SIZE] = {};

static double floatPointer[1] = {};

static struct percentageStructHidden percentagePointer[1] = {};

mtd_mode_toggle_cb_t on_mtd_mode_toggle;

/* Options supported by the server */
static const char *const light_option[] = {LIGHT_URI_PATH, NULL};
static const char *const provisioning_option[] = {PROVISIONING_URI_PATH, NULL};
static const char *const generic_option[] = {GENERIC_URI_PATH, NULL};
static const char *const float_option[] = {FLOAT_URI_PATH, NULL};
static const char *const percentage_option[] = {PERCENTAGE_URI_PATH,NULL};

/* Thread multicast mesh local address */
static struct sockaddr_in6 multicast_local_addr = {
    .sin6_family = AF_INET6,
    .sin6_port = htons(COAP_PORT),
    .sin6_addr.s6_addr = {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    .sin6_scope_id = 0U};

/* Variable for storing server address acquiring in provisioning handshake */
static char unique_local_addr_str[SERVERS][INET6_ADDRSTRLEN];
static struct sockaddr_in6 unique_local_addr[SERVERS] = {
    {.sin6_family = AF_INET6,
     .sin6_port = htons(COAP_PORT),
     .sin6_addr.s6_addr =
         {
             0,
         },
     .sin6_scope_id = 0U},
    {.sin6_family = AF_INET6,
     .sin6_port = htons(COAP_PORT),
     .sin6_addr.s6_addr =
         {
             0,
         },
     .sin6_scope_id = 0U},
    {.sin6_family = AF_INET6,
     .sin6_port = htons(COAP_PORT),
     .sin6_addr.s6_addr =
         {
             0,
         },
     .sin6_scope_id = 0U}};

// m
void serverScroll(void) {
  serverSelector++;
  serverSelector = serverSelector % SERVERS;
  LOG_INF("Selected Server %i Address: %s", serverSelector,
          unique_local_addr_str[serverSelector]);
}
// m/

static bool is_mtd_in_med_mode(otInstance *instance) {
  return otThreadGetLinkMode(instance).mRxOnWhenIdle;
}

static void poll_period_response_set(void) {
  otError error;

  otInstance *instance = openthread_get_default_instance();

  if (is_mtd_in_med_mode(instance)) {
    return;
  }

  if (!poll_period) {
    poll_period = otLinkGetPollPeriod(instance);

    error = otLinkSetPollPeriod(instance, RESPONSE_POLL_PERIOD);
    __ASSERT(error == OT_ERROR_NONE, "Failed to set pool period");

    LOG_INF("Poll Period: %dms set", RESPONSE_POLL_PERIOD);
  }
}

static void poll_period_restore(void) {
  otError error;
  otInstance *instance = openthread_get_default_instance();

  if (is_mtd_in_med_mode(instance)) {
    return;
  }

  if (poll_period) {
    error = otLinkSetPollPeriod(instance, poll_period);
    __ASSERT_NO_MSG(error == OT_ERROR_NONE);

    LOG_INF("Poll Period: %dms restored", poll_period);
    poll_period = 0;
  }
}

static int on_provisioning_reply(const struct coap_packet *response,
                                 struct coap_reply *reply,
                                 const struct sockaddr *from) {
  int ret = 0;
  const uint8_t *payload;
  uint16_t payload_size = 0u;

  ARG_UNUSED(reply);
  ARG_UNUSED(from);

  payload = coap_packet_get_payload(response, &payload_size);

  if (payload == NULL ||
      payload_size != sizeof(unique_local_addr[serverSelector].sin6_addr)) {
    LOG_ERR("Received data is invalid");
    ret = -EINVAL;
    goto exit;
  }

  memcpy(&unique_local_addr[serverSelector].sin6_addr, payload, payload_size);

  if (!inet_ntop(AF_INET6, payload, unique_local_addr_str[serverSelector],
                 INET6_ADDRSTRLEN)) {
    LOG_ERR("Received data is not IPv6 address: %d", errno);
    ret = -errno;
    goto exit;
  }

  LOG_INF("Received peer address: %s", unique_local_addr_str[serverSelector]);

exit:
  if (IS_ENABLED(CONFIG_OPENTHREAD_MTD_SED)) {
    poll_period_restore();
  }

  return ret;
}

static void toggle_one_light(struct k_work *item) {
  uint8_t payload = (uint8_t)THREAD_COAP_UTILS_LIGHT_CMD_TOGGLE;

  ARG_UNUSED(item);

  if (unique_local_addr[serverSelector].sin6_addr.s6_addr16[0] == 0) {
    LOG_WRN("Peer address not set. Activate 'provisioning' option "
            "on the server side");
    return;
  }

  LOG_INF("Send 'light' request to: %s", unique_local_addr_str[serverSelector]);
  coap_send_request(COAP_METHOD_PUT,
                    (const struct sockaddr *)&unique_local_addr[serverSelector],
                    light_option, &payload, sizeof(payload), NULL);
}

static void toggle_mesh_lights(struct k_work *item) {
  static uint8_t command = (uint8_t)THREAD_COAP_UTILS_LIGHT_CMD_OFF;

  ARG_UNUSED(item);

  command = ((command == THREAD_COAP_UTILS_LIGHT_CMD_OFF)
                 ? THREAD_COAP_UTILS_LIGHT_CMD_ON
                 : THREAD_COAP_UTILS_LIGHT_CMD_OFF);

  LOG_INF("Send multicast mesh 'light' request");
  coap_send_request(COAP_METHOD_PUT,
                    (const struct sockaddr *)&multicast_local_addr,
                    light_option, &command, sizeof(command), NULL);
}

static void send_provisioning_request(struct k_work *item) {
  ARG_UNUSED(item);

  if (IS_ENABLED(CONFIG_OPENTHREAD_MTD_SED)) {
    /* decrease the polling period for higher responsiveness */
    poll_period_response_set();
  }

  LOG_INF("Send 'provisioning' request");
  coap_send_request(COAP_METHOD_GET,
                    (const struct sockaddr *)&multicast_local_addr,
                    provisioning_option, NULL, 0u, on_provisioning_reply);
}

static void toggle_minimal_sleepy_end_device(struct k_work *item) {
  otError error;
  otLinkModeConfig mode;
  struct openthread_context *context = openthread_get_default_context();

  __ASSERT_NO_MSG(context != NULL);

  openthread_api_mutex_lock(context);
  mode = otThreadGetLinkMode(context->instance);
  mode.mRxOnWhenIdle = !mode.mRxOnWhenIdle;
  error = otThreadSetLinkMode(context->instance, mode);
  openthread_api_mutex_unlock(context);

  if (error != OT_ERROR_NONE) {
    LOG_ERR("Failed to set MLE link mode configuration");
  } else {
    on_mtd_mode_toggle(mode.mRxOnWhenIdle);
  }
}

static void update_device_state(void) {
  struct otInstance *instance = openthread_get_default_instance();
  otLinkModeConfig mode = otThreadGetLinkMode(instance);
  on_mtd_mode_toggle(mode.mRxOnWhenIdle);
}

static void on_thread_state_changed(otChangedFlags flags,
                                    struct openthread_context *ot_context,
                                    void *user_data) {
  if (flags & OT_CHANGED_THREAD_ROLE) {
    switch (otThreadGetDeviceRole(ot_context->instance)) {
    case OT_DEVICE_ROLE_CHILD:
    case OT_DEVICE_ROLE_ROUTER:
    case OT_DEVICE_ROLE_LEADER:
      k_work_submit(&on_connect_work);
      is_connected[serverSelector] = true;
      break;

    case OT_DEVICE_ROLE_DISABLED:
    case OT_DEVICE_ROLE_DETACHED:
    default:
      k_work_submit(&on_disconnect_work);
      is_connected[serverSelector] = false;
      break;
    }
  }
}
static struct openthread_state_changed_cb ot_state_chaged_cb = {
    .state_changed_cb = on_thread_state_changed};

// m
static void genericSend(struct k_work *item) {
  ARG_UNUSED(item);

  LOG_DBG("Generic send to %s", unique_local_addr_str[serverSelector]);

  if (coap_send_request(
          COAP_METHOD_PUT,
          (const struct sockaddr *)&unique_local_addr[serverSelector],
          generic_option, messagePointer, GENERIC_PAYLOAD_SIZE, NULL) >= 0) {

    LOG_DBG("Generic message send success!\n%s", messagePointer);
  } else {
    LOG_DBG("Generic message send fail.\n%s", messagePointer);
  }
}
static void floatSend(struct k_work *item) {
  ARG_UNUSED(item);
  LOG_DBG("Float send to %s", unique_local_addr_str[serverSelector]);
  if (coap_send_request(
          COAP_METHOD_PUT,
          (const struct sockaddr *)&unique_local_addr[serverSelector],
          float_option, (char *)floatPointer, sizeof(double), NULL) >= 0) {

    LOG_DBG("Float message send success!\n%.3f", *floatPointer);
  } else {
    LOG_DBG("Float message send fail.\n%.3f", *floatPointer);
  }
}
static void percentageSend(struct k_work *item) {
  ARG_UNUSED(item);
  if (coap_send_request(
          COAP_METHOD_PUT,
          (const struct sockaddr *)&unique_local_addr[serverSelector],
          percentage_option, (char *)percentagePointer, PERCENTAGE_PAYLOAD_SIZE, NULL) >= 0) {
    LOG_DBG("Percentage message send success!\n%s", percentagePointer->identifier);
  } else {
    LOG_DBG("Percentage message send fail.\n%s", percentagePointer->identifier);
  }
}
// m/

static void submit_work_if_connected(struct k_work *work) {
  if (is_connected) {
    k_work_submit(work);
  } else {
    LOG_INF("Connection is broken");
  }
}

// Don't need on connection/toggles because server code handles light on joint
// node.
void coap_client_utils_init(/*
				ot_connection_cb_t on_connect,
			    ot_disconnection_cb_t on_disconnect,
			    mtd_mode_toggle_cb_t on_toggle
				*/)
{
  // on_mtd_mode_toggle = on_toggle;

  coap_init(AF_INET6, NULL);

  // k_work_init(&on_connect_work, on_connect);
  // k_work_init(&on_disconnect_work, on_disconnect);
  k_work_init(&unicast_light_work, toggle_one_light);
  k_work_init(&multicast_light_work, toggle_mesh_lights);
  k_work_init(&provisioning_work, send_provisioning_request);

  k_work_init(&genericSend_work, genericSend);
  k_work_init(&floatSend_work, floatSend);
  k_work_init(&percentageSend_work,percentageSend);

  // openthread_state_changed_cb_register(openthread_get_default_context(),
  // &ot_state_chaged_cb); openthread_start(openthread_get_default_context());
  /*
  if (IS_ENABLED(CONFIG_OPENTHREAD_MTD_SED)) {
          k_work_init(&toggle_MTD_SED_work,
                      toggle_minimal_sleepy_end_device);
          update_device_state();
  }
  */
}

void coap_client_toggle_one_light(void) {
  submit_work_if_connected(&unicast_light_work);
}

void coap_client_toggle_mesh_lights(void) {
  submit_work_if_connected(&multicast_light_work);
}

void coap_client_send_provisioning_request(void) {
  submit_work_if_connected(&provisioning_work);
}

void coap_client_genericSend(char *msg) {
  memcpy(messagePointer, msg, GENERIC_PAYLOAD_SIZE);
  submit_work_if_connected(&genericSend_work);
}

void coap_client_floatSend(double num) {
  memcpy(floatPointer, &num, sizeof(double));
  submit_work_if_connected(&floatSend_work);
}

void coap_client_toggle_minimal_sleepy_end_device(void) {
  if (IS_ENABLED(CONFIG_OPENTHREAD_MTD_SED)) {
    k_work_submit(&toggle_MTD_SED_work);
  }
}
void coap_client_percentageSend(struct percentageStruct input) {
  static uint16_t counter = 0;
  for (int i = 0;i<3;i++) {
    percentagePointer->percentages[i] = input.percentages[i] * (double)4294967295;
  }
  memcpy(&percentagePointer->identifier, &input.identifier, 8);
  counter++;
  submit_work_if_connected(&percentageSend_work);
}
