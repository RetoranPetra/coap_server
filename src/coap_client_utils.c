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
#include "node.h"
#include "zephyr/kernel.h"
#include "zephyr/net/coap.h"

LOG_MODULE_REGISTER(coap_client_utils, CONFIG_COAP_CLIENT_UTILS_LOG_LEVEL);

// How often client will poll for response.
#define RESPONSE_POLL_PERIOD 100

static int serverSelector = 0;
static int connectSelector = 0;
static int serverTarget = 0;

static uint32_t poll_period;

static bool is_connected[6];
CONNECTIONS();

static struct k_work unicast_light_work;
static struct k_work multicast_light_work;
static struct k_work toggle_MTD_SED_work;
static struct k_work provisioning_work;

static struct k_work genericSend_work;
static struct k_work floatSend_work;
static struct k_work percentageSend_work;
static struct k_work encoderSend_work;
static struct k_work commandSend_work;

// Must point to something of size GENERIC_PAYLOAD_SIZE
static char messagePointer[GENERIC_PAYLOAD_SIZE] = {};
static struct encoderMessage encoderPointer[1] = {};

static double floatPointer[1] = {};

static struct percentageStructHidden percentagePointer[1] = {};
static struct commandMsg cmdPointer[1] = {};
static int cmdDoMulti = 0;

mtd_mode_toggle_cb_t on_mtd_mode_toggle;

/* Options supported by the server */
static const char *const light_option[] = {LIGHT_URI_PATH, NULL};
static const char *const provisioning_option[] = {PROVISIONING_URI_PATH, NULL};
static const char *const generic_option[] = {GENERIC_URI_PATH, NULL};
static const char *const float_option[] = {FLOAT_URI_PATH, NULL};
static const char *const percentage_option[] = {PERCENTAGE_URI_PATH, NULL};
static const char *const encoder_option[] = {ENCODER_URI_PATH, NULL};
static const char *const cmd_option[] = {CMD_URI_PATH, NULL};

/* Thread multicast mesh local address */
static struct sockaddr_in6 multicast_local_addr = {
    .sin6_family = AF_INET6,
    .sin6_port = htons(COAP_PORT),
    .sin6_addr.s6_addr = {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    .sin6_scope_id = 0U};

/* Variable for storing server address acquiring in provisioning handshake */
static char unique_local_addr_str[6][INET6_ADDRSTRLEN];
static struct sockaddr_in6 unique_local_addr[6] = {
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
  connectSelector++;
  connectSelector = connectSelector % SERVERS;
  serverSelector = connections[connectSelector];
  LOG_INF("Server %i | On? %i | Addr %s", serverSelector,
          is_connected[serverSelector], unique_local_addr_str[serverSelector]);
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

  LOG_INF("Received peer address: %s for %d",
          unique_local_addr_str[serverSelector], serverSelector);

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
// m
static void genericSend(struct k_work *item) {
  ARG_UNUSED(item);

  LOG_DBG("Generic send to %s", unique_local_addr_str[serverTarget]);

  if (coap_send_request(
          COAP_METHOD_PUT,
          (const struct sockaddr *)&unique_local_addr[serverTarget],
          generic_option, messagePointer, GENERIC_PAYLOAD_SIZE, NULL) >= 0) {

    LOG_DBG("Generic message send success!\n%s", messagePointer);
  } else {
    LOG_DBG("Generic message send fail.\n%s", messagePointer);
  }
}
static void floatSend(struct k_work *item) {
  ARG_UNUSED(item);
  LOG_DBG("Float send to %s", unique_local_addr_str[serverTarget]);
  if (coap_send_request(
          COAP_METHOD_PUT,
          (const struct sockaddr *)&unique_local_addr[serverTarget],
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
          (const struct sockaddr *)&unique_local_addr[serverTarget],
          percentage_option, (char *)percentagePointer, PERCENTAGE_PAYLOAD_SIZE,
          NULL) >= 0) {
    LOG_DBG("Percentage message send success!\n%s",
            percentagePointer->identifier);
  } else {
    LOG_DBG("Percentage message send fail.\n%s", percentagePointer->identifier);
  }
}
static void encoderSend(struct k_work *item) {
  ARG_UNUSED(item);
  if (coap_send_request(
          COAP_METHOD_PUT,
          (const struct sockaddr *)&unique_local_addr[serverTarget],
          encoder_option, (char *)encoderPointer, ENCODER_PAYLOAD_SIZE,
          NULL) >= 0) {
    LOG_DBG("Encoder message send success!\n");
  } else {
    LOG_DBG("Encoder message send fail!\n");
  }
}
static void cmdSend(struct k_work *item) {
  const struct sockaddr *address;
  if (cmdDoMulti) {
    address = (const struct sockaddr *)&multicast_local_addr;
  } else {
    address = (const struct sockaddr *)&unique_local_addr[serverTarget];
  }
  ARG_UNUSED(item);
  if (coap_send_request(COAP_METHOD_PUT, address, cmd_option,
                        (char *)cmdPointer, CMD_PAYLOAD_SIZE, NULL) >= 0) {
    LOG_DBG("Cmd message send success!\n");
  } else {
    LOG_DBG("Cmd message send fail!\n");
  }
}
// m/

static void submit_work_if_connected(struct k_work *work) {
  //  if (is_connected[serverTarget]) {
  if (serverTarget == -1) {
    LOG_DBG("Multicast Target!");
  } else {
    LOG_DBG("Target is %d AKA %s", serverTarget,
            unique_local_addr_str[serverTarget]);
  }
  if (true) {
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
  k_work_init(&percentageSend_work, percentageSend);
  k_work_init(&encoderSend_work, encoderSend);
  k_work_init(&commandSend_work, cmdSend);

  // openthread_state_changed_cb_register(openthread_get_default_context(),
  // &ot_state_chaged_cb); openthread_start(openthread_get_default_context());
  /*
  if (IS_ENABLED(CONFIG_OPENTHREAD_MTD_SED)) {
          k_work_init(&toggle_MTD_SED_work,
                      toggle_minimal_sleepy_end_device);
          update_device_state();
  }
  */
  serverSelector = connections[0];
}

void coap_client_toggle_one_light(void) {
  submit_work_if_connected(&unicast_light_work);
}

void coap_client_toggle_mesh_lights(void) {
  submit_work_if_connected(&multicast_light_work);
}

void coap_client_send_provisioning_request(void) {
  k_work_submit(&provisioning_work);
}

void coap_client_genericSend(int server, char *msg) {
  memcpy(messagePointer, msg, GENERIC_PAYLOAD_SIZE);
  serverTarget = server;
  submit_work_if_connected(&genericSend_work);
}

void coap_client_floatSend(int server, double num) {
  memcpy(floatPointer, &num, sizeof(double));
  serverTarget = server;
  submit_work_if_connected(&floatSend_work);
}

void coap_client_toggle_minimal_sleepy_end_device(void) {
  if (IS_ENABLED(CONFIG_OPENTHREAD_MTD_SED)) {
    k_work_submit(&toggle_MTD_SED_work);
  }
}
void coap_client_percentageSend(int server, struct percentageStruct input) {
  static uint16_t counter = 0;
  for (int i = 0; i < 3; i++) {
    percentagePointer->percentages[i] =
        input.percentages[i] * (double)4294967295;
  }
  memcpy(&percentagePointer->identifier, &input.identifier, 8);
  counter++;
  serverTarget = server;
  submit_work_if_connected(&percentageSend_work);
}
void coap_client_encoderSend(int server, struct encoderMessage input) {
  static uint16_t counter = 0;
  memcpy(encoderPointer, &input, ENCODER_PAYLOAD_SIZE);
  encoderPointer->messageNum = counter;
  encoderPointer->nodeOrigin = NODE;
  counter++;
  serverTarget = server;
  submit_work_if_connected(&encoderSend_work);
}

void coap_client_cmdSend(int server, struct commandMsg input) {
  static uint16_t counter = 0;
  cmdDoMulti = -1 == server;
  memcpy(cmdPointer, &input, CMD_PAYLOAD_SIZE);
  encoderPointer->messageNum = counter;
  encoderPointer->nodeOrigin = NODE;
  counter++;
  serverTarget = server;
  submit_work_if_connected(&commandSend_work);
}
