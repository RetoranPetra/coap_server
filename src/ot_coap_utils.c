/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <openthread/coap.h>
#include <openthread/ip6.h>
#include <openthread/message.h>
#include <openthread/thread.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/openthread.h>
#include <zephyr/net/socket.h>

#include "coap_server_client_interface.h"
#include "ot_coap_utils.h"

#include <net/coap_utils.h>

LOG_MODULE_REGISTER(ot_coap_utils, CONFIG_OT_COAP_UTILS_LOG_LEVEL);

double message_double;

struct server_context {
  struct otInstance *ot;
  bool provisioning_enabled;
  light_request_callback_t on_light_request;
  provisioning_request_callback_t on_provisioning_request;
  generic_request_callback_t on_generic_request;
  float_request_callback_t on_float_request;
};

static struct server_context srv_context = {
    .ot = NULL,
    .provisioning_enabled = false,
    .on_light_request = NULL,
    .on_provisioning_request = NULL,
    .on_generic_request = NULL,
};

/**@brief Definition of CoAP resources for provisioning. */
static otCoapResource provisioning_resource = {
    .mUriPath = PROVISIONING_URI_PATH,
    .mHandler = NULL,
    .mContext = NULL,
    .mNext = NULL,
};

/**@brief Definition of CoAP resources for light. */
static otCoapResource light_resource = {
    .mUriPath = LIGHT_URI_PATH,
    .mHandler = NULL,
    .mContext = NULL,
    .mNext = NULL,
};

static otCoapResource generic_resource = {.mUriPath = GENERIC_URI_PATH,
                                          .mHandler = NULL,
                                          .mContext = NULL,
                                          .mNext = NULL};

static otCoapResource float_resource = {.mUriPath = FLOAT_URI_PATH,
                                        .mHandler = NULL,
                                        .mContext = NULL,
                                        .mNext = NULL};

static otError provisioning_response_send(otMessage *request_message,
                                          const otMessageInfo *message_info) {
  otError error = OT_ERROR_NO_BUFS;
  otMessage *response;
  const void *payload;
  uint16_t payload_size;

  response = otCoapNewMessage(srv_context.ot, NULL);
  if (response == NULL) {
    goto end;
  }

  otCoapMessageInit(response, OT_COAP_TYPE_NON_CONFIRMABLE,
                    OT_COAP_CODE_CONTENT);

  error =
      otCoapMessageSetToken(response, otCoapMessageGetToken(request_message),
                            otCoapMessageGetTokenLength(request_message));
  if (error != OT_ERROR_NONE) {
    goto end;
  }

  error = otCoapMessageSetPayloadMarker(response);
  if (error != OT_ERROR_NONE) {
    goto end;
  }

  payload = otThreadGetMeshLocalEid(srv_context.ot);
  payload_size = sizeof(otIp6Address);

  error = otMessageAppend(response, payload, payload_size);
  if (error != OT_ERROR_NONE) {
    goto end;
  }

  // Sends response back as a return
  error = otCoapSendResponse(srv_context.ot, response, message_info);

  LOG_HEXDUMP_INF(payload, payload_size, "Sent provisioning response:");

end:
  if (error != OT_ERROR_NONE && response != NULL) {
    otMessageFree(response);
  }

  return error;
}

static void provisioning_request_handler(void *context, otMessage *message,
                                         const otMessageInfo *message_info) {
  otError error;
  otMessageInfo msg_info;

  ARG_UNUSED(context);

  if (!srv_context.provisioning_enabled) {
    LOG_WRN("Received provisioning request but provisioning "
            "is disabled");
    return;
  }

  LOG_INF("Received provisioning request");

  if ((otCoapMessageGetType(message) == OT_COAP_TYPE_NON_CONFIRMABLE) &&
      (otCoapMessageGetCode(message) == OT_COAP_CODE_GET)) {
    msg_info = *message_info;
    memset(&msg_info.mSockAddr, 0, sizeof(msg_info.mSockAddr));

    error = provisioning_response_send(message, &msg_info);
    if (error == OT_ERROR_NONE) {
      srv_context.on_provisioning_request();
      srv_context.provisioning_enabled = false;
    }
  }
}

static void light_request_handler(void *context, otMessage *message,
                                  const otMessageInfo *message_info) {
  uint8_t command;

  ARG_UNUSED(context);

  if (otCoapMessageGetType(message) != OT_COAP_TYPE_NON_CONFIRMABLE) {
    LOG_ERR("Light handler - Unexpected type of message");
    goto end;
  }

  if (otCoapMessageGetCode(message) != OT_COAP_CODE_PUT) {
    LOG_ERR("Light handler - Unexpected CoAP code");
    goto end;
  }

  if (otMessageRead(message, otMessageGetOffset(message), &command, 1) != 1) {
    LOG_ERR("Light handler - Missing light command");
    goto end;
  }

  LOG_INF("Received light request: %c", command);

  srv_context.on_light_request(command);

end:
  return;
}
// m
// My new handler for just sending text messages to one another.
static void generic_request_handler(void *context, otMessage *message,
                                    const otMessageInfo *message_info) {
  // Need to do this with malloc if it's going to be accessed outside this
  // function using srv_context.
  char myBuffer[GENERIC_PAYLOAD_SIZE] = {};

  // otMessageLength could be used in place of generic payload size.

  otMessageRead(message, otMessageGetOffset(message), &myBuffer,
                GENERIC_PAYLOAD_SIZE);

  ARG_UNUSED(context);
  ARG_UNUSED(message_info);

  LOG_INF("Message received is:\n%s", myBuffer);

  srv_context.on_generic_request(myBuffer);
}
// m/
static void float_request_handler(void *context, otMessage *message,
                                  const otMessageInfo *message_info) {
  // Need to do this with malloc if it's going to be accessed outside this
  // function using srv_context.
  double myBuffer = 0.0;

  // otMessageLength could be used in place of generic payload size.

  otMessageRead(message, otMessageGetOffset(message), &myBuffer,
                sizeof(double));

  ARG_UNUSED(context);
  ARG_UNUSED(message_info);

  message_double = myBuffer;
  LOG_INF("Message received is:\n%f", myBuffer);
  
  srv_context.on_float_request(myBuffer);
}
double get_double(void){
  return message_double;
}

static void coap_default_handler(void *context, otMessage *message,
                                 const otMessageInfo *message_info) {
  ARG_UNUSED(context);
  ARG_UNUSED(message);
  ARG_UNUSED(message_info);

  LOG_INF("Received CoAP message that does not match any request "
          "or resource");
}

void ot_coap_activate_provisioning(void) {
  srv_context.provisioning_enabled = true;
}

void ot_coap_deactivate_provisioning(void) {
  srv_context.provisioning_enabled = false;
}

bool ot_coap_is_provisioning_active(void) {
  return srv_context.provisioning_enabled;
}

int ot_coap_init(provisioning_request_callback_t on_provisioning_request,
                 light_request_callback_t on_light_request,
                 generic_request_callback_t on_generic_request,
                 float_request_callback_t on_float_request) {
  otError error;

  srv_context.provisioning_enabled = false;
  srv_context.on_provisioning_request = on_provisioning_request;
  srv_context.on_light_request = on_light_request;
  // m
  srv_context.on_generic_request = on_generic_request;
  srv_context.on_float_request = on_float_request;
  // m/

  srv_context.ot = openthread_get_default_instance();
  if (!srv_context.ot) {
    LOG_ERR("There is no valid OpenThread instance");
    error = OT_ERROR_FAILED;
    goto end;
  }

  provisioning_resource.mContext = srv_context.ot;
  provisioning_resource.mHandler = provisioning_request_handler;

  light_resource.mContext = srv_context.ot;
  light_resource.mHandler = light_request_handler;

  // m
  generic_resource.mContext = srv_context.ot;
  generic_resource.mHandler = generic_request_handler;

  float_resource.mContext = srv_context.ot;
  float_resource.mHandler = float_request_handler;
  // m/

  otCoapSetDefaultHandler(srv_context.ot, coap_default_handler, NULL);
  otCoapAddResource(srv_context.ot, &light_resource);
  otCoapAddResource(srv_context.ot, &provisioning_resource);
  // m
  otCoapAddResource(srv_context.ot, &generic_resource);
  otCoapAddResource(srv_context.ot, &float_resource);
  // m/

  error = otCoapStart(srv_context.ot, COAP_PORT);
  if (error != OT_ERROR_NONE) {
    LOG_ERR("Failed to start OT CoAP. Error: %d", error);
    goto end;
  }
end:
  return error == OT_ERROR_NONE ? 0 : 1;
}
