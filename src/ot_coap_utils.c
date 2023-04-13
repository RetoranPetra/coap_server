/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/logging/log.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/openthread.h>
#include <zephyr/net/socket.h>
#include <openthread/coap.h>
#include <openthread/ip6.h>
#include <openthread/message.h>
#include <openthread/thread.h>

#include "ot_coap_utils.h"

#include <net/coap_utils.h>

LOG_MODULE_REGISTER(ot_coap_utils, CONFIG_OT_COAP_UTILS_LOG_LEVEL);


// Client setup stuff
#define RESPONSE_POLL_PERIOD 100

static int serverSelector = 0;

static uint32_t poll_period;

static bool is_connected[SERVERS];

static struct k_work provisioning_work;
static struct k_work genericSend_work;
static struct k_work unicast_light_work;
//Must point to something of size GENERIC_PAYLOAD_SIZE
static char messagePointer[GENERIC_PAYLOAD_SIZE]= {};

/* Options supported by the server */
static const char *const light_option[] = { LIGHT_URI_PATH, NULL };
static const char *const provisioning_option[] = { PROVISIONING_URI_PATH,
						   NULL };
static const char *const generic_option[] = { GENERIC_URI_PATH, NULL};

/* Thread multicast mesh local address */
static struct sockaddr_in6 multicast_local_addr = {
	.sin6_family = AF_INET6,
	.sin6_port = htons(COAP_PORT),
	.sin6_addr.s6_addr = { 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 },
	.sin6_scope_id = 0U
};

/* Variable for storing server address acquiring in provisioning handshake */
static char unique_local_addr_str[SERVERS][INET6_ADDRSTRLEN];
static struct sockaddr_in6 unique_local_addr[SERVERS] = {
	{.sin6_family = AF_INET6,.sin6_port = htons(COAP_PORT),.sin6_addr.s6_addr = {0, },.sin6_scope_id = 0U},
	{.sin6_family = AF_INET6,.sin6_port = htons(COAP_PORT),.sin6_addr.s6_addr = {0, },.sin6_scope_id = 0U},
	{.sin6_family = AF_INET6,.sin6_port = htons(COAP_PORT),.sin6_addr.s6_addr = {0, },.sin6_scope_id = 0U}
};

//m
void serverScroll(void) {
	serverSelector++;
	serverSelector = serverSelector%SERVERS;
	LOG_INF("Selected Server %i Address: %s",serverSelector,unique_local_addr_str[serverSelector]);
}
//m/

//m
static void genericSend(struct k_work *item) {
	ARG_UNUSED(item);

	LOG_DBG("Generic send to %s",unique_local_addr_str[serverSelector]);
	

	if (coap_send_request(COAP_METHOD_PUT,
			  (const struct sockaddr *)&unique_local_addr[serverSelector],
			  generic_option, messagePointer, GENERIC_PAYLOAD_SIZE, NULL) >= 0) {
		
		LOG_DBG("Generic message send success!\n%s",messagePointer);
	}
	else {
		LOG_DBG("Generic message send fail.\n%s",messagePointer);
	}
}
//m/
static int on_provisioning_reply(const struct coap_packet *response,
				 struct coap_reply *reply,
				 const struct sockaddr *from)
{
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

static void toggle_one_light(struct k_work *item)
{
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

static void send_provisioning_request(struct k_work *item)
{
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

static void submit_work_if_connected(struct k_work *work)
{
	if (is_connected) {
		k_work_submit(work);
	} else {
		LOG_INF("Connection is broken");
	}
}

void coap_client_toggle_one_light(void)
{
	submit_work_if_connected(&unicast_light_work);
}

void coap_client_send_provisioning_request(void)
{
	submit_work_if_connected(&provisioning_work);
}

void coap_client_genericSend(char* msg) {
	memcpy(messagePointer,msg,GENERIC_PAYLOAD_SIZE);
	submit_work_if_connected(&genericSend_work);
}
// End of client setup

struct server_context {
	struct otInstance *ot;
	bool provisioning_enabled;
	light_request_callback_t on_light_request;
	provisioning_request_callback_t on_provisioning_request;
	generic_request_callback_t on_generic_request;
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

static otCoapResource generic_resource = {
	.mUriPath = GENERIC_URI_PATH,
	.mHandler = NULL,
	.mContext = NULL,
	.mNext = NULL
};

static otError provisioning_response_send(otMessage *request_message,
					  const otMessageInfo *message_info)
{
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

	error = otCoapMessageSetToken(
		response, otCoapMessageGetToken(request_message),
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

	//Sends response back as a return
	error = otCoapSendResponse(srv_context.ot, response, message_info);

	LOG_HEXDUMP_INF(payload, payload_size, "Sent provisioning response:");

end:
	if (error != OT_ERROR_NONE && response != NULL) {
		otMessageFree(response);
	}

	return error;
}

static void provisioning_request_handler(void *context, otMessage *message,
					 const otMessageInfo *message_info)
{
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
				  const otMessageInfo *message_info)
{
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

	if (otMessageRead(message, otMessageGetOffset(message), &command, 1) !=
	    1) {
		LOG_ERR("Light handler - Missing light command");
		goto end;
	}

	LOG_INF("Received light request: %c", command);

	srv_context.on_light_request(command);

end:
	return;
}
//m
//My new handler for just sending text messages to one another.
static void generic_request_handler(void *context, otMessage *message,
				 const otMessageInfo *message_info)
{
	//Need to do this with malloc if it's going to be accessed outside this function using srv_context.
	char myBuffer[GENERIC_PAYLOAD_SIZE] = {};

	//otMessageLength could be used in place of generic payload size.

	otMessageRead(message,otMessageGetOffset(message),&myBuffer, GENERIC_PAYLOAD_SIZE);

	ARG_UNUSED(context);
	ARG_UNUSED(message_info);

	LOG_INF("Message received is:\n%s",myBuffer);

	srv_context.on_generic_request(myBuffer);
}
//m/

static void coap_default_handler(void *context, otMessage *message,
				 const otMessageInfo *message_info)
{
	ARG_UNUSED(context);
	ARG_UNUSED(message);
	ARG_UNUSED(message_info);

	LOG_INF("Received CoAP message that does not match any request "
		"or resource");
}

void ot_coap_activate_provisioning(void)
{
	srv_context.provisioning_enabled = true;
}

void ot_coap_deactivate_provisioning(void)
{
	srv_context.provisioning_enabled = false;
}

bool ot_coap_is_provisioning_active(void)
{
	return srv_context.provisioning_enabled;
}

int ot_coap_init(provisioning_request_callback_t on_provisioning_request,
		 light_request_callback_t on_light_request, generic_request_callback_t on_generic_request)
{
	otError error;

	srv_context.provisioning_enabled = false;
	srv_context.on_provisioning_request = on_provisioning_request;
	srv_context.on_light_request = on_light_request;
	//m
	srv_context.on_generic_request = on_generic_request;
	//m/

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

	//m
	generic_resource.mContext = srv_context.ot;
	generic_resource.mHandler = generic_request_handler;
	//m/

	otCoapSetDefaultHandler(srv_context.ot, coap_default_handler, NULL);
	otCoapAddResource(srv_context.ot, &light_resource);
	otCoapAddResource(srv_context.ot, &provisioning_resource);
	//m
	otCoapAddResource(srv_context.ot, &generic_resource);
	//m/

	error = otCoapStart(srv_context.ot, COAP_PORT);
	if (error != OT_ERROR_NONE) {
		LOG_ERR("Failed to start OT CoAP. Error: %d", error);
		goto end;
	}

	coap_init(AF_INET6, NULL);

	//k_work_init(&on_connect_work, on_connect);
	//k_work_init(&on_disconnect_work, on_disconnect);
	k_work_init(&unicast_light_work, toggle_one_light);
	//k_work_init(&multicast_light_work, toggle_mesh_lights);
	k_work_init(&provisioning_work, send_provisioning_request);

	k_work_init(&genericSend_work,genericSend);

end:
	return error == OT_ERROR_NONE ? 0 : 1;
}
