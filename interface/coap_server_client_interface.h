/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef __COAP_SERVER_CLIENT_INTRFACE_H__
#define __COAP_SERVER_CLIENT_INTRFACE_H__

#include <zephyr/kernel.h>
#define COAP_PORT 5683

/**@brief Enumeration describing light commands. */
enum light_command {
	THREAD_COAP_UTILS_LIGHT_CMD_OFF = '0',
	THREAD_COAP_UTILS_LIGHT_CMD_ON = '1',
	THREAD_COAP_UTILS_LIGHT_CMD_TOGGLE = '2'
};

struct testStruct {
  char name[8];
  double position;
  float velocity;
  uint16_t timeSinceLast;
};
// Payload size seems to be limited to https://stackoverflow.com/questions/42203857/how-much-data-can-hold-coap-in-single-message 127 bytes.

#define PROVISIONING_URI_PATH "provisioning"


#define LIGHT_URI_PATH "light"
#define LIGHT_PAYLOAD_SIZE 1

#define GENERIC_URI_PATH "generic"
#define GENERIC_PAYLOAD_SIZE 64

#define FLOAT_URI_PATH "float"

#define SERVERS 3
#define CLIENTS 1

#endif
