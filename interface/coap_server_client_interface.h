/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef __COAP_SERVER_CLIENT_INTRFACE_H__
#define __COAP_SERVER_CLIENT_INTRFACE_H__
#include "node.h"
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
struct percentageStructHidden {
  uint32_t percentages[3];
  uint16_t messageNum;
  char identifier[8];
};
struct percentageStruct {
  double percentages[3];
  uint16_t messsageNum;
  char identifier[8];
};
struct encoderMessage {
  int8_t nodeOrigin;
  int32_t payload;
  int8_t command;
  uint16_t messageNum;
};
struct commandMsg {
  uint8_t cmd;
  int8_t nodeOrigin;
  uint32_t datum1;
  uint32_t datum2;
  uint32_t datum3;
  uint16_t msgNum;
};
// Payload size seems to be limited to https://stackoverflow.com/questions/42203857/how-much-data-can-hold-coap-in-single-message 127 bytes.

#define PROVISIONING_URI_PATH "provisioning"

#define CMD_URI_PATH "cmd"
#define CMD_PAYLOAD_SIZE sizeof(struct commandMsg)

#define LIGHT_URI_PATH "light"
#define LIGHT_PAYLOAD_SIZE 1

#define GENERIC_URI_PATH "generic"
#define GENERIC_PAYLOAD_SIZE 64

#define FLOAT_URI_PATH "float"

#define ENCODER_URI_PATH "encoder"
#define ENCODER_PAYLOAD_SIZE sizeof(struct encoderMessage)

#define PERCENTAGE_URI_PATH "percentage"
#define PERCENTAGE_PAYLOAD_SIZE sizeof(struct percentageStructHidden) //Should be 120.

#endif
