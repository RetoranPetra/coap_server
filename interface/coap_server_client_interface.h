/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef __COAP_SERVER_CLIENT_INTRFACE_H__
#define __COAP_SERVER_CLIENT_INTRFACE_H__

#define COAP_PORT 5683

/**@brief Enumeration describing light commands. */
enum light_command {
	THREAD_DRIVE_MOTORS_BACKWARDS = '0',
	THREAD_STOP_MOTORS = '1',
	THREAD_DRIVE_MOTORS_FORWARD = '2'
};

#define PROVISIONING_URI_PATH "provisioning"


#define LIGHT_URI_PATH "light"
#define LIGHT_PAYLOAD_SIZE 1

#define GENERIC_URI_PATH "generic"
#define GENERIC_PAYLOAD_SIZE 64

#define FLOAT_URI_PATH "float"
#define FLOAT_PAYLOAD_SIZE 64

#endif
