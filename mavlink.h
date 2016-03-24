#ifndef _MAVLINK_H_
#define _MAVLINK_H_
/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include "mavlink/common/mavlink.h"
#include "mw.h"
#include "udp.h"

uint8_t mavlink_init();
void mavlink_end();

void mavlink_loop();

void msg_sys_status();

void msg_radio_status();

void msg_heartbeat();

void msg_gps_raw_int();

void msg_attitude_quaternion();

void msg_command_long(mavlink_message_t *msg);

void msg_param_set_pid(mavlink_message_t *msg);

void msg_param_set(mavlink_message_t *msg);

void msg_param_request_read(mavlink_message_t *msg);

void msg_param_request_list(mavlink_message_t *msg);

uint8_t msg_mission_request_list(mavlink_message_t *msg);


void msg_manual_control(mavlink_message_t *msg);

#endif