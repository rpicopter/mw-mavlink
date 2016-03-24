#ifndef _UDP_H_
#define _UDP_H_

#include "mavlink/common/mavlink.h"

void udp_init(const char *target, const int target_port, const int local_port);

void udp_send(mavlink_message_t *mavlink_msg);

uint8_t udp_recv(mavlink_message_t *msg);

void udp_close();


void dispatch(mavlink_message_t *mavlink_msg);

#endif
