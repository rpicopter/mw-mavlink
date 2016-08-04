#ifndef _CHANNEL_H_
#define _CHANNEL_H_

#include "udp.h"
#include "uart.h"

void channel_init();

void channel_close();

uint8_t channel_recv(mavlink_message_t *msg);

void channel_send(mavlink_message_t *mavlink_msg);

void channel_set_gcip(char *arg);

void channel_set_uartpath(char *arg);

void channel_set_gcport(int port);

void channel_set_localport(int port);

int channel_get_gcport();

int channel_get_localport();

#endif