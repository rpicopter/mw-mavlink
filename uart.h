#ifndef _UART_H_
#define _UART_H_

#include "mavlink/common/mavlink.h"

void uart_init(const char *path);

void uart_send(mavlink_message_t *mavlink_msg);

uint8_t uart_recv(mavlink_message_t *msg);

void uart_close();

#endif