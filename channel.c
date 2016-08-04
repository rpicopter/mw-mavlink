#include "channel.h"

static uint8_t type = 0;
static char target_ip[64];
static int target_port=14550, local_port=14551;
static char uart_path[255] = "/dev/ttyUSB0";

void channel_set_gcip(char *arg) {
	type = 0;
	strcpy(target_ip,arg);
}

void channel_set_uartpath(char *arg) {
	type = 1;
	strcpy(uart_path,arg);
}

void channel_set_gcport(int port) {
	target_port = port;
}

void channel_set_localport(int port) {
	local_port = port;
}

int channel_get_gcport() {
	return target_port;
}

int channel_get_localport() {
	return local_port;
}


void channel_init() {
	if (type==0) udp_init(target_ip,target_port,local_port);
	if (type==1) uart_init(uart_path);
}

void channel_close() {
	if (type==0) udp_close();
	if (type==1) uart_close();		
}

uint8_t channel_recv(mavlink_message_t *msg) {
	if (type==0)
		return udp_recv(msg);
	if (type==1)
		return uart_recv(msg);	

	return 0;
}

void channel_send(mavlink_message_t *mavlink_msg) {
	if (type==0) udp_send(mavlink_msg);
	if (type==1) uart_send(mavlink_msg);		
}
