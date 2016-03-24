#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include "udp.h"

#define BUFFER_LENGTH 2021

static int sock;

static struct sockaddr_in gcAddr; 
static struct sockaddr_in locAddr;

static uint8_t buf[BUFFER_LENGTH];
static int bytes_sent;
static uint16_t len;


void dispatch(mavlink_message_t *mavlink_msg) {
	udp_send(mavlink_msg);
}

void udp_init(const char *target, const int target_port, const int local_port) {
	sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(local_port);
 
	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
    {
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    } 
 
	/* Attempt to make it non blocking */
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
    }
 
 
	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target);
	gcAddr.sin_port = htons(target_port);	
}

void udp_send(mavlink_message_t *mavlink_msg) {
	len = mavlink_msg_to_send_buffer(buf, mavlink_msg);
	bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
}

uint8_t udp_recv(mavlink_message_t *msg) {
	static ssize_t recsize;
	static socklen_t fromlen;
	static int i = 0;
	static unsigned int temp = 0;
	static mavlink_status_t status;

	memset(buf, 0, BUFFER_LENGTH);
	recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
	if (recsize > 0)
		{
		// Something received - print out all bytes and parse packet
		//printf("Bytes Received: %d\nDatagram: ", (int)recsize);
		//printf("Received packet from: %s\n",inet_ntoa(gcAddr.sin_addr));
		for (i = 0; i < recsize; ++i)
		{
			temp = buf[i];
			//printf("%02x ", (unsigned char)temp);
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], msg, &status))
			{
				// Packet received
				//printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n\n", msg->sysid, msg->compid, msg->len, msg->msgid);
				return 1;
			}
		}
	};

	return 0;
}

void udp_close() {
	close(sock);
}

