#include "uart.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>

#define BUFFER_LENGTH 2021
static uint8_t buf[BUFFER_LENGTH];

static int uart_fd = -1;

int uart_read(uint8_t *buf, int size);
int uart_write(uint8_t *buf, int count);

void uart_init(const char *path) {
    printf("Openining %s ...",path);
    uart_fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd == -1) {
            perror("Failed to open UART device!\n");
            return;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &options);
    printf("Done.\n");
    return;
}

void uart_close() {
        printf("Closing UART.\n");
        close (uart_fd);
        uart_fd = -1;
}

void uart_send(mavlink_message_t *mavlink_msg) {
    int ret;
    ret = mavlink_msg_to_send_buffer(buf, mavlink_msg);
    ret=write(uart_fd, buf, ret);    
    if (ret<0) {
            perror("UART: Error writing");
    }
}

uint8_t uart_recv(mavlink_message_t *msg) {
	static ssize_t recsize = 0;
	static int i = 0;
	static mavlink_status_t status;

	i = read(uart_fd, (void *)buf+recsize, BUFFER_LENGTH-recsize);
	if (i >= 0) {
		recsize += i;
		for (i = 0; i < recsize; ++i)
		{

			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], msg, &status))
			{
				if ((i+1)<recsize) {
					memmove(buf,buf+i,recsize-i);
					recsize -= i;
				}
				return 1;
			}
		}
	} else {
		perror("UART: Error reading");
	}

	return 0;
}
