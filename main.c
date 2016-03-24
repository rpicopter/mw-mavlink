
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <signal.h>


#include "mw.h"
#include "udp.h"
#include "mavlink.h"
#include "def.h"
#include "global.h"

#ifdef CFG_ENABLED
	#include <sys/stat.h> 
	#include <fcntl.h>
	#define CFG_FILE "/etc/mw-mavlink.cfg"
	config_t cfg;
#endif

uint8_t debug = 1;

uint8_t stop = 0;
uint16_t loop_counter = 0;

typedef void (*t_cb)();

struct _S_TASK {
	uint16_t freq; //has to be less than loop_counter max value
	t_cb cb_fn;
};

typedef struct _S_TASK S_TASK;

void check_incoming_udp(); //checks for messages on UDP port

#define MAX_TASK 3
S_TASK task[MAX_TASK] = {
	{1, check_incoming_udp},
	{1, mw_loop},
	{1, mavlink_loop}
};

static mavlink_message_t mav_msg;


void check_incoming_udp() {

	while (udp_recv(&mav_msg)) {
		if (debug) printf("<- MsgID: %u\n",mav_msg.msgid);
		switch (mav_msg.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT: break;

			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				msg_param_request_list(&mav_msg);
				break;
			case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
				msg_param_request_read(&mav_msg);
				break;
			case MAVLINK_MSG_ID_PARAM_SET:
				msg_param_set(&mav_msg);
				break;				
			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
				msg_mission_request_list(&mav_msg);
				break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
				msg_command_long(&mav_msg);
				break;	
			case MAVLINK_MSG_ID_MANUAL_CONTROL:
				msg_manual_control(&mav_msg);
				break;			
			default: printf("Unknown message id: %u\n",mav_msg.msgid);
		}
		//process message
	}
}



void mssleep(unsigned int ms) {
  struct timespec tim;
   tim.tv_sec = ms/1000;
   tim.tv_nsec = 1000000L * (ms % 1000);
   if(nanosleep(&tim , &tim) < 0 )
   {
      printf("Nano sleep system call failed \n");
   }
}


//runs all tasks as per defined frequency
void loop() {
	uint8_t i;
	loop_counter=0;

	static int test = 20;
	while (!stop) {

		for (i=0;i<MAX_TASK;i++)
			if (loop_counter%task[i].freq==0) {
				if (test) {
					printf("Running task %u\n",i);
					test--;
				}
				task[i].cb_fn();
			}

		mssleep(LOOP_MS);
		loop_counter++;
		if (loop_counter==1000) loop_counter=0;
	}
}

char target_ip[64];
int target_port=14550, local_port;

void print_usage() {
    printf("Usage:\n");
	printf("-h\thelp\n");
    printf("-t TARGET\tip address of QGroundControl\n");
    printf("-p PORT\tQGroundControl port to use (default: %i)\n",target_port);
    printf("-l PORT\tlocal port to use\n");
}

int set_defaults(int c, char **a) {
	int required = 2;
    int option;
    while ((option = getopt(c, a,"ht:p:l:")) != -1) {
        switch (option)  {
            case 't': strcpy(target_ip,optarg); required--; break;
            case 'p': target_port = atoi(optarg); break;
            case 'l': local_port = atoi(optarg); required--; break;
            default: print_usage(); return -1;
        }
    }
   	
   	if (required) {
   		print_usage();
   		return -1;
   	}

   	return 0;
}

void catch_signal(int sig)
{
        stop = 1;
}

int main(int argc, char* argv[])
{
	int ret;
	signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    dbg_init(0); //0b11111111 init the mw library debug

    if (set_defaults(argc,argv)) {
    	return -1;
    }

#ifdef CFG_ENABLED
	printf("Opening config: %s...\n",CFG_FILE);

	if (access(CFG_FILE,F_OK)) {
		printf("Not found. Creating...\n");
		ret = creat(CFG_FILE, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
		if (ret>0) {
			printf("Success.\n");
			close(ret);
		}
	}

  	if(!config_read_file(&cfg, CFG_FILE))
  	{
  		printf("Unable to open.\n");
    	fprintf(stderr, "%s:%d - %s\n", config_error_file(&cfg),
            config_error_line(&cfg), config_error_text(&cfg));
    		config_destroy(&cfg);
    	return(EXIT_FAILURE);
  	}	
#endif

    printf("Initializing UDP...\n");
 	udp_init(target_ip,target_port,local_port);

 	printf("Setting up mw...\n");
 	if (mw_init()) {
 		printf("Error!\n");
 		return -1;
 	}

  	printf("Setting up mavlink...\n");
 	if (mavlink_init()) {
  		printf("Error!\n");
 		return -1;		 
 	}	

 	printf("Started.\n");
 	loop();
 	
 	printf("Cleaning up...\n");
 	mavlink_end();
 	mw_end();

 	udp_close();

#ifdef CFG_ENABLED
 	if (!config_write_file(&cfg,CFG_FILE)) 
 		printf("Error while writing config file. Settings won't be stored.\n");
 	config_destroy(&cfg);
#endif 	

 	printf("Bye.\n");
 	return 0;
}
 

