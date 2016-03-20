
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <signal.h>

#include <mw/shm.h>
#include "mw_mav.h"
#include "udp.h"

//defines list of messages that should be automatically send
const uint8_t active_mav_msg[] = {
	MAVLINK_MSG_ID_HEARTBEAT,
	MAVLINK_MSG_ID_SYS_STATUS,
	MAVLINK_MSG_ID_GPS_RAW_INT,
	MAVLINK_MSG_ID_ATTITUDE_QUATERNION
};

static uint8_t is_msg_active(uint8_t msg_id) {
	uint8_t i;
	for (i=0;i<sizeof(active_mav_msg);i++)
		if (active_mav_msg[i] == msg_id) return 1;

	return 0;
}

uint8_t debug = 1;

uint8_t stop = 0;
uint16_t loop_counter = 0;

#define LOOP_MS 25

enum e_mode { //loop activity
	MODE_DEFAULT,
	MODE_PARAM_REQUEST_LIST
};

uint8_t mode = MODE_DEFAULT; //operation mode for the main loop

int16_t rc_throttle,rc_yaw,rc_pitch,rc_roll;
uint8_t manual_control_timeout = 0;

typedef void (*t_cb)();

struct _S_TASK {
	uint16_t freq; //has to be less than loop_counter max value
	t_cb cb_fn;
};

typedef struct _S_TASK S_TASK;

void check_incoming_udp(); //checks for messages on UDP port
//note: we do not have to actively check for messages on MW side as we have continues access to them through SHM
void mw_keepalive();
void heartbeat();
void sys_status();
void every_50ms();
void attitude_quaternion();
void gps_raw_int();
void manual_control();

void dispatch(mavlink_message_t *mavlink_msg);

#define MAX_TASK 8
S_TASK task[MAX_TASK] = {
	{1, check_incoming_udp},
	{1, manual_control},
	{2, every_50ms},
	{4, attitude_quaternion}, //every 100ms
	{40, gps_raw_int},
	{40, mw_keepalive}, //uses MW_TIMEOUT hence to run every second
	{40, heartbeat},
	{40, sys_status}
};

mavlink_message_t mav_msg;

void mssleep(unsigned int ms);
uint64_t microsSinceEpoch();

void sys_status() {
	if (!is_msg_active(MAVLINK_MSG_ID_SYS_STATUS)) return;
	mavlink_msg_sys_status_pack(1, 200, &mav_msg,
		mav_sys_status_sensors(), //present sensors
		mav_sys_status_sensors(), //active sensors (assume all present are active for MW) //could use 0xFFFFFFFF ?
		mav_sys_status_sensors(), //error sensors (assume all are ok for MW) //could use 0xFFFFFFFF ?
		500, //load 50%
		11000, //voltage 11V
		-1, //current
		-1, //remaining
		get_comm_drop_rate(), //drop rate
		get_comm_drop_count(), //comm error count
		0, 
		0, 
		0, 
		0
	);
	dispatch(&mav_msg);	
}

void heartbeat() {
	if (!is_msg_active(MAVLINK_MSG_ID_HEARTBEAT)) return;
	mavlink_msg_heartbeat_pack(1, 200, &mav_msg, 
		mav_type(),
		MAV_AUTOPILOT_GENERIC,
		mav_mode_flag(),
		0,
		mav_state()
	);
	dispatch(&mav_msg);
}

void attitude_quaternion() {
	if (!is_msg_active(MAVLINK_MSG_ID_ATTITUDE_QUATERNION)) return;
	float w,x,y,z;
	mav_attitude_refresh();

	mav_attitude_quaternions(&w, &x, &y, &z);

	mavlink_msg_attitude_quaternion_pack(1,200, &mav_msg,
		microsSinceEpoch(),
		w, x, y, z, 0.f, 0.f,0.f
	);

	dispatch(&mav_msg);
}

void gps_raw_int() {
	if (!is_msg_active(MAVLINK_MSG_ID_GPS_RAW_INT)) return;
	uint8_t fix = 0;
	int32_t lat = 0;
	int32_t lon = 0;
	int32_t alt = 0;
	uint16_t eph = UINT16_MAX;
	uint16_t epv = UINT16_MAX;
	uint16_t vel = 0;
	uint16_t cog = 0;
	uint8_t satellites_visible = 0;

	mav_gps_refresh();

	mav_raw_gps(&fix, &lat, &lon, &alt, &vel, &cog, &satellites_visible);

	mavlink_msg_gps_raw_int_pack(1,200, &mav_msg,
		microsSinceEpoch(),
		fix,lat,lon, alt, eph, epv, vel, cog, satellites_visible
	);

	dispatch(&mav_msg);
}

uint8_t send_param(uint8_t p) {
	//we can only send params if they are refreshed
	if (mav_param_refresh(0)==0) {
		if (debug) printf("Awaiting params to refresh...\n");
		return 0;
	}
	if (debug) printf("Sending message param_value id: %u name: %s value: %u\n",p,mav_get_param_name(p),mav_get_param_value(p));
	mavlink_param_union_t param;
	param.param_uint8 = mav_get_param_value(p); //value for the param
	param.type = 	MAV_PARAM_TYPE_UINT8;

	mavlink_msg_param_value_pack(1,200,&mav_msg,mav_get_param_name(p),param.param_float, param.type,mav_param_count(),p);

	dispatch(&mav_msg);
	return 1;
}

uint8_t do_param_request_list() {
	static uint8_t state = 0;

	if (state==mav_param_count()) {
		state = 0;
		return 0; //all params have been sent
	}

	if (send_param(state)) state++;

	return 1;
}


void msg_param_request_list(mavlink_message_t *msg) {
	mav_param_refresh(1); //request pids from mw
	mode = MODE_PARAM_REQUEST_LIST;	//set out mode for awaiting for params
}

uint8_t msg_mission_request_list(mavlink_message_t *msg) {
	if (debug) printf("Sending message mission_count\n");
	mavlink_msg_mission_count_pack(1,200,&mav_msg,1,200,0);

	dispatch(&mav_msg);

	return 0;
}

void every_50ms() {
	switch (mode) {
		case MODE_PARAM_REQUEST_LIST: if (do_param_request_list()==0) mode=MODE_DEFAULT; //we handle one param at a time not to congest the link (as per docs)
			break;
		default:
			break;
	}
}

void msg_param_request_read(mavlink_message_t *msg) {
	int16_t p;
	p = mavlink_msg_param_request_read_get_param_index(&mav_msg);
	send_param(p);
}

void msg_param_set(mavlink_message_t *msg) {
	//it takes 2 set messages to set it
	//on the first set we send a set message to mw and request a refresh
	//on the second set - we just check if the value was refreshed to the value that is being set
	//firstly check if the value we have is different
	char name[16+1];
	uint8_t id;
	mavlink_param_union_t param;

	mavlink_msg_param_set_get_param_id(&mav_msg,name); //get param name from the message

	id = mav_get_param_id(name); //resolve the name into id

	param.param_float = mavlink_msg_param_set_get_param_value(&mav_msg);
	//if (debug) printf("Got message param_set id: %u name: %s value: %u\n",id,name,param.param_uint8);

	//if (debug) printf("Current value: %u\n",mav_get_param_value(id));
	
	if (param.param_uint8==mav_get_param_value(id)) {
		//if (debug) printf("Value has not changed.\n");
		send_param(id);
		return;
	}

	mav_set_param(id,param.param_uint8); 

	//if (debug) printf("Refreshing params...\n");
	mav_param_refresh(1);
}

void mav_cmd_arm_disarm(mavlink_message_t *msg) {
	//arm disarm via box
	float p = mavlink_msg_command_long_get_param1(msg);

	if (p==1.f) mav_arm();
	else mav_disarm();
}

void printBits(unsigned int num)
{
		int bit;
   for(bit=0;bit<(sizeof(unsigned int) * 8); bit++)
   {
      printf("%i ", num & 0x01);
      num = num >> 1;
   }
}

void msg_manual_control(mavlink_message_t *msg) {
	static uint16_t old_btn = 0;
	uint16_t btn;

	manual_control_timeout = 60; //1.5sec timeout for manual_control (see main loop for manual_control handling)

	rc_pitch=mavlink_msg_manual_control_get_x(msg); //pitch [-1000,1000], back, front
	rc_roll=mavlink_msg_manual_control_get_y(msg); //roll [-1000,1000], left, right
	rc_throttle=mavlink_msg_manual_control_get_z(msg); //throttle [0-1000]
	rc_yaw=mavlink_msg_manual_control_get_r(msg); //yaw [-1000,1000], left, right
	btn=mavlink_msg_manual_control_get_buttons(msg);

	if (btn==old_btn) return;

	if (get_bit(old_btn,11) && (get_bit(btn,11)==0)) { mav_toggle_pos_hold(); } //r1
	if (get_bit(old_btn,9) && (get_bit(btn,9)==0)) { mav_toggle_pos_home(); } //r2
	if (get_bit(old_btn,12) && (get_bit(btn,12)==0)) { mav_toggle_horizon(); } //triangle
	if (get_bit(old_btn,14) && (get_bit(btn,14)==0)) { mav_toggle_baro(); } //x
	if (get_bit(old_btn,13) && (get_bit(btn,13)==0)) { mav_toggle_mag(); } //circle
	if (get_bit(old_btn,15) && (get_bit(btn,15)==0)) { mav_toggle_land(); } //square
	old_btn = btn;
}

void manual_control() {
	//this is run from a loop
	if (!manual_control_timeout) return; //dont feed manual_control if we have not received them for a while
	manual_control_timeout--;

	mav_rc(rc_throttle,rc_yaw,rc_pitch,rc_roll);
}


void msg_command_long(mavlink_message_t *msg) {
	uint16_t cmd;
	cmd = mavlink_msg_command_long_get_command(msg);

	switch (cmd) {
		case MAV_CMD_COMPONENT_ARM_DISARM:
			mav_cmd_arm_disarm(msg);
			break;
		default: printf("Unknown mav_cmd: %u\n",cmd);
	}
	printf("mav_cmd: %u\n",cmd);
}

void check_incoming_udp() {

	while (udp_recv(&mav_msg)) {
		switch (mav_msg.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT: break;

			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				if (debug) printf("PARAM_REQUEST_LIST\n");
				msg_param_request_list(&mav_msg);
				break;
			case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
				if (debug) printf("PARAM_REQUEST_READ\n");
				msg_param_request_read(&mav_msg);
				break;
			case MAVLINK_MSG_ID_PARAM_SET:
				if (debug) printf("PARAM_SET\n");
				msg_param_set(&mav_msg);
				break;				
			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
				if (debug) printf("MISSION_REQUEST_LIST\n");
				msg_mission_request_list(&mav_msg);
				break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
				if (debug) printf("COMMAND_LONG\n");
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

void dispatch(mavlink_message_t *mavlink_msg) {
	udp_send(mavlink_msg);
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


//runs all tasks as per defines frequency
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

uint64_t microsSinceEpoch()
{
 
	struct timeval tv;
 
	uint64_t micros = 0;
 
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
 
	return micros;
}

void catch_signal(int sig)
{
        stop = 1;
}

int main(int argc, char* argv[])
{

	signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    dbg_init(0); //0b11111111 

    if (set_defaults(argc,argv)) {
    	return -1;
    }

    if (debug) printf("Initializing UDP...\n");
 	udp_init(target_ip,target_port,local_port);

 	if (debug) printf("Running shm_client_init...\n");
 	if (shm_client_init()) {//initiate channel to mw-service
 		printf("Error!");
 		return -1; 
 	}

 	if (debug) printf("Retrieveing initial settings...\n");
 	mav_retrieve_init();

 	if (debug) printf("Started.\n");
 	loop();

 	if (debug) printf("Cleaning up...\n");
 	shm_client_end(); //close channel to mw-service

 	udp_close();

 	if (debug) printf("Bye.\n");
 	return 0;
}
 

