#include "mavlink.h"
#include "def.h"
#include "global.h"
#include "params.h"
#include <stdio.h>
#include <stdlib.h>
#include "gamepad.h"
#include <sys/time.h>

static uint8_t debug = 0;
static uint16_t manual_control_counter = 0;
static uint64_t current_time; //ms

typedef uint8_t (*t_cb_i)(uint8_t);
t_cb_i loop_callback; //we use loop_callback to send certain messages with a delay

static mavlink_message_t mav_msg;


void msg_sys_status();
void msg_radio_status();
void msg_heartbeat();
void msg_gps_raw_int();
void msg_altitude();
void msg_global_position_int();
void msg_attitude_quaternion();
void msg_home_position();

typedef void (*t_cb)();

struct _S_TASK {
	uint16_t freq; //has to be less than loop_counter max value
	t_cb cb_fn;
};
typedef struct _S_TASK S_TASK;

#define MAX_TASK 7

static S_TASK task[MAX_TASK] = {
	{4, msg_attitude_quaternion}, //run every LOOP_MS (see global.h)
	{40, msg_gps_raw_int},
	{20, msg_global_position_int},
	{40, msg_heartbeat},
	{40, msg_sys_status},
	{40, msg_radio_status},	
	{80, msg_home_position}
};

uint64_t microsSinceEpoch()
{
	struct timeval tv;
	uint64_t micros = 0;
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	return micros;
}

void mavlink_loop() {
	static uint8_t counter = 0;
	uint8_t i;

	current_time = microsSinceEpoch();

	for (i=0;i<MAX_TASK;i++)
		if (counter%task[i].freq==0) {
			task[i].cb_fn();
		}

	//callbacks etc
	if (loop_callback) if (loop_callback(0)) loop_callback = NULL;

	counter++;
	if (counter==100) counter=0;
}

uint8_t mavlink_init() {
	return 0;
}

void mavlink_end() {
}

void mav_cmd_arm_disarm(mavlink_message_t *msg) {
	//arm disarm via box
	float p = mavlink_msg_command_long_get_param1(msg);

	if (p==1.f) mw_arm();
	else mw_disarm();
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

void msg_param_set(mavlink_message_t *msg) {
	//it takes 2 set messages to set it
	//on the first set we send a set message to mw and request a refresh
	//on the second set - we just check if the value was refreshed to the value that is being set
	//firstly check if the value we have is different

	uint8_t component;
	char name[16+1];
	float value;

	component = mavlink_msg_param_set_get_target_component(msg);
	mavlink_msg_param_set_get_param_id(msg,name); //get name from the param
	value = mavlink_msg_param_set_get_param_value(msg);

	printf("Set id: %s\n",name);

	params_set(component,name,value);
}

void msg_param_request_read(mavlink_message_t *msg) {
	int16_t idx;
	uint8_t component;
	component = mavlink_msg_param_request_read_get_target_component(msg);
	idx = mavlink_msg_param_request_read_get_param_index(msg);

	printf("Requesting param id: %i, component: %u\n",idx,component);
	params_send(component,idx); //TODO: the idx has to be global, but idx is local?
}

void msg_param_request_list(mavlink_message_t *msg) {
	loop_callback = params_get_all;
	params_get_all(1);
}

uint8_t msg_mission_request_list(mavlink_message_t *msg) {
	if (debug) printf("-> mission_count\n");
	mavlink_msg_mission_count_pack(1,200,&mav_msg,1,200,0);

	dispatch(&mav_msg);

	return 0;
}

void msg_altitude() {
	int32_t alt;

	mw_altitude(&alt);

	mavlink_msg_altitude_pack(1,200, &mav_msg,
		current_time,
		0.f, //monotonic
		0.f, //amsl
		0.f, //local
		alt, //relative 
		0.f, //terrain
		0.f //clearance
	);

	dispatch(&mav_msg);
}

void msg_home_position() {
	static uint8_t position_set = 1;
	if (mw_state()==MAV_STATE_STANDBY) position_set = 0; //set home_position only in standby mode

	if (position_set) return;

	int32_t lat = 0;
	int32_t lon = 0;
	int32_t alt = 0;

	float dummy[4];

	mw_get_homepos(&lat,&lon,&alt);

	mavlink_msg_home_position_pack(1,200, &mav_msg,
		lat,lon, alt*10.f, 0.f, 0.f, 0.f, dummy, 0.f, 0.f, 0.f
	);
}


void msg_global_position_int() {
	int32_t lat = 0;
	int32_t lon = 0;
	int32_t alt = 0;
	int32_t ralt = 0;

	mw_raw_gps(NULL, &lat, &lon, &alt, NULL, NULL, NULL);
	mw_altitude(&ralt);

	mavlink_msg_global_position_int_pack(1,200, &mav_msg,
		current_time,
		lat,lon, alt*10.f, ralt*10.f, 0.f, 0.f, 0.f, 0.f
	);

	dispatch(&mav_msg);
}

void msg_gps_raw_int() {
	uint8_t fix = 0;
	int32_t lat = 0;
	int32_t lon = 0;
	int32_t alt = 0;
	uint16_t eph = UINT16_MAX;
	uint16_t epv = UINT16_MAX;
	uint16_t vel = 0;
	uint16_t cog = 0;
	uint8_t satellites_visible = 0;

	mw_raw_gps(&fix, &lat, &lon, &alt, &vel, &cog, &satellites_visible);

	mavlink_msg_gps_raw_int_pack(1,200, &mav_msg,
		current_time,
		fix,lat,lon, alt*10.f, eph, epv, vel, cog, satellites_visible
	);

	dispatch(&mav_msg);
}

void msg_radio_status() {
	int8_t rssi,noise;

	mw_get_signal(&rssi,&noise);

	//printf("RSTATUS\n");
	mavlink_msg_radio_status_pack(1,200, &mav_msg,
		rssi, //rssi
		-1, //remrssi
		0, //txbuf
		noise, //noise
		0, //remnoise
		0, //rxerrors
		0 //fixed
	);
	dispatch(&mav_msg);
}

void msg_sys_status() {
	static uint64_t prev_time = 0;

	uint8_t dt_ms = (current_time - prev_time)/1000;
	prev_time = current_time;



	mavlink_msg_sys_status_pack(1, 200, &mav_msg,
		mw_sys_status_sensors(), //present sensors
		mw_sys_status_sensors(), //active sensors (assume all present are active for MW) //could use 0xFFFFFFFF ?
		mw_sys_status_sensors(), //error sensors (assume all are ok for MW) //could use 0xFFFFFFFF ?
		500, //load 50%
		11000, //voltage 11V
		-1, //current
		-1, //remaining
		0,//mav_drop_rate(), //drop rate
		0,//mav_drop_count(), //comm error count
		mw_get_i2c_drop_count(), //errors_count1
		manual_control_counter, //errors_count2
		dt_ms/1000, //errors_count3
		0 //errors_count4
	);
	dispatch(&mav_msg);	

	manual_control_counter = 0;
}

void msg_heartbeat() {

	mavlink_msg_heartbeat_pack(1, 200, &mav_msg, 
		mw_type(),
		MAV_AUTOPILOT_GENERIC,
		mw_mode_flag(),
		0,
		mw_state()
	);
	dispatch(&mav_msg);
}

void msg_attitude_quaternion() {
	float w,x,y,z;

	mw_attitude_quaternions(&w, &x, &y, &z);

	mavlink_msg_attitude_quaternion_pack(1,200, &mav_msg,
		current_time,
		w, x, y, z, 0.f, 0.f,0.f
	);

	dispatch(&mav_msg);
}

void msg_manual_control(mavlink_message_t *msg) {
	static uint16_t old_btn = 0;

	manual_control_counter++;

	uint16_t btn;
	uint16_t i;
	uint8_t btn_mapping;

	int16_t t,y,p,r;
	//TODO: check if msg timestamp is correct (>prev_timestamp)
	t = mavlink_msg_manual_control_get_z(msg);
	y = mavlink_msg_manual_control_get_r(msg);
	p = mavlink_msg_manual_control_get_x(msg);
	r = mavlink_msg_manual_control_get_y(msg);

	btn=mavlink_msg_manual_control_get_buttons(msg);

	if (btn!=old_btn) {

		for (i=0;i<mw_box_count();i++) { //check for box buttons
			gamepad_get_mapping(&btn_mapping,i);
			if (get_bit(old_btn,btn_mapping) && (get_bit(btn,btn_mapping)==0)) { mw_toggle_box(i); }
		}

		//check for throttle low button
		gamepad_get_mapping(&btn_mapping,i);

		if (get_bit(old_btn,btn_mapping) && (get_bit(btn,btn_mapping)==0)) { gamepad_control_reset_throttle(); } 	

		old_btn = btn;
	}

	gamepad_control_calculate(&t,&y,&p,&r);

	mw_manual_control(t,y,p,r);

}



