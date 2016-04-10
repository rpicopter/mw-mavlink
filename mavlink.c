#include "mavlink.h"
#include "def.h"
#include "global.h"
#include "params.h"
#include <stdio.h>
#include <stdlib.h>
#include "gamepad.h"
#include <sys/time.h>

static uint8_t debug = 0;

static uint8_t loop_counter = 0;

typedef uint8_t (*t_cb)(uint8_t);
t_cb loop_callback; //we use loop_callback to send certain messages with a delay

static mavlink_message_t mav_msg;


//defines list of active messages
const uint8_t active_mav_msg[] = {
	MAVLINK_MSG_ID_HEARTBEAT,
	MAVLINK_MSG_ID_SYS_STATUS,
	MAVLINK_MSG_ID_RADIO_STATUS,
	MAVLINK_MSG_ID_GPS_RAW_INT,
	MAVLINK_MSG_ID_ATTITUDE_QUATERNION
};

static uint8_t is_msg_active(uint8_t msg_id) {
	uint8_t i;
	for (i=0;i<sizeof(active_mav_msg);i++)
		if (active_mav_msg[i] == msg_id) return 1;

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

void printBits(unsigned int num)
{
		int bit;
   for(bit=0;bit<(sizeof(unsigned int) * 8); bit++)
   {
      printf("%i ", num & 0x01);
      num = num >> 1;
   }
}

uint8_t mavlink_init() {
	params_init();
	return 0;
}

void mavlink_end() {
	params_end();
}

void mavlink_loop() {

	//send default messages
	if (loop_counter%4==0) msg_attitude_quaternion();  //every 100ms
	if (loop_counter%40==0) msg_gps_raw_int(); //every sec
	if (loop_counter%40==0) msg_heartbeat(); //every sec
	if (loop_counter%40==0) msg_sys_status(); //every sec
	if (loop_counter%40==0) msg_radio_status(); //every sec

	//callbacks etc
	if (loop_callback) if (loop_callback(0)) loop_callback = NULL;

	loop_counter++;
	if (loop_counter==100) loop_counter=0;
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
	mavlink_msg_param_set_get_param_id(msg,name); //get pid name from the message
	value = mavlink_msg_param_set_get_param_value(msg);

	printf("Set id: %s\n",name);

	params_set(component,name,value);
}

void msg_param_request_read(mavlink_message_t *msg) {
	int16_t idx;
	uint8_t component;
	component = mavlink_msg_param_request_read_get_target_component(msg);
	idx = mavlink_msg_param_request_read_get_param_index(msg);

	params_send(component,idx);
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

void msg_gps_raw_int() {
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

	mw_gps_refresh();

	mw_raw_gps(&fix, &lat, &lon, &alt, &vel, &cog, &satellites_visible);

	mavlink_msg_gps_raw_int_pack(1,200, &mav_msg,
		microsSinceEpoch(),
		fix,lat,lon, alt, eph, epv, vel, cog, satellites_visible
	);

	dispatch(&mav_msg);
}

void msg_radio_status() {
	if (!is_msg_active(MAVLINK_MSG_ID_RADIO_STATUS)) return;

	//printf("RSTATUS\n");
	mavlink_msg_radio_status_pack(1,200, &mav_msg,
		-50-127, //rssi
		-50-127, //remrssi
		1, //txbuf
		-71-127, //noise
		-71-127, //remnoise
		0, //rxerrors
		1 //fixed
	);
	dispatch(&mav_msg);
}

void msg_sys_status() {
	if (!is_msg_active(MAVLINK_MSG_ID_SYS_STATUS)) return;
	mavlink_msg_sys_status_pack(1, 200, &mav_msg,
		mw_sys_status_sensors(), //present sensors
		mw_sys_status_sensors(), //active sensors (assume all present are active for MW) //could use 0xFFFFFFFF ?
		mw_sys_status_sensors(), //error sensors (assume all are ok for MW) //could use 0xFFFFFFFF ?
		500, //load 50%
		11000, //voltage 11V
		-1, //current
		-1, //remaining
		mw_get_comm_drop_rate(), //drop rate
		mw_get_comm_drop_count(), //comm error count
		0, 
		0, 
		0, 
		0
	);
	dispatch(&mav_msg);	
}

void msg_heartbeat() {
	if (!is_msg_active(MAVLINK_MSG_ID_HEARTBEAT)) return;
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
	if (!is_msg_active(MAVLINK_MSG_ID_ATTITUDE_QUATERNION)) return;
	float w,x,y,z;
	mw_attitude_refresh();

	mw_attitude_quaternions(&w, &x, &y, &z);

	mavlink_msg_attitude_quaternion_pack(1,200, &mav_msg,
		microsSinceEpoch(),
		w, x, y, z, 0.f, 0.f,0.f
	);

	dispatch(&mav_msg);
}

void msg_manual_control(mavlink_message_t *msg) {
	static uint16_t old_btn = 0;
	uint16_t btn;
	uint16_t i;
	uint8_t btn_mapping;

	int16_t t,y,p,r;

	t = mavlink_msg_manual_control_get_z(msg);
	y = mavlink_msg_manual_control_get_r(msg);
	p = mavlink_msg_manual_control_get_x(msg);
	r = mavlink_msg_manual_control_get_y(msg);

	btn=mavlink_msg_manual_control_get_buttons(msg);

	if (btn!=old_btn) {

		for (i=0;i<mw_box_count();i++) { //check for box buttons
			btn_mapping = gamepad_get_mapping(i);
			if (get_bit(old_btn,btn_mapping) && (get_bit(btn,btn_mapping)==0)) { mw_toggle_box(i); }
		}

		//check for throttle low button
		btn_mapping = gamepad_get_mapping(i);

		if (get_bit(old_btn,btn_mapping) && (get_bit(btn,btn_mapping)==0)) { gamepad_control_reset_throttle(); } 	

		old_btn = btn;
	}

	gamepad_control_calculate(&t,&y,&p,&r);

	mw_manual_control(t,y,p,r);

}



