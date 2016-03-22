#include "messages.h"
#include <stdio.h>
#include <stdlib.h>
#include "def.h"

static uint8_t debug = 1;
static uint8_t loop_counter = 0;

typedef void (*t_cb)(uint8_t);
t_cb loop_callback; //we use loop_callback to send certain messages with a delay

static mavlink_message_t mav_msg;

uint8_t param_pid_count();
void param_pid_set(char *name, float value);
void param_pid_send(uint8_t idx);

uint8_t param_manual_control_count();
void param_manual_control_set(char *name, float value);
void param_manual_control_send(uint8_t idx);

static uint8_t run_every_ms(uint16_t ms) {
	if (loop_counter%(ms/LOOP_MS)==0) return 1;
	return 0;
}

//defines list of active messages
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

void dispatch(mavlink_message_t *mavlink_msg) {
	udp_send(mavlink_msg);
}

void msg_loop() {

	//send default messages
	if (run_every_ms(100)) msg_attitude_quaternion();
	//if (counter%4==0)  //every 100ms
	if (loop_counter%40==0) msg_gps_raw_int(); //every sec
	if (loop_counter%40==0) msg_heartbeat(); //every sec
	if (loop_counter%40==0) msg_sys_status(); //every sec

	//callbacks etc
	if (loop_callback) loop_callback(0);

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

uint8_t param_pid_count() {
	return mw_pid_count();
}

void param_pid_set(char *name, float value) {
	uint8_t id;	
	mavlink_param_union_t param;
	param.param_float = value;

	id = mw_get_pid_id(name); //resolve the name into id
	//if (debug) printf("Got message param_set id: %u name: %s value: %u\n",id,name,param.param_uint8);

	//if (debug) printf("Current value: %u\n",mav_get_param_value(id));
	
	if (param.param_uint8==mw_get_pid_value(id)) { //confirm with send only if we are sure that it is saved
		//if (debug) printf("Value has not changed.\n");
		param_pid_send(id);
		return;
	}

	mw_set_pid(id,param.param_uint8); 


	//if (debug) printf("Refreshing params...\n");
	mw_pid_refresh(1);
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

	switch (component) {
		case 1: param_manual_control_set(name,value); break;
		case 200: param_pid_set(name,value); break;
		default: printf("Unknown component: %i\n",component); break;
	}

}

void msg_param_request_read(mavlink_message_t *msg) {
	int16_t idx;
	uint8_t component;
	component = mavlink_msg_param_request_read_get_target_component(msg);
	idx = mavlink_msg_param_request_read_get_param_index(msg);
	switch (component) {
		case 1: param_manual_control_send(idx); break;
		case 200: param_pid_send(idx); break;
	}
}

void msg_param_request_list_loop(uint8_t reset) {
	static uint8_t step = 0;
	uint8_t param_count,i;

	if (reset) {
		mw_pid_refresh(1);
		step=0;
		loop_callback = msg_param_request_list_loop;
	}

	switch (step) {
		case 0: //wait for pids
			if (mw_pid_refresh(0)) step++;
			break;
		case 1: //send params
			param_count = param_pid_count();
			for (i=0;i<param_count;i++)
				param_pid_send(i);
			step++;
			break;
		case 2: //send params
			param_count = param_manual_control_count();
			for (i=0;i<param_count;i++)
				param_manual_control_send(i);
			loop_callback = NULL;
			step = 0;
			break;
	}
}

void msg_param_request_list(mavlink_message_t *msg) {
	msg_param_request_list_loop(1);
}


uint8_t msg_mission_request_list(mavlink_message_t *msg) {
	if (debug) printf("-> mission_count\n");
	mavlink_msg_mission_count_pack(1,200,&mav_msg,1,200,0);

	dispatch(&mav_msg);

	return 0;
}

void param_pid_send(uint8_t idx) {
	//send all pids under COMPONENT 200
	if (debug) printf("-> param_value id: %u name: %s value: %u\n",idx,mw_get_pid_name(idx),mw_get_pid_value(idx));
	mavlink_param_union_t param;
	param.param_uint8 = mw_get_pid_value(idx); //value for the param
	param.type = 	MAV_PARAM_TYPE_UINT8;

	mavlink_msg_param_value_pack(1,200,&mav_msg,mw_get_pid_name(idx),param.param_float, param.type,mw_pid_count(),idx);

	dispatch(&mav_msg);
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

/*
	MANUAL CONTROL handling and params
*/
//static uint8_t box_btn_mapping[CHECKBOXITEMS];

void msg_manual_control(mavlink_message_t *msg) {
	static uint16_t old_btn = 0;
	uint16_t btn;

	mw_manual_control(
		mavlink_msg_manual_control_get_z(msg), 
		mavlink_msg_manual_control_get_r(msg), 
		mavlink_msg_manual_control_get_x(msg), 
		mavlink_msg_manual_control_get_y(msg)
	);

	btn=mavlink_msg_manual_control_get_buttons(msg);

	if (btn==old_btn) return;

	if (get_bit(old_btn,11) && (get_bit(btn,11)==0)) { mw_toggle_pos_hold(); } //r1
	if (get_bit(old_btn,9) && (get_bit(btn,9)==0)) { mw_toggle_pos_home(); } //r2
	if (get_bit(old_btn,12) && (get_bit(btn,12)==0)) { mw_toggle_horizon(); } //triangle
	if (get_bit(old_btn,14) && (get_bit(btn,14)==0)) { mw_toggle_baro(); } //x
	if (get_bit(old_btn,13) && (get_bit(btn,13)==0)) { mw_toggle_mag(); } //circle
	if (get_bit(old_btn,15) && (get_bit(btn,15)==0)) { mw_toggle_land(); } //square
	old_btn = btn;
}

uint8_t param_manual_control_count() {
	return 0;
	//return mw_box_count();
}

void param_manual_control_send(uint8_t idx) {
	/*
	//send all manual control parameters
	//a manual control parameter is equivalent to box

	if (debug) printf("-> param_value id: %u name: %s value: %u\n",p,mw_get_box_name(p),butt);
	mavlink_param_union_t param;
	param.param_uint8 = mw_get_pid_value(p); //value for the param
	param.type = 	MAV_PARAM_TYPE_UINT8;

	mavlink_msg_param_value_pack(1,200,&mav_msg,mw_get_pid_name(p),param.param_float, param.type,mw_pid_count(),p);

	dispatch(&mav_msg);
	*/
}

void param_manual_control_set(char *name, float value) {

}


