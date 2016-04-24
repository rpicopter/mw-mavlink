
#include "mw.h"
#include <mw/shm.h>
#include <stdio.h>
#include <math.h>

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include "mavlink/common/mavlink.h"

#define MW_TIMEOUT 3 //3s timeout for mw
static uint8_t mw_status=0; //0-all ok; 1-no connection?

static struct S_MSG mw_msg;
static struct S_MSP_BOXCONFIG boxconf;
static struct S_MSP_RC rc = {.throttle=1000,.yaw=1500,.pitch=1500,.roll=1500,.aux1=1500,.aux2=1500,.aux3=1500,.aux4=1500};

#define RC_TIMEOUT 60 //1.5sec timeout for manual_control (see main loop for manual_control handling)
static uint8_t rc_count;


void mw_keepalive();
void mw_feed_rc();

uint8_t mw_init() {
 	if (shm_client_init()) return -1; 

	//this retrievs the initial set of settings from MW like boxconfiguration, etc
	uint8_t filter;

	rc_count = 0;
	//ident
	filter = MSP_IDENT;
	shm_scan_incoming_f(&mw_msg,&filter,1); //invalidate
	while (1) {
		mspmsg_IDENT_serialize(&mw_msg,NULL);
		shm_put_outgoing(&mw_msg);
		mssleep(50);
		if (shm_scan_incoming_f(&mw_msg,&filter,1)) break; //got the response
	} 

	//status
	filter = MSP_STATUS;
	shm_scan_incoming_f(&mw_msg,&filter,1); //invalidate
	while (1) {
		mspmsg_STATUS_serialize(&mw_msg,NULL);
		shm_put_outgoing(&mw_msg);
		mssleep(50);
		if (shm_scan_incoming_f(&mw_msg,&filter,1)) break; //got the response
	} 

	//boxids
	filter = MSP_BOXIDS;
	shm_scan_incoming_f(&mw_msg,&filter,1); //invalidate
	while (1) {
		mspmsg_BOXIDS_serialize(&mw_msg,NULL);
		shm_put_outgoing(&mw_msg);
		mssleep(50);
		if (shm_scan_incoming_f(&mw_msg,&filter,1)) {
			mspmsg_BOXIDS_parse(&boxconf,&mw_msg);
			break; //got the response
		}
	} 	

	mspmsg_BOX_serialize(&mw_msg,NULL);
	shm_put_outgoing(&mw_msg);	

	return 0;
}

void mw_end() {
 	shm_client_end(); //close channel to mw-service	
}

void mw_keepalive() {
	//keep alive for MultiWii and the service
	static uint8_t err_counter = 0; //number of missed status messages
	uint8_t filter;

	mspmsg_LOCALSTATUS_serialize(&mw_msg,NULL);
	shm_put_outgoing(&mw_msg);	

	mspmsg_STATUS_serialize(&mw_msg,NULL);
	shm_put_outgoing(&mw_msg);

	mspmsg_BOX_serialize(&mw_msg,NULL);
	shm_put_outgoing(&mw_msg);	

	filter = MSP_STATUS;
	if (!shm_scan_incoming_f(&mw_msg,&filter,1)) err_counter++;
	else {
		err_counter = 0;
		mw_status = 0;
	}

	if (err_counter>MW_TIMEOUT) {
		mw_status = 1;
	}
}


void mw_feed_rc() {
	//this is run from a loop
	if (rc_count==0) return; //dont feed manual_control if we have not received them for a while
	rc_count--;

	//printf("Throttle: %i\n",rc.throttle);

	mspmsg_SET_RAW_RC_serialize(&mw_msg,&rc);
	shm_put_outgoing(&mw_msg);
}

void mw_loop() { //
	static uint8_t counter = 0;

	//mw_keepalive every second
	if (counter%(1000/25)==0) mw_keepalive(); //every 1000ms but the loop_ms is 25

	if (counter%1==0) mw_feed_rc(); //ever 25ms



	counter++;
	if (counter==100) counter=0;

}

void mw_arm() {
	//TODO
	//boxconf.active[BOXARM] = toggleBox(boxconf.active[BOXARM]);
	//mspmsg_SET_BOX_serialize(&msg,&boxconf);
	//shm_put_outgoing(&msg);	
}

void mw_disarm() {
	//TODO
}

uint16_t mw_get_comm_drop_count() {
	//this get count of errors on MW->MW_SERVICE link only
	struct S_MSP_LOCALSTATUS lstatus;
	
	shm_get_incoming(&mw_msg,MSP_LOCALSTATUS);
	mspmsg_LOCALSTATUS_parse(&lstatus,&mw_msg);	

	return lstatus.crc_error_count;
}

uint16_t mw_get_comm_drop_rate() {
	//this get count of errors on MW->MW_SERVICE link only
	struct S_MSP_LOCALSTATUS lstatus;
	
	shm_get_incoming(&mw_msg,MSP_LOCALSTATUS);
	mspmsg_LOCALSTATUS_parse(&lstatus,&mw_msg);	

	if (!lstatus.rx_count) return 0;
	return (lstatus.crc_error_count/lstatus.rx_count)*10000; //100%=10000
}

void mw_manual_control(int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll) {

	rc.throttle = throttle;
	rc.yaw = yaw;
	rc.roll = roll;
	rc.pitch = pitch;

	rc_count = RC_TIMEOUT; 

}


void mw_attitude_refresh() {
	mspmsg_ATTITUDE_serialize(&mw_msg,NULL);
	shm_put_outgoing(&mw_msg);
}

void mw_attitude_quaternions(float *w, float *x, float *y, float *z) {
	struct S_MSP_ATTITUDE attitude;
	
	shm_get_incoming(&mw_msg,MSP_ATTITUDE);
	mspmsg_ATTITUDE_parse(&attitude,&mw_msg);

	//printf("yaw: %i x: %i y: %i\n",attitude.heading,attitude.angx/10,attitude.angy/10);

	float a = (M_PI / 180) * attitude.angx/10.f;
	float b = (M_PI / 180) * attitude.heading;
	float c = -(M_PI / 180) * attitude.angy/10.f;

    double c1 = cos(a/2);
    double s1 = sin(a/2);
    double c2 = cos(b/2);
    double s2 = sin(b/2);
    double c3 = cos(c/2);
    double s3 = sin(c/2);
    double c1c2 = c1*c2;
    double s1s2 = s1*s2;
    (*w) =c1c2*c3 - s1s2*s3;
  	(*x) =c1c2*s3 + s1s2*c3;
	(*y) =s1*c2*c3 + c1*s2*s3;
	(*z) =c1*s2*c3 - s1*c2*s3;
}

void mw_altitude_refresh() {
	mspmsg_ALTITUDE_serialize(&mw_msg,NULL);
	shm_put_outgoing(&mw_msg);
}

void mw_altitude(int32_t *alt) {
	struct S_MSP_ALTITUDE altitude;
	
	shm_get_incoming(&mw_msg,MSP_ALTITUDE);
	mspmsg_ALTITUDE_parse(&altitude,&mw_msg);

	(*alt) = altitude.EstAlt;	
}

void mw_gps_refresh() {
	mspmsg_RAW_GPS_serialize(&mw_msg,NULL);
	shm_put_outgoing(&mw_msg);	
}

void mw_raw_gps(uint8_t *fix, int32_t *lat, int32_t *lon, int32_t *alt, uint16_t *vel, uint16_t *cog, uint8_t *satellites_visible) {
	struct S_MSP_RAW_GPS gps;
	
	shm_get_incoming(&mw_msg,MSP_RAW_GPS);
	mspmsg_RAW_GPS_parse(&gps,&mw_msg);	

	(*fix) = gps.fix?3:0;
	(*lat) = gps.lat;
	(*lon) = gps.lon;
	(*alt) = gps.alt;
	(*vel) = gps.speed;
	(*cog) = gps.ground_course*10;
	(*satellites_visible) = gps.num_sat;
}

//re-requests pid values and names
//if reset is set - re-request is issues
uint8_t mw_pid_refresh(uint8_t reset) {
	uint8_t filter;
	static uint8_t state = 0;
	static uint8_t got_pid = 0;
	static uint8_t got_pidnames = 0;

	if (reset) {
		state=0;
		got_pid = 0;
		got_pidnames = 0;
	}


	switch (state) {
		case 0: //request pids
			//read existing value to invalidate them

			//pidnames are currently hardcoded in MW hence no need to request them
			//filter = MSP_PIDNAMES;
			//shm_scan_incoming_f(&mw_msg,&filter,1));

			filter = MSP_PID;
			shm_scan_incoming_f(&mw_msg,&filter,1);
			
			//re-request
			//mspmsg_PIDNAMES_serialize(&mw_msg,NULL);
			//shm_put_outgoing(&mw_msg);	

			mspmsg_PID_serialize(&mw_msg,NULL);
			shm_put_outgoing(&mw_msg);	
			state = 1;
			break;
		case 1: //read pids
			//check if we got a new responses
			//filter = MSP_PIDNAMES;
			//if (shm_scan_incoming_f(&mw_msg,&filter,1)) got_pidnames=1;
			got_pidnames=1;
			filter = MSP_PID;
			if (shm_scan_incoming_f(&mw_msg,&filter,1)) {
				got_pid=1;
				state = 2;
			}
			break;
	}

	if (got_pidnames && got_pid) return 1;

	return 0;
}

void mw_rth_start() {
	uint8_t rth_box = mw_get_box_id("GPS HOME");
	if (rth_box==UINT8_MAX) return; //RTH not availble
	mw_box_activate(rth_box);
}

uint8_t mw_box_count() { //gets number of supported boxes
	return msp_get_box_count();
}

char *mw_get_box_name(uint8_t id) { //gets name of box based on id (for supported boxes only)
	return msp_get_boxname(id);
}

uint8_t mw_get_box_id(const char *name) {
	uint8_t ret;
	ret = msp_get_boxid(name);
	if (ret==UINT8_MAX) return UINT8_MAX;
	return ret;
}

uint8_t mw_box_is_supported(uint8_t id) {
	return boxconf.supported[id];
}

uint8_t mw_pid_count() { //this should be only called once mav_param_refresh returns 1 to ensure it is up to date
	return msp_get_pid_count()*3; //each pid has p,i,d values
}

char *mw_get_pid_name(uint8_t id) { //this should be only called once mav_param_refresh returns 1 to ensure it is up to date
	static char buf[16];
	char *pidname;
	pidname = msp_get_pidname(id/3);
	switch (id%3) {
		case 0: sprintf(buf,"%s_P",pidname); break;
		case 1: sprintf(buf,"%s_I",pidname); break;
		case 2: sprintf(buf,"%s_D",pidname); break;
	}

	return buf;
}

uint8_t mw_get_pid_id(const char *name) {
	//we appended 2 chars (_P, _i, _D) when reading names of params, here we need to strip them out
	uint8_t ret = 0;

	uint8_t nlen = strlen(name);
	uint8_t reminder = 0;
	char buf[16];
	strncpy(buf,name,nlen-2);
	buf[nlen-2]=0;

	switch (name[nlen-1]) {
		case 'P': reminder = 0; break;
		case 'I': reminder = 1; break;
		case 'D': reminder = 2; break;
	}

	//printf("Resolved param %s into %u *3 + %u\n",name,msp_get_pidid(buf),reminder);
	ret = msp_get_pidid(buf);
	if (ret==UINT8_MAX) return UINT8_MAX;
	return ret*3+reminder;
}

uint8_t mw_get_pid_value(uint8_t id) { //this should be only called once mav_param_refresh returns 1 to ensure it is up to date
	struct S_MSP_PIDITEMS pids;
	shm_get_incoming(&mw_msg,MSP_PID);
	mspmsg_PID_parse(&pids,&mw_msg);	

	switch (id%3) {
		case 0: return pids.pid[id/3].P8;
		case 1: return pids.pid[id/3].I8;
		case 2: return pids.pid[id/3].D8;
	}

	return 0;
}

void mw_set_pid(uint8_t id, uint8_t v) {
	struct S_MSP_PIDITEMS pids;

	//get current value of pids
	shm_get_incoming(&mw_msg,MSP_PID);
	mspmsg_PID_parse(&pids,&mw_msg);

	//write new param
	switch (id%3) {
		case 0: pids.pid[id/3].P8 = v; break;
		case 1: pids.pid[id/3].I8 = v; break;
		case 2: pids.pid[id/3].D8 = v; break;
	}

	//send it to the service
	mspmsg_SET_PID_serialize(&mw_msg,&pids);
	shm_put_outgoing(&mw_msg);

	mw_pid_refresh(1);
}

void mw_get_signal(int8_t *rssi, int8_t *noise) {
	struct S_MSP_LOCALSTATUS t;
	
	shm_get_incoming(&mw_msg,MSP_LOCALSTATUS);
	mspmsg_LOCALSTATUS_parse(&t,&mw_msg);	

	(*rssi) = t.rssi;
	(*noise) = t.noise;	
}

uint32_t mw_sys_status_sensors() {
	//we take it from status message
	uint32_t ret = 0;
	struct S_MSP_STATUS status;
//	struct S_MSP_BOXCONFIG boxconfig;
	
	shm_get_incoming(&mw_msg,MSP_STATUS);
	mspmsg_STATUS_parse(&status,&mw_msg);

/*
	shm_get_incoming(&mw_msg,MSP_BOXIDS);
	mspmsg_BOXIDS_parse(&boxconfig,&mw_msg);
*/

	ret = MAV_SYS_STATUS_SENSOR_3D_GYRO; //assume we always have gyro


	if (get_bit(status.sensor,0)) ret |= (MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION);	//acc
	//if (get_bit(status.sensors,1)) ret |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL	//baro
	if (get_bit(status.sensor,2)) ret |= MAV_SYS_STATUS_SENSOR_3D_MAG;	//mag
	if (get_bit(status.sensor,3)) ret |= MAV_SYS_STATUS_SENSOR_GPS; 	//gps
	//if (get_bit(status.sensors,4)) ret |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL	//sonar

	return ret;
}

uint8_t mw_state() {
	struct S_MSP_STATUS status;

	shm_get_incoming(&mw_msg,MSP_STATUS);
	mspmsg_STATUS_parse(&status,&mw_msg);

	if (mw_status) //no connection?
		return MAV_STATE_UNINIT;

	if (msp_is_armed(&status)) return MAV_STATE_ACTIVE;

	return MAV_STATE_STANDBY;
}

uint8_t mw_mode_flag() {
	struct S_MSP_STATUS status;

	shm_get_incoming(&mw_msg,MSP_STATUS);
	mspmsg_STATUS_parse(&status,&mw_msg);

	uint8_t ret = 0;
	if (msp_is_armed(&status)) ret |= (MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
	if (boxconf.active[BOXHORIZON]) ret |= MAV_MODE_FLAG_STABILIZE_ENABLED;

	return ret;
}

uint8_t mw_type() {
	struct S_MSP_IDENT ident;

	shm_get_incoming(&mw_msg,MSP_IDENT);
	mspmsg_IDENT_parse(&ident,&mw_msg);

	switch (ident.multitype) {

		case MULTITYPETRI: return MAV_TYPE_TRICOPTER;

		case MULTITYPEVTAIL4: return MAV_TYPE_VTOL_QUADROTOR;

		case MULTITYPEY4:
		case MULTITYPEQUADP: 
		case MULTITYPEQUADX: return MAV_TYPE_QUADROTOR;

		case MULTITYPEY6:
		case MULTITYPEHEX6:
		case MULTITYPEHEX6H:
		case MULTITYPEHEX6X: return MAV_TYPE_HEXAROTOR;

		case MULTITYPEOCTOFLATP:
		case MULTITYPEOCTOFLATX:
		case MULTITYPEOCTOX8: return MAV_TYPE_OCTOROTOR;

		case MULTITYPEGIMBAL: return MAV_TYPE_GIMBAL;

		case MULTITYPEAIRPLANE:
		case MULTITYPEFLYING_WING: return MAV_TYPE_FIXED_WING;

		case MULTITYPEHELI_120_CCPM:
		case MULTITYPEHELI_90_DEG: return MAV_TYPE_HELICOPTER;

		case MULTITYPEBI:
		case MULTITYPEDUALCOPTER: return MAV_TYPE_VTOL_DUOROTOR;

		case MULTITYPESINGLECOPTER:
		case MULTITYPENONE0:		
		case MULTITYPENONE19:
		default: return MAV_TYPE_GENERIC;	
	}
}

void mw_box_activate(uint8_t i) {
	if (!boxconf.supported[i]) {
		printf("BOX %u not supported\n",i);
		return;
	}

	boxconf.active[i] = 0xFFFF;
	mspmsg_SET_BOX_serialize(&mw_msg,&boxconf);
	shm_put_outgoing(&mw_msg);	
}

void mw_box_deactivate(uint8_t i) {
	if (!boxconf.supported[i]) {
		printf("BOX %u not supported\n",i);
		return;
	}

	boxconf.active[i] = 0;
	mspmsg_SET_BOX_serialize(&mw_msg,&boxconf);
	shm_put_outgoing(&mw_msg);	
}


uint8_t mw_box_is_active(uint8_t i) {
	return boxconf.active[i]?1:0;
}

void mw_toggle_box(uint8_t i) {
	if (mw_box_is_active(i)) mw_box_deactivate(i);
	else mw_box_activate(i);
}
