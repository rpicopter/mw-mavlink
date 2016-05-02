
#include "mw.h"
#include <mw/shm.h>
#include <stdio.h>
#include <math.h>

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include "mavlink/common/mavlink.h"

#define MW_TIMEOUT 3 //3s timeout for mw
static uint8_t mw_status=0; //0-standby, 1-armed; 2-no connection?
static uint8_t failsafe=0;
static uint8_t rth_failsafe=0; //has rth as failsafe

static uint8_t has_homepos=0;
static struct S_MSP_WP homepos;

static struct S_MSG mw_msg;
static struct S_MSP_BOXCONFIG boxconf;
static struct S_MSP_RC rc = {.throttle=1000,.yaw=1500,.pitch=1500,.roll=1500,.aux1=1500,.aux2=1500,.aux3=1500,.aux4=1500};

#define RC_TIMEOUT 40 //1.5sec timeout for manual_control (see main loop for manual_control handling)
static uint8_t rc_count;

void mw_keepalive();
void mw_altitude_refresh();
void mw_attitude_refresh();
void mw_gps_refresh();
void mw_box_refresh();
void mw_feed_rc();
void mw_standby();
void mw_homepos_refresh();

typedef void (*t_cb)();

struct _S_TASK {
	uint16_t freq; //has to be less than loop_counter max value
	t_cb cb_fn;
};
typedef struct _S_TASK S_TASK;

#define MAX_TASK 7 //6

static S_TASK task[MAX_TASK] = {
	{1, mw_feed_rc}, //run every LOOP_MS (see global.h)
	{4, mw_standby},
	{20, mw_altitude_refresh},
	{4, mw_attitude_refresh},
	{20, mw_gps_refresh},
	{40, mw_keepalive},
	{40, mw_box_refresh}
};


void mw_loop() { //
	static uint8_t counter = 0;
	uint8_t i;

	for (i=0;i<MAX_TASK;i++)
		if (counter%task[i].freq==0) {
			task[i].cb_fn();
		}

	counter++;
	if (counter==100) counter=0;

}

uint8_t mw_init() {
 	if (shm_client_init()) return -1; 

	//this retrievs the initial set of settings from MW like boxconfiguration, etc
	uint8_t filter;

	rc_count = 0;
	//ident
	filter = MSP_IDENT;
	shm_scan_incoming_f(&mw_msg,&filter,1); //invalidate
	while (1) {
		mspmsg_IDENT_serialize(&mw_msg);
		shm_put_outgoing(&mw_msg);
		mssleep(50);
		if (shm_scan_incoming_f(&mw_msg,&filter,1)) break; //got the response
	} 

	//status
	filter = MSP_STATUS;
	shm_scan_incoming_f(&mw_msg,&filter,1); //invalidate
	while (1) {
		mspmsg_STATUS_serialize(&mw_msg);
		shm_put_outgoing(&mw_msg);
		mssleep(50);
		if (shm_scan_incoming_f(&mw_msg,&filter,1)) break; //got the response
	}

	filter = MSP_MISC;
	shm_scan_incoming_f(&mw_msg,&filter,1); //invalidate
	while (1) {
		mspmsg_MISC_serialize(&mw_msg);
		shm_put_outgoing(&mw_msg);
		mssleep(50);
		if (shm_scan_incoming_f(&mw_msg,&filter,1)) break; //got the response
	} 	

	filter = MSP_RC_TUNING;
	shm_scan_incoming_f(&mw_msg,&filter,1); //invalidate
	while (1) {
		mspmsg_RC_TUNING_serialize(&mw_msg);
		shm_put_outgoing(&mw_msg);
		mssleep(50);
		if (shm_scan_incoming_f(&mw_msg,&filter,1)) break; //got the response
	} 	

	filter = MSP_NAV_CONFIG;
	shm_scan_incoming_f(&mw_msg,&filter,1); //invalidate
	while (1) {
		mspmsg_NAV_CONFIG_serialize(&mw_msg);
		shm_put_outgoing(&mw_msg);
		mssleep(50);
		if (shm_scan_incoming_f(&mw_msg,&filter,1)) break; //got the response
	} 

	//boxids
	filter = MSP_BOXIDS;
	shm_scan_incoming_f(&mw_msg,&filter,1); //invalidate
	while (1) {
		mspmsg_BOXIDS_serialize(&mw_msg);
		shm_put_outgoing(&mw_msg);
		mssleep(50);
		if (shm_scan_incoming_f(&mw_msg,&filter,1)) {
			mspmsg_BOXIDS_parse(&boxconf,&mw_msg);
			break; //got the response
		}
	} 	

	mspmsg_BOX_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);	

	return 0;
}

void mw_end() {
 	shm_client_end(); //close channel to mw-service	
}

void initiate_failsafe() {
	uint8_t fallback = 1;
	if (rth_failsafe && has_homepos) fallback = mw_rth_start();
	
	if (fallback) failsafe = 1;
}

void mw_feed_rc() {
	//this is run from a loop
	if ((rc_count==0) || (failsafe)) return; //dont feed manual_control if in failsafe
	rc_count--;

	mspmsg_SET_RAW_RC_serialize(&mw_msg,&rc);
	shm_put_outgoing(&mw_msg);

	if (rc_count==0) { //rc has just timed-out 
		initiate_failsafe();
	}
}

void mw_standby() {
	if (mw_status!=0) return; //not in standby

	failsafe = 0;
	mw_homepos_refresh();
}

/* ============== REFRESH FUNCTIONS ==============  */
void mw_homepos_refresh() {
	struct S_MSP_WP wp;
	uint8_t filter;

	mspmsg_WP_serialize(&mw_msg,0);
	shm_put_outgoing(&mw_msg);

	filter = MSP_WP;
	if (shm_scan_incoming_f(&mw_msg,&filter,1)) {
		mspmsg_WP_parse(&wp,&mw_msg);
		if ((wp.wp_no==0) && (wp.lat!=0) && (wp.lon!=0)) {
			homepos=wp;
			has_homepos = 1;
		} else {
			has_homepos = 0;
		}
	}
}


void mw_box_refresh() {
	mspmsg_BOX_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);	

	shm_get_incoming(&mw_msg,MSP_BOX);
	mspmsg_BOX_parse(&boxconf,&mw_msg);
}

void mw_attitude_refresh() {
	mspmsg_ATTITUDE_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);
}

void mw_altitude_refresh() {
	mspmsg_ALTITUDE_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);
}

void mw_gps_refresh() {
	mspmsg_RAW_GPS_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);	
}

void mw_keepalive() {
	//keep alive for MultiWii and the service
	static uint8_t err_counter = 0; //number of missed status messages
	struct S_MSP_STATUS status;
	uint8_t filter;

	mspmsg_LOCALSTATUS_serialize(&mw_msg,NULL);
	shm_put_outgoing(&mw_msg);	

	mspmsg_STATUS_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);

	filter = MSP_STATUS;
	if (!shm_scan_incoming_f(&mw_msg,&filter,1)) err_counter++;
	else {
		mspmsg_STATUS_parse(&status,&mw_msg);
		err_counter = 0;
		if (msp_is_armed(&status)) mw_status=1;
		else mw_status = 0;
	}

	if (err_counter>MW_TIMEOUT) {
		mw_status = 2;
	}
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

			mspmsg_PID_serialize(&mw_msg);
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
/* ============== END REFRESH FUNCTIONS ============== */

void mw_arm() {
	struct S_MSP_STICKCOMBO msg;
	msg.combo = STICKARM;

	mspmsg_STICKCOMBO_serialize(&mw_msg,&msg);
	shm_put_outgoing(&mw_msg);	

	//trigger status refresh for quicker response
	mspmsg_STATUS_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);
}

void mw_disarm() {
	struct S_MSP_STICKCOMBO msg;
	msg.combo = STICKDISARM;

	mspmsg_STICKCOMBO_serialize(&mw_msg,&msg);
	shm_put_outgoing(&mw_msg);	

	//trigger status refresh for quicker response
	mspmsg_STATUS_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);
}

void mw_eeprom_write(uint8_t *dummy) {
	mspmsg_EEPROM_WRITE_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);	
}

uint16_t mw_get_i2c_drop_count() {
	//this get count of errors on MW->MW_SERVICE link only
	struct S_MSP_LOCALSTATUS lstatus;
	
	shm_get_incoming(&mw_msg,MSP_LOCALSTATUS);
	mspmsg_LOCALSTATUS_parse(&lstatus,&mw_msg);	

	return lstatus.crc_error_count;
}

uint16_t mw_get_i2c_drop_rate() {
	//this get count of errors on MW->MW_SERVICE link only
	struct S_MSP_LOCALSTATUS lstatus;
	
	shm_get_incoming(&mw_msg,MSP_LOCALSTATUS);
	mspmsg_LOCALSTATUS_parse(&lstatus,&mw_msg);	

	if (!lstatus.rx_count) return 0;
	return (lstatus.crc_error_count/lstatus.rx_count)*10000; //100%=10000
}

char *mw_get_rc_tunning_name(uint8_t i) {
	switch(i) {
		case 0: return "RC_RATE";
		case 1: return "RC_EXPO";
		case 2: return "ROLL_PITCH_R";
		case 3: return "YAW_RATE";
		case 4: return "DYN_THR_PID";
		case 5: return "THR_MID";
		case 6: return "THR_EXPO";
	}
	return "...";
}

void mw_get_rc_tunning(uint8_t* v, uint8_t id) {
	struct S_MSP_RC_TUNING rct;
	
	shm_get_incoming(&mw_msg,MSP_RC_TUNING);
	mspmsg_RC_TUNING_parse(&rct,&mw_msg);

	(*v) = ((uint8_t*)&rct)[id];
}

void mw_set_rc_tunning(uint8_t* v, uint8_t id) {
	struct S_MSP_RC_TUNING rct;
	
	shm_get_incoming(&mw_msg,MSP_RC_TUNING);
	mspmsg_RC_TUNING_parse(&rct,&mw_msg);

	((uint8_t*)&rct)[id] = (*v);
	//save
	mspmsg_SET_RC_TUNING_serialize(&mw_msg,&rct);

	shm_put_outgoing(&mw_msg);

	//refresh
	mspmsg_RC_TUNING_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);	
}

void mw_get_rth_alt(uint16_t *alt) {
	struct S_MSP_NAV_CONFIG nav;
	
	shm_get_incoming(&mw_msg,MSP_NAV_CONFIG);
	mspmsg_NAV_CONFIG_parse(&nav,&mw_msg);

	(*alt) = nav.rth_altitude;
	printf("RTH %u\n",*alt);
}

void mw_set_rth_alt(uint16_t *alt) {
	struct S_MSP_NAV_CONFIG nav;
	
	shm_get_incoming(&mw_msg,MSP_NAV_CONFIG);
	mspmsg_NAV_CONFIG_parse(&nav,&mw_msg);

	nav.rth_altitude = (*alt);
	//save
	mspmsg_NAV_CONFIG_SET_serialize(&mw_msg,&nav);

	shm_put_outgoing(&mw_msg);

	//refresh
	mspmsg_NAV_CONFIG_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);
}

void mw_get_failsafe_throttle(uint16_t* throttle) {
	struct S_MSP_MISC misc;
	
	shm_get_incoming(&mw_msg,MSP_MISC);
	mspmsg_MISC_parse(&misc,&mw_msg);

	(*throttle) = misc.failsafe_throttle;
}

void mw_set_failsafe_throttle(uint16_t* throttle) {
	struct S_MSP_MISC misc;
	
	shm_get_incoming(&mw_msg,MSP_MISC);
	mspmsg_MISC_parse(&misc,&mw_msg);

	misc.failsafe_throttle = (*throttle);
	//save
	mspmsg_SET_MISC_serialize(&mw_msg,&misc);

	shm_put_outgoing(&mw_msg);

	//refresh
	mspmsg_MISC_serialize(&mw_msg);
	shm_put_outgoing(&mw_msg);	
}

void mw_manual_control(int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll) {

	rc.throttle = throttle;
	rc.yaw = yaw;
	rc.roll = roll;
	rc.pitch = pitch;

	rc_count = RC_TIMEOUT; 

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
    if (w) (*w) = c1c2*c3 - s1s2*s3;
  	if (x) (*x) = c1c2*s3 + s1s2*c3;
	if (y) (*y) = s1*c2*c3 + c1*s2*s3;
	if (z) (*z) = c1*s2*c3 - s1*c2*s3;
}

void mw_altitude(int32_t *alt) {
	struct S_MSP_ALTITUDE altitude;
	
	shm_get_incoming(&mw_msg,MSP_ALTITUDE);
	mspmsg_ALTITUDE_parse(&altitude,&mw_msg);

	if (alt) (*alt) = altitude.EstAlt;	
}


void mw_raw_gps(uint8_t *fix, int32_t *lat, int32_t *lon, int32_t *alt, uint16_t *vel, uint16_t *cog, uint8_t *satellites_visible) {
	struct S_MSP_RAW_GPS gps;
	
	shm_get_incoming(&mw_msg,MSP_RAW_GPS);
	mspmsg_RAW_GPS_parse(&gps,&mw_msg);	

	if (fix) (*fix) = gps.fix?3:0;
	if (lat) (*lat) = gps.lat;
	if (lon) (*lon) = gps.lon;
	if (alt) (*alt) = gps.alt;
	if (vel) (*vel) = gps.speed;
	if (cog) (*cog) = gps.ground_course*10;
	if (satellites_visible) (*satellites_visible) = gps.num_sat;
}

void mw_get_homepos(int32_t *lat, int32_t *lon, int32_t *alt) {
	if (!has_homepos) {
		if (lat) lat = NULL;
		if (lon) lon = NULL;
		if (alt) alt = NULL;
		return;
	}

	if (lat) (*lat) = homepos.lat;
	if (lon) (*lon) = homepos.lon;
	if (alt) (*alt) = homepos.alt_hold;
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

void mw_get_pid_value(uint8_t *ret, uint8_t id) { //this should be only called once mav_param_refresh returns 1 to ensure it is up to date
	struct S_MSP_PIDITEMS pids;
	shm_get_incoming(&mw_msg,MSP_PID);
	mspmsg_PID_parse(&pids,&mw_msg);	

	switch (id%3) {
		case 0: (*ret)=pids.pid[id/3].P8; break;
		case 1: (*ret)=pids.pid[id/3].I8; break;
		case 2: (*ret)=pids.pid[id/3].D8; break;
	}
}

void mw_set_pid(uint8_t *v, uint8_t id) {
	struct S_MSP_PIDITEMS pids;

	//get current value of pids
	shm_get_incoming(&mw_msg,MSP_PID);
	mspmsg_PID_parse(&pids,&mw_msg);

	//write new param
	switch (id%3) {
		case 0: pids.pid[id/3].P8 = (*v); break;
		case 1: pids.pid[id/3].I8 = (*v); break;
		case 2: pids.pid[id/3].D8 = (*v); break;
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

	if (rssi) (*rssi) = t.rssi;
	if (noise) (*noise) = t.noise;	
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

	ret |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; 

	if (get_bit(status.sensor,0)) ret |= (MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION);	//acc
	if (get_bit(status.sensor,1)) ret |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;	//baro
	if (get_bit(status.sensor,2)) ret |= MAV_SYS_STATUS_SENSOR_3D_MAG;	//mag
	if (get_bit(status.sensor,3)) ret |= MAV_SYS_STATUS_SENSOR_GPS; 	//gps
	if (get_bit(status.sensor,4)) ret |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;	//sonar

	return ret;
}

uint8_t mw_state() {
	if (failsafe) return MAV_STATE_EMERGENCY;

	switch (mw_status) {
		case 0: return MAV_STATE_STANDBY;
		case 1: return MAV_STATE_ACTIVE;
		case 2: return MAV_STATE_UNINIT;
	}

	return MAV_STATE_UNINIT;
}

uint8_t mw_mode_flag() {
	struct S_MSP_STATUS status;

	shm_get_incoming(&mw_msg,MSP_STATUS);
	mspmsg_STATUS_parse(&status,&mw_msg);

	uint8_t ret = 0;
	if (msp_is_armed(&status)) ret |= (MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
	if (boxconf.value[BOXBARO] || boxconf.value[BOXGPSHOLD]) ret |= MAV_MODE_FLAG_STABILIZE_ENABLED;
	if (boxconf.value[BOXGPSHOME]) ret |= MAV_MODE_FLAG_AUTO_ENABLED;
	if (boxconf.value[BOXGPSNAV]) ret |= MAV_MODE_FLAG_GUIDED_ENABLED;

	if (failsafe) ret = (MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED);
	
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

uint8_t mw_rth_start() {
	if (!has_homepos) return 1;
	uint8_t rth_box = mw_get_box_id("GPS HOME");
	if (rth_box==UINT8_MAX) return 2; //RTH not availble
	mw_box_activate(rth_box);

	return 0;
}	

void mw_box_activate(uint8_t i) {
	if (!boxconf.supported[i]) {
		printf("BOX %u not supported\n",i);
		return;
	}

	boxconf.value[i] = 0xFFFF;
	mspmsg_SET_BOX_serialize(&mw_msg,&boxconf);
	shm_put_outgoing(&mw_msg);	
}

void mw_box_deactivate(uint8_t i) {
	if (!boxconf.supported[i]) {
		printf("BOX %u not supported\n",i);
		return;
	}

	boxconf.value[i] = 0;
	mspmsg_SET_BOX_serialize(&mw_msg,&boxconf);
	shm_put_outgoing(&mw_msg);	
}


uint8_t mw_box_is_active(uint8_t i) {
	return boxconf.value[i]?1:0;
}

void mw_toggle_box(uint8_t i) {
	if (mw_box_is_active(i)) mw_box_deactivate(i);
	else mw_box_activate(i);
}