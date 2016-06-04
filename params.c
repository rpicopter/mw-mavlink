/* 
	the important functions are params_init and params_count
	all other functions should not needing changed
*/



#include "params.h"

#include <stdio.h>
#include <stdlib.h>

#include "def.h"
#include "global.h"
#include "gamepad.h"

#ifdef CFG_ENABLED
	#include <sys/stat.h> 
	#include <fcntl.h>
	config_t cfg;
#endif

static mavlink_message_t mav_msg;

static uint8_t debug = 1; 

typedef void (*t_param_set)(void* val);
typedef void (*t_param_get)(void* ret);

typedef void (*_t_param_set_idx)(void* val, uint8_t idx);
typedef void (*_t_param_get_idx)(void* ret, uint8_t idx);

struct s_param {
	uint8_t component;
	uint8_t id; //calculated automatically, consecutive id within component
	char name[17];
	uint8_t type; //see enum
	uint8_t idx; //used only for get and set with idx
	t_param_get get_value;
	t_param_set set_value;	
	uint8_t delayed; //delayed parameter - when setting it, the get should be performed with a delay
	uint8_t can_save;
};

static struct s_param *param;

uint8_t params_count();
void params_cfg_save();

static struct s_param *_get_param(uint8_t component, uint8_t id) {
	uint8_t i;
	uint8_t p_count = params_count();

	if (!param) return NULL;

	for (i=0;i<p_count;i++)
		if ((param[i].component==component) && (param[i].id==id)) return &param[i];

	return NULL;
}

static struct s_param *_get_param_by_name(uint8_t component, char *name) {
	uint8_t i;
	uint8_t p_count = params_count();

	if (!param) return NULL;

	for (i=0;i<p_count;i++)
		if (param[i].component==component)
			if (strcmp(param[i].name,name)==0) return &param[i];

	return NULL;
}

static mavlink_param_union_t *_get_value(uint8_t component, uint8_t id) {
	static mavlink_param_union_t ret;
	ret.type = UINT8_MAX;
	ret.param_float = 0.f; //reset value to 0

	struct s_param *p = _get_param(component,id);

	if (!p) return NULL;
	if (!p->get_value) return NULL;

	if (p->idx!=UINT8_MAX) ((_t_param_get_idx)p->get_value)(ret.bytes,p->idx);
	else p->get_value(ret.bytes);

	ret.type=p->type;

	return &ret;
}

static void _set_value(uint8_t component, uint8_t id, mavlink_param_union_t *v) {
	struct s_param *p = _get_param(component,id);
	if (!p) return;
	if (!p->set_value) return;

	if (p->idx!=UINT8_MAX) ((_t_param_set_idx)p->set_value)(v->bytes,p->idx);
	else p->set_value(v->bytes);
}


/* ============== RPI CAMERA HANDLING ========== */
#ifdef RPICAM_ENABLED

#include "udp.h"

uint8_t rpicam_debug = 1;

char rpicmd[256];
uint8_t rpicam_mode = 0;
uint8_t rpicam_on = 0;

void rpicam_stop() {
	if (!rpicam_on) return;
	int ret;
	memset(rpicmd, '\0', 256);
	sprintf(rpicmd, "%s stop",CAM_CMD);
	if (rpicam_debug) printf("Executing: %s\n",rpicmd);
	ret = system(rpicmd);
	if (ret==0) rpicam_on = 0;
	if (rpicam_debug) printf("Stoping camera_streamer returned: %i\n",ret);	
}

void rpicam_start(uint8_t type) {
	if (rpicam_on) {
		if (rpicam_debug) printf("Camera is already streaming. Stopping.\n");
		rpicam_stop();
		//return;
	}
	int ret;
	memset(rpicmd, '\0', 256);
	sprintf(rpicmd, "%s start %s %i %i",CAM_CMD, get_gc_ip(),5600,type);
	if (rpicam_debug) printf("Executing: %s\n",rpicmd);
	ret=system(rpicmd);

	if (ret==0) rpicam_on = 1;

	if (rpicam_debug) printf("Starting camera_streamer %i returned: %i\n",type,ret);
}

void rpicam_emergency() {
	if (!rpicam_on) return;
	rpicam_stop();
	rpicam_start(11);
}

void rpicam_set(uint8_t* _value) {
	rpicam_mode = *_value;

	if (rpicam_mode==0) rpicam_stop();
	else rpicam_start(rpicam_mode);
}

void rpicam_get(uint8_t* _value) {
	(*_value) = rpicam_mode;
}

#endif
/* ============== RPI CAMERA HANDLING END ========== */

uint8_t system_debug = 1;
char syscmd[256];

void system_set(uint8_t* _value) {
	int ret = 0;
	switch (*_value) {
		case 1: //reboot;
			ret = system(REBOOT_CMD);
			break;
		case 2: //save & sync;
			params_cfg_save();
			ret = system("/bin/sync");
			break;
	}

	if (system_debug) printf("Run system command %i. Ret: %i\n",*_value,ret);
}

static uint8_t failsafe_mode = 0;

uint8_t failsafe_rth() {
	return failsafe_mode;
}

void failsafe_set(uint8_t *_value) {
	failsafe_mode = (*_value);
	mw_set_rth(failsafe_mode);
}

void failsafe_get(uint8_t *_value) {
	(*_value) = failsafe_mode;
}

//============================================


uint8_t params_count() {
	static uint8_t ret = 0;

	if (ret) return ret;

	ret = mw_pid_count()
		+ gamepad_button_count()
		+ 1 //gamepad_mode
		+ 3 //gamepad_threshold
		+ 1 //rth on connection loss
		+ 1 //reboot
		+ 7 //rc_tunning
#ifdef RPICAM_ENABLED		
		+ 1 //camera config
#endif
		+ 1 //eeprom save
		+ 1 //rth_altitude
		+ 1 //failsafe_throttle
		;

	return ret;
}

uint8_t params_count_component(uint8_t component) {
	uint8_t i;
	uint8_t ret = 0;
	uint8_t all_count = params_count();

	for (i=0;i<all_count;i++)
		if (param[i].component==component) ret++;

	return ret;
}

int params_cfg_load() {
#ifdef CFG_ENABLED
	uint8_t i,j;
  	if(!config_read_file(&cfg, CFG_FILE))
  	{
  		printf("Unable to open.\n");
    	fprintf(stderr, "%s:%d - %s\n", config_error_file(&cfg),
            config_error_line(&cfg), config_error_text(&cfg));
    		config_destroy(&cfg);
    	return(EXIT_FAILURE);
  	}	

//config initialization
	config_setting_t *root = config_root_setting(&cfg);
	config_setting_t *setting = config_lookup(&cfg,"params");
	uint8_t count = 0;

	for (i=0;i<params_count();i++) 
		if (param[i].can_save) count++;

	if (setting) {
		if (config_setting_length(setting)!=count) { //ensure the count of params has not changed, otherwise we invalidate the config
			config_setting_remove(root,"params");
			config_write_file(&cfg,CFG_FILE);
			setting = NULL; 
			printf("Params changed. Resetting.\n");
		} else { //load them
			j=0;
			for (i=0;i<params_count();i++)
				if (param[i].can_save) {
					mavlink_param_union_t v;
					v.param_float = config_setting_get_float_elem(setting,j++);
					_set_value(param[i].component,param[i].id, &v );
				}
		}
	}

	config_destroy(&cfg);

#endif
	return 0;
}

void params_cfg_save() {
#ifdef CFG_ENABLED
	uint8_t i;
	config_read_file(&cfg, CFG_FILE);
	config_setting_t *root = config_root_setting(&cfg);
	config_setting_t *ptr;

	config_setting_t *setting = config_lookup(&cfg,"params");

	config_setting_remove(root,"params");

	setting = config_setting_add(root, "params", CONFIG_TYPE_ARRAY);
	for (i=0;i<params_count();i++) 
		if (param[i].can_save) {
			ptr=config_setting_add(setting,NULL,CONFIG_TYPE_FLOAT);
			mavlink_param_union_t *v = _get_value(param[i].component,param[i].id);
			config_setting_set_float(ptr,v->param_float);
		}

 	if (!config_write_file(&cfg,CFG_FILE)) 
 		printf("Error while writing config file. Settings won't be stored.\n");

 	config_destroy(&cfg);
#endif 	
}

void params_cfg_open() {
#ifdef CFG_ENABLED
	int ret;
//config file open
	printf("Opening config: %s...\n",CFG_FILE);

	if (access(CFG_FILE,F_OK)) {
		printf("Not found. Creating...\n");
		ret = creat(CFG_FILE, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
		if (ret>0) {
			printf("Success.\n");
			close(ret); //we need to close as config_read_file re-opens it
		}
	}

#endif
}

void params_cfg_end() {
#ifdef CFG_ENABLED

#endif
}

void params_init() {
	uint8_t offset = 0;
	uint8_t i;
	uint8_t p_count = params_count();
	char *ptr;

	param = malloc(sizeof(struct s_param)*p_count);
	if (!param) {
		printf("Error allocating memory for params\n");
		return;
	}

	for (i=0;i<p_count;i++) {
		param[i].idx = UINT8_MAX;
		param[i].component = 0;
		param[i].get_value = NULL;
		param[i].set_value = NULL;
		param[i].can_save = 0;
		param[i].delayed = 0;
		param[i].type = MAV_PARAM_TYPE_UINT8;
	}

	offset = 0;


	//construct all params, NOTE: params relating to given component have to be added at one go
	//NOTE: only int elements can be currently saved
	param[offset].component = 200;
	sprintf(param[offset].name,"%s","!EEPROM_SAVE");
	param[offset].set_value = (t_param_set)mw_eeprom_write;

	offset++;

	for (i=0;i<mw_pid_count();i++) {
		param[i+offset].component = 200;
		ptr = mw_get_pid_name(i);
		strcpy(param[i+offset].name,ptr);
		param[i+offset].idx = i; //used for the get and set functions
		param[i+offset].get_value = (t_param_get)mw_get_pid_value;
		param[i+offset].set_value = (t_param_set)mw_set_pid;
		param[i+offset].delayed = 1;
		param[i+offset].can_save = 0; //we dont want to save in CFG but in MW board
	}
	offset += i;

	for (i=0;i<gamepad_button_count();i++) {
		param[i+offset].component = 201;
		sprintf(param[i+offset].name,"%s",gamepad_get_button_name(i));
		param[i+offset].idx = i;
		param[i+offset].get_value = (t_param_get)gamepad_get_mapping;
		param[i+offset].set_value = (t_param_set)gamepad_set_mapping;
		param[i+offset].can_save = 1;
	}
	offset += i;

	param[offset].component = 201;
	sprintf(param[offset].name,"%s","!GAMEPAD_MODE");
	param[offset].get_value = (t_param_get)gamepad_get_mode;
	param[offset].set_value = (t_param_set)gamepad_set_mode;
	param[offset].can_save = 1;
	offset += 1;


	param[offset].component = 201;
	sprintf(param[offset].name,"%s","!SYS");
	param[offset].set_value = (t_param_set)system_set;
	offset += 1;

#ifdef RPICAM_ENABLED
	param[offset].component = 201;
	sprintf(param[offset].name,"%s","!VIDEO");
	param[offset].get_value = (t_param_get)rpicam_get;
	param[offset].set_value = (t_param_set)rpicam_set;
	offset += 1;
#endif

	param[offset].component = 202;
	sprintf(param[offset].name,"%s%s",(mw_box_is_supported(BOXGPSHOME)?"!":"~"),"RTH_FAILSAFE");
	param[offset].get_value = (t_param_get)failsafe_get;
	param[offset].set_value = (t_param_set)failsafe_set;
	param[offset].can_save = 1;
	offset += 1;

	param[offset].component = 202;
	sprintf(param[offset].name,"%s","FAILSAFE_THROT");
	param[offset].get_value = (t_param_get)mw_get_failsafe_throttle;
	param[offset].set_value = (t_param_set)mw_set_failsafe_throttle;
	param[offset].type = MAV_PARAM_TYPE_UINT16;
	param[offset].delayed = 1;
	offset += 1;

	param[offset].idx = 0;
	param[offset].component = 202;
	sprintf(param[offset].name,"%s","THRESHOLD_YAW");
	param[offset].get_value = (t_param_get)gamepad_get_threshold;
	param[offset].set_value = (t_param_set)gamepad_set_threshold;
	param[offset].can_save = 1;
	offset += 1;

	param[offset].idx = 1;
	param[offset].component = 202;
	sprintf(param[offset].name,"%s","THRESHOLD_PITCH");
	param[offset].get_value = (t_param_get)gamepad_get_threshold;
	param[offset].set_value = (t_param_set)gamepad_set_threshold;
	param[offset].can_save = 1;
	offset += 1;

	param[offset].idx = 2;
	param[offset].component = 202;
	sprintf(param[offset].name,"%s","THRESHOLD_ROLL");
	param[offset].get_value = (t_param_get)gamepad_get_threshold;
	param[offset].set_value = (t_param_set)gamepad_set_threshold;
	param[offset].can_save = 1;
	offset += 1;

	param[offset].component = 202;
	sprintf(param[offset].name,"%s","RTH_ALT");
	param[offset].get_value = (t_param_get)mw_get_rth_alt;
	param[offset].set_value = (t_param_set)mw_set_rth_alt;
	param[offset].type = MAV_PARAM_TYPE_UINT16;
	param[offset].delayed = 1;
	offset += 1;

	for (i=0;i<7;i++) {
		param[offset+i].component = 202;
		sprintf(param[offset+i].name,"%s",mw_get_rc_tunning_name(i));
		param[offset+i].idx = i;
		param[offset+i].get_value = (t_param_get)mw_get_rc_tunning;
		param[offset+i].set_value = (t_param_set)mw_set_rc_tunning;
		param[offset+i].delayed = 1;
	}
	offset += i;

	//calculate id for each component
	uint8_t j = 0,c = 0;
	for (i=0;i<p_count;i++) {
		if (param[i].component!=c) {
			c = param[i].component;
			j = 0;
		} 
		param[i].id=j++;
	}

	gamepad_init();
	params_cfg_open();
	params_cfg_load();
}

void params_end() {
	params_cfg_save();
	params_cfg_end();
	if (param) free(param);
	param = NULL;
}



void params_set(uint8_t component, char *name, float value) {
	//sets the value of a param at idx
	//send the param back
	struct s_param *p = _get_param_by_name(component,name);

	//uint8_t old_val;

	mavlink_param_union_t m_param;
	mavlink_param_union_t* old;
	m_param.param_float = value;

	_set_value(p->component,p->id,&m_param);

	//for certain params it takes time to refresh them. We not gonna send back until it is set
	
	old = _get_value(p->component,p->id);
	
	if (old && p->delayed && p->delayed<4) { //try 4 times max
		if (old->param_float != value) { //it got set correctly
			p->delayed++;
			return;
		}
	}
	if (p->delayed) p->delayed=1;

	params_send(p->component,p->id);

}

void params_send(uint8_t component, uint8_t id) {
	//reads value of a param at idx
	//send is back

	struct s_param *p = _get_param(component,id);

	mavlink_param_union_t m_param;
	mavlink_param_union_t *ptr;
	ptr = _get_value(component, id);
	if (ptr) m_param = (*ptr);
	else {
		m_param.type = p->type;
		m_param.param_float = 0.f;
	}

	mavlink_msg_param_value_pack(1,p->component,&mav_msg,p->name,m_param.param_float, m_param.type,params_count_component(p->component),p->id);

	dispatch(&mav_msg);

	//send all pids under COMPONENT 200
	printf("-> param_value id: %u component: %u, name: %s value: %i. ALL: %u\n",id,component,p->name,m_param.param_int32,params_count_component(p->component));
}

//before we can send all values we need to refresh some of them
uint8_t params_get_all(uint8_t reset) {
	static uint8_t step = 0;
	uint8_t i;

	if (reset) {
		if (debug) printf("Requesting pid_values...\n");
		mw_pid_refresh(1);
		step=0;
	}

	switch (step) {
		case 0: //wait for params refresh to finish
			if (debug) printf("Waiting...\n");
			if (mw_pid_refresh(0)) step++;
			break;
		case 1: //send params
			for (i=0;i<params_count();i++)
				params_send(param[i].component,param[i].id);
			
			return 1;
			break;
	}

	return 0;
}

