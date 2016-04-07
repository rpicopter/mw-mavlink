/* 
	this only support params with uint8_t value
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
	#define CFG_FILE "/etc/mw-mavlink.cfg"
	config_t cfg;
#endif


static mavlink_message_t mav_msg;

static uint8_t debug = 1;

typedef void (*t_param_set)(uint8_t idx, uint8_t val);
typedef uint8_t (*t_param_get)(uint8_t idx);

struct s_param {
	uint8_t internal_idx;
	char name[17];
	t_param_get get_value;
	t_param_set set_value;
	uint8_t can_save;
};

static struct s_param *param;


static uint8_t _get_value(uint8_t idx) {
	if (!param) return 0;
	return param[idx].get_value(param[idx].internal_idx);
}

static void _set_value(uint8_t idx, uint8_t v) {
	if (!param) return;
	param[idx].set_value(param[idx].internal_idx,v);
}

uint8_t params_count() {
	static uint8_t ret = 0;

	if (ret) return ret;

	ret = mw_pid_count()
		+ mw_box_count() //gamepad_mapping
		+ 1 //gamepad_mapping
		+ 1 //gamepad_mode
		+ 3 //gamepad_threshold
		;

	return ret;
}

int params_cfg_load() {
	uint8_t i,j;
#ifdef CFG_ENABLED
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
				if (param[i].can_save) _set_value( i, config_setting_get_int_elem(setting,j++) );
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
			ptr=config_setting_add(setting,NULL,CONFIG_TYPE_INT);
			config_setting_set_int(ptr,_get_value(i));
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
	char *ptr;

	param = malloc(sizeof(struct s_param)*params_count());
	if (!param) {
		printf("Error allocating memory for params\n");
		return;
	}

	offset = 0;

	for (i=0;i<mw_pid_count();i++) {
		param[i+offset].internal_idx = i;
		ptr = mw_get_pid_name(i);
		strcpy(param[i+offset].name,ptr);
		param[i+offset].get_value = mw_get_pid_value;
		param[i+offset].set_value = mw_set_pid;
		param[i+offset].can_save = 0;
	}
	offset += i;

	/*
		we link gamepad buttons to boxes 
	*/
	for (i=0;i<mw_box_count();i++) {
		param[i+offset].internal_idx = i;
		ptr = mw_get_box_name(i);
		if (mw_box_is_supported(i)) 
			sprintf(param[i+offset].name,"BTN_%s",ptr);
		else
			sprintf(param[i+offset].name,"~BTN_%s",ptr);
		param[i+offset].get_value = gamepad_get_mapping;
		param[i+offset].set_value = gamepad_set_mapping;
		param[i+offset].can_save = 1;
	}
	offset += i;

	param[offset].internal_idx = i;
	sprintf(param[offset].name,"%s","BTN_THROT_OFF");
	param[offset].get_value = gamepad_get_mapping;
	param[offset].set_value = gamepad_set_mapping;
	param[offset].can_save = 1;
	offset += 1;

	param[offset].internal_idx = 0;
	sprintf(param[offset].name,"%s","!GAMEPAD_MODE");
	param[offset].get_value = gamepad_get_mode;
	param[offset].set_value = gamepad_set_mode;
	param[offset].can_save = 1;
	offset += 1;

	param[offset].internal_idx = 0;
	sprintf(param[offset].name,"%s","THRESHOLD_YAW");
	param[offset].get_value = gamepad_get_threshold;
	param[offset].set_value = gamepad_set_threshold;
	param[offset].can_save = 1;
	offset += 1;

	param[offset].internal_idx = 1;
	sprintf(param[offset].name,"%s","THRESHOLD_PITCH");
	param[offset].get_value = gamepad_get_threshold;
	param[offset].set_value = gamepad_set_threshold;
	param[offset].can_save = 1;
	offset += 1;

	param[offset].internal_idx = 2;
	sprintf(param[offset].name,"%s","THRESHOLD_ROLL");
	param[offset].get_value = gamepad_get_threshold;
	param[offset].set_value = gamepad_set_threshold;
	param[offset].can_save = 1;
	offset += 1;

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

	uint8_t idx;	
	uint8_t old_val;
	uint8_t i;

	mavlink_param_union_t m_param;

	if (component!=200) {
		printf("params_set: Unsupported component %u\n",component);
		return;
	}

	m_param.param_float = value;

	for (i=0;i<params_count();i++)
		if (strcmp(param[i].name,name)==0) idx = i;

	old_val = _get_value(idx);

	if (m_param.param_uint8==old_val) { //confirm with send only if we are sure that it is saved
		//if (debug) printf("Value has not changed.\n");
		params_send(component,idx);
		params_cfg_save();
		return;
	}

	_set_value(idx,m_param.param_uint8);
}

void params_send(uint8_t component, uint8_t idx) {
	//reads value of a param at idx
	//send is back
	//we are combining all the params

	mavlink_param_union_t m_param;
	m_param.param_uint8 = _get_value(idx);
	m_param.type = MAV_PARAM_TYPE_UINT8;

	mavlink_msg_param_value_pack(1,component,&mav_msg,param[idx].name,m_param.param_float, m_param.type,params_count(),idx);

	dispatch(&mav_msg);

	//send all pids under COMPONENT 200
	printf("-> param_value id: %u name: %s value: %u. ALL: %u\n",idx,param[idx].name,m_param.param_uint8,params_count());
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
				params_send(200,i);
			return 1;
			break;
	}

	return 0;
}


