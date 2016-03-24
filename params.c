#include "params.h"

#include <stdio.h>
#include <stdlib.h>

#include "def.h"
#include "global.h"

#ifdef CFG_ENABLED
	#include <sys/stat.h> 
	#include <fcntl.h>
	#define CFG_FILE "/etc/mw-mavlink.cfg"
#endif

// THIS NEEDS TO BE REVISITED

#define MANUAL_CONTROL_MODE_NAME "!GAMEPAD_MODE"
#define MANUAL_CONTROL_BTN_MAPPING_NAME "BTN_"
#define MANUAL_CONTROL_BTN_MAPPING_NAME1 "~~~_"

static uint8_t manual_control_btn_mapping[CHECKBOXITEMS];
static uint8_t manual_control_mode; 

static mavlink_message_t mav_msg;

uint8_t params_count();
mavlink_param_union_t params_get_value(uint8_t component, uint8_t idx);
char *params_get_name(uint8_t component, uint8_t idx);
uint8_t params_get_idx(uint8_t component, char *name);

//======
void qqq() {
	uint8_t i;
	printf("mode: %u\n",manual_control_mode);
	for (i=0;i<mw_box_count();i++)
		printf("%u ",manual_control_btn_mapping[i]);
	printf("\n");
}

void params_init() {
	uint8_t i;

	for (i=0;i<mw_box_count();i++) 
		manual_control_btn_mapping[i] = i;

	manual_control_mode = 0;

#ifdef CFG_ENABLED
	config_setting_t *root = config_root_setting(&cfg);
	config_setting_t *setting = config_lookup(&cfg,"boxmapping");
	config_setting_t *ptr;

	if (setting)
		if (config_setting_length(setting)!=mw_box_count()) {
			config_setting_remove(root,"boxmapping");
			setting = NULL; //ensure the box count has not changed, otherwise we invalidate the config
		}

	if (!setting) {
		printf("boxmapping does not exist. creating\n");
		setting = config_setting_add(root, "boxmapping", CONFIG_TYPE_ARRAY);
		for (i=0;i<mw_box_count();i++) {
			ptr=config_setting_add(setting,NULL,CONFIG_TYPE_INT);
			config_setting_set_int(ptr,manual_control_btn_mapping[i]);
		}
		//setting = ptr;
	}
	
	for (i=0;i<mw_box_count();i++)
		if (setting)
			manual_control_btn_mapping[i] = config_setting_get_int_elem(setting,i);

	setting = config_lookup(&cfg,"gamepad_mode");
	if (!setting) {
		printf("gamepad_mode does not exist. creating\n");
		setting = config_setting_add(root, "gamepad_mode", CONFIG_TYPE_INT);
		config_setting_set_int(setting, 0);
	}

	manual_control_mode = config_setting_get_int(setting);


#endif
	qqq();
}

void params_end() {
#ifdef CFG_ENABLED
	uint8_t i;
	config_setting_t *root = config_root_setting(&cfg);
	config_setting_t *ptr;

	config_setting_t *setting = config_lookup(&cfg,"boxmapping");

	config_setting_remove(root,"boxmapping");

	setting = config_setting_add(root, "boxmapping", CONFIG_TYPE_ARRAY);
	for (i=0;i<mw_box_count();i++) {
		ptr=config_setting_add(setting,NULL,CONFIG_TYPE_INT);
		config_setting_set_int(ptr,manual_control_btn_mapping[i]);
	}

	setting = config_lookup(&cfg,"gamepad_mode");
	config_setting_remove(root,"gamepad_mode");

	setting = config_setting_add(root, "gamepad_mode", CONFIG_TYPE_INT);
	config_setting_set_int(setting, manual_control_mode);
#endif
}

uint8_t params_list_loop(uint8_t reset) {
	static uint8_t step = 0;
	uint8_t param_count,i;

	if (reset) {
		mw_pid_refresh(1);
		step=0;
	}

	switch (step) {
		case 0: //wait for pids
			if (mw_pid_refresh(0)) step++;
			break;
		case 1: //send params
			param_count = params_count();
			for (i=0;i<param_count;i++)
				params_send(200,i);
			return 1;
			break;
	}

	return 0;
}


uint8_t params_count() {
	return mw_pid_count()+1+mw_box_count();
}


void params_send(uint8_t component, uint8_t idx) {
	//reads value of a param at idx
	//send is back
	//we are combining all the params

	mavlink_param_union_t param = params_get_value(component,idx);
	mavlink_msg_param_value_pack(1,component,&mav_msg,params_get_name(component,idx),param.param_float, param.type,params_count(),idx);

	dispatch(&mav_msg);

	//send all pids under COMPONENT 200
	printf("-> param_value id: %u name: %s value: %u. ALL: %u\n",idx,params_get_name(component,idx),param.param_uint8,params_count());


}

void params_set(uint8_t component, char *name, float value) {
	//sets the value of a param at idx
	//send the param back

	uint8_t idx,idx_copy;	
	float old_val;

	mavlink_param_union_t param;

	if (component!=200) {
		printf("params_set: Unsupported component %u\n",component);
		return;
	}


	param.param_float = value;

	idx = params_get_idx(component,name); //resolve the name into id
	idx_copy = idx;
	old_val = params_get_value(component,idx).param_float;

#ifdef CFG_ENABLED

#endif

	if (param.param_float==old_val) { //confirm with send only if we are sure that it is saved
		//if (debug) printf("Value has not changed.\n");
		params_send(component,idx);
		return;
	}

	if (idx<mw_pid_count()) { //pid value
		mw_set_pid(idx,param.param_uint8);
		mw_pid_refresh(1);
		return;
	} else idx-=mw_pid_count();

	if (idx==0) { //manual_control_mode
		manual_control_mode = param.param_uint8;
		params_send(component,idx_copy);
		return;
	} else idx--;
 
	if (idx<mw_box_count()) { //request for box button mapping
		manual_control_btn_mapping[idx] = param.param_uint8;
		params_send(component,idx_copy);
		return;
	}	
	
}

mavlink_param_union_t params_get_value(uint8_t component, uint8_t idx) {
	static mavlink_param_union_t ret;
	ret.param_float = 0.f;

	//idx values:
	//[0 - pid_count] = pid params
	//[pid_count - (pid_count+1)] = manual_control_mode
	//[(pid_count+1) - (pid_counter+1+box_count)] = box mappings


	if (component!=200) {
		printf("params_get_value: Unsupported component %u\n",component);
		return ret;
	}

	if (idx<mw_pid_count()) { //request for pid value
		ret.param_uint8 = mw_get_pid_value(idx);
		ret.type = 	MAV_PARAM_TYPE_UINT8;
		return ret;
	} else idx-=mw_pid_count();


	if (idx==0) { //request for manual_control_mode
		ret.param_uint8 = manual_control_mode;
		ret.type = 	MAV_PARAM_TYPE_UINT8;
		return ret;
	} else idx--;
 
	if (idx<mw_box_count()) { //request for box button mapping
		ret.param_uint8 = manual_control_btn_mapping[idx];
		ret.type = 	MAV_PARAM_TYPE_UINT8;
		return ret;
	}	


	return ret;
}

uint8_t params_manual_control_mapping_get_value(uint8_t idx) {
	return params_get_value(200,mw_pid_count()+1+idx).param_uint8;
}

uint8_t params_manual_control_mode_get_value() {
	return params_get_value(200,mw_pid_count()).param_uint8;
}

char *params_get_name(uint8_t component, uint8_t idx) {
	//see params_get_value for idx meaning
	static char buf[16+1];

	if (component!=200) {
		printf("params_get_name: Unsupported component %u\n",component);
		return NULL;
	}	


	if (idx<mw_pid_count()) { //request for pid name
		return mw_get_pid_name(idx);
	} else idx-=mw_pid_count();


	if (idx==0) { //request for manual_control_mode name
		sprintf(buf,MANUAL_CONTROL_MODE_NAME);
		return buf;
	} else idx--;
 
	if (idx<mw_box_count()) { //request for box button mapping
		sprintf(buf,"%s%s",mw_box_is_supported(idx)?MANUAL_CONTROL_BTN_MAPPING_NAME:MANUAL_CONTROL_BTN_MAPPING_NAME1,mw_get_box_name(idx));
		return buf;
	}

	return NULL;
}
 
uint8_t params_get_idx(uint8_t component, char *name) {
	//see params_get_value for idx meaning
	uint8_t ret;
	char buf[16+1];

	if (component!=200) {
		printf("params_get_name: Unsupported component %u\n",component);
		return 0;
	}	

	ret=mw_get_pid_id(name); //check if name is a pid name
	if (ret!=UINT8_MAX) return ret;

	ret=strcmp(name,MANUAL_CONTROL_MODE_NAME);
	if (ret==0) return mw_pid_count();

	//must be a box

	sprintf(buf,name+4);
	ret=mw_get_box_id(buf); //check if name is a box name
	if (ret!=UINT8_MAX) return mw_pid_count()+1+ret;	

	return UINT8_MAX;
}



