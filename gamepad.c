#include "gamepad.h"
#include "mw.h"
#include <stdio.h>
#include <stdlib.h>

#define TRIMDELTA 5

#define BUTTONS_COUNT CHECKBOXITEMS+2+4 //+1 for reset_throttle, +1 for test mapping + 4 for trims

static uint8_t mapping[BUTTONS_COUNT]; 
static uint8_t threshold[3]; //yaw, pitch, roll
static int8_t trim[2]; //0: trim_roll; 1: _pitch, 
/*
	MODE 0: relative control (additive) for throttle
	MODE 1: absolute control
*/
static uint8_t mode; 

void gamepad_init() {
	uint8_t i;
	for (i=0;i<BUTTONS_COUNT;i++)
		mapping[i] = UINT8_MAX;

	trim[0] = 0;
	trim[1] = 0;
}

uint8_t gamepad_button_count() {
	return BUTTONS_COUNT;
}

const char* gamepad_get_button_name(uint8_t i) {
	static char ret[16];
	char *ptr;
	uint8_t c = 0;

	if (i<CHECKBOXITEMS) {
		ptr = mw_get_box_name(i);
		if (mw_box_is_supported(i)) 
			sprintf(ret,"BTN_%s",ptr);
		else
			sprintf(ret,"~BTN_%s",ptr);
		return ret;
	}
	c=CHECKBOXITEMS;
	
	if (i-c++==0) return "BTN_THROT_OFF";

	if (i-c++==0) return "BTN_TEST";

	if (i-c++==0) return "BTN_TRIM_L";

	if (i-c++==0) return "BTN_TRIM_R";

	if (i-c++==0) return "BTN_TRIM_U";

	if (i-c++==0) return "BTN_TRIM_D";

	return "!UNKNOWN";
}

void gamepad_button_pressed(uint8_t i) { //i - 
	uint8_t c = 0;

	if (i<CHECKBOXITEMS) {
		mw_toggle_box(i);
		return;
	}
	c=CHECKBOXITEMS;

	if (i-c++==0) {	gamepad_control_reset_throttle(); return; }

	//if (i-c++==0) {	mw_rth_start(); return; }

	if (i-c++==0) {	initiate_failsafe(); return; }

	if (i-c<2) {
		gamepad_update_trim(0,i-c==0?-TRIMDELTA:TRIMDELTA); 
		return;
	}
	c+=2;

	if (i-c<2) {
		gamepad_update_trim(1,i-c==0?TRIMDELTA:-TRIMDELTA); 
		return;
	}
	c+=2;
}

void gamepad_set_mode(uint8_t *_mode) {
	mode = (*_mode);	
}

void gamepad_get_mode(uint8_t *_mode) {
	(*_mode) = mode;
}

void gamepad_set_mapping(uint8_t *action, uint8_t btn) {
	mapping[btn] = (*action);
}

void gamepad_get_mapping(uint8_t *action, uint8_t btn) {
	(*action) = mapping[btn];
}

void gamepad_set_threshold(uint8_t *value, uint8_t i) {
	threshold[i] = (*value);
}

void gamepad_get_threshold(uint8_t *value, uint8_t i) {
	(*value) = threshold[i];
}

int32_t constrain(int32_t v, int32_t min, int32_t max) {
	if (v<min) return min;
	if (v>max) return max;
	return v;
}

static int16_t throttle = 1000;

void gamepad_update_trim(uint8_t _trim, int8_t delta) { //0 -roll, 1- pitch
	int16_t tmp;
	tmp = trim[_trim] + delta;
	trim[_trim] = constrain(tmp,INT8_MIN,INT8_MAX);
}

void gamepad_control_reset_throttle() {
	uint8_t i;
	throttle = 1000;

	trim[0] = 0;
	trim[1] = 0;

	for (i=0;i<mw_box_count();i++) { //check for box buttons
		mw_box_deactivate(i);
	}	
}

void gamepad_control_calculate(int16_t *_throttle, int16_t *_yaw, int16_t *_pitch, int16_t *_roll) {
	
	int16_t y,r,p;
	y = *_yaw;
	p = *_pitch;
	r = *_roll;
	int16_t throttle_mod = 0; //used in baro mode only

	//throttle: 0-1000
	(*_throttle)-=500; //mid is 0, -500, 500

	if (mode==0) {
		if (is_mode_baro()) { //max throttle: +100, min throttle: -100 temporaily
			if (*_throttle==-500) throttle_mod = -100;
			if (*_throttle==500) throttle_mod = 100;
		} else throttle += (*_throttle)/30; 
	}
	else throttle = 1000+(*_throttle)*2; //mode = 1

	throttle = constrain(throttle,1000,2000);

	//translate between -1000 1000 and mw pwm (1000,2000)
	y = (y/2)+1500;
	r = (r/2)+1500;
	p = (p/2)+1500;

	//check thresholds
	if (abs(y-1500)<threshold[0]) y=1500;
	if (y<1500 && y!=1000) y+= threshold[0];
	if (y>1500 && y!=2000) y-= threshold[0];

	if (abs(r-1500)<threshold[0]) r=1500;
	if (r<1500 && r!=1000) r+= threshold[0];
	if (r>1500 && r!=2000) r-= threshold[0];	

	if (abs(p-1500)<threshold[0]) p=1500;
	if (p<1500 && p!=1000) p+= threshold[0];
	if (p>1500 && p!=2000) p-= threshold[0];	

	*_throttle = throttle + throttle_mod;
	*_yaw = constrain(y,1000,2000);
	*_pitch = constrain(p+trim[1],1000,2000);
	*_roll = constrain(r+trim[0],1000,2000);

	//TODO: constrain
}

