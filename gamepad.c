#include "gamepad.h"
#include "mw.h"

static uint8_t mapping[CHECKBOXITEMS+1]; //+1 for reset_throttle
static uint8_t threshold[3]; //yaw, pitch, roll

/*
	MODE 0: relative control (additive) for throttle
	MODE 1: absolute control
*/
static uint8_t mode; 

void gamepad_init() {
	uint8_t i;
	for (i=0;i<CHECKBOXITEMS+1;i++)
		mapping[i] = UINT8_MAX;
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

static int16_t throttle = 1000;

void gamepad_control_reset_throttle() {
	uint8_t i;
	throttle = 1000;

	for (i=0;i<mw_box_count();i++) { //check for box buttons
		mw_box_deactivate(i);
	}	
}

void gamepad_control_calculate(int16_t *_throttle, int16_t *_yaw, int16_t *_pitch, int16_t *_roll) {
	
	int16_t y,r,p;
	y = *_yaw;
	p = *_pitch;
	r = *_roll;

	(*_throttle)-=500; //mid is 0

	if (mode==0) throttle += (*_throttle)/30; //0-1000 (500 mid)
	else throttle = 1000+(*_throttle)*2; //mode = 1

	if (throttle>2000) throttle=2000;
	if (throttle<1000) throttle=1000;

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

	*_throttle = throttle;
	*_yaw = y;
	*_pitch = p;
	*_roll = r;
}

