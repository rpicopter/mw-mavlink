#include "gamepad.h"

#define MANUAL_CONTROL_MODE_NAME "!GAMEPAD_MODE"
#define MANUAL_CONTROL_BTN_MAPPING_NAME "BTN_"
#define MANUAL_CONTROL_BTN_MAPPING_NAME1 "~~~_"

static uint8_t gamepad_mapping[CHECKBOXITEMS];
static uint8_t mode; 

void gamepad_set_mode(uint8_t i, uint8_t _mode) {
	mode = _mode;	
}

uint8_t gamepad_get_mode(uint8_t i) {
	return mode;
}

void gamepad_set_mapping(uint8_t btn, uint8_t action) {
	gamepad_mapping[btn] = action;
}

uint8_t gamepad_get_mapping(uint8_t btn) {
	return gamepad_mapping[btn];
}
