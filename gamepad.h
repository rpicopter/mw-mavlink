#ifndef _GAMEPAD_H_
#define _GAMEPAD_H_

#include <stdint.h>
#include <mw/msp.h>
#include <stdlib.h>


void gamepad_init();

void gamepad_set_mode(uint8_t i, uint8_t mode);
uint8_t gamepad_get_mode(uint8_t i);

void gamepad_set_mapping(uint8_t btn, uint8_t action);
uint8_t gamepad_get_mapping(uint8_t btn);

void gamepad_set_threshold(uint8_t i, uint8_t t);
uint8_t gamepad_get_threshold(uint8_t i);

void gamepad_control_calculate(int16_t *throttle, int16_t *yaw, int16_t *pitch, int16_t *roll);
void gamepad_control_reset_throttle();

#endif
