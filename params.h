#ifndef _PARAMS_H_
#define _PARAMS_H_
#include "mavlink/common/mavlink.h"
#include "mw.h"
#include "udp.h"
#include "def.h"


void params_init();
void params_end();

uint8_t params_list_loop(uint8_t reset);

void params_send(uint8_t component, uint8_t idx);

void params_set(uint8_t component, char *name, float value);

uint8_t params_manual_control_mapping_get_value(uint8_t idx);

uint8_t params_manual_control_mode_get_value();

#endif