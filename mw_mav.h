//most of the translate happens in the main code; this is just a set of helper functions

#ifndef _MWMAV_H
#define _MWMAV_H

#include <unistd.h>
#include <stdint.h>

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include "mavlink/common/mavlink.h"
#include <mw/msp.h>

void mav_retrieve_init();
void mw_keepalive();

void mav_arm();
void mav_disarm();

void mav_rc(int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll);

void mav_attitude_refresh();
void mav_attitude_quaternions(float *w, float *x, float *y, float *z);

void mav_gps_refresh();
void mav_raw_gps(uint8_t *fix, int32_t *lat, int32_t *lon, int32_t *alt, uint16_t *vel, uint16_t *cog, uint8_t *satellites_visible);

uint8_t mav_param_refresh(uint8_t reset);
uint8_t mav_param_count();
char *mav_get_param_name(uint8_t id);
uint8_t mav_get_param_id(const char *name);
uint8_t mav_get_param_value(uint8_t id);
void mav_set_param(uint16_t id, uint8_t v);


uint16_t get_comm_drop_count();
uint16_t get_comm_drop_rate();

uint32_t mav_sys_status_sensors();
uint8_t mav_type();
uint8_t mav_mode_flag();
uint8_t mav_state();

void mav_toggle_horizon();
void mav_toggle_baro();
void mav_toggle_land();
void mav_toggle_mag();
void mav_toggle_pos_hold();
void mav_toggle_pos_home();
#endif
