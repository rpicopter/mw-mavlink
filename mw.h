
//most of the translate happens in the main code; this is just a set of helper functions

#ifndef _MW_H
#define _MW_H

#include <unistd.h>
#include <stdint.h>
#include "def.h"

#include <mw/msp.h>
#include <mw/shm.h>

void mw_loop();
uint8_t mw_init();
void mw_end();

void mw_arm();
void mw_disarm();

void mw_manual_control(int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll);

void mw_altitude(int32_t *alt);
void mw_attitude_quaternions(float *w, float *x, float *y, float *z);
void mw_raw_gps(uint8_t *fix, int32_t *lat, int32_t *lon, int32_t *alt, uint16_t *vel, uint16_t *cog, uint8_t *satellites_visible);

uint8_t mw_box_count();
char *mw_get_box_name(uint8_t id);
uint8_t mw_get_box_id(const char *name);
uint8_t mw_box_is_supported(uint8_t id);

uint8_t mw_pid_refresh(uint8_t reset);
uint8_t mw_pid_count();
char *mw_get_pid_name(uint8_t id);
uint8_t mw_get_pid_id(const char *name);
uint8_t mw_get_pid_value(uint8_t id);
void mw_set_pid(uint8_t id, uint8_t v);
void mw_get_signal(int8_t *rssi, int8_t *noise);
uint16_t mw_get_comm_drop_count();
uint16_t mw_get_comm_drop_rate();

uint32_t mw_sys_status_sensors();
uint8_t mw_type();
uint8_t mw_mode_flag();
uint8_t mw_state();

void mw_rth_start();
void mw_toggle_box(uint8_t i);
void mw_box_activate(uint8_t i);
void mw_box_deactivate(uint8_t i);

#endif
