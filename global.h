#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include "def.h"

#define CFG_FILE "/usr/local/etc/mw/mw-mavlink.cfg"

void mssleep(unsigned int ms);

extern uint16_t heartbeat;

#ifdef CFG_ENABLED
#include <libconfig.h>
extern config_t cfg;
#endif

#define RPICAM_ENABLED 1

#define LOOP_MS 25

#endif
