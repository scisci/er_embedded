/*
  OSMonitor.h - Provides watchdog and crash reports
*/

#ifndef OSMONITOR_h
#define OSMONITOR_h

#include "avr/wdt.h"

extern uint8_t osmonitor_wd_enabled;

/* Inits the Operating System Monitor */
void osmonitor_init(void);
/* Begins operating system monitor for loop code */
void osmonitor_begin(void);
/* Starts operating system monitoring */
void osmonitor_start_wd(void);
/* Pauses operating system watch dog */
void osmonitor_pause_wd(void);
/* Refreshes operating system monitor to prevent crashing */
void osmonitor_refresh(void);
/* Resets the device by setting a short timeout and entering a for loop */
void osmonitor_reset(void);
/* Set's a crash point in case of crash */
void osmonitor_crash_point(uint8_t id);
/* Reads the last set crash point */
uint8_t osmonitor_read_crash(void);

#endif  // OSMONITOR_h
