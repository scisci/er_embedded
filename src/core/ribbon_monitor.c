/*
  OSMonitor - Provides Operating System watch dog and crash reports
*/

#include "ribbon_monitor.h"

#include <avr/eeprom.h> 


/* A variable to hold the crash id */
static uint8_t EEMEM crash_id = 0u;

uint8_t osmonitor_wd_enabled = 0u;

void osmonitor_init(void) 
{
	/* Clear WatchDog Timer */
	MCUSR &= ~(1<<WDRF);
	WDTCSR &= ~(1<<WDE);
	wdt_disable();									// Prevent infinite reset
}

/* Refreshes operating system monitor to prevent crashing */
void osmonitor_refresh(void){wdt_reset();}

void osmonitor_begin(void)
{
	osmonitor_start_wd();							// Enable watch dog timer
}


uint8_t osmonitor_read_crash(void)
{
    return eeprom_read_byte(&crash_id); 
}


void osmonitor_start_wd(void)
{
	if(!osmonitor_wd_enabled)
	{
		osmonitor_wd_enabled = 1u;
		wdt_enable(WDTO_8S);
	}
}

void osmonitor_reset(void)
{
	wdt_enable(WDTO_15MS);
	for(;;)
	{
	}
}

void osmonitor_pause_wd(void)
{
	//if(osmonitor_wd_enabled)
	//{
		wdt_disable();
		osmonitor_wd_enabled = 0u;
	//}
}

void osmonitor_crash_point(uint8_t id)
{
	eeprom_write_byte(&crash_id,id);
}
