/* Ribbon Prototype 1.0 Board. This is designed for the first prototype vest
 * which contains 8 keys on the left side and one slider on the right side */

#define KEY_CHANNEL CHANNEL_0  // The channel where the main set of 8 keys start
#define SLIDER_CHANNEL CHANNEL_24  // The channel where the first slider starts

#include "ribbon_api.h"
#include "ribbon_event.h"
#include "ribbon_globals.h"

static void configure_sensors(void);

#ifndef RIBBON_DEF_PROCESSING
static uint8_t process_sensors(uint16_t status);
#endif

#ifndef RIBBON_DEF_FILTERING
static void filter_sensors(void);
#endif

DEF_GLOBAL_PARAM_MAP(10, ER_ADDRESS_PARAM, ER_DI_PARAM, ER_MAX_ON_PARAM,
                     ER_POS_DRIFT_PARAM, ER_NEG_DRIFT_PARAM,
                     ER_DRIFT_HOLD_PARAM, ER_RECAL_PARAM,
                     ER_POS_RECAL_DELAY_PARAM, ER_DATA_INTERVAL_PARAM,
                     ER_MEASURE_PERIOD_MS_PARAM);

DEF_SENSOR_PARAM_MAP(3, ER_THRESHOLD_PARAM, ER_HYSTERESIS1_PARAM,
                     ER_HYSTERESIS2_PARAM);

/*----------------------------------------------------------------------------
  main
----------------------------------------------------------------------------*/
int main(void) {
  ribbon_configure_callback = configure_sensors;
  // ribbon_status_callback = process_sensors;
  // ribbon_filter_callback = filter_sensors;

  ribbon_main();
}

/*============================================================================
Name    :   configure_sensors
------------------------------------------------------------------------------
Purpose :   Configure the Sensors as keys and Rotor/Sliders for 64 channels only
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/

static void configure_sensors(void) {
  // Enable 8 key bank
  enable_key(KEY_CHANNEL + 0, NO_AKS_GROUP);
  enable_key(KEY_CHANNEL + 1, NO_AKS_GROUP);
  enable_key(KEY_CHANNEL + 2, NO_AKS_GROUP);
  enable_key(KEY_CHANNEL + 3, NO_AKS_GROUP);
  enable_key(KEY_CHANNEL + 4, NO_AKS_GROUP);
  enable_key(KEY_CHANNEL + 5, NO_AKS_GROUP);
  enable_key(KEY_CHANNEL + 6, NO_AKS_GROUP);
  enable_key(KEY_CHANNEL + 7, NO_AKS_GROUP);

  // Enable slider
  enable_slider(SLIDER_CHANNEL, SLIDER_CHANNEL + 7, NO_AKS_GROUP);
}

#ifndef RIBBON_DEF_PROCESSING
/*============================================================================
Name    :   process_sensors
------------------------------------------------------------------------------
Purpose :   Process sensor statusand create events to be broadcasted
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
uint8_t process_sensors(uint8_t status_flag) { return 0; }
#endif

#ifndef RIBBON_DEF_FILTERING
/*============================================================================
Name    :   filter_sensors
------------------------------------------------------------------------------
Purpose :   Optionally filter sensor data before it reaches the library
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
static void filter_sensors(void) {}
#endif
