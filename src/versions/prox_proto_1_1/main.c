
#define KEY_CHANNEL CHANNEL_3  // The channel where the main set of 8 keys start
#define SLIDER_CHANNEL CHANNEL_8  // The channel where the first slider starts

#include "ribbon_api.h"
#include "ribbon_event.h"
#include "ribbon_globals.h"

static void configure_sensors(void);

#ifndef RIBBON_DEF_PROCESSING
static void process_sensors(uint16_t status);
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

#define QOT(b) #b
#define QOT1(b) QOT(b)
#define QOT2(b) QOT1(b)
/*----------------------------------------------------------------------------
  main
----------------------------------------------------------------------------*/
int main(void) {
  ribbon_configure_callback = configure_sensors;

  // const char name[] = QOT1(BURST_FUNC_NAME);

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
  enable_key(CHANNEL_3, NO_AKS_GROUP);
  enable_key(CHANNEL_4, NO_AKS_GROUP);

  qt_enable_slider(CHANNEL_5, CHANNEL_7, NO_AKS_GROUP, 16u, HYST_6_25,
                   RES_8_BIT, 0u);
  // Create a slider with delta area between 4-8 (maximum proximity)
  //	enable_slider( 	CHANNEL_8,
  //					CHANNEL_10,
  //					NO_AKS_GROUP);
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
void process_sensors(uint16_t status_flag) {}

#endif  // RIBBON_DEF_PROCESSING

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
