#ifndef RIBBON_PARAM_h
#define RIBBON_PARAM_h

/*----------------------------------------------------------------------------
  this only contains the parameter definitions that are shared between
ribbon_globals.h and ribbon_api.h
----------------------------------------------------------------------------*/

// Custom config allows a custom sensor configuration to be loaded, otherwise
// the default configuration is used
#ifndef ER_CUSTOM_CONFIG_DEF
#define ER_CUSTOM_CONFIG_DEF 0u
#endif

#ifndef ER_ADDRESS_DEF
#define ER_ADDRESS_DEF 1
#endif

#ifndef ER_DI_DEF
#define ER_DI_DEF 2
#endif

#ifndef ER_NEG_DRIFT_DEF
#define ER_NEG_DRIFT_DEF 20
#endif

#ifndef ER_POS_DRIFT_DEF
#define ER_POS_DRIFT_DEF 7
#endif

#ifndef ER_MAX_ON_DEF
#define ER_MAX_ON_DEF 127  // Max On of ~25 seconds (200ms increments)
#endif

#ifndef ER_DRIFT_HOLD_DEF
#define ER_DRIFT_HOLD_DEF 20
#endif

// Recalibrates if greater than 1/2 of threshold but less than threshold
#ifndef ER_RECAL_DEF
#define ER_RECAL_DEF 1
#endif

#ifndef ER_THRESHOLD_DEF
#define ER_THRESHOLD_DEF 16u
#endif

// Detect hysteresis (set to zero to have the fastest key off)
#ifndef ER_HYSTERESIS1_DEF
#define ER_HYSTERESIS1_DEF 0u
#endif

// Position hysteresis
#ifndef ER_HYSTERESIS2_DEF
#define ER_HYSTERESIS2_DEF 2u
#endif

#ifndef ER_RESOLUTION_DEF
#define ER_RESOLUTION_DEF 7
#endif

#ifndef ER_BURST_LENGTH_DEF
#define ER_BURST_LENGTH_DEF 32
#endif

#ifndef ER_DATA_INTERVAL_DEF
#define ER_DATA_INTERVAL_DEF 0
#endif

#ifndef ER_BURST_LENGTH_DEF
#define BURST_LENGTH_DEF 48
#endif

#ifndef ER_MAX_DELTA_DEF
#define ER_MAX_DELTA_DEF MAX_DELTA_12_5
#endif

#ifndef ER_MEASURE_PERIOD_MS_DEF
#define ER_MEASURE_PERIOD_MS_DEF 25
#endif

// Positive recal seems to mess up when on body, it is the number
// of cycles that a sensor needs to be either negative, or between
// RECAL and THRESHOLD
#ifndef ER_POS_RECAL_DELAY_DEF
#define ER_POS_RECAL_DELAY_DEF 1
#endif

typedef enum tag_max_delta_t {
  MAX_DELTA_100,   // Deltas can be up to ref
  MAX_DELTA_75,    // Deltas can be up to 3/4 ref
  MAX_DELTA_50,    // Deltas can be up to 1/2 ref
  MAX_DELTA_25,    // Deltas can be up to 1/4 ref
  MAX_DELTA_12_5,  // Deltas can be up to 1/8 ref
} max_delta_t;

// Available global parameters

enum ribbon_global_params_t {
  ER_ADDRESS_PARAM,
  ER_DI_PARAM,
  ER_MAX_ON_PARAM,
  ER_POS_DRIFT_PARAM,
  ER_NEG_DRIFT_PARAM,
  ER_DRIFT_HOLD_PARAM,
  ER_POS_RECAL_DELAY_PARAM,
  ER_RECAL_PARAM,
  ER_DATA_INTERVAL_PARAM,
  ER_MEASURE_PERIOD_MS_PARAM,
  ER_MAX_GLOBAL_PARAMS
};

// Initializing array for eeprom
#define ER_GLOBAL_PARAM_DEFS()                                                 \
  {                                                                            \
    ER_ADDRESS_DEF, ER_DI_DEF, ER_MAX_ON_DEF, ER_POS_DRIFT_DEF,                \
        ER_NEG_DRIFT_DEF, ER_DRIFT_HOLD_DEF, ER_RECAL_DEF,                     \
        ER_POS_RECAL_DELAY_DEF, ER_DATA_INTERVAL_DEF, ER_MEASURE_PERIOD_MS_DEF \
  }

// Available parameters per sensor

enum ribbon_sensor_params_t {
  ER_THRESHOLD_PARAM,
  ER_HYSTERESIS1_PARAM,
  ER_HYSTERESIS2_PARAM,
  ER_RESOLUTION_PARAM,
  ER_MAX_SENSOR_PARAMS
};

#define ER_SENSOR_PARAM_DEFS()                                \
  {                                                           \
    ER_THRESHOLD_DEF, ER_HYSTERESIS1_DEF, ER_HYSTERESIS2_DEF, \
        ER_RESOLUTION_DEF                                     \
  }

#endif  // RIBBON_PARAM_h
