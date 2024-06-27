
#include "ribbon_api.h"
#include "ribbon_event.h"
#include "ribbon_monitor.h"

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <util/delay.h>

/*----------------------------------------------------------------------------
                                timing vars
----------------------------------------------------------------------------*/

#define TICKS_4MHZ (500u * ribbon_library_info.measurement_period_ms)
#define TICKS_16MHZ (2000u * ribbon_library_info.measurement_period_ms)

#ifdef _QMATRIX_

extern y_line_info_t y_line_info[NUM_Y_LINES];
extern x_line_info_t x_line_info[NUM_X_LINES];

/* Fill out the X-Line masks on the X- Port selected.
 * The order of X - Line numbering follows from the way the
 * X-Lines are filled as below.
 * Here, X0,X1,X2,X3,X4,X5,X6,X7 are on port-pin specified.
 * 1 - if to specify if X line is on PORT_X_1,pin on the same selected port.
 * 2 - if to specify if X line is on PORT_X_2,pin on the same selected port.
 * 3 - if to specify if X line is on PORT_X_3,pin on the same selected port.
 *
 * Note: 1. The Number entries should be based on NUM_X_LINES
 *          4 entries when NUM_X_LINES =4 and
 *          8 entries when NUM_X_LINES=8
 */

x_line_info_t x_line_info[NUM_X_LINES] = {
    FILL_OUT_X_LINE_INFO(1, 0u), FILL_OUT_X_LINE_INFO(1, 1u),
    FILL_OUT_X_LINE_INFO(1, 2u), FILL_OUT_X_LINE_INFO(1, 3u),
    FILL_OUT_X_LINE_INFO(1, 4u), FILL_OUT_X_LINE_INFO(1, 5u),
    FILL_OUT_X_LINE_INFO(1, 6u), FILL_OUT_X_LINE_INFO(1, 7u),
};
/* Fill out the Y-Line masks on the Y- Line port selected
 * The order of Y - Line numbering follows from the way the
 * Y-Lines are filled as below
 * Here, Y0,Y1,Y2,Y3 on 0,1,2,3
 * Note: 1. The Number entries should be based on NUM_X_LINES
 *          2 entries when NUM_Y_LINES=2
 *          4 entries when NUM_Y_LINES=4
 *          8 entries when NUM_Y_LINES=8
 */
y_line_info_t y_line_info[NUM_Y_LINES] = {
    // FILL_OUT_Y_LINE_INFO(  0u ),
    // FILL_OUT_Y_LINE_INFO(  1u ),
    FILL_OUT_Y_LINE_INFO(2u), FILL_OUT_Y_LINE_INFO(3u),
    FILL_OUT_Y_LINE_INFO(4u), FILL_OUT_Y_LINE_INFO(5u),
    FILL_OUT_Y_LINE_INFO(6u), FILL_OUT_Y_LINE_INFO(7u),
};

#endif  //_QMATRIX

/*----------------------------------------------------------------------------
                                extern variables
----------------------------------------------------------------------------*/
/* This configuration data structure parameters if needs to be changed will be
   changed in the qt_set_parameters function */
extern qt_touch_lib_config_data_t qt_config_data;
/* measurement data */
extern qt_touch_lib_measure_data_t qt_measure_data;
/* Get sensor delta values */
extern int16_t qt_get_sensor_delta(uint8_t sensor);

/* Returns the current measured channel reference */
#define GET_CHANNEL_REF(ch) qt_measure_data.channel_references[(ch)]
/* Returns the last measured channel reference */
#define GET_LAST_CHANNEL_REF(ch) ribbon_channel_data[(ch)].reference
/* Records the current measured channel reference to the measured position */
#define UPDATE_CHANNEL_REF(ch)           \
  (ribbon_channel_data[(ch)].reference = \
       qt_measure_data.channel_references[(ch)])

/* Returns the current measured channel delta (ref-sig) */
#define GET_CHANNEL_DELTA(ch)                 \
  (qt_measure_data.channel_references[(ch)] - \
   qt_measure_data.channel_signals[(ch)])
/* Returns the last measured channel delta after being clipped */
#define GET_LAST_CHANNEL_DELTA(ch) ribbon_channel_data[(ch)].delta
/* Updates the last measured channel delta */
#define UPDATE_CHANNEL_DELTA(ch, delta) \
  (ribbon_channel_data[(ch)].delta = delta)
/* Returns the current key reference */
#define GET_KEY_REF(s) GET_CHANNEL_REF(sensors[(s)].from_channel)
/* Returns the last key reference */
#define GET_LAST_KEY_REF(s) GET_LAST_CHANNEL_REF(sensors[(s)].from_channel)
/* Updates the key reference */
#define UPDATE_KEY_REF(s) UPDATE_CHANNEL_REF(sensors[(s)].from_channel)

/* Quick line to get the channel used by a key */
#define KEY_CH(s) sensors[(s)].from_channel

#define GET_KEY_DELTA(s) GET_CHANNEL_DELTA(sensors[(s)].from_channel)
#define GET_LAST_KEY_DELTA(s) GET_LAST_CHANNEL_DELTA(sensors[(s)].from_channel)
#define UPDATE_KEY_DELTA(s, d) \
  UPDATE_CHANNEL_DELTA(sensors[(s)].from_channel, d)

#define SNS_PARAM_ADR(s, p) &sensor_params[ER_MAX_SENSOR_PARAMS * (s) + (p)]

#define ER_DEFAULT_LOOP ER_PERFORM_LOOP

#define ER_DEBUG_DATA_FLAG 0x10
#define ER_DEBUG_STATUS_FLAG 0x20

#define CLAMP_INT16_TO_INT8(x)     \
  (((int16_t)(x) > (int16_t)(127)) \
       ? (int8_t)(127)             \
       : (((int16_t)(x) < (int16_t)(-127)) ? (int8_t)(-127) : (int8_t)(x)))

//----------------------------------------------------------------------------
//	extern methods from ribbon_event.h
//----------------------------------------------------------------------------
extern void broadcast(uint8_t message_type);
extern void broadcast_mode(void);

//----------------------------------------------------------------------------
//	extern methods from uart.h
//----------------------------------------------------------------------------
extern uint8_t uart_unbuffer_rx(void);
extern uint8_t uart_rx_avail(void);

//----------------------------------------------------------------------------
//	extern methods from ribbon_monitor.h
//----------------------------------------------------------------------------
extern void osmonitor_init(void);
extern void osmonitor_begin(void);
extern void osmonitor_refresh(void);

//----------------------------------------------------------------------------
//	Parameter variables
//----------------------------------------------------------------------------
// Global parameter count should be set via a macro from main.c
extern uint8_t global_param_count;
// Sensor parameter count, should be set via a macro from main.c
extern uint8_t sensor_param_count;
// Global parameter map should be set via a macro from main.c
extern uint8_t global_param_map[];
// Sensor parameter map should be set via a macro from main.c
extern uint8_t sensor_param_map[];

//----------------------------------------------------------------------------
//	EEPROM variables
//----------------------------------------------------------------------------
// A flag that describes whether eeprom values are corrupted or not
uint8_t eeprom_valid = 0u;
// A byte to check for eeprom corruption
uint8_t EEMEM eeprom_validator0 = 'S';
// A byte to check for eeprom corruption
uint8_t EEMEM eeprom_validator1 = 'C';
// The global parameter save space, initialized to defaults
uint8_t EEMEM global_params[ER_MAX_GLOBAL_PARAMS] = ER_GLOBAL_PARAM_DEFS();
// The sensor parameter save space, not initialized
uint8_t EEMEM sensor_params[ER_MAX_SENSOR_PARAMS * ER_MAX_SENSORS];

uint8_t EEMEM custom_config = ER_CUSTOM_CONFIG_DEF;

// Holds the number of sensors in the current configuration
uint8_t EEMEM sensor_config_count = 0u;
// Holds the sensor types in the current configuration, type, from, to
uint8_t EEMEM sensor_config[ER_MAX_SENSORS * 3u];

#ifdef _QMATRIX_
// The burst length save space, not initialized
uint8_t EEMEM ch_burst_lengths[QT_NUM_CHANNELS];
#endif

// The sensor parameter defaults used if the eeprom is corrupted
uint8_t sensor_param_defaults[ER_MAX_SENSOR_PARAMS] = ER_SENSOR_PARAM_DEFS();

/*----------------------------------------------------------------------------
        public virtual methods
----------------------------------------------------------------------------*/
uint8_t (*ribbon_status_callback)(uint16_t status) = 0;
void (*ribbon_data_callback)(uint16_t status) = 0;
void (*ribbon_filter_callback)(void) = 0;
void (*ribbon_configure_callback)(void) = 0;

/*----------------------------------------------------------------------------
        private methods
----------------------------------------------------------------------------*/
/*  initialise host app, pins, watchdog, etc    */
static void ribbon_init(void);
/*  configure timer ISR to fire regularly   */
static void init_timer_isr(void);
/*  Assign the parameters values to global configuration parameter structure */
static void set_parameters(void);
/* Configure the sensors */
static void config_sensors(void);
/* The main loop for sending both status and data information */
static void data_loop(void);
/* The main loop for sending only status info */
static void perform_loop(void);
/* The fallback loop for debugging problems */
static void debug_loop(void);
static void command_loop(void);
/* Called before beginning execution to notify via serial */
static void notify_init();

#ifdef DYNAMIC_CLOCKING
/* Change mcu clock to high speed */
static void enable_serial_clock(void);
/* Change mcu clock to low speed */
static void disable_serial_clock(void);

#endif  // DYNAMIC_CLOCKING

#ifdef RIBBON_DEF_PROCESSING
/* Process sensor status data */
static uint8_t default_ribbon_status_callback(uint16_t status);
/* Process sensor extra data */
static void default_ribbon_data_callback(uint16_t status);

#endif  // RIBBON_DEF_PRECESSING

/*----------------------------------------------------------------------------
        private variables
----------------------------------------------------------------------------*/

#ifdef USE_8BIT_TIMER
/* Number of times 8-bit timer has rolled over */
static volatile uint8_t timer_cycle_count = 0u;
static volatile uint8_t timer_cycle = 0u;
static volatile uint8_t timer_remainder = 0u;
#endif  // USE_8BIT_TIMER

/* Number of sensors initialized */
uint8_t sensor_count = 0u;

uint8_t loaded_sensor_count = 0u;

#if (ER_MAX_SLIDERS > 0)
/* Number of sliders initialized */
uint8_t slider_count = 0;
#endif  // NUM_SLIDERS>0

/* Running mode bits 0:rsv,0:rsv,0:rsv,0:rsv,0:rsv,0:rsv,1:dispatch data
 * messages,1:dispatch status messages */
uint8_t mode_flags = ER_PERFORM_LOOP;

/*============================================================================
Name    :   get_mapped_global_param
------------------------------------------------------------------------------
Purpose :   returns the actual value of the global parameter as it has been set
from the last reset Input   :   (uint8_t)index: parameter index Output  :   n/a
Notes   :
============================================================================*/
uint8_t get_mapped_global_param(uint8_t index) {
  uint8_t value = 0;

  if (index < global_param_count) {
    switch (global_param_map[index]) {
      case ER_ADDRESS_PARAM:
        value = ribbon_library_info.address;
        break;
      case ER_DI_PARAM:
        value = qt_config_data.qt_di;
        break;
      case ER_MAX_ON_PARAM:
        value = qt_config_data.qt_max_on_duration;
        break;
      case ER_POS_DRIFT_PARAM:
        value = qt_config_data.qt_pos_drift_rate;
        break;
      case ER_NEG_DRIFT_PARAM:
        value = qt_config_data.qt_neg_drift_rate;
        break;
      case ER_RECAL_PARAM:
        value = qt_config_data.qt_recal_threshold;
        break;
      case ER_POS_RECAL_DELAY_PARAM:
        value = qt_config_data.qt_pos_recal_delay;
        break;
      case ER_DRIFT_HOLD_PARAM:
        value = qt_config_data.qt_drift_hold_time;
        break;
      case ER_DATA_INTERVAL_PARAM:
        value = ribbon_library_info.data_interval;
        break;
      case ER_MEASURE_PERIOD_MS_PARAM:
        value = ribbon_library_info.measurement_period_ms;
        break;

      default:
#if (RIBBON_DEBUG_MODE == DEBUG_EVENT)
        uart_string_P(PSTR("UNKNOWN PARAM!"), 16);
#endif
        value = 0;
    }
  } else {
#if (RIBBON_DEBUG_MODE == DEBUG_EVENT)
    uart_string_P(PSTR("PARAM OUR OF RANGE!"), 19);
#endif
  }

  return value;
}

/*============================================================================
Name    :   set_mapped_global_param
------------------------------------------------------------------------------
Purpose :   sets a global parameter in memory
Input   :   (uint8_t)index: parameter index into mapped array, (uint8_t) value:
parameter value Output  :   n/a Notes   :	does not change the library
until reset
============================================================================*/
bool set_mapped_global_param(uint8_t index, uint8_t value) {
  if (index < global_param_count) {
    eeprom_write_byte(&global_params[global_param_map[index]], value);
    return true;
  }

  return false;
}

/*============================================================================
Name    :   get_mapped_sensor_param
------------------------------------------------------------------------------
Purpose :   returns the actual value of the sensor parameter as it has been set
from the last reset Input   :   (uint8_t)sensor: sensor index, (uint8_t)index:
parameter index Output  :   n/a Notes   :
============================================================================*/
uint8_t get_mapped_sensor_param(uint8_t sensor, uint8_t index) {
  if (index < sensor_param_count && sensor < ER_MAX_SENSORS) {
    switch (sensor_param_map[index]) {
      case ER_THRESHOLD_PARAM:
        return sensors[sensor].threshold;
      case ER_HYSTERESIS1_PARAM:
        return (sensors[sensor].type_aks_pos_hyst & 0x03);

#if (NUM_SLIDERS > 0)
      case ER_HYSTERESIS2_PARAM:
        return slider_params[sensors[sensor].index].position_hysteresis;
      case ER_RESOLUTION_PARAM:
        return slider_params[sensors[sensor].index].position_resolution;
#endif
      default:
        return 0;
    }
  }

  return 0;
}

/*============================================================================
Name    :   set_mapped_sensor_param
------------------------------------------------------------------------------
Purpose :   sets a sensor parameter in memory
Input   :   (uint8_t)sensor: sensor index, (uint8_t)index: parameter index,
(uint8_t) value: parameter value Output  :   n/a Notes   :	does not change
the sensor until reset
============================================================================*/
bool set_mapped_sensor_param(uint8_t sensor, uint8_t index, uint8_t value) {
  if (index < sensor_param_count && sensor < ER_MAX_SENSORS) {
    eeprom_write_byte(
        &sensor_params[(uint16_t)sensor * ER_MAX_SENSOR_PARAMS + index], value);
    return true;
  }

  return false;
}

/*============================================================================
Name    :   set_mapped_channel_param
------------------------------------------------------------------------------
Purpose :   sets a channel parameter in memory
Input   :   (uint8_t)sensor: sensor index, (uint8_t)channel: channel index,
(uint8_t) value: parameter value Output  :   n/a Notes   :	can only set
burst lengths for now
============================================================================*/
bool set_mapped_channel_param(uint8_t sensor, uint8_t channel, uint8_t value) {
  if (sensor < ER_MAX_SENSORS) {
    uint8_t abs_ch = channel + sensors[sensor].from_channel;

    if (abs_ch <= sensors[sensor].to_channel) {
#ifdef _QMATRIX_
      eeprom_write_byte(&ch_burst_lengths[abs_ch], value);
      return true;
#endif
    }
  }

  return false;
}

/*============================================================================
Name    :   get_mapped_channel_param
------------------------------------------------------------------------------
Purpose :   gets a channel parameter in memory
Input   :   (uint8_t)sensor: sensor index, (uint8_t)channel: channel index
Output  :   n/a
Notes   :	can only get burst lengths for now
============================================================================*/
uint8_t get_mapped_channel_param(uint8_t sensor, uint8_t channel) {
  if (sensor < ER_MAX_SENSORS) {
    uint8_t abs_ch = channel + sensors[sensor].from_channel;

    if (abs_ch <= sensors[sensor].to_channel) {
#ifdef _QMATRIX_
      return qt_burst_lengths[abs_ch];
#else
      return 0;
#endif
    }
  }

  return 0;
}

/*============================================================================
Name    :   ribbon_hard_reset
------------------------------------------------------------------------------
Purpose :   resets the device hardware
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/
void ribbon_hard_reset(void) {
  osmonitor_reset();  // Tell the watchdog to reset the chip
}

void ribbon_recal(void) { qt_calibrate_sensing(); }

void ribbon_eeprom_reset(void) {
  uint16_t i;
#ifdef RIBBON_DEBUG_ON
  uart_string_P(PSTR("EEPROM DEFAULTING!\r\n"), 20);
#endif
  uint8_t global_param_defaults[ER_MAX_GLOBAL_PARAMS] = ER_GLOBAL_PARAM_DEFS();

  // Reset global parameters
  for (i = 0; i < ER_MAX_GLOBAL_PARAMS; i++) {
    eeprom_write_byte(&global_params[i], global_param_defaults[i]);
  }

  // Reset sensor parameters
  for (i = 0; i < ER_MAX_SENSORS * ER_MAX_SENSOR_PARAMS; i++) {
    eeprom_write_byte(&sensor_params[i],
                      sensor_param_defaults[i % ER_MAX_SENSOR_PARAMS]);
  }

  // Reset custom configuration count
  eeprom_write_byte(&sensor_config_count, 0);

  // Reset custom configuration params
  for (i = 0; i < ER_MAX_SENSORS; i++) {
    eeprom_write_byte(&sensor_config[i], 0);
  }

  // Reset burst length defaults
#ifdef _QMATRIX_
  for (i = 0; i < QT_NUM_CHANNELS; i++) {
    eeprom_write_byte(&ch_burst_lengths[i], ER_BURST_LENGTH_DEF);
  }
#endif

  // Reset custom configuration flags
  eeprom_write_byte(&custom_config, ER_CUSTOM_CONFIG_DEF);
}

/*============================================================================
Name    :   main
------------------------------------------------------------------------------
Purpose :   main code entry point
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/

void ribbon_main(void) {
  current_time_ms_touch = 0u;
  time_to_measure_touch = 0u;

  ribbon_library_info.pid[0] = DEVICE_PID1;  // Store device id in memory
  ribbon_library_info.pid[1] = DEVICE_PID2;
  ribbon_library_info.hid = DEVICE_HID;
  ribbon_library_info.fid = DEVICE_FID;

  ribbon_init();  // Init pins, watchdog etc.
  notify_init();  // Send any serial data for init

  qt_filter_callback = ribbon_filter_callback;

#ifdef RIBBON_DEF_PROCESSING
  ribbon_status_callback = default_ribbon_status_callback;
  ribbon_data_callback = default_ribbon_data_callback;
#endif  // RIBBON_DEF_PROCESSING

  config_sensors();   // Setup sensor params
  qt_init_sensing();  // Start touch sensing
  set_parameters();   // Set general params
  init_timer_isr();   // Start timer for ms watch (must come after
                      // set parameters since that is when measurement
                      // period is set

  osmonitor_begin();  // Begin the watchdog
  sei();

  // Start mode change loop
  for (;;) {
    broadcast_mode();  // Broadcast the mode change

    switch (RIBBON_MODE) {
      case ER_DATA_LOOP:
#ifdef RIBBON_DEBUG_ON
        uart_string_P(PSTR("Data Loop\r\n"), 11);
#endif  // RIBBON_DEBUG_ON
        data_loop();
        break;

      case ER_PERFORM_LOOP:
#ifdef RIBBON_DEBUG_ON
        uart_string_P(PSTR("Perform Loop\r\n"), 14);
#endif
        perform_loop();
        break;

      case ER_DEBUG_LOOP:
#ifdef RIBBON_DEBUG_ON
        uart_string_P(PSTR("Debug Loop\r\n"), 12);
#endif
        debug_loop();
        break;

      case ER_CMD_LOOP:
#ifdef RIBBON_DEBUG_ON
        uart_string_P(PSTR("Command Loop\r\n"), 14);
#endif
        command_loop();

        break;

      default:
#ifdef RIBBON_DEBUG_ON
        uart_string_P(PSTR("Invalid Mode!\r\n"), 15);
#endif
        SET_RIBBON_MODE(ER_DEFAULT_LOOP);
        break;
    }

    osmonitor_refresh();  // Refresh the watchdog
  }
}

void command_loop(void) {
  while (RIBBON_MODE == ER_CMD_LOOP) {
    while (uart_rx_avail()) {
      parse_rx_event();
    }

    osmonitor_refresh();
  }
}

bool ribbon_set_sensor(uint8_t sensor_index, uint8_t sensor_type, uint8_t from,
                       uint8_t to) {
  if (sensor_index < ER_MAX_SENSORS) {
    uint8_t eeprom_idx = sensor_index * 3u;

    eeprom_write_byte(&sensor_config[eeprom_idx], sensor_type);
    eeprom_write_byte(&sensor_config[eeprom_idx], from);
    eeprom_write_byte(&sensor_config[eeprom_idx], to);
    return true;
  }

  return false;
}

bool ribbon_set_sensors(uint8_t sensors) {
  if (sensors < ER_MAX_SENSORS) {
    eeprom_write_byte(&sensor_config_count, sensors);

    // Clear the custom configuration
    if (sensors == 0) {
      eeprom_write_byte(&custom_config, 0);
    }
    return true;
  }

  return false;
}

/*============================================================================
Name    :   data_loop
------------------------------------------------------------------------------
Purpose :   main loop for maximum data acquisition such as calibration or
visualization Input   :   n/a Output  :   n/a Notes   :
============================================================================*/

void data_loop(void) {
  uint16_t status_flag;
  uint16_t burst_flag;
  uint8_t status_response;
  uint8_t data_timer = 0u;

  while (RIBBON_MODE == ER_DATA_LOOP) {
    if (time_to_measure_touch) {
      time_to_measure_touch = 0u;  //  clear flag: it's time to measure touch

      do {
        status_flag = qt_measure_sensors(current_time_ms_touch);
        burst_flag = status_flag & QTLIB_BURST_AGAIN;

        // Process sensor results
        status_response = ribbon_status_callback(status_flag);

        // If we need to notify of performance data, do it
        if (event_count) {
          broadcast(MSG_SNS_STATUS);
        }

        // If the status processor indicates a calibration,
        // then do that now and break this loop
        if (status_response & ER_NEEDS_RECAL) {
          qt_calibrate_sensing();
          break;
        }
      } while (burst_flag);

      if ((data_timer++ == ribbon_library_info.data_interval) ||
          (status_flag & DATA_FLAG) || (status_flag & STATUS_FLAG)) {
        ribbon_data_callback(status_flag);

        if (event_count) {
          broadcast(MSG_SNS_CHANNEL);
        }

        data_timer = 0;
      }
    }

    if (uart_rx_avail()) {
      parse_rx_event();
    }

    osmonitor_refresh();
  }
}

/*============================================================================
Name    :   perform_loop
------------------------------------------------------------------------------
Purpose :   main loop for typical application use, only dispatches status events
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/

void perform_loop(void) {
  uint16_t status_flag;
  uint16_t burst_flag;
  uint8_t status_response;

  while (RIBBON_MODE == ER_PERFORM_LOOP) {
    if (time_to_measure_touch) {
      time_to_measure_touch = 0u;  //  clear flag: it's time to measure touch

      do {
        status_flag = qt_measure_sensors(current_time_ms_touch);
        burst_flag = status_flag & QTLIB_BURST_AGAIN;

        // Process sensor results
        status_response = ribbon_status_callback(status_flag);

        // If we need to notify of performance data, do it
        if (event_count) {
          broadcast(MSG_SNS_STATUS);
        }

        // If the status processor indicates a calibration,
        // then do that now and break this loop
        if (status_response & ER_NEEDS_RECAL) {
          qt_calibrate_sensing();
          break;
        }
      } while (burst_flag);
    }

    // Check for incoming data
    if (uart_rx_avail()) {
      parse_rx_event();
    }

    osmonitor_refresh();  // Refresh watchdog
  }
}

/*============================================================================
Name    :   debug_loop
------------------------------------------------------------------------------
Purpose :   main loop for debug mode, prints data serially in a readable format
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/

void debug_loop(void) {
  uint16_t status_flag;
  uint8_t measure_count = 0u;

  while (RIBBON_MODE == ER_DEBUG_LOOP) {
    if (time_to_measure_touch) {
      /*  clear flag: it's time to measure touch  */
      time_to_measure_touch = 0u;
      measure_count = 0;

      do {
        status_flag = qt_measure_sensors(current_time_ms_touch);
        measure_count++;
      } while (status_flag & QTLIB_BURST_AGAIN);

      if (measure_count > 1) {
        uart_string_P(PSTR("Multiple Measurements\r\n"), 23);
      }

      if (mode_flags & ER_DEBUG_STATUS_FLAG && status_flag & STATUS_FLAG) {
        ribbon_status_callback(status_flag);
      }
    }

    /* If there is a byte in the rx buffer read it into the parser */
    while (uart_rx_avail()) {
      parse_rx_event();
    }

    osmonitor_refresh();  // Refresh watchdog
  }
}

#ifdef RIBBON_DEF_PROCESSING

/*============================================================================
Name    :   process_sensors
------------------------------------------------------------------------------
Purpose :   Process sensor statusand create events to be broadcasted
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
uint8_t default_ribbon_status_callback(uint16_t sflag) {
  uint8_t response = 0u;

  uint8_t i, j, s, position, changed;

#if (RIBBON_DEBUG_MODE == DEBUG_LOOP)
  char str[4];
#endif

  if (sflag & QTLIB_STATUS_CHANGE) {
    // Check if its bad first
    uint8_t bad_data = 0u;

    for (i = 0; i < sensor_count; i++) {
      for (j = sensors[i].from_channel; j <= sensors[i].to_channel; j++) {
        if (qt_measure_data.channel_signals[(j)] <=
            qt_measure_data.channel_references[(j)] / 2) {
#if (RIBBON_DEBUG_MODE == DEBUG_LOOP)
          uart_string_P(PSTR("badsns:"), 7);
          utoa(i, str, 10);
          uart_string(str, 3);
          uart_string_P(PSTR("\r\n"), 2);
          uart_string_P(PSTR("D/R:"), 4);
          itoa(GET_CHANNEL_DELTA(j), str, 10);
          uart_string(str, 6);
          uart_byte('/');
          utoa(GET_CHANNEL_REF(j), str, 10);
          uart_string(str, 6);
          uart_string_P(PSTR("\r\n"), 2);
#endif
          bad_data++;
        }
      }
    }

    if (bad_data >= 4) {
#if (RIBBON_DEBUG_MODE == DEBUG_LOOP)
      uart_string_P(PSTR("baddata!\r\n"), 10);
#endif
      return ER_NEEDS_RECAL;
    }

    s = 0;

    // Go through each sensor state (8 per byte)
    for (i = 0; i < GET_NUM_SNS_BYTES(sensor_count); i++) {
      // Check if any of the 8 sensors of this byte have changed
      if ((changed = STATUS_BYTE_CHANGE(i))) {
        // Check each bit to find which ones have changed
        for (j = 0; j < 8; j++, s++) {
          if ((changed >> j) & 0x01) {
            if (IS_KEY(s))  // KEY
            {
#if (RIBBON_DEBUG_MODE == DEBUG_LOOP)
              uart_string("Key", 3);
              utoa(s, str, 10);
              uart_string(str, 3);
              if ((qt_measure_data.qt_touch_status.sensor_states[i] >> j) & 1) {
                uart_string(" On\r\n", 5);
              } else {
                uart_string(" Off\r\n", 6);
              }
#else
              // The key changed state, send the state
              add_event(
                  BYTE_DATA, s,
                  (qt_measure_data.qt_touch_status.sensor_states[i] >> j) &
                      0x01);
#endif              // DEBUG_LOOP
            } else  // SLIDER
            {
#if (RIBBON_DEBUG_MODE == DEBUG_LOOP)
              uart_string_P(PSTR("Slider"), 6);
              utoa(sensors[s].index, str, 10);
              uart_string(str, 3);
              if ((qt_measure_data.qt_touch_status.sensor_states[i] >> j) & 1) {
                uart_string_P(PSTR(" On\r\n"), 5);
              } else {
                uart_string_P(PSTR(" Off\r\n"), 6);
              }
#else
              // Send the slider on event
              if ((qt_measure_data.qt_touch_status.sensor_states[i] >> j) &
                  0x01) {
                position = UPDATE_SLIDER_POS(sensors[s].index);
                add_event(BYTE_DATA, s, position > 0 ? position : 1);
              } else  // Send the slider off event
              {
                add_event(BYTE_DATA, s, 0);
              }
#endif  // DEBUG_LOOP
            }
            /*
            #if (RIBBON_DEBUG_MODE==DEBUG_LOOP)
                                                            uart_string_P(PSTR("D/R:"),4);
                                                            itoa(GET_CHANNEL_DELTA(sensors[s].from_channel),str,10);
                                                            uart_string(str,6);
                                                            uart_byte('/');
                                                            utoa(GET_CHANNEL_REF(sensors[s].from_channel),str,10);
                                                            uart_string(str,6);
                                                            uart_string_P(PSTR("\r\n"),2);

                                                            uart_string_P(PSTR("PRC:"),4);
                                                            utoa(POS_RECAL_FLAG(s),str,10);
                                                            uart_string(str,6);
                                                            uart_string_P(PSTR("\r\n"),2);
            #endif
            */
          }
        }

        // Update sensor states
        UPDATE_STATUS_BYTE(i);
      } else {
        // Nothing changed so skip these
        // 8 sensors
        s += 8;
      }
    }
  }

  if (sflag & QTLIB_ROTOR_SLIDER_POS_CHANGE) {
    for (i = 0; i < slider_count; i++) {
      if (SLIDER_POS_CHANGE(i)) {
        position = UPDATE_SLIDER_POS(i);

#if (RIBBON_DEBUG_MODE == DEBUG_LOOP)
        uart_string_P(PSTR("Slider"), 6);
        utoa(i, str, 10);
        uart_string(str, 3);
        uart_string_P(PSTR(" Changed:"), 9);
        utoa(position, str, 10);
        uart_string(str, 4);
        uart_string("\r\n", 2);
#else
        // If the slider is active then send the slider change
        if (GET_SENSOR_STATE(slider_index[i])) {
          add_event(BYTE_DATA, slider_index[i], position > 0 ? position : 1);
        }
#endif
      }
    }
  }

  return response;
}

/*============================================================================
Name    :   default_ribbon_data_callback
------------------------------------------------------------------------------
Purpose :   Checks all sensors to see if any channel data has changed, if so
                        a events are generated and then dispatched
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
void default_ribbon_data_callback(uint16_t sflag) {
  uint8_t i, s, changed;
  int8_t delta;

#if (RIBBON_DEBUG_MODE == DEBUG_LOOP)
  char str[4];
#endif

  // Go through each sensor, check if its channel's reference
  // or delta has changed and trigger events
  for (s = 0; s < sensor_count; s++) {
    changed = 0u;

    // Loop through each channel of the sensor
    for (i = sensors[s].from_channel; i <= sensors[s].to_channel; i++) {
      if (GET_CHANNEL_REF(i) != GET_LAST_CHANNEL_REF(i)) {
        UPDATE_CHANNEL_REF(i);
        changed |= CHANNEL_REF_CHANGE;
      }

      delta = CLAMP_INT16_TO_INT8(GET_CHANNEL_DELTA(i));

      if (delta != GET_LAST_CHANNEL_DELTA(i)) {
        UPDATE_CHANNEL_DELTA(i, delta);
        changed |= CHANNEL_DELTA_CHANGE;
      }

      // Check bottom 3 type bits for change
      if (changed & 0x07) {
        // Generate an event for the sensor if not done already
        if (!(changed & 0x80)) {
          changed |= 0x80;

#if (RIBBON_DEBUG_MODE == DEBUG_LOOP)
          uart_string_P(PSTR("SNSCH"), 6);
          utoa(s, str, 10);
          uart_string(str, 3);
          uart_string_P(PSTR(" Change:"), 8);
          uart_string_P(PSTR("\r\n"), 2);
#else
          // Create a sensor channel change method
          add_ch_data_event(s);
#endif
        }

#if (RIBBON_DEBUG_MODE == DEBUG_LOOP)
        uart_string_P(PSTR(" ch"), 3);
        utoa(i, str, 10);
        uart_string(str, 4);
        if (changed & CHANNEL_REF_CHANGE) {
          uart_string_P(PSTR(",ref:"), 5);
          utoa(GET_LAST_CHANNEL_REF(i), str, 10);
          uart_string(str, 4);
        }

        if (changed & CHANNEL_DELTA_CHANGE) {
          uart_string_P(PSTR(",dta:"), 5);
          itoa(GET_LAST_CHANNEL_DELTA(i), str, 10);
          uart_string(str, 4);
        }
        uart_string_P(PSTR("\r\n"), 2);
#else
        // Subtract one since delta define starts at 0
        add_ch_data((changed & 0x07) - 1, i);
#endif

        // Clear bottom 3 type bits
        changed &= ~0x07;
      }
    }
  }
}

#endif  // RIBBON_DEF_PROCESSING

/*============================================================================
Name    :   config_sensor
------------------------------------------------------------------------------
Purpose :   Initializes data structures and calls user function to initialize
keys and sliders Input   :   n/a Output  :   n/a Notes   :   initialize
configuration data for processing
============================================================================*/
static void config_sensors(void) {
  uint8_t i = 0;

  sensor_count = 0u;

  for (i = 0; i < GET_NUM_SNS_BYTES(ER_MAX_SENSORS); i++) {
    sensor_states[i] = 0;
  }

#if (ER_MAX_SLIDERS > 0)
  for (i = 0; i < ER_MAX_SLIDERS; i++) {
    slider_positions[i] = 0;
  }
#endif

  if (ribbon_configure_callback && !ribbon_library_info.config_flags)
    ribbon_configure_callback();
  else {
    loaded_sensor_count = eeprom_read_byte(&sensor_config_count);

    if (loaded_sensor_count > ER_MAX_SENSORS) {
      broadcast_error(ER_SENSOR_COUNT_ERROR);
      return;  // error
    }

    for (i = 0; i < loaded_sensor_count; i += 3) {
      switch (eeprom_read_byte(&sensor_config[i])) {
        case 0:  // Key
          enable_key(eeprom_read_byte(&sensor_config[i + 1]), NO_AKS_GROUP);
          break;
        case 1:
          enable_slider(eeprom_read_byte(&sensor_config[i + 1]),
                        eeprom_read_byte(&sensor_config[i + 2]), NO_AKS_GROUP);
          break;

        default:
          // broadcast_error(ER_SENSOR_TYPE_ERROR);

          // qt_reset_sensing();
          break;
      }
    }
  }
}

/*============================================================================
Name    :   enable_key
------------------------------------------------------------------------------
Purpose :   Enables a key sensor
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/

#ifdef _QMATRIX_

void enable_key(channel_t channel, aks_group_t aks_group) {
  uint8_t threshold, hysteresis;

  if ((threshold = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_THRESHOLD_PARAM))) == 0xFF)
    threshold = ER_THRESHOLD_DEF;

  if ((hysteresis = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_HYSTERESIS1_PARAM))) == 0xFF)
    hysteresis = ER_HYSTERESIS1_DEF;

  qt_enable_key(channel, aks_group, threshold, hysteresis);

  sensor_count++;
}

#endif  //_QMATRIX_

#ifdef _QTOUCH_

void enable_key(channel_t channel, aks_group_t aks_group) {
  uint8_t threshold, hysteresis;

  if ((threshold = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_THRESHOLD_PARAM))) == 0xFF)
    threshold = ER_THRESHOLD_DEF;

  if ((hysteresis = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_HYSTERESIS1_PARAM))) == 0xFF)
    hysteresis = ER_HYSTERESIS1_DEF;

  qt_enable_key(channel, aks_group, threshold, hysteresis);

  sensor_count++;
}

#endif  //_QTOUCH_

/*============================================================================
Name    :   enable_slider
------------------------------------------------------------------------------
Purpose :   Enables a slider sensor
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/

#ifdef _QMATRIX_

void enable_slider(channel_t from_channel, channel_t to_channel,
                   aks_group_t aks_group) {
  uint8_t threshold, hysteresis, resolution, pos_hysteresis;

  if ((threshold = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_THRESHOLD_PARAM))) == 0xFF)
    threshold = ER_THRESHOLD_DEF;

  if ((hysteresis = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_HYSTERESIS1_PARAM))) == 0xFF)
    hysteresis = ER_HYSTERESIS1_DEF;

  if ((resolution = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_RESOLUTION_PARAM))) == 0xFF)
    resolution = ER_RESOLUTION_DEF;

  if ((pos_hysteresis = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_HYSTERESIS2_PARAM))) == 0xFF)
    pos_hysteresis = ER_HYSTERESIS2_DEF;

  qt_enable_slider(from_channel, to_channel, aks_group, threshold, hysteresis,
                   resolution, pos_hysteresis);

  slider_index[slider_count++] = sensor_count++;
}

#endif  //_QMATRIX_

#ifdef _QTOUCH_

void enable_slider(channel_t from_channel, channel_t to_channel,
                   aks_group_t aks_group) {
  uint8_t threshold, hysteresis, resolution, pos_hysteresis;

  if ((threshold = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_THRESHOLD_PARAM))) == 0xFF)
    threshold = ER_THRESHOLD_DEF;

  if ((hysteresis = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_HYSTERESIS1_PARAM))) == 0xFF)
    hysteresis = ER_HYSTERESIS1_DEF;

  if ((resolution = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_RESOLUTION_PARAM))) == 0xFF)
    resolution = ER_RESOLUTION_DEF;

  if ((pos_hysteresis = eeprom_read_byte(
           SNS_PARAM_ADR(sensor_count, ER_HYSTERESIS2_PARAM))) == 0xFF)
    pos_hysteresis = ER_HYSTERESIS2_DEF;

  qt_enable_slider(from_channel, to_channel, aks_group, 12u, 0u, 3u,
                   0u);  // threshold,hysteresis,resolution,pos_hysteresis);

  slider_index[slider_count++] = sensor_count++;
}

#endif  //_QTOUCH_

/*============================================================================
Name    :   qt_set_parameters
------------------------------------------------------------------------------
Purpose :   This will fill the default threshold values in the configuration
            data structure.But User can change the values of these parameters .
Input   :   n/a
Output  :   n/a
Notes   :   initialize configuration data for processing
============================================================================*/

static void set_parameters(void) {
  /*  This will be modified by the user to different values   */
  ribbon_library_info.address =
      eeprom_read_byte(&global_params[ER_ADDRESS_PARAM]);
  ribbon_library_info.data_interval =
      eeprom_read_byte(&global_params[ER_DATA_INTERVAL_PARAM]);
  ribbon_library_info.config_flags = eeprom_read_byte(&custom_config);
  ribbon_library_info.measurement_period_ms =
      eeprom_read_byte(&global_params[ER_MEASURE_PERIOD_MS_PARAM]);

  qt_config_data.qt_di = eeprom_read_byte(&global_params[ER_DI_PARAM]);
  qt_config_data.qt_neg_drift_rate =
      eeprom_read_byte(&global_params[ER_NEG_DRIFT_PARAM]);
  qt_config_data.qt_pos_drift_rate =
      eeprom_read_byte(&global_params[ER_POS_DRIFT_PARAM]);
  qt_config_data.qt_max_on_duration =
      eeprom_read_byte(&global_params[ER_MAX_ON_PARAM]);
  qt_config_data.qt_drift_hold_time =
      eeprom_read_byte(&global_params[ER_DRIFT_HOLD_PARAM]);
  qt_config_data.qt_recal_threshold =
      eeprom_read_byte(&global_params[ER_RECAL_PARAM]);
  qt_config_data.qt_pos_recal_delay =
      eeprom_read_byte(&global_params[ER_POS_RECAL_DELAY_PARAM]);

#ifdef _QMATRIX_
  eeprom_read_block((void*)&qt_burst_lengths, (const void*)&ch_burst_lengths,
                    QT_NUM_CHANNELS);
#endif
}

/*============================================================================
Name    :   notify_init
------------------------------------------------------------------------------
Purpose :   serial out any initial information
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/
static void notify_init() {
#ifdef RIBBON_DEBUG_ON
  char str[3];

  uart_string_P(PSTR("Electric Ribbon "), 17);
  uart_byte(DEVICE_PID1);
  uart_byte(DEVICE_PID2);
  uart_byte(' ');
  utoa(DEVICE_HID, str, 10);
  uart_string(str, 3);
  uart_byte(' ');
  utoa(DEVICE_FID, str, 10);
  uart_string(str, 3);
  uart_string("\r\n", 2);

  if (!eeprom_valid) uart_string_P(PSTR("EEPROM INVALID!\r\n"), 17);
#endif  // RIBBON_DEBUG_ON
}

/*============================================================================
Name    :   ribbon_init
------------------------------------------------------------------------------
Purpose :   initializes clock to correct speed, disables general io pins and
sets up uart baud Input   :   n/a Output  :   n/a Notes   :
============================================================================*/

static void ribbon_init(void) {
  CLKPR = 0x80u;

#ifdef STATIC_CLOCKING_NOFF
  CLKPR = 0x02u;  // Run at 4mhz
  uart_init(RB_LOW);
#else
  CLKPR = 0x00u;  // Run at 16mhz
  uart_init(RB_HIGH);
#endif

  uart_byte('A');

  /*  disable pull-ups    */
  MCUCR |= (1u << PUD);
  /* Disable the JTAG Pins */
  MCUCR |= (1u << JTD);
  MCUCR |= (1u << JTD);

  osmonitor_init();  // Disable watchdogs

  // Test the eeprom
  if (eeprom_read_byte(&eeprom_validator0) == 'S' &&
      eeprom_read_byte(&eeprom_validator1) == 'C')
    eeprom_valid = 1;

  // initialize eeprom
  uint16_t i;
  uint8_t ep_default = 1;

  // Check for defaults error
  if (eeprom_valid) {
    uint8_t ep_first = eeprom_read_byte(&sensor_params[0]);

    for (i = 1; i < ER_MAX_SENSORS * ER_MAX_SENSOR_PARAMS; i++) {
      if (eeprom_read_byte(&sensor_params[i]) != ep_first) {
        ep_default = 0;
        break;
      }
    }
  }

  // If defaults not written, then write
  if (ep_default) {
    ribbon_eeprom_reset();
  }
}

/*============================================================================
Name    :   timer_isr
------------------------------------------------------------------------------
Purpose :   timer 1 compare ISR
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/

ISR(TIMER1_COMPA_vect) {
  time_to_measure_touch = 1u;
  current_time_ms_touch += ribbon_library_info.measurement_period_ms;
}

/*============================================================================
Name    :   init_timer_isr
------------------------------------------------------------------------------
Purpose :   setup timer 1 compare ISR
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/
static void init_timer_isr(void) {
  /*  set timer compare value (how often timer ISR will fire) */
#ifdef STATIC_CLOCKING_NOFF
  OCR1A = TICKS_4MHZ;
#else
  OCR1A = TICKS_16MHZ;
#endif

  /*  enable timer ISR */
  TIMSK1 |= (1u << OCIE1A);

  /*  timer prescaler = system clock / 8  */
  TCCR1B |= (1u << CS11);

  /*  timer mode = CTC (count up to compare value, then reset)    */
  TCCR1B |= (1u << WGM12);
}
