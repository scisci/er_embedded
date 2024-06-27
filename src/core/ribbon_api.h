#ifndef RIBBON_API_h
#define RIBBON_API_h

#include "ribbon_debug.h"
#include "ribbon_param.h"
/*----------------------------------------------------------------------------
  electric ribbon device information
----------------------------------------------------------------------------*/

#if defined(RIBBON_DEBUG_MODE) && !defined(DEBUG_LEVEL)
#error No DEBUG_LEVEL specified
#undef RIBBON_DEBUG_MODE
#endif

#ifdef DEBUG_LEVEL
#define RIBBON_DEBUG_ON
#define RIBBON_DEBUG_MODE DEBUG_LEVEL
#else
#define RIBBON_DEBUG_MODE DEBUG_OFF
#endif

/* Maximum number of channels per slider */
#define ER_MAX_CH_PER_SLIDER 8u
/* Maximum number of channels per key */
#define ER_MAX_CH_PER_KEY 1u

/* The high clock speed */
#ifndef F_CPU
#define F_CPU 16000000
#endif

/* The low clock speed (/4) */
#ifndef F_CPU_LOW
#define F_CPU_LOW 4000000
#endif

#define ER_MAX_SENSORS QT_NUM_CHANNELS
#define ER_MAX_SLIDERS QT_MAX_NUM_ROTORS_SLIDERS

/* Baud rate for all communications */
#ifndef RIBBON_BAUD_RATE
#define RIBBON_BAUD_RATE 62500UL  // The baud rate for uart communication
#endif

/* Baud rate counter for high cpu */
#define RB_HIGH ((F_CPU / (RIBBON_BAUD_RATE << 4)) - 1)

/* Baud rate counter for low cpu */
#define RB_LOW ((F_CPU_LOW / (RIBBON_BAUD_RATE << 4)) - 1)

#define DEVICE_PID1 'S'  // SCISCI Product ID (2 chars, broadcasted)
#define DEVICE_PID2 'C'  // SCISCI Product ID (2 chars, broadcasted)

#ifndef DEVICE_HID
#error Need to define device hardware id (DEVICE_HID, 0-255).
#endif

#ifndef DEVICE_FID
#error Need to define device firmware id (DEVICE_FID, 0-1023).
#endif

#ifndef DEVICE_ADR
#define DEVICE_ADR \
  2u  // Unique Address assigned to each device (reprogrammable)
      // (0-15s, broadcasted)
#endif

//#define DYNAMIC_CLOCKING		// Changes clock rates to 4mhz when
// sampling and 16mhz during serial comms #define DYNAMIC_CLOCKING_NOFF 	//
// Makes the program start at 4mhz and only increase to 16mhz for serial
#define STATIC_CLOCKING_NOFF  // Specifies that the static clock run at low
                              // speed normally

//#ifndef NUM_KEYS
//#error Need to define number of keys (NUM_KEYS, 0-64u).
//#endif

//#ifndef NUM_SLIDERS
//#error Need to define number of slider (NUM_SLIDERS, 0-8u).
//#endif

//#define NUM_SENSORS ((NUM_KEYS)+(NUM_SLIDERS))

//#define NUM_SENSOR_BYTES ((sensor_count + 7u) >> 3) //((NUM_SENSORS + 7u) / 8u
//)
#define GET_NUM_SNS_BYTES(cnt) (((cnt) + 7u) / 8)

/* A flag from tlib that tells whether we need to run the status method */
#define STATUS_FLAG (QTLIB_STATUS_CHANGE | QTLIB_ROTOR_SLIDER_POS_CHANGE)
/* A flag from tlib that tells whether we need to dispatch data */
#define DATA_FLAG QTLIB_CHANNEL_REF_CHANGE

#if (QT_MAX_NUM_ROTORS_SLIDERS > 0)
#define _ROTOR_SLIDER_
#endif

#ifndef QT_DELAY_CYCLES
#define QT_DELAY_CYCLES 2u
#endif

#define DEF_GLOBAL_PARAM_MAP(count, ...) \
  uint8_t global_param_count = count;    \
  uint8_t global_param_map[count] = {__VA_ARGS__}

#define DEF_SENSOR_PARAM_MAP(count, ...) \
  uint8_t sensor_param_count = count;    \
  uint8_t sensor_param_map[count] = {__VA_ARGS__}

#ifdef _QMATRIX_

#ifndef QT_MAX_NUM_ROTORS_SLIDERS
#define QT_MAX_NUM_ROTORS_SLIDERS 8
#endif

#ifndef QT_NUM_CHANNELS
#define QT_NUM_CHANNELS 64
#endif

#ifndef SMP_PIN
#define SMP_PIN 0
#endif

#ifndef NUM_X_PORTS
#define NUM_X_PORTS 1
#endif

#ifndef NUM_X_LINES
#define NUM_X_LINES 8
#endif

#ifndef NUM_Y_LINES
#define NUM_Y_LINES 6
#endif

#ifndef PORT_X
#define PORT_X C
#endif

#ifndef PORT_X_1
#define PORT_X_1 C
#endif

#ifndef PORT_NUM_1
#define PORT_NUM_1 1
#endif

#ifndef PORT_YA
#define PORT_YA D
#endif

#ifndef PORT_YB
#define PORT_YB A
#endif

#ifndef PORT_SMP
#define PORT_SMP B
#endif

#endif  //_QMATRIX_

// Mode Flags
typedef enum tag_er_mode {
  // This mode is optimized for performance, only sending status messages
  ER_PERFORM_LOOP,
  // This mode is optimized for watching params
  ER_DATA_LOOP,
  // This mode is not complete but should do  debug
  ER_DEBUG_LOOP,
  // This mode is optimized for receiving data and changing parameter
  ER_CMD_LOOP,
  // Term used to determine valid Modes
  ER_MAX_LOOP
} er_mode_t;

// Error Types
typedef enum tag_er_err {
  ER_EEPROM_ERROR,
  ER_SENSOR_COUNT_ERROR,
  ER_SENSOR_TYPE_ERROR
} er_err_t;

/*----------------------------------------------------------------------------
                                include files
----------------------------------------------------------------------------*/

#include <avr/interrupt.h>
#include <avr/io.h>
#include "uart.h"

#define __delay_cycles(n) __builtin_avr_delay_cycles(n)
#define __enable_interrupt() sei()
#include "touch_api.h"

/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/
/* Returns if the given sensor index is a key type */
#define IS_KEY(sensor) ((sensors[(sensor)].type_aks_pos_hyst >> 6) == 0u)

/* Returns if the given sensor index is a slider type */
#define IS_SLIDER(sensor) ((sensors[(sensor)].type_aks_pos_hyst >> 6) == 2u)

#define POS_RECAL_FLAG(sensor) (sensors[(sensor)].type_aks_pos_hyst & (1 << 2))

#define CLEAR_POS_RECAL_FLAG(s) (sensors[(s)].type_aks_pos_hyst &= ~(1 << 2))

// Check if there is a change in a specific byte of the sensor state array and
// return the bits that have changed
#define STATUS_BYTE_CHANGE(i) \
  (sensor_states[(i)] ^ qt_measure_data.qt_touch_status.sensor_states[(i)])

// Updates the local copy of the sensor states, by copying from qt_measure_data
// */
#define UPDATE_STATUS_BYTE(i) \
  (sensor_states[(i)] = qt_measure_data.qt_touch_status.sensor_states[(i)])

// Checks if the position of a sider has changed since last time (slider index,
// not sensor)
#define SLIDER_POS_CHANGE(s) \
  (slider_positions[(s)] !=  \
   qt_measure_data.qt_touch_status.rotor_slider_values[(s)])

// Updates the local copy of the slider position by copying from qt_measure_data
#define UPDATE_SLIDER_POS(s) \
  (slider_positions[(s)] =   \
       qt_measure_data.qt_touch_status.rotor_slider_values[(s)])

// Returns 1 or 0 depending on the sensor state

#define GET_SENSOR_STATE(s) (sensor_states[((s) / 8u)] & (1 << ((s) % 8)))

// Gets the local copy of the slider position
#define GET_SLIDER_POSITION(s) slider_positions[(s)]

// Returns the current mode
#define RIBBON_MODE (mode_flags & 0xF)
// Sets the current mode as long as a valid mode is supplied
#define SET_RIBBON_MODE(mode)       \
  if (((mode)&0xF) < ER_MAX_LOOP) { \
    mode_flags &= 0xF0;             \
    mode_flags |= ((mode)&0xF);     \
  }

#define ER_NEEDS_RECAL 0x01

typedef struct tag_ribbon_library_info_t {
  // The product id, used as sync chars
  uint8_t pid[2];
  // A hardware signature
  uint8_t hid;
  // A firmware signature
  uint8_t fid;
  // Holds the device address for communication
  uint8_t address;
  // Holds the number of cycles for printing data, in data mode
  uint8_t data_interval;
  // Holds information about the sensor configuration
  uint8_t config_flags;

  volatile uint8_t measurement_period_ms;
} ribbon_library_info_t;

/* This configuration data structure parameters if needs to be changed will be
   changed in the qt_set_parameters function */
ribbon_library_info_t ribbon_library_info;

typedef struct tag_ribbon_channel_data_t {
  int8_t delta;  // Holds the last read delta of this channel (clamped to -127
                 // +127)
  uint16_t reference;  // Holds the last read reference of this channel

} ribbon_channel_data_t;

typedef struct tag_ribbon_slider_params_t {
  uint8_t position_hysteresis;
  uint8_t position_resolution;

} slider_params_t;

/*----------------------------------------------------------------------------
        public variables
----------------------------------------------------------------------------*/

/* flag set by timer ISR when it's time to measure touch */
volatile uint8_t time_to_measure_touch;
volatile uint16_t current_time_ms_touch;

/* Holds the last on/off state of each sensor, bit-packed */
uint8_t sensor_states[GET_NUM_SNS_BYTES(ER_MAX_SENSORS)];

#if (ER_MAX_SLIDERS > 0)
/* Holds the last position of each slider */
uint8_t slider_positions[ER_MAX_SLIDERS];
/* Holds the index into the sensor array */
uint8_t slider_index[ER_MAX_SLIDERS];
/* Holds slider parameters that we can't find in qt */
slider_params_t slider_params[ER_MAX_SLIDERS];
#endif  // NUM_SLIDERS>0

/* Holds channel deltas and references */
ribbon_channel_data_t ribbon_channel_data[QT_NUM_CHANNELS];
/* Can be used to change mode via SET_RIBBON_MODE, or RIBBON_MODE */
uint8_t mode_flags;

/*----------------------------------------------------------------------------
        public methods
----------------------------------------------------------------------------*/
/* Called from main file to begin sensing loop */
void ribbon_main(void);
/* Get a global parameter with its map index */
uint8_t get_mapped_global_param(uint8_t index);
/* Set a global parameter with its map index */
bool set_mapped_global_param(uint8_t index, uint8_t value);
/* Get a sensor parameter with the sensor index and param index */
uint8_t get_mapped_sensor_param(uint8_t sensor, uint8_t index);
/* Set a sensor parameter with the sensor index, param index, and value */
bool set_mapped_sensor_param(uint8_t sensor, uint8_t index, uint8_t value);
/* Get a channel param, for now this only does burst length */
uint8_t get_mapped_channel_param(uint8_t sensor, uint8_t channel);
/* Set a channel parameter, for now this only does burst length */
bool set_mapped_channel_param(uint8_t sensor, uint8_t channel, uint8_t value);
/* Uses watchdog to reset the device */
void ribbon_hard_reset(void);
void ribbon_eeprom_reset(void);
void ribbon_recal(void);
bool ribbon_set_sensor(uint8_t sensor_index, uint8_t sensor_type,
                       uint8_t from_ch, uint8_t to_ch);
bool ribbon_set_sensors(uint8_t sensors);
#ifdef _QTOUCH_

void enable_key(channel_t channel, aks_group_t aks_group);

void enable_slider(channel_t from_channel, channel_t to_channel,
                   aks_group_t aks_group);

#endif  //_QTOUCH_

#ifdef _QMATRIX_

void enable_key(channel_t channel, aks_group_t aks_group);

void enable_slider(channel_t from_channel, channel_t to_channel,
                   aks_group_t aks_group);

#endif  //_QMATRIX_

/*----------------------------------------------------------------------------
        public virtual methods
----------------------------------------------------------------------------*/
uint8_t (*ribbon_status_callback)(uint16_t status);
void (*ribbon_data_callback)(uint16_t status);
void (*ribbon_filter_callback)(void);
void (*ribbon_configure_callback)(void);

#endif  // RIBBON_API_h
