#ifndef RIBBON_GLOBALS_h
#define RIBBON_GLOBALS_h

#include "ribbon_debug.h"

/*----------------------------------------------------------------------------
        ribbon api definitions
----------------------------------------------------------------------------*/
// SCISCI's internal hardware id number for the device
// tells which interface this device is capable of
// (not broadcasted)
#define DEVICE_HID 1u
// SCISCI's firmware id for the device, which indicates
// what version of firmware is installed on the device
// which, along with the version identifies this device's
// functionality (0-1024, not broadcasted)
#define DEVICE_FID 3u

#define NUM_KEYS 2u     // Number of sensors for this device
#define NUM_SLIDERS 1u  // Number of sliders for this device

#define RIBBON_DEF_PROCESSING
#define RIBBON_DEF_FILTERING

/*----------------------------------------------------------------------------
        compiler definitions
----------------------------------------------------------------------------*/
//#define DEBUG_MODE DEBUG_LOOP

#define ER_THRESHOLD_DEF 12u

/*----------------------------------------------------------------------------
        qtouch definitions
----------------------------------------------------------------------------*/
#define QT_MAX_NUM_ROTORS_SLIDERS 1
#define QT_NUM_CHANNELS 8
#define _QTOUCH_

#define SNS1 A
#define SNSK1 D
#define _POWER_OPTIMIZATION_ 0

#define _ROTOR_SLIDER_
#define QT_DELAY_CYCLES 2u

#endif  // RIBBON_GLOBALS_H
