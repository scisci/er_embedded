
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
#define DEVICE_FID 1u

#define RIBBON_DEF_PROCESSING  // Use the internal sensor processor to dispatch
                               // touch events
#define RIBBON_DEF_FILTERING   // Use the internal/no filtering

//----------------------------------------------------------------------------
//	debugging definitions
//----------------------------------------------------------------------------
//#define DEBUG_LEVEL DEBUG_EVENT

#define ER_THRESHOLD_DEF 12u

/*----------------------------------------------------------------------------
        qtouch definitions
----------------------------------------------------------------------------*/
#define QT_MAX_NUM_ROTORS_SLIDERS 4
#define QT_NUM_CHANNELS 64
#define _QMATRIX_

#define SMP_PIN 0
#define NUM_X_PORTS 1
#define NUM_X_LINES 8
#define NUM_Y_LINES 6

#define PORT_X C
#define PORT_X_1 C
#define PORT_NUM_1 1
#define PORT_YA D
#define PORT_YB A
#define PORT_SMP B

#define _ROTOR_SLIDER_
#define QT_DELAY_CYCLES 2u

#endif  // RIBBON_GLOBALS_h
