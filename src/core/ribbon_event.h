/*
	Code for adding and broadcasting touch events
*/

#ifndef RIBBON_EVENT_h
#define RIBBON_EVENT_h

#include "ribbon_api.h"

/* The event_t type is used for compiling and sending information. It can
 * be used to send sensor status, channel data, and parameters.
 *
 * id_and_type (status) 6-bits for key id, 2-bits for event type
 * id_and_type (param)  6-bits for param id, 2-bits for param type
 */
struct event_t
{
	uint8_t id_and_type;	// 6-bits for key id,2-bits for event type
							// type is (0=key on/off event, 1=slider event

	uint8_t data[2];		// Data is usually going to be just 1 byte, but
							// in the case of a key data message the data
							// could be 2 or 3 bytes, or a slider data message
							// 1st byte is index into data chunk array,
							// 2nd byte is length
};

enum tag_sns_t
{
	KEY_TYPE,
	SLIDER_TYPE,
	BANK_TYPE
} sns_t;

/* The command type is a received command which has a message type and up to 16 commands */
struct command_t
{
	uint8_t command_type;
	uint8_t command_count;
	// commands stored in event_buffer
};


/* Data Flags */
#define CHANNEL_DELTA_CHANGE 	0x01
#define CHANNEL_REF_CHANGE 		0x02

/* Data Types that can be used with a Channel Message (0-3) */
enum
{
	/* The message contains only a single 8-bit signed delta */
	DELTA_TYPE,
	/* The message contains only a single 10-bit unsigned ref data */
	REF_TYPE,
	/* The message contains a single packed 10-bit ref followed by an 8-bit signed delta */
	REF_DELTA_TYPE
};


/* Slider Event Types (0-7) */
//enum
//{
	//SLIDER_SENSOR_TYPE,	// 1 Byte 8-bit signed delta
	//SLIDER_CHANNEL_TYPE	// 2 Bytes that combine to form a 16-bit slider data type (3-bit type,3-bit channel,10-bit value)
//};


/* Message types (0-15) */
typedef enum tag_er_msg_t
{
	MSG_SNS_UNKNOWN,
	MSG_SNS_STATUS,			// Message type for sending sensor states (on/off, slider pos)
	MSG_SNS_CHANNEL,		// Message type for sending channels data including signals/refs
	MSG_PARAM,				// Message type for sending parameters
	MSG_MODE,				// Message type for sending mode change
	MSG_CMD,				// Message type for acknowledging the receipt of a command
	MSG_CONFIG,				// Message type for sending sensor configuration data
	MSG_SIG,				// Message type for sending device signature
	MSG_ERROR,				// Message type for sending an error message
	MSG_COUNT				// The maximum message id we can receive
} er_msg_t;


/* Device */
typedef enum tag_er_cmd_t
{
	// Changes the mode to the one in the following byte 
	CMD_MODE,
	// Resets the device 
	CMD_RESET,
	// Exits the current loop and reconfigures 
	CMD_RESET_SENSING,
	// Recalibrates all sensors 
	CMD_RECAL,
	// Will dump all global and sensor parameters 
	CMD_DUMP_ALL_PARAMS,
	// Will dump specified param id parameters 
	CMD_DUMP_PARAM,
	// Sets the following parameter index, to the following value 
	CMD_SET_PARAM,
	// Resets the parameter to its default value 
	CMD_RESET_PARAM,

	CMD_EEPROM_RESET,

	CMD_SET_SENSOR,

	CMD_SET_SENSOR_COUNT,

	CMD_DUMP_CONFIG,

	CMD_COUNT // The maximum command we can receive
} er_cmd_t;

/* When reading a dump param message, one can specify the parameter type,
   by using the top 2 bits to specify one of the below values */
typedef enum tag_er_param_t
{
	/* The id is a global param id */
	GLOBAL_PARAM,
	/* The id is a sensor param id */
	SENSOR_PARAM,
	/* The parameter is just a burst length */
	CHANNEL_PARAM
} er_param_t;

/* Data types for data event*/
typedef enum tag_er_data_t
{
	BYTE_DATA, // 1-byte to follow
	WORD_DATA, // 2-bytes to follow
	BLOB_DATA, // Top 5-bits determine number of bytes to follow
	ARRAY_DATA // First byte determines number of bytes to follows
} er_data_t;

#ifndef ER_MIN_RX_BYTES_FOR_EVENT
	#define ER_MIN_RX_BYTES_FOR_EVENT 4 // Need Product Flag (2) + Address/Msg + Cmd
#endif 

#ifndef MAX_EVENTS
	#define MAX_EVENTS 40	// The maximum number of simultaneous events in the queue
#endif

#define ER_GLOBAL_ADDRESS 0
#define ER_MAX_ADDRESSES 16

/* Public */

/* Number of events in the queue, used externally to check whether to call broadcast */
uint8_t event_count;

/* Broadcasts any status events in the queue */
void broadcast(uint8_t message_type);
/* A call to check the uart rx and parse an event */
void parse_rx_event(void);
/* Adds a single byte event to the event queue */
void add_event(uint8_t event_type, uint8_t sensor_id, uint8_t data);

void add_ch_data_event(uint8_t sensor_id);
void add_ch_data(uint8_t type, uint8_t channel);

void event_reset(void);


void broadcast_mode(void);
void broadcast_error(er_err_t error);
void add_global_param_event(uint8_t param_id, uint8_t value);
void add_sensor_param_event(uint8_t sensor, uint8_t param_id, uint8_t value);
void add_channel_param_event(uint8_t sensor, uint8_t channel, uint8_t value);



#endif //RIBBON_EVENT_h


