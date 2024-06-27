

#include "uart.h"
#include "ribbon_api.h"
#include "ribbon_event.h"

#include <stdlib.h>


/* Maximum number of bytes per channel data */
#define MAX_CH_DATA_BYTES 3u
/* The maximum allocation for slider data chunks, each slider has up to 8 channels, and each
   channel can have up to 3 data bytes (delta_ref), each key can have 1 channel,
   there can only be 1 event per sensor so we figure that if all of them were slider events
   there could be ER_MAX_SLIDERS*8 and there could be MAX_EVENTS-ER_MAX_SLIDERS more data*/
#define MAX_CHANNEL_DATA ((ER_MAX_SLIDERS)*(ER_MAX_CH_PER_SLIDER)*MAX_CH_DATA_BYTES)+((MAX_EVENTS-ER_MAX_SLIDERS)*(ER_MAX_CH_PER_KEY)*MAX_CH_DATA_BYTES)

// Maximum number of commands
#define MAX_CMD_DATA 64

// Create a buffer for this event that can hold both incoming command messages
// and store outgoing channel data
#define EVENT_MAX_BUFFER_SIZE (MAX_CHANNEL_DATA>MAX_CMD_DATA?MAX_CHANNEL_DATA:MAX_CMD_DATA)


#define GET_EVENT_ID(i) ((events[(i)].id_and_type>>2)&0x3F)
#define GET_EVENT_TYPE(i) (events[(i)].id_and_type&0x03)

/* Include ribbon library information for dispatching */
extern ribbon_library_info_t ribbon_library_info;

extern uint8_t sensor_count;
//extern uint8_t qt_burst_lengths[];

/* Include parameter counts */
extern uint8_t global_param_count;
extern uint8_t sensor_param_count;
extern uint8_t get_mapped_global_param(uint8_t index);
extern uint8_t get_mapped_sensor_param(uint8_t sensor, uint8_t index);
extern uint8_t get_mapped_channel_param(uint8_t sensor, uint8_t channel);
extern bool set_mapped_global_param(uint8_t index, uint8_t value);
extern bool set_mapped_sensor_param(uint8_t sensor, uint8_t index, uint8_t value);
extern bool set_mapped_channel_param(uint8_t sensor, uint8_t channel, uint8_t value);

extern void ribbon_hard_reset(void);
extern void ribbon_set_mode(uint8_t mode);


extern volatile uint8_t time_to_measure_touch;

extern uint8_t uart_unbuffer_rx(void);
extern uint8_t uart_rx_avail(void);



uint8_t event_count = 0u;

static struct command_t last_cmd;
static uint16_t last_cmd_time = 0;

static uint8_t ch_data_count = 0u;

static uint8_t last_ch_evt = 0u;		// When adding extra ch data, need to store this
static uint8_t last_ch_idx = 0u;		// The last ch index

static uint8_t prs_flags = 0u;			// Parse flags, indicate the parser context
static uint8_t prs_count = 0u;			// The byte count that has been parsed
static uint8_t prs_cksum = 0u;			// The current parser check sum

#define PRS_PRID1_FLAG	0x01	// If set in prs_flags, the first sync is read
#define PRS_ADDRS_FLAG	0x02	// If set in prs_flags, the second sync is read
#define PRS_PRID2_FLAG	0x04	// If set in prs_flags, the adr and msg is read
#define PRS_CMDTP_FLAG	0x08	// If set in prs_flags, the cmd and type
#define PRS_CMDCT_FLAG	0x10	// If set in prs_flags, the count is read

#if (RIBBON_DEBUG_MODE==DEBUG_RX)
char small_buffer[20];
uint8_t small_buffer_pos = 0;
#endif


	

/*----------------------------------------------------------------------------
  vars
----------------------------------------------------------------------------*/

/* The event queue */
static struct event_t events[MAX_EVENTS];
// Event buffer used for storing command data and storeing outgoing channel data
static uint8_t event_buffer[EVENT_MAX_BUFFER_SIZE];	


/*----------------------------------------------------------------------------
                                methods
----------------------------------------------------------------------------*/



void add_global_param_event(uint8_t param_id, uint8_t value)
{
	events[event_count].id_and_type = ((GLOBAL_PARAM&0x03)<<6);
	events[event_count].data[0] = param_id;
	events[event_count].data[1] = value;


	if(++event_count==MAX_EVENTS)
		broadcast(MSG_PARAM);
}

void add_sensor_param_event(uint8_t sensor, uint8_t param_id, uint8_t value)
{
	events[event_count].id_and_type = ((SENSOR_PARAM&0x03)<<6)|(sensor&0x3F);
	events[event_count].data[0] = param_id;
	events[event_count].data[1] = value;

	if(++event_count==MAX_EVENTS)
		broadcast(MSG_PARAM);
}

void add_channel_param_event(uint8_t sensor, uint8_t channel, uint8_t value)
{
	events[event_count].id_and_type = ((CHANNEL_PARAM&0x03)<<6)|(sensor&0x3F);
	events[event_count].data[0] = channel;
	events[event_count].data[1] = value;

	if(++event_count==MAX_EVENTS)
		broadcast(MSG_PARAM);
}

/*============================================================================
Name    :   add_event
------------------------------------------------------------------------------
Purpose :   adds a status event to the event queue that will be broadcasted
			when broadcast() is called
Input   :   uint8_t event_type - the type of status event (right now only STATUS_EID)
			uint8_t sensor_id - the sensor id of the event
			uint8_t data - the data associated with the event (for a key, 1 or 0, for a slider 1-255 or 0)
Output  :   n/a
Notes   :   n/a
============================================================================*/
void add_event(uint8_t event_type, uint8_t sensor_id, uint8_t data)
{


#if (RIBBON_DEBUG_MODE==DEBUG_EVENT)
	char str[4]; 
	uart_string("STAT EVT:",9);
	utoa(event_type,str,10);
	uart_string(str,3);
	uart_byte(',');
	utoa(sensor_id,str,10);
	uart_string(str,3);
	uart_byte(',');
	utoa(data,str,10);
	uart_string(str,3);
	uart_string("\r\n",2);
#endif


	events[event_count].id_and_type = ((sensor_id<<2)|(event_type));
	events[event_count].data[0] = data;

	if(++event_count==MAX_EVENTS)
		broadcast(MSG_SNS_STATUS);
}



/*============================================================================
Name    :   add_ch_data_event
------------------------------------------------------------------------------
Purpose :   adds a channel data event to the event queue meaning that it will be
			followed by 1 or more channel data blocks when broadcast() is called, 
			data events contain references and deltas
			used in software calibration and debugging
Input   :   uint8_t sensor_id - the sensor id of the event
Output  :   n/a
Notes   :   Data events cannot be combined with status events in the same broadcast!
			Directly following any call to add_slider_data_event with a channel data tyep, 
			one or more calls to add_slider_channel_data must be called to fill it with channel data
============================================================================*/
void add_ch_data_event(uint8_t sensor_id)
{
	// If the queue is about to overflow then dispatch it (this shouldn't happen)
	if(event_count+1==MAX_EVENTS)
		broadcast(MSG_SNS_CHANNEL);


#if (RIBBON_DEBUG_MODE==DEBUG_EVENT)
	char str[4];
	uart_string("SLD DATA EVT",13); 
	utoa(sensor_id,str,10);
	uart_string(str,3);
	uart_string("\r\n",2);
#endif

	// The type will be determined once we know the number of bytes;
	// A single delta or single reference will be WORD_DATA, a single
	// ref_delta is best if just split into multi-channel information
	events[event_count].id_and_type = ((sensor_id<<2));
	// Store a pointer to the channel data array
	events[event_count].data[0] = ch_data_count;
	// Store the number of bytes of channel data pushed in
	events[event_count].data[1] = 0;

	last_ch_idx = sensor_id;
	last_ch_evt = event_count++;
}

void broadcast_config(void)
{
	for(event_count=0;event_count<sensor_count;event_count++)
	{
		events[event_count].id_and_type = ((IS_KEY(event_count)?KEY_TYPE:SLIDER_TYPE)<<2) | WORD_DATA;
		events[event_count].data[0] = sensors[event_count].from_channel;
		events[event_count].data[1] = sensors[event_count].to_channel;
	}

	broadcast(MSG_CONFIG);
}

/*============================================================================
Name    :   add_ch_data
------------------------------------------------------------------------------
Purpose :   adds channel data to the previously added channel event.
Input   :   uint8_t data_type - the type of data event (delta, reference, delta/ref) (0-3)
			uint8_t channel - the absolute channel id of this channel 
Output  :   n/a
Notes   :   n/a
============================================================================*/
void add_ch_data(uint8_t data_type, uint8_t channel)
{


#if (RIBBON_DEBUG_MODE==DEBUG_EVENT)
	char str[4];
#endif


	// Store type and channel number
	event_buffer[ch_data_count] = ((data_type&3)<<6) | (((channel-sensors[last_ch_idx].from_channel)&0x1F)<<2);

	if(data_type==DELTA_TYPE)
	{

#if (RIBBON_DEBUG_MODE==DEBUG_EVENT)
		uart_string(" dta",4); 
		utoa(channel-sensors[last_ch_idx].from_channel,str,10);
		uart_string(str,2);
		uart_byte(':');
		itoa(ribbon_channel_data[channel].delta,str,10);
		uart_string(str,4);
#endif

		// Store the signed delta value in the channel data array
		event_buffer[ch_data_count+1] = ribbon_channel_data[channel].delta;
		// Update the byte count of the last channel event
		events[last_ch_evt].data[1] += 2;
		// Increment the channel data pointer
		ch_data_count += 2;
	}
	else if(data_type==REF_TYPE)
	{

#if (RIBBON_DEBUG_MODE==DEBUG_EVENT)
		uart_string(" ref",4); 
		utoa(channel-sensors[last_ch_idx].from_channel,str,10);
		uart_string(str,2);
		uart_byte(':');
		utoa(ribbon_channel_data[channel].reference,str,10);
		uart_string(str,4);
#endif
		// Use the top two bits of the first byte to store the reference value
		event_buffer[ch_data_count] |= (ribbon_channel_data[channel].reference>>8)&3;
		// Put the remaining reference value in the second byte
		event_buffer[ch_data_count+1] = (ribbon_channel_data[channel].reference&0xFF);
		// Update the byte count of the last channel event
		events[last_ch_evt].data[1] += 2;
		// Increment the channel data pointer
		ch_data_count += 2;
	}
	else if(data_type==REF_DELTA_TYPE)
	{

#if (RIBBON_DEBUG_MODE==DEBUG_EVENT)
		uart_string(" dta/ref",8); 
		utoa(channel-sensors[last_ch_idx].from_channel,str,10);
		uart_string(str,2);
		uart_byte(':');
		itoa(ribbon_channel_data[channel].delta,str,10);
		uart_string(str,4);
		uart_byte(',');
		utoa(ribbon_channel_data[channel].reference,str,10);
		uart_string(str,4);
#endif

		// Use the top two bits of the first byte to store the ref value
		event_buffer[ch_data_count] |= (ribbon_channel_data[channel].reference>>8)&3;
		// Store the remaining bit int he second byte
		event_buffer[ch_data_count+1] = ribbon_channel_data[channel].reference&0xFF;
		// Store the delta in the following byte
		event_buffer[ch_data_count+2] = ribbon_channel_data[channel].delta;
		// Update the byte count of the last channel event
		events[last_ch_evt].data[1] += 3;
		// Increment the channel data pointer
		ch_data_count += 3;
	}


#if (RIBBON_DEBUG_MODE==DEBUG_EVENT)
	uart_string("\r\n",2);
#endif


}

/*============================================================================
Name    :   broadcast_mode
------------------------------------------------------------------------------
Purpose :   Broadcasts the current device mode
Output  :   n/a
Notes   :   n/a
============================================================================*/
void broadcast_mode(void)
{
	event_count = 0;
	events[event_count++].id_and_type = RIBBON_MODE;
	broadcast(MSG_MODE);
}

/*============================================================================
Name    :   broadcast_cmd
------------------------------------------------------------------------------
Purpose :   Broadcasts the receipts of a command
Input   :   uint8_t cmd - the command, bool valid - whether the command completed successfully
Output  :   n/a
Notes   :   n/a
============================================================================*/
static void broadcast_cmd(uint8_t cmd, bool valid)
{
	event_count = 0;
	events[event_count++].id_and_type = (valid ? 0x80 : 0) | (cmd&0x7F);
	broadcast(MSG_CMD);
}

void broadcast_error(er_err_t error)
{
	event_count = 0;
	events[event_count++].id_and_type = error;
	broadcast(MSG_ERROR);
}


/*============================================================================
Name    :   broadcast
------------------------------------------------------------------------------
Purpose :   Broadcasts the current event queue with the provided header
Input   :   uint8_t message_type - the type of data events that are in the queue
			this could be status events or data events
Output  :   n/a
Notes   :   n/a
============================================================================*/
void broadcast(uint8_t message_type)
{
	// write device signature

	// Protocol is
	// 2-byte product code
	// 4-bit address
	// 4-bit message type
	// 1-byte # of frames to follow
	// frame content
	//	for message type 0 (sensors)
	//	6-bit sensor id
	//	2-bit sensor data type
	//	1+bits of sensor data

	// Setup the 8-bit checksum
	uint8_t check_sum = 0u;

	uint8_t i;



#if ((RIBBON_DEBUG_MODE==DEBUG_EVENT) || (RIBBON_DEBUG_MODE==DEBUG_BROADCAST))
	char str[4];
	uart_string("\r\nBroadcast(",12);
	utoa(message_type,str,10);
	uart_string(str,2);
	uart_byte(',');
	utoa(event_count,str,10);
	uart_string(str,2);
	uart_string(")\r\n\r\n",5);
#elif(RIBBON_DEBUG_MODE==DEBUG_BROADCAST)
	uart_byte(',');
	utoa(DEVICE_ADR,str,10);
	uart_string(str,2);
	uart_string(")\r\n\r\n",5);
#endif // DEBUG


#if !defined(RIBBON_DEBUG_ON) || (RIBBON_DEBUG_MODE==DEBUG_RX)

	// Send the Product ID - 2 bytes
	uart_byte(ribbon_library_info.pid[0]);
	check_sum += ribbon_library_info.pid[0];

	uart_byte(ribbon_library_info.pid[1]);
	check_sum += ribbon_library_info.pid[1];

	// Send the 4-bit address combined with the 4-bit message type (in this case, sensor data)
	uart_byte(((ribbon_library_info.address&0xF)<<4)|(message_type&0xF));
	check_sum += ((ribbon_library_info.address&0xF)<<4)|(message_type&0xF);

	// Send the number of message frames to follow
	uart_byte(event_count);
	check_sum += event_count;

#endif // RIBBON_DEBUG_ON

	// Send message-type specific data
	if(message_type==MSG_SNS_STATUS)			// Status messages are for performance mode
	{											// Minimal bytes is best
		for(i=0;i<event_count;i++)
		{

#if !defined(RIBBON_DEBUG_ON) || (RIBBON_DEBUG_MODE==DEBUG_RX)
			uart_byte(events[i].id_and_type);	// Send id and type
			check_sum += events[i].id_and_type;

			uart_byte(events[i].data[0]);		// Send associated data, always 1 byte
			check_sum += (events[i].data[0]);
#endif // RIBBON_DEBUG_ON
		}
	}
	else if(message_type==MSG_SNS_CHANNEL)
	{
#if !defined(RIBBON_DEBUG_ON) || (RIBBON_DEBUG_MODE==DEBUG_RX)
		uint8_t data_size;
#endif

		for(i=0;i<event_count;i++)
		{
#if !defined(RIBBON_DEBUG_ON) || (RIBBON_DEBUG_MODE==DEBUG_RX)

			// Determine the event size  via data[1] and add it to the id_and_type header
			if(events[i].data[1]==2)
			{
				events[i].id_and_type |= (data_size = WORD_DATA)&0x03;
			}
			else if(events[i].data[1]>2)
			{
				events[i].id_and_type |= (data_size = ARRAY_DATA)&0x03;
			}
			else
			{
				events[i].id_and_type |= (data_size = BYTE_DATA);
			}

			uart_byte(events[i].id_and_type);	// Send id and type
			check_sum += events[i].id_and_type;

			if(data_size == ARRAY_DATA)
			{
				// send the array size
				uart_byte(events[i].data[1]);
				check_sum += events[i].data[1];
			}

#elif (RIBBON_DEBUG_MODE==DEBUG_BROADCAST)

			uart_string("EVT ID:",7);
			utoa(GET_EVENT_ID(i),str,10);
			uart_string(str,2);
			uart_string(", SIZE:",7);
			utoa(data_size,str,10);
			uart_string(str,2);
			uart_string("\r\n",2);
#endif


#if !defined(RIBBON_DEBUG_ON) || (RIBBON_DEBUG_MODE==DEBUG_RX)
			// Pull the data bytes out of the channel event array
			for(int j=0;j<events[i].data[1];j++)
			{
				uart_byte(event_buffer[events[i].data[0]+j]);
				check_sum += event_buffer[events[i].data[0]+j];
			}

#elif (RIBBON_DEBUG_MODE==DEBUG_BROADCAST)

			uart_string("CH DATA:",7);
			utoa(events[i].data[1],str,10);
			uart_string(str,3);
			uart_string("b\r\n",3);	

#endif
		}
		// Reset the channel_data count pointer since it will be flushed
		ch_data_count = 0u;

	}
	else if(message_type==MSG_PARAM || message_type==MSG_CONFIG)
	{
#if !defined(RIBBON_DEBUG_ON) || (RIBBON_DEBUG_MODE==DEBUG_RX)
		for(i=0;i<event_count;i++)
		{
			uart_byte(events[i].id_and_type);	// Send id and type
			check_sum += events[i].id_and_type;

			uart_byte(events[i].data[0]);		// Send associated data, always 2 bytes
			check_sum += (events[i].data[0]);

			uart_byte(events[i].data[1]);		
			check_sum += (events[i].data[1]);
		}
#endif // RIBBON_DEBUG_ON

	}
	else if((message_type==MSG_MODE) || (message_type==MSG_CMD) || (message_type==MSG_ERROR))
	{
		uart_byte(events[0].id_and_type);
		check_sum += events[0].id_and_type;
	}

#if !defined(RIBBON_DEBUG_ON) || (RIBBON_DEBUG_MODE==DEBUG_RX)
	// Send the check sum
	uart_byte(check_sum);
#endif

	event_count = 0u;
}


static void handle_message(void)
{
	uint8_t i, s;

#if (RIBBON_DEBUG_MODE==DEBUG_RX)
	uart_string_P(PSTR("CMD:"),4);
	uart_byte(last_cmd.command_type);
	uart_string_P(PSTR("\r\n"),2);
#endif

	bool command_valid = false;

	switch(last_cmd.command_type)
	{
		case CMD_DUMP_ALL_PARAMS:
#ifdef RIBBON_DEBUG_ON
			uart_string_P(PSTR("CMD: Dump All Params\r\n"),22);
#endif
			broadcast_cmd(last_cmd.command_type,true);

			// Dump all the global parameters
			for(i=0;i<global_param_count;i++)
				add_global_param_event(i,get_mapped_global_param(i));

			// Dump the sensor and channel parameters
			for(s=0;s<sensor_count;s++)
			{
				// Dump the general sensor parameters
				for(i=0;i<sensor_param_count;i++)
					add_sensor_param_event(s,i,get_mapped_sensor_param(s,i));

				// Dump the per channel parameters (for now just bl)
				for(i=sensors[s].from_channel;i<=sensors[s].to_channel;i++)
					add_channel_param_event(s,i-sensors[s].from_channel,
							get_mapped_channel_param(s,i-sensors[s].from_channel));
			}

			

			broadcast(MSG_PARAM);
			
			break;

		case CMD_MODE:
			// Change the mode to the one defined by the first
			if(last_cmd.command_count)
			{
				SET_RIBBON_MODE(event_buffer[0]);

				broadcast_cmd(last_cmd.command_type,true);
			}

			break;

		case CMD_SET_PARAM:
#ifdef RIBBON_DEBUG_ON
			uart_string_P(PSTR("CMD: Set Param\r\n"),16);
#endif
			if(last_cmd.command_count>=3)
			{
				switch(event_buffer[0]>>6)
				{
					case GLOBAL_PARAM:
#ifdef RIBBON_DEBUG_ON
					uart_string_P(PSTR("CMD: Set Global Param\r\n"),23);
#endif
						command_valid = set_mapped_global_param(
											event_buffer[1],
											event_buffer[2]);
					break;

					case SENSOR_PARAM:
#ifdef RIBBON_DEBUG_ON
					uart_string_P(PSTR("CMD: Set Sensor Param\r\n"),23);
#endif
						command_valid = set_mapped_sensor_param(
											event_buffer[0]&0x3F,
											event_buffer[1],
											event_buffer[2]);
					break;

					case CHANNEL_PARAM:
#ifdef RIBBON_DEBUG_ON
					uart_string_P(PSTR("CMD: Set Channls Param\r\n"),23);
#endif
						command_valid = set_mapped_channel_param(
											event_buffer[0]&0x3F,
											event_buffer[1],
											event_buffer[2]);
					break;

#ifdef RIBBON_DEBUG_ON					
					default:
					uart_string_P(PSTR("CMD: Invalid Param Type\r\n"),25);
#endif
				}
			}

			broadcast_cmd(last_cmd.command_type,command_valid);
			break;

		case CMD_RESET:
#ifdef RIBBON_DEBUG_ON
				uart_string_P(PSTR("CMD: Hard Reset\r\n"),17);
#endif
				broadcast_cmd(last_cmd.command_type,true);

				ribbon_hard_reset();
			break;

		case CMD_EEPROM_RESET:
#ifdef RIBBON_DEBUG_ON
				uart_string_P(PSTR("CMD: Eprm Reset\r\n"),17);
#endif
				ribbon_eeprom_reset();
			break;

		case CMD_RECAL:
				ribbon_recal();

				broadcast_cmd(last_cmd.command_type,true);
			break;

		case CMD_SET_SENSOR:
				if(last_cmd.command_count>=4)
				{
					command_valid = ribbon_set_sensor(	event_buffer[0],
														event_buffer[1],
														event_buffer[2],
														event_buffer[3]);
				}

				broadcast_cmd(last_cmd.command_type,command_valid);
			break;
		case CMD_SET_SENSOR_COUNT:
				if(last_cmd.command_count>=1)
				{
					command_valid = ribbon_set_sensors(	event_buffer[0]);
				}

				broadcast_cmd(last_cmd.command_type,command_valid);
			break;

		case CMD_DUMP_CONFIG:
				broadcast_config();
				broadcast_cmd(last_cmd.command_type,command_valid);
			break;

		default:
			return;
				
	}

	
}


void parse_rx_event(void)
{
	uint8_t rx;
	uint16_t rx_timeout;
	
#if (RIBBON_DEBUG_MODE==DEBUG_RX)
	uint8_t print_small_buffer = 0;
#endif	

	/* If the time since the last event is more than 25ms then reset parsing */
	if(current_time_ms_touch-last_cmd_time > 25U )
	{
		prs_flags = 0;
	}

	last_cmd_time = current_time_ms_touch;

	rx_timeout = current_time_ms_touch+50u;	// Set a timeout so we don't neglect the rest of the app

	rx = uart_unbuffer_rx(); 		// Grab the top char, (make sure to check uart_avail before)

	do
	{				
		while(!prs_flags)						// Scan for first product id
		{
			if(rx == ribbon_library_info.pid[0])// Match the first product id
			{
				prs_flags = PRS_PRID1_FLAG;		// Store the match
	
#if (RIBBON_DEBUG_MODE==DEBUG_RX)
				small_buffer_pos = 0;
				small_buffer[small_buffer_pos++] = rx;
#endif
			}
			
			if(uart_rx_avail())				// Get next char
				rx = uart_unbuffer_rx();
			else
				return;						// No more data
		}

		if(!(prs_flags & PRS_ADDRS_FLAG))		// Check if we alread got the address
		{
#if (RIBBON_DEBUG_MODE==DEBUG_RX)
			small_buffer[small_buffer_pos++] = rx;
#endif

			// The address matches if we have a global address,
			// if the sent address is global, or if the sent
			// address matches this address
			if(rx==ER_GLOBAL_ADDRESS || (rx<ER_MAX_ADDRESSES && ribbon_library_info.address==ER_GLOBAL_ADDRESS) || 
					ribbon_library_info.address == rx)
			{

				prs_flags |= PRS_ADDRS_FLAG;	// Store the address match

				// Init check sum
				prs_cksum = ribbon_library_info.pid[0] + rx;
			
				rx_timeout += 25u;							// Since the message is for us,
															// wait a bit longer
				while(!uart_rx_avail() && 
						current_time_ms_touch<rx_timeout);	// We got a matching address
															// Wait for the next char

				if(uart_rx_avail())
					rx = uart_unbuffer_rx();
				else
					break;
			}
			else
			{
#if (RIBBON_DEBUG_MODE==DEBUG_RX)
				uart_byte('a');
				uart_byte('d');
				uart_byte('r');

				print_small_buffer = 1;
#endif
				
				prs_flags = 0;	// Address was wrong, need to recheck this char

				continue;
			}
		}

		if(!(prs_flags & PRS_PRID2_FLAG))			// Look for the second sync flag
		{
			if(rx == ribbon_library_info.pid[1] )
			{
				prs_flags |= PRS_PRID2_FLAG;		// Store the match

#if (RIBBON_DEBUG_MODE==DEBUG_RX)
				small_buffer[small_buffer_pos++] = rx;
#endif

				prs_cksum += rx;					// Update checksum

				
				while(!uart_rx_avail() && 					// Wait until we get the next
						current_time_ms_touch<rx_timeout);	// char or reach the max timeout

				if(uart_rx_avail())							// If we got a char then read
					rx = uart_unbuffer_rx();				// it and continue
				else
					break;									// Otherwise, return
			}
			else
			{
				prs_flags = 0;						// Looking for second sync flag, but
															// not there so reset and look for the 
				continue;									// first again using this char			
			}
		}

		

		if(!(prs_flags & PRS_CMDTP_FLAG))				// See if we parse the command and type
		{
			last_cmd.command_type = rx>>2;				// Command is the top 6 bits (0-63)

			if(((RIBBON_MODE != ER_CMD_LOOP) && 			// Command is only good if we are in command mode
					(last_cmd.command_type != CMD_MODE)) || // Or if it is a mode change
							last_cmd.command_type>=CMD_COUNT)
			{
#if (RIBBON_DEBUG_MODE==DEBUG_RX)
				uart_byte('c');
				uart_byte('m');
				uart_byte('d');
#endif

				prs_flags = 0;			// The command can not be executed now,
										// If we're not in command mode, the only
				continue;				// command we listen to is the mode command
			}
			else
			{
				switch(rx&0x03)						// Bottom two bits are the data type/size
				{
					case BYTE_DATA:					// 1 byte to follow
						last_cmd.command_count = 1;
						prs_flags |= PRS_CMDCT_FLAG;// Don't need command count flag
						break;
					case WORD_DATA:					// 2 bytes to follow
						last_cmd.command_count = 2;
						prs_flags |= PRS_CMDCT_FLAG;// Don't need command count flag
						break;
					case BLOB_DATA:
						last_cmd.command_count = 4;// 4 bytes will follow
						prs_flags |= PRS_CMDCT_FLAG;
						break;
					case ARRAY_DATA:				// Next byte contains number of bytes
						// wait for next byte
						break;
				}

				prs_cksum += rx;					// Update checksum

#if (RIBBON_DEBUG_MODE==DEBUG_RX)
				small_buffer[small_buffer_pos++] = rx;
#endif

				prs_count = 0u;
				prs_flags |= PRS_CMDTP_FLAG;		// Mark that we got the command type

				while(!uart_rx_avail() && 
						current_time_ms_touch<rx_timeout);	// Wait until we get the next
															// char

				if(uart_rx_avail())
					rx = uart_unbuffer_rx();
				else
					break;
			}
		}

		if(!(prs_flags & PRS_CMDCT_FLAG))
		{
			last_cmd.command_count = rx;
			prs_cksum += rx;

			while(!uart_rx_avail() && 
						current_time_ms_touch<rx_timeout);	// Wait until we get the next
															// char
			if(uart_rx_avail())
				rx = uart_unbuffer_rx();
			else
				break;
		}

		while(prs_count < last_cmd.command_count)
		{
			event_buffer[prs_count] = rx;
			prs_cksum += rx;

#if (RIBBON_DEBUG_MODE==DEBUG_RX)
			small_buffer[small_buffer_pos++] = rx;
#endif

			while(!uart_rx_avail() && 
					current_time_ms_touch<rx_timeout);	// Wait until we need to touch

			if(uart_rx_avail())
				rx = uart_unbuffer_rx();
			else
			{
				return;														
			}

			prs_count++;
		}

		// Execute the check_sum
		if(prs_count==last_cmd.command_count)
		{
			uint8_t compare_check_sum = rx;

#if (RIBBON_DEBUF_MODE==DEBUG_RX)
			small_buffer[small_buffer_pos++] = rx;
#endif
			prs_flags = 0;			// Either way we're done parsing now

			if(compare_check_sum==prs_cksum)
			{
#if (RIBBON_DEBUG_MODE==DEBUG_RX)
				uart_string_P(PSTR("CHECK SUM GD\r\n"),14);
#endif
				handle_message();
			}
#if (RIBBON_DEBUG_MODE==DEBUG_RX)
			else
			{
				uart_string_P(PSTR("CHECK SUM FAIL\r\n"),16);
				continue;					// Check this char again from the start
			}
#endif
			if(uart_rx_avail())
				rx = uart_unbuffer_rx();
			else
				break;
			
		}
	}
	while(uart_rx_avail());

#if (RIBBON_DEBUG_MODE==DEBUG_RX)
	if(small_buffer_pos>2)
		print_small_buffer = 1;

	if(print_small_buffer)
	{
		uint8_t sbp;
		for(sbp = 0;sbp<small_buffer_pos;sbp++)
		{
			uart_byte(small_buffer[sbp]);
		}
		uart_byte('\r');
		uart_byte('\n');
	}
#endif
}



void event_reset(void)
{
	event_count = 0;
	prs_flags = 0;
}

