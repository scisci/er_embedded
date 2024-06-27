/*
  UART.h - Super basic UART debugging library
*/

#ifndef UART_h
#define UART_h

#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>


#ifndef UART_MAX_RX_BUFFER_SIZE
	#define UART_MAX_RX_BUFFER_SIZE 32
#endif

uint8_t uart_rx_avail(void);
uint8_t uart_unbuffer_rx(void);

void uart_buffer_rx(void);

/* Inits the UART for debugging */
void uart_init(uint16_t baud_calc);
/* Sends a single byte */
void uart_byte(uint8_t b);
/* Sends a null terminated string, or up to length characters */
void uart_string(const char *str, uint8_t length);

void uart_string_P(const char *str, uint8_t length);

void uart_flush(void);


#endif //UART_h
