/*
  UART.h - Super basic UART debugging library
*/

#include "uart.h"

/* The buffer for storing incoming data */
static uint8_t uart_rx_buffer[UART_MAX_RX_BUFFER_SIZE];
/* The position in the buffer */
static uint8_t uart_rx_buffer_end = 0u;
static uint8_t uart_rx_buffer_start = 0u;

static volatile uint8_t dropped_char;

#define ROLLOVER(x, max) x = ++x >= max ? 0 : x

void uart_init(uint16_t baud_calc) {
  /* Set baud rate */
  UBRR0H = (uint8_t)(baud_calc >> 8);
  UBRR0L = (uint8_t)baud_calc;
  /* Enable transmitter AND receiver */
  UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
  /* Set frame format: 8data, 1stop bit */
  UCSR0C = (3 << UCSZ00);

  uart_rx_buffer_end = 0;
  uart_rx_buffer_start = 0;
}

void uart_byte(uint8_t b) {
  /* Wait for empty transmit buffer */
  while (!(UCSR0A & (1 << UDRE0)))
    ;

  /* Put data into buffer, sends the data */
  UDR0 = b;

  UCSR0A |= (1 << TXC0);
}

void uart_string(const char *s, uint8_t length) {
  uint8_t i = 0;

  while (i++ < length && *s) {
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1 << UDRE0)))
      ;

    /* Put data into buffer, sends the data */
    UDR0 = *s;

    s++;
  }

  UCSR0A |= (1 << TXC0);
}

void uart_string_P(const char *s, uint8_t length) {
  uint8_t i = 0;

  while (i++ < length && pgm_read_byte(s)) {
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1 << UDRE0)))
      ;

    /* Put data into buffer, sends the data */
    UDR0 = pgm_read_byte(s++);
  }

  UCSR0A |= (1 << TXC0);
}

uint8_t uart_rx_avail(void) {
  return uart_rx_buffer_start ^
         (*(volatile uint8_t *)&(
             uart_rx_buffer_end));  // rx_in modified by interrupt !
}

uint8_t uart_unbuffer_rx(void) {
  uint8_t data;

  while (!uart_rx_avail())
    ;  // until at least one byte in

  data = uart_rx_buffer[uart_rx_buffer_start];  // get byte

  ROLLOVER(uart_rx_buffer_start, UART_MAX_RX_BUFFER_SIZE);

  UCSR0B |= (1 << RXCIE0);  // Make sure interrupts are enabled

  return data;
}

void uart_buffer_rx(void) {
  uint8_t i = uart_rx_buffer_end;

  ROLLOVER(i, UART_MAX_RX_BUFFER_SIZE);

  if (i == uart_rx_buffer_start) {  // buffer overflow
    UCSR0B &= ~(1 << RXCIE0);       // disable RX interrupt
    return;
  }

  uart_rx_buffer[uart_rx_buffer_end] = UDR0;
  uart_rx_buffer_end = i;
}

void uart_flush(void) {
  while (UCSR0A & (1 << RXC0)) dropped_char = UDR0;

  uart_rx_buffer_start = uart_rx_buffer_end = 0;
}

ISR(USART0_RX_vect) {
  uint8_t i = uart_rx_buffer_end;

  ROLLOVER(i, UART_MAX_RX_BUFFER_SIZE);

  if (i == uart_rx_buffer_start) {  // buffer overflow
    UCSR0B &= ~(1 << RXCIE0);       // disable RX interrupt
    return;
  }

  uart_rx_buffer[uart_rx_buffer_end] = UDR0;
  uart_rx_buffer_end = i;
}
