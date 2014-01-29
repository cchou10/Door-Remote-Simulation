#include "bsp.h"
#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"


/* define global variables ----------------------------------------------------------------*/
extern unsigned int state;
/* 0-open, 1-closed, 2-opening, 3-closing, 4-paused while opening, 5-paused while closing */ 

/* Function prototypes --------------------------------------------------------------------*/
void sleep(unsigned int count);

/* define global functions ----------------------------------------------------------------*/
void send(void);

/* UART printing --------------------------------------------------------------------------*/
/* Transmit a single character over UART interface */
void uart_putc(char c) {
	while(!(IFG2 & UCA0TXIFG)); /* Wait for TX buffer to empty */
	UCA0TXBUF = c;				/* Transmit character */
}

/* Transmit a nul-terminated string over UART interface */
void uart_puts(char *str) {
	while (*str) {
		/* Replace newlines with \r\n carriage return */
		if(*str == '\n') { uart_putc('\r'); }
		uart_putc(*str++);
	}
}

