/*------------------------------------------------------------
 * Example radio "receive" application
 *   Always blinks green LED.
 *   Toggles red LED every time a packet is received.
 *----------------------------------------------------------*/

#include "bsp.h"
#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"

/* Useful #defines */
#define RED_RECEIVE_LED 0x01
#define RED_LED 0x01
#define GREEN_LED   0x02

/* Function prototypes */
void sleep(unsigned int count);

/* Main function for receive application */
void main(void) {
	/* Set a filter address for packets received by the radio
	 *   This should match the "destination" address of
	 *   the packets sent by the transmitter. */
	uint8_t address[] = {0x12,0x34,0xab,0xcd};
	
	/* Filter setup return value: success or failure */
	unsigned char status;
	
	/* Perform board-specific initialization */
	BSP_Init();
	
	/* Initialize minimal RF interface, wake up radio */
	MRFI_Init();
	MRFI_WakeUp();
			
	/* Attempt to turn on address filtering
	 *   If unsuccessful, turn on both LEDs and wait forever */
	status = MRFI_SetRxAddrFilter(address);	
	MRFI_EnableRxAddrFilter();
	if( status != 0) {
		P1OUT = RED_RECEIVE_LED | GREEN_LED;
		while(1);
	}
		
	/* Red and green LEDs are output, green starts on */
	P1DIR = RED_RECEIVE_LED | GREEN_LED;
	P1OUT = GREEN_LED;
	
	/* Turn on the radio receiver */
	MRFI_RxOn();
	
	/* Main loop just toggles the green LED with some delay */
	__bis_SR_register(GIE);
	while(1){
		if (state == 0) {
			P1OUT = RED_LED | GREEN_LED;
		}
		if (state == 1) {
			sleep(60000);
		}
		if (state == 2) {
			P1OUT = GREEN_LED;
		}
		if (state == 3) {
			P1OUT = RED_LED;
		}
	}
}


/* Function to execute upon receiving a packet
 *   Called by the driver when new packet arrives */
void MRFI_RxCompleteISR(void) {
	/* Read the received data packet */
	mrfiPacket_t	packet;
	MRFI_Receive(&packet);
	
	/* We ignore the data in the packet structure.
	 * You probably want to do something with it here. */
	 
	/* Toggle the red LED to signal that data has arrived */
	P1OUT ^= RED_RECEIVE_LED;
}

/* Parameterized "sleep" helper function */
void sleep(unsigned int count) {
	int i;
	for (i = 0; i < 10; i++) {
		while(count > 0) {
			count--;
			__no_operation();
		}
	}
}
