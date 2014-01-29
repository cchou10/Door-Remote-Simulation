/*------------------------------------------------------------
 * Example radio "send" application
 *   Sends packets with a data payload in an infinite loop
 *----------------------------------------------------------*/

#include "bsp.h"
#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include <string.h>

/* Useful #defines */
#define RED_SEND_LED 		0x01

/* Function prototypes */
void sleep(unsigned int count);

/* Main function for transmit application */
void main(void) {
	
	/* to initialize components for the button */
	P1IE &= 0xfc;				// make sure LED ports don't trigger interrupt
	P1IE |= 0x04;				// switch bit will trigger interrupt
	P1IES |= 0x04;				// trigger interrupt at falling edge

	P1REN |= 0x04;				// add resistors to the switch
	P1DIR &= 0xfb;				// set switch to input mode
	P1DIR |= 0x03;				// set LED's to output mode
	P1OUT |= 0x04;				// set switch bit output to high
	
	/* Perform board-specific initialization */
	BSP_Init();
	
	/* Initialize minimal RF interface, wake up radio */
	MRFI_Init();
	MRFI_WakeUp();
	
	/* Set red LED to output */
	P1DIR = RED_SEND_LED;
	P1OUT = RED_SEND_LED;
	
	__bis_SR_register(GIE);
	
	/* Main (infinite) transmit loop */
	while(1){
		/* Construct a packet to send over the radio.
		 * 
		 *  Packet frame structure:
		 *  ---------------------------------------------------
		 *  | Length (1B) | Dest (4B) | Source (4B) | Payload |
		 *  ---------------------------------------------------
		 */
		mrfiPacket_t 	packet;
		char msg[] = "ECE3140 rocks!\r\n"; 

		/* First byte of packet frame holds message length in bytes */
		packet.frame[0] = strlen(msg) + 8;	/* Includes 8-byte address header */
		
		/* Next 8 bytes are addresses, 4 each for source and dest. */
		packet.frame[1] = 0x12;		/* Destination */
		packet.frame[2] = 0x34;
		packet.frame[3] = 0xab;
		packet.frame[4] = 0xcd;
		
		packet.frame[5] = 0x02;		/* Source */
		packet.frame[6] = 0x00;
		packet.frame[7] = 0x01;
		packet.frame[8] = 0x02;
		
		/* Remaining bytes are the message/data payload */
		strcpy( (char *) &packet.frame[9] , msg );
		
		unsigned char press = P1IN & 0x04;	// read the switch's input state
		/* Transmit the packet over the radio if switch is pressed */
		if (press > 1) {
			MRFI_Transmit(&packet , MRFI_TX_TYPE_FORCED);
		}
		
		/* Toggle red LED after transmitting, then wait a while */
		P1OUT ^= RED_SEND_LED;
		sleep(60000);
	}
}

/* Function to execute upon receiving a packet
 *   Called by the driver when new packet arrives */
void MRFI_RxCompleteISR(void) {
/*   Since this is a transmit-only application we do nothing.
 *   This function still needs to be defined for the project
 *   to compile properly. */
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
