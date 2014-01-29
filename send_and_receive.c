/*------------------------------------------------------------
 * Final Project: Send and Receiving Application
 * Taken from Example radio "receive" application
 *   Always blinks green LED.
 *   Toggles red LED every time a packet is received.
 * 
 * Can receive and send data packets. Always in receive mode
 * until button is pressed which throws and interrupt and 
 * switches it to send mode. Interrupt also thrown when a
 * packet is received. Initializes at door closed
 *----------------------------------------------------------*/

#include "bsp.h"
#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "final.h"

const unsigned int validCount = 2500;	// valid count for debouncing
const unsigned int closeTime = 10000;	// time it takes for door to close/open

unsigned char switchState;				//initialize variable to determine whether button is pressed
volatile unsigned int debounceCounter;		//to use to test for debouncing
volatile unsigned int dbResetCounter;		//to use to test for debouncing

mrfiPacket_t 	packet;		//the packet to be sent
unsigned int state = 1;		//keeps track of which state door is in, initiates at closed

unsigned int counter_open = 0;	//times the closing and opening of door (transition from closing/opening to closed/open)
unsigned int counter_close = 0;

/* Useful #defines */
#define RED_RECEIVE_LED 0x01
#define RED_LED 0x01
#define GREEN_LED   0x02
#define RED_SEND_LED 		0x01

/* Main function for receive application */
void main(void) {
	/* Set a filter address for packets received by the radio
	 *   This should match the "destination" address of
	 *   the packets sent by the transmitter. */
	uint8_t address[] = {0x12,0x34,0xab,0xcd};
	
	/* Filter setup return value: success or failure */
	unsigned char status;
	
	/* to initialize components for the button -------------------------------*/
	P1IE &= 0xfc;				// make sure LED ports don't trigger interrupt
	P1IE |= 0x04;				// switch bit will trigger interrupt
	P1IES |= 0x04;				// trigger interrupt at falling edge

	P1REN |= 0x04;				// add resistors to the switch
	P1DIR &= 0xfb;				// set switch to input mode
	P1DIR |= 0x03;				// set LED's to output mode
	P1OUT |= 0x04;				// set switch bit output to high
	/*------------------------------------------------------------------------*/
	
	/* Perform board-specific initialization */
	BSP_Init();
	
	/* Initialize minimal RF interface, wake up radio */
	MRFI_Init();
	MRFI_WakeUp();
			
	/* Attempt to turn on address filtering
	 *   If unsuccessful, toggle on both LEDs and wait forever */
	status = MRFI_SetRxAddrFilter(address);	
	MRFI_EnableRxAddrFilter();
	if( status != 0) {
		P1OUT ^= RED_RECEIVE_LED | GREEN_LED;
		while(1);
	}
		
	/* Red and green LEDs are output, green starts on */
	P1DIR = RED_RECEIVE_LED | GREEN_LED;
	
	/* Turn on the radio receiver */
	MRFI_RxOn();
	
	/* Main loop just toggles the proper state */
	__bis_SR_register(GIE);
	while(1){
		if (state == 0) {				//open state
			P1OUT = RED_LED | GREEN_LED;
		}
		else if (state == 1) {			//closed state
			sleep(60000);
		}
		else if (state == 2) {			//opening state
			P1OUT = GREEN_LED;
			counter_open++;
			if (counter_open >= closeTime) {	//changes to open state after a period of time (simulate actual door closing)
				state = 0;
				counter_open = 0;
			}
		}
		else if (state == 3) {			//closing state
			P1OUT = RED_LED;
			counter_close++;
			if (counter_close >= closeTime) {	//changes to close state after a period of time
				state = 1;
				counter_close = 0;
			}
		}
		else if (state == 4) {			//paused while opening state
			P1OUT ^= GREEN_LED;
		}
		else if (state == 5) {			//paused while closing state
			P1OUT ^= RED_LED;
		}
		else {							//forced open state (theoretically) 
			sleep(60000);
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
	 * You probably want to do something with it here. 
	 *
	 * In this case we're going to change the state to the
	 * next in the state machine. We will also print out UART
	 * of the packet information. 
	 * ---------------------------------------------------*/
	 
//	 if (state == 0) {
//	 	state = 4;
//	 }
	 //etcetc. better way to do this?
	 
	 //uart_puts(&packet.frame[9]);
	 uart_puts("sent");
	 
	/* Toggle the red LED to signal that data has arrived */
	P1OUT ^= RED_RECEIVE_LED;
}

/* interrupt handler */
#pragma vector=PORT1_VECTOR
__interrupt void switch_handler (void)
{
	P1IFG &= 0x00;			// clear upon entering interrupt handling
	debounceCounter = 0;	// initialize counters to 0
	dbResetCounter = 0;
	while(1)
	{
		switchState = P1IN & 0x04;
		if (debounceCounter < validCount)	{
			debounceCounter++;		// increment counter at each cycle
		}

		if (switchState > 0)	{
			debounceCounter = 0;	// reset counter if switch signal goes high
			dbResetCounter++;		// record the number of times we are resetting because of noise
		}
		if (debounceCounter >= validCount)	{		//sends the packet, button pressed read while reading through noise
				/* Construct a packet to send over the radio.
		 		* 
		 		*  Packet frame structure:
		 		*  ---------------------------------------------------
		 		*  | Length (1B) | Dest (4B) | Source (4B) | Payload |
		 		*  ---------------------------------------------------
		 		*/
				char msg[] = "signal sent"; 

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
		
				/* Transmit the packet over the radio if switch is pressed */
				MRFI_Transmit(&packet , MRFI_TX_TYPE_FORCED);
		
				/* Toggle red LED after transmitting, then wait a while */
				P1OUT ^= RED_SEND_LED;
				sleep(60000);
				break;			// exit loop
		}
		else if (dbResetCounter >= validCount)	{
			break;				// exit loop if the button press isn't long enough
		}
	}
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

