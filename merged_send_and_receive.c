/*------------------------------------------------------------
 * Final Project: Send and Receiving Application
 *
 * ! forced open state can only be entered while opening and closing
 *----------------------------------------------------------*/

#include "bsp.h"
#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "final.h"

/* debugging */
volatile unsigned int checkVar;

/* modifiable parameters */
const unsigned int isDoor = 1 ;				// for designating addresses for door and remote
const unsigned int totalProgress = 12; 		// time it takes for door to open
const unsigned int validCount = 5000;		// valid count for debouncing
const unsigned int maxAckWait = 5;
const char key[] = "predeterminedkeylongerthanmsg";		// key used for encryption; length longer than msg

/* debouncing parameters */
volatile unsigned char buttonState;					// variable to determine whether button is pressed
volatile unsigned int debounceCounter;		//to use to test for debouncing
volatile unsigned int dbResetCounter;		//to use to test for debouncing

/* global parameters */
volatile unsigned int waitingForAck = 0;	// flag to prevent remote from sending before receiving acknowledgement from door
volatile unsigned int acknowledged = 0;
volatile unsigned int ackWaitCount = 0;
mrfiPacket_t packet;		//the packet to be sent

volatile unsigned int state;		//keeps track of which state door is in
volatile unsigned int door_progress = 0;	//times the closing and opening of door (transition from closing/opening to closed/open)
volatile unsigned int AckWaitCount = 0;

/* Main function for receive application */
void main(void) {
	WDTCTL = WDTPW + WDTHOLD; 	// turn off watch-dog timer

	init_uart();	// initialize uart

	uart_clear_screen();
	uart_puts("\n\nInitializing remote controlled door...\n");
	if (isDoor > 0){
		uart_puts("\nMonitoring Door: \n");
	}
	else{
		uart_puts("\n\Monitoring Remote: \n");
	}

	/* Set a filter address for packets received by the radio
	 *   This should match the "destination" address of
	 *   the packets sent by the transmitter. */

	uint8_t address[] = {0x51,0x34,0xab,0xcd};
	// give the door a different address to avoid conflict
	if (isDoor > 0 ){
		address[1] = 0x33;
	}

	/* Filter setup return value: success or failure */
	unsigned char status;

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

	/* Turn on the radio receiver */
	MRFI_RxOn();
	__bis_SR_register(GIE);

	/* to initialize components for port 1: leds, button -------------------------------*/
	P1IE = 0x00;				// nothing triggers interrupt
	P1IE |= 0x04;				// except button press
	P1IES |= 0x04;				// trigger interrupt at falling edge

	P1REN |= 0x04;				// add resistor to the switch
	P1DIR &= ~0x04;				// set switch to input mode
	P1DIR |= 0x03;				// set LED's to output mode

	P1OUT |= 0x04;					// set switch bit output to high

	P1OUT &= ~RED_LED;			// turn off red and turn on green at first
	P1OUT |= GREEN_LED;
	/*------------------------------------------------------------------------*/

	// initialize state to be closed
	state = CLOSED;

	/* Main loop toggles the proper state */
	while(1){
		busywait(POLLING_INTERVAL);
		checkVar = P1IFG;
		__disable_interrupt();
		checkVar = P1IFG;
		if (isDoor > 0){	// door changes states upon receiving radio packet

			if (state == OPEN) {
				P1OUT |= RED_LED;		// both LED's on
				P1OUT |= GREEN_LED;
				// enable interrupts, and go to sleep
				__bis_SR_register(LPM4_bits + GIE);
			}

			else if (state == CLOSED) {
				P1OUT &= ~RED_LED;		// both LED's off
				P1OUT &= ~GREEN_LED;
				// enable interrupts, and go to sleep
				__bis_SR_register(LPM4_bits + GIE);
			}
			else if (state == OPENING) {
				P1OUT &= ~RED_LED;
				P1OUT |= GREEN_LED;		// green on, red off

				door_progress++;
				if (door_progress >= totalProgress) {	//changes to open state after a period of time (simulate actual door closing)
					state = OPEN;
					door_progress = totalProgress;

					uart_puts("\ndoor opened\n");
					// door lets the remote know the door is now opened
					char openedMsg[20];
					strcpy (openedMsg, "door opened");
					send(openedMsg);
				}
			}
			else if (state == CLOSING) {
				P1OUT &= ~GREEN_LED;
				P1OUT |= RED_LED;		// red on, green off

				door_progress--;
				if (door_progress < 1) {	//changes to close state after a period of time
					state = CLOSED;
					door_progress = 0;
					uart_puts("\ndoor closed\n");
					// door lets the remote know the door is now closed
					char closedMsg[20];
					strcpy (closedMsg, "door closed");
					send(closedMsg);
				}
			}
			else if (state == STOPOP) {
				P1OUT &= ~RED_LED;
				P1OUT ^= GREEN_LED;				// blink green, red off

			}
			else if (state == STOPCL) {
				P1OUT &= ~GREEN_LED;
				P1OUT ^= RED_LED;				// blink red, green off

			}
			else if (state == FORCEOP){
				P1OUT |= RED_LED;				// blink green while red stays on
				P1OUT ^= GREEN_LED;
			}
			else {
				__no_operation();
				break;							// something's wrong... unknown state
			}

		}
		else {	// the remote blinks red
			P1OUT ^= RED_LED;
			P1OUT ^= GREEN_LED;
		}
		checkVar = P1IFG;
		__enable_interrupt();
	}
}


/* Function to execute upon receiving a packet
 *   Called by the driver when new packet arrives */
void MRFI_RxCompleteISR(void) {

	/* Read the received data packet */
	mrfiPacket_t	packet;
	MRFI_Receive(&packet);

	char msg[20];
	int msgL = sizeof(packet.frame);
	int i;
	for (i = 9; i < msgL; i++ ){
		msg[i-9] = packet.frame[i];
	}
	// ?? how to get the message from packet
	//	strcpy(msg, packet.frame[9]);
	//	int i;
	//	for (i = 0; i < strlen(msg) - 1; i++){
	//		msg[i] ^= key[i];
	//	}

	/*
	 * In this case we're going to change the state to the
	 * next in the state machine. We will also print out UART
	 * of the packet information.
	 * ---------------------------------------------------*/
	if (isDoor > 0){
		// door changes state and sends acknowledgement once receiving a packet
		char ackMsg[20];
		if (state == OPEN || state == STOPOP) {
			state = CLOSING;
			strcpy (ackMsg, "closing door");
		}
		else if (state == CLOSED || state == STOPCL) {
			state = OPENING;
			strcpy (ackMsg, "opening door");
		}
		else if (state == OPENING) {
			state = STOPOP;
			strcpy (ackMsg, "stop opening");
		}
		else if (state == CLOSING) {
			state = STOPCL;
			strcpy (ackMsg, "stop closing");
		}
		else if (state == FORCEOP) {
			state = OPENING;
			strcpy (ackMsg, "opening forced door");
		}
		else {
			uart_puts("\n door is in unknown state\n");
			return;
		}
		send(ackMsg);
	}

	// remote has received a packet: change states or see if its just an ack
	else if (isDoor == 0 ){

		// check message to see if remote is receiving just ack or forced open signal
		if ( strncmp(msg, "door obstructed", 15) == 0){
			// door obstructed, change state
			state = FORCEOP;
		}
		else if (strncmp(msg, "door opened", 11) == 0){
			// door opened, change state
			state = OPEN;
		}
		else if (strncmp(msg, "door closed", 11) == 0){
			// door closed, change state
			state = CLOSED;
		}
		else if (waitingForAck == 1){
			// if just received acknowledgement, set flag and change state

			waitingForAck = 0;
			uart_puts("\nAcknowledging: ");
			if (strncmp(msg, "closing door", 12) == 0) {
				state = CLOSING;
			}
			else if (strncmp(msg, "opening door", 12) == 0) {
				state = OPENING;
			}
			else if (strncmp(msg, "stop opening", 12) == 0) {
				state = STOPOP;
			}
			else if (strncmp(msg, "stop closing", 12) == 0) {
				state = STOPCL;
			}
			else if (strncmp(msg, "opening forced door", 19) == 0) {
				state = CLOSING;
			}
			else {
				// uart_puts("\n remote is in unknown state");
				// return;
			}
		}

	}
	uart_puts("\n");
	uart_puts(msg);
	uart_puts("\n");

	P1IFG &= 0x00;			// clear flags
}

/* button press interrupt handler */
#pragma vector=PORT1_VECTOR
__interrupt void switch_handler (void)
{
	checkVar = P1IFG;

	debounceCounter = 0;	// initialize counters to 0
	dbResetCounter = 0;
	while(1)
	{
		buttonState = P1IN & 0x04;

		if (debounceCounter < validCount)	{
			debounceCounter++;		// increment counter at each cycle
		}

		if (buttonState > 0)	{
			debounceCounter = 0;	// reset counter if switch signal goes low/high
			dbResetCounter++;		// record the number of times we are resetting because of noise
		}

		if (debounceCounter >= validCount)	{
			// switch debounced and registered

			if (isDoor > 0 ){
				// the door senses obstruction, not waiting for ack, thus cannot continue opening or closing
				if (state == OPENING || state == CLOSING){
					state = FORCEOP;
					uart_puts("\ndoor obstructed\n");
					// send this information to remote

					char obstMsg[20];
					strcpy (obstMsg, "door obstructed");
					send (obstMsg);
				}
			}
			else if (isDoor == 0 ){
				// check if remote is waiting for acknowledgement
				if (waitingForAck > 0){
					if (ackWaitCount >= maxAckWait) {
						uart_puts("\nNo response from Door. Please check distance and restart. \n");
					}
					else{
						ackWaitCount++;
						uart_puts("\nWaiting for acknowledgement...\n");
					}
				}
				// not waiting for ack, so send the next command
				else{
					// initialize msg to be 20 characters
					char sendingMsg[20];
					// the remote sends signal to door when button is pressed
					if (state == OPEN || state == STOPOP){
						strcpy (sendingMsg, "closing door");
					}
					else if (state == OPENING){
						strcpy (sendingMsg, "stop opening");
					}
					else if (state == CLOSED || state == STOPCL){
						strcpy (sendingMsg, "opening door");
					}
					else if (state == CLOSING){
						strcpy (sendingMsg, "stop closing");
					}
					else if (state == FORCEOP){
						strcpy (sendingMsg, "opening obstructed door");
					} else {
						strcpy (sendingMsg, "unknown state");
					}

					send(sendingMsg);
					waitingForAck = 1;
				}
			}
			break;				// exit loop
		}
		else if (dbResetCounter >= validCount)	{
			break;				// exit loop if the button press isn't long enough
		}
	}
	P1IFG &= 0x00;			// clear flags
	//	__bic_SR_register_on_exit(CPUOFF);		// exit low power mode
}

/* assign source and destination addresses and sends packet */
void send(char msg[]){
	/* First byte of packet frame holds message length in bytes */
	packet.frame[0] = strlen(msg) + 8;	/* Includes 8-byte address header */

	if (isDoor > 0){
		/* Next 8 bytes are addresses, 4 each for source and dest. */
		packet.frame[1] = 0x51;		/* Destination */
		packet.frame[2] = 0x34;
		packet.frame[3] = 0xab;
		packet.frame[4] = 0xcd;

		packet.frame[5] = 0x02;		/* Source */
		packet.frame[6] = 0x00;
		packet.frame[7] = 0x11;
		packet.frame[8] = 0x02;
	}
	else {
		packet.frame[1] = 0x51;		/* Destination */
		packet.frame[2] = 0x33;
		packet.frame[3] = 0xab;
		packet.frame[4] = 0xcd;

		packet.frame[5] = 0x02;		/* Source */
		packet.frame[6] = 0x00;
		packet.frame[7] = 0x01;
		packet.frame[8] = 0x02;
	}


	/* Remaining bytes are the message/data payload */
	strcpy( (char *) &packet.frame[9] , msg );

	/* Transmit the packet over the radio if switch is pressed */
	MRFI_Transmit(&packet , MRFI_TX_TYPE_FORCED);

}

/* Parameterized "busywait" helper function */
void busywait(unsigned int count) {
	int i;
	int inputCount;
	inputCount = count;
	for (i = 0; i < 10; i++) {
		while(count > 0) {
			count--;
			__no_operation();
		}
		count = inputCount;
	}
}

