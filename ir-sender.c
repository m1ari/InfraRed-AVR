/*
 * IR - Sender & Receiver
 * Author: Mike Axford <mfaxford@gmail.com>
 * Ver 0.1 - 02/04/2016

 * Designed to run on a ATMega 328p
 * hC-06 Bluetooth module connected to the UART for remote access
 * Serial on PD0 & PD1
 * Status LED on PD4
 * IR Detector on PD2 (INT0) (could add 2nd detector on PD3 (INT1))
 * IR LED carrier created with Timer0 (OC0A - PD6 or OC0B - PD5)
 * Pulse length measuring done with Timer1
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Serial Port setup
//#define BAUD 250000				// Baud Rate for serial Port, (Fast for FTDI debug)
#define BAUD 9600				// Baud Rate for serial Port, (HC-06 uses 9600 by default so use that)
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)	// set baud rate value for UBRR
#define SERIAL_TXBUF 80
#define SERIAL_RXBUF 40
char serial_tx[SERIAL_TXBUF+1];
uint8_t serial_txpos=0;

char serial_rx[SERIAL_RXBUF+1];
uint8_t serial_rxpos=0;
int serial_rxready=0;

// IR Detetor Setup
#define IR_MAX 70				// Max pulse times we can store
#define IR_CARRIER 38000UL			// IR Carrier rate (38KHz)
#define IR_RX					// Enable IR Detection (needs something connected to INT0 pin)
#define IR_TX					// Enable IR Transmission (Not used)
int16_t raw_ir[IR_MAX];
int ir_pos=0;
int ir_ready=0;
#define IRRANGE(x,l,h)  (abs(x) > l) && (abs(x) < h)

// http://www.sbprojects.com/knowledge/ir/rc5.php
struct irrc5_t {
	uint8_t header:3;		// Start bits and Toggle bit
	uint8_t	address:5;		// Address bits
	uint8_t command:6;		// Command bits
};

struct irnec_t {
	uint8_t address;		// Device
	uint8_t	address2;		// Sub Device	(In NEC1 this is inverted Device)
	uint8_t command;		// Command
	uint8_t _command;		// Invereted Command
};

/* Put data into the serial Buffer and enable interrupts */
void serial_send(const char* tx){
	if (SERIAL_TXBUF < strlen(tx)){
		// Error
	} else {
		// Wait until the the buffer is free
		// TODO, this causes issues if serial_send is called from an ISR
		// If we change to a circular buffer we don't need to wait as long as it has space
		while (serial_txpos != 0)
			_delay_ms(1);

		strncpy(serial_tx,tx,SERIAL_TXBUF);
		UCSR0B |= (1<<UDRIE0);		// Enable Transmit interrupt
	}
}

void serial_rx_reset(){
	serial_rxready=0;
	serial_rxpos=0;
	memset(serial_rx,0,SERIAL_RXBUF+1);
}

/* Interrupt routine for sending data to the Host */
ISR(USART_UDRE_vect){
	// See if there's something to send
	if (serial_tx[serial_txpos] != '\0'){
		while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
		UDR0 = serial_tx[serial_txpos];          
		serial_tx[serial_txpos]='\0';
		serial_txpos++;
	}

	//   If we're at the end               Or the next character is null
	if ((serial_txpos > SERIAL_TXBUF) | (serial_tx[serial_txpos] == '\0')) {
		serial_txpos=0;			// Reset to start of string
		UCSR0B &= ~(1<<UDRIE0);		// Clear Transmit interrupt
		
	}
}

/* Interrupt routine for Receiving Data */
ISR(USART_RX_vect){
	char data = UDR0;

	// Echo output
	char buff[3];
	if ('\r' == data)
		sprintf(buff,"\r\n");
	else
		sprintf(buff,"%c",data);
	serial_send(buff);		// TODO This could hang if there's other stuff in the serial buffer

	// Update Buffer
	if ( ('\r' == data) || ('\n' == data) ) {		// Newline
		serial_rxready=1;
	} else {
		serial_rx[serial_rxpos++]=data;
		if (SERIAL_RXBUF == serial_rxpos ){		// Buffer full
			serial_send("rx Buffer overflow\r\n");	// TODO More hang risk
			serial_rxready=1;
		}
	}
}

#ifdef IR_RX
ISR(INT0_vect){
	// Interrupt from IR receiver
	static uint8_t state=1;		// Detector idles high

	// Read state
	uint8_t newstate = ((PIND & _BV(PIND2)) == _BV(PIND2));

	// Check for overflow
	if (ir_pos >= IR_MAX) {
		TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));	// Stop Timer 1
		ir_pos=0;		// and/or set ir_ready
		memset(raw_ir,0,(sizeof(int16_t) * IR_MAX));
		UDR0 = '#';
		//serial_send("IR: Overflow\r\n");	// serial_send from ISR is bad
		return;
	}

	// Check we're not currently processing a packet
	if (ir_ready){
		TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));	// Stop Timer 1
		UDR0 = '@';
		//serial_send("IR: update when processing\n\n");	// serial_send from ISR is bad
		return;
	}

	// Check Timer1 state, if stopped, start it
	if ((TCCR1B & (_BV(CS10) | _BV(CS11) | _BV(CS12)) )  == 0 ){	// Clock stopped
		TCCR1B |= _BV(CS11); // CLK/8 (1 tick per uS @ 8MHz)	// Start Clock
		ir_pos=0;
		memset(raw_ir,0,(sizeof(int16_t) * IR_MAX));
	} else {
		// Read in Timer1 value
		uint16_t time = TCNT1;

		// Reset to 0
		TCNT1 = 0;

		// Determine Pulse type and store time
		if ( (0 == state) && (1 == newstate) ) {		// Space (Carrier Absent)
			raw_ir[ir_pos++] = time;
		} else if ( (1 == state) && (0 == newstate) ) {		// Mark (Carrier present)
			raw_ir[ir_pos++] = -1 * time;
		} else if ( state == newstate) {			// Bad Interupt
			serial_send(".");
		} else {						// Bad Code
			serial_send("x");
		}
	}

	state=newstate;
}

ISR(TIMER1_COMPA_vect){			// Compare OCR1A
	ir_ready=1;
	TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));	// Stop Timer 1
}
//#define TIMER1_COMPB_vect _VECTOR(12)  /* Timer/Counter1 Compare Match B */


ISR(TIMER1_OVF_vect){	// Timer1 Overflow
	TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));	// Stop Timer 1
}

#endif

void NEC(struct irnec_t data, uint8_t ver){
	uint8_t byte;
	int8_t i,c;
	// Might be better if we can handle turning the pin on and off using the pwm interrupt
	// using ISR(TIMER0_COMPB_vect) as per http://forum.arduino.cc/index.php?topic=102430.0#msg769342

	// We could also do this by starting and stopping the Timer

	// TODO, We ought to store the current value so we can restore to the sane
	EIMSK &= ~_BV(INT0);		// Disable Int0

	// Header 9ms On, 4.5ms off
	DDRD |= _BV(PIND6);
	_delay_ms(9);
	DDRD &= ~_BV(PIND6);
	_delay_ms(4.5);

	for (c=0;c<=3;c++){
		switch (c){
		case 0:
			byte=data.address;
		break;
		case 1:
			if (2 == ver)
				byte=data.address2;
			else
				byte=~data.address;
		break;
		case 2:
			byte=data.command;
		break;
		case 3:
			byte=~data.command;
		break;
			
		}

		for (i=0; i<=7; i++){
			DDRD |= _BV(PIND6);
			_delay_us(560);
			DDRD &= ~_BV(PIND6);
			if (byte & _BV(i)){
				_delay_ms(1.69);	//1
			} else {
				_delay_us(560);		//0
			}
		}
	}

	// End Pulse
	DDRD |= _BV(PIND6);
	_delay_us(560);
	DDRD &= ~_BV(PIND6);

	EIMSK |= _BV(INT0);		// Enable Int0
}

uint8_t IRisRC5(void){
	uint8_t count;
	uint16_t t=889;					// The first part of bit 0 is low
	char buff[25];

	//ir_pos between 16 and 28
	if ( (ir_pos >=16) && (ir_pos <=28) ){
		// Determine length of packet
		for (count=0; count<ir_pos; count++){
			if (IRRANGE(raw_ir[count],780,1000)) 			// Short Pulse
				t+= 889;
			else if (IRRANGE(raw_ir[count],1628,1850)) 		// Long Pulse
				t+=1778;
			else {
				sprintf(buff,"RC: Bad pulse %i\r\n",raw_ir[count]);
				serial_send(buff);
			}
		}
		sprintf(buff,"RC: length %i\r\n",t);
		serial_send(buff);
		if ((t==24003) || (t==24892))					// 14 bits (24003 or 24892) depends on final bit
			return 1;
	}
	return 0;
}

struct irrc5_t IRDecodeRC5(void){
	struct irrc5_t data = {0,0,0};
	uint8_t bit=0;
	uint8_t count;
	uint16_t t=889;					// The first part of bit 0 is low

	//ir_pos between 16 and 28
	// sum raw_ir[0] .. raw_ir[ir_pos] should be standard
	for (count=0; count<=ir_pos; count++){
		if (IRRANGE(raw_ir[count],780,1000)){ 			// Short Pulse
			if ((t%1778) == 889) {				// End of a bit
				if (bit<=2){				// Header
					if (raw_ir[count] >0)
						data.header |= 1<<(2-bit);
				} else if ((bit>2) && (bit<=7) ){	// Address
					if (raw_ir[count] >0)
						data.address |= 1<<(7-bit);
				} else if ((bit>7) && (bit<=13) ){	// Command
					if (raw_ir[count] >0)
						data.command |= 1<<(13-bit);
				} else {
					serial_send("IR: Parse error\r\n");
				}
				bit++;
			}
			t+=889;
		} else if (IRRANGE(raw_ir[count],1628,1850)) {		// Long Pulse
			if (bit<=2){				// Header
				if (raw_ir[count] >0)
					data.header |= 1<<(2-bit);
			} else if ((bit>2) && (bit<=7) ){	// Address
				if (raw_ir[count] >0)
					data.address |= 1<<(7-bit);
			} else if ((bit>7) && (bit<=13) ){	// Command
				if (raw_ir[count] >0)
					data.command |= 1<<(13-bit);
			} else {
				serial_send("IR: Parse error\r\n");
			}
			bit++;
			
			t+=1778;
		} else {
		}

	}

	return data;
}

// Handle NEC and NECx2 together then determine which it is in main loop
uint8_t IRisNEC(void){
	// TODO also check the number of samples? 
	if (IRRANGE(raw_ir[0],3000,26000) && IRRANGE(raw_ir[1],4400,4500) ) {
		return 1;
	}
	return 0;
}

/* Samsung (Channel1)
4474 -4448
540 -1668 540 -1668 541 -1667 540 -563 540 -563 540 -563 540 -564 539 -564	1110 0000
540 -1668 540 -1669 540 -1669 540 -564 541 -564 540 -564 540 -564 540 -563	1110 0000
541 -564 540 -564 539 -1669 539 -565 540 -564 540 -564 540 -563 541 -563	0010 0000
540 -1669 541 -1668 540 -563 540 -1669 540 -1668 540 -1668 540 -1668 540 -1668	1101 1111
540 */
struct irnec_t IRDecodeNEC(void){
	struct irnec_t data={0,0,0,0};
	uint8_t c;

	// Address Byte
	for (c=0; c<=7; c++){
		if (IRRANGE(raw_ir[(c*2)+3],1600,1700))
			data.address |= _BV(c);
	}

	// 2nd Address Byte (~Address in NEC1, sub device in NECx2)
	for (c=0; c<=7; c++){
		if (IRRANGE(raw_ir[(c*2)+19],1600,1700))
			data.address2 |= _BV(c);
	}

	// Command Byte
	for (c=0; c<=7; c++){
		if (IRRANGE(raw_ir[(c*2)+35],1600,1700))
			data.command |= _BV(c);
	}

	// Inverted Command Byte
	for (c=0; c<=7; c++){
		if (IRRANGE(raw_ir[(c*2)+51],1600,1700))
			data._command |= _BV(c);
	}
	return data;
}

ISR(WDT_vect){
	uint8_t i;
	for (i=0;i<6;i++) {
		PORTD ^= _BV(4);
		_delay_ms(100);
	}
}

int main(void) {	
	wdt_disable();

	serial_rx_reset();
	memset(serial_tx,0,SERIAL_TXBUF+1);
	serial_txpos=0;

	// Setup LED
        // Green on PD4
        DDRD |= _BV(PIND4);
        PORTD |= _BV(PIND4);

	//Uart 8n1
	UBRR0H = (BAUDRATE>>8);			// shift the register right by 8 bits
	UBRR0L = BAUDRATE;			// set baud rate
	UCSR0B |= (1<<TXEN0); // | (1<<UDRIE0);	// Enable Transmit and Interupt	(Also TXCIE0)
	UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);	// Enable Recieve and Interrupt
	UCSR0C |= (1<<USBS0) | (3 << UCSZ00);	// 8 Data bit and 2 stop bit

	// Setup PWM
	// http://letsmakerobots.com/node/17179#comment-41552
	// Toggle twice per cycle
	TCCR0A = _BV(WGM01) | _BV(COM0A0);	// Mode 2 (CTC), Toggle on Compare
	TCCR0B = _BV(CS00);			// No Prescaling
  	OCR0A = (F_CPU/(IR_CARRIER*2L)-1);	// Run at twice the carrier (toggle mode)
	//DDRD |= _BV(PIND6);			// Enable PD6 (OCR0A)

	// Configure Timer1
	// Normal mode
	TCCR1A = 0;	// Normal Mode
	TCCR1B = 0;	// Don't enable clock yet (we enable it on first INT0 transition)
	TIMSK1 = (_BV(OCIE1A) | _BV(TOIE1) );	// Enable Interrupts on (OCIE1A, OCIE1B, TOIE1) 
	// OCR1AH and OCR1AL
	OCR1A = 16000;	// Packet finished after 10ms
	// OCR1A set to trigger when packet complete
	// OCR1B Set to trigger when no more signal ?
	
	// Interrupts on INT0
#ifdef IR_RX	// Enabling this with nothing on the Pin causes issues
	EICRA |= _BV(ISC00);		// Any change causes interrupt
	EIMSK |= _BV(INT0);		// Enable Int0
#endif

	// Setup Watchdog
	wdt_enable(WDTO_1S);	// Only Enables WDE not WDIE
	WDTCSR = _BV(WDIE);	// Add in WDIE
	sei();		// Enable interrupts

	serial_send("Starting\r\n");
	int count=0;
	char buff[SERIAL_RXBUF+7];
	char *tok;
	uint8_t clilevel=0;

	sprintf(buff,"WDT: %#x\r\n",WDTCSR);
	serial_send(buff);
	while (1) {
		wdt_reset();
		_delay_ms(10);
		if (1 == serial_rxready){
			//tok=strtok(serial_rx," ");
			struct irnec_t necdata;
			tok=serial_rx;
			switch (clilevel){
				case 1:	// PVR
					//tok=strtok(NULL," ");
					necdata.address=0;
					necdata.address2=16;
					if (strcmp(tok,"..") == 0)		// Up a level
						clilevel=0;
					else if (strcmp(tok,"power") == 0)	// Power
						necdata.command=0x00;
					else if (strcmp(tok,"up") == 0)		// Up
						necdata.command=0x11;
					else if (strcmp(tok,"left") == 0)	// Left
						necdata.command=0x12;
					else if (strcmp(tok,"ok") == 0)		// OK
						necdata.command=0x13;
					else if (strcmp(tok,"right") == 0)	// Right
						necdata.command=0x14;
					else if (strcmp(tok,"down") == 0)	// Down
						necdata.command=0x15;
					else if (strcmp(tok,"media") == 0)	// Media
						necdata.command=0x6f;
					else if (strcmp(tok,"back") == 0)	// Back
						necdata.command=0x41;
					else if (strcmp(tok,"exit") == 0)	// Exit
						necdata.command=0x16;
					else if (strcmp(tok,"->") == 0)		// Step Forward
						necdata.command=0x67;
					else if (strcmp(tok,"<-") == 0)		// Step Back
						necdata.command=0x66;
					else if (strcmp(tok,"guide") == 0)	// Guide
						necdata.command=0x1b;
					else {
						sprintf(buff,"PVR: Unknown command \"%s\"\r\n",serial_rx);
						serial_send(buff);
						necdata.command=0xff;
					}
					NEC(necdata,2);
				break;

				case 2: // TV
					//tok=strtok(NULL," ");
					necdata.address=7;
					necdata.address2=7;
					if (strcmp(tok,"..") == 0)		// Up a level
						clilevel=0;
					else
						sprintf(buff,"TV: Unknown command \"%s\"\r\n",serial_rx);
						serial_send(buff);
					//necdata.command=31;	// Vol Up
					NEC(necdata,2);
				break;

				default:
					if (strcasecmp(tok,"PVR") == 0)
						clilevel=1;
					else if (strcasecmp(tok,"TV") == 0)
						clilevel=2;
					else
						sprintf(buff,"rx: %s\r\n",serial_rx);
				break;
			}

			if (1 == clilevel)
				serial_send("PVR> ");
			else if (2 == clilevel)
				serial_send("TV> ");
			else
				serial_send("> ");
				
	
			serial_rx_reset();
		}

		//Handle IR Data
		if (1 == ir_ready){
			if ( IRisRC5()) {
				struct irrc5_t data = IRDecodeRC5();
				sprintf(buff,"RC5: %#x, %#x, %#x\r\n",data.header,data.address,data.command);
				serial_send(buff);
			} else if ( IRisNEC()) {
				struct irnec_t data = IRDecodeNEC();
				sprintf(buff,"NEC: %#x, %#x, %#x(%#x}\r\n",data.address,data.address2,data.command,data._command);
				serial_send(buff);
			} else {
				int c;
				for (c=0; c<= ir_pos; c++){
					sprintf(buff,"% 4i ",raw_ir[c]);
					serial_send(buff);
				}
				serial_send("\r\n");
			}
			ir_ready=0;
			ir_pos=0;
			memset(raw_ir,0,(sizeof(int16_t) * IR_MAX));
		}
		if (0 == count)
        		PORTD |= _BV(4);

		if (10 == count)
			PORTD &= ~_BV(4);

		if (100 < count++){
        		//PORTD ^= _BV(4);
			count=0;
		}

		// Sending IR Signal should just be a case of pulsing: DDRD |= _BV(PIND6);
	}
}


/* Background Reading
 * SPI
 * http://maxembedded.com/2013/11/26/the-spi-of-the-avr/
 * USART
 * http://maxembedded.com/2013/09/30/the-usart-of-the-avr/

 * Bit Operations
 * http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=40348

 * Examples
 * http://svn.lee.org/swarm/trunk/dboard/atmega168/UART.c

 * Manual Sections
 *   : Timers
 * 19: SPI 	161
 * 20: USart0	171

	// http://www.avrfreaks.net/comment/1632186#comment-1632186
	// F_CPU / Carrier = Cycles for overflow => 8e6/38e3 = 210 cycles

 * Timer Calculator
 * http://eleccelerator.com/avr-timer-calculator/
 */



/* Sample Remote output

Hauppauge (Channel1)
988 -808 876 -876 875 -877 873 -878 873 -878 874 -877 1728 -850 901 -850 900 -850 901 -850 900 -851 900 -1776 845 -564

NEC Timings should be 564 and 1692, Start 9024 -4512, End gap 40844?
Samsung (Channel1)
4474 -4448
540 -1668 540 -1668 541 -1667 540 -563 540 -563 540 -563 540 -564 539 -564	1110 0000
540 -1668 540 -1669 540 -1669 540 -564 541 -564 540 -564 540 -564 540 -563	1110 0000
541 -564 540 -564 539 -1669 539 -565 540 -564 540 -564 540 -563 541 -563	0010 0000
540 -1669 541 -1668 540 -563 540 -1669 540 -1668 540 -1668 540 -1668 540 -1668	1101 1111
540
19348 4447 -4448
540 -1669 540 -1668 540 -1669 540  -563 541  -563 540  -564 541  -563 541  -563	1110 0000
540 -1668 540 -1668 540 -1669 540  -563 541  -563 541  -563 541  -563 541  -562	1110 0000
541  -563 540  -564 541 -1669 541  -562 541  -563 540  -565 540  -563 541  -563	0010 0000
540 -1669 540 -1668 540  -562 540 -1670 540 -1668 540 -1668 540 -1669 540 -1668	1101 1111
540 0

Humax (Channel1)
8907 -4458
556 -561  513 -586  539 -587  486 -587  513 -615  485 -587  511 -588  512 -586		0000 0000
512 -586  513 -587  512 -585  512 -587  510 -1687  546 -577  511 -587  511 -614		0000 1000
485 -1710  513 -1683  540 -584  540 -585  487 -588  510 -613  482 -589  537 -560	1100 0000
538 -562  535 -562  512 -1710  538 -1684  512 -1710  511 -1710  513 -1711  514 -1710	0011 1111
512    0

8907 -4458
556 -561  513 -586  539 -587  486 -587  513 -615  485 -587  511 -588  512 -586
512 -586  513 -587  512 -585  512 -587  510 -1687  546 -577  511 -587  511 -614
485 -1710  513 -1683  540 -584  540 -585  487 -588  510 -613  482 -589  537 -560
538 -562  535 -562  512 -1710  538 -1684  512 -1710  511 -1710  513 -1711  514 -1710
512  18200  8883 -2230  559  380




Sovos (Channel1)
8947 -4396
591 -524 567 -525 591 -524 592 -1618 591 -523 592 -524 567 -525 591 -524	0001 0000
592 -1618 590 -1641 566 -1641 592 -524 567 -1641 593 -1617 591 -1641 567 -1642	1110 1111
592 -1618 591 -522 593 -523 568 -524 592 -523 593 -523 567 -1642 592 -524	1000 0010
567 -524 591 -1641 566 -1640 591 -1616 591 -1640 566 -1640 592 -523 567 -1640	0111 1101
591
26209 8914 -2188 564 -1641
8945 -2188 563 -524



Hauppage Codes
00	0
01	1
02	2
03	3
04	4
05	5
06	6
07	7
08	8
09	9
0a	*
0b	Red
0c	Radio
0d	menu
0e	#
0f	mute
10	vol up
11	vol down
12	prev chan
13
14	up
15	down
16	left
17	right
18	Videos
19	Muic
1a	pictures
1b	guide
1c	TV
1d
1e	step forward
1f	back/exit
20	chan+
21	chan-
22
23
24	step back
25	ok
26
27
28	green
29	blue
2a
2b
2c
2d
2e
2f
30	pause
31
32	re-wind
33
34	fast forward
35	play
36	stop
37	rec
38	yellow
39
3a
3b	Go(home)
3c
3d	power
3e
3f

*/
