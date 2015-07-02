#ifndef POLARGRAPH_STANDALONE_H_
#define POLARGRAPH_STANDALONE_H_

#include "pff2a/src/pff.h"

#define BAUD_57600
#define MCLK_FREQUENCY		16000000
#define WDT_DIVIDER			8192

#define WDT_FREQUENCY		(MCLK_FREQUENCY / WDT_DIVIDER) / 1000
volatile unsigned long wdtCounter = 0;

// prototypes
unsigned long millis(void);
void delayMillis(unsigned long milliseconds);
void ConfigWDT(void);
void FaultRoutine(void);
void ConfigClocks(void);
void ConfigPorts(void);
void Config_UART(void);
unsigned char wait_button(void);
char main_menu(char);
void setting_menu(void);
void sd_menu(void);
char SD_view_menu(void);
char view_tree(char *);
unsigned long file_menu(void);
void send_commands(unsigned long nr_commands);
char Serial_read();
unsigned char Serial_available();
void Serial_print(char string[], unsigned char line_feed);
int strncmp(const char *s1, const char *s2, size_t n);
void wait_ready();
unsigned char wait_ack();
//int strlen(const char *text);
void wait_connection(void);
void substring(char* string, char* string_out, unsigned char start, unsigned char end);
int pause();

// pins define
// port1
#define nok_reset	BIT0
#define nok_ce		BIT1
#define nok_sdin	BIT3
#define nok_sclk	BIT2
// serial
#define TX			BIT2
#define RX			BIT1
// button
#define left_btn	BIT3
#define right_btn	BIT4
#define right		0
#define left		1

// RX BUFFER
#define rx_buffer_lenght 45
unsigned char rx_index = 0;
unsigned char rx_pointer = 0;
unsigned char rx_lenght = 0;
char rx_buffer[rx_buffer_lenght];
char last_rx_char;
char new_line = 0;
#define path_lenght 40
char path[path_lenght] = "";

char string[30];
char temp_char[10];

unsigned long command_time = 0,command_time2 = 0;
unsigned long total_time = 0;

unsigned char contrast;
char SD_mounted;

unsigned char button_press = 0;

//fat filesystem variables
FATFS fs;					/* File system object */
DIR dir;					/* Directory object */
FILINFO fno;				/* File information */

//////////////////////////////////////////////////////////////////////////
// Return milliseconds from power-up or reset
//////////////////////////////////////////////////////////////////////////
unsigned long millis(){
	return wdtCounter / ((unsigned long)WDT_FREQUENCY);
}
//////////////////////////////////////////////////////////////////////////
// Pause for milliseconds
//////////////////////////////////////////////////////////////////////////
void delayMillis(unsigned long milliseconds){
	unsigned long wakeTime = wdtCounter + (milliseconds * WDT_FREQUENCY);
	while(wdtCounter < wakeTime);
}
//////////////////////////////////////////////////////////////////////////
// ConfigWDT
//////////////////////////////////////////////////////////////////////////
void ConfigWDT(void)
{
	//WDTCTL = WDTPW + WDTHOLD;					// Stop watchdog timer
	WDTCTL = WDTPW + WDTTMSEL + WDTIS0;			// Watchdog timer = SMCLK / 8192
	IE1 |= WDTIE;
}
//////////////////////////////////////////////////////////////////////////
// FaultRoutine
//////////////////////////////////////////////////////////////////////////
void FaultRoutine(void)
{
	//P1OUT = BIT0;								// P1.0 on (red LED)
	while(1);									// TRAP
}
//////////////////////////////////////////////////////////////////////////
// ConfigClocks
//////////////////////////////////////////////////////////////////////////
void ConfigClocks(void)
{
	if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)
	FaultRoutine();							// If calibration data is erased
											// run FaultRoutine()
	BCSCTL1 = CALBC1_16MHZ;					// Set range
	DCOCTL = CALDCO_16MHZ;					// Set DCO step + modulation
	BCSCTL3 |= LFXT1S_2;					// LFXT1 = VLO
	IFG1 &= ~OFIFG;							// Clear OSCFault flag
	BCSCTL2 |= SELM_0 + DIVM_0 + DIVS_0;	// MCLK = DCO/1, SMCLK = DCO/1
}
//////////////////////////////////////////////////////////////////////////
// ConfigPorts
//////////////////////////////////////////////////////////////////////////
void ConfigPorts(void)
{
	P1DIR = 0;
	P2DIR = nok_reset + nok_ce + nok_sdin + nok_sclk;
	P2OUT = 0;
}
//////////////////////////////////////////////////////////////////////////
// ConfigUART
//////////////////////////////////////////////////////////////////////////
void Config_UART()
{
	UCA0CTL1 = UCSWRST;			// UCSI software reset (OFF STATE)
	UCA0CTL0 = 0;
	UCA0CTL1 |= UCSSEL_3;		// SMCLK is clock for UCSI
#ifdef BAUD_9600
	// 9600 @ 16MHz
	UCA0BR0 = 130;
	UCA0BR1 = 6;
	UCA0MCTL = UCBRS1+UCBRS2;	// UCOS16=0; UCBRS=6; UCBRF=0;
#endif
#ifdef BAUD_57600
	// 57600 @ 16MHz
	UCA0BR0 = 17;
	UCA0BR1 = 0;
	UCA0MCTL = UCBRF2 + UCBRF1 + UCOS16;	// UCOS16=1; UCBRS=0; UCBRF=6;
#endif
#ifdef BAUD_115200
	// 115200 @ 16MHz
	UCA0BR0 = 138;
	UCA0BR1 = 0;
	UCA0MCTL = UCBRS0+UCBRS1+UCBRS2;		// UCOS16=0; UCBRS=7; UCBRF=0;
#endif

	P1SEL |= TX + RX;		// config pins
	P1SEL2 |= TX + RX;
	P1DIR |= TX;
	UCA0CTL1 &= ~UCSWRST;	// UCSI software reset (ON STATE)
	//5. Enable interrupts (optional) via UCAxRXIE and/or UCAxTXIE
	IE2 |= UCA0RXIE;	// enable irq in rx
}

int strncmp(const char *s1, const char *s2, size_t n)
{
	unsigned char uc1, uc2;
	/* Nothing to compare?  Return zero. */
	if (n == 0)
		return 0;
	/* Loop, comparing bytes. */
	while (n-- > 0 && *s1 == *s2) {
		/* If we've run out of bytes or hit a null, return zero
		since we already know *s1 == *s2.  */
		if (n == 0 || *s1 == '\0')
			return 0;
		s1++;
		s2++;
	}
	uc1 = (*(unsigned char *) s1);
	uc2 = (*(unsigned char *) s2);
	return ((uc1 < uc2) ? -1 : (uc1 > uc2));
}

//////////////////////////////////////////////////////////////////////////
// Wait for button press
//////////////////////////////////////////////////////////////////////////
unsigned char wait_button(void)
{
	while((P1IN & right_btn)&&(P1IN & left_btn)){}
	if((P1IN & left_btn)== 0) 
	{
		delayMillis(200);
		return 1;
	}
	if((P1IN & right_btn)== 0)
	{
		delayMillis(200);
		return 0;
	}
	return 1;
}

//////////////////////////////////////////////////////////////////
// Tell how many chars in rx buffer								//
//////////////////////////////////////////////////////////////////
unsigned char Serial_available(){
	return rx_lenght;
}
//////////////////////////////////////////////////////////////////
// return first byte received or -1 if no byte in buffer		//
//////////////////////////////////////////////////////////////////
char Serial_read(){
	char byte;

	if(rx_lenght == 0){
		return (char)-1;
	}
	else{
		byte = rx_buffer[rx_pointer];
		rx_buffer[rx_pointer] = 0;
		rx_pointer++;
		if (rx_pointer == rx_buffer_lenght) rx_pointer = 0;
		rx_lenght--;
		return byte;
	}
}
//////////////////////////////////////////////////////////////////
// print string to serial tx buffer								//
//////////////////////////////////////////////////////////////////
void Serial_print(char string[], unsigned char line_feed){
	unsigned char i;
	unsigned char tx_lenght;

	tx_lenght = strlen(string);
		
	for(i=0; i < tx_lenght; i++){
		UCA0TXBUF = string[i];
		while(UCA0STAT & UCBUSY){}
	}
	if (line_feed)
	{
		UCA0TXBUF = 13;
		while(UCA0STAT & UCBUSY){}
		UCA0TXBUF = 10;
		while(UCA0STAT == UCBUSY){}
	}
}

/************************************************************************
 *
 *	Find the length of a string - does not check if we fall off
 *	the end of the text buffer. oops.
 *
 ************************************************************************/
//int strlen(const char *text)
//{
//  int  count=-1;				/* Character counter	*/
//
//  while(text[++count] != '\0') ;		/* Serach for a null	*/
//
//  return(count);				/* Return the position 
//						 * of the NULL-1	*/
//}

//////////////////////////////////////////////////////////////////
// Extract substring from string from start to end				//
//////////////////////////////////////////////////////////////////
void substring(char* string, char* string_out, unsigned char start, unsigned char end){
	unsigned char i;
	
	for(i=0; i<end-start; i++){
		string_out[i] = string[i+start];
	}
}

#endif /*POLARGRAPH_STANDALONE_H_*/
