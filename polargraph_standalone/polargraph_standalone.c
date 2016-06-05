/********************************************************************
*	Polargraph Standalone											*
*																	*
*	Author: Matteo Geromin											*
*	Description : MSP430 launchpad with MSP430G2553 with micro SD	*
*				  and Nokia 8310 LCD. Read polargraph file on SD 	*
* 				  and send command to polargraph controller 		*
* 				  through serial port								*
*																	*
********************************************************************/

#include <msp430G2553.h>
#include <stdlib.h>
#include <string.h>
#include "polargraph_standalone.h"
#include "lcd/Nokia_8310_LCD.h"
#include "drivers/spi.h"
#include "pff2a/src/diskio.h"
#include "pff2a/src/pff.h"

void main( void )
{
	unsigned long k;

	ConfigWDT();
	ConfigClocks();
	ConfigPorts();
	spi_initialize();
	Config_UART();
	// init interrupt
	_BIS_SR(GIE);

	nokia_init();
	nokia_lcd_go(3,0);
	nokia_write_text("POLARGRAPH SD");
	nokia_lcd_go(36,1);
	nokia_write_text("by");
	nokia_lcd_go(0,2);
	nokia_write_text("Matteo Geromin");
	nokia_lcd_go(9,5);
	nokia_write_text("Press ENTER");

	SD_mounted=0;
	contrast = 40;
	nokia_contrast(contrast);

	while(1)
	{
		wait_button();
		switch (main_menu(SD_mounted)){
		case 0:
			sd_menu();
			break;
		case 1:
			if(SD_view_menu())
			{
				k = file_menu();
				send_commands(k);
			}
			break;
		case 2:
			setting_menu();
			break;
		}
	}
}


//////////////////////////////////////////////////////////////////////////
// Open File and start check or send it to the machine
//////////////////////////////////////////////////////////////////////////
unsigned long file_menu()
{
	int res , i;
	unsigned char c[4];
	WORD s1,w;
	unsigned long pointer = 0;
	unsigned long command = 0;

	strcat(path,".chk");
	if(pf_open(path)!= FR_OK) goto open_error;
	for(i=0; i<4; i++)
	{
		pf_read(c, 1, &s1);
		pointer = pointer + ( ((unsigned long)c[0] & 0xFF) << (3-i)*8);
	}
	for(i=0; i<4; i++)
	{
		pf_read(c, 1, &s1);
		command = command + ( ((unsigned long)c[0] & 0xFF) << (3-i)*8);
	}

	if((pointer == 0) && (command == 0))
	{
		path[strlen(path)-4] = 0;
		if(pf_open(path)!= FR_OK) goto open_error;
		nokia_clear();
		nokia_lcd_go(18,1);
		nokia_write_text("Checking");
		while(1)
		{
			i=0;
			res = pf_read(string, 30, &s1);
			if (res != FR_OK) break;
			if(s1==0) break;
			while(string[i++] != 10){}
			string[i] = 0;
			pointer = pointer + strlen(string);
			pf_lseek(pointer);

			nokia_lcd_go(3,2);
			switch(command%128)
			{
				case 0: nokia_write_text("|||||||||||||");
					break;
				case 31: nokia_write_text("/////////////");
					break;
				case 63: nokia_write_text("-------------");
					break;
				case 95: nokia_write_text("\\\\\\\\\\\\\\\\\\\\\\\\\\");
					break;
			}

			if((string[0] == 'C')&&(string[i-6]== ',')&&(string[i-5]== 'E')&&(string[i-4]== 'N')&&(string[i-3]== 'D')&&(string[i-2]== 13)) command++;
			else goto command_error;
			nokia_lcd_go(12,4);
			nokia_write_text("Command nr.");
			nokia_lcd_go(24,5);
			ltoa(command,string);
			nokia_write_text(string);
		}
		strcat(path,".chk");
		if(pf_open(path)!= FR_OK) goto open_error;

		for(i=0; i<4; i++)
		{
			temp_char[i] = (pointer >> (3-i)*8) & 0xFF;
		}
		for(i=0; i<4; i++)
		{
			temp_char[i+4] = (command >> (3-i)*8) & 0xFF;
		}
		pf_write(temp_char, 8, &w);	/* Write data to the file */
		pf_write(0, 0, &w);		/* Finalize the write process */
	}
	nokia_clear();
	nokia_lcd_go(3,1);
	nokia_write_text("Read ");
	ltoa(pointer,temp_char);
	nokia_write_text(temp_char);
	nokia_write_text(" B");
	nokia_lcd_go(21,3);
	ltoa(command,temp_char);
	nokia_write_text(temp_char);
	nokia_lcd_go(15,4);
	nokia_write_text("Commands");
	wait_button();
	path[strlen(path)-4] = 0;
	if(pf_open(path)!= FR_OK) goto open_error;
	return command;

open_error:
	nokia_clear();
	nokia_lcd_go(6,1);
	nokia_write_text("Opening file");
	nokia_lcd_go(21,2);
	nokia_write_text("ERROR!!");
	wait_button();
	return 0;
command_error:
	nokia_clear();
	nokia_lcd_go(6,1);
	nokia_write_text("Command ");
	ltoa(command,string);
	nokia_write_text(string);
	nokia_lcd_go(21,2);
	nokia_write_text("ERROR!!");
	wait_button();
	return 0;
}

void send_commands(unsigned long nr_commands)
{
	int res , i;
	WORD s1;
	unsigned long command = 0;
	unsigned long pointer = 0;
	unsigned long percentual = 0;
	unsigned char retry_cnt = 0;

	wait_connection();

	nokia_clear();
	nokia_lcd_go(3,1);
	nokia_write_text("Set Home");
	nokia_lcd_go(4,3);
	nokia_write_text("Select");
	nokia_lcd_go(0,5);
	nokia_write_text("AUTO    MANUAL");
send_home:
	if(wait_button() == right)
	{
			nokia_lcd_go(3,3);
			nokia_write_text("Hold pen &");
			nokia_lcd_go(3,4);
			nokia_write_text("Press Button");
			nokia_lcd_go(0,5);
			nokia_write_text("              ");
			delayMillis(500);
			wait_button();
			Serial_print("C09,4115,4115,END",0);
			if(wait_ack() == 0) goto send_home;
			else {
					Serial_print("EXEC",0);
				 	goto go_commands;
				 }
	} else {
			nokia_lcd_go(3,4);
			nokia_write_text("AUTOMATIC");
			nokia_lcd_go(0,5);
			nokia_write_text("     WAIT     ");
			Serial_print("C99,4115,4115,END",0);
			if(wait_ack() == 0) goto send_home;
			else {
					Serial_print("EXEC",0);
				 	goto go_commands;
				 }
	}

go_commands:
	nokia_clear();
	nokia_lcd_go(3,1);
	nokia_write_text("Send Commands");
	nokia_lcd_go(3,2);
	nokia_write_text("to Machine");
	nokia_lcd_go(3,5);
	nokia_write_text("Press Button");
	wait_button();
	nokia_clear();
	nokia_lcd_go(12,0);
	nokia_write_text("Sending...");
	
	while(1)
	{
		i=0;
		retry_cnt = 0;
retry:
		res = pf_read(string, 30, &s1);
		if ((res != FR_OK) || (s1==0)) {
			retry_cnt++;
			if (retry_cnt > MAX_RETRY) break;
			goto retry;
		}
		while(string[i++] != 10){}
		string[i] = 0;
		pointer = pointer + strlen(string);
		string[i-2] = 0;
		string[i-1] = 0;
		pf_lseek(pointer);

		command++;
		wait_ready();
resend:
		Serial_print(string,0);
		// Wait ACKnowledge
		if(wait_ack() == 0) goto resend;
		Serial_print("EXEC",0);
		nokia_lcd_go(6,2);
		ltoa(command,temp_char);
		nokia_write_text(temp_char);
		nokia_write_text(" of ");
		ltoa(nr_commands,temp_char);
		nokia_write_text(temp_char);
		nokia_lcd_go(30,4);
		percentual = ( (command * 100) / nr_commands );
		ltoa(percentual,temp_char);
		nokia_write_text(temp_char);
		nokia_write_text(" %");
		if(((P1IN & left_btn)== 0)||((P1IN & right_btn)== 0))
		{
			button_press = 0;
			if(pause() == 0) break;
		}
	}
	// At the end set pen UP
resend_pen_up1:
	Serial_print("C14,END",0);
	if(wait_ack() == 0) goto resend_pen_up1;
	Serial_print("EXEC",0);
	nokia_clear();
	nokia_lcd_go(6,3);
	nokia_write_text("----END----");
	wait_button();
	return;
}

int pause()
{
	nokia_lcd_go(0,0);
	nokia_write_text("----PAUSE----");
	nokia_lcd_go(0,5);
	nokia_write_text("end       cont");
	wait_ready();
resend_pen_up:
	Serial_print("C14,END",0);
	if(wait_ack() == 0) goto resend_pen_up;
	Serial_print("EXEC",0);
	delayMillis(2000);
	if(wait_button()== left) return 0;
	nokia_lcd_go(0,0);
	nokia_write_text("             ");
	nokia_lcd_go(0,5);
	nokia_write_text("             ");
	wait_ready();
resend_pen_down:
	Serial_print("C13,END",0);
	if(wait_ack() == 0) goto resend_pen_down;
	Serial_print("EXEC",0);
	return 1;
}

void wait_connection()
{
	nokia_clear();
	nokia_lcd_go(12,1);
	nokia_write_text("Waiting...");
	nokia_lcd_go(3,2);
	nokia_write_text("Connection...");
	wait_ready();
	nokia_lcd_go(15,4);
	nokia_write_text("ESTABLISHED");
	wait_button();
}

// Wait for string "READY"
void wait_ready()
{
	char string2[10];
	unsigned char i,k;

	while(1)
	{
		while(new_line == 0){}
		while(rx_lenght < 4){}
		{
			k = rx_pointer;
			new_line = 0;
			for(i=0; i<5; i++)
			{
				string2[i] = rx_buffer[k];
				k++;
				if(k == rx_buffer_lenght) k=0;
			}
			if(strncmp(string2,"READY",5) == 0) return;
		}
	}
}

// Wait for string "ACK,"
unsigned char wait_ack()
{
	char string2[10];
	int j=3;
	unsigned char i,k;

	while(j != 0)
	{
		while(new_line == 0){}
		while(rx_lenght < 3){}
		{
			j--;
			k = rx_pointer;
			new_line = 0;
			for(i=0; i<4; i++)
			{
				string2[i] = rx_buffer[k];
				k++;
				if(k == rx_buffer_lenght) k=0;
			}
			if(strncmp(string2,"ACK,",4) == 0) return 1;
		}
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
// Print Main Menu and wait for selection
//////////////////////////////////////////////////////////////////////////
char main_menu(char mounted)
{
	nokia_clear();
	nokia_lcd_go(18,0);
	nokia_write_text("> MENU <");
	nokia_lcd_go(0,1);
	nokia_write_text("==============");
	nokia_lcd_go(0,5);
	nokia_write_text("change   enter");
init_main_menu:
	nokia_lcd_go(0,2);
	nokia_write_text("=> Mount SD");
	if(mounted) nokia_write_text(" OK");
	nokia_lcd_go(0,3);
	nokia_write_text("   SD view");
	nokia_lcd_go(0,4);
	nokia_write_text("   Settings");
	if(wait_button() == left)
	{
		nokia_lcd_go(0,2);
		nokia_write_text("   Mount SD");
		if(mounted) nokia_write_text(" OK");
		nokia_lcd_go(0,3);
		nokia_write_text("=> SD view");
		nokia_lcd_go(0,4);
		nokia_write_text("   Settings");
		if(wait_button() == left)
		{
			nokia_lcd_go(0,2);
			nokia_write_text("   Mount SD");
			if(mounted) nokia_write_text(" OK");
			nokia_lcd_go(0,3);
			nokia_write_text("   SD view");
			nokia_lcd_go(0,4);
			nokia_write_text("=> Settings");
			if(wait_button() == left)
				goto init_main_menu;
			else return 2;
		}
		else return 1;
	}
	else return 0;
}

//////////////////////////////////////////////////////////////////////////
// Print Setting Menu and wait for selection
//////////////////////////////////////////////////////////////////////////
void setting_menu(void)
{
	int i, dir = 1;
	nokia_clear();
	nokia_lcd_go(6,0);
	nokia_write_text("> SETTINGS <");
	nokia_lcd_go(0,1);
	nokia_write_text("==============");
	nokia_lcd_go(0,5);
	nokia_write_text("back       set");
	nokia_lcd_go(0,3);
	nokia_write_text("Set Contrast");
	// contrast from 35 to 105
	nokia_lcd_go(0,5);
	for(i=0; i<((contrast/5)-35);i++)
		nokia_write_text("#");
	while(1)
	{
		if(wait_button() == right)
		{
			if(dir)
				contrast += 5;
			else
				contrast -= 5;
			if(contrast > 105) 
			{
				contrast = 100;
				dir = 0;
			}
			else if(contrast < 35)
			{
				contrast = 40;
				dir = 1;
			}
			nokia_contrast(contrast);
			nokia_lcd_go(0,5);
			nokia_write_text("              ");
			nokia_lcd_go(0,5);
			for(i=0; i<((contrast/5)-35);i++)
				nokia_write_text("#");
		}
		else return;
	}
}

//////////////////////////////////////////////////////////////////////////
// Print SD card Menu and wait for selection
//////////////////////////////////////////////////////////////////////////
void sd_menu(void)
{
	nokia_clear();
	nokia_lcd_go(6,0);
	nokia_write_text("> SD Mount <");
	nokia_lcd_go(0,5);
	nokia_write_text("back     retry");
	while(pf_mount(&fs))
	{
		nokia_lcd_go(6,2);
		nokia_write_text("Press Button");
		nokia_lcd_go(12,3);
		nokia_write_text("to Retry");
		SD_mounted=0;
		//if(wait_button()== left) return;
	}
	nokia_lcd_go(0,2);
	nokia_write_text("              ");
	nokia_lcd_go(0,3);
	nokia_write_text("              ");
	nokia_lcd_go(6,2);
	nokia_write_text(">>  OK!!  <<");
	SD_mounted=1;
}

//////////////////////////////////////////////////////////////////////////
// View SD files and directories
//////////////////////////////////////////////////////////////////////////
char SD_view_menu(void)
{
	unsigned char i;
	for(i=0; i<path_lenght; i++)
	{
		path[i] = 0;
	}

	if(SD_mounted == 0) sd_menu();
	nokia_clear();
	nokia_lcd_go(6,0);
	nokia_write_text("> SD VIEW <");
	nokia_lcd_go(0,5);
	nokia_write_text("change   enter");

		if(view_tree(""))
		{
			while(1)
			{
			if(fno.fattrib & AM_DIR)
			{
				strcat(path,"/");
				strcat(path,fno.fname);
				view_tree(path);
			}
			else
			{
				nokia_lcd_go(0,2);
				nokia_write_text("              ");
				nokia_lcd_go(0,2);
				nokia_write_text("Sure to open?");
				nokia_lcd_go(0,3);
				nokia_write_text("              ");
				nokia_lcd_go(0,3);
				nokia_write_text(fno.fname);

				nokia_lcd_go(0,5);
				nokia_write_text("              ");
				nokia_lcd_go(0,5);
				nokia_write_text(" OK        NO ");
				if(wait_button() == left)
				{
					strcat(path,"/");
					strcat(path,fno.fname);
					return 1;
				}
				else return 0;
			}
			}
		}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
// View tree
//////////////////////////////////////////////////////////////////////////
char view_tree(char *path)
{
	int res;
	unsigned char lenght;

while(1)
{
	pf_opendir(&dir, path);
	while(1)
	{
		res = pf_readdir(&dir, &fno);
		if (res != FR_OK) break;
		if (!fno.fname[0]) break;
		lenght= strlen(fno.fname);
		if(fno.fname[lenght-4] != '.')
		{
			if (fno.fattrib & AM_DIR)
			{
				nokia_lcd_go(0,2);
				nokia_write_text("             ");
				nokia_lcd_go(27,2);
				nokia_write_text("DIR :");
				nokia_lcd_go(0,3);
				nokia_write_text("             ");
				nokia_lcd_go(0,3);
				nokia_write_text(fno.fname);
			}
			else
			{
				nokia_lcd_go(0,2);
				nokia_write_text("             ");
				nokia_lcd_go(24,2);
				nokia_write_text("FILE :");
				nokia_lcd_go(0,3);
				nokia_write_text("             ");
				nokia_lcd_go(0,3);
				nokia_write_text(fno.fname);
			}
			if(wait_button() == right) return 1;
		}	
	}
	if (res != FR_OK) break;
}
	nokia_lcd_go(0,2);
	nokia_write_text("FILE ERROR!!");
	nokia_lcd_go(0,3);
	nokia_write_text(fno.fname);
	wait_button();
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPTS
///////////////////////////////////////////////////////////////////////////////////////////
#pragma vector=PORT1_VECTOR
__interrupt void port_irq (void)
{
	button_press = 1;
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void UART_RX (void)
{
	char i = UCA0RXBUF;
	rx_buffer[rx_index] = i;
	if(i == 10) new_line = 1;
	rx_index++;
	rx_lenght++;
	if(rx_index == rx_buffer_lenght) rx_index = 0;
	if(last_rx_char == 10)
	{
		rx_lenght = 1;
		new_line = 0;
		if(rx_index == 0) rx_pointer = rx_buffer_lenght - 1;
		else rx_pointer = rx_index - 1;
	}
	last_rx_char = i;
} // end interrupt

#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void){
	wdtCounter++;
}
