#include <stdlib.h>
#include <math.h>
#include <msp430g2553.h>
#include "polargraph_server.h"

///////////////////////////////////////////////////////////////////////////
// LOOP function
///////////////////////////////////////////////////////////////////////////
void loop()
{
	// send ready
	// wait for instruction
	unsigned char lenght,i;
	unsigned int idleTime = millis();
	unsigned int timeSince;
	// put last command to all '0'
	for(i=0; i<max_command_lenght; i++){
		lastCommand[i] = 0;
	}

	while (!lastCommandConfirmed)
	{
		char inS[rx_buffer_lenght] = "";
		//idle
		// get incoming command
		while (strlen(inS) == 0)
		{
			timeSince = millis() - idleTime;
			if (timeSince > 1000)
			{
				ready();
				idleTime = millis();
			}
			readCommand(inS);
		}

		// this is confirming the previous command
		if (strncmp(inS,CMD_EXEC,4)==0) // compare string with command CMD_EXEC
		{
			lastCommandConfirmed = true;
			PRINT("Command confirmed: " , 0)
			PRINT(lastCommand , 1)
			idleTime = millis();
		}
		else if (strncmp(inS,"CANCEL",6)==0) // this is cancelling the previous command
		{
			lastCommand[0] = 0;
			lastCommandConfirmed = false;
			ready();
			idleTime = millis();
		}
		else // new command
		{
			lenght = strlen(inS);
			for(i=0; i<lenght; i++){
				lastCommand[i] = inS[i];
			}
			lastCommandConfirmed = false;
			acknowledge(lastCommand);
			idleTime = millis();
		}
	}

	unsigned char commandParsed = parseCommand(lastCommand);
	if (commandParsed)
	{
		PRINT("Executing command." , 1)
		executeCommand(lastCommand);
		lastCommand[0] = 0;
		lastCommandConfirmed = false;
		ready();
	}
	else
	{
		PRINT("Command not parsed." , 1)
		lastCommand[0] = 0;
		lastCommandConfirmed = false;
		ready();
	}
}

///////////////////////////////////////////////////////////////////////////
// MAIN
///////////////////////////////////////////////////////////////////////////
void main()
{
	mmPerStep = mmPerRev / motorStepsPerRev;
	stepsPerMM = motorStepsPerRev / mmPerRev;
	pageWidth = machineWidth * stepsPerMM;
	pageHeight = machineHeight * stepsPerMM;

	// coinfigure hardware
	ConfigWDT();
	ConfigClocks();
	ConfigPorts();
	Config_UART();
	// init interrupt
	_BIS_SR(GIE);

	Serial_print("POLARGRAPH ON!" , 1);

	loadMachineSpecFromEeprom();

	//accelA
	setMaxSpeed(currentMaxSpeed , SX);
	setAcceleration(currentAcceleration , SX);
	//accelB
	setMaxSpeed(currentMaxSpeed , DX);
	setAcceleration(currentAcceleration , DX);

	float startLength = ((float) startLengthMM / (float) mmPerRev) * (float) motorStepsPerRev;
	//accelA
	setCurrentPosition(startLength , SX);
	//accelB
	setCurrentPosition(startLength , DX);

	delay(1000);
	movePenUp();

	establishContact();
	delayMillis(500);

	// call loop
	while(1){
		loop();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPTS
///////////////////////////////////////////////////////////////////////////////////////////
#pragma vector=USCIAB0RX_VECTOR
__interrupt void UART_RX (void)
{
	rx_buffer[rx_index] = UCA0RXBUF;
	rx_index++;
	rx_lenght++;
	if(rx_index == rx_buffer_lenght) rx_index = 0;
} // end interrupt

#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void){
	wdtCounter++;
}
