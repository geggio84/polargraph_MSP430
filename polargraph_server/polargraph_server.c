//#include "AFMotor.h"
//#include "Servo.h"
//#include "EEPROM.h"

#include <stdlib.h>
//#include <stdio.h>
//#include <string.h>
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
	//rove();
	char inS[rx_buffer_lenght] = "";
    //idle
    // get incoming command
    while (strlen(inS) == 0)
    {
	//rove();
      timeSince = millis() - idleTime;
      if (timeSince > 5000)
      {
        ready();
        idleTime = millis();
      }
      //inS = readCommand();
      readCommand(inS);
    }

    //if (inS.equals(CMD_EXEC)) // this is confirming the previous command
    if (strncmp(inS,CMD_EXEC,4)==0) // compare string with command CMD_EXEC
    {
      // this shit is on
      lastCommandConfirmed = true;
      PRINT("Command confirmed: " , 0)
      PRINT(lastCommand , 1)
      idleTime = millis();
    }
    //else if (inS.startsWith("CANCEL")) // this is cancelling the previous command
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
	
	//Serial_begin(57600);           // set up Serial library at 9600 bps
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

	//testServoRange();
	delay(1000);
	
	movePenUp();

  	//readyString = READY;
  	establishContact();
  	delayMillis(500);
  	//outputAvailableMemory();
  
  	// call loop	
  	while(1){
  		//delayMillis(1000);
  		//P1OUT |= LED1;
  		loop();
  		//delayMillis(1000);
  		//P1OUT &= ~LED1;
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
}     // end interrupt

#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void){
  wdtCounter++;
}
