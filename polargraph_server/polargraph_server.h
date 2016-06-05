///////////////////////////////////////////////////////////////////////
// Header file for polargraph controller
///////////////////////////////////////////////////////////////////////
#ifndef POLARGRAPH_SERVER_H_
#define POLARGRAPH_SERVER_H_

#define BAUD_57600
#define MCLK_FREQUENCY		16000000
#define WDT_DIVIDER			8192

#define PI					3.1415

#define WDT_FREQUENCY		(MCLK_FREQUENCY / WDT_DIVIDER) / 1000
volatile unsigned long wdtCounter = 0;

// PINS defines
// PIN Stepper Left
#define sx_stepper_port_dir		P2DIR
#define sx_stepper_port_out		P2OUT
#define sx_stepper_dir			BIT1
#define sx_stepper_step			BIT2
#define sx_stepper_enable		BIT0
// PIN Stepper Right
#define dx_stepper_port_dir		P2DIR
#define dx_stepper_port_out		P2OUT
#define dx_stepper_dir			BIT4
#define dx_stepper_step			BIT3
#define dx_stepper_enable		BIT5
// UART pins
#define RX			BIT1
#define TX			BIT2
// Servo motor
#define servo_pin	BIT0
// BUTTON
#define button		BIT3
// home switches
#define S1_PIN		BIT4
#define S2_PIN		BIT5
// PIN per debug
#define Debug1		BIT5
#define LED1		BIT0

// general defines
#define true 1
#define false 0

// define directions
#define FORWARD		0
#define BACKWARD	1

// define sides
#define SX		0
#define DX		1

// EEPROM addresses
#define EEPROM_MACHINE_WIDTH			0
#define EEPROM_MACHINE_HEIGHT			2
#define EEPROM_MACHINE_NAME				4
#define EEPROM_MACHINE_MM_PER_REV		14
#define EEPROM_MACHINE_STEPS_PER_REV	16
#define EEPROM_LENGHT					20

// switch state
static char switch1;
static char switch2;
long lengthHome;
long lengthHomeSteps;
long leftStepsRetract;
long machineWd;
#define yHome 120 //distance in mm from motor to home location
#define gondolaOffset 75 //distance from centre point of gondola to switching edge in mm
int const rightExtend = 750; //number of steps to extend right motor during home routine to prevent missed steps during left home sequence
// motor position
long laststep1, laststep2, stepsNeeded;

// print define
#define POLARGRAPH_STANDALONE
#define PRINT(text,lf) 
	#ifndef POLARGRAPH_STANDALONE
		Serial_print(text , lf);
	#endif

// Pen raising servo
//Servo penHeight;
unsigned char const SERVO_TIME = 30;
#define PEN_UP 1000
#define PEN_DOWN 1700
unsigned char isPenUp = true;

// Stepper caracteristics
int motorStepsPerRev = 1536;
float mmPerRev = 65;
float mmPerStep;
float stepsPerMM;
// type of stepper movements
//#define	INTERLEAVE		1
//const int stepType = INTERLEAVE;

long current_pos[2] = {0 , 0};
long target_pos[2] = {0 , 0};
float speed[2] = {0 , 0};
float maxSpeed[2] = {1 , 1};
float acceleration[2] = {1 , 1};
unsigned long stepInterval[2] = {0 , 0};
unsigned long lastStepTime[2] = {0 , 0};

// Machine dimensions
int machineWidth = 722;
int machineHeight = 1000;
int pageWidth;
int pageHeight;
int maxLength = 0;
#define DEFAULT_MACHINE_WIDTH 		722
#define DEFAULT_MACHINE_HEIGHT		1000
#define DEFAULT_MM_PER_REV			65
#define DEFAULT_STEPS_PER_REV		1536

// Machine Name
#define machine_name_lenght 5
const char DEFAULT_MACHINE_NAME[machine_name_lenght] =	"PG01";
char machineName[machine_name_lenght] = {0,0,0,0,0};

// Speed and Acceleration
float currentMaxSpeed = 500.0;
float currentAcceleration = 200.0;
#define SUPERFAST_ACCELERATION 6000
// Start lenght
#define startLengthMM				800

//static unsigned char rowAxis[] = "A";
#define INLENGTH 		50
const char INTERMINATOR = 10;

#define DIRECTION_STRING_LTR	"LTR"
#define SRAM_SIZE 				512
#define FREE_MEMORY_STRING		"MEMORY,"
int availMem = 0;

// line width in mm
static float penWidth = 0.4;

// FLAGS
unsigned char reportingPosition = true;

// Command and parameters of the actual command
#define param_lenght 6
#define cmd_lenght 4
static char inCmd[cmd_lenght];
static char inParam1[param_lenght];
static char inParam2[param_lenght];
static char inParam3[param_lenght];
static char inParam4[param_lenght];
// number of parameters of the actual command
int inNoOfParams;

static unsigned char lastWaveWasTop = true;
static unsigned char lastMotorBiasWasA = true;

// RX BUFFER
#define rx_buffer_lenght 40
unsigned char rx_index = 0;
unsigned char rx_pointer = 0;
unsigned char rx_lenght = 0;
char rx_buffer[rx_buffer_lenght];

//  Drawing direction
#define DIR_NE	1
#define DIR_SE	2
#define DIR_SW	3
#define DIR_NW	4
#define DIR_N	5
#define DIR_E	6
#define DIR_S	7
#define DIR_W	8
static unsigned char globalDrawDirection = 4; //DIR_NW;

#define DIR_MODE_AUTO 		1
#define DIR_MODE_PRESET 	2
#define DIR_MODE_RANDOM 	3
static unsigned char globalDrawDirectionMode = 1; //DIR_MODE_AUTO;

#define READY					"READY"
#define RESEND					"RESEND"
#define DRAWING					"DRAWING"
#define OUT_CMD_CARTESIAN		"CARTESIAN,"
#define OUT_CMD_SYNC			"SYNC,"

#define max_command_lenght 40
static char lastCommand[max_command_lenght] = "";
unsigned char lastCommandConfirmed = false;

#define COMMA			","
#define CMD_EXEC		"EXEC"
#define CMD_ACK			"ACK,"

// Define COMMANDS
#define CMD_CHANGELENGTH				"C01"
#define CMD_CHANGEPENWIDTH				"C02"
#define CMD_CHANGEMOTORSPEED			"C03"
#define CMD_CHANGEMOTORACCEL			"C04"
#define CMD_DRAWPIXEL					"C05"
#define CMD_DRAWSCRIBBLEPIXEL			"C06"
#define CMD_DRAWRECT					"C07"
#define CMD_CHANGEDRAWINGDIRECTION		"C08"
#define CMD_SETPOSITION					"C09"
#define CMD_AUTOSETPOSITION				"C99"
#define CMD_TESTPATTERN					"C10"
#define CMD_TESTPENWIDTHSQUARE			"C11"
#define CMD_TESTPENWIDTHSCRIBBLE		"C12"
#define CMD_PENDOWN						"C13"
#define CMD_PENUP						"C14"
//#define CMD_DRAWCIRCLEPIXEL				"C16"
#define CMD_CHANGELENGTHDIRECT			"C17"
#define CMD_SETMACHINESIZE				"C24"
#define CMD_SETMACHINENAME				"C25"
#define CMD_GETMACHINEDETAILS			"C26"
#define CMD_RESETEEPROM					"C27"
#define CMD_DRAWDIRECTIONTEST			"C28"
#define CMD_SETMACHINEMMPERREV			"C29"
#define CMD_SETMACHINESTEPSPERREV		"C30"
#define CMD_SETMOTORSPEED				"C31"
#define CMD_SETMOTORACCEL				"C32"
// NEW COMMANDS not yet implemented
#define CMD_SETPENLIFTRANGE				"C45"
#define CMD_SETMACHINESTEPMULTIPLIER	"C37"
#define CMD_DRAWSAWPIXEL				"C15"
#define CMD_SET_ROVE_AREA				"C21"
#define CMD_MODE_STORE_COMMANDS			"C33"
#define CMD_MODE_EXEC_FROM_STORE		"C34"
#define CMD_MODE_LIVE					"C35"
#define CMD_RANDOM_DRAW					"C36"
#define CMD_START_TEXT					"C38"
#define CMD_DRAW_SPRITE					"C39"
#define CMD_CHANGELENGTH_RELATIVE		"C40"
#define CMD_SWIRLING					"C41"
#define CMD_DRAW_RANDOM_SPRITE			"C42"
#define CMD_DRAW_NORWEGIAN				"C43"
#define CMD_DRAW_NORWEGIAN_OUTLINE		"C44"

#define CMD_END							",END"

///////////////////////////////////////////////////////////////////////
//  Function prototypes
///////////////////////////////////////////////////////////////////////
// Command Functions
void changeLength();						// C01
void changePenWidth();						// C02
void changeMotorSpeed();					// C03
void changeMotorAcceleration();				// C04
void drawSquarePixel_command();				// C05
void drawScribblePixel_from_command();		// C06
void drawRectangle();						// C07
void changeDrawingDirection();				// C08
void setPosition();							// C09
void testPattern();							// C10
void testPenWidth();						// C11
void testPenWidthScribble();				// C12
void penDown();								// C13
void penUp();								// C14
void curves_pixel_drawCircularPixel();		// C16
void changeLengthDirect();					// C17
void setMachineSizeFromCommand();			// C24
void setMachineNameFromCommand();			// C25
void reportMachineSpec();					// C26
void resetEeprom();							// C27
void drawTestDirectionSquare();				// C28
void setMachineMmPerRevFromCommand();		// C29
void setMachineStepsPerRevFromCommand();	// C30
void setMotorSpeed_from_parameter();		// C31
void setMotorAcceleration_from_parameter();	// C32

void homeRoutine();
void readSwitches();

void delay_us(unsigned int us);
void ConfigWDT(void);
void FaultRoutine(void);
void ConfigClocks(void);
void ConfigPorts(void);
void Config_UART();
void strreverse(char* begin, char* end);
void itoa(int value, char* str, int base);
void loadMachineSpecFromEeprom();

// EEPROM Functions
void EEPROMWriteInt(int p_address, int p_value);
unsigned int EEPROMReadInt(int p_address);
void EEPROM_write(int p_address, unsigned char lowByte);
unsigned char EEPROM_read(int p_address);
void dumpEeprom();

// Serial TX-RX
char Serial_read();
unsigned char Serial_available();
void Serial_print(char string[], unsigned char line_feed);
// Servo Motor Functions
void movePenUp();
void movePenDown();
void testPenHeight();

int strncmp(const char *s1, const char *s2, size_t n);

void establishContact();
void ready();
void drawing();
void acknowledge(char command[]);
void setMaxSpeed(float MaxSpeed , unsigned char side);
void setAcceleration(float Acceleration , unsigned char side);
void setCurrentPosition(long Position , unsigned char side);
void delay(unsigned int ms);
char* readCommand(char* inString);
unsigned long millis();
void delayMillis(unsigned long milliseconds);
unsigned char parseCommand(char* inS);
void extractParams(char* inS);
void executeCommand(char inS[]);
long asLong(char* inParam);
int asInt(char* inParam);
char asByte(char* inParam);
float asFloat(char* inParam);
int availableMemory();
void outputAvailableMemory();
void changeLength_float(float tA, float tB);
void changeLength_long(long tA, long tB);
void reportPosition();
long currentPosition(unsigned char side);
long getMaxLength();
float getMachineA(float cX, float cY);
float getMachineB(float cX, float cY);
void drawBetweenPoints(float p1a, float p1b, float p2a, float p2b, int maxSegmentLength);
float getCartesianXFP(float aPos, float bPos);
float getCartesianYFP(float cX, float aPos);
void useAcceleration(unsigned char use);
void setMotorSpeed(float speed);
void setMotorAcceleration(float accel);
void drawSquarePixel(int length, int width, int density, unsigned char drawDirection);
void drawScribblePixel(long originA, long originB, int size, int density);
unsigned char getAutoDrawDirection(long targetA, long targetB, long sourceA, long sourceB);
unsigned char getRandomDrawDirection();
int scaleDensity(int inDens, int inMax, int outMax);
int maxDensity(float penSize, int rowSize);
void drawSquareWaveAlongA(int waveAmplitude, int waveLength, int totalWaves, int waveNo);
void drawSquareWaveAlongB(int waveAmplitude, int waveLength, int totalWaves, int waveNo);
void flipWaveDirection();
void moveA(long dist);
void moveB(long dist);
void moveTo(long destination , unsigned char side);
void move(long relative , unsigned char side);
unsigned char run(unsigned char side);
unsigned char runSpeed(unsigned char side);
void step(unsigned int step, unsigned char side);
void computeNewSpeed(unsigned char side);
float desiredSpeed(unsigned char side);
float desiredSpeed_new(long distanceTo, float currentSpeed, float acceleration);
void setSpeed(float speed_target , unsigned char side);
long distanceToGo_function(unsigned char side);
int random(int min, int max);
void runToPosition(unsigned char side);
void engageMotors();
void runToNewPosition(long position , unsigned char side);
void releaseMotors();
void curves_drawSpiral(long centerx, long centery, int maxRadius, int increment, int density);

////////////////////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////
/*float sqrt(float m)
{
	int j;	
	float i=0;
	float x1,x2;
	while( (i*i) <= m )
		i+=0.1;
	x1=i;
	for(j=0;j<10;j++)
	{
		x2=m;
		x2/=x1;
		x2+=x1;
		x2/=2;
		x1=x2;
	}
	return x2;
}*/

int strncmp(const char *s1, const char *s2, size_t n)
{
	unsigned char uc1, uc2;
	/* Nothing to compare?  Return zero.  */
	if (n == 0)
		return 0;
	/* Loop, comparing bytes.  */
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

/************************************************************************
 *
 *	Find the length of a string - does not check if we fall off
 *	the end of the text buffer. oops.
 *
 ************************************************************************/
int strlen(const char *text)
{
	int count=-1;				/* Character counter */

	while(text[++count] != '\0') ;		/* Search for a null */

	return(count);				/* Return the position * of the NULL-1 */
}

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
// reverse string
//////////////////////////////////////////////////////////////////////////
char temp_char[10];
void strreverse(char* begin, char* end)
{
	char aux;

	while(end>begin)
		aux=*end, *end--=*begin, *begin++=aux;
}
//////////////////////////////////////////////////////////////////////////
// Integer to string
//////////////////////////////////////////////////////////////////////////
void itoa(int value, char* str, int base)
{
	static char num[] = "0123456789abcdefghijklmnopqrstuvwxyz";
	char* wstr=str;
	int sign;

	// Validate base
	if (base<2 || base>35) { *wstr='\0'; return; }

	// Take care of sign
	if ((sign=value) < 0) value = -value;

	// Conversion. Number is reversed.
	do *wstr++ = num[value%base]; while(value/=base);
	if(sign<0) *wstr++='-';
	*wstr='\0';
	// Reverse string
	strreverse(str,wstr-1);	
}
//////////////////////////////////////////////////////////////////////////
// ConfigWDT
//////////////////////////////////////////////////////////////////////////
void ConfigWDT(void)
{
	//WDTCTL = WDTPW + WDTHOLD;				// Stop watchdog timer
	WDTCTL = WDTPW + WDTTMSEL + WDTIS0;		// Watchdog timer = SMCLK / 8192
	IE1 |= WDTIE;
}
//////////////////////////////////////////////////////////////////////////
// FaultRoutine
//////////////////////////////////////////////////////////////////////////
void FaultRoutine(void)
{
	//P1OUT = BIT0;		// P1.0 on (red LED)
	while(1); 			// TRAP
}
//////////////////////////////////////////////////////////////////////////
// ConfigClocks
//////////////////////////////////////////////////////////////////////////
void ConfigClocks(void)
{
	if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)
		FaultRoutine();							// If calibration data is erased run FaultRoutine()
	BCSCTL1 = CALBC1_16MHZ; 					// Set range
	DCOCTL = CALDCO_16MHZ;  					// Set DCO step + modulation
	BCSCTL3 |= LFXT1S_2;						// LFXT1 = VLO
	IFG1 &= ~OFIFG;								// Clear OSCFault flag
	BCSCTL2 |= SELM_0 + DIVM_0 + DIVS_0;		// MCLK = DCO/1, SMCLK = DCO/1
}
//////////////////////////////////////////////////////////////////////////
// ConfigPorts
//////////////////////////////////////////////////////////////////////////
void ConfigPorts(void)
{
	sx_stepper_port_dir = sx_stepper_dir + sx_stepper_step + sx_stepper_enable;
	P1DIR = servo_pin;
	P1REN = S1_PIN + S2_PIN;
	P1OUT = S1_PIN + S2_PIN;
	dx_stepper_port_dir |= dx_stepper_dir + dx_stepper_step + dx_stepper_enable;
	sx_stepper_port_out = sx_stepper_enable;
	dx_stepper_port_out |= dx_stepper_enable;
}
//////////////////////////////////////////////////////////////////////////
// ConfigUART
//////////////////////////////////////////////////////////////////////////
void Config_UART()
{
	UCA0CTL1 = UCSWRST;		// UCSI software reset (OFF STATE)
	UCA0CTL0 = 0; 
	UCA0CTL1 |= UCSSEL_3;	// SMCLK is clock for UCSI

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
	
	P1SEL = TX + RX;		// config pins
	P1SEL2 = TX + RX;
	P1DIR |= TX;
	UCA0CTL1 &= ~UCSWRST;	// UCSI software reset (ON STATE)
	//5. Enable interrupts (optional) via UCAxRXIE and/or UCAxTXIE
	IE2 |= UCA0RXIE;		// enable irq in rx
}
//////////////////////////////////////////////////////////////////////////
// Resume Machine Specifications from EEPROM
//////////////////////////////////////////////////////////////////////////
void loadMachineSpecFromEeprom()
{
	unsigned char i;
	char name[4];

	machineWidth = EEPROMReadInt(EEPROM_MACHINE_WIDTH);
	if (machineWidth < 1)
	{
		machineWidth = DEFAULT_MACHINE_WIDTH;
	}
	Serial_print("Loaded machine width:" , 0);
	itoa(machineWidth,temp_char,10);
	Serial_print(temp_char , 1);

	machineHeight = EEPROMReadInt(EEPROM_MACHINE_HEIGHT);
	if (machineHeight < 1)
	{
		machineHeight = DEFAULT_MACHINE_HEIGHT;
	}
	Serial_print("Loaded machine height:" , 0);
	itoa(machineHeight,temp_char,10);
	Serial_print(temp_char , 1);

	mmPerRev = EEPROMReadInt(EEPROM_MACHINE_MM_PER_REV);
	if ((mmPerRev < 1)|(mmPerRev > 100))
	{
		mmPerRev = DEFAULT_MM_PER_REV;
	}
	Serial_print("Loaded mm per rev:" , 0);
	itoa(mmPerRev,temp_char,10);
	Serial_print(temp_char , 1);

	motorStepsPerRev = EEPROMReadInt(EEPROM_MACHINE_STEPS_PER_REV);
	if (motorStepsPerRev < 1)
	{
		motorStepsPerRev = DEFAULT_STEPS_PER_REV;
	}
	Serial_print("Loaded motor steps per rev:" , 0);
	itoa(motorStepsPerRev,temp_char,10);
	Serial_print(temp_char , 1);

	mmPerStep = mmPerRev / motorStepsPerRev;
	stepsPerMM = motorStepsPerRev / mmPerRev;

	Serial_print("Recalculated mmPerStep (" , 0);
	ltoa(mmPerStep,temp_char);
	Serial_print(temp_char , 0);
	Serial_print(") and stepsPerMM (" , 0);
	ltoa(stepsPerMM,temp_char);
	Serial_print(temp_char , 0);
	Serial_print(")" , 1);

	pageWidth = machineWidth * stepsPerMM;
	Serial_print("Recalculated pageWidth in steps (" , 0);
	itoa(pageWidth,temp_char,10);
	Serial_print(temp_char , 0);
	Serial_print(")" , 1);
	pageHeight = machineHeight * stepsPerMM;
	Serial_print("Recalculated pageHeight in steps (" , 0);
	itoa(pageHeight,temp_char,10);
	Serial_print(temp_char , 0);
	Serial_print(")" , 1);

	for (i = 0; i < 4; i++)
	{
		char b = EEPROM_read(EEPROM_MACHINE_NAME+i);
		name[i] = b;
	}
	if (name[0] == 0)
	for (i = 0; i < 4; i++)
	{
		name[i] = DEFAULT_MACHINE_NAME[i];
	}
	maxLength = 0;
	for (i = 0; i < 4; i++)
	{
		machineName[i] = name[i];
	}
	Serial_print("Loaded machine name:" , 0);
	Serial_print(machineName , 1);
}
//////////////////////////////////////////////////////////////////////////
//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
//////////////////////////////////////////////////////////////////////////
void EEPROMWriteInt(int p_address, int p_value)
{
	Serial_print("Writing Int " , 0);
	itoa(p_value,temp_char,10);
	Serial_print(temp_char , 0);
	Serial_print(" to address " , 0);
	itoa(p_address,temp_char,10);
	Serial_print(temp_char , 1);

	unsigned char lowByte = ((p_value >> 0) & 0xFF);
	unsigned char highByte = ((p_value >> 8) & 0xFF);
	EEPROM_write(p_address, lowByte);
	EEPROM_write(p_address + 1, highByte);
}
//////////////////////////////////////////////////////////////////////////
//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
//////////////////////////////////////////////////////////////////////////
unsigned int EEPROMReadInt(int p_address)
{
	unsigned char lowByte = EEPROM_read(p_address);
	unsigned char highByte = EEPROM_read(p_address + 1);
	return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}
//////////////////////////////////////////////////////////////////////////
//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
//////////////////////////////////////////////////////////////////////////
unsigned char EEPROM_read(int p_address)
{
	unsigned char *Flash_ptr;				// Segment D pointer
	char data;
	int addr = 0x1000 + p_address;
	Flash_ptr = (unsigned char *)addr;		// Point to beginning of seg D
	data = *Flash_ptr;
	return data;
}
//////////////////////////////////////////////////////////////////////////
//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
//////////////////////////////////////////////////////////////////////////
void EEPROM_write(int p_address, unsigned char lowByte)
{
	int j;
	unsigned char *Flash_ptr;				// Segment D pointer
	char data[EEPROM_LENGHT];

	for (j = 0; j < EEPROM_LENGHT; j++)
	{
		data[j] = EEPROM_read(j);
	}

	Flash_ptr = (unsigned char *)0x1000;					// Point to beginning of seg D
	FCTL2 = FWKEY + FSSEL0 + FN5 + FN3 + FN2 + FN1 + FN0;	// MCLK = 16M  /48 = 333kHz for Flash Timing Generator
	FCTL1 = FWKEY + ERASE;									// Set Erase bit
	FCTL3 = FWKEY;											// Clear LOCK & LOCKA bits
	*Flash_ptr = 0x00;										// Dummy write to erase Flash seg D
	FCTL1 = FWKEY + WRT;									// Set WRT bit for write operation
	for (j = 0; j < EEPROM_LENGHT; j++)
	{
		if(j == p_address)
			*Flash_ptr++ = lowByte;			// flash write
		else
			*Flash_ptr++ = data[j];			// flash write
	}
	FCTL1 = FWKEY;							// Clear WRT bit
	FCTL3 = FWKEY + LOCKA + LOCK;			// Set LOCK & LOCKA bit
}
//////////////////////////////////////////////////////////////////
// Delay ms milliseconds										//
//////////////////////////////////////////////////////////////////
void delay(unsigned int ms)
{
	unsigned int count = ms;
	while (count != 0)
	{
		__delay_cycles(16000); // set for 16Mhz change it to 1000 for 1 Mhz
		count--;
	}
}
//////////////////////////////////////////////////////////////////
// Delay us microseconds										//
//////////////////////////////////////////////////////////////////
void delay_us(unsigned int us)
{
	unsigned int count = us;
	while (count != 0)
	{
		__delay_cycles(16); // set for 16Mhz change it to 1000 for 1 Mhz
		count--;
	}
}
//////////////////////////////////////////////////////////////////
// Move pen Up													//
//////////////////////////////////////////////////////////////////
#ifdef CMD_PENUP
void movePenUp()
{
	int i;
	for (i=0; i<SERVO_TIME; i++) {
		P1OUT |= servo_pin;
		delay_us(PEN_UP);
		P1OUT &= ~servo_pin;
		delayMillis(17);
	}

	isPenUp = true;
}
//////////////////////////////////////////////////////////////////
// If Down move Up												//
//////////////////////////////////////////////////////////////////
void penUp()
{
	if (isPenUp == false)
	{
		movePenUp();
	}
}
#endif /* CMD_PENUP */
//////////////////////////////////////////////////////////////////
// Move Pen down												//
//////////////////////////////////////////////////////////////////
#ifdef CMD_PENDOWN
void movePenDown()
{
	int i;
	int pulse_time = PEN_UP;

	for (i=0; i<SERVO_TIME; i++) {
		P1OUT |= servo_pin;
		delay_us(pulse_time);
		P1OUT &= ~servo_pin;
		pulse_time = pulse_time + ((PEN_DOWN-PEN_UP)/SERVO_TIME);
		delayMillis(17);
	}

	isPenUp = false;
}
//////////////////////////////////////////////////////////////////
// If Up move down												//
//////////////////////////////////////////////////////////////////
void penDown()
{
	if (isPenUp == true)
	{
		movePenDown();
	}
}
#endif /* CMD_PENDOWN */
//////////////////////////////////////////////////////////////////
// Test Pen Height												//
//////////////////////////////////////////////////////////////////
void testPenHeight()
{
	delayMillis(3000);
	penUp();
	delayMillis(3000);
	penDown();
	delayMillis(3000);
}

void readSwitches() {
	// get the current switch state
	switch1= P1IN & S1_PIN;
	switch2= P1IN & S2_PIN;
}

void homeRoutine(){
	int i;
	
	penUp();
	// find the current robot position and 
	// reel in the left motor until contact is made.
	//let out gondola to avoid being at home position

	//at this point gondola is away from both home location
	//note holding current draw ~600mA

	sx_stepper_port_out &= ~sx_stepper_enable;
	dx_stepper_port_out &= ~dx_stepper_enable;

	setSpeed(-500,SX);
	setSpeed(500,DX);
	dx_stepper_port_out |= dx_stepper_enable;

	do {
		//move left motor back and right motor forward to find left hime location
		while(runSpeed(SX)==false);

		readSwitches();
	} while (switch1 != 0);

	setSpeed(500,SX);
	setSpeed(-500,DX); 
	dx_stepper_port_out &= ~dx_stepper_enable; 
	sx_stepper_port_out |= sx_stepper_enable;
	//now we know that left motor is at home location
	//count # steps for right home
	do {
		//move right  motor back and left motor released to find right home location
		while(runSpeed(DX)==false);

		readSwitches();
	} while (switch2 != 0);

	sx_stepper_port_out &= ~sx_stepper_enable;
	//We have now homed left and right, but # steps to home right contains steps to pull in slack in cord
	//So, know we now we have no slack, do home routine again to find true # steps R->L
	//due to small spools size and build up of cord, there are slightly different take up/let down rates for each spool
	//that can cause missed steps, to remove this right mor will release slightly and steps will be accounted for in calc (rightExtend)
	setSpeed(-500,SX);
	setSpeed(500,DX);

	for(i=0; i<rightExtend; i++){
		while(runSpeed(DX)==false);
	}

	setSpeed(500,DX);
	laststep1=0;	//set step counter to zero
	do {
		//move left motor back and right motor forward to find left hime location
		while(runSpeed(SX)==false);
		while(runSpeed(DX)==false);

		laststep1++;
		readSwitches();
	} while (switch1 != 0);

	//at this point right motor at home and we know how many steps from L->R
	//using pythagoras, machineWidth/2 contains home x location
	//set number of steps needed to get to home location
	//method using pythagorus calculate # of steps needed from home position from left and right
	//laststep1 contains #of steps traveresed by left so move left back to get to above
	//forward right motor by calc # of steps to get to home
	//laststep2 = (laststep1-(stepsPerMM/sensorGap))/2;
	machineWd=machineWidth/2;	//convert to long, scale for half way

	lengthHome = (sqrt((machineWd*machineWd)-(yHome*yHome))) - gondolaOffset; //in mm and allow for gondola offset

	//convert to steps
	lengthHomeSteps=lengthHome*stepsPerMM; //not sure why but need to correct steps/mm by factor of 2
	//calc # of steps for left motor to retract
	leftStepsRetract = laststep1 - lengthHomeSteps + rightExtend;

	setSpeed(500,SX);	//motors can use normal speed
	setSpeed(-500,DX);

	do {
		while(runSpeed(SX)==false);
		lengthHomeSteps--;
	} while (lengthHomeSteps > 0);		//allow for extension in above

	do {
		while(runSpeed(DX)==false);
		leftStepsRetract--;
	} while (leftStepsRetract > 0);		//allow for extension in above

	setSpeed(500,DX);
	//Now at home location
}

//////////////////////////////////////////////////////////////////
// Read incoming command										//
//////////////////////////////////////////////////////////////////
char* readCommand(char* inString)
{
	// check if data has been sent from the computer:
	int inCount = 0;
	while (Serial_available() > 0)
	{
	char ch = Serial_read();	// get it
	delayMillis(3);
	inString[inCount] = ch;
	if (ch == INTERMINATOR)
	{
	break;
	}
	inCount++;
	}
	inString[inCount] = 0;		// null terminate the string
	char* inS = inString;
	return inS;
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
//////////////////////////////////////////////////////////////////
// Send READY string to host									//
//////////////////////////////////////////////////////////////////
void ready()
{
	Serial_print(READY , 1);
}
//////////////////////////////////////////////////////////////////
// establish connection with host								//
//////////////////////////////////////////////////////////////////
void establishContact() 
{
	ready();
}
//////////////////////////////////////////////////////////////////
// Send DRAWING string to host									//
//////////////////////////////////////////////////////////////////
void drawing()
{
	Serial_print(DRAWING , 1);
}
//////////////////////////////////////////////////////////////////
// Send ACK to received command									//
//////////////////////////////////////////////////////////////////
void acknowledge(char command[])
{
#ifdef POLARGRAPH_STANDALONE
		Serial_print(CMD_ACK , 1);
#else
		Serial_print(CMD_ACK , 0);
		Serial_print(command , 1);
#endif
}
//////////////////////////////////////////////////////////////////
// Extract substring from string from start to end				//
//////////////////////////////////////////////////////////////////
void substring(char* string, char* string_out, unsigned char start, unsigned char end){
	unsigned char i;

	for(i=0; i<end-start; i++){
		string_out[i] = string[i+start];
	}
}
//////////////////////////////////////////////////////////////////
// Parse Command												//
//////////////////////////////////////////////////////////////////
unsigned char parseCommand(char* inS)
{
	// check if command ends with ",END"
	char endChars[4];
	char lenght;
	unsigned char i;

	lenght = strlen(inS);
	for(i=0; i<4; i++){
		endChars[i] = inS[lenght-4+i];
	}
	if (strncmp(endChars,CMD_END,4)==0)
	{
		extractParams(inS);
		return true;
	}
	else
		return false;
}

//void requestResend()
//{
//  Serial_print(RESEND , 1);
//}
//char* extractCommandFromExecute(char inS[])
//{
//  char result[] = inS.substring(8);
//  return result;
//}

//////////////////////////////////////////////////////////////////
// Execute Command												//
//////////////////////////////////////////////////////////////////
void executeCommand(char inS[])
{
	if (strncmp(inS,CMD_CHANGELENGTH,3)==0)
	{
	changeLength();
	}
#ifdef CMD_CHANGELENGTHDIRECT
	else if (strncmp(inS,CMD_CHANGELENGTHDIRECT,3)==0)
		changeLengthDirect();
#endif /* CMD_CHANGELENGTHDIRECT */
#ifdef CMD_DRAWPIXEL
	else if (strncmp(inS,CMD_DRAWPIXEL,3)==0)
		drawSquarePixel_command();
#endif /* CMD_DRAWPIXEL */
#ifdef CMD_DRAWCIRCLEPIXEL
	else if (strncmp(inS,CMD_DRAWCIRCLEPIXEL,3)==0)
    	curves_pixel_drawCircularPixel();
#endif /* CMD_DRAWCIRCLEPIXEL */
#ifdef CMD_DRAWSCRIBBLEPIXEL
	else if (strncmp(inS,CMD_DRAWSCRIBBLEPIXEL,3)==0)
		drawScribblePixel_from_command();
#endif /* CMD_DRAWSCRIBBLEPIXEL */
#ifdef CMD_DRAWRECT
	else if (strncmp(inS,CMD_DRAWRECT,3)==0)
		drawRectangle();
#endif /* CMD_DRAWRECT */
#ifdef CMD_CHANGEDRAWINGDIRECTION
	else if (strncmp(inS,CMD_CHANGEDRAWINGDIRECTION,3)==0)
		changeDrawingDirection();
#endif /* CMD_CHANGEDRAWINGDIRECTION */
#ifdef CMD_PENDOWN
	else if (strncmp(inS,CMD_PENDOWN,3)==0)
		penDown();
#endif /* CMD_PENDOWN */
#ifdef CMD_PENUP
	else if (strncmp(inS,CMD_PENUP,3)==0)
		penUp();
#endif /* CMD_PENUP */
	else if (strncmp(inS,CMD_SETPOSITION,3)==0)
	{
	setPosition();
	}
	else if (strncmp(inS,CMD_AUTOSETPOSITION,3)==0)
	{
	homeRoutine();
	setPosition();
	}
	else if (strncmp(inS,CMD_CHANGEPENWIDTH,3)==0)
	{
	changePenWidth();
	}
	else if (strncmp(inS,CMD_CHANGEMOTORSPEED,3)==0)
	{
	changeMotorSpeed();
	}
	else if (strncmp(inS,CMD_CHANGEMOTORACCEL,3)==0)
	{
	changeMotorAcceleration();
	}
	else if (strncmp(inS,CMD_SETMOTORSPEED,3)==0)
	{
	setMotorSpeed_from_parameter();
	}
	else if (strncmp(inS,CMD_SETMOTORACCEL,3)==0)
	{
	setMotorAcceleration_from_parameter();
	}
	else if (strncmp(inS,CMD_TESTPATTERN,3)==0)
	{
	testPattern();
	}
	else if (strncmp(inS,CMD_TESTPENWIDTHSQUARE,3)==0)
	{
	testPenWidth();
	}
	else if (strncmp(inS,CMD_TESTPENWIDTHSCRIBBLE,3)==0)
	{
	testPenWidthScribble();
	}
	else if (strncmp(inS,CMD_SETMACHINESIZE,3)==0)
	{
	setMachineSizeFromCommand();
	}
	else if (strncmp(inS,CMD_SETMACHINENAME,3)==0)
	{
	setMachineNameFromCommand();
	}
	else if (strncmp(inS,CMD_SETMACHINEMMPERREV,3)==0)
	{
	setMachineMmPerRevFromCommand();
	}
	else if (strncmp(inS,CMD_SETMACHINESTEPSPERREV,3)==0)
	{
	setMachineStepsPerRevFromCommand();
	}
	else if (strncmp(inS,CMD_GETMACHINEDETAILS,3)==0)
	{
	reportMachineSpec();
	}
	else if (strncmp(inS,CMD_RESETEEPROM,3)==0)
	{
	resetEeprom();
	}
	else if (strncmp(inS,CMD_DRAWDIRECTIONTEST,3)==0)
	{
	drawTestDirectionSquare();
	}
	else
	{
	Serial_print("Sorry, " , 0);
	Serial_print(inS , 0);
	Serial_print(" isn't a command I recognise." , 1);
	ready();
	}
	PRINT("After execute:" , 1)
}
//////////////////////////////////////////////////////////////////
// Convert input parameters										//
//////////////////////////////////////////////////////////////////
long asLong(char* inParam)
{
	return atol(inParam);
}
int asInt(char* inParam)
{
	return atoi(inParam);
}
char asByte(char* inParam)
{
	int i = asInt(inParam);
	return (char) i;
}
float asFloat(char* inParam)
{
	return atof(inParam);
}

///****************************************************************************************************************/
///****************************************************************************************************************/
///****************************************************************************************************************/
///****************************************************************************************************************/
///****************************            BELOW IS THE CODE THAT DOES THE WORK      ******************************/
///****************************************************************************************************************/
///****************************************************************************************************************/
///****************************************************************************************************************/
///****************************************************************************************************************/

//////////////////////////////////////////////////////////////////
// Reset EEPROM													//
//////////////////////////////////////////////////////////////////
void resetEeprom()
{
	int i;
	for (i = 0; i <20; i++)
	{
		EEPROM_write(i, 0);
	}
	loadMachineSpecFromEeprom();
}
//////////////////////////////////////////////////////////////////
// Dump EEPROM													//
//////////////////////////////////////////////////////////////////
void dumpEeprom()
{
	int i;
	char k[2] = {0 ,0};
	for (i = 0; i <20; i++)
	{
		itoa(i,temp_char,10);
		Serial_print(temp_char , 0);
		Serial_print(". " , 0);
		k[0] = EEPROM_read(i);
		Serial_print(k , 1);
	}
}
//////////////////////////////////////////////////////////////////
// Print machine Specs											//
//////////////////////////////////////////////////////////////////
void reportMachineSpec()
{
	dumpEeprom();
	Serial_print("PGNAME," , 0);
	Serial_print(machineName , 0);
	Serial_print(CMD_END , 1);

	Serial_print("PGSIZE," , 0);
	itoa(machineWidth, temp_char, 10);
	Serial_print(temp_char , 0);
	Serial_print(COMMA , 0);
	itoa(machineHeight, temp_char, 10);
	Serial_print(temp_char , 0);
	Serial_print(CMD_END , 1);

	Serial_print("PGMMPERREV," , 0);
	ltoa(mmPerRev, temp_char);
	Serial_print(temp_char , 0);
	Serial_print(CMD_END , 1);

	Serial_print("PGSTEPSPERREV," , 0);
	itoa(motorStepsPerRev, temp_char, 10);
	Serial_print(temp_char , 0);
	Serial_print(CMD_END , 1);
}
//////////////////////////////////////////////////////////////////
// Set machine size from command								//
//////////////////////////////////////////////////////////////////
void setMachineSizeFromCommand()
{
	int width = asInt(inParam1);
	int height = asInt(inParam2);

	if (width > 10)
	{
	EEPROMWriteInt(EEPROM_MACHINE_WIDTH, width);
	}
	if (height > 10)
	{
	EEPROMWriteInt(EEPROM_MACHINE_HEIGHT, height);
	}

	loadMachineSpecFromEeprom();
}
//////////////////////////////////////////////////////////////////
// Set machine name from command								//
//////////////////////////////////////////////////////////////////
void setMachineNameFromCommand()
{
	char* name = inParam1;
	int i;
	if (name != DEFAULT_MACHINE_NAME)
	{
		for (i = 0; i < 4; i++)
		{
			EEPROM_write(EEPROM_MACHINE_NAME+i, name[i]);
		}
	}
	loadMachineSpecFromEeprom();
}
//////////////////////////////////////////////////////////////////
// Set machine mm per rev from command							//
//////////////////////////////////////////////////////////////////
void setMachineMmPerRevFromCommand()
{
	float mmPerRev = asFloat(inParam1);
	EEPROMWriteInt(EEPROM_MACHINE_MM_PER_REV, mmPerRev);
	loadMachineSpecFromEeprom();
}
//////////////////////////////////////////////////////////////////
// Set steps per rev from command								//
//////////////////////////////////////////////////////////////////
void setMachineStepsPerRevFromCommand()
{
	int stepsPerRev = asInt(inParam1);
	EEPROMWriteInt(EEPROM_MACHINE_STEPS_PER_REV, stepsPerRev);
	loadMachineSpecFromEeprom();
}
//////////////////////////////////////////////////////////////////
// Set max speed from parameters								//
//////////////////////////////////////////////////////////////////
void setMotorSpeed_from_parameter()
{
	setMotorSpeed(asFloat(inParam1));
}
//////////////////////////////////////////////////////////////////
// Set max speed												//
//////////////////////////////////////////////////////////////////
void setMaxSpeed(float MaxSpeed, unsigned char side)
{
	maxSpeed[side] = MaxSpeed;
	computeNewSpeed(side);
}
//////////////////////////////////////////////////////////////////
// Change Stepper motor speed									//
//////////////////////////////////////////////////////////////////
void setMotorSpeed(float speed)
{
	currentMaxSpeed = speed;
	setMaxSpeed(currentMaxSpeed , SX);
	setMaxSpeed(currentMaxSpeed , DX);
	ltoa(currentMaxSpeed, temp_char);
	Serial_print("New max speed: " , 0);
	Serial_print(temp_char , 1);
}
//////////////////////////////////////////////////////////////////
// Change Stepper motor speed									//
//////////////////////////////////////////////////////////////////
void changeMotorSpeed()
{
	float speedChange = asFloat(inParam1);
	float newSpeed = currentMaxSpeed + speedChange;
	setMotorSpeed(newSpeed);
}
//////////////////////////////////////////////////////////////////
// Set Stepper motor acceleration from parameter				//
//////////////////////////////////////////////////////////////////
void setMotorAcceleration_from_parameter()
{
	setMotorAcceleration(asFloat(inParam1));
}
//////////////////////////////////////////////////////////////////
// Change Stepper motor acceleration							//
//////////////////////////////////////////////////////////////////
void setMotorAcceleration(float accel)
{
	currentAcceleration = accel;
	setAcceleration(currentAcceleration , SX);
	setAcceleration(currentAcceleration , DX);
	Serial_print("New acceleration: " , 0);
	ltoa(currentAcceleration, temp_char);
	Serial_print(temp_char , 1);
}
//////////////////////////////////////////////////////////////////
// Change Stepper motor acceleration							//
//////////////////////////////////////////////////////////////////
void changeMotorAcceleration()
{
	float speedChange = asFloat(inParam1);
	float newAccel = currentAcceleration + speedChange;
	setMotorAcceleration(newAccel);
}
//////////////////////////////////////////////////////////////////
// Change pen width												//
//////////////////////////////////////////////////////////////////
void changePenWidth()
{
	penWidth = asFloat(inParam1);
	Serial_print("Changed Pen width to " , 0);
	ltoa(penWidth,temp_char);
	Serial_print(temp_char , 0);
	Serial_print("mm" , 1);
}
//////////////////////////////////////////////////////////////////
// Change Drawing Direction										//
//////////////////////////////////////////////////////////////////
#ifdef CMD_CHANGEDRAWINGDIRECTION
void changeDrawingDirection() 
{
	globalDrawDirectionMode = asInt(inParam1);
	globalDrawDirection = asInt(inParam2);
	PRINT("Changed draw direction mode to be " , 0)
#ifndef POLARGRAPH_STANDALONE
	itoa(globalDrawDirectionMode, temp_char, 10);
#endif
	PRINT(temp_char , 0)
	PRINT(" and direction is " , 0)
#ifndef POLARGRAPH_STANDALONE
	itoa(globalDrawDirection, temp_char, 10);
#endif
	PRINT(temp_char , 1)
}
#endif /* CMD_CHANGEDRAWINGDIRECTION */
//////////////////////////////////////////////////////////////////
// Extract parameters from received Command						//
//////////////////////////////////////////////////////////////////
void extractParams(char* inS) {

	// get number of parameters
	// by counting commas
	unsigned char length = strlen(inS);
	unsigned char startPos = 0;
	unsigned char paramNumber = 0;
	unsigned char i;

	for(i=0; i<cmd_lenght; i++){
		inCmd[i] = 0;
	}
	for(i=0; i<param_lenght; i++){
		inParam1[i] = 0;
		inParam2[i] = 0;
		inParam3[i] = 0;
		inParam4[i] = 0;
	}

	for (i = 0; i < length; i++) {
		if (inS[i] == ',') {
					
			switch(paramNumber) {
				case 0:
					substring(inS, inCmd, startPos, i);
					break;
				case 1:
					substring(inS, inParam1, startPos, i);
					break;
				case 2:
					substring(inS, inParam2, startPos, i);
					break;
				case 3:
					substring(inS, inParam3, startPos, i);
					break;
				case 4:
					substring(inS, inParam4, startPos, i);
					break;
				default:
					break;
			}
			paramNumber++;
			startPos = i+1;
		}
	}
	inNoOfParams = paramNumber;
}
//////////////////////////////////////////////////////////////////
// Test Pattern													//
//////////////////////////////////////////////////////////////////
void testPattern()
{
	int rowWidth = asInt(inParam1);
	int noOfIncrements = asInt(inParam2);
	int w,i;
	unsigned char ltr = true;

	for (w = rowWidth; w < (w+(noOfIncrements*5)); w+=5)
	{
		for (i = 0;  i <= maxDensity(penWidth, w); i++)
		{
			drawSquarePixel(w, w, i, ltr);
		}
		if (ltr)
			ltr = false;
		else
			ltr = true;

		moveB(w);
	}
}
//////////////////////////////////////////////////////////////////
// Test Pen Width												//
//////////////////////////////////////////////////////////////////
void testPenWidth()
{
	int rowWidth = asInt(inParam1);
	float startWidth = asFloat(inParam2);
	float endWidth = asFloat(inParam3); 
	float incSize = asFloat(inParam4);
	float pw;
	int i;

	int tempDirectionMode = globalDrawDirectionMode;
	globalDrawDirectionMode = DIR_MODE_PRESET;

	float oldPenWidth = penWidth;
	int iterations = 0;

	for (pw = startWidth; pw <= endWidth; pw+=incSize)
	{
		iterations++;
		penWidth = pw;
		int maxDens = maxDensity(penWidth, rowWidth);
		drawSquarePixel(rowWidth, rowWidth, maxDens, DIR_SE);
	}

	penWidth = oldPenWidth;

	moveB(0-rowWidth);
	for (i = 1; i <= iterations; i++)
	{
		moveB(0-(rowWidth/2));
		moveA(0-rowWidth);
		moveB(rowWidth/2);
	}

	penWidth = oldPenWidth;
	globalDrawDirectionMode = tempDirectionMode;
}
//////////////////////////////////////////////////////////////////
// Test Pen Width Scribble										//
//////////////////////////////////////////////////////////////////
void testPenWidthScribble()
{
	int rowWidth = asInt(inParam1);
	float startWidth = asFloat(inParam2);
	float endWidth = asFloat(inParam3); 
	float incSize = asFloat(inParam4);

	float pw;
	int density , i;
	float oldPenWidth = penWidth;
	int iterations = 0;
	long posA = currentPosition(SX);
	long posB = currentPosition(DX);
	long startRow = posB;

	for (pw = startWidth; pw <= endWidth; pw+=incSize)
	{
		iterations++;

		penWidth = pw;
		int maxDens = maxDensity(penWidth, rowWidth);
		for (density = maxDens; density >= 0; density--)
		{
			drawScribblePixel(posA, posB, rowWidth, density);
			posB+=rowWidth;
		}
		posA+=rowWidth;
		posB = startRow;
	}
	changeLength(posA-(rowWidth/2), startRow-(rowWidth/2));
	penWidth = oldPenWidth;
	moveB(0-rowWidth);
	for (i = 1; i <= iterations; i++)
	{
		moveB(0-(rowWidth/2));
		moveA(0-rowWidth);
		moveB(rowWidth/2);
	}
	penWidth = oldPenWidth;
}
//////////////////////////////////////////////////////////////////
// Draw a Rectangle												//
//////////////////////////////////////////////////////////////////
#ifdef CMD_DRAWRECT
void drawRectangle()
{
	long v1A = asLong(inParam1);
	long v1B = asLong(inParam2);
	long v2A = asLong(inParam3);
	long v2B = asLong(inParam4);

	changeLength(v1A, v1B);
	moveTo(v2A, SX);
	runToPosition(SX);

	moveTo(v2B, DX);
	runToPosition(DX);

	moveTo(v1A, SX);
	runToPosition(SX);

	moveTo(v1B, DX);
	runToPosition(DX);
}
#endif /* CMD_DRAWRECT */
//////////////////////////////////////////////////////////////////
// move until reach final position								//
//////////////////////////////////////////////////////////////////
void runToPosition(unsigned char side)
{
	while (run(side));
}
//////////////////////////////////////////////////////////////////
// Change lenght Command										//
//////////////////////////////////////////////////////////////////
void changeLength()
{
	long lenA = asLong(inParam1);
	long lenB = asLong(inParam2);

	if (lenA == 0)
		lenA = 10;
	if (lenB == 0)
		lenB = 10;

	changeLength_long(lenA, lenB);
}
//////////////////////////////////////////////////////////////////
// Change lenght long integer									//
//////////////////////////////////////////////////////////////////
void changeLength_long(long tA, long tB)
{
	moveTo(tA , SX);
	moveTo(tB , DX);

	while (distanceToGo_function(SX) != 0 || distanceToGo_function(DX) != 0)
	{
		run(SX);
		run(DX);
	}
	reportPosition();
}
//////////////////////////////////////////////////////////////////
// Change lenght floating point									//
//////////////////////////////////////////////////////////////////
void changeLength_float(float tA, float tB)
{
	moveTo(tA , SX);
	moveTo(tB , DX);

	while (distanceToGo_function(SX) != 0 || distanceToGo_function(DX) != 0)
	{
		run(SX);
		run(DX);
	}
	reportPosition();
}

//void changeLengthRelative(long tA, long tB)
//{
//  accelA.move(tA);
//  accelB.move(tB);
//  
//  while (accelA.distanceToGo() != 0 || accelB.distanceToGo() != 0)
//  {
//    accelA.run();
//    accelB.run();
//  }
//  
//  reportPosition();
//}
//////////////////////////////////////////////////////////////////
// Get Max Lenght												//
//////////////////////////////////////////////////////////////////
long getMaxLength()
{
	if (maxLength == 0)
	{
		float length = getMachineA(machineWidth * stepsPerMM, machineHeight * stepsPerMM);
		maxLength = (long)(length+0.5);
	}
	return maxLength;
}
//////////////////////////////////////////////////////////////////
// Change Lenght Direct											//
//////////////////////////////////////////////////////////////////
#ifdef CMD_CHANGELENGTHDIRECT
void changeLengthDirect()
{
	float endA = asFloat(inParam1);
	float endB = asFloat(inParam2);
	int maxSegmentLength = asInt(inParam3);

	float startA = currentPosition(SX);
	float startB = currentPosition(DX);

	if (endA < 20 || endB < 20 || endA > getMaxLength() | endB > getMaxLength())
	{
		Serial_print("This point falls outside the area of this machine. Skipping it." , 1);
	}
	else
	{
		drawBetweenPoints(startA, startB, endA, endB, maxSegmentLength);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//Thanks to Andy Kinsman for help with this method.														//
//																										//
//This moves the gondola in a straight line between p1 and p2.  Both input coordinates are in 			//
//the native coordinates system.  																		//
//																										//
//The fidelity of the line is controlled by maxLength - this is the longest size a line segment is 		//
//allowed to be.  1 is finest, slowest.  Use higher values for faster, wobblier.						//
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawBetweenPoints(float p1a, float p1b, float p2a, float p2b, int maxSegmentLength)
{
	// ok, we're going to plot some dots between p1 and p2.  Using maths. I know! Brave new world etc.
	reportingPosition = false;

	// First, convert these values to cartesian coordinates
	// We're going to figure out how many segments the line
	// needs chopping into.
	float c1x = getCartesianXFP(p1a, p1b);
	float c1y = getCartesianYFP(c1x, p1a);

	float c2x = getCartesianXFP(p2a, p2b);
	float c2y = getCartesianYFP(c2x, p2a);

	// test to see if it's on the page
	if (c2x > 20 && c2x<pageWidth-20 && c2y > 20 && c2y <pageHeight-20)
	{
		float deltaX = c2x-c1x;		// distance each must move (signed)
		float deltaY = c2y-c1y;

		int linesegs = 1;			// assume at least 1 line segment will get us there.
		if (abs(deltaX) > abs(deltaY))
		{
			// slope <=1 case
			while ((abs(deltaX)/linesegs) > maxSegmentLength)
			{
				linesegs++;
			}
		}
		else
		{
			// slope >1 case
			while ((abs(deltaY)/linesegs) > maxSegmentLength)
			{
				linesegs++;
			}
		}

		// reduce delta to one line segments' worth.
		deltaX = deltaX/linesegs;
		deltaY = deltaY/linesegs;

		// render the line in N shorter segments
		while (linesegs > 0)
		{
			// compute next new location
			c1x = c1x + deltaX;
			c1y = c1y + deltaY;

			// convert back to machine space
			float pA = getMachineA(c1x, c1y);
			float pB = getMachineB(c1x, c1y);

			// do the move
			useAcceleration(false);
			changeLength_float(pA, pB);

			// one line less to do!
			linesegs--;
		}

		// do the end point in case theres been some rounding errors etc
		reportingPosition = true;
		changeLength_float(p2a, p2b);
		useAcceleration(true);
	}
	else
	{
		Serial_print("Line is not on the page. Skipping it." , 1);
	}
	outputAvailableMemory();
}
#endif /* CMD_CHANGELENGTHDIRECT */
//////////////////////////////////////////////////////////////////
// Use Accelaration												//
//////////////////////////////////////////////////////////////////
void useAcceleration(unsigned char use)
{
	if (use)
	{
		setAcceleration(currentAcceleration , SX);
		setAcceleration(currentAcceleration , DX);
	}
	else
	{
		setAcceleration(SUPERFAST_ACCELERATION , SX);
		setAcceleration(SUPERFAST_ACCELERATION , DX);
	}
}
//////////////////////////////////////////////////////////////////
// set stepper acceleration										//
//////////////////////////////////////////////////////////////////
void setAcceleration(float Acceleration , unsigned char side)
{
	acceleration[side] = Acceleration;
	computeNewSpeed(side);
}
//////////////////////////////////////////////////////////////////
// get machine A lenght											//
//////////////////////////////////////////////////////////////////
float getMachineA(float cX, float cY)
{
	float a = sqrt(cX*cX + cY*cY);
	return a;
}
//////////////////////////////////////////////////////////////////
// get machine B lenght											//
//////////////////////////////////////////////////////////////////
float getMachineB(float cX, float cY)
{
	float b = sqrt((pageWidth-cX)*(pageWidth-cX) + cY*cY);
	return b;
}
//////////////////////////////////////////////////////////////////
// Test direction Square										//
//////////////////////////////////////////////////////////////////
void drawTestDirectionSquare()
{
	int rowWidth = asInt(inParam1);
	int segments = asInt(inParam2);
	drawSquarePixel(rowWidth, rowWidth, segments, DIR_SE);
	moveA(rowWidth*2);

	drawSquarePixel(rowWidth, rowWidth, segments, DIR_SW);
	moveB(rowWidth*2);

	drawSquarePixel(rowWidth, rowWidth, segments, DIR_NW);
	moveA(0-(rowWidth*2));

	drawSquarePixel(rowWidth, rowWidth, segments, DIR_NE);
	moveB(0-(rowWidth*2));
}
//////////////////////////////////////////////////////////////////
// Draw Square Pixel from command parameters					//
//////////////////////////////////////////////////////////////////
#ifdef CMD_DRAWPIXEL

void drawSquarePixel_command() 
{
	long originA = asLong(inParam1);
	long originB = asLong(inParam2);
	int size = asInt(inParam3);
	int density = asInt(inParam4);
	
	int halfSize = size / 2;
	
	long startPointA;
	long startPointB;
	long endPointA;
	long endPointB;
	
	int calcFullSize = halfSize * 2; // see if there's any rounding errors
	int offsetStart = size - calcFullSize;
	
	if (globalDrawDirectionMode == DIR_MODE_AUTO)
	globalDrawDirection = getAutoDrawDirection(originA, originB, currentPosition(SX), currentPosition(DX));

	if (globalDrawDirection == DIR_SE)
	{
		startPointA = originA - halfSize;
		startPointA += offsetStart;
		startPointB = originB;
		endPointA = originA + halfSize;
		endPointB = originB;
	}
	else if (globalDrawDirection == DIR_SW)
	{
		startPointA = originA;
		startPointB = originB - halfSize;
		startPointB += offsetStart;
		endPointA = originA;
		endPointB = originB + halfSize;
	}
	else if (globalDrawDirection == DIR_NW)
	{
		startPointA = originA + halfSize;
		startPointA -= offsetStart;
		startPointB = originB;
		endPointA = originA - halfSize;
		endPointB = originB;
	}
	else //(drawDirection == DIR_NE)
	{
		startPointA = originA;
		startPointB = originB + halfSize;
		startPointB -= offsetStart;
		endPointA = originA;
		endPointB = originB - halfSize;
	}

	if (isPenUp == false)
	{
		density = scaleDensity(density, 255, maxDensity(penWidth, size));

		changeLength_long(startPointA, startPointB);
		if (density > 1)
		{
			drawSquarePixel(size, size, density, globalDrawDirection);
		}
	}
	changeLength_long(endPointA, endPointB);

	outputAvailableMemory(); 
}
//////////////////////////////////////////////////////////////////
// Get Draw Direction											//
//////////////////////////////////////////////////////////////////
unsigned char getAutoDrawDirection(long targetA, long targetB, long sourceA, long sourceB)
{
	unsigned char dir = DIR_SE;
// some bitchin triangles, I goshed-well love triangles.
//  long diffA = sourceA - targetA;
//  long diffB = sourceB - targetB;
//  long hyp = sqrt(sq(diffA)+sq(diffB));
//  
//  float bearing = atan(hyp/diffA);

//  Serial.print("bearing:");
//  Serial.println(bearing);
//  
//  Serial.print(F("TargetA: "));
//  Serial.print(targetA);
//  Serial.print(F(", targetB: "));
//  Serial.print(targetB);
//  Serial.print(F(". SourceA: "));
//  Serial.print(sourceA);
//  Serial.print(F(", sourceB: "));
//  Serial.print(sourceB);
//  Serial.println(F("."));

	if (targetA<sourceA && targetB<sourceA)
	{
		dir = DIR_NW;
	}
	else if (targetA>sourceA && targetB>sourceB)
	{
		dir = DIR_SE;
	}
	else if (targetA<sourceA && targetB>sourceB)
	{
		dir = DIR_SW;
	}
	else if (targetA>sourceA && targetB<sourceB)
	{
		dir = DIR_NE;
	}
	else if (targetA==sourceA && targetB<sourceB)
	{
		dir = DIR_NE;
	}
	else if (targetA==sourceA && targetB>sourceB)
	{
		dir = DIR_SW;
	}
	else if (targetA<sourceA && targetB==sourceB)
	{
		dir = DIR_NW;
	}
	else if (targetA>sourceA && targetB==sourceB)
	{
		dir = DIR_SE;
	}
	else
	{
	}
	return dir;
}
#endif /* CMD_DRAWPIXEL */
//////////////////////////////////////////////////////////////////
// Draw Scribble Pixel from command parameters					//
//////////////////////////////////////////////////////////////////
#ifdef CMD_DRAWSCRIBBLEPIXEL
void drawScribblePixel_from_command()
{
	long originA = asLong(inParam1);
	long originB = asLong(inParam2);
	int size = asInt(inParam3);
	int density = asInt(inParam4);

	int maxDens = maxDensity(penWidth, size);

	density = scaleDensity(density, 255, maxDens);
	drawScribblePixel(originA, originB, size*1.1, density);

	outputAvailableMemory();
}
//////////////////////////////////////////////////////////////////
// Draw Scribble Pixel											//
//////////////////////////////////////////////////////////////////
void drawScribblePixel(long originA, long originB, int size, int density)
{
	long lowLimitA = originA-(size/2);
	long highLimitA = lowLimitA+size;
	long lowLimitB = originB-(size/2);
	//long highLimitB = lowLimitB+size;
	int randA;
	int randB;
	int i;

	int inc = 0;
	int currSize = size;

	for (i = 0; i <= density; i++)
	{
		randA = random(0, currSize);
		randB = random(0, currSize);
		changeLength_long(lowLimitA+randA, lowLimitB+randB);

		lowLimitA-=inc;
		highLimitA+=inc;
		currSize+=inc*2;
	}
}
#endif /* CMD_DRAWSCRIBBLEPIXEL */
//////////////////////////////////////////////////////////////////
// Random Draw Direction										//
//////////////////////////////////////////////////////////////////
unsigned char getRandomDrawDirection()
{
	return random(1, 5);
}
//////////////////////////////////////////////////////////////////
// Return random number from min to max							//
//////////////////////////////////////////////////////////////////
int random(int min, int max)
{
	int num = rand();
	return ((num % (max-min)) + min);
}

//int minSegmentSizeForPen(float penSize)
//{
//  float penSizeInSteps = penSize * stepsPerMM;
//
//  int minSegSize = 1;
//  if (penSizeInSteps >= 2.0)
//    minSegSize = int(penSizeInSteps);
//    
////  Serial.print(F("Min segment size for penSize "));
////  Serial.print(penSize);
////  Serial.print(F(": "));
////  Serial.print(minSegSize);
////  Serial.print(F(" steps."));
////  Serial.println();
//  
//  return minSegSize;
//}
//////////////////////////////////////////////////////////////////
// get Max Density												//
//////////////////////////////////////////////////////////////////
int maxDensity(float penSize, int rowSize)
{
	float rowSizeInMM = mmPerStep * rowSize;
	PRINT("rowsize in mm: " , 0)
#ifndef POLARGRAPH_STANDALONE
	ltoa(rowSizeInMM,temp_char);
#endif
	PRINT(temp_char , 1)

	float numberOfSegments = rowSizeInMM / penSize;
	int maxDens = 1;
	if (numberOfSegments >= 2.0)
		maxDens = (int)numberOfSegments;

	PRINT("Max density: ",0)
#ifndef POLARGRAPH_STANDALONE
	itoa(maxDens,temp_char,10);
#endif
	PRINT(temp_char, 1)

	return maxDens;
}
//////////////////////////////////////////////////////////////////
// Scale Density												//
//////////////////////////////////////////////////////////////////
int scaleDensity(int inDens, int inMax, int outMax)
{
	float reducedDens = ((float)inDens / (float)inMax) * (float)outMax;
	reducedDens = outMax-reducedDens;

	// round up if bigger than .5
	int result = (int)(reducedDens);
	if (reducedDens - (result) > 0.5)
		result ++;

	return result;
}
//////////////////////////////////////////////////////////////////
// Draw Square Pixel											//
//////////////////////////////////////////////////////////////////
void drawSquarePixel(int length, int width, int density, unsigned char drawDirection)
{
	// work out how wide each segment should be
	int segmentLength = 0;
	int i;

	if (density > 0)
	{
		// work out some segment widths
		int basicSegLength = length / density;
		int basicSegRemainder = length % density;
		float remainderPerSegment = (float)basicSegRemainder / (float)density;
		float totalRemainder = 0.0;
		int lengthSoFar = 0;

		for (i = 0; i <= density; i++)
		{
			totalRemainder += remainderPerSegment;
			if (totalRemainder >= 1.0)
			{
				totalRemainder -= 1.0;
				segmentLength = basicSegLength+1;
			}
			else
			{
				segmentLength = basicSegLength;
			}

			if (drawDirection == DIR_SE) {
				drawSquareWaveAlongA(width, segmentLength, density, i);
			}
			if (drawDirection == DIR_SW) {
				drawSquareWaveAlongB(width, segmentLength, density, i);
			}
			if (drawDirection == DIR_NW) {
				segmentLength = 0 - segmentLength; // reverse
				drawSquareWaveAlongA(width, segmentLength, density, i);
			}
			if (drawDirection == DIR_NE) {
				segmentLength = 0 - segmentLength; // reverse
				drawSquareWaveAlongB(width, segmentLength, density, i);
			}
			lengthSoFar += segmentLength;
			reportPosition();
		} // end of loop
	}
}
//////////////////////////////////////////////////////////////////
// Draw Square Wave A long A									//
//////////////////////////////////////////////////////////////////
void drawSquareWaveAlongA(int waveAmplitude, int waveLength, int totalWaves, int waveNo)
{
	if (waveNo == 0)
	{
		// first one, half a line and an along
		PRINT("First wave half" , 1)
		if (lastWaveWasTop) {
			moveB(waveAmplitude/2);
			moveA(waveLength);
		}
		else {
			moveB(0-(waveAmplitude/2));
			moveA(waveLength);
		}
		flipWaveDirection();
	}
	else if (waveNo == totalWaves)
	{
		// last one, half a line with no along
		PRINT("Last wave half" , 1)
		if (lastWaveWasTop) {
			moveB(waveAmplitude/2);
		}
		else {
			moveB(0-(waveAmplitude/2));
		}
	}
	else
	{
		// intervening lines - full lines, and an along
		PRINT("Intermediate waves" , 1)
		if (lastWaveWasTop) {
			moveB(waveAmplitude);
			moveA(waveLength);
		}
		else {
			moveB(0-waveAmplitude);
			moveA(waveLength);
		}
		flipWaveDirection();
	}
}
//////////////////////////////////////////////////////////////////
// Draw Square Wave A long B									//
//////////////////////////////////////////////////////////////////
void drawSquareWaveAlongB(int waveAmplitude, int waveLength, int totalWaves, int waveNo)
{
	if (waveNo == 0)
	{
		// first one, half a line and an along
		PRINT("First wave half" , 1)
		if (lastWaveWasTop) {
			moveA(waveAmplitude/2);
			moveB(waveLength);
		}
		else {
			moveA(0-(waveAmplitude/2));
			moveB(waveLength);
		}
		flipWaveDirection();
	}
	else if (waveNo == totalWaves)
	{
		// last one, half a line with no along
		PRINT("Last wave half" , 1)
		if (lastWaveWasTop) {
			moveA(waveAmplitude/2);
		}
		else {
			moveA(0-(waveAmplitude/2));
		}
	}
	else
	{
		// intervening lines - full lines, and an along
		PRINT("Intermediate waves" , 1)
		if (lastWaveWasTop) {
			moveA(waveAmplitude);
			moveB(waveLength);
		}
		else {
			moveA(0-waveAmplitude);
			moveB(waveLength);
		}
		flipWaveDirection();
	}
}
//////////////////////////////////////////////////////////////////
// Flip Wave Direction											//
//////////////////////////////////////////////////////////////////
void flipWaveDirection()
{
	if (lastWaveWasTop)
		lastWaveWasTop = false;
	else
		lastWaveWasTop = true;
}
//////////////////////////////////////////////////////////////////
// Move A (SX)													//
//////////////////////////////////////////////////////////////////
void moveA(long dist)
{
	move(dist, SX);
	while (distanceToGo_function(SX) != 0)
		run(SX);
}
//////////////////////////////////////////////////////////////////
// Move B (DX)													//
//////////////////////////////////////////////////////////////////
void moveB(long dist)
{
	move(dist , DX);
	while (distanceToGo_function(DX) != 0)
		run(DX);
}

long distanceToGo_function(unsigned char side)
{
	return target_pos[side] - current_pos[side];
}
//////////////////////////////////////////////////////////////////
// Run from current position to destination						//
// This function do the step									//
//////////////////////////////////////////////////////////////////
unsigned char run(unsigned char side)
{
	if (target_pos[side] == current_pos[side])
		return false;

	if (runSpeed(side))
		computeNewSpeed(side);

	return true;
}
// Implements steps according to the current speed
// You must call this at least once per step
// returns true if a step occurred
unsigned char runSpeed(unsigned char side)
{
	unsigned long time = millis();

	if (time > lastStepTime[side] + stepInterval[side])
	{
		if (speed[side] > 0)
		{
			// Clockwise
			current_pos[side] += 1;
		}
		else if (speed[side] < 0)
		{
			// Anticlockwise
			current_pos[side] -= 1;
		}
		step(current_pos[side] & 0x3 , side); // Bottom 2 bits (same as mod 4, but works with + and - numbers) 

		lastStepTime[side] = time;
		return true;
	}
	else
		return false;
}
// Subclasses can override
void step(unsigned int step, unsigned char side)  // ???????????
{
	//digitalWrite(_pin2, _speed > 0); // Direction
	// Caution 200ns setup time 
	//digitalWrite(_pin1, HIGH);
	// Caution, min Step pulse width for 3967 is 1microsec
	// Delay 1microsec
	//delayMicroseconds(1);
	//digitalWrite(_pin1, LOW);
	if(side)
	{
		if(speed[side] > 0)
			dx_stepper_port_out |= dx_stepper_dir;
		else dx_stepper_port_out &= ~dx_stepper_dir;
		delay_us(1);
		dx_stepper_port_out |= dx_stepper_step;
		delay_us(5);
		dx_stepper_port_out &= ~dx_stepper_step;
	}
	else
	{
		if(speed[side] > 0)
			sx_stepper_port_out |= sx_stepper_dir;
		else sx_stepper_port_out &= ~sx_stepper_dir;
		delay_us(1);
		sx_stepper_port_out |= sx_stepper_step;
		delay_us(5);
		sx_stepper_port_out &= ~sx_stepper_step;
	}
}
//////////////////////////////////////////////////////////////////
// Compute new speed											//
//////////////////////////////////////////////////////////////////
void computeNewSpeed(unsigned char side)
{
	setSpeed(desiredSpeed(side) , side);
}
void setSpeed(float speed_target , unsigned char side)
{
	speed[side] = speed_target;
	stepInterval[side] = abs(1000.0 / speed[side]);
}
// Work out and return a new speed.
// Subclasses can override if they want
// Implement acceleration, deceleration and max speed
// Negative speed is anticlockwise
// This is called:
//  after each step
//  after user changes:
//   maxSpeed
//   acceleration
//   target position (relative or absolute)
float desiredSpeed(unsigned char side)
{
	long distanceTo = distanceToGo_function(side);

	// Max possible speed that can still decelerate in the available distance
	float requiredSpeed;
	if (distanceTo == 0)
		return 0.0; // Were there
	else if (distanceTo > 0) // Clockwise
		requiredSpeed = sqrt(2.0 * distanceTo * acceleration[side]);
	else  // Anticlockwise
		requiredSpeed = -sqrt(2.0 * -distanceTo * acceleration[side]);

	if (requiredSpeed > speed[side])
	{
		// Need to accelerate in clockwise direction
		if (speed[side] == 0)
			requiredSpeed = sqrt(2.0 * acceleration[side]);
		else
			requiredSpeed = speed[side] + abs(acceleration[side] / speed[side]);
		if (requiredSpeed > maxSpeed[side])
			requiredSpeed = maxSpeed[side];
	}
	else if (requiredSpeed < speed[side])
	{
		// Need to accelerate in anticlockwise direction
		if (speed[side] == 0)
			requiredSpeed = -sqrt(2.0 * acceleration[side]);
		else
			requiredSpeed = speed[side] - abs(acceleration[side] / speed[side]);
		if (requiredSpeed < - maxSpeed[side])
			requiredSpeed = - maxSpeed[side];
	}
	return requiredSpeed;
}
//////////////////////////////////////////////////////////////////
// Move relative from current position							//
//////////////////////////////////////////////////////////////////
void move(long relative , unsigned char side)
{
	moveTo(current_pos[side] + relative , side);
}
//////////////////////////////////////////////////////////////////
// move to new position											//
//////////////////////////////////////////////////////////////////
void moveTo(long destination , unsigned char side)
{
	target_pos[side] = destination;
	computeNewSpeed(side);
}
//////////////////////////////////////////////////////////////////
// Return current position										//
//////////////////////////////////////////////////////////////////
long currentPosition(unsigned char side)
{
	return current_pos[side];
}
//////////////////////////////////////////////////////////////////
// Print out current position									//
//////////////////////////////////////////////////////////////////
void reportPosition()
{
#ifndef POLARGRAPH_STANDALONE
	long pos_sx, pos_dx;
	if (reportingPosition)
	{
		PRINT(OUT_CMD_SYNC , 0)
		pos_sx = currentPosition(SX);
		ltoa(pos_sx,temp_char);
		PRINT(temp_char , 0)
		PRINT(COMMA , 0)
		pos_dx = currentPosition(DX);
		ltoa(pos_dx,temp_char);
		PRINT(temp_char , 0)
		PRINT(CMD_END , 1)

		outputAvailableMemory();
	}
#endif
}
//////////////////////////////////////////////////////////////////
// Set position													//
//////////////////////////////////////////////////////////////////
void setPosition()
{
	long targetA = asLong(inParam1);
	long targetB = asLong(inParam2);

	setCurrentPosition(targetA , SX);
	setCurrentPosition(targetB , DX);

	engageMotors();
	reportPosition();
}
//////////////////////////////////////////////////////////////////
// Set Current Position											//
//////////////////////////////////////////////////////////////////
void setCurrentPosition(long Position , unsigned char side)
{
	current_pos[side] = Position;
}
//////////////////////////////////////////////////////////////////
// Engage Motors												//
//////////////////////////////////////////////////////////////////
void engageMotors()
{
	sx_stepper_port_out &= ~sx_stepper_enable;
	dx_stepper_port_out &= ~dx_stepper_enable;
}
//////////////////////////////////////////////////////////////////
// Run to the new position										//
//////////////////////////////////////////////////////////////////
void runToNewPosition(long position , unsigned char side)
{
	moveTo(position , side);
	runToPosition(side);
}
//////////////////////////////////////////////////////////////////
// Release Motor												//
//////////////////////////////////////////////////////////////////
void releaseMotors()
{
#ifdef CMD_PENUP
	penUp();
#endif /* CMD_PENUP */
}
//////////////////////////////////////////////////////////////////
// Get cartesian X coordinate									//
//////////////////////////////////////////////////////////////////
float getCartesianXFP(float aPos, float bPos)
{
	float calcX = (pageWidth*pageWidth - bPos*bPos + aPos*aPos) / (pageWidth*2);
	return calcX;
}
//////////////////////////////////////////////////////////////////
// Get cartesian Y coordinate									//
//////////////////////////////////////////////////////////////////
float getCartesianYFP(float cX, float aPos)
{
	float calcY = sqrt(aPos*aPos - cX*cX);
	return calcY;
}

//////////////////////////////////////////////////////////////////
// Find Available Memory										//
//from http://www.arduino.cc/playground/Code/AvailableMemory	//
//////////////////////////////////////////////////////////////////
int availableMemory()
{
	unsigned int *heapptr, *stackptr;
	stackptr = (unsigned int*)malloc(4);
	heapptr = stackptr;
	free(stackptr);

	stackptr = (void*) _get_SP_register();
	return stackptr - heapptr;
}
//////////////////////////////////////////////////////////////////
// Output Available Memory to Host								//
//////////////////////////////////////////////////////////////////
void outputAvailableMemory()
{
#ifndef POLARGRAPH_STANDALONE
	long avMem = availableMemory();
	if (avMem != availMem)
	{
		availMem = avMem;
		PRINT(FREE_MEMORY_STRING , 0)
		itoa(availMem,temp_char,10);
		PRINT(temp_char , 0)
		PRINT(CMD_END , 1)
	}
#endif
}

float rads(int n) {
	// Return an angle in radians
	return (n/180.0 * PI);
}

float desiredSpeed_new(long distanceTo, float currentSpeed, float acceleration)
{
    float requiredSpeed;

    if (distanceTo == 0)
	return 0.0f; // We're there

    // sqrSpeed is the signed square of currentSpeed.
    float sqrSpeed = currentSpeed * currentSpeed;
    if (currentSpeed < 0.0)
      sqrSpeed = -sqrSpeed;
      
    float twoa = 2.0f * acceleration; // 2ag
    
    // if v^^2/2as is the the left of target, we will arrive at 0 speed too far -ve, need to accelerate clockwise
    if ((sqrSpeed / twoa) < distanceTo)
    {
	// Accelerate clockwise
	// Need to accelerate in clockwise direction
	if (currentSpeed == 0.0f)
	    requiredSpeed = sqrt(twoa);
	else
	    requiredSpeed = currentSpeed + fabs(acceleration / currentSpeed);

	if (requiredSpeed > currentMaxSpeed)
	    requiredSpeed = currentMaxSpeed;
    }
    else
    {
	// Decelerate clockwise, accelerate anticlockwise
	// Need to accelerate in clockwise direction
	if (currentSpeed == 0.0f)
	    requiredSpeed = -sqrt(twoa);
	else
	    requiredSpeed = currentSpeed - fabs(acceleration / currentSpeed);
	if (requiredSpeed < -currentMaxSpeed)
	    requiredSpeed = -currentMaxSpeed;
    }
    
    //Serial.println(requiredSpeed);
    return requiredSpeed;
}

#ifdef CMD_DRAWCIRCLEPIXEL
void curves_pixel_drawCircularPixel() 
{
    long originA = asLong(inParam1);
    long originB = asLong(inParam2);
    int size = asInt(inParam3);
    int density = asInt(inParam4);

    int radius = size / 2;

    //Serial.print("Density before: ");
    //Serial.println(density);
    
    int maxDensity_var = maxDensity(penWidth, radius);
    //Serial.print("Max density: " );
    //Serial.print(maxDensity);
    //Serial.print(", penWidth: ");
    //Serial.print(penWidth);
    //Serial.print(", radius: ");
    //Serial.println(radius);
     
    density = scaleDensity(density, 255, maxDensity_var);
    //Serial.print("Density scaled");
    //Serial.println(density);
    
    int increment = radius / maxDensity_var;

    if (density > 0)
    {
      //Serial.print("Density:");
      //Serial.println(density);
      curves_drawSpiral(originA, originB, radius, increment, density);
    }
    
}
void curves_drawCurve(long x, long y, long fx, long fy, long cx, long cy, int speedOfSegment) {
  // Draw a Quadratic Bezier curve from (x, y) to (fx, fy) using control pt
  // (cx, cy)
  float xt=0;
  float yt=0;
  float t;

  for (t=0; t<=1; t+=.01) {
    xt = pow((1-t),2) *x + 2*t*(1-t)*cx+ pow(t,2)*fx;
    yt = pow((1-t),2) *y + 2*t*(1-t)*cy+ pow(t,2)*fy;
    
    if (speedOfSegment != 0)
    {
      setSpeed(speedOfSegment,SX);
      setSpeed(speedOfSegment,DX);
      //usingAcceleration = false;
    }
    reportingPosition = false;
    changeLength_float(xt, yt);
    reportingPosition = true;
    //usingAcceleration = true;
  }  
}

void curves_drawSpiral(long centerx, long centery, int maxRadius, int increment, int density) 
{
  //Serial.println("Draw spiral.");
  //Serial.print("Max radius: ");
  //Serial.println(maxRadius);
  //Serial.print("Increment:");
  //Serial.println(increment);
  //Serial.print("density:");
  //Serial.println(density);
  // Estimate a circle using 20 arc Bezier curve segments
  int segments = 20;
  int totalSegments = segments * density;

  
  float radius = (float)(increment);
  
  int turns;
  // work out how many shells to draw
  int runSpeed_var = 0;
  for (turns = 0; turns < density; turns++)
  {
//    if (increment < 80)
//      segments = radius / 4;
//    if (segments < 5)
//      segments = 5;
      

    float anglePerSegment = 360/segments;
    
    float radiusIncrementPerSegment = (float)(increment) / (float)(segments);
  
    int angle1 = 0;
    int midpoint=0;
    char firstMove = true;
    float angle2;
    for (angle2=anglePerSegment; angle2<=360; angle2+=anglePerSegment) 
    {

      midpoint = angle1+(angle2-angle1)/2;
  
      float startx=centerx+radius*cos(rads(angle1));
      float starty=centery+radius*sin(rads(angle1));
      float endx=centerx+radius*cos(rads(angle2));
      float endy=centery+radius*sin(rads(angle2));
      
      int t1 = rads(angle1)*1000 ;
      int t2 = rads(angle2)*1000;
      //int t3 = angle1;
      //int t4 = angle2;
  
      if (firstMove)
      {
        changeLength_float(startx, starty);
        firstMove = false;
      }
      // do the move
      runSpeed_var = desiredSpeed_new(totalSegments, runSpeed_var, currentAcceleration*2);
//      Serial.print("segment: ");
//      Serial.print(totalSegments);
//      Serial.print(", runSpeed: ");
//      Serial.println(runSpeed);
      //usingAcceleration = false;
      curves_drawCurve(startx,starty,endx,endy
                ,centerx+2*(radius*cos(rads(midpoint))-.25*(radius*cos(rads(angle1)))-.25*(radius*cos(rads(angle2))))
                ,centery+2*(radius*sin(rads(midpoint))-.25*(radius*sin(rads(angle1)))-.25*(radius*sin(rads(angle2))))
                ,runSpeed_var
      );
      totalSegments--;
      //usingAcceleration = true;
      angle1=angle2;
      radius += radiusIncrementPerSegment;

    }
//    Serial.println("Finished drawing turn.");
  }
  //Serial.println("Finished spiral pixel.");
}
#endif /* CMD_DRAWCIRCLEPIXEL */

long getCartesianX() {
	long calcX = getCartesianXFP(currentPosition(SX), currentPosition(DX));
	return calcX;
}

long getCartesianY() {
	return getCartesianYFP(getCartesianX(), currentPosition(SX));
}

#endif /*POLARGRAPH_SERVER_H_*/
