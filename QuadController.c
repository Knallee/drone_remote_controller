/*
 * QuadController.c
 *
 * Created: 2016-04-27 16:04:19
 *  Author: et05cc8
 */ 
#define F_CPU 16000000UL

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>

#include "mrLCD/MrLCD.h"
#include "uart/Uart.h"
#include "mpu6050/mpu6050.h"
#include "i2chw/TWI_master.h"

// Macros to initialize the variables controlled by the rotary encoder
//////////////////////////////////////////////////////////////////////
#define KP_PITCH_x10		150
#define KI_PITCH_x10		20
#define KD_PITCH_x10		80
#define KP_FF_PITCH_x10		20
#define PITCH_FF_GAIN_x10	8
#define PITCH_OFFSET		500

#define KP_ROLL_x10			150
#define KI_ROLL_x10			10
#define KD_ROLL_x10			80
#define KP_FF_ROLL_x10		20
#define ROLL_FF_GAIN_x10	8
#define ROLL_OFFSET			500

#define KP_YAW_x10		350
#define KI_YAW_x10		10
#define KD_YAW_x10		50
#define KP_FF_YAW_x10	20
#define YAW_FF_GAIN_x10	10
#define YAW_OFFSET		500

#define THRUST_CENTER_LEVEL		500
//////////////////////////////////////////////////////////////////////
#define THRUST_RANGE_OUTDOORS	249
#define THRUST_RANGE_INDOORS	149


//////////////////////////////////////////////////////////////////////

// Joysticks/ADC Variables
//////////////////////////////////////////////////////////////////////
static volatile unsigned long	potentiometer[4]		= {0};		//Holds ADC single values
static volatile unsigned long	potentiometerBuffer[4]	= {0};		//Holds ADC average values
static volatile unsigned long	pBias[4]				= {0};
	
static volatile uint8_t pBiasFlag			= 0;					//Stores the initial values of the potentiometers
static volatile uint8_t	countADC			= 0;					//Each ADC value is the average of 4 values
//////////////////////////////////////////////////////////////////////

// Rotary encoder Variables
//////////////////////////////////////////////////////////////////////	
static volatile int16_t		trim[19]				= {KP_PITCH_x10, KI_PITCH_x10, KD_PITCH_x10, KP_FF_PITCH_x10, PITCH_FF_GAIN_x10, PITCH_OFFSET,
													   KP_ROLL_x10, KI_ROLL_x10, KD_ROLL_x10, KP_FF_ROLL_x10, ROLL_FF_GAIN_x10, ROLL_OFFSET,
													   KP_YAW_x10, KI_YAW_x10, KD_YAW_x10, KP_FF_YAW_x10, YAW_FF_GAIN_x10, YAW_OFFSET,
													   THRUST_CENTER_LEVEL};
													   
static volatile uint8_t	trimmedVariable		= 0;
static volatile uint8_t trimRPY				= 0;
//////////////////////////////////////////////////////////////////////



// Function declarations
//////////////////////////////////////////////////////////////////////
void init_LEDs();
void init_buttons();
void init_ADC();
void init_encoder();

void send_initialization_data();
void waiting_for_quad();
void send_trim_packet(uint8_t Variable);

unsigned long thrust_range_transform(unsigned long input, uint8_t environment);
unsigned long RandP_range_transform(unsigned long input, uint8_t index);
//////////////////////////////////////////////////////////////////////

// Main Function
int main(void) {
	
	// Give time for the controller circuits to start
	_delay_ms(2000);
	
	//Final potentiometer value that is only changed after a successful transformation
	unsigned long	potentiometerBufferB[4] = {0};			
	
	uint8_t dataPacket[18]	= {0};	// The data packet to be sent through the radio
	uint8_t pressed[6]		= {0};	// Variable to keep track of the buttons release
	char BTD[8]	 = {0};				// Binary to Decimal for LCD
	uint8_t mode = 0;				// Controller Mode
	
	// Pitch, Roll, and Yaw and quaternions from the MPU6050
	double qw = 1.0f;
	double qx = 0.0f;
	double qy = 0.0f;
	double qz = 0.0f;
	double roll = 0.0f;
	double pitch = 0.0f;
	double yaw = 0.0f;
	
	// Final Roll, Pitch, and Yaw to be sent to the Quad in Telekinesis mode
	uint16_t rollF  = 0;
	uint16_t pitchF = 0;
	
	uint16_t mergedData[10] = {500,500,500,500};	// Data from Computer after merge
	
	uint8_t		no_PC = 1;	// Haven't received any data from PC
	uint8_t		turn_off_thrust = 0;
	uint8_t		environment = 0;
	
	
	uint8_t temp = 0;	// A temporary index variable
	
	//The float trimming values that will be shown on the screen
	float	trimF[18]	= {0};
	
	//Initialize LCD	
	init_LCD();
		
	//Initialize UART
	init_uart(9600,8,0,0);
	
	//Enable global interrupt
	sei();
	
	//Initialize test LEDs pins (unused)
	init_LEDs();
		
	//Initialize buttons
	init_buttons();					   
	
	//Initialize ADC
	init_ADC();
		
	// delay for the potentiometers to settle on a value
	gotoMrLCDLocation(0, 0);
	send_A_String("Initializing");
	gotoMrLCDLocation(1, 0);
	send_A_String("Controller.  ");
	_delay_ms(600);
	gotoMrLCDLocation(1, 0);
	send_A_String("Controller.. ");
	_delay_ms(600);
	gotoMrLCDLocation(1, 0);
	send_A_String("Controller...");
	_delay_ms(600);
	
	// Start ADC	
	ADCSRA |=  1 << ADSC;  
	
	//Initialize the rotary encoder pins
	init_encoder();	
	//Initialize MPU6050
	mpu6050_init();
	_delay_ms(50);

	//Initialize mpu6050 DMP processor
	mpu6050_dmpInitialize();
	mpu6050_dmpEnable();
	_delay_ms(20);
	
	//Hold and display a message while waiting for data from the quad
	waiting_for_quad();
	
	//This conflicts with the dmp_init routine if set as 2 from the beginning
	MPUFlag = 2;
	
	//Start checking for connection failure from now
	QuadDisconnectedcount = 0;
	
	
	for(;;) 
	{	
		// If no signal is received from the Quad for more than 10 seconds
		if (QuadDisconnectedcount >= 100)
		{
			//Hold and display a message while waiting for data
			waiting_for_quad();
			
		// Else if the Quad is running:	
		} else {
			
			// Switching between the modes: Joysticks, MPU, PC, Trimming -- Orange Button
			if (bit_is_clear(PINB, 0))
			{
				if (pressed[0] == 0)
				{
					pressed[0] = 1;
					mode++;
					trimmedVariable		= 0;
					trimRPY				= 0;
					
					//Clear Screen!
					gotoMrLCDLocation(0, 0);
					send_A_String("                 ");
					gotoMrLCDLocation(1, 0);
					send_A_String("                 ");
					gotoMrLCDLocation(0, 0);

					if (mode == 4) mode = 0;
				}
			}else
			{
				pressed[0] = 0;
			}
			// End of Orange Button!
			
			
			// Send the initialization data on command -- Green Button
			if (bit_is_clear(PINB, 1))	
			{
				if (pressed[1] == 0)
				{
					pressed[1] = 1;
					// send trimming data!!
					send_initialization_data();
				}
			}else
			{
				pressed[1] = 0;
			}
			// End of Green Button!
			
			// Thrust center is set by the decoder and the potentiometer is used to tune around it :D
			potentiometerBufferB[0] = thrust_range_transform(potentiometerBuffer[0], environment);
			
			// Prepare, pack, and send data to Quad depending on the mode: Joystick | Telekinesis | PC | Trimming
			switch(mode)
			{
				case 0:		// Joysticks
				
				// Thrust is controlled by the encoder :D
				trimRPY	 = 4;
				trimmedVariable = 18;
				
				// Motors on/off --Joystick Button Left				
				if (bit_is_clear(PINA, 0))		
				{
					if (pressed[4] == 0)
					{
						pressed[4] = 1;
						turn_off_thrust ^= 0x01;
					}
				}else {
					pressed[4] = 0;
				}
				// End of Joystick Button Left!
				
				// Environment Indoors/Outdoors --Joystick Button Right
				if (bit_is_clear(PINA, 1))
				{
					if (pressed[5] == 0)
					{
						pressed[5] = 1;
						environment ^= 0x01;
					}
					}else {
					pressed[5] = 0;
				}
				// End of Joystick Button Right!
				
				// Increase/Decrease Yaw -- Red/White Button
				if (bit_is_clear(PINB, 2))		
				{
					potentiometerBufferB[1] = 750;
				}
				else if (bit_is_clear(PINB, 3))
				{
					potentiometerBufferB[1] = 250;
				}
				else{
					potentiometerBufferB[1] = 500;
				}
				// End of White Button!
				
				if (turn_off_thrust == 1) potentiometerBufferB[0] = 500;   // Turn off Motors
				
				// Setting the bias for the roll and pitch potentiometer:
				potentiometerBufferB[2] = RandP_range_transform(potentiometerBuffer[2], 2);
				potentiometerBufferB[3] = RandP_range_transform(potentiometerBuffer[3], 3);
								
				// Load the data into dataPacket
				dataPacket[0] = 0xFF;	// The Unique Header for Flight Control
				dataPacket[1] = 0xFE;
				dataPacket[2] = potentiometerBufferB[0]	>> 8;	// The Joystick data: Thrust, Yaw, Pitch, Roll
				dataPacket[3] = potentiometerBufferB[0]	& 0xFF;
				dataPacket[4] = potentiometerBufferB[1]	>> 8;
				dataPacket[5] = potentiometerBufferB[1]	& 0xFF;
				dataPacket[6] = potentiometerBufferB[2]	>> 8;
				dataPacket[7] = potentiometerBufferB[2]	& 0xFF;
				dataPacket[8] = potentiometerBufferB[3]	>> 8;
				dataPacket[9] = potentiometerBufferB[3]	& 0xFF;
				dataPacket[10] = 0xF7;	// The Unique Tail
				dataPacket[11] = 0x12;
				
				// Wait for the Quad to ask for data
				if (receivedData1 == 0xFF)
				{
					receivedData1 = 0;				
					transmitUART1(dataPacket, 12);
				}
				break;
				// End Mode 1
				
				
				case 1:		// MPU
				
				// Thrust is controlled by the encoder :D
				trimRPY	 = 4;
				trimmedVariable = 18;
				
				// Motors on/off --Joystick Button Left
				if (bit_is_clear(PINA, 0))
				{
					if (pressed[4] == 0)
					{
						pressed[4] = 1;
						turn_off_thrust ^= 0x01;
					}
					}else {
					pressed[4] = 0;
				}
				// End of Joystick Button Left!
				
				// Environment Indoors/Outdoors --Joystick Button Right
				if (bit_is_clear(PINA, 1))
				{
					if (pressed[5] == 0)
					{
						pressed[5] = 1;
						environment ^= 0x01;
					}
					}else {
					pressed[5] = 0;
				}
				// End of Joystick Button Right!
				
				// Increase/Decrease Yaw -- Red/White Button
				if (bit_is_clear(PINB, 2))
				{
					potentiometerBufferB[1] = 750;
				}
				else if (bit_is_clear(PINB, 3))
				{
					potentiometerBufferB[1] = 250;
				}
				else{
					potentiometerBufferB[1] = 500;
				}
				// End of White Button!
				
				if (turn_off_thrust == 1) potentiometerBufferB[0] = 500; // Turn off Motors
				
				// If the i2c is not busy with the MPU, get the MPU data
				if (MPUFlag == 2)
				{
					// Get Roll, Pitch, and Yaw from the sensor in degrees
					mpu6050_getQuaternion(mpu6050_fifoBuffer, &qw, &qx, &qy, &qz);
					mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);
					
					// Adjust the sensor data to make them look like joystick data
					rollF	= 500 + (uint16_t)(roll *5.6);			//500/180
					pitchF	= 500 + (uint16_t)(pitch*12.5);			//500/80
					
					// make sure the value is in the required range: 0-1024
					if (rollF > 2024)	rollF = 0;
					if (pitchF > 2024) pitchF = 0;
				}
				
				// Load the data into dataPacket
				dataPacket[0] = 0xFF;	// The Unique Header for Flight Control
				dataPacket[1] = 0xFE;
				dataPacket[2] = potentiometerBufferB[0]	>> 8;	// The Joystick data: Thrust, Yaw, Pitch, Roll
				dataPacket[3] = potentiometerBufferB[0]	& 0xFF;
				dataPacket[4] = potentiometerBufferB[1]	>> 8;
				dataPacket[5] = potentiometerBufferB[1]	& 0xFF;
				dataPacket[6] = rollF	>> 8;
				dataPacket[7] = rollF	& 0xFF;
				dataPacket[8] = pitchF	>> 8;
				dataPacket[9] = pitchF	& 0xFF;
				dataPacket[10] = 0xF7;	// The Unique Tail
				dataPacket[11] = 0x12;
				
				// Wait for the Quad to ask for data
				if (receivedData1 == 0xFF)
				{
					receivedData1 = 0;			
					transmitUART1(dataPacket, 12);
				}
				break;
				// End mode 2
				
				
				case 2:		// Computer
				
				// See whether or not we receive data from PC
				if (CPUDataReady == 1)
				{
					no_PC = 0;
					CPUDataReady = 0;
					
					// If the data that we receive says disconnect, do so
					if (receivedData0[8] == 0x02) no_PC = 1;
					
					// Load the data into dataPacket
					dataPacket[0] = 0xFF;	// The Unique Header for Flight Control
					dataPacket[1] = 0xFE;
					dataPacket[2] = receivedData0[0];	// The Joystick data: Thrust, Yaw, Pitch, Roll
					dataPacket[3] = receivedData0[1];
					dataPacket[4] = receivedData0[2];
					dataPacket[5] = receivedData0[3];
					dataPacket[6] = receivedData0[4];
					dataPacket[7] = receivedData0[5];
					dataPacket[8] = receivedData0[6];
					dataPacket[9] = receivedData0[7];
					dataPacket[10] = 0xF7;	// The Unique Tail
					dataPacket[11] = 0x12;
					
					// Wait for the Quad to ask for data 
					if (receivedData1 == 0xFF)
					{
						receivedData1 = 0;	
						transmitUART1(dataPacket, 12);
					}
					
					// Merge the data to decide where to move the Quad
					mergedData[0] = (receivedData0[0] << 8) | receivedData0[1];
					mergedData[1] = (receivedData0[2] << 8) | receivedData0[3];
					mergedData[2] = (receivedData0[4] << 8) | receivedData0[5];
					mergedData[3] = (receivedData0[6] << 8) | receivedData0[7];
					
				// Else if we are not connected to PC
				}else if (no_PC == 1)
				{
					// Reset the Roll, Pitch, and Yaw. Keep the thrust as is
					dataPacket[0] = 0xFF;	// The Unique Header for Flight Control
					dataPacket[1] = 0xFE;
					dataPacket[2] = potentiometerBufferB[0]	>> 8;	// The Joystick data: Thrust, Yaw, Pitch, Roll
					dataPacket[3] = potentiometerBufferB[0]	& 0xFF;
					dataPacket[4] = 0x01;
					dataPacket[5] = 0xF4;
					dataPacket[6] = 0x01;
					dataPacket[7] = 0xF4;
					dataPacket[8] = 0x01;
					dataPacket[9] =	0xF4;
					dataPacket[10] = 0xF7;	// The Unique Tail
					dataPacket[11] = 0x12;
					
					// Wait for the Quad to ask for data
					if (receivedData1 != 0xFF)
					{
						receivedData1 = 0;
						transmitUART1(dataPacket, 12);
					}										
				}
				break;
				// End of PC mode
				
				
				case 3:			// Trimmer
				
				if (bit_is_clear(PINB, 2))		// Switch between Roll, Pitch, , Thrust, Pitch&Roll and Yaw ---- Red Button
				{
					if (pressed[2] == 0)
					{
						pressed[2] = 1;
						trimRPY++;
						if (trimRPY == 4)	trimmedVariable = 0;	// Because it was set to 18 in the previous step
						if (trimRPY == 5)	trimRPY			= 0;
					}
				}else
				{
					pressed[2] = 0;
				}
				// End of Red Button!
				
				if (bit_is_clear(PINB, 3))		// Switch between the 5 trimmed variables in Roll, Yaw, or Pitch ---- White Button
				{
					if (pressed[3] == 0)
					{
						pressed[3] = 1;
						trimmedVariable++;
						if (trimmedVariable == 6) trimmedVariable = 0;
					}
				}else
				{
					pressed[3] = 0;
				}
				// End of White Button!
				
				// Sends data depending on what axis is being tuned
				switch (trimRPY)
				{
					case 0: 
					temp = 0;
					send_trim_packet(0);
					break;
					
					case 1: 
					temp = 6;
					send_trim_packet(1);
					break;
					
					case 2: 
					temp = 12;
					send_trim_packet(2);
					break;
					
					case 3: 
					trimmedVariable = 18;
					send_trim_packet(3);
					break;
					
					case 4: 
					temp = 0;
					
					send_trim_packet(0);
					_delay_ms(20);
					
					for (uint8_t i=0; i<6; i++) trim[i+6] = trim[i];
					
					send_trim_packet(1);
					_delay_ms(20);
					break;
				}
				break;
				// End of Trimming Mode
				
			}
			// End of Data Code!!!
			
//////////////////////////////////////////////////////////////////////////////////////////////////
			
			// Print Data on the LCD depending on the mode: Joystick | Telekinesis | PC | Trimming
			switch(mode)
			{		
				case 0:		// Joysticks			
				// Print Thrust, Roll, Pitch, and Yaw
				
				gotoMrLCDLocation(0, 0);
				send_A_String("P = ");
				gotoMrLCDLocation(0, 4);
				itoa(potentiometerBufferB[2], BTD, 10);
				send_A_String(BTD);
				send_A_String("  ");

				gotoMrLCDLocation(0, 8);
				send_A_String("T = ");
				gotoMrLCDLocation(0, 11);
				itoa((potentiometerBufferB[0] - 500), BTD, 10);
				send_A_String(BTD);
				send_A_String("   ");

				gotoMrLCDLocation(1, 0);
				send_A_String("R = ");
				gotoMrLCDLocation(1, 4);
				itoa(potentiometerBufferB[3], BTD, 10);
				send_A_String(BTD);
				send_A_String("  ");

				gotoMrLCDLocation(1, 8);
				send_A_String("Y = ");
				gotoMrLCDLocation(1, 11);
				itoa((potentiometerBufferB[1] - 500), BTD, 10);
				send_A_String(BTD);
				send_A_String("   ");
				
				gotoMrLCDLocation(1, 15);
				itoa(environment, BTD, 10);
				send_A_String(BTD);
				
				break;
				// End Mode 1
				
				
				case 1:		// MPU

				// Print Thrust, Roll, Pitch, and Yaw
				gotoMrLCDLocation(0, 0);
				send_A_String("P = ");
				gotoMrLCDLocation(0, 4);
				dtostrf (roll, 3, 1, BTD);
				send_A_String(BTD);
				send_A_String("  ");

				gotoMrLCDLocation(0, 9);
				send_A_String("T = ");
				gotoMrLCDLocation(0, 12);
				itoa((potentiometerBufferB[0] - 500), BTD, 10);
				send_A_String(BTD);
				send_A_String("   ");

				gotoMrLCDLocation(1, 0);
				send_A_String("R = ");
				gotoMrLCDLocation(1, 4);
				dtostrf (pitch, 3, 1, BTD);
				send_A_String(BTD);
				send_A_String("  ");

				gotoMrLCDLocation(1, 9);
				send_A_String("Y = ");
				gotoMrLCDLocation(1, 12);
				itoa((potentiometerBufferB[1] - 500), BTD, 10);
				send_A_String(BTD);
				send_A_String("   ");
				
				gotoMrLCDLocation(1, 15);
				itoa(environment, BTD, 10);
				send_A_String(BTD);
				
				break;
				// End mode 2
				
				
				case 2:		// Computer
				
				// See whether or not we receive data from PC
				if (no_PC == 0)
				{				
					// If data is in navigation mode
					if (receivedData0[8] == 0x01)
					{
						//Clear Screen!
						gotoMrLCDLocation(0, 0);
						send_A_String("               ");
						gotoMrLCDLocation(1, 0);
						send_A_String("               ");
						gotoMrLCDLocation(0, 0);
						
						//Display Navigation steps
						if (mergedData[2] > 555)	// Roll
						{
							send_A_String("Moving Forward");
						} else if (mergedData[2] < 445)
						{
							send_A_String("Moving Backward");
						} else if (mergedData[3] > 555)		// Pitch
						{
							send_A_String("Moving Right");
						} else if (mergedData[3] < 445)
						{
							send_A_String("Moving Left");
						} else
						{
							send_A_String("Idle           ");
						}
						
						// Else if data is in sliders mode
					}else if (receivedData0[8] == 0x00)
					{
						// Print Thrust, Roll, Pitch, and Yaw
						gotoMrLCDLocation(0, 0);
						send_A_String("P = ");
						gotoMrLCDLocation(0, 4);
						itoa(mergedData[2], BTD, 10);
						send_A_String(BTD);
						send_A_String("  ");

						gotoMrLCDLocation(0, 9);
						send_A_String("T = ");
						gotoMrLCDLocation(0, 12);
						itoa(mergedData[0], BTD, 10);
						send_A_String(BTD);
						send_A_String("  ");

						gotoMrLCDLocation(1, 0);
						send_A_String("R = ");
						gotoMrLCDLocation(1, 4);
						itoa(mergedData[3], BTD, 10);
						send_A_String(BTD);
						send_A_String("  ");

						gotoMrLCDLocation(1, 9);
						send_A_String("Y = ");
						gotoMrLCDLocation(1, 12);
						itoa(mergedData[1], BTD, 10);
						send_A_String(BTD);
						send_A_String("  ");
					}
					
				// Else if we are not connected to PC
				}else if (no_PC == 1)
				{
					// Display the corresponding message on the screen
					gotoMrLCDLocation(0, 0);
					send_A_String("PC Control Mode");
					gotoMrLCDLocation(1, 0);
					send_A_String("No data from PC");
				}
				
				break;
				// End of PC mode
							
				case 3:			// Trimmer
				
				// Print what axis we are dealing with in the lower left corner
				gotoMrLCDLocation(1, 13);
				switch (trimRPY)
				{
					case 0:
					send_A_String("  P");
					break;
					
					case 1:
					send_A_String("  R");
					break;
					
					case 2:
					send_A_String("  Y");
					break;
					
					case 3:
					send_A_String("  T");
					break;
					
					case 4:
					send_A_String("R&P");
					break;
				}
				
				// Pack the data into floats to print on screen
				for (uint8_t i=0; i<18; i++) trimF[i] = trim[i]*0.1;
				trimF[5]  = (trim[5] - 500)*0.1;
				trimF[11] = (trim[11] - 500)*0.1;
				trimF[17] = (trim[17] - 500)*0.1;
				
				// Print Pff, Pp, Dp, Ip, and Gyro_p trim as floats
				switch (trimmedVariable)
				{
					case 0:
					gotoMrLCDLocation(0, 0);
					send_A_String("P = ");
					gotoMrLCDLocation(0, 4);
					dtostrf (trimF[0+temp], 3, 1, BTD);
					send_A_String(BTD);
					send_A_String("     ");
					break;
					
					case 1:
					gotoMrLCDLocation(0, 0);
					send_A_String("I = ");
					gotoMrLCDLocation(0, 4);
					dtostrf (trimF[1+temp], 3, 1, BTD);
					send_A_String(BTD);
					send_A_String("     ");
					break;
					
					case 2:
					gotoMrLCDLocation(0, 0);
					send_A_String("D = ");
					gotoMrLCDLocation(0, 4);
					dtostrf (trimF[2+temp], 3, 1, BTD);
					send_A_String(BTD);
					send_A_String("     ");
					break;
					
					case 3:
					gotoMrLCDLocation(0, 0);
					send_A_String("PFF = ");
					gotoMrLCDLocation(0, 5);
					dtostrf (trimF[3+temp], 3, 1, BTD);
					send_A_String(BTD);
					send_A_String("     ");
					break;
					
					case 4:
					gotoMrLCDLocation(0, 0);
					send_A_String("Gyro = ");
					gotoMrLCDLocation(0, 7);
					dtostrf (trimF[4+temp], 3, 1, BTD);
					send_A_String(BTD);
					send_A_String("     ");
					break;
					
					case 5:
					gotoMrLCDLocation(0, 0);
					send_A_String("Offset = ");
					gotoMrLCDLocation(0, 9);
					dtostrf (trimF[5+temp], 3, 1, BTD);
					send_A_String(BTD);
					send_A_String("     ");
					break;
					
					case 18:
					gotoMrLCDLocation(0, 0);
					send_A_String("Thrust = ");
					gotoMrLCDLocation(0, 9);
					itoa(potentiometerBufferB[0], BTD, 10);
					send_A_String(BTD);
					send_A_String("     ");
					break;
				}
				break;
				// End of Trimming Mode
			}
			// End of Printing Code!!!
			
//////////////////////////////////////////////////////////////////////////////////////

		} // End the Quad is running check
	} // End of infinite loop	
} // End of main


//Interrupt service routines
//////////////////////////////////////////////////////////////////////////////////////
ISR (ADC_vect)
{
	countADC++;  // Do 16 ADCs 4 for each potentiometer
	uint16_t lowPart = 0;
	
	if (countADC == 1)
	{
		for (uint8_t i=0; i < 4; i++) potentiometer[i] = 0;  // Clear potentiometers
	}
	
	lowPart = ADCL;	// Must read this before reading ADCH
	
	switch (ADMUX)
	{
		case 0x40:
		potentiometer[0] += (ADCH << 8 | lowPart);
		ADMUX             = 0x41;
		break;
		
		case 0x41:
		potentiometer[1] += (ADCH << 8 | lowPart);
		ADMUX             = 0x42;
		break;
		
		case 0x42:
		potentiometer[2] += (ADCH << 8 | lowPart);
		ADMUX             = 0x43;
		break;
		
		case 0x43:
		potentiometer[3] += (ADCH << 8 | lowPart);
		ADMUX			  = 0x40;
		break;
		
		default:
		break;
	}
	
	if (countADC == 16)
	{
		QuadDisconnectedcount++;
		countADC = 0;
		for (uint8_t i= 0; i<4; i++) potentiometerBuffer[i] = potentiometer[i] >> 2;  // When 16 samples have been gathered, divide by 4 and reset counter
		if (pBiasFlag == 0)
		{
			pBiasFlag = 1;
			for (uint8_t i=0; i<4; i++) pBias[i] = potentiometerBuffer[i];
		}
	}else
	{
		ADCSRA |= 1 << ADSC; // Restart ADC until we have 16 samples
	}
}

ISR (INT5_vect) {
	uint8_t index = 0;
	switch (trimRPY)
	{
		case 0:
		index = trimmedVariable;
		break;
		
		case 4:
		index = trimmedVariable;
		break;
		
		case 1:
		index = trimmedVariable + 6;
		break;
		
		case 2:
		index = trimmedVariable + 12;
		break;
		
		case 3:
		index = 18;
		break;		
	}
	
	if (bit_is_clear(PINE, 5))
	{
		if (bit_is_clear(PINE, 6))
		{
			trim[index]++;
		}else
		{
			trim[index]--;
		}
	}else
	{
		if (bit_is_clear(PINE, 6))
		{
			trim[index]--;
		}else
		{
			trim[index]++;
		}
	}
	
	if (trim[index] < 0)	// make sure it doesn't underflow
	{
		trim[index] = 0;
	}
}

ISR (INT6_vect) {
	uint8_t index = 0;
	switch (trimRPY)
	{
		case 0:
		index = trimmedVariable;
		break;
		
		case 4:
		index = trimmedVariable;
		break;
		
		case 1:
		index = trimmedVariable + 6;
		break;
		
		case 2:
		index = trimmedVariable + 12;
		break;
		
		case 3:
		index = 18;
		break;
	}
		
	if (bit_is_clear(PINE, 6))
	{
		if (bit_is_clear(PINE, 5))
		{
			trim[index]--;
		}else
		{
			trim[index]++;
		}
	}else
	{
		if (bit_is_clear(PINE, 5))
		{
			trim[index]++;
		}else
		{
			trim[index]--;
		}
	}
	
	if (trim[index] < 0)	// make sure it doesn't underflow
	{
		trim[index] = 0;
	}
}


// Function implementations
//////////////////////////////////////////////////////////////////////
void init_LEDs()
{
	DDRB |= 0x70;
	DDRE |= 0x08;
}

void init_buttons()
{
	DDRB &= 0xF0;		// Set button pin direction to 0
	DDRA &= ~(0x03);
	PORTB|= 0x0F;		// Set button input high
	PORTA|= 0x03;
}

void init_ADC()
{
	DDRF &= 0xF0;		// Which pins did you use for ADC
	ADCSRA |=  1 << ADIE | 1 << ADEN | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0; // IE | ADC EN | pre-scaler 128 => 125 kHz @ FCPU 16 MHz
	ADMUX  |=  1 << REFS0;	// Voltage reference reference source AVCC | Data out Reg set
}

void init_encoder()
{
	EICRB |=  1 << ISC50 | 1 << ISC60 ;
	EIMSK |=  1 << INT5  | 1 << INT6  ;
}


void send_initialization_data()
{
	uint8_t dataPacket[18] = {0};
	
	dataPacket[0] = 0xFD;	// The Unique Header
	dataPacket[1] = 0xFC;
	dataPacket[16] = 0xF6;	// The Unique Tail
	dataPacket[17] = 0x12;
	
	dataPacket[2]  = trim[0] >> 8;		// The trimming data: Pitch, Roll, Yaw
	dataPacket[3]  = trim[0] & 0xFF;
	dataPacket[4]  = trim[1] >> 8;
	dataPacket[5]  = trim[1] & 0xFF;
	dataPacket[6]  = trim[2] >> 8;
	dataPacket[7]  = trim[2] & 0xFF;
	dataPacket[8]  = trim[3] >> 8;
	dataPacket[9]  = trim[3] & 0xFF;
	dataPacket[10] = trim[4] >> 8;
	dataPacket[11] = trim[4] & 0xFF;
	dataPacket[12] = trim[18] >> 8;
	dataPacket[13] = trim[18] & 0xFF;
	dataPacket[14] = trim[5] >> 8;
	dataPacket[15] = trim[5] & 0xFF;
	
	
	while (receivedData1 != 0xFF)
	
	receivedData1 = 0;	
	transmitUART1(dataPacket, 18);
	_delay_ms(50);
	
	
	dataPacket[0] = 0xFB;	// The Unique Header
	dataPacket[1] = 0xFA;
	dataPacket[16] = 0xF5;	// The Unique Tail
	dataPacket[17] = 0x12;
	
	dataPacket[2]  = trim[6] >> 8;		// The trimming data: Pitch, Roll, Yaw
	dataPacket[3]  = trim[6] & 0xFF;
	dataPacket[4]  = trim[7] >> 8;
	dataPacket[5]  = trim[7] & 0xFF;
	dataPacket[6]  = trim[8] >> 8;
	dataPacket[7]  = trim[8] & 0xFF;
	dataPacket[8]  = trim[9] >> 8;
	dataPacket[9]  = trim[9] & 0xFF;
	dataPacket[10] = trim[10] >> 8;
	dataPacket[11] = trim[10] & 0xFF;
	dataPacket[12] = trim[18] >> 8;
	dataPacket[13] = trim[18] & 0xFF;
	dataPacket[14] = trim[11] >> 8;
	dataPacket[15] = trim[11] & 0xFF;
	
	while (receivedData1 != 0xFF)
	
	receivedData1 = 0;
	transmitUART1(dataPacket, 18);
	_delay_ms(50);
	
	
	dataPacket[0] = 0xF9;	// The Unique Header
	dataPacket[1] = 0xF8;
	dataPacket[16] = 0xF4;	// The Unique Tail
	dataPacket[17] = 0x12;
	
	dataPacket[2]  = trim[12] >> 8;		// The trimming data: Pitch, Roll, Yaw
	dataPacket[3]  = trim[12] & 0xFF;
	dataPacket[4]  = trim[13] >> 8;
	dataPacket[5]  = trim[13] & 0xFF;
	dataPacket[6]  = trim[14] >> 8;
	dataPacket[7]  = trim[14] & 0xFF;
	dataPacket[8]  = trim[15] >> 8;
	dataPacket[9]  = trim[15] & 0xFF;
	dataPacket[10] = trim[16] >> 8;
	dataPacket[11] = trim[16] & 0xFF;
	dataPacket[12] = trim[18] >> 8;
	dataPacket[13] = trim[18] & 0xFF;
	dataPacket[14] = trim[17] >> 8;
	dataPacket[15] = trim[17] & 0xFF;
	
	
	while (receivedData1 != 0xFF)
		
	receivedData1 = 0;	
	transmitUART1(dataPacket, 18);
	_delay_ms(50);
}

void send_trim_packet(uint8_t Variable)
{
	uint8_t dataPacket[18] = {0};
	uint8_t temp = 0;
	
	switch(Variable)
	{
		case 0:
		temp = 0;
		dataPacket[0] = 0xFD;	// The Unique Header Pitch
		dataPacket[1] = 0xFC;
		dataPacket[16] = 0xF6;	// The Unique Tail
		dataPacket[17] = 0x12;
		break;
		
		case 1:
		temp = 6;
		dataPacket[0] = 0xFB;	// The Unique Header Roll
		dataPacket[1] = 0xFA;
		dataPacket[16] = 0xF5;	// The Unique Tail
		dataPacket[17] = 0x12;
		break;
		
		case 2:
		temp = 12;
		dataPacket[0] = 0xF9;	// The Unique Header Yaw
		dataPacket[1] = 0xF8;
		dataPacket[16] = 0xF4;	// The Unique Tail
		dataPacket[17] = 0x12;
		break;
		
		case 3:
		trimmedVariable = 18;
		dataPacket[0] = 0xFD;	// The Unique Header Pitch
		dataPacket[1] = 0xFC;
		dataPacket[16] = 0xF6;	// The Unique Tail
		dataPacket[17] = 0x12;
		
		case 4: // Invalid
		temp = 0;
	}
	
	dataPacket[2]  = trim[0+temp] >> 8;		// The trimming data: Pitch, Roll, Yaw
	dataPacket[3]  = trim[0+temp] & 0xFF;
	dataPacket[4]  = trim[1+temp] >> 8;
	dataPacket[5]  = trim[1+temp] & 0xFF;
	dataPacket[6]  = trim[2+temp] >> 8;
	dataPacket[7]  = trim[2+temp] & 0xFF;
	dataPacket[8]  = trim[3+temp] >> 8;
	dataPacket[9]  = trim[3+temp] & 0xFF;
	dataPacket[10] = trim[4+temp] >> 8;
	dataPacket[11] = trim[4+temp] & 0xFF;
	dataPacket[12] = trim[18] >> 8;
	dataPacket[13] = trim[18] & 0xFF;
	dataPacket[14] = trim[5+temp] >> 8;
	dataPacket[15] = trim[5+temp] & 0xFF;
	
	if (receivedData1 == 0xFF)
	{
		receivedData1 = 0;		
		transmitUART1(dataPacket, 18);
	}
}
//////////////////////////////////////////////////////////////////////

//The potentiometers have different initial values and they are not centered at 0. These functions transform the data into the desired shape
//Output = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

unsigned long thrust_range_transform(unsigned long input, uint8_t environment)
{
	uint16_t range = THRUST_RANGE_OUTDOORS;
	unsigned long output;
	
	if (environment == 1) range = THRUST_RANGE_INDOORS;
	
	if (input > (pBias[0] + 10))
	{
		output = ((input - pBias[0])*(range)/(1024 - pBias[0])) + trim[18];
	}else if (input < (pBias[0] - 10))
	{
		output = (input*(range)/pBias[0]) + trim[18] - range;
	}else
	{
		output = trim[18];
	}
	
	return output;
}

unsigned long RandP_range_transform(unsigned long input, uint8_t index)
{
	unsigned long output;
	if (input > (pBias[index] + 10))
	{
		output = ((input - pBias[index])*(499)/(1024 - pBias[index])) + 501;  // in_min = trim[18], in_max = 1024, out_min = 501,  out_max = 1000
	}else if (input < (pBias[index] - 10))
	{
		output = (input*(499)/pBias[index]);		// in_min = 0, in_max = trim[18], out_min = 0,  out_max = 499
	}else
	{
		output = 500;
	}
	
	return output;
}

//Hold and display a message while waiting for data from the quad
void waiting_for_quad()
{
	//Clear Screen!
	gotoMrLCDLocation(0, 0);
	send_A_String("               ");
	gotoMrLCDLocation(1, 0);
	send_A_String("               ");
	gotoMrLCDLocation(0, 0);
	
	while (receivedData1 != 0xFF)
	{
		
		gotoMrLCDLocation(0, 0);
		send_A_String("Waiting for ");
		gotoMrLCDLocation(1, 0);
		send_A_String("Quad.        ");
		_delay_ms(300);
		gotoMrLCDLocation(1, 0);
		send_A_String("Quad..       ");
		_delay_ms(300);
		gotoMrLCDLocation(1, 0);
		send_A_String("Quad...      ");
		_delay_ms(300);
	}
	
	// send the quad initialization!!
	send_initialization_data();
	
	//Display "Quad Connected"
	gotoMrLCDLocation(0, 0);
	send_A_String("Quad        ");
	gotoMrLCDLocation(1, 0);
	send_A_String("Connected       ");
	
	// Just for the Quad connected message to last for a while
	_delay_ms(2000);
	
	//Clear Screen!
	gotoMrLCDLocation(0, 0);
	send_A_String("               ");
	gotoMrLCDLocation(1, 0);
	send_A_String("               ");
	gotoMrLCDLocation(0, 0);
	
}