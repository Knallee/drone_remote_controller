/*
 * TWI_master.c
 *
 * Created: 2016-03-09 14:28:06
 *  Author: et05cc8
 */ 

#include "TWI_master.h"
#include "..\mpu6050\mpu6050.h"
#include <avr/io.h>
#include <avr/interrupt.h>

uint8_t TWI_buf[TWI_BUFFER_SIZE];     // Transceiver buffer
uint8_t TWI_msgSize;                  // Number of bytes to be transmitted.
uint8_t TWI_state = TWI_NO_STATE;     // State byte. Default set to TWI_NO_STATE.
uint8_t readFlag = 0;
uint8_t MPUFlag = 0;

union TWI_statusReg_f TWI_statusReg = {0};         // TWI_statusReg is defined in TWI_master.h

/****************************************************************************
Call this function to set up the TWI master to its initial standby state.
Remember to enable interrupts from the main application after initializing the TWI.
****************************************************************************/
void TWI_Master_Initialise(void)
{
	TWBR = TWI_TWBR;                                // Set bit rate register (Baudrate). Defined in header file.
	TWSR = TWI_TWPS;                               // Set prescaler
	TWDR = 0xFF;                                    // Default content = SDA released.
	TWCR = (1<<TWEN)|                               // Enable TWI-interface and release TWI pins.
	(0<<TWIE)|(0<<TWINT)|                    // Disable Interrupt.
	(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|         // No Signal requests.
	(0<<TWWC);
}

/****************************************************************************
Call this function to test if the TWI_ISR is busy transmitting.
****************************************************************************/
uint8_t TWI_Transceiver_Busy(void)
{
	return TWCR & (1<<TWIE);                     // IF TWI Interrupt is enabled then the Transceiver is busy
}

/****************************************************************************
Call this function to fetch the state information of the previous operation. The function will hold execution (loop)
until the TWI_ISR has completed with the previous operation. If there was an error, then the function
will return the TWI State code.
****************************************************************************/
uint8_t TWI_Get_State_Info(void)
{
	while(TWI_Transceiver_Busy());               // Wait until TWI has completed the transmission.
	return TWI_state;                            // Return error state.
}

/****************************************************************************
Call this function to send a prepared message. The first byte must contain the slave address and the
read/write bit. Consecutive bytes contain the data to be sent, or empty locations for data to be read
from the slave. Also include how many bytes that should be sent/read including the address byte.
The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
then initialize the next operation and return.
****************************************************************************/
void TWI_Start_Transceiver_With_Data(uint8_t regAddr, uint8_t *msg, uint8_t msgSize)
{
	uint8_t i;
	uint8_t realMsgSize = 2;

	while(TWI_Transceiver_Busy());                // Wait until TWI is ready for next transmission.


	TWI_buf[0]  = MPU6050_ADDR | I2C_WRITE;       // Store slave address with R/W setting.
	TWI_buf[1]  = regAddr;						  // Store register address
	if (readFlag == 1)
	{
		TWI_buf[2]  = MPU6050_ADDR | I2C_READ;    // Store slave address with R/W setting.
		realMsgSize	= 3;
	}
	
	TWI_msgSize = msgSize + realMsgSize;          // Number of data to transmit.
	
	for(i = 0; i < msgSize; i ++) TWI_buf[i + realMsgSize] = msg[i];  // Copy data.

	TWI_statusReg.all = 0;
	TWI_state = TWI_NO_STATE;
	TWCR = (1<<TWEN)|                      // TWI Interface enabled.
	(1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
	(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
	(0<<TWWC);
}

/****************************************************************************
Call this function to resend the last message. The driver will reuse the data previously put in the transceiver buffers.
The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
then initialize the next operation and return.
****************************************************************************/
void TWI_Start_Transceiver(void)
{
	while(TWI_Transceiver_Busy());                // Wait until TWI is ready for next transmission.
	TWI_statusReg.all = 0;
	TWI_state = TWI_NO_STATE;
	TWCR = (1<<TWEN)|                             // TWI Interface enabled.
	(1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
	(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
	(0<<TWWC);
}

/****************************************************************************
Call this function to read out the requested data from the TWI transceiver buffer. I.e. first call
TWI_Start_Transceiver to send a request for data to the slave. Then Run this function to collect the
data when they have arrived. Include a pointer to where to place the data and the number of bytes
requested (including the address field) in the function call. The function will hold execution (loop)
until the TWI_ISR has completed with the previous operation, before reading out the data and returning.
If there was an error in the previous transmission the function will return the TWI error code.
****************************************************************************/
uint8_t TWI_Get_Data_From_Transceiver(uint8_t *msg, uint8_t msgSize)
{
	uint8_t i;
	while(TWI_Transceiver_Busy());               // Wait until TWI is ready for next transmission.

	if(TWI_statusReg.lastTransOK)                // Last transmission competed successfully.
	{
		for(i = 0; i < msgSize; i += 1)           // Copy data from Transceiver buffer.
		msg[i] = TWI_buf[i+3];
	}
	return TWI_statusReg.lastTransOK;
}


// ********** Interrupt Handlers ********** //
/****************************************************************************
This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
that is whenever a TWI event has occurred. This function should not be called directly from the main
application.
****************************************************************************/
ISR(TWI_vect)
{
	static uint8_t TWI_bufPtr;
	switch(TWSR)
	{
		case TWI_START:             // START has been transmitted
		TWI_bufPtr = 0;          // Set buffer pointer to the TWI Address location
		case TWI_REP_START:         // Repeated START has been transmitted
		
		case TWI_MTX_ADR_ACK:       // SLA+W has been transmitted and ACK received
		case TWI_MTX_DATA_ACK:      // Data byte has been transmitted and ACK received
		if((TWI_bufPtr < TWI_msgSize) && (TWI_buf[TWI_bufPtr] == (MPU6050_ADDR | I2C_READ)) && (readFlag == 1))
		{
			readFlag = 0;
			TWCR = (1<<TWEN)|                             // TWI Interface enabled.
			(1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
			(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
			(0<<TWWC);
		}
		else if (TWI_bufPtr < TWI_msgSize)
		{
			TWDR = TWI_buf[TWI_bufPtr++];
			TWCR = (1<<TWEN)|                          // TWI Interface enabled
			(1<<TWIE)|(1<<TWINT)|               // Enable TWI Interrupt and clear the flag to send byte
			(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
			(0<<TWWC);
		}
		else                    // Send STOP after last byte
		{
			TWI_statusReg.lastTransOK = TRUE;          // Set status bits to completed successfully.
			TWCR = (1<<TWEN)|                          // TWI Interface enabled
			(0<<TWIE)|(1<<TWINT)|               // Disable TWI Interrupt and clear the flag
			(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|    // Initiate a STOP condition.
			(0<<TWWC);
		
		}
		break;
		case TWI_MRX_DATA_ACK:      // Data byte has been received and ACK transmitted
		TWI_buf[TWI_bufPtr++] = TWDR;
		case TWI_MRX_ADR_ACK:       // SLA+R has been transmitted and ACK received
		if(TWI_bufPtr < (TWI_msgSize-1))              // Detect the last byte to NACK it.
		{
			TWCR = (1<<TWEN)|                          // TWI Interface enabled
			(1<<TWIE)|(1<<TWINT)|               // Enable TWI Interrupt and clear the flag to read next byte
			(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|    // Send ACK after reception
			(0<<TWWC);
		}
		else                     // Send NACK after next reception
		{
			TWCR = (1<<TWEN)|                          // TWI Interface enabled
			(1<<TWIE)|(1<<TWINT)|                   // Enable TWI Interrupt and clear the flag to read next byte
			(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|        // Send NACK after reception
			(0<<TWWC);
		}
		break;
		case TWI_MRX_DATA_NACK:     // Data byte has been received and NACK transmitted
		TWI_buf[TWI_bufPtr] = TWDR;
		TWI_statusReg.lastTransOK = TRUE;             // Set status bits to completed successfully.
		TWCR = (1<<TWEN)|                             // TWI Interface enabled
		(0<<TWIE)|(1<<TWINT)|                  // Disable TWI Interrupt and clear the flag
		(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|       // Initiate a STOP condition.
		(0<<TWWC);
		
		if (MPUFlag == 1) 
		{
			TWI_Get_Data_From_Transceiver(mpu6050_fifoBuffer, 16);
			uint8_t temp = 0b11101100;
			TWI_Start_Transceiver_With_Data(MPU6050_RA_USER_CTRL, &temp, 1);	// Reset FIFO
			MPUFlag = 2;
			ADCSRA |= 1 << ADSC; // Start the ADC every 50ms
			
			//	You can use a counter to make the ADC run even slower than that
		}
		
		break;
		case TWI_ARB_LOST:          // Arbitration lost
		TWCR = (1<<TWEN)|                             // TWI Interface enabled
		(1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag
		(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a (RE)START condition.
		(0<<TWWC);
		break;
		case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
		case TWI_MTX_ADR_NACK:      // SLA+W has been transmitted and NACK received
		case TWI_MRX_ADR_NACK:      // SLA+R has been transmitted and NACK received
		case TWI_MTX_DATA_NACK:     // Data byte has been transmitted and NACK received
		//      case TWI_NO_STATE           // No relevant state information available; TWINT = '0'
		default:
		TWI_state = TWSR;                              // Store TWSR and automatically clears no Errors bit.
		// Reset TWI Interface
		TWCR = (1<<TWEN)|                              // Enable TWI-interface and release TWI pins
		(0<<TWIE)|(1<<TWINT)|                   // Disable Interrupt
		(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|        // Initiate a STOP condition.
		(0<<TWWC);
	}
}