/*
 * Uart.c
 *
 * Created: 2016-05-13 18:28:03
 *  Author: et05cc8
 */ 

#include "Uart.h"

volatile uint8_t receivedData0[9] = {0};
volatile uint8_t receivedData1 = 0;
volatile uint8_t dataCount = 0;
volatile uint8_t CPUDataReady = 0;
volatile uint8_t dataCountT = 0;
volatile uint8_t size = 0;
volatile uint8_t dataBuffer[20] = {0};
	
volatile uint16_t	QuadDisconnectedcount	= 0;

// Receiving 8 bytes from PC
ISR(USART0_RX_vect) {
	receivedData0[dataCount] = UDR0;
	dataCount ++;
	if (dataCount == 9) 
	{
		CPUDataReady = 1;
		dataCount = 0;
	}
}

ISR(USART0_TX_vect) {
	if (size > dataCountT)
	{
		UDR0 = dataBuffer[dataCountT];
		dataCountT++;
		if (dataCountT == size)
		{
			dataCountT = 0;
			size = 0;
		}
	}
}

ISR(USART1_RX_vect) {
	receivedData1 = UDR1;
	QuadDisconnectedcount = 0;
	//dataCount ++;
	//if (dataCount == 9) 
	//{
		//CPUDataReady = 1;
		//dataCount = 0;
	//}
}


ISR(USART1_TX_vect) {
	if (size > dataCountT)
	{
		UDR1 = dataBuffer[dataCountT];
		dataCountT++;
		if (dataCountT == size)
		{
			dataCountT = 0;
			size = 0;
		}
	}
}

void transmitUART1(uint8_t* data, uint8_t dataSize) {
	//Wait until the Transmitter is ready
	while (! (UCSR1A & (1 << UDRE1)) );
	//Send data
	if (dataSize > 0)
	{
		size = dataSize;
		dataCountT = 0;
		for (uint8_t i=0; i<dataSize; i++) dataBuffer[i] = data[i];
		UDR1 = dataBuffer[dataCountT];
		dataCountT++;
	}
}

void transmitUART0(uint8_t* data, uint8_t dataSize) {
	//Wait until the Transmitter is ready
	while (! (UCSR0A & (1 << UDRE0)) );
	//Send data
	if (dataSize > 0)
	{
		size = dataSize;
		dataCountT = 0;
		for (uint8_t i=0; i<dataSize; i++) dataBuffer[i] = data[i];
		UDR0 = dataBuffer[dataCountT];
		dataCountT++;
	}
}


void init_uart(int baud, char dataSizeInBits, char parityModeEVENorODD, char stopBits) {
	
	//int UBRR_value = 16;		//57300 Baud @ 16 MHz
	//int UBRR_value = 8;		//115200 Baud @ 16 MHz
	int UBRR_value = 103;	// 9600 Baud @ 16 MHz
	//int UBRR_value = 51;		// 9600 Baud @ 8 MHz
	
	UBRR0H = (unsigned char) (UBRR_value >> 8); // convert the int to char, first five bits are assigned to other stuff
	UBRR1H = (unsigned char) (UBRR_value >> 8); // convert the int to char, first five bits are assigned to other stuff
	
	UBRR0L = (unsigned char) UBRR_value;
	UBRR1L = (unsigned char) UBRR_value;
	
	UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable both receiver and transmitter. Note that the TX- and RX-pins are now no longer IO-pins.
	UCSR1B = (1 << RXEN1) | (1 << TXEN1); // Enable both receiver and transmitter. Note that the TX- and RX-pins are now no longer IO-pins.
	
	UCSR0C |= (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ01);	// Stop bits (two of them) and length (8 bit)
	UCSR1C |= (1 << USBS1) | (1 << UCSZ11) | (1 << UCSZ11);	// Stop bits (two of them) and length (8 bit)
	
	UCSR0B |= (1 << RXCIE0) | (1 << TXCIE0);	// Enable interrupts!
	UCSR1B |= (1 << TXCIE1) | (1 << RXCIE1);
	
	UCSR0C |= (1 << USBS0);
	UCSR1C |= (1 << USBS1);
	
	if (dataSizeInBits == 8) UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data length
	if (dataSizeInBits == 8) UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10); // 8-bit data length
	
	
	//if (dataSizeInBits == 5) {
		//UCSR0A |= (0 << UCSZ02);
		//UCSR0C |= (0 << UCSZ01) | (0 << UCSZ00); // 5-bit data length
	//}
	
	//if (dataSizeInBits == 6) UCSR0C |= (0 << UCSZ01) | (1 << UCSZ00); // 6-bit data length
	//if (dataSizeInBits == 7) UCSR0C |= (1 << UCSZ01) | (0 << UCSZ00); // 7-bit data length
	//if (dataSizeInBits == 9) {
		//UCSR0A |= (1 << UCSZ02);
		//UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // 9-bit data length
	//}
	//

	
	//if (parityModeEVENorODD == EVEN) UCSR0C |= (1 << UPM01); //Sets parity to EVEN
	//if (parityModeEVENorODD == ODD) UCSR0C |= (3 << UPM00)|(1 << UPM01); //Alternative way to set parity to ODD
	
	//if (stopBits == 2) UCSR0C |= (1 << USBS0); //Sets 2 stop bits, 1 stop bit is default
	//if (stopBits == 1) UCSR0C |= (0 << USBS0);
}
