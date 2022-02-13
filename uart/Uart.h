/*
 * Uart.h
 *
 * Created: 2016-05-13 18:27:53
 *  Author: et05cc8
 */ 


#ifndef UART_H_
#define UART_H_

#include <avr/io.h>
#include <avr/interrupt.h>

void transmitUART1(uint8_t* data, uint8_t dataSize);
void transmitUART0(uint8_t* data, uint8_t dataSize);
void init_uart(int baud, char dataSizeInBits, char parityModeEVENorODD, char stopBits);

extern volatile uint8_t receivedData0[9];
extern volatile uint8_t receivedData1;
extern volatile uint8_t CPUDataReady;
extern volatile uint8_t dataCountT;
extern volatile uint8_t size;
extern volatile uint8_t dataBuffer[20];

extern volatile uint16_t	QuadDisconnectedcount;


#endif /* UART_H_ */