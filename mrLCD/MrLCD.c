/*
 * MrLCD.c
 *
 * Created: 2016-02-29 22:26:49
 *  Author: et05cc8
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "MrLCD.h"

void check_If_MrLCD_IsBusy(void)
{
	DataDirection_MrLCDCrib = 0x00;
	MrLCDControl |= 1 << ReadWrite;
	MrLCDControl &= ~(1 << BipolarMood);
	
	while (MrLCDCrib >= 0x80)
	{
		peek_A_Boo();
	}
	
	DataDirection_MrLCDCrib = 0xFF;
}

void peek_A_Boo(void)
{
	MrLCDControl |= 1 << LightSwitch;
	_delay_us(50);
	MrLCDControl &= ~(1 << LightSwitch);
}

void send_A_Command(uint8_t command)
{
	check_If_MrLCD_IsBusy();
	MrLCDCrib = command;
	MrLCDControl &= ~(1 << ReadWrite | 1 << BipolarMood);
	peek_A_Boo();
	MrLCDCrib = 0;
}

void send_A_Character(unsigned char character)
{
	check_If_MrLCD_IsBusy();
	MrLCDCrib = character;
	MrLCDControl &= ~(1 << ReadWrite);
	MrLCDControl |= 1 << BipolarMood;
	peek_A_Boo();
	MrLCDCrib = 0;
}

void send_A_String(char *string)
{
	while (*string > 0)
	{
		send_A_Character(*string++);
	}
}

void gotoMrLCDLocation(uint8_t y, uint8_t x)
{
	uint8_t firstDigitPositiononMrLCD[4] = {0, 40};
	send_A_Command(0x80 + firstDigitPositiononMrLCD[y] + x);
}

void init_LCD()
{
	DataDirection_MrLCDControl |= 1 << LightSwitch | 1 << ReadWrite | 1 << BipolarMood;
	
	_delay_ms(15);
	
	send_A_Command(0x01);
	_delay_ms(2);
	send_A_Command(0x38);
	_delay_us(50);
	send_A_Command(0b00001110);
	_delay_us(50);
}