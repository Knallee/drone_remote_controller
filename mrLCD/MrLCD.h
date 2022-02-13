/*
 * MrLCD.h
 *
 * Created: 2016-02-29 22:26:59
 *  Author: et05cc8
 */ 



#ifndef MRLCD_H_
#define MRLCD_H_


#define	MrLCDCrib					PORTC
#define DataDirection_MrLCDCrib		DDRC
#define MrLCDControl				PORTD
#define DataDirection_MrLCDControl	DDRD
#define LightSwitch		7
#define ReadWrite		6
#define BipolarMood		5


unsigned char swap_input(unsigned char b);
void check_If_MrLCD_IsBusy(void);
void peek_A_Boo(void);
void send_A_Command(unsigned char command);
void send_A_Character(unsigned char character);
void send_A_String(char *string);
void gotoMrLCDLocation(uint8_t y, uint8_t x);
void init_LCD();




#endif /* MRLCD_H_ */