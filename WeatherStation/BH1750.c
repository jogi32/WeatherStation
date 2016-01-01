/*
 * BH1750.c
 *
 * Created: 2016-01-01 22:12:54
 *  Author: tszafran
 */ 

#include "BH1750.h"
#include "i2c_twi.h"

//--------//--------//--------//--------//--------//--------//--------//--------//--------
void BH1750_init (void)
{
	//funkcja inicjalizacji pracy czujnika BH1750
	
	TWI_start();
	TWI_write(BH1750_ADR);
	TWI_write(BH1750_POWER_ON);
	TWI_write(BH1750_RESET);
	TWI_stop();
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void BH1750_read(uint8_t Adress, uint8_t Mode, uint8_t Length, uint8_t *buf)
{
	//funkcja odczytu danych z czujnika BH1750
	TWI_start();
	TWI_write(Adress);
	TWI_write(Mode);
	TWI_start();
	TWI_write(Adress +1);
	while (Length--) *buf++ = TWI_read( Length ? ACK : NACK );
	TWI_stop();
}