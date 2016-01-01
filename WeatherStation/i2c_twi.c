/*
 * i2c_twi.c
 *
 * Created: 2015-12-29 20:39:12
 *  Author: tszafran
 */ 
#include <avr/io.h>
#include "i2c_twi.h"

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
//--------//--------//--------//--------//--------//--------//--------//--------//--------
void i2cSetBitrate(uint8_t SCL_CLOCK)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;									/* no prescaler */
  TWBR = ((F_CPU/(SCL_CLOCK*1000))-16)/2;  /* must be > 10 for stable operation */
}/* i2c_init */


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void TWI_start(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA);
	
	while (!(TWCR & (1<<TWINT)));
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void TWI_stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	while (!(TWCR & (1<<TWSTO)));
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void TWI_write(uint8_t bajt)
{
	TWDR = bajt;
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	while (!(TWCR & (1<<TWINT)));
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
uint8_t TWI_read(uint8_t ack)
{
	TWCR = (1<<TWINT) | (ack<<TWEA) | (1<<TWEN);
	
	while (!(TWCR & (1<<TWINT)));
	
	return TWDR;
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void TWI_write_buf(uint8_t SLA, uint8_t adr, uint8_t len, uint8_t* buf)
{
	TWI_start();
	TWI_write(SLA);
	TWI_write(adr);
	while (len--) TWI_write(*buf++);
	TWI_stop();
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void TWI_read_buf(uint8_t SLA, uint8_t adr, uint8_t len, uint8_t* buf)
{
	TWI_start();
	TWI_write(SLA);
	TWI_write(adr);
	TWI_start();
	TWI_write(SLA + 1);
	while (len--) *buf++ = TWI_read(len ? ACK : NACK);
	TWI_stop();
}
