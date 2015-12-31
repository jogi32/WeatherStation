/*
 * i2c_twi.h
 *
 * Created: 2015-12-29 20:33:34
 *  Author: tszafran
 */ 


#ifndef I2C_TWI_H_
#define I2C_TWI_H_

#define ACK		1
#define NACK	0

/* I2C clock in Hz */
//#define SCL_CLOCK  100000L


/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2cSetBitrate(uint8_t SCL_CLOCK);
void TWI_start(void);
void TWI_stop(void);
void TWI_write(uint8_t bajt);
uint8_t TWI_read(uint8_t ack);
void TWI_write_buf(uint8_t SLA, uint8_t adr, uint8_t len, uint8_t* buf);
void TWI_read_buf(uint8_t SLA, uint8_t adr, uint8_t len, uint8_t* buf);






#endif /* I2C_TWI_H_ */