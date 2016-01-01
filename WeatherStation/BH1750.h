/*
 * BH1750.h
 *
 * Created: 2016-01-01 22:11:06
 *  Author: tszafran
 */ 


#ifndef BH1750_H_
#define BH1750_H_

#include <inttypes.h>

//adres czujnika BH1750 0x23 przemno¿ony x2 dla trybu 8 bit
#define BH1750_ADR 0x46

// za³¹czenie czujnika
#define BH1750_POWER_ON 0x01

// reset rejestru pomiaru
#define BH1750_RESET 0x07

//tryby pomiaru do wybrania
#define BH1750_ONE_TIME_HIGH_RES_MODE    0x20
#define BH1750_ONE_TIME_HIGH_RES_MODE_2  0x21
#define BH1750_ONE_TIME_LOW_RES_MODE     0x23

void BH1750_init (void);
void BH1750_read (uint8_t Adress, uint8_t Mode, uint8_t Length, uint8_t *buf);

#endif /* BH1750_H_ */