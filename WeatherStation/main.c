/*
 * WeatherStation.cpp
 *
 * Created: 2015-12-01
 * Author : Tomasz Szafrañski
 */ 

#define BAUD 9600

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <util/setbaud.h>

#include "BH1750.h"
#include "BMP180.h"
#include "dht.h"
#include "ds18x20.h"
#include "i2c_twi.h"
#include "lcd_displ.h"


#define SPEAKER		(1<<PB3)
#define HUM_EARTH	(1<<PB2)
#define RAIN		(1<<PD2)
#define _18B20		(1<<PD3)
#define DHT22		(1<<PD6)
#define HUM_E_ADC	(0<<MUX2) |(0<<MUX1) | (0<<MUX0)
#define RAIN_ADC	(0<<MUX2) |(0<<MUX1) | (1<<MUX0)
#define NUM_FOR_MES 10
#define GLUE(x, y)  x##y
#define VCC			3.5

void ADC_init();
void ADC_mes();
void ADCReadVaue();
void ADC_RUNTIME(uint32_t* MES);
void BH1750_getData();
void BMP180_getData();
void DS18X20Init();
void DS18X20Read();
void Hello();
void IntToStr();
void LcdDisplayData();
void SecondsRuntime();
void timer0Init();
void USARTMakeDataFrame();
void USART_Init();
void USART_SendText(char* a_Text);
void USART_TransmitByte(uint8_t a_data);

volatile uint8_t s1_flag;
volatile uint8_t second;
volatile uint16_t ms;

uint32_t humEarthValue;
uint32_t rainValue;
uint8_t	 subzero, cel, cel_frac, ds18b;
char     temperatureDS18b20[6];
char     usartData[128];
char     buf[64];

const char CR = 13;

float dthTemperature = 0;
float dthHumidity = 0;

// bufor dla dczytu pomiarów
uint8_t light_buf [2];

uint64_t lux_temp = 0;
uint64_t lux = 0;

long temp, press;

//--------//--------//--------//--------//--------//--------//--------//--------//--------
int main(void)
{
	DDRB  |= (SPEAKER);
	PORTB &= ~SPEAKER;
	
	ADC_init();
	DS18X20Init();
	lcd_init();
	timer0Init();	
	USART_Init();
	
	i2cSetBitrate(100);
	BH1750_init();
	BMP180_init();
	
	_delay_ms(1000);
		
	sei();
	
	Hello();

    while (1) 
    {		
		if (s1_flag)
		{
			BH1750_getData();
			
			BMP180_getData();
			
			SecondsRuntime();
			
			LcdDisplayData();
			
			ADCReadVaue();			
			
			USARTMakeDataFrame();
			
			USART_SendText(usartData);
		}
		
    }
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
ISR(TIMER0_COMP_vect)
{
	if (++ms>99)
	{
		s1_flag = 1;
		second++;
		
		if (second>59)
		{
			second = 0;
		}

		ms = 0;
	}
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void ADC_init()
{
	//Start ADC, external Vcc, one conversion mode, preskaler 128, input PIN0-7, 
	//align to right
	ADCSRA	|=	(1<<ADEN)					//Bit 7 – ADEN: ADC Enable
	|	(1<<ADPS0)
	|	(1<<ADPS1)
	|	(1<<ADPS2);							//ADPS2:0: ADC Prescaler Select Bits 
											//(set prescaler) preskaler= 128

	ADMUX	=	(0<<REFS1) | (1<<REFS0)		//External 3.50V Voltage Reference with
											// external capacitor at AREF pin
	|HUM_E_ADC;								//Input Channel Selections (ADC3 - Pin 3 )
	
	DDRC	&=~	(1<<PA0);					//Set input ADC
	DDRC	&=~	(1<<PA1);					//Set input ADC
	DDRC	&=~ (1<<PA2);					//Set input ADC
	DDRC	&=~ (1<<PA3);					//Set input ADC
	DDRC	&=~ (1<<PA4);					//Set input ADC
	DDRC	&=~ (1<<PA5);					//Set input ADC
	DDRC	&=~ (1<<PA6);					//Set input ADC
	DDRC	&=~ (1<<PA7);					//Set input ADC
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void ADC_mes()
{
	ADCSRA |= (1<<ADSC);		//Bit 6 – ADSC: ADC Start Conversion 
								//(run single conversion)
	while(ADCSRA & (1<<ADSC));	//wait for end of conversion
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void ADCReadVaue()
{
	ADMUX  = (0<<REFS1) | (1<<REFS0)    //External 3.50V Voltage Reference with 
										//external capacitor at AREF pin
	|HUM_E_ADC;
	_delay_ms(10);
	ADC_RUNTIME(&humEarthValue);
	
	ADMUX  = (0<<REFS1) | (1<<REFS0)
	|RAIN_ADC;
	_delay_ms(10);
	ADC_RUNTIME(&rainValue);
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void ADC_RUNTIME(uint32_t* MES)
{
	uint8_t i = 0;
	*MES = 0;
	for (i = 0; i<NUM_FOR_MES; i++)
	{
		ADC_mes();
		*MES += ADC;
	}
	*MES /= NUM_FOR_MES;
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void BH1750_getData()
{
	BH1750_read( BH1750_ADR, BH1750_ONE_TIME_HIGH_RES_MODE, 2, light_buf );

	lux_temp = (light_buf[0]<<8);
	lux_temp +=light_buf[1];
	lux = (float)(lux_temp)/(1.2);
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void BMP180_getData()
{
	temp = BMP180_gett();
	press = BMP180_getp();
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void DS18X20Init()
{
	ds18b = search_sensors();
	DS18X20_start_meas(DS18X20_POWER_EXTERN,NULL);
	
	_delay_ms(750);
	DS18X20Read();
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void DS18X20Read()
{
	if (DS18X20_OK == DS18X20_read_meas_single(ds18b, &subzero, &cel, &cel_frac))
	{
		if (subzero)
		{
			strcpy(temperatureDS18b20,"-");
		}
		else
		{
			strcpy(temperatureDS18b20," ");
		}
		
		IntToStr();
	}
	else
	{
		lcd_gotoxy(1,0);
		lcd_swrite("error");
	}
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void Hello()
{
	lcd_gotoxy(0,0);
	lcd_swrite_P(PSTR("WeatherStation"));
	lcd_gotoxy(0,1);
	for (subzero=0;subzero<16;subzero++)
	{
		lcd_swrite("+");
		_delay_ms(200);
	}
	
	_delay_ms(500);
	lcd_clear();
	lcd_home();
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void IntToStr()
{
	itoa(cel,buf,10);
	strcat(temperatureDS18b20,buf);
	strcat(temperatureDS18b20,",");
	itoa(cel_frac,buf,10);
	strcat(temperatureDS18b20,buf);
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void LcdDisplayData()
{
	lcd_clear();
	lcd_home();
	
	lcd_swrite(temperatureDS18b20);
	lcd_swrite(" ");
	lcd_iwrite(dthHumidity*10);
	lcd_swrite(" ");
	lcd_iwrite(dthTemperature*10);
	lcd_swrite(" ");
	
	lcd_gotoxy(0,1);
	lcd_iwrite(humEarthValue);
	lcd_swrite(" ");
	lcd_iwrite(rainValue);
	lcd_swrite(" ");
	
	lcd_iwrite(lux);
	lcd_swrite(" ");
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void SecondsRuntime()
{
	if ((second%3) == 0)
	{
		if(dht_gettemperaturehumidity(&dthTemperature, &dthHumidity) != -1)
		{
		}
	}
	if ((second%3) == 1)
	{
		DS18X20_start_meas(DS18X20_POWER_EXTERN,NULL);
	}
	if ((second%3) == 2)
	{
		DS18X20Read();
	}
	
	s1_flag = 0;
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void timer0Init()
{
	TCCR0 |= (1<<WGM01);			//tryb CTC
	TCCR0 |= (1<<CS02) | (1<<CS00); //preskaler 1024
	OCR0   = 108;					//przerwanie co 10ms
	TIMSK |= (1<<OCIE0);			//zezwolenie na przerwanie compare match
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void USARTMakeDataFrame()
{
	strcpy(usartData,temperatureDS18b20);
	strcat(usartData," ");
	
	itoa(dthTemperature * 10,buf,10);
	strcat(usartData,buf);
	strcat(usartData," ");
	
	itoa(dthHumidity * 10,buf,10);
	strcat(usartData,buf);
	strcat(usartData," ");
	
	itoa(humEarthValue,buf,10);
	strcat(usartData,buf);
	strcat(usartData," ");
	
	itoa(rainValue,buf,10);
	strcat(usartData,buf);
	strcat(usartData," ");
	
	itoa(lux,buf,10);
	strcat(usartData,buf);
	strcat(usartData," ");
	
	itoa(light_buf[0],buf,10);
	strcat(usartData,buf);
	strcat(usartData,",");
	itoa(light_buf[1],buf,10);
	strcat(usartData,buf);
	strcat(usartData," ");
	
	itoa(temp,buf,10);
	strcat(usartData,buf);
	strcat(usartData," ");
	
	itoa(press>>8,buf,10);
	strcat(usartData,buf);
	strcat(usartData," ");
	
	strcat(usartData," ");
	itoa(press & 0x00FF,buf,10);
	strcat(usartData,buf);
	strcat(usartData," ");
	
	
	strcat(usartData,&CR);
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void USART_Init()
{
	/*Set baud rate */
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
	
	/*Enable transmitter */
	UCSRB |= (1<<TXEN);
	
	/* Set frame format: 8data, 1stop bit */
	UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void USART_TransmitByte(uint8_t a_data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) );
	
	/* Put data into buffer, sends the data */
	UDR = a_data;
}


//--------//--------//--------//--------//--------//--------//--------//--------//--------
void USART_SendText(char* a_Text)
{
	while(*a_Text != 0x00)
	{
		USART_TransmitByte(*a_Text);
		a_Text++;
	}
}

