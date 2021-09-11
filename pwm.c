#ifndef F_CPU
#define F_CPU 16000000
#endif

/********************************************************************************************************/

#include <avr/io.h>
#include <util/delay.h>

/********************************************************************************************************/

#define speedInput PORTD0       //D21
#define motorTempInput PORTF7   //A0
#define pot PORTF3              //A3

/********************************************************************************************************/


uint16_t read_adc(uint8_t adcChannel);
void init_adc();
uint16_t adcValue = 0;
void adjust_pwm();

/********************************************************************************************************/

void adc_protection();
uint8_t protectionFlag = 0;

/********************************************************************************************************/

void init_io();
void init_pwm_timer();

/********************************************************************************************************/

int main(void)
{

  init_io();
  init_pwm_timer();
  init_adc();

  while (1)
  {
     adjust_pwm();
  }
}


/********************************************************************************************************/

void init_io() {
  DDRB |= (1 << PB6); //PORTB6 Output   Arduino Mega D12
  DDRF &= ~(1 << pot);  //PORTF3 INPUT Potentiometer
  DDRF &= ~(1 << motorTempInput );

}

/********************************************************************************************************/

void init_pwm_timer() {

  TCCR1A |= (1 << WGM11) | (1 << WGM10);
  TCCR1A |= (1 << COM1B1);
  TCCR1B |= (1 << CS11);
  TCCR1B |= (1 << WGM13); //Phase Correct PWM , top : OCR1A , prescaler 8

  OCR1A = 66;
  OCR1B = 25;
}

/********************************************************************************************************/

void init_adc() {
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) ; //ADC frequency prescaler 128 125kHz
  ADMUX |= (1 << REFS0);              //Voltage reference selection AVCC
  ADCSRA |= (1 << ADEN);              //ADC enable
  ADCSRA |= (1 << ADSC);              //Make first conversion
}

/*****************************************************************************************************/

void adc_protection() {
  adcValue = read_adc(PF3);

  if (adcValue > 5) {
    protectionFlag = 1;
  }

}

/***************************************************************************************************/

void adjust_pwm() {

  adcValue = read_adc(pot);
  adcValue = adcValue / 20;
  OCR1B = adcValue;

}

/***************************************************************************************************/

uint16_t read_adc(uint8_t adcChannel) {
  ADMUX &= 0xf0;                          //Reset last four bit of admux to set new adc channel
  ADMUX |= adcChannel;                    //Set our adc channel
  ADCSRA |= (1 << ADSC);                  //Make first conversion

  while (ADCSRA & (1 << ADSC)) {}         //Wait until first conversion done

  return ADCW;                //Return adc result
}

/**************************************************************************************************/
