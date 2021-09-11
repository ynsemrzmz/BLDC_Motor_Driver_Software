#ifndef F_CPU
#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <util/delay.h>

/************************************/

#define highSidePWM_A_EN PORTL6
#define lowSidePWM_A_EN_not PORTL2
#define lowSide_A_EN PORTB6

#define lowSidePWM_A_EN PORT5           //Phase A

#define highSidePWM_A_MCU PORTL5
#define lowSidePWM_A_MCU  PORTL4

/*******************************/

#define highSidePWM_B_EN  PORTB4
#define lowSidePWM_B_EN_not  PORTH6
#define lowSide_B_EN      PORTH5

#define lowSidePWM_B_EN PORTH4          //Phase B

#define highSidePWM_B_MCU   PORTL3
#define lowSidePWM_B_MCU  PORTB7

/*******************************/

#define highSidePWM_C_EN      PORTH1
#define lowSidePWM_C_EN_not   PORTH0
#define lowSide_C_EN          PORTE4

#define lowSidePWM_C_EN       PORTE5    //Phase C

#define highSidePWM_C_MCU     PORTE3
#define lowSidePWM_C_MCU      PORTG5


/*****************************************************/

#define hallA PL7
#define hallB PD0         //Hall Effect Sensors
#define hallC PD1


/*****************************************************/

#define potInF PK7
#define dirIn  PA5
bool dir = 0;


/*****************************************************/

void commute_bc();
void commute_ac();
void commute_ab();
void commute_cb();        //MOSFET Commutation functions
void commute_ca();
void commute_ba();

void init_mosfet_gpios();

/*****************************************************/

void read_hall_sensors();
void init_hall_sensors();

uint8_t hallSensorA = 0;
uint8_t hallSensorB = 0;
uint8_t hallSensorC = 0;

void init_hall_interrupts();

volatile uint8_t hallIntB;
volatile uint8_t hallIntC;

/*

  HALLB -> INT0 PD0
  HALLC -> INT1 PD1

*/

/*****************************************************/

void drive_forward();
void drive_reverse();

/*****************************************************/

uint16_t read_adc(uint8_t adcChannel);
void init_adc();

volatile uint16_t adcValue = 0;

/*****************************************************/




int main() {

  init_mosfet_gpios();
  init_hall_sensors();

  
  while (1) {


    if (!(PINA & (1 << PA5))) {
      read_hall_sensors();
      drive_forward();
    }
    else {
      read_hall_sensors();
      drive_reverse();
    }

  }

}

void init_mosfet_gpios() {

  //---------------------------------------//

  DDRL |= (1 << highSidePWM_A_EN);
  DDRB |= (1 << lowSide_A_EN);

  DDRB |= (1 << highSidePWM_B_EN);
  DDRH |= (1 << lowSide_B_EN );             //mosfet control pins

  DDRH |= (1 << highSidePWM_C_EN);
  DDRE |= (1 << lowSide_C_EN);

  // ---------------------------------- - //

  DDRL |= (1 << highSidePWM_A_MCU);
  DDRL |= (1 << lowSidePWM_A_MCU);          //unused mosfet control pins
  DDRL |= (1 << lowSidePWM_A_EN_not);

  // ------------------------------------ //

  DDRL |=  (1 << highSidePWM_B_MCU);
  DDRB |=  (1 << lowSidePWM_B_MCU);         //unused mosfet control pins
  DDRH |=  (1 << lowSidePWM_B_EN_not);

  // ------------------------------------ //
  DDRE |= (1 << highSidePWM_C_MCU);
  DDRG |= (1 << lowSidePWM_C_MCU);        //unused mosfet control pins
  DDRH |= (1 << lowSidePWM_C_EN_not);

  // ------------------------------------ //


  DDRL &= ~(1 << highSidePWM_A_MCU);
  DDRL &= ~(1 << lowSidePWM_A_MCU);
  DDRL &= ~(1 << lowSidePWM_A_EN_not);



  DDRL &=  ~(1 << highSidePWM_B_MCU);
  DDRB &=  ~(1 << lowSidePWM_B_MCU);         //disable unused mosfet control pins
  DDRH &=  ~(1 << lowSidePWM_B_EN_not);


  DDRE &= ~(1 << highSidePWM_C_MCU);
  DDRG &= ~(1 << lowSidePWM_C_MCU);
  DDRH &= ~(1 << lowSidePWM_C_EN_not);

  //------------------------------------ //


}

void commute_bc() {
  PORTL &= ~(1 << highSidePWM_A_EN);
  PORTB &= ~(1 << lowSide_A_EN);
  PORTH &= ~(1 << lowSide_B_EN );      //disable unused pins  for protection
  PORTH &= ~(1 << highSidePWM_C_EN);

  PORTB |= (1 << highSidePWM_B_EN);   //current flows through B => C
  PORTE |= (1 << lowSide_C_EN);

}

void commute_ac() {
  PORTB &= ~(1 << highSidePWM_B_EN );
  PORTH &= ~(1 << lowSide_B_EN );
  PORTB &= ~(1 << lowSide_A_EN);        //disable unused pins for protection
  PORTH &= ~(1 << highSidePWM_C_EN);

  PORTL |= (1 << highSidePWM_A_EN);   //current flows  A => c
  PORTE |= (1 << lowSide_C_EN );

}

void commute_ab() {
  PORTE &= ~(1 << lowSide_C_EN );
  PORTH &= ~(1 << highSidePWM_C_EN);
  PORTB &= ~(1 << lowSide_A_EN);        //disable unused pins for protection
  PORTB &= ~(1 << highSidePWM_B_EN );

  PORTL |= (1 << highSidePWM_A_EN);
  PORTH |= (1 << lowSide_B_EN );        //current flows  A => B

}

void commute_cb() {
  PORTL &= ~(1 << highSidePWM_A_EN);
  PORTB &= ~(1 << lowSide_A_EN);          //disable unused pins for protection
  PORTB &= ~(1 << highSidePWM_B_EN );
  PORTE &= ~(1 << lowSide_C_EN );


  PORTH |= (1 << highSidePWM_C_EN);
  PORTH |= (1 << lowSide_B_EN);         //current flows  C => B
}

void commute_ca() {
  PORTB &= ~(1 << highSidePWM_B_EN );
  PORTH &= ~(1 << lowSide_B_EN );
  PORTE &= ~(1 << lowSide_C_EN );       //disable unused pins for protection
  PORTL &= ~(1 << highSidePWM_A_EN);

  PORTH |= (1 << highSidePWM_C_EN );    //current flows  C => A
  PORTB |= (1 << lowSide_A_EN);
}

void commute_ba() {
  PORTE &= ~(1 << lowSide_C_EN );
  PORTH &= ~(1 << highSidePWM_C_EN);    //disable unused pins for protection
  PORTH &= ~(1 << lowSide_B_EN );
  PORTL &= ~(1 << highSidePWM_A_EN);

  PORTB |= (1 << highSidePWM_B_EN );    //current flows  B => A
  PORTB |= (1 << lowSide_A_EN);
}


void init_hall_sensors() {

  DDRL &= ~(1 << hallA);
  DDRB &= ~(1 << hallB);
  DDRC &= ~(1 << hallC);        //set hall sensors to input

  DDRA &= ~(1 << dirIn);    //set direction pin to input

}

void read_hall_sensors() {

  if (PINL & (1 << hallA))
    hallSensorA = 1;
  else
    hallSensorA = 0;


  if (PIND & (1 << hallB))
    hallSensorB = 1;
  else
    hallSensorB = 0;            //read hall sensors


  if (PIND & (1 << hallC))
    hallSensorC = 1;
  else
    hallSensorC = 0;

}

void drive_forward() {

  if (hallSensorA == 0 && hallSensorB == 0 && hallSensorC == 1) {
    commute_bc();
  }

  else if (hallSensorA == 0 && hallSensorB == 1 && hallSensorC == 1) {
    commute_ac();
  }

  else if (hallSensorA == 0 && hallSensorB == 1 && hallSensorC == 0) {
    commute_ab();
  }

  else if (hallSensorA == 1 && hallSensorB == 1 && hallSensorC == 0) {
    commute_cb();
  }

  else if (hallSensorA == 1 && hallSensorB == 0 && hallSensorC == 0) {
    commute_ca();
  }

  else { //(hallSensorA == 1 && hallSensorB == 0 && hallSensorC == 1)
    commute_ba();
  }

}

void drive_reverse() {

  if (hallSensorA == 0 && hallSensorB == 0 && hallSensorC == 1) {
    commute_cb();
  }

  else if (hallSensorA == 0 && hallSensorB == 1 && hallSensorC == 1) {
    commute_ca();
  }

  else if (hallSensorA == 0 && hallSensorB == 1 && hallSensorC == 0) {
    commute_ba();
  }

  else if (hallSensorA == 1 && hallSensorB == 1 && hallSensorC == 0) {
    commute_bc();
  }

  else if (hallSensorA == 1 && hallSensorB == 0 && hallSensorC == 0) {
    commute_ac();
  }

  else { //(hallSensorA == 1 && hallSensorB == 0 && hallSensorC == 1)
    commute_ab();
  }

}


void init_adc() {
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) ; //ADC frequency prescaler 128 125kHz
  ADMUX |= (1 << REFS0);              //Voltage reference selection AVCC
  ADCSRA |= (1 << ADEN);              //ADC enable
  ADCSRA |= (1 << ADSC);              //Make first conversion
}


uint16_t read_adc(uint8_t adcChannel) {
  ADMUX &= 0xf0;                          //Reset last four bit of admux to set new adc channel
  ADMUX |= adcChannel;                    //Set our adc channel
  ADCSRA |= (1 << ADSC);                  //Make first conversion

  while (ADCSRA & (1 << ADSC)) {}         //Wait until first conversion done

  return ADCW;                //Return adc result
}

/*
  void init_hall_interrupts() {
  DDRD &= ~(1 << PORTD0);
  DDRD &= ~(1<<PORTD1);
  EICRA |= (1 << ISC00 ) | (1 << ISC10);
  EIMSK |= (1 << INT0) | (1 << INT1);
  sei();
  }*/

/*
  ISR(INT0_vect) {
  hallIntB != hallIntB;
  }

  ISR(IN1_vect) {
  hallIntC != hallIntC;
  }*/
