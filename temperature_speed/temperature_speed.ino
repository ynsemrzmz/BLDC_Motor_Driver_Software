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

/********************************************************************************************************/

void adc_protection();
uint8_t protectionFlag = 0;

/********************************************************************************************************/

void init_io();
/********************************************************************************************************/

uint32_t rpm = 0;
uint32_t hiz = 0;
bool timeFlag = 0;
unsigned long firstTime = 0;
unsigned long secondTime = 0;
unsigned long timeDiff = 0;
uint32_t sec = 0;
void init_interrupt();


/********************************************************************************************************/


void read_motor_temp();
float motorTemp = 0.0;
unsigned int motorAdc = 0;


/********************************************************************************************************/

char tx_buff[17];
uint8_t checksum = 0;

void convert_2_int(int sayi, uint8_t *temp);
uint8_t calculate_checksum(uint8_t *array, uint16_t len);
void send_data_checksum();
void timer_kesmesi_setup();
float hamData[2] = {34.45, 55.42};

/********************************************************************************************************/
uint32_t sayac_1_u32;
uint32_t sayac_2_u32;
uint32_t sayac_3_u32;
uint8_t hiz_kontrolu_u8;
uint32_t hesaplanan_hiz_u32 ;
uint32_t gecen_zaman_u32;
uint32_t pulse_sayac_1_u32;
uint32_t pulse_sayac_2_u32;
int ledState = 0;
float sicaklik = 0.0;
void timer_kesmesi_setup();
void hiz_olcumu();
void pulse_yakalama();
void sicaklik_oku() ;

void timer_init();

/********************************************************************************************************/

void setup()
{
  init_io();
  init_adc();
  init_interrupt();
  timer_kesmesi_setup();
  timer_init();
  
  Serial.begin(115200);
  Serial3.begin(115200);

  pinMode(21, INPUT);
  pinMode(A7, INPUT);
}

void loop()
{
  
  calculate_speed();
  sicaklik_oku();


}



/********************************************************************************************************/

ISR(INT0_vect) {


  hiz_kontrolu_u8 = 0;     // bu deger araba duruyorken hiz verisini sifirlamak icin kullanilir
  sayac_2_u32 = 0;
  pulse_sayac_1_u32++;  // hiz olcumu icin kullanilir

  if (sayac_1_u32 > 0)
  {
    pulse_sayac_1_u32 = 0;
    sayac_1_u32 = 0;
  }



}

/********************************************************************************************************/

ISR(TIMER4_COMPA_vect)
{
  sayac_1_u32++;   //pulse ölçümü yaparken
  sayac_2_u32++;  //pulse gelmediginde hizi sifirlamak için
  sayac_3_u32++;

  if (hiz_kontrolu_u8 == 0)
  {
    if (sayac_2_u32 >= 3)
      hiz_kontrolu_u8 = 1;
  }
  else
  {
    sayac_2_u32 = 0;
  }
  
  send_data_checksum();
  //Serial.print(rpm);
  //Serial.print(" ");
  //Serial.println(hiz);
  
}

/********************************************************************************************************/

void calculate_speed() {

  if (hiz_kontrolu_u8 == 0)
  {
    rpm = ( 60 * pulse_sayac_1_u32 ) / 5;
    hiz = 0.105 * rpm ;
  }
  else
  {
    rpm = 0;
    hiz = 0;
  }

}



/********************************************************************************************************/

void sicaklik_oku() {

  float average[20] = {0};
  float sum = 0.0;
  float cValue = 0.0;
  int k = 0;
  int z = 0;
  for (k = 0 ; k < 20 ; k++) {
    average[k] = analogRead(A7);
  }

  sum = 0.0;

  for (z = 0; z < 20 ; z++) {
    sum += average[z];
  }

  cValue = sum / 20;

  sicaklik = ((float)cValue / 1023.0) * 5000;
  sicaklik = (sicaklik - 500) / 10.0;

}

/********************************************************************************************************/


void init_io() {
  DDRB |= (1 << PB6); //PORTB6 Output   Arduino Mega D12
  DDRF &= ~(1 << pot);  //PORTF3 INPUT Potentiometer
  DDRF &= ~(1 << motorTempInput );

}

/********************************************************************************************************/

void init_interrupt() {
  DDRD &= ~(1 << speedInput);     //buton giriş
  EICRA |=  (1 << ISC01);   
  EICRA &= ~(1<<ISC00);   //interrupt falling edge
  EIMSK |= (1 << INT0);   //interrupt aktif
  sei();
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

uint16_t read_adc(uint8_t adcChannel) {
  ADMUX &= 0xf0;                          //Reset last four bit of admux to set new adc channel
  ADMUX |= adcChannel;                    //Set our adc channel
  ADCSRA |= (1 << ADSC);                  //Make first conversion

  while (ADCSRA & (1 << ADSC)) {}         //Wait until first conversion done

  return ADCW;                //Return adc result
}

/**************************************************************************************************/

void read_motor_temp() {

  /*motorAdc = read_adc(motorTempInput);
    motorTemp = (motorAdc / 1023) * 5.00;
    motorTemp = motorTemp / 10.0;*/

  sicaklik_oku();
  //Serial.println(sicaklik);

}

/**************************************************************************************/

void convert_2_int(int sayi, uint8_t *temp)
{
  uint32_t sonuc = 0;
  uint32_t bolum, kalan; //bolum1, kalan1;

  if (sayi < 0)
  {
    sayi = -sayi;
  }
  else if (sayi > 0) {
    sayi = sayi;
  }

  if (sayi >= 10000)
  {
    bolum = (int)sayi / 100000;
    kalan = (int)sayi % 100000;

    *temp++ = bolum + '0';

    bolum = kalan / 10000;
    kalan = kalan % 10000 ;

    *temp++ = bolum + '0';

    bolum = kalan / 1000;
    kalan = kalan % 1000;

    *temp++ = bolum + '0';

    bolum = kalan / 100;
    kalan = kalan % 100;

    *temp++ = bolum + '0';

    bolum = kalan / 10;
    kalan = kalan % 10;

    *temp++ = bolum + '0';
    *temp++ = kalan + '0';

  }


  if (sayi < 10000 && sayi >= 1000)
  {
    bolum = (long)sayi / 1000;
    kalan = (long)sayi % 1000;

    *temp++ = bolum + '0';

    bolum = kalan / 100;
    kalan = kalan % 100 ;

    *temp++ = bolum + '0';

    bolum = kalan / 10;
    kalan = kalan % 10 ;

    *temp++ = bolum + '0';
    *temp++ = kalan + '0';


  }

  if (sayi < 1000 && sayi >= 100)
  {
    *temp++ = '0';

    bolum = (long)sayi / 100;
    kalan = (long)sayi % 100;

    *temp++ = bolum + '0';

    bolum = kalan / 10;
    kalan = kalan % 10 ;

    *temp++ = bolum + '0';
    *temp++ = kalan + '0';
  }

  if (sayi < 100 && sayi >= 10)
  {

    *temp++ = '0';
    *temp++ = '0';
    sonuc = sayi - (int)sayi;

    bolum = (long)sayi / 10;
    kalan = (long)sayi % 10;

    *temp++ = bolum + '0' ;
    *temp++ = kalan + '0';
  }
  if (sayi < 10)
  {
    *temp++ = '0';
    *temp++ = '0';
    *temp++ = '0';
    sonuc = sayi - (long)sayi;

    bolum = (long)sayi;

    *temp++ = bolum + '0';

  }
}


/*************************************************************************************************/

uint8_t calculate_checksum(uint8_t *array, uint16_t len) {
  uint8_t rem = 0x41;
  uint16_t  i = 1, j = 0;

  for (i = 1; i < len; i++) {

    rem = rem ^ array[i];

    for (j = 0; j < 8; j++) {

      if (rem & 0x80) {  // if leftmost (most significant) bit is set
        rem = (rem << 1) ^ 0x07;
      }
      else {
        rem = rem << 1;
      }

    }

  }

  return rem;
}


/****************************************************************************************************/

void send_data_checksum() {
  uint8_t temp_buff[4]; //Checksum için gerekli değişkenler

  tx_buff[0] = '<';
  tx_buff[1] = '[';

  convert_2_int(int(hiz), temp_buff);
  tx_buff[2] = temp_buff[2];              //hiz
  tx_buff[3] = temp_buff[3];
  tx_buff[4] = '0';
  tx_buff[5] = '0';
  tx_buff[6] = '|'; //ayirma biti

  convert_2_int(int(sicaklik * 100), temp_buff);
  tx_buff[7] = temp_buff[0];
  tx_buff[8] = temp_buff[1]; //motor sicakliği
  tx_buff[9] = temp_buff[2];
  tx_buff[10] = temp_buff[3];

  tx_buff[11] = ']';

  uint8_t chcksm_array[4];
  checksum = calculate_checksum(tx_buff, 12);
  convert_2_int(int(checksum), chcksm_array);

  tx_buff[12] = chcksm_array[1];
  tx_buff[13] = chcksm_array[2];
  tx_buff[14] = chcksm_array[3];

  tx_buff[15] = '*'; //checksum bitiş biti
  tx_buff[16] = '>'; //bitis biti


  for (int i = 0; i < 17; i++)
  {
    Serial3.print(tx_buff[i]);
  }
  Serial3 .println();

}

/**************************************************************************************************/

void timer_init()
{
  // TIMER 0 for interrupt frequency 1000 Hz:
  cli(); // stop interrupts
  TCCR0A = 0; // set entire TCCR0A register to 0
  TCCR0B = 0; // same for TCCR0B
  TCNT0  = 0; // initialize counter value to 0
  // set compare match register for 1000 Hz increments
  OCR0A = 249; // = 16000000 / (64 * 1000) - 1 (must be <256)
  // turn on CTC mode
  TCCR0B |= (1 << WGM01);
  // Set CS02, CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei(); // allow interrupts
}


void timer_kesmesi_setup()
{

  //set timer4 interrupt at 1Hz
  TCCR4A = 0;// set entire TCCR1A register to 0
  TCCR4B = 0;// same for TCCR1B
  TCNT4  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR4A = 15624 / 1; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR4B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR4B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);

  sei();//allow interrupts


}
