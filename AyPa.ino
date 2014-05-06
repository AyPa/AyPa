#include <avr/pgmspace.h>

// примерное число миллисекунд в часе

//#define MILS 3640282 //  надо 20:55:13 показал 21:00:00 +47
//#define MILS 3650000 //  надо 04:19:16 показал 04:19:00 -16
//#define MILS 3648000 //  за 4 часа убежал на 25с  
//#define MILS 3650500 // за 2 часа убежал на 20с  
//#define MILS 3655500 //  за 4 часа отстал на 16  c
//#define MILS 3653000 //   за2:43 убежал на 3с
//#define MILS 3653300 //  pза 10:22 отстал на 34с
#define MILS 3652800 //  

// watts:  0: 1.1w  1: 8.5 w 2: 9.6w 3:  9.7w 4: 9.9 w  (running fan +0.5w)


#include "AyPa_m.h" // полезные макросы
#include "AyPa_fonts.h" // шрифты
#include "AyPa_n.h" // модуль для дисплея Nokia3110

//#include "DHT.h"
//#define DHTPIN A0     // what pin we're connected to
// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

//DHT dht(DHTPIN, DHTTYPE);

void (* reboot) (void) = 0; //declare reset function @ address 0

//word freeRam(void){ extern unsigned int __heap_start; extern void *__brkval; int v; return (int)&v-(__brkval==0?(int)&__heap_start:(int)__brkval);}


#define SetADC(bandgap,input,us){ ADCSRA|=(1<<ADEN);delayMicroseconds(2);ADMUX=(bandgap<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input;delayMicroseconds(us);} // input (0..7,8,14) (bg/vcc analogReference )
#define ADCoff{ ADCSRA&=~(1<<ADEN); }
#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 

//byte Intensity[24] = {0,0,0,0,0,0, 1,2,3,3,3,3, 3,3,3,3,3,3, 3,3,3,3,2,1}; // почасовая интенсивность 
//byte Intensity[24] = {3,0,0,0,0,0, 1,2,3,4,4,4, 4,4,4,4,4,4, 4,4,4,3,2,1}; // почасовая интенсивность 
//byte Intensity[24] = {0xE,0,0,0,0,0, 0x8,0xA,0xB,0xF,0xF,0xF, 0xF,0xF,0xF,0xF,0xF,0xF, 0xF,0xF,0xF,0xE,0xA,0x8}; // почасовая интенсивность 

uint8_t TempH[76]={0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0}; // архив температур

//uint8_t Intensity[24] = {0,0,0,0,0,0, 1,1,2,2,3,3, 4,4,4,3,3,3, 2,2,2,1,1,1}; // почасовая интенсивность  1-8
//uint8_t Intensity[24] = {0,0,0,0,0,0, 1,2,3,4,4,4, 4,4,4,4,4,4, 4,4,4,4,2,1}; // почасовая интенсивность  9 up
//uint8_t Intensity[24] = {0,0,0,0,0,0, 1,1,1,1,1,1, 1,1,1,1,1,1, 1,1,1,1,1,0}; // почасовая интенсивность  15 up
//uint8_t Intensity[24] = {0,0,0,0,0,0, 1,2,2,2,2,2, 2,2,2,2,2,2, 2,2,2,2,1,0}; // почасовая интенсивность  01/05 (20) up
uint8_t Intensity[24] = {0,0,0,0,0,0, 1,2,2,2,2,2, 2,2,2,2,2,2, 2,2,2,2,2,1}; // почасовая интенсивность  01/05 (20) up
//uint8_t Intensity[24] = {0,1,1,1,2,2, 2,3,3,3,4,4, 4,4,4,4,4,4, 4,4,3,2,1,0}; // почасовая интенсивность  01/05 (20) up
//uint8_t decode[5]={0,0x8,0xA,0xE,0xF};
// 0xF 1111 (4)
// 0xE 1110 (3)
// 0xA 1010 (2)
// 0x8 1000 (1)
// 0x0 0000 (0)
uint8_t FlashIntensity=0;

//uint8_t volatile ticks=0; // 1/2с
uint8_t HR=0;
uint8_t prevHR=0xFF;


//byte __attribute__ ((noinline)) unBCD(byte bcd){return (((bcd>>4)*10)+(bcd&0xF)); }
/*
word TouchSensor(void) // 
{
      word cycles=30000;
      Pin2Output(DDRC,0);Pin2LOW(PORTC,0); // discharge sensor  pin
//      delayMicroseconds(50); // works fine without???
      delayMicroseconds(100);
      Pin2Input(DDRC,0);
      for(int i=0;i<cycles;i++){if (PINC&0b00000001){cycles=i;break;}}
      Pin2Output(DDRC,0);Pin2LOW(PORTC,0); // discharge sensor  pin
      Pin2Input(DDRC,0);Pin2LOW(PORTC,0); // sensor pin
      return cycles;
}*/


byte volatile WDsleep;
byte volatile WDflag;
byte sleeps=0;

// Watchdog timeout values
#define T16MS 0
#define T32MS 1
#define T64MS 2
#define T128MS 3
#define T250MS 4
#define T500MS 5
#define T1S 6
#define T2S 7
#define T4S 8
#define T8S 9
 
//#define system_sleep(mode,flags) { set_sleep_mode(mode); sleeps=0;do{sleep_enable();sleep_cpu();sleep_disable();if(flags){break;}else{sleeps++;}}while(1);}
 
#define setup_watchdog(timeout){cli(); __asm__ __volatile__("wdr\n\t"); MCUSR&=~(1<<WDRF);WDTCSR|=(1<<WDCE)|(1<<WDE);WDTCSR=((1<<WDIE)|timeout);sei();}
 
ISR(WDT_vect) { if(WDsleep){WDflag=1; WDsleep=0; } else{ reboot(); }} // Watchdog timer interrupt

uint8_t DHTdata[5];

/*
uint8_t DHT_ReadData(void) 
{ 
   uint8_t i; 
   uint8_t v = 0; 
   uint16_t d;

   for(i=0; i<8; i++) 
   { 
      d=10000;while (!(PINB&(1<<1))) {if(--d==0){return 0xBA;}}; 
      delayMicroseconds(30);
      if (PINB&(1<<1)) { v = v |(1<<(7 - i)); d=10000;while (PINB&(1<<1)){if(--d==0){return 0xBA;}}; } 
   } 
   return v;    
} */

//#define TIMEOUT (F_CPU/1600);
#define TIMEOUT (8000000L/400);
//#define TIMEOUT (8000000L/1200);
#define DHTLIB_OK				0
#define DHTLIB_ERROR_CHECKSUM	-1
#define DHTLIB_ERROR_TIMEOUT	-2
#define DHTLIB_INVALID_VALUE	-999

extern unsigned long timer0_millis;
//extern unsigned long timer0_overflow_count;

int dhtread(void) //PB1 (9)
{
    uint8_t mask = 128;
    uint8_t idx = 0;

    // EMPTY BUFFER
    for (uint8_t i=0; i< 5; i++) DHTdata[i] = 0;

    // REQUEST SAMPLE
Pin2Output(DDRB,1);//    pinMode(pin, OUTPUT);
Pin2LOW(PORTB,1);//    digitalWrite(pin, LOW);
    //delay(20);
    delayMicroseconds(20000);

//OCR2A = 250; // Set CTC compare value 
    
    
Pin2HIGH(PORTB,1);//    digitalWrite(pin, HIGH);
    delayMicroseconds(40);
Pin2Input(DDRB,1);//    pinMode(pin, INPUT);

//TIMSK2 = 0;//(1 << OCIE2A); // Enable CTC interrupt 

 // TCCR2B=0;//(0<<WGM22)|(0<<CS22)|(1<<CS21)|(0<<CS20); // 8
 // TCCR2A=0;//(1<<WGM21);//ctc mode |(1<<CS22)|(0<<CS21)|(0<<CS20); // 256?
 // TIMSK2 = 0; // Enable CTC interrupt 
//  OCR2A = 8; // Set CTC compare value 

cli();
//TCNT2=0;

    // GET ACKNOWLEDGE or TIMEOUT
    unsigned int loopCnt = TIMEOUT;
//    while(digitalRead(pin) == LOW)
    while((PINB&(1<<1)) == 0)
    if (--loopCnt == 0) return DHTLIB_ERROR_TIMEOUT;

    loopCnt = TIMEOUT;
//    while(digitalRead(pin) == HIGH)
    while((PINB&(1<<1)) > 0)
    if (--loopCnt == 0) return DHTLIB_ERROR_TIMEOUT;


    // READ THE OUTPUT - 40 BITS => 5 BYTES
    for (uint8_t i=0; i<40; i++)
    {
        loopCnt = TIMEOUT;
//        while(digitalRead(pin) == LOW)
        while((PINB&(1<<1)) == 0)
        if (--loopCnt == 0) return DHTLIB_ERROR_TIMEOUT;

//        unsigned long t = micros(); // this is where interrupts are enabled
     //   unsigned long t = mmicros();
        uint8_t t=TCNT0;

        loopCnt = TIMEOUT;
//        while(digitalRead(pin) == HIGH)
        while((PINB&(1<<1)) > 0)
        if (--loopCnt == 0) return DHTLIB_ERROR_TIMEOUT;

//        if ((micros() - t) > 40) DHTdata[idx] |= mask;
       // if ((mmicros() - t) > 50) DHTdata[idx] |= mask;
//        if ((TCNT0 - t) > 5) DHTdata[idx] |= mask;
//        if ((TCNT0 - t) > 6) DHTdata[idx] |= mask;
        if ((TCNT0 - t) > 7) DHTdata[idx] |= mask;
        mask >>= 1;
        if (mask == 0)   // next byte?
        {
            mask = 128;
            idx++;
        }
    }

    return DHTLIB_OK;
}

int read22(void)
{
    // READ VALUES
//    int rv = dhtread(pin);
    int rv = dhtread(); // PB1
  
   // TIMSK2 = (1 << OCIE2A); // Enable CTC interrupt 
//   OCR2A = 12; // Set CTC compare value 
//TCNT2=0;
  //TCCR2B=(0<<WGM22)|(0<<CS22)|(1<<CS21)|(0<<CS20); // 8
  //TCCR2A=(1<<WGM21);//ctc mode |(1<<CS22)|(0<<CS21)|(0<<CS20); // 256?
  //TIMSK2 = (1 << OCIE2A); // Enable CTC interrupt 
  //OCR2A = 8; // Set CTC compare value 
 
    sei();
    
    if (rv != DHTLIB_OK)
    {
//        humidity    = DHTLIB_INVALID_VALUE;  // invalid value, or is NaN prefered?
  //      temperature = DHTLIB_INVALID_VALUE;  // invalid value
        return rv; // propagate error value
    }

    // CONVERT AND STORE
//    humidity = word(bits[0], bits[1]) * 0.1;

//    if (bits[2] & 0x80) // negative temperature
  //  {
    //    temperature = -0.1 * word(bits[2] & 0x7F, bits[3]);
//    }
  //  else
    //{
      //  temperature = 0.1 * word(bits[2], bits[3]);
//    }

    // TEST CHECKSUM
    uint8_t sum = DHTdata[0] + DHTdata[1] + DHTdata[2] + DHTdata[3];
    if (DHTdata[4] != sum) return DHTLIB_ERROR_CHECKSUM;

    return DHTLIB_OK;
}

// port C
#define moisture_input 0
#define divider_top 2
#define divider_bottom 1
//int moisture; // analogical value obtained from the experiment
//long NextSoilMoistureCheck=0;
//long NextTmpHumCheck=0;
//word volatile uptime=0; // uptime в секундах

word MCUtemp;
word MCU_Vcc;

word moisture;
void SoilMoisture(){
  uint8_t i;
//  int reading;
  // set driver pins to outputs
  Pin2Output(DDRC,divider_top);//pinMode(divider_top,OUTPUT);
  Pin2Output(DDRC,divider_bottom);//  pinMode(divider_bottom,OUTPUT);

  // drive a current through the divider in one direction
  Pin2LOW(PORTC,divider_top);//digitalWrite(divider_top,LOW);
  Pin2HIGH(PORTC,divider_bottom);//digitalWrite(divider_bottom,HIGH);

  // wait a moment for capacitance effects to settle
    SetADC(0,moisture_input,2); // A0
//  delay(250);
for(i=0;i<4;i++){  delayMicroseconds(65000); } //260ms

//  while(!ADCfree);
//  ADCfree=0;// to avoid collision with Vcc  
//  while(!ADCready);// у нас примерно 125 раз в секунду измеряется напряжение питание. здесь мы пожертвуем одним измерением ради датчика влажности почвы
//  cli();
//TCNT1=0;
         mRawADC(moisture,2);
  //     mRawADC(reading,2);
//  moisture=analogRead(moisture_input);  // take a reading
  
 
       
  
//      SetADC(0,14,1); // restore vcc adc channel
  // ADCfree=1;


//  reading=TCNT1;
//  ADCready=0;
  //sei();

  // reverse the current
  Pin2HIGH(PORTC,divider_top);//digitalWrite(divider_top,HIGH);
  Pin2LOW(PORTC,divider_bottom);//digitalWrite(divider_bottom,LOW);

  // give as much time in 'reverse' as in 'forward'
//  delay(250);
for(i=0;i<4;i++){  delayMicroseconds(65000); } //260ms

  // stop the current
  Pin2LOW(PORTC,divider_bottom);//digitalWrite(divider_bottom,LOW);


// back to high imp state
  Pin2Input(DDRC,divider_top);//pinMode(divider_top,INPUT);
  Pin2Input(DDRC,divider_bottom);//pinMode(divider_bottom,INPUT);
//  digitalWrite(divider_top,LOW);
//  digitalWrite(divider_bottom,LOW);

//  return reading;
}

uint8_t v;
//uint8_t   extreset;

//word InitialFreeRAM;

void setup() {                

//  extreset=MCUSR;
  
    Pin2Input(DDRC,3);Pin2HIGH(PORTC,3); // pull up on A3 (user button)
  //delay(100);// pull up settle time

//  AddMillis(12*MILS);// set time to noon
//  AddMillis(6*MILS);// set time to 6:00
// startup at midnight
//  uptime=0;
//  if(eeprom_read_byte((byte*)0)!=0xAA){eeprom_write_byte((byte*)0,0xAA);} // our signature
//  else{
//v=PINC&(1<<3);    
//8 if A3 is not pressed
//0 if A3 is pressed
//  if ((PINC&(1<<3))==0){ uptime=0; } // if A3 is low (button is pressed during startup)
  //eeprom_update_byte((byte*)1,0);eeprom_update_byte((byte*)2,0);
  // if A3 button is held pressed during startup - uptime is set to 0
  //else
  //{
//  if (PINC&(1<<3)==0){ 
//  uptime=(eeprom_read_byte((byte*)1)<<8)|eeprom_read_byte((byte*)2); //}
//  uptimeS=uptime+5;// 5 seconds to reset
  //}
//  bts=uptime;// to prevent false trigger on startup
//}

//    PCMSK1 = 1<<PCINT11; // setup pin change interrupt on A3 pin 
  //  PCICR |= 1<<PCIE1; 

//    Pin2Input(DDRD,2);Pin2HIGH(PORTD,2);// pinMode(1,INPUT_PULLUP);      
//    attachInterrupt(0, pin2_isr, RISING);
//sss=0;
//      watchdogSleep(SLEEP_MODE_PWR_DOWN,T2S); // 2 sleeps - INT0 every second
//delay(2000);      if(!sss){ERR=ERR_NO_SQW;}

//TWI setup
  //  TWSR&=~((1<<TWPS0)|(1<<TWPS1));
  //  TWBR=40; // 1215
//    TWBR=32; //  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;   // 1023
//    TWBR=24; // 833
//    TWBR=20; // 741
//    TWBR=16; // 649
//    TWBR=12; // 558
//    TWBR=8; // 475
//    TWBR=4; // 403
//    TWBR=2; // 370 unstable
//    TWBR=1; // unstable


    SetADC(0,14,500); // vcc

  //setup timer1
  cli();
//  TCCR1A=0x00;
  TCCR1A=(0<<WGM11)|(0<<WGM10);
  //  TCCR1B=(1 << WGM12)|(0<<CS22)|(0<<CS21)|(1<<CS20); // /no prescaler;
  TCCR1B=(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10); // /8; 1us clock //ctc mode
//  TCCR1B=(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10); // /64; 8us clock //ctc mode
//  TCCR1B=0b00001010;
//  TCCR1B=(1<<WGM12)|(0<<CS22)|(1<<CS21)|(0<<CS20); // /8; 1us clock
  //TCNT1H=0x00; TCNT1L=0x00;
  TCNT1=0;
//  ICR1H=0x00; ICR1L=0x00;
//  OCR1AH=0xFF; OCR1AL=0xFF;  //OCR1AH=0x9C;  //OCR1AL=0x40; // 40000//  OCR1AH=0x4E;//  OCR1AL=0x20; // 20000//  OCR1AH=0x03;//  OCR1AL=0xE8; // 1000
//  OCR1A=0xFFFF;
  OCR1A=62496;//2Hz at *MHz /64
// OCR1AH=0x9C;  OCR1AL=0x40; // 40000
  //  TCCR1B |= (1 << WGM12);  //  TCCR1B |= (1 << CS10);
  // no interrupts just counting 1 tick is 1 microsecond

//   TIMSK1 = (1 << OCIE1A);
//   TIMSK1 |= (1 << TOIE1);

// setup timer2 
/*
  TCNT2=0;
  TIMSK2=0; 
  TCCR2B=(0<<WGM22)|(0<<CS22)|(0<<CS21)|(1<<CS20); // 1 xxxxx000 32 microseconds
//  TCCR2B=(0<<WGM22)|(0<<CS22)|(1<<CS21)|(0<<CS20); // 8
  TCCR2A=(1<<WGM21);//ctc mode |(1<<CS22)|(0<<CS21)|(0<<CS20); // 256?
  //TIMSK2 = (1 << OCIE2A); // Enable CTC interrupt 
  OCR2A = 0xFF; // Set CTC compare value 
  //18 min to humtmp work?
*/
  sei();



//  dht.begin();
//pinMode(A0, INPUT);
//digitalWrite(A0, HIGH);  
Pin2Input(DDRB,1);
//Pin2Output(DDRB,2);
Pin2HIGH(PORTB,1);//  digitalWrite(A0, HIGH);  // internal pull up



setup_watchdog(T2S); // если в течении 2s не сбросить сторожевого пса то перезагрузка. (защита от зависаний)

//    Pin2Input(DDRC,3);Pin2HIGH(PORTC,3); // pull up on A0
// IDEA
// IDEA 2 just get VCC! :)))




//DDRD|=(1<<5)|(1<<6)|(1<<7); // same same\
//Pin2Output(DDRD,5);Pin2Output(DDRD,6);Pin2Output(DDRD,7);

      LcdBack();
// NextSoilMoistureCheck=NextTmpHumCheck=uptime;

    Pin2Output(DDRB,6);
    Pin2Output(DDRB,7);
 //   InitialFreeRAM=freeRam();
}


//ISR (PCINT1_vect){ if (PINC&(1<<3)==0){ HR++; HR&=0xF; FlashIntensity=Intensity[HR]; }}  // A3 user button handler

//ISR (PCINT1_vect){ if(uptime!=bts){//A3 is low here

//if (uptime<uptimeS){uptime=0; eeprom_update_byte((byte*)1,0);eeprom_update_byte((byte*)2,0); reboot(); }
//else{
//HR++; uptime+=(60*60/2); if(HR==24){HR=0;uptime=0;};  FlashIntensity=Intensity[HR]; LastTimeFan=uptime;
//bts=uptime;
//}}  // A3 user button handler

/*
ISR(TIMER1_OVF_vect)
 {
 //Toggle pin PD0 every second
 //    PIND=(1<<PD0);
 t1ovf++;
 }
 */
 
//uint16_t volatile ctr;
//uint8_t volatile checktime=0;

//IR: 1969 Full: 6750 Max (IR|Full):1978 6768   -- 4920LUX -рассеянный дневной свет 12:39    0.29

//IR: 3327 Full: 11525 Max (IR|Full):3331 11590 у стеклопакета  12790 LUX     0.287

//IR: 8609 Full: 33199 Max (IR|Full):8843 33936   у стеклопакета   36700 lux   солнце сквозь дымку облаков. 15:00     0.26    меньше красного




/*
 ISR (TIMER1_COMPA_vect)
{
  checktime=1;ctr++;
}

ISR(TIMER1_OVF_vect)
 {
  checktime=1;ctr++;

   //Toggle pin PD0 every second
 //    PIND=(1<<PD0);
 }
*/

uint8_t volatile t2ovf=0;
boolean volatile UpdateS=true;

 ISR (TIMER2_COMPA_vect)
{
   UpdateS=true;
}


//long wctr;
/*ISR (TIMER2_COMPA_vect)
{
//    if (++t2ovf==249) // everry 2s   8sec slow on 38min
    if (++t2ovf==248) // everry 2s
    {
        t2ovf=0; uptime++; UpdateS=true;
//        if(ADCfree){ADCfree=0;ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(1<<ADIE)|2;} // start vcc measurement (every 1/125s)
    }
}*/
/*        "in r21,3\n\t" // PINB
        "mov r20,r21\n\t"  // bits 6&7 cleared
        "ori r21,0b01000000\n\t" // bit6set
        "sts 0x0084,r1\n\t" // low(TCNT1)=0
        "out 5,r21\n\t" // 6 ON
        "mov r22,r20\n\t"
        "ori r22,0b10000000\n\t" // bit7set
*/

//ISR(TIMER2_COMPA_vect,ISR_NAKED) { 
//ISR(TIMER2_COMPA_vect) { 
  //ca++;
  // 0x21 0x22 0x28(EEPROML EEPROMH OCR0B)
    //__asm__ __volatile__(
    //"out 0x28,r1\n\t"
    //"mov r1,r24\n\t"
    //"in r1,0x28\n\t"
    //"out 0x22,r1\n\t"
    //"mov r1,r24\n\t"
    //"in r1,0x22\n\t"
  //    "in r21,3\n\t" // PINB // bits 6&7 cleared
    //  "in r20,3\n\t" // PINB // bits 6&7 cleared
    //);
//  PORTB|=0x7E;  
//OCR0A=45;
/*  if(ist==0)
  {
      PORTB&=~(1<<7); 
      PORTB|=(1<<6);
      ist=1;
  }
  else
  {
      PORTB&=~(1<<6); 
      PORTB|=(1<<7);
      ist=0;    
  }*/
  
//  if(ist==0)
//  if(TCNT0&1)
  //{
//      PORTB|=(1<<6);
    //  NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP;  NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP;   NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP; // NOP; NOP;
  //    PORTB&=~(1<<6); 
//      PORTB|=(1<<7);
  //    NOP; NOP; NOP; NOP; NOP; NOP;      //NOP; //NOP; NOP; //NOP; NOP; NOP;
    //  PORTB&=~(1<<7); 
  //    ist=1;
//  }
 //  else if(TCNT0&3==2)
  //{
    //  PORTB|=(1<<7);
      //NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP;  NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP;  // NOP; NOP; NOP; NOP; //NOP; NOP; NOP; NOP;  
      //PORTB&=~(1<<7); 
//      PORTB|=(1<<7);
  //    NOP; NOP; NOP; NOP; NOP; NOP;      //NOP; //NOP; NOP; //NOP; NOP; NOP;
    //  PORTB&=~(1<<7); 
  //    ist=1;
  //}
//  else
  //{
  
    //  ist=0;    
//  }
  
  
/*
  PORTB|=(1<<6);  //__asm__ __volatile__("sbi 5,6\n\t");
//  __asm__ __volatile__("push r1\n\t");//2
  //__asm__ __volatile__("in r1,0x3F\n\t");//1
                         //    NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP; 
                             NOP; NOP; NOP; NOP; //NOP; NOP; NOP; NOP; 
                           //  NOP; NOP; NOP; NOP; NOP; NOP; NOP; //NOP; 
                            // NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP; 

  PORTB&=~(1<<6); 
  PORTB|=(1<<7);
                       //      NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP; 
                             NOP; NOP; NOP; NOP; //NOP; NOP; NOP; NOP;  
                             //NOP; NOP; NOP; NOP; NOP; NOP; NOP; //NOP; 
                          //   NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP; 
//  __asm__ __volatile__("out 0x3F,r1\n\t");//1
  //__asm__ __volatile__("pop r1\n\t");//2

                             PORTB&=~(1<<7); 
*/
//__asm__ __volatile__("reti\n\t");
  //PORTD ^= (1 << PORTD6);    // toggle PD6 
//} 

//ISR(TIMER2_COMPB_vect) { 
  //cb++;
  //RTB ^= (1 << PORTB0);    // toggle PB0 
//} 

//ISR(TIMER2_OVF_vect) { 
  //cc++;
  //RTD ^= (1 << PORTD7);    // toggle PD7 
//} 
//word LastVcc=0;
//ISR(ADC_vect){ ADCresult=ADCW; if(LastVcc<ADCresult) { if(ADCresult>260){SaveUptime();reboot();} } LastVcc=ADCresult; ADCfree=1;} // 1.25s delay after SaveUptime


uint16_t Flashes=0; // число вспышек в секунду

/*

uint16_t st1,st2,delta,flash_duration;
uint8_t flash_start_mask,flash_stop_mask; // channel for flash

uint8_t a1,b1,c1,d1,e1;

byte tqq;



//byte pinmask,prt;
//word max0=0,max1=0,max01=0,max10=0;
word Flashes=0; // число вспышек в секунду
long FlasheS=0; // общее число вспышек

void Flash(byte chips,byte Duration)
{
//  word lamps=chips*8;
  
//  Pin2Output(DDRD,7);Pin2HIGH(PORTD,7);  // G stop light
//  Pin2Output(DDRD,5);Pin2HIGH(PORTD,5);  // SRCLR to HIGH. when it is LOW all regs are cleared
//  Pin2Output(DDRB,1);
//  Pin2Output(DDRD,6); 
  
//  for(byte n=0;n<Duration;n++)
// {
    PORTB=0b00000010;//    Pin2HIGH(PORTB,1);  // DATAPIN to HIGH  
PORTD=0xF0;PORTD=0xB0;  
//Pin2HIGH(PORTD,6);Pin2LOW(PORTD,6); // clock pulse 
    PORTB=0;    //Pin2LOW(PORTB,1); // next data are zeroes

//  Pin2Output(DDRB,6);Pin2LOW(PORTB,6);  // test latch


//  delayMicroseconds(5);



// shift register is cleared
// what is in storage register we don't care because G is HIGH and all outputs are OFF

//  delayMicroseconds(2);

//t=0;

//  if((z&7)==0){Pin2HIGH(PORTD,1);} // 1st bit is "1"
//  Pin2HIGH(PORTD,1); // 1st bit is "1"

//byte pb=PORTB;
//byte pd=PORTD; //0xB0 10110000 pins 4,5&7 are set
//grr=pd;
//byte pd7set=PORTD;
//byte pd7setd6set=PORTD&0x40;
//byte pd7clear=pd7set&~0x80;
//byte pd67clear=pd7set&~0xC0;

//for(byte z=0;z<lamps;z++)// serie of flashes
//{
//PORTD=0xF0;PORTD=0xB0;  //  Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6); // clock pulse 


//PORTB=pb;
cli();

PORTD=0x70; delayMicroseconds(15); PORTD=0x30; //PORTD=0xB0;//NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(15); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(15); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(15); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(15); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(15); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(15); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(15); PORTD=0x30;// PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(15); PORTD=0x30;// PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0xB0;// restore pin7 high pin6 low



//PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
//PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
//PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
//PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
//PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
//PORTD=0x30; NOP; PORTD=0x30; PORTD=0xB0;


//PORTD=0xB0; //4,5,7


// Pin2LOW(PORTD,7);  PORTB=pb;  Pin2HIGH(PORTD,7); 
// Pin2LOW(PORTD,7);  PORTB=pb;  Pin2HIGH(PORTD,7); 
// Pin2LOW(PORTD,7);  PORTB=pb;  Pin2HIGH(PORTD,7); 


sei();



//}// for
//}

//  Pin2LOW(PORTD,5);Pin2Input(DDRD,5);  // SRCLR to LOW. Clear regs
//  Pin2Input(DDRB,1);  // DATAPIN
//  Pin2Input(DDRD,6); // CLK&LATCH 
//  Pin2LOW(PORTD,7);Pin2Input(DDRD,7);  // detach G control pin (it is pull upped by resistor) --- no so really
}



// charging capacitor  on D2 pin. when it is discharged   to logic "0" INT0 on low level is fired
//===========================================================================================

byte pin2_interrupt_flag=0;
byte pin3_interrupt_flag=0;
byte pin0_interrupt_flag=0;
byte pin7_interrupt_flag=0;


//ISR (BAD_vect)
//{
//  bbb++;
//}

//boolean volatile UpdateRTC=true; // Update RTC at least once (to reset it if needed)
//boolean volatile Update500=true; // Update once in 500ms

ISR (PCINT1_vect)  // A3
{ 
//  PINC&(1<<3)
//    if((++ticks&0x1)==0){UpdateRTC=true;}// update clock every second
//    Update500=true;
    
    // alarms ?
} 


void pin2_isr()
{
  cnt1=TCNT1;
  detachInterrupt(0);  //INT 0
//  sleep_disable();// a bit later
  pin2_interrupt_flag = 1;
}
void pin3_isr()
{
  cnt1=TCNT1;
  detachInterrupt(1); //INT 1
//  sleep_disable();// a bit later
  pin3_interrupt_flag = 1;
}

word VH(void)
{
  word t1;
    
//    ADCSRA|=(1<<ADEN); //turn on ADC    
//    SetADC(1,8,500);  //  select temperature sensor 352 (need calibration)  
//    mRawADC(t1,2);
//    mRawADC(t1,2);
    
    SetADC(0,2,500);

    mRawADC(t1,2);
    mRawADC(t1,2);

    ADCoff;
//    ADCSRA&=~(1<<ADEN); //turn off ADC 
//    ACSR = (1<<ACD); // turn off analog comparator    
    return t1;
}

word Vcc(void)
{
  word t1;
    // measure with ADC inner voltage
//    ADCSRA|=(1<<ADEN); //turn on ADC    
//    SetADC(1,8,500);  //  select temperature sensor 352 (need calibration)  
//    mRawADC(t1,2);
//    mRawADC(t1,2);
    
    SetADC(0,14,500);

    mRawADC(t1,2);
    mRawADC(t1,2);

    ADCoff;
//    ADCSRA&=~(1<<ADEN); //turn off ADC 
//    ACSR = (1<<ACD); // turn off analog comparator    
    return t1;
}
word Tmp(void)
{
  word t1;
    // measure with ADC inner voltage
//    ADCSRA|=(1<<ADEN); //turn on ADC    
    SetADC(1,8,500);  //  select temperature sensor 352 (need calibration)  

    mRawADC(t1,2);
    mRawADC(t1,2);

//    ADCSRA&=~(1<<ADEN); //turn off ADC 
  ADCoff;
//    ACSR = (1<<ACD); // turn off analog comparator    
    return t1;
}

word tc1;
word sc[16];
word mn=5555,mx=5000;
uint16_t t1,t2,tt1,tt2,tt3,ttt1,ttt2;

word it=0;
byte rnd;
//long flashes=0;
long ln=0;
long fn=0;
word tim;
word fnt,lnt; // fast/long nap time

void unap(byte timeout)
{

  // Setup the WDT 
  cli();
  __asm__ __volatile__("wdr\n\t");//  wdt_reset();
  MCUSR &= ~(1<<WDRF);  // Clear the reset flag. 
  WDTCSR |= (1<<WDCE) | (1<<WDE); //  In order to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles).
//  WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s
  WDTCSR = ((1<<WDIE) | timeout);
  sei();

            WDflag=0;
            WDsleep=1;// notify WD that we are sleeping (to avoid reboot)
        sleeps=0;
      do{
        sleep_enable();
        sleep_cpu();
//wake up here
// check if it us or not
        sleep_disable();
        if(WDflag){break;}else{sleeps++;}
      }while(1);
}

void fastnap(void)
{
      word prev;

Pin2Output(DDRD,2);Pin2HIGH(PORTD,2);// charge cap
//delayMicroseconds(2);

      WDflag=0;
      sleeps=0;
      prev=TCNT1;
      TCNT1=0;
      do{
            cli();
            pin2_interrupt_flag=0;
            sleep_enable();
            attachInterrupt(0, pin2_isr, LOW);

            Pin2LOW(PORTD,2);Pin2Input(DDRD,2); // controlled charging (very impurtant set it 2 input (high impedance state))
            sei();
            sleep_cpu();
//wake up here
            sleep_disable();
  //      if(pin3_interrupt_flag||WDhappen){break;}else{sleeps++;}
  // if wakeup by WD then reboot?
            if(pin2_interrupt_flag||WDflag){break;}else{sleeps++;} // WD is important in case  RC sleeper went off
          }while(1);
          TCNT1+=prev;
//  cli();t1111=TCNT1;sei();//atomic read
}
void longnap(void)
{
      word prev;
      
      Pin2Output(DDRD,3);Pin2HIGH(PORTD,3);// charge cap

      WDflag=0;
      sleeps=0;
      prev=TCNT1;
      TCNT1=0;
      do{
            cli();
            pin3_interrupt_flag=0;
            sleep_enable();
            attachInterrupt(1, pin3_isr, LOW);

            Pin2LOW(PORTD,3);Pin2Input(DDRD,3); // controlled charging (very impurtant set it 2 input (high impedance state))
            sei();
            sleep_cpu();
//wake up here
            sleep_disable();
  //      if(pin3_interrupt_flag||WDhappen){break;}else{sleeps++;}
            if(pin3_interrupt_flag||WDflag){break;}else{sleeps++;} // WD is important in case  RC sleeper went off
          }while(1);
          TCNT1+=prev;
//  cli();t1111=TCNT1;sei();//atomic read
}

/*
void NiceBack(byte x1,byte y1,byte x2,byte y2)
{
  byte r,g,b;
  
  fillScreen(0x000000);

  setAddrWindow(y1,x1,y2,x2);
  
  Pin2HIGH(PORTD,4); 
//  Pin2LOW(PORTD,1);

  for(byte j=x1;j<x2;j++)
  {  
  for(byte i=y2+1;i>0;i--)
  {
      r=(i<<4);    g=(i<<3);      b=(i<<4);
  
      spiwrite(r);
      spiwrite(g);
      spiwrite(b);
    }
  }
//  Pin2HIGH(PORTD,1);
  
  setAddrWindow(144,0,167,127);
  
  Pin2HIGH(PORTD,4); 
//  Pin2LOW(PORTD,1);

  for(byte j=0;j<128;j++)
  {  
  for(byte i=0;i<16;i++)
  {
      r=(i<<4);    g=(i<<4);      b=(i<<2);
  
      spiwrite(r);
      spiwrite(g);
      spiwrite(b);
    }
  }
//  Pin2HIGH(PORTD,1);
  
//  fillRect(16,32,48,64,0);
//  fillRect(0,16,128,96,0);

}


void DrawBox(byte x,byte y, byte x2,byte y2,byte r,byte g,byte b)
{
    setAddrWindow(x,y,x2,y2); Pin2HIGH(PORTD,4); for (word k=0;k<((x2-x+1)*(y2-y+1)*3);k++){spiwrite(r);spiwrite(g);spiwrite(b);}             
}
*/
void ShowBars(void)
{
  uint8_t r,g,b;

//  setAddrWindow(70,4,77,123);
  //ta("hr:");t3(hr);

  

  
  // m max=11
  //16
  // *15/m
  
  // gradient background
//  setAddrWindow(0,4,15,123);
//NiceBack(0,0,128,15);
//NiceBack(0,16,127,31);
/*
  setAddrWindow(0,0,15,127);

  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1);

  for(byte j=0;j<128;j++)
  {  
  for(byte i=16;i>0;i--)
  {
      r=(i<<4);    g=(i<<3);      b=(i<<4);
  
      spiwrite(r);
      spiwrite(g);
      spiwrite(b);
    }
  }
  Pin2HIGH(PORTD,1);
*/
  // bars
  for(uint8_t i=0;i<16;i++)
  {
//    gg=(Intensity[i]*14)/mi;if(!gg){gg=1;} 
    uint8_t gg=Intensity[i];//if(!gg){gg=1;} 

    if(i!=HR){r=0x8c;g=0xac;b=0x8c;}else{r=0x8c;g=0xfc;b=0x4c;}
  //  DrawBox(16-gg,i*8,15,i*8+6,r,g,b);    
  }
    //DrawBox(17,HR*8,17,HR*8+6,0x8c,0xfc,0x4c);    

}
/*
word fastnaptime;
word longnaptime;
word nextnaptime;

long sssn;
long sssb;

void SleepTime(void)
{
    fastnaptime=0;
    longnaptime=0;
    nextnaptime=0;
    
    set_sleep_mode(SLEEP_MODE_IDLE);
    WDsleep=1;// notify WD that we are sleeping (to avoid reboot)
    __asm__ __volatile__("wdr\n\t");//  wdt_reset(); // to avoid WD fire first
    fastnap();
    if (pin2_interrupt_flag){fastnaptime=cnt1;}else { ERR=ERR_BROKEN_SLEEP; }

    WDsleep=1;// notify WD that we are sleeping (to avoid reboot)
    __asm__ __volatile__("wdr\n\t");//  wdt_reset(); // to avoid WD fire first
    longnap();
    if (pin3_interrupt_flag){longnaptime=cnt1;}else { ERR=ERR_BROKEN_SLEEP; }

//    WDsleep=1;// notify WD that we are sleeping (to avoid reboot)
    //__asm__ __volatile__("wdr\n\t");//  wdt_reset(); // to avoid WD fire first
  //  nextnap();
    //if (pin0_interrupt_flag){nextnaptime=cnt1;}else { ERR=ERR_BROKEN_SLEEP; }
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}*/
/*
boolean Touched(void)
{
    byte x,e=2;

    for (x=0;x<3;x++){ delay(700); Cherry(15+x*36,100); }
    do        
    {
        for (x=0;x<4;x++){ if (TouchSensor()>Etouch){ return true; } delay(200); }
     //   DrawBox(100,15+e*30,123,15+23+e*36,0x00,0x00,0x00);    
         if(e==0){break;}
         e--;
     }while(1);
     return false;
}*/
/*
void Settings(void)
{
    
//    if(TFT_IS_ON)  // а как иначе? :)
  //  {
       
        NiceBack(0,0,128,15);
        setAddrWindow(16,0,16+7,127);ta("Настройки * Settings");
        setAddrWindow(24,0,24+7,127);ta("====================");
        
        setAddrWindow(50,15,50+7,127);ta("Тестовый режим?");setAddrWindow(70,15,70+7,127);ta("Set test mode ?");
        
        // кубики
        if (Touched()){ 
          
        FlashTest();
        DrawBox(16,0,159,127,0x00,0x00,0x00);    // очистка экрана
     
        return; } // переход в тестовый режим

//        setAddrWindow(60,0,60+7,119);ta(" false");
//        delay(5000);
//    }
}

void Cherry(byte x,byte y)
{
    word pos=0;
    byte r,g,b,z;
    setAddrWindow(y,x,y+23,x+23);
    Pin2HIGH(PORTD,4);
    do
    {
        for(z=0;z<24;z++)
        {
            b=pgm_read_byte(&(C24[pos++]));     
            g=pgm_read_byte(&(C24[pos++]));     
            r=pgm_read_byte(&(C24[pos++]));     
            spiwrite(r); spiwrite(g); spiwrite(b);
        }//  pos+=2;// row padding
    }
    while (pos<(24*24*3));
}*/

/*
#define LONG_TOUCH_THRESHOLD 4 // критерий длинного нажатия в  секундах

  volatile long lvv,lmv,lm2;// luminous
  volatile long mlvv,mlmv,mlm2;// luminous
  volatile word lasttouch,ltp,CurrentTouch;
  byte LongTouch=0;
word rtcl;
word FT[5];

void UpdateScreen(void)
{
//    DrawBox(0,0,159,127,0x00,0x00,0x00);    // очистка экрана

    setAddrWindow(20,0,27,127);
  ta("ERR");th(ERR);
    ta(" CT");tn(1000,CurrentTouch);
    ta(" Ti");tn(100,ticks);
    ta(" L");tn(100,LCD);
    ta(" H");tn(10,HR);
//    th(rtc8);ta(" fn");tn(10000,fastnaptime);ta(" ln");tn(10000,longnaptime);
    setAddrWindow(30,0,37,127);
    
    ta("Et");tn(1000,Etouch);ta(" rtcl");tn(10000,rtcl);ta(" Fi");th(FlashIntensity);

    setAddrWindow(40,0,47,127);
    ta(" it");tn(10000,it);ta(" LT");th(LongTouch);   ta(" sl");tn(10,sleeps);ta(" ");th(grr);th(grr2);

    setAddrWindow(50,0,57,127);
    //ta("sss=");tn(100000,sss);
    for(byte w=1;w<9;w++){th(TimeS[w]);ta(":");}
    

//3.61v - 319 3.93v -291     5v~190

    setAddrWindow(60,0,67,127);
    
//    ta(" sss=");tn(1000,sss);    
//    ta(" bbb=");tn(1000,bbb);    
    ta(" if=");th((pin0_interrupt_flag<<5)|(pin2_interrupt_flag<2)|(pin3_interrupt_flag<<1)|(WDflag));
    ta("Vcc");tn(100,VccN);ta(" L");tn(100,VccL);ta(" H");tn(100,VccH);
    
    
    setAddrWindow(70,0,77,127);

    ta("Vcc");tn(100,114000L/VccN);ta(" L");tn(100,114000L/VccL);ta(" H");tn(100,114000L/VccH); // 1125300L
    ta(" VH=");tn(1000,VH());

    setAddrWindow(80,0,87,127);

ta(" f");tn(10000,fastnaptime);ta(" l");tn(10000,longnaptime);   ta(" n");tn(10000,nextnaptime);   

    setAddrWindow(90,0,97,127);
//    ta("ch0=");tn(1000,chan0);ta("ch1=");tn(1000,chan1);
    ta("ch0=");tn(1000,Light.W[0]);ta(" ch1=");tn(1000,Light.W[1]);
//    ta("ch0=");wh(Light.W[0]);ta(" ch1=");wh(Light.W[1]);

    LightMax0.W[0]=0;LightMax1.W[1]=0;

    setAddrWindow(100,0,107,127);    
    ta("0 ");tn(10000,LightMax0.W[0]);tn(10000,LightMax0.W[1]);
    ta(" 1 ");tn(10000,LightMax1.W[0]);tn(10000,LightMax1.W[1]);

  
//   ta("grr ");tn(100,grr);  ta(" ");th(grr);ta(" ");th(grr2);
  //  ta("FCPU ");tn(1000000,I2C_CPUFREQ);
//    ta("DC ");lh(I2C_DELAY_COUNTER);

    setAddrWindow(110,0,117,127);
    ta("uptime");tn(10000,uptime);
    setAddrWindow(120,0,127,127);
    ta(" fps");tn(1000,Flashes);
    ta(" ");tn(1000000000,FlasheS);

    setAddrWindow(130,0,137,127);
    tn(1000,FT[0]);ta(" ");    tn(1000,FT[1]);ta(" ");    tn(1000,FT[2]);ta(" ");    tn(1000,FT[3]);ta(" ");    tn(1000,FT[4]);

    setAddrWindow(140,0,147,127);

  TouchSample();
ta("Touch:");wh(TouchD[0]);wh(TouchD[1]);wh(TouchD[2]);wh(TouchD[3]);

    Cherry(103,108);
    */


    // 42x24
/*
    setAddrWindow(100,100,141,123);

   // setAddrWindow(x,y,x2,y2);
    Pin2HIGH(PORTD,4);
//  for (word k=0;k<((x2-x+1)*(y2-y+1)*3);k++){spiwrite(r);spiwrite(g);spiwrite(b);}             
word pos=0x36;
do
//for(word z=0;z<42*24*3;z++)
{
  for(byte z=0;z<42;z++)
  {
      byte b=pgm_read_byte(&(Bat[pos++]));     
      byte g=pgm_read_byte(&(Bat[pos++]));     
      byte r=pgm_read_byte(&(Bat[pos++]));     
       spiwrite(r);
        spiwrite(g);
         spiwrite(b);
  }
  pos+=2;// row padding
}
while(pos<(42*24*3+0x36));
*/
    
    

//    setAddrWindow(152,0,159,127);ta("-----==<АУРА>==-----3");

//}

word DHThum,DHTtmp;


boolean DHTreadAll(void) {
boolean  res=false;
  //if (DHTread()) {
if(read22()==DHTLIB_OK)
{
      DHThum = DHTdata[0];
      DHThum *= 256;
      DHThum += DHTdata[1];
      DHTtmp = (DHTdata[2] & 0x7F);
      DHTtmp *= 256;
      DHTtmp += DHTdata[3];
      res=true;
  }
  return res;
}

//uint8_t masks[8]={0,0x40,0x60,0x70,0x78,0x7C,0x7E,0x7F};  // filled
uint8_t masks[10]={0,0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01,0xFF}; // outline with t >29 is heavy selected
uint8_t tmin=0xFF;
uint8_t tmax=0;
uint8_t vmin=0xFF;
uint8_t vmax=0;

void TempBar(uint8_t v)
{
  uint8_t i,d;

    LcdSetPos(0,5);

    Pin2HIGH(PORTD,4); 
    Pin2LOW(PORTD,1); 

    for(i=75;i>0;i--){TempH[i]=TempH[i-1];}// shift right
    TempH[0]=v; // put new value

    for(i=0;i<76;i++){ d=0; if (TempH[i]>=21){d=TempH[i]-21;} if(d>9){d=9;} spiwrite(masks[d]);}// display graph [22..29]
    Pin2HIGH(PORTD,1);
}

uint8_t dot[5]={0,0x10,0x18,0x1C,0x1E};

void IntBar(void)
{
    uint8_t i,c;
    Pin2HIGH(PORTD,4); Pin2LOW(PORTD,1); 
    for (i=0;i<24;i++){ c=dot[Intensity[i]]; if (i==HR){c|=0x40;} spiwrite(c);}
    Pin2HIGH(PORTD,1);
}


void SoilBar(void)
{
    uint8_t i,b=moisture/23;  // 0..39 0..920 1 pixel=920/40=23

    Pin2HIGH(PORTD,4); 
    Pin2LOW(PORTD,1); 

    spiwrite(0x7C);
    for (i=0;i<b;i++){spiwrite(0x44);}
    for (i=b;i<41;i++){spiwrite(0x7C);}

    Pin2HIGH(PORTD,1);
}

//void GetVcc(void){ VccN=Vcc(); if (VccN<VccH){VccH=VccN;} if (VccN>VccL){VccL=VccN;} } //280us
uint8_t chip[6]={0x00,0x55,0x7F,0x7F,0x7F,0x55};
uint8_t degree[5]={0x00,0x06,0x09,0x09,0x06};
uint8_t del[2]={0x00,0x7F};

void LcdBack(void)
{

    LcdInit();
    LcdClear();

    LcdSetPos(0,0);ta("АуРа");
    LcdSetPos(0,1);ta("Яркость");     
    LcdSetPos(0,2);ta("Полив");     
//    LcdSetPos(0,3);ta("Влажность"); LcdSetPos(70,3);ta("%");
    LcdSetPos(0,3);ta("Влажн"); LcdSetPos(44,3);ta("%");
//    LcdSetPos(0,4);ta("Температура"); LcdSetPos(70,4); ta("+"); // будем оптимистами
    
    LcdSetPos(0,4);ta("t");spiout(&degree[0],5);ta("C"); LcdSetPos(44,4); ta("+"); // будем оптимистами
    FanIcon(0);
}

//void SaveUptime(void){eeprom_update_byte((byte*)1,(uptime>>8));eeprom_update_byte((byte*)2,(uptime&0xFF));}

uint8_t Fan0[7]={0x0E,0x6A,0x5C,0x77,0x1D,0x2B,0x38};
uint8_t Fan1[7]={0x0E,0x6E,0x7C,0x7F,0x1F,0x3B,0x38};

void spiout(uint8_t *s,uint8_t len){Pin2HIGH(PORTD,4); Pin2LOW(PORTD,1); for (uint8_t i=0;i<len;i++){spiwrite(s[i]);}Pin2HIGH(PORTD,1);}

void FanIcon(uint8_t t)
{
    uint8_t *icon=&Fan0[0];
    if (t){icon=&Fan1[0];}
  
    LcdSetPos(77,5);//combine them?
    spiout(icon,7);
//    Pin2HIGH(PORTD,4); 
  //  Pin2LOW(PORTD,1); 
    //for (uint8_t i=0;i<7;i++){spiwrite(icon[i]);}
//    Pin2HIGH(PORTD,1);
}


//word co1=0;


//  timer0_millis+=3651351L; //3600000L;
//  timer0_millis+=3554700L; //3600000L;
//  timer0_millis+=3620000L; //3600000L; // чуток бегут
//  timer0_millis+=3625000L; //3600000L;//чуток бегут
//  timer0_millis+=3630000L; //+19с за 1ч40

//  timer0_millis+=3637000L; //3600000L; //1s 40min
//  timer0_millis+=3637500L; //3600000L; 7s 1:39 ahead
  //timer0_millis+=3638500L; //3600000L; +25s 2:38
//  timer0_millis+=3636000L;  //  +10s 01:03
//  timer0_millis+=3646000L; //  +3s 1:43
//  timer0_millis+=3647000L;  +2s 1:48
//  timer0_millis+=3648500L; //   -44s 10:09
//  timer0_millis+=3647800L; // -5s 3:36

/*
ISR(TIMER0_OVF_vect)
{
//  timer0_overflow_count++;
  NOP;
}*/

//void*() df=Delay1;
//void eeprom_update_byte(byte*addr,byte v){ if(eeprom_read_byte((byte*)addr)!=v){eeprom_write_byte((byte*)addr,v);}}
void AddMillis(long a)
{
    cli();
    timer0_millis+=a; //
//    if(timer0_millis>=24*MILS){timer0_millis-=(24*MILS);} // overlap 24h    // later
//    LastTimeFan=timer0_millis; // чтобы не жужжал когда часы переводим
    sei();
}
// the loop routine runs over and over again forever:
//byte last_checked;
uint8_t button_is_pressed=0;
uint8_t prevMN=0xFF;
long milli;
long whh;
uint8_t MN;
uint8_t c;

long mi;

void x0(void)
{
    __asm__ __volatile__(
//"Start1:\n\t"
      "in r18,3\n\t" // r18=PINB (6OFF 7OFF)
      "mov r19,r18\n\t"
      "mov r20,r18\n\t"
      "ori r19, 0b01000000\n\t" // bit 6 is ON
      "ori r20, 0b10000000\n\t" // bit 7 is ON
"000:\n\t"    
// 1st===============      
      "cli\n\t"
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
//      "out 5,r19\n\t" // set pin 6 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t"//      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 2nd===============
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
//      "out 5,r19\n\t" // set pin 6 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t"//      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "lds r24,Flashes\n\t" "lds r25,Flashes+1\n\t""nop\n\t"//      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 3rd===============
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF//      "out 5,r19\n\t" // set pin 6 ON
      "adiw r24,1\n\t""sts Flashes+1,r25\n\t""nop\n\t"
//      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
"nop\n\t"
      "sts Flashes,r24\n\t" //Flashes++; 
       "in r22,6\n\t" // check pinA3: 0:LOW (is pressed) >0:HIGH (is not pressed)
       "nop\n\t""nop\n\t"
// 4th=============== absent here
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
//      "out 5,r19\n\t" // set pin 6 ON
      "sbrs r22,3\n\t" // следующая инструкция выполнится только если бит 3 в r22 сброшен (то есть кнопка нажата и лапка A3 притянута к земле)
      "sts button_is_pressed,r20\n\t"  // надо сохранить что нибудь >0!!! r20 точно > 0
      "nop\n\t"
//      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "nop\n\t"//      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t"//"nop\n\t"
           "sei \n\t"   //   "ldi r23,3\n\t"       "1:\n\t"      "dec r23\n\t"       "brne 1b\n\t" // wait with both OFF
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
     "sbrs r25,7\n\t" // следующая инструкция выполнится только если бит 7 в r25 сброшен (Flashes<32768)// 1004ms - то что нужно
//     "sbrs r25,6\n\t" // следующая инструкция выполнится только если бит 7 в r25 сброшен (Flashes<16384))// 1004ms - то что нужно
//     "sbrs r25,4\n\t" // следующая инструкция выполнится только если бит 7 в r25 сброшен (Flashes<4096))// 1004ms - то что нужно
  //     "sbrs r25,2\n\t" // следующая инструкция выполнится только если бит 7 в r25 сброшен (Flashes<102416384))// 1004ms - то что нужно
    "rjmp 000b\n\t"
      "wdr\n\t" // проведаем сторожевого пса
      );
  
}

void x1(void)
{
    __asm__ __volatile__(
//"Start1:\n\t"
      "in r18,3\n\t" // r18=PINB (6OFF 7OFF)
      "mov r19,r18\n\t"
      "mov r20,r18\n\t"
      "ori r19, 0b01000000\n\t" // bit 6 is ON
      "ori r20, 0b10000000\n\t" // bit 7 is ON
"111:\n\t"    
// 1st===============      
      "cli\n\t"
      "out 5,r19\n\t" // set pin 6 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 2nd===============
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
//      "out 5,r19\n\t" // set pin 6 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t"//      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "lds r24,Flashes\n\t" "lds r25,Flashes+1\n\t""nop\n\t"//      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 3rd===============
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF//      "out 5,r19\n\t" // set pin 6 ON
      "adiw r24,1\n\t""sts Flashes+1,r25\n\t""nop\n\t"
//      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
"nop\n\t"
      "sts Flashes,r24\n\t" //Flashes++; 
       "in r22,6\n\t" // check pinA3: 0:LOW (is pressed) >0:HIGH (is not pressed)
       "nop\n\t""nop\n\t"
// 4th=============== absent here
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
//      "out 5,r19\n\t" // set pin 6 ON
      "sbrs r22,3\n\t" // следующая инструкция выполнится только если бит 3 в r22 сброшен (то есть кнопка нажата и лапка A3 притянута к земле)
      "sts button_is_pressed,r20\n\t"  // надо сохранить что нибудь >0!!! r20 точно > 0
      "nop\n\t"
//      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "nop\n\t"//      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t"//"nop\n\t"
           "sei \n\t"   //   "ldi r23,3\n\t"       "1:\n\t"      "dec r23\n\t"       "brne 1b\n\t" // wait with both OFF
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
     "sbrs r25,7\n\t" // следующая инструкция выполнится только если бит 7 в r25 сброшен (Flashes<32768)// 1004ms - то что нужно
      "rjmp 111b\n\t"
  
  
    // 8000 bit 7 in r25
          
//"Check1:\n\t"
      "wdr\n\t" // проведаем сторожевого пса
      );
  
}

void x2(void)
{
    __asm__ __volatile__(
//"Start3:\n\t"
      "in r18,3\n\t" // r18=PINB (6OFF 7OFF)
      "mov r19,r18\n\t"
      "mov r20,r18\n\t"
      "ori r19, 0b01000000\n\t" // bit 6 is ON
      "ori r20, 0b10000000\n\t" // bit 7 is ON
"222:\n\t"    
// 1st===============      
      "cli\n\t"
      "out 5,r19\n\t" // set pin 6 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 2nd===============
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
//      "out 5,r19\n\t" // set pin 6 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t"//      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "lds r24,Flashes\n\t" "lds r25,Flashes+1\n\t""nop\n\t"//      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 3rd===============
      "out 5,r19\n\t" // set pin 6 ON
      "adiw r24,1\n\t""sts Flashes+1,r25\n\t""nop\n\t"
      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "sts Flashes,r24\n\t" //Flashes++; 
       "in r22,6\n\t" // check pinA3: 0:LOW (is pressed) >0:HIGH (is not pressed)
       "nop\n\t""nop\n\t"
// 4th=============== absent here
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
//      "out 5,r19\n\t" // set pin 6 ON
      "sbrs r22,3\n\t" // следующая инструкция выполнится только если бит 3 в r22 сброшен (то есть кнопка нажата и лапка A3 притянута к земле)
      "sts button_is_pressed,r20\n\t"  // надо сохранить что нибудь >0!!! r20 точно > 0
      "nop\n\t"
//      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "nop\n\t"//      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t"//"nop\n\t"
           "sei \n\t"   //   "ldi r23,3\n\t"       "1:\n\t"      "dec r23\n\t"       "brne 1b\n\t" // wait with both OFF
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF

     "sbrs r25,7\n\t" // следующая инструкция выполнится только если бит 7 в r25 сброшен (Flashes<32768)// 1004ms - то что нужно
      "rjmp 222b\n\t"
  
  
    // 8000 bit 7 in r25
          
//"Check:\n\t"
      "wdr\n\t" // проведаем сторожевого пса
      );
  
}
void x3(void)
{
    __asm__ __volatile__(
//"Start3:\n\t"
      "in r18,3\n\t" // r18=PINB (6OFF 7OFF)
      "mov r19,r18\n\t"
      "mov r20,r18\n\t"
      "ori r19, 0b01000000\n\t" // bit 6 is ON
      "ori r20, 0b10000000\n\t" // bit 7 is ON
"333:\n\t"    
// 1st===============      
      "cli\n\t"
      "out 5,r19\n\t" // set pin 6 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 2nd===============
      "out 5,r19\n\t" // set pin 6 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "lds r24,Flashes\n\t" "lds r25,Flashes+1\n\t""nop\n\t"//      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 3rd===============
      "out 5,r19\n\t" // set pin 6 ON
      "adiw r24,1\n\t""sts Flashes+1,r25\n\t""nop\n\t"
      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "sts Flashes,r24\n\t" //Flashes++; 
       "in r22,6\n\t" // check pinA3: 0:LOW (is pressed) >0:HIGH (is not pressed)
       "nop\n\t""nop\n\t"
// 4th=============== absent here
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
//      "out 5,r19\n\t" // set pin 6 ON
      "sbrs r22,3\n\t" // следующая инструкция выполнится только если бит 3 в r22 сброшен (то есть кнопка нажата и лапка A3 притянута к земле)
      "sts button_is_pressed,r20\n\t"  // надо сохранить что нибудь >0!!! r20 точно > 0
      "nop\n\t"
//      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "nop\n\t"//      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t"//"nop\n\t"
           "sei \n\t"   //   "ldi r23,3\n\t"       "1:\n\t"      "dec r23\n\t"       "brne 1b\n\t" // wait with both OFF
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
     "sbrs r25,7\n\t" // следующая инструкция выполнится только если бит 7 в r25 сброшен (Flashes<32768)// 1004ms - то что нужно
      "rjmp 333b\n\t"
  
  
    // 8000 bit 7 in r25
          
//"Check:\n\t"
      "wdr\n\t" // проведаем сторожевого пса
      );
  
}

void x4(void)
{
    __asm__ __volatile__(
//"Start4:\n\t"
      "in r18,3\n\t" // r18=PINB (6OFF 7OFF)
      "mov r19,r18\n\t"
      "mov r20,r18\n\t"
      "ori r19, 0b01000000\n\t" // bit 6 is ON
      "ori r20, 0b10000000\n\t" // bit 7 is ON
"444:\n\t"    
// 1st===============      
      "cli\n\t"
      "out 5,r19\n\t" // set pin 6 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 2nd===============
      "out 5,r19\n\t" // set pin 6 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "lds r24,Flashes\n\t" "lds r25,Flashes+1\n\t""nop\n\t"//      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
// 3rd===============
      "out 5,r19\n\t" // set pin 6 ON
      "adiw r24,1\n\t""sts Flashes+1,r25\n\t""nop\n\t"
      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "sts Flashes,r24\n\t" //Flashes++; 
       "in r22,6\n\t" // check pinA3: 0:LOW (is pressed) >0:HIGH (is not pressed)
       "nop\n\t""nop\n\t"
// 4th===============
      "out 5,r19\n\t" // set pin 6 ON
      "sbrs r22,3\n\t" // следующая инструкция выполнится только если бит 3 в r22 сброшен (то есть кнопка нажата и лапка A3 притянута к земле)
      "sts button_is_pressed,r20\n\t"  // надо сохранить что нибудь >0!!! r20 точно > 0
      "nop\n\t"
//      "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
      "out 5,r20\n\t" // set pin 6 OFF pin7 ON
      "nop\n\t""nop\n\t""nop\n\t""nop\n\t"//"nop\n\t"
           "sei \n\t"   //   "ldi r23,3\n\t"       "1:\n\t"      "dec r23\n\t"       "brne 1b\n\t" // wait with both OFF
          "out 5,r18\n\t" // set pin 6 OFF pin7 OFF
     "sbrs r25,7\n\t" // следующая инструкция выполнится только если бит 7 в r25 сброшен (Flashes<32768)// 1004ms - то что нужно
      "rjmp 444b\n\t"
  
  
    // 8000 bit 7 in r25
          
//"Check:\n\t"
      "wdr\n\t" // проведаем сторожевого пса
      );
  
}

uint32_t NextFanTime=0; // время следующего включения. не раньше.
uint32_t StopFanTime=0; // время выключения вентилятора
uint8_t FanIsON=0;
void FanON(uint8_t d){Pin2Output(DDRB,0);Pin2HIGH(PORTB,0);StopFanTime=milli+(d<<10);FanIsON=1;FanIcon(1);}
void FanOFF(uint8_t t){Pin2LOW(PORTB,0);Pin2Input(DDRB,0);NextFanTime=milli+(t<<10);FanIsON=0;FanIcon(0);}

void loop() {
//while(1){

//  if(ADCready)
  //{
    //  if (ADCresult>260){SaveUptime(); delay(3000);}// wait till power off (after 2s watchdog will inforce reboot)
      //ADCready=0;
//  }
      //  __asm__ __volatile__("wdr\n\t");//  wdt_reset();
  //      LcdSetPos(0,3);tn(100000,ca);   //tn(100000,cb);//   tn(10000,cc);
    //  ca=0;//cb=0;cc=0;  
//       delay(1000); 
     //  cb=ca;
// __asm__ __volatile__("wdr\n\t");//  wdt_reset();

// Flash here

// FlashIntensity здесь постоянна

/*Call-Used Registers The call-used or call-clobbered general purpose registers (GPRs) are registers that might be destroyed (clobbered) by a function call
R18–R27, R30, R31
These GPRs are call clobbered. An ordinary function may use them without restoring the contents. Interrupt service routines (ISRs) must save and restore each register they use.
R0, T-Flag
The temporary register and the T-flag in SREG are also call-clobbered, but this knowledge is not exposed explicitly to the compiler (R0 is a fixed register).
Call-Saved Registers
R2–R17, R28, R29
The remaining GPRs are call-saved, i.e. a function that uses such a registers must restore its original content. This applies even if the register is used to pass a function argument.
R1
The zero-register is implicity call-saved (implicit because R1 is a fixed register).*/
//button_is_pressed=TCNT0;

          __asm__ __volatile__("Start:\n\t");

      if (FlashIntensity==4){x4();}
      else if (FlashIntensity==3){x3();}
      else if (FlashIntensity==2){x2();}
      else if (FlashIntensity==1){x1();}
      else {x0();}
      
  // if (InitialFreeRAM<freeRam()){reboot();}

    if(button_is_pressed) { button_is_pressed=0;AddMillis(MILS); } // плюс час если кнопка A3 нажата
   
      cli();milli=timer0_millis;sei();
    
      HR=milli/MILS; 
      if (HR!=prevHR)
      {
     //     if (HR==24){HR=0;cli();timer0_millis=0;milli=0;sei();}// fix timer0_millis
          if (HR==24){reboot();}// перезагрузка в полночь

          prevHR=HR;
          whh=HR*MILS; // остаток секунд в часе
//          FlashIntensity=decode[Intensity[HR]]; // текущая интенсивность освещения
          FlashIntensity=Intensity[HR]; // текущая интенсивность освещения
          LcdSetPos(65,0);tn(10,HR);
          LcdSetPos(60,1);IntBar();
          LcdSetPos(55,1);tc(Intensity[HR]);

          SoilMoisture();  LcdSetPos(30,2); SoilBar(); LcdSetPos(72,2); tn(100,moisture);
          
      }

      MN=(milli-whh)/(MILS/60); 
      if (MN!=prevMN)
      {
          prevMN=MN;LcdSetPos(76,0);tn(10,MN);

         if (DHTreadAll())
         {
           DHThum=(DHThum+5)/10; if(DHThum>vmax){vmax=DHThum;} if(DHThum<vmin){vmin=DHThum;}
           DHTtmp=(DHTtmp+5)/10;  if(DHTtmp>tmax){tmax=DHTtmp;} if(DHTtmp<tmin){tmin=DHTtmp;}
           
            LcdSetPos(50,3);tn(10,DHThum); LcdSetPos(66,3); tn(10,vmin);spiout(&del[0],2);tn(10,vmax);
          //  LcdSetPos(70,5); char* cc; if (DHTdata[2] & 0x80){cc="-";}else{cc="+";}
        //    ta(cc);
            LcdSetPos(50,4);tn(10,DHTtmp); LcdSetPos(66,4); tn(10,tmin);spiout(&del[0],2);tn(10,tmax);
            // иногда нужен ветерок CО2 свежего подкачать (c 6 утра)

            
            
            TempBar(DHTtmp);

//            if ((!FanTimeout)&&(!RunningFan)){if ((DHTtmp>=29)||(DHThum>=50)||(milli-LastTimeFan)>1800000){if(DHTtmp>=31){delay(60000);}else if(HR>=0){FanON(55);}}}
  //          if ((!FanTimeout)&&(!RunningFan)){if ((DHTtmp>=29)||(DHThum>=50)||(milli-LastTimeFan)>3600000){if(DHTtmp>=32){for(uint8_t d=0;d<255;d++){delayMicroseconds(65000); } }else if(HR>=6){FanON(61);}}}
            if (milli>NextFanTime) { if ((DHTtmp>=28)||(DHThum>=65)){FanON(120);} }
//            if ((!FanTimeout)&&(!RunningFan)){if ((DHTtmp>=28)||(DHThum>60)){FanON(90);}}
            if(DHTtmp>=32){ FlashIntensity=0; } // если припекает то выключаем лампы до следующего начала часа. Пусть остывает.
      //16s
      //for(i=0;i<4;i++){  delayMicroseconds(65000); } //260ms
         }

      SetADC(1,8,500);  //  select inner temperature sensor 
       mRawADC(MCUtemp,2); // прочитаем заодно и этот датчик
    
      SetADC(0,14,500); // select bandgap voltage
      mRawADC(MCU_Vcc,2);

      byte CHIPtemp=MCUtemp/12; // 11.68 better
           if(CHIPtemp>40){reboot();}
           LcdSetPos(32,0);tn(10,CHIPtemp); spiout(&degree[0],5);
           tf(10,11200/MCU_Vcc,1); ta("в");
    
          if (FanIsON==0) // зачем включать уже включенный вентилятор
          {
              if((MN==1)||(MN==21)||(MN==41)){ FanON(15); } // каждые 20 минут маленькое проветривание
          }

      }// next minute

      LcdSetPos(74,0);c=0;if(milli&1024){c=0x36;}Pin2HIGH(PORTD,4);Pin2LOW(PORTD,1);spiwrite(c);spiwrite(c);Pin2HIGH(PORTD,1); // flip flop - анимация часов
//      if(FanTimeout){FanTimeout--;} else if(RunningFan){if((--RunningFan)==0){FanOFF(32);LastTimeFan=milli;}}
      if ((FanIsON)&&(milli>=StopFanTime)){FanOFF(30);}
//      if(FanTimeout){FanTimeout--;} else if(RunningFan){if((--RunningFan)==0){FanOFF(32);}}

      Flashes=0;

//word t1=TCNT1;   LcdSetPos(23,2);tn(10000,t1);       
//       LcdSetPos(32,2);tn(100000000,timer0_millis);//ta("-");th((timer0_millis&0xff));

/* useless still 1.1w
      //else { // nothing to do/ sleep time
if(      FlashIntensity==0)
{
  //    #define watchdogSleep(mode,timeout){setup_watchdog(timeout);system_sleep(mode,WDhappen);setup_watchdog(T2S);}

  WDsleep=1;

  // Setup the WDT 
  cli();
    __asm__ __volatile__("wdr\n\t");//  wdt_reset();
  MCUSR &= ~(1<<WDRF);  // Clear the reset flag. 
  WDTCSR |= (1<<WDCE) | (1<<WDE); //  In order to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles).
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//15ms (16280us)
  // WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//30ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (0<<WDP0);//60ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0);//120ms
  WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0);//240ms
//  WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);//480ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);//960ms
//  WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);//2s
  // WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//4s
  //   WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s


  sei();
  set_sleep_mode (SLEEP_MODE_IDLE);
  //  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  WDhappen=0;
  sleeps=0;
  ss=0;
  //TCNT1=0;
  do{
    sleep_cpu();
    sleeps++;
    if (digitalRead(A3)){ss++;}
   //   x0();
    if(WDhappen||ss){break;}
  }
  while(1); // 9 times within 16ms
  // wake up here
  sleep_disable();
//  wdt_disable();
setup_watchdog(T2S); // если в течении 2s не сбросить сторожевого пса то перезагрузка. (защита от зависаний)
//    watchdogSleep(SLEEP_MODE_IDLE,T500MS);
//delay(470);

//if(PINC&&(1<<3)){button_is_pressed++;}
if(ss>1){button_is_pressed=1;}
//if(digitalRead(A3)){button_is_pressed=1;}
//    __asm__ __volatile__("wdr\n\t");
  //  setup_watchdog(T2S);
    
//      for(uint8_t d=0;d<15;d++){delayMicroseconds(65000); }  //~1s
      } 
*/
    
    __asm__ __volatile__("rjmp Start\n\t");
}


