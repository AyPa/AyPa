//#define __AVR_ATmega328P__ 
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <Arduino.h>



#include <SPI.h>


//#include <SoftI2CMaster.h>

#include "AyPa_m.h"
#include "AyPa_fonts.h"
//#include "AyPa_n.h"
#include "AyPa_TFT.h"
//#include "AyPa_i2c.h"
//#include "AyPa_rtc.h"


#define reboot {__asm__ __volatile__ ("rcall 0\n\t" );} //void(* resetFunc) (void) = 0; //declare reset function @ address 0

//#include <Wire.h>
//#include "TSL2561.h"
//TSL2561 tsl(TSL2561_ADDR_LOW); 
/*
uint16_t read16(uint8_t reg,int addr)
{
  uint16_t x; uint16_t t;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(addr, 2);
  t = Wire.read();
  x = Wire.read();
  x <<= 8;
  x |= t;
  return x;
}

uint8_t read8(uint8_t reg,int addr)
{
  uint8_t x;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(addr, 1);
  x = Wire.read();
  return x;
}


void write8 (uint8_t reg, uint8_t value,byte addr)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
*/


//#include <TFT.h> // Hardware-specific library

//#define CS   0
//#define DC   1
//#define RESET  8  

//TFT myScreen = TFT(CS, DC, RESET);
//TFT myScreen = TFT(0, 1, 8);




// set up variables using the SD utility library functions:
//Sd2Card card;
//SdVolume volume;
//SdFile root;
// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
//const int chipSelect = 4;    


int freeRam(void)
{
  extern unsigned int __heap_start;
  extern void *__brkval;
  int v;
  return (int)&v-(__brkval==0?(int)&__heap_start:(int)__brkval);
}



/*
//SPI.begin();
 //SPI.setDataMode(SPI_MODE0);
 //SPI.setBitOrder(MSBFIRST);
 //SPI.setClockDivider(SPI_CLOCK_DIV2);//max
 
 digitalWrite (53, LOW);
 SPSR = (1 << SPI2X);
 SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first
 // polarity and phase = 0
 // clock = fosc / 2
 SPDR = 0b10000001;// start transfer
 while(!(SPSR&(1<<SPIF)));// interrupt also can!
 SPDR = 0b11000001;// start transfer
 while(!(SPSR&(1<<SPIF)));// interrupt also can!
 */


void InitSPI(void)
{
  //SPI.setDataMode(SPI_MODE0);//default
  //SPI.setBitOrder(MSBFIRST);// maybe
  //SPI.setClockDivider(SPI_CLOCK_DIV2);//max
  SPSR = (1 << SPI2X);//2
  //SPSR = (0 << SPI2X); //4
  //SPCR = (1 << MSTR) | (1 << SPE) |(1<<SPR0);      // enable, master, msb first
  SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first
}


unsigned long resetTime = 0;
#define TIMEOUTPERIOD 100             // You can make this time as long as you want,
// it's not limited to 8 seconds like the normal
// watchdog
//#define doggieTickle() resetTime = millis();  // This macro will reset the timer
void(* resetFunc) (void) = 0; //declare reset function @ address 0

//#define SetupWD(timeout){cli();wdt_reset();MCUSR&=~(1<<WDRF);WDTCSR=(1<<WDCE)|(1<<WDE);WDTCSR=(1<<WDIE)|timeout;WDhappen;sei();}

//byte odd=0;
//long r1=0,r2;
//word VccN[8][8];
//word Vcc1;



volatile byte WDhappen;
volatile word  t1111; // vars updated in ISR should be declared as volatile and accessed with cli()/sei() ie atomic
volatile boolean WDsleep=false;


//uint32_t ticks=0;
//uint8_t seconds=0;
//uint8_t minutes=0;
//uint8_t hours=0;
//uint8_t days=0;

/*
ISR (TIMER1_COMPA_vect){
 ticks++;
 if(ticks==500){
 ticks=0;
 seconds++;
 if(seconds==60){
 seconds=0;
 minutes++;
 if(minutes==60){
 minutes=0;
 hours++;
 if(hours==24){
 hours=0;
 days++;
 }
 }
 }
 }
 }*/

byte T1happen=0;

/*
ISR (TIMER1_COMPA_vect){
//  ticks++;
  T1happen=1;
}*/


#define SetADC(bandgap,input,us){ ADCSRA|=(1<<ADEN);delayMicroseconds(2);ADMUX=(bandgap<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input;delayMicroseconds((us>>1));} // input (0..7,8,14) (bg/vcc analogReference )
#define ADCoff{ ADCSRA&=~(1<<ADEN); }
#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 
/*
void SetADCinputChannel(boolean REFS1bit,uint8_t input,uint16_t us)
 {
 //  ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input; // input (0..7) (default VCC reference)
 ADMUX = (REFS1bit<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input; // input (0..7) (1.1v analogReference reference)
 if(us)delayMicroseconds(us); // Wait for input channel to settle (300us)
 }*/

/*
#define tRST 8
#define tCE 2 // don't go along with CErtc but works on PIN4!
#define tDC 5
#define tDIN 11
#define tCLK 13
*/
//tpic6a595
//#define DATAPIN 9
//#define CLOCKPIN 8
//#define LATCHPIN 6

//
//    Pin2Output(DDRD,1);Pin2LOW(PORTD,1); \ 
//    Pin2LOW(PORTD,1); Pin2Input(DDRD,1); \

#define ERR_WHERE_IS_THE_CLOCK    0x20;
#define ERR_STRANGE_CLOCK_DATA   0x28;
#define ERR_STOPPED_CLOCK            0b00000100;
#define ERR_WHERE_IS_THE_TSL2561 0x10;
#define ERR_BROKEN_SLEEP              0b00010000;
#define ERR_NO_SQW                        0b00100000;  
#define ERR_SET_CLOCK                    0x2F;
#define ERR_I2C                                0x30;
byte ERR=0; // ошибки (255)

word TFT_IS_ON=0;

void TFT_ON(byte duration){
  Pin2Output(DDRB,0);Pin2HIGH(PORTB,0); 
//    Pin2Output(DDRB,7);Pin2HIGH(PORTB,7);
  
    Pin2Output(DDRD,1);Pin2HIGH(PORTD,1); 
    Pin2Output(DDRD,4);Pin2LOW(PORTD,4); 
    Pin2Output(DDRB,2);Pin2LOW(PORTB,2); // SS(SPI) init will put it low  (reset)
    Pin2Output(DDRB,3);Pin2LOW(PORTB,3); 
    Pin2Output(DDRB,5);Pin2LOW(PORTB,5);
    delay(10); // time for power pin to stabilize  
    TFT_IS_ON=duration;
    InitTFT();
//    delay(120);
    Pin2LOW(PORTD,1); 
} // включаем питание  дисплея
    
void TFT_OFF(void){ writecommand(ST7735_SLPIN); 
    Pin2LOW(PORTB,2); Pin2Input(DDRB,2); 
    Pin2LOW(PORTB,3); Pin2Input(DDRB,3); 
    Pin2LOW(PORTB,5); Pin2Input(DDRB,5); // need to close all  connected pins before sinking  remaining charge. otherwise it will be suck current from them :)
    Pin2LOW(PORTB,0); // sink charge first  then to input
    Pin2LOW(PORTB,7); // sink charge first  then to input
//    delay(1);
//    delayMicroseconds(1);
    Pin2Input(DDRB,0);
  //  Pin2Input(DDRB,7);
    TFT_IS_ON=0; 
}// вЫключаем питание  дисплея

long LCD;

void LCD_ON(void){
  Pin2Output(DDRB,0);Pin2HIGH(PORTB,0); 
//    Pin2Output(DDRB,7);Pin2HIGH(PORTB,7);
  
    Pin2Output(DDRD,1);Pin2HIGH(PORTD,1); 
    Pin2Output(DDRD,4);Pin2LOW(PORTD,4); 
    Pin2Output(DDRB,2);Pin2LOW(PORTB,2); // SS(SPI) init will put it low  (reset)
    Pin2Output(DDRB,3);Pin2LOW(PORTB,3); 
    Pin2Output(DDRB,5);Pin2LOW(PORTB,5);
    delay(11); // time for power pin to stabilize  
    InitTFT();//    delay(120);
    Pin2LOW(PORTD,1); 
} // включаем питание  дисплея
    
void LCD_OFF(void){ writecommand(ST7735_SLPIN); 
    Pin2LOW(PORTB,2); Pin2Input(DDRB,2); 
    Pin2LOW(PORTB,3); Pin2Input(DDRB,3); 
    Pin2LOW(PORTB,5); Pin2Input(DDRB,5); // need to close all  connected pins before sinking  remaining charge. otherwise it will be suck current from them :)
    Pin2LOW(PORTB,0); // sink charge first  then to input
    Pin2LOW(PORTB,7); // sink charge first  then to input
//    delay(1);
//    delayMicroseconds(1);
    Pin2Input(DDRB,0);
  //  Pin2Input(DDRB,7);
    LCD=0; 
}// вЫключаем питание  дисплея

//void RTC_ON(void){Pin2Output(DDRD,0);Pin2HIGH(PORTD,0);Pin2Output(DDRC,1);Pin2Output(DDRC,2);}
//void RTC_OFF(void){Pin2LOW(PORTD,0);Pin2Input(DDRC,1);Pin2Input(DDRC,2);Pin2LOW(PORTC,1);Pin2LOW(PORTC,2); delayMicroseconds(1);Pin2Input(DDRD,0);}

//#define TSL2561_ON {Pin2Output(DDRC,4);Pin2Output(DDRC,5);}
//#define TSL2561_OFF {Pin2Input(DDRC,4);Pin2LOW(PORTC,4);Pin2Input(DDRC,5);Pin2LOW(PORTC,5);}




// каждый раз перед обращением к часам проверяем их вменяемость

byte pH; // предыдущий час
byte cH; // текущий час
byte pM; // предыдущая минута
byte cM; // текущая минута
byte cS=0xff; // текущая секунда
byte pS;   // предыдущая секунда
word sS=0; // частота опроса часиков

byte CS; // текущая секунда
byte CM; // текущая минута
byte CH; // текущий час (0..23)

      //void CPUSlowDown(void) {
  // slow down processor by a factor of 8
  //CLKPR = _BV(CLKPCE);
//  CLKPR = _BV(CLKPS1) | _BV(CLKPS0);
//}

byte grr,grr2;



void __attribute__ ((noinline)) wait4int(void)
{
    byte n=0;
    do{if(TWCR&(1<<TWINT)){  return; }}while(--n);  // linited wait for TWINT bit    
    ERR=ERR_I2C;
}

byte TimeS[9]={0,0,0,0,0x05,0x01,0x03,0x22,0x10}; // 1 марта 7522 SQW 1s

void SetTime(void)
{
    TWCR = (1<<TWEN) |  (1<<TWINT) | (1<<TWSTA);        // send start condition
    wait4int();//    {}while(!(TWCR&(1<<TWINT)));  // wait for TWINT bit    // limiting waiting
    TWDR=(0x68<<1); // SLA+W
    TWCR = (1<<TWEN) | (1<<TWINT);        // proceed with SLA+W
    wait4int();
    for(byte v=0;v<9;v++) // burst write
    {
        TWDR=TimeS[v];
        TWCR = (1<<TWEN) | (1<<TWINT);        // proceed with data
        wait4int();
        if (TWSR!=0x28){ ERR=ERR_SET_CLOCK; break; } // got ack?
    }
}

void RequestFrom(byte addr,byte reg)
{    
    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWSTA);        // send start condition
    wait4int();//    {}while(!(TWCR&(1<<TWINT)));  // wait for TWINT bit    // limiting waiting

    TWDR=addr; // SLA+W
    TWCR = (1<<TWEN) | (1<<TWINT);        // proceed with SLA+W
    wait4int();//    if (TWSR!=0x18){return false;} // got ack
    TWDR=reg;
    TWCR = (1<<TWEN) | (1<<TWINT);        // proceed with reg number
    wait4int();//    if (TWSR!=0x28){return false;} // got ack
    
    TWCR = (1<<TWEN) |  (1<<TWINT) | (1<<TWSTA);        // send repeated start condition
    wait4int();//    if (TWSR!=0x10){return false;}
    TWDR=addr|1; // SLA+R
    TWCR = (1<<TWEN) | (1<<TWINT);        // proceed with SLA+R
    wait4int();  //  if (TWSR!=0x40){return false;} // got ack

}

byte Intensity[16] = {0,1,2,3, 4,5,6,7, 8,9,10,11, 12,13,14,15}; // почасовая интенсивность яркости
byte FlashIntensity=0;

long volatile ticks; // 172800 в день 1/2с
byte HR;

byte __attribute__ ((noinline)) unBCD(byte bcd){return (((bcd>>4)*10)+(bcd&0xF)); }

void RTC(void)
{  
    Pin2Input(DDRC,4);Pin2Input(DDRC,5);Pin2HIGH(PORTC,4);Pin2HIGH(PORTC,5);   // activate internal pullups for twi.
 
    RequestFrom((0x68<<1),7);
    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);         // proceed with reading + send ack
    wait4int();//    if (TWSR!=0x50){ ERR=ERR_I2C; return;} // ack sent
    if((TWSR==0x50)&&(TWDR==0x10))
    {
        RequestFrom((0x68<<1),0);
        TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);         // proceed with reading + ack
        wait4int();//    if (TWSR!=0x50){return false;} // ack sent
        TimeS[3]=TWDR;
        TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);         // proceed with reading + ack
        wait4int();  //  if (TWSR!=0x50){return false;} // ack sent
        TimeS[2]=TWDR;
        TWCR = (1<<TWEN) | (1<<TWINT) | (0<<TWEA);         // proceed with reading + nack
        wait4int();
        if (TWSR!=0x58){ERR=ERR_WHERE_IS_THE_CLOCK; } // nack sent
        TimeS[1]=TWDR;
        
        ticks=unBCD(TimeS[1])*7200L+unBCD(TimeS[2])*120L+unBCD(TimeS[3])*2;
        HR=ticks/10800L;//90*120;
        FlashIntensity=Intensity[HR];
        
    }
    else{ ERR=ERR_WHERE_IS_THE_CLOCK; SetTime(); ticks=0;}
    TWCR = (1<<TWEN) |  (1<<TWINT) | (1<<TWSTO);        // send stop and forget     // wait for stop condition to be exectued on bus  (TWINT is not set after a stop condition!)

    Pin2LOW(PORTC,4);Pin2LOW(PORTC,5);Pin2Input(DDRC,4);Pin2Input(DDRC,5); // если STOP не успевает - поставь задержку
}

typedef union{byte B[4];word W[2];long L[1];} Data4;
Data4 Light;

void WriteByte(byte addr, byte reg, byte val)
{
    TWCR = (1<<TWEN) |  (1<<TWINT) | (1<<TWSTA);        // send start condition
    wait4int();//    {}while(!(TWCR&(1<<TWINT)));  // wait for TWINT bit    // limiting waiting

    TWDR=addr; // SLA+W
    TWCR = (1<<TWEN) | (1<<TWINT);        // proceed with SLA+W
    wait4int();    

    TWDR=reg;
    TWCR = (1<<TWEN) | (1<<TWINT);        // proceed with reg number
    wait4int();   // if (TWSR!=0x28){ ERR=ERR_WHERE_IS_THE_TSL2561; return;} 

    TWDR=val;
    TWCR = (1<<TWEN) | (1<<TWINT)|(1<<TWEA);         // proceed with reg number
    wait4int();  //  if (TWSR!=0x28){ ERR=ERR_WHERE_IS_THE_TSL2561; return;} 
}

#define TSL2561_READBIT           (0x01)
#define TSL2561_COMMAND_BIT       (0x80)    // Must be 1
#define TSL2561_CLEAR_BIT         (0x40)    // Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT          (0x20)    // 1 = read/write word (rather than byte)
#define TSL2561_BLOCK_BIT         (0x10)    // 1 = using block read/write
#define TSL2561_CONTROL_POWERON   (0x03)
#define TSL2561_CONTROL_POWEROFF  (0x00)

enum
{
  TSL2561_REGISTER_CONTROL          = 0x00,
  TSL2561_REGISTER_TIMING           = 0x01,
  TSL2561_REGISTER_THRESHHOLDL_LOW  = 0x02,
  TSL2561_REGISTER_THRESHHOLDL_HIGH = 0x03,
  TSL2561_REGISTER_THRESHHOLDH_LOW  = 0x04,
  TSL2561_REGISTER_THRESHHOLDH_HIGH = 0x05,
  TSL2561_REGISTER_INTERRUPT        = 0x06,
  TSL2561_REGISTER_CRC              = 0x08,
  TSL2561_REGISTER_ID               = 0x0A,
  TSL2561_REGISTER_CHAN0_LOW        = 0x0C,
  TSL2561_REGISTER_CHAN0_HIGH       = 0x0D,
  TSL2561_REGISTER_CHAN1_LOW        = 0x0E,
  TSL2561_REGISTER_CHAN1_HIGH       = 0x0F
};

typedef enum
{
  TSL2561_INTEGRATIONTIME_13MS      = 0x00,    // 13.7ms
  TSL2561_INTEGRATIONTIME_101MS     = 0x01,    // 101ms
  TSL2561_INTEGRATIONTIME_402MS     = 0x02,    // 402ms
  TSL2561_INTEGRATIONTIME_MANUAL  = 0x08     // manual bit
}
tsl2561IntegrationTime_t;

typedef enum
{
  TSL2561_GAIN_0X                   = 0x00,    // No gain
  TSL2561_GAIN_16X                  = 0x10,    // 16x gain
}
tsl2561Gain_t;

void TSLstart(void)
{    
    Pin2Input(DDRC,4);Pin2Input(DDRC,5);Pin2HIGH(PORTC,4);Pin2HIGH(PORTC,5);   // activate internal pullups for twi.  
    WriteByte((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING), (TSL2561_INTEGRATIONTIME_101MS |TSL2561_GAIN_0X));
    WriteByte((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL), TSL2561_CONTROL_POWERON);
    if (TWSR!=0x28){ ERR=ERR_WHERE_IS_THE_TSL2561; } 
    TWCR = (1<<TWEN) |  (1<<TWINT) | (1<<TWSTO);   // send stop condition
}

void TSLstop(void)
{
    RequestFrom((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW));

    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);         // proceed with reading + ack
    wait4int();//    if (TWSR!=0x50){return false;} // ack sent
    Light.B[0]=TWDR;
    TWCR = (1<<TWEN) | (1<<TWINT) | (0<<TWEA);         // proceed with reading + ack
    wait4int();//    if (TWSR!=0x50){return false;} // ack sent
    Light.B[1]=TWDR;

    RequestFrom((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW));
    
    TWCR = (1<<TWEN) | (1<<TWINT)|(1<<TWEA);         // proceed with reading + ack
    wait4int();//    if (TWSR!=0x50){return false;} // ack sent
    Light.B[2]=TWDR;
    TWCR = (1<<TWEN) | (1<<TWINT) |(0<<TWEA);         // proceed with reading + ack
    wait4int();//    if (TWSR!=0x50){return false;} // ack sent
    Light.B[3]=TWDR;  if (TWSR!=0x58){ ERR=ERR_WHERE_IS_THE_TSL2561; return;} 

//WriteByte((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL), TSL2561_CONTROL_POWEROFF);  ///  глючит ну и ладно
//  if (TWSR!=0x28){ ERR=ERR_WHERE_IS_THE_TSL2561; return;} 
  
    TWCR = (1<<TWEN) |  (1<<TWINT) | (1<<TWSTO);   // send stop condition
    Pin2LOW(PORTC,4);Pin2LOW(PORTC,5);Pin2Input(DDRC,4);Pin2Input(DDRC,5); // если STOP не успевает - поставь задержку
}

word TouchSensor(void) // 
{
      word cycles=30000;
      Pin2Output(DDRC,0);Pin2LOW(PORTC,0); // discharge sensor  pin
//      delayMicroseconds(50); // works fine without
      Pin2Input(DDRC,0);
      for(int i=0;i<cycles;i++){if (PINC&0b00000001){cycles=i;break;}}
      Pin2Output(DDRC,0);Pin2LOW(PORTC,0); // discharge sensor  pin
      Pin2Input(DDRC,0);Pin2LOW(PORTC,0); // sensor pin
      return cycles;
}

byte sleeps=0;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

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
 
#define system_sleep(mode,flags) { set_sleep_mode(mode); sleeps=0;do{sleep_enable();sleep_cpu();sleep_disable();if(flags){break;}else{sleeps++;}}while(1);}
 
// Watchdog timeout values
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms, 6=1sec, 7=2sec, 8=4sec, 9=8sec
#define setup_watchdog(timeout){cli(); __asm__ __volatile__("wdr\n\t"); MCUSR&=~(1<<WDRF);WDTCSR|=(1<<WDCE)|(1<<WDE);WDTCSR=((1<<WDIE)|timeout);sei();}
 
// Turn off the Watchdog
// Watchdog sleep function - combines all the above functions into one
#define watchdogSleep(mode,timeout){setup_watchdog(timeout);system_sleep(mode,WDhappen);wdt_disable();}
 
 volatile word  cnt1; // vars updated in ISR should be declared as volatile and accessed with cli()/sei() ie atomic

ISR(WDT_vect) // Watchdog timer interrupt.
{ 
  //reboot();  //reboot
  //  r2=TCNT1;
  if(WDsleep)
  {
  cnt1=TCNT1;
  WDhappen=1;
  WDsleep=0;
  }
  //debug
//  else{    reboot();     }// This will call location zero and cause a reboot.
}


word TouchD[4];
//byte TouchPos=0;
word Etouch;
void TouchSample(void){for(byte i=0;i<4;i++){TouchD[i]=TouchSensor();} Etouch=TouchT(); } // calibrate touch sensor
word TouchT(void)
{
  word res=0;
  
  for(byte i=0;i<4;i++){ res+= TouchD[i]; }
  res=res+(res>>4); // среднее+ 1/16
  return (res>>2);
}

//#define DS1307_I2C_ADDRESS 0x68 // read and write address is different for DS1307 

// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively TSL2561_ADDR_FLOAT)
//TSL2561 tsl(TSL2561_ADDR_LOW); 

// port B: 5 port C: 8 port D:11
//byte cports[10]={PORTB1,PORTB2,PORTB1,PORTB1,PORTB1,PORTB1,PORTB1,PORTB1,PORTB1,PORTB1};
//  char tstr[7];// строка даты и времени


//byte PS=0xFF; // предыдущая секунда
//byte PH=0xFF; // предыдущий час
//byte HR; // текущий час (0..23)
//byte PM; // предыдущая минута
//byte MN;
//byte CHAS,LEFT;
// интенсивность/продолжительность пыхи в микросекундах 0..255
//byte mi=25; 
//byte mi=0;// максимальная интенсивность или 16 если меньше 
//byte Intensity[24] = {2,2,2,2,2,2, 3,5,7,9,12,14, 15,16,16,16,15,14, 12,9,7,5,3,0};
//byte Intensity[16] = {0,0,0,1, 1,2,3,3, 4,4,5,4, 3,3,2,1};

//volatile long sss=0;
//volatile long bbb=0;


//byte pp[10]={
//  5,5,5,5,5,5,5,5,5,5}; //port
//byte pb[10]={
//  1,0,0,0,0,0,0,0,0,0}; // bit in port
// the setup routine runs once when you press reset:
byte v;
byte   extreset;
void setup() {                
  byte pmask,idx;

  wdt_disable();

     // Pin2Output(DDRD,2);Pin2HIGH(PORTD,2);// start charging timeout capacitor (default state)

  extreset=MCUSR;

//    Pin2Input(DDRD,2);Pin2HIGH(PORTD,2);// pinMode(1,INPUT_PULLUP);      
//    attachInterrupt(0, pin2_isr, RISING);
//sss=0;
//      watchdogSleep(SLEEP_MODE_PWR_DOWN,T2S); // 2 sleeps - INT0 every second
//delay(2000);      if(!sss){ERR=ERR_NO_SQW;}

//TWI setup
    TWSR&=~((1<<TWPS0)|(1<<TWPS1));
//    TWBR=40; // 1215
//    TWBR=32; //  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;   // 1023
//    TWBR=24; // 833
//    TWBR=20; // 741
//    TWBR=16; // 649
//    TWBR=12; // 558
//    TWBR=8; // 475
    TWBR=4; // 403
//    TWBR=2; // 370 unstable
//    TWBR=1; // unstable


  //setup timer1
  cli();
  TCCR1A=0x00;
  //  TCCR1B=(1 << WGM12)|(0<<CS22)|(0<<CS21)|(1<<CS20); // /no prescaler;
  TCCR1B=(1<<WGM12)|(0<<CS22)|(1<<CS21)|(0<<CS20); // /8; 1us clock
  TCNT1H=0x00; TCNT1L=0x00;
  ICR1H=0x00; ICR1L=0x00;
  OCR1AH=0xFF; OCR1AL=0xFF;  //OCR1AH=0x9C;  //OCR1AL=0x40; // 40000//  OCR1AH=0x4E;//  OCR1AL=0x20; // 20000//  OCR1AH=0x03;//  OCR1AL=0xE8; // 1000

  //  TCCR1B |= (1 << WGM12);  //  TCCR1B |= (1 << CS10);
  // TIMSK1 |= (1 << OCIE1A);// no interrupts just counting 1 tick is 1 microsecond

  // setup timer2 
  TCCR2A=0x00;
  TCCR2B=(1 << WGM12)|(0<<CS22)|(0<<CS21)|(1<<CS20); // /no prescaler;
//  TCCR2B=(1<<WGM12)|(0<<CS22)|(1<<CS21)|(0<<CS20); // /8; 1us clock
  TCNT2=0x00;
//  ICR2=0x00;
  OCR2A=0xFF;

// every 1/2 second interrupt from RTC
//Pin2Input(DDRC,0);Pin2HIGH(PORTC,0); // pull up on A0



  sei();




//for(byte i=0;i<16;i++){if(mi<Intensity[i]){mi=Intensity[i];}} if(mi<16){mi=16;}// множитель для столбиков


  TouchSample();

/*
    TFT_ON(5);
//    fillScreen(0xfcfc00);
        DrawBox(0,0,159,127,0xfc,0xfc,0x00);    // очистка экрана

    word cycles=TouchSensor();
    setAddrWindow(0,0,7,119);
//    byte rtc=Check_RTC(16);
    ta("SET");//th(ERR);ta(" RTC:");tn(10,rtc);th(rtc8);
    setAddrWindow(10,0,17,119);
    ta("cycles:");tn(10000,cycles);

    delay(4500);
    TFT_OFF(); // close rtft/rtc mosfet

delay(1000);
    TFT_ON(5);
//    fillScreen(0xfc00fc);
        DrawBox(0,0,159,127,0xfc,0x00,0xfc);    // очистка экрана

    cycles=TouchSensor();
    setAddrWindow(0,0,7,119);
//    byte rtc=Check_RTC(16);
    ta("SET");//th(ERR);ta(" RTC:");tn(10,rtc);th(rtc8);
    setAddrWindow(10,0,17,119);
    ta("cycles:");tn(10000,cycles);

    delay(4500);
    TFT_OFF(); // close rtft/rtc mosfet
*/

/*
Pin2Output(DDRD,6);Pin2HIGH(PORTD,6); 
Pin2Output(DDRB,7);Pin2HIGH(PORTB,7); 
delay(2000);
Pin2LOW(PORTD,6);Pin2Input(DDRD,6);
Pin2LOW(PORTB,7);Pin2Input(DDRB,7);
delay(2000);
*/

//word vvv;

/*
Wire.begin();
  //t=TCNT1;sei();
  //digitalWrite(CE,LOW);
  //PORTD&=~(1<<CErtc);//digitalWrite(CErtc,LOW);
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0x07);
  Wire.write(0x11);
  Wire.endTransmission();
  delay(5000);
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0x07);
  Wire.write(0x00);
  Wire.endTransmission();
  */
//Wire.requestFrom();
  //PORTD|=(1<<CErtc);//digitalWrite(CErtc,HIGH);
//  rtcpoke(15,0xAC);
//cli();
//	pinMode(_scl_pin, OUTPUT);
//	pinMode(_sda_pin, OUTPUT);

//Pin2Output(DDRB,7);Pin2HIGH(PORTB,7);// включаем питание  дисплея

//Pin2Output(DDRC,4);Pin2Output(DDRC,5);

//PORTC=0;Pin2Input(DDRC,_scl_pin);Pin2Input(DDRC,_sda_pin);//  pinMode(_scl_pin, INPUT);pinMode(_sda_pin, INPUT);// high imp state to avoid current  sipping through SDA/SCL pullup resistors


//Pin2Input(DDRC,CLKrtc);Pin2LOW(PORTC,CLKrtc);
//Pin2Input(DDRC,IOrtc);Pin2LOW(PORTC,IOrtc);
//Pin2Input(DDRC,CErtc);Pin2LOW(PORTC,CErtc);

//delay(1900);

//  Serial.begin(19200);
//  Serial.println("Hi!");


  //  Pin2Input(DDRD,0);Pin2HIGH(PORTD,0);//  pinMode(0,INPUT_PULLUP);

  //  myScreen.begin();  
  //myScreen.background(0,0,0);  // clear the screen with black
  //delay(2000);  // pause for dramatic effect
/*
  // check setup function
  PORTC=0x3;//  set pins 0123 HIGH (internal pullups) //digitalWrite(A1,HIGH);digitalWrite(A2,HIGH);digitalWrite(A3,HIGH);digitalWrite(A4,HIGH);
  //  pinMode(A1,INPUT_PULLUP); pinMode(A2,INPUT_PULLUP);  pinMode(A3,INPUT_PULLUP);  pinMode(A4,INPUT_PULLUP);   same same
  delay(1);
  v=(~PINC)&0x03;

  if (v>0){// 0 all are OFF // 1 1st is ON // 4 3rd is ON // C 3&4 are ON // F all are ON  
    //    setup function v (1..0xF) is called
    //idx=0; // lamp index
    //pmask=pp[0];//read port mask



  //  Pin2Output(DDRB,pb[0]);
    if(v==1)
    {

    }// if 1
   
    

//    __asm__ __volatile__ ("nop\n\t");

    //v=PORTB;// 5
    //pmask=PINB;
    //PORTB=0xff;
    //v=PORTC;// 8
    //PORTC=0xff;
    //v=PORTD;// 11
    //PORTD=0xff;

    //__asm__ __volatile__ ("in r24,%0\n\t"::"i"(PORTB));

    //pp[0]=pmask|pb[0];
    //PORTB=1<<1;// bit 1 of portb


    //    delay(2000);
  }
*/
//  PORTC=0;//digitalWrite(A1,LOW);digitalWrite(A2,LOW);digitalWrite(A3,LOW);digitalWrite(A4,LOW);
//  DDRC=0x3;//  pinMode(A1,OUTPUT);  pinMode(A2,OUTPUT);  pinMode(A3,OUTPUT);  pinMode(A4,OUTPUT);
  //DDRC=0; //input



  // initialize the digital pin as an output.
  //  pinMode(led, OUTPUT);  
  //  digitalWrite(A5,HIGH); pullup resistor
  // pinMode(9, OUTPUT); //ACS712 module

  // digitalWrite(9,HIGH);// lcd+acs712 


  //  cli();
  //TCCR2A=(1<<WGM21); // ctc mode
  //  TCCR2B=(0<<CS22)|(0<<CS21)|(1<<CS20); // /no prescaler
  //  TCCR2B=(0<<CS22)|(1<<CS21)|(0<<CS20); // /8
  //  TCCR2B=(1<<CS22)|(1<<CS21)|(1<<CS20); // 1024
  //TCNT2 = 0; // clear counter
  //  OCR2A=255; // clear timer on compare
  // TIMSK2 |= (1<<OCIE2A); // no need that frequent interrupt at all
  //  sei();
  /*cli();
   TIMSK2 &= ~(_BV(OCIE2A) | _BV(OCIE2B) | _BV(TOIE2)); // disable interrupts 
   ASSR = _BV(AS2);     // select clock source -8MHz crystal connected on xtal1, xtal2 
   
   TCCR2B=(0<<CS22)|(0<<CS21)|(1<<CS20); // /no prescaler
   //  TCCR2B=(0<<CS22)|(1<<CS21)|(0<<CS20); // /8
   //  TCCR2B=(1<<CS22)|(1<<CS21)|(1<<CS20); // 1024
   
   //  TCCR2B = _BV(CS22) | _BV(CS20); 
   
   TCNT2 = 0; // clear counter
   OCR2A=255; // clear timer on compare
   
   // clear the Timer/Counter2 Interrupt Flags 
   TIFR2 = _BV(TOV2); 
   
   // enable counter2 overflow flag 
   //    TIMSK2 = _BV(TOIE2); 
   sei();
   */

  //  Serial.begin(9600);

  //pinMode(7,OUTPUT);
  //pinMode(7,INPUT);
  //digitalWrite(7,LOW);
  // setup analog comparator
  //  cli();
  //  ADCSRB = 0;
  //  ACSR = (1<<ACBG); //bandgap instead of AIN0 int on falling edge 
  //  delay(1); // to avoid false interrupt due bandgap voltage settle

  //  ACSR = (1<<ACI)|(1<<ACBG)|(1<<ACIE)|(1<<ACIS1)|(0<<ACIS0); //bandgap instead of AIN0 int on falling edge 
  //  ACSR = (1<<ACBG)|(1<<ACIE)|(0<<ACIS1)|(0<<ACIS0); //bandgap instead of AIN0 int on toggle
  //  ACSR = (1<<ACBG); //bandgap instead of AIN0 int on toggle
  //  DIDR1 = (1<<AIN0D); // it will be bandgap on AIN0  (debug rtc)
  //  sei();


  //TIMSK0 &= ~_BV(TOIE0); // disable timer0 overflow interrupt
  // when it is not needed

  //delay(2000);
  //set date/time
  //rtcwriteprotect(false);//20us
  //rtc.writeProtect(false);
  //rtc.setTime(22,49,0);
  //   rtcsettime(0x15,0x59,0x35);
  // rtcsetdate(0x09,0x12,0x13);
  //rtcsetDOW(1);


//Pin2Output(DDRD,0);// sreset mosfet 2n7000 control
//Pin2Output(DDRD,1);// 3.3/5v mosfet 2n7000 control
//Pin2Output(DDRD,2);// INT0 line

//Pin2Output(DDRD,3);// INT1 line /TFT CS
//Pin2Output(DDRD,4);// TFT DC




//Pin2Output(DDRB,0); // CLOCKPIN 8
//Pin2Output(DDRD,1); // DATAPIN 9
//Pin2Output(DDRB,2);
//Pin2Output(DDRB,6);


//Pin2Output(DDRC,2);
//Pin2Output(DDRC,3);
//Pin2Output(DDRC,4);
//Pin2Output(DDRC,5);




//  pinMode(DATAPIN,OUTPUT);
//  pinMode(CLOCKPIN,OUTPUT);
//  pinMode(LATCHPIN,OUTPUT);




// test LCD

//Pin2Output(DDRC,4);
//Pin2HIGH(PORTC,4);
//delay(1);




//TFT_ON(2);
  //InitTFT();   // initialize a ST7735S chip, black tab


//  fillRect(0,0,27,91,0xfcfcfc);

//fillScreen(0x000000);

//word cycles=TouchSensor();
//  setAddrWindow(0,0,7,119);
//  ta("cycles:");tn(10000,cycles);

//delay(2000);
//TFT_OFF;

//  setAddrWindow(0,0,20,20);
//  drawPixel(5,10,0x000000);
//  drawPixel(5,11,0x000000);
  //drawPixel(5,12,0x000000);

//delay(5000);

/*
SPI.begin();
//    SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
//    SPI.setClockDivider(SPI_CLOCK_DIV8); // 4 MHz (half speed)
//    SPI.setClockDivider(SPI_CLOCK_DIV16); // 4 MHz (half speed)
    SPI.setClockDivider(SPI_CLOCK_DIV32); // 4 MHz (half speed)
    //Due defaults to 4mHz (clock divider setting of 21)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
*/

//  uint16_t time = millis();
  //fillScreen(0x000000);// black (64ms)
//  fillScreen(0x005000);// green

//  time = millis() - time;

 //setAddrWindow(0,0,7,120);
//wh(time);ta("TIME");

//delay(1500);
  // a single pixel
//  tft.drawPixel(tft.width()/2, tft.height()/2, ST7735_GREEN);
//fillRect(5,5,10,100,0xE400fc);

//TSL2561

  //if (tsl.begin()) {vvv=1;
  // tsl.begin
/*		_sendStart(TSL2561_ADDR_LOW);
		_waitForAck();
		_writeByte(TSL2561_REGISTER_ID);
		_waitForAck();
		_sendStop();
		_sendStart(TSL2561_ADDR_LOW);
		_waitForAck();
		vv = _readByte();
		_sendNack();
		_sendStop();*/

  /*
    Wire.begin();

		_sendStart(DS1307_ADDR_W);
		_waitForAck();
		_writeByte(addr);
		_waitForAck();
		_writeByte(value);
		_waitForAck();
		_sendStop();

		_sendStart(DS1307_ADDR_W);
		_waitForAck();
		_writeByte(addr);
		_waitForAck();
		_sendStop();
		_sendStart(DS1307_ADDR_R);
		_waitForAck();
		readValue = _readByte();
		_sendNack();
		_sendStop();

    Wire.begin();
  Wire.beginTransmission(TSL2561_ADDR_LOW);
  Wire.write(TSL2561_REGISTER_ID);
  Wire.endTransmission();
  Wire.requestFrom(TSL2561_ADDR_LOW, 1);
  int wirex = Wire.read();
  if (wirex & 0x0A ) {//Found TSL2561"
    
    write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);  //  enable();
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,  TSL2561_INTEGRATIONTIME_402MS |TSL2561_GAIN_16X);    //  // Set default integration time and gain

  // Note: by default, the device is in power down mode on bootup
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);  //  disable();
  }
*/  
  
//  tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
//  tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);  // longest integration time (dim light)
  
  // Now we're ready to get readings!
  



//  setAddrWindow(20,0,27,127);ta("val:");

/*
  setAddrWindow(30,20,37,51);
th((x>>8));th((x&0xFF));
  setAddrWindow(40,0,47,91);
th((xf>>8));th((xf&0xFF));
th((xi>>8));th((xi&0xFF));*/

//delay(5000);

//  setAddrWindow(30,10,37,120);
//ta(buf);

//  setAddrWindow(0,0,7,119);

//th(tstr[2]);ta(":");th(tstr[1]);ta(".");th(tstr[0]);ta(" ");th(tstr[4]);ta("-");th(tstr[5]);ta("-20");th(tstr[6]);ta(" ");th(tstr[3]);

  //setAddrWindow(50,10,57,120);

//ta("croCodile");
//delay(5555);


/*

//  fillRect(5,5,120,150,0x00fc00);
  


  fillScreen(0x0000fc);// blue

  setAddrWindow(10,20,17,31);
  t3(entries);
   setAddrWindow(20,30,27,41);
  t3(flag);

  setAddrWindow(80,10,87,120);
ta(nam);

  delay(1500);

  fillRect(10,20,70,18,0x00fcfc);

  delay(1500);
*/
//  	writecommand(0x39);  //IDMON
//  delay(1500);
//  	writecommand(0x38);  //IDMOFF // no visible effect

//  	writecommand(0x28);  //DISPOFF -- белая подсветка пустой экран
//  delay(2500);
//  	writecommand(0x29);  //DISPON

//  	writecommand(0x10);  //SLPIN- белая подсветка пустой экран
//  delay(2500);
//  	writecommand(0x11);  //SLPOUT

//delay(5000);





//PCICR |= (1 << PCIE0);     // set PCIE0 to enable PCMSK0 scan
  //  PCMSK0 |= (1 << PCINT0);   // set PCINT0 to trigger an interrupt on state change 


  // Setup the WDT  (16ms or reboot)
  /*
  cli();
//  NOP;
  __asm__ __volatile__("wdr\n\t");//  wdt_reset();
//  NOP;
  MCUSR &= ~(1<<WDRF);  // Clear the reset flag. 
  WDTCSR |= (1<<WDCE) | (1<<WDE); //  In order to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles).
WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//15ms (16280us)
 //  WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//30ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (0<<WDP0);//60ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0);//120ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0);//240ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);//480ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);//960ms
 // WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);//2s
  // WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//4s
//     WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s
  sei();*/



    Pin2Input(DDRC,3);Pin2HIGH(PORTC,3); // pull up on A0
    PCMSK1 = 1<<PCINT11; // setup pin change interrupt on A3 pin (SQuareWave from RTC)
    PCICR |= 1<<PCIE1; 

}

/*
ISR(TIMER1_OVF_vect)
 {
 //Toggle pin PD0 every second
 //    PIND=(1<<PD0);
 t1ovf++;
 }
 
 ISR(TIMER2_OVF_vect)
 {
 //Toggle pin PD0 every second
 //    PIND=(1<<PD0);
 t2ovf++;
 }*/


uint16_t st1,st2,delta,flash_duration;
uint8_t flash_start_mask,flash_stop_mask; // channel for flash

uint8_t a1,b1,c1,d1,e1;

byte tqq;

// flash delay in clock cycles
void flashold(void)
{
  int nn=0;

  //sei();

  //cli();
  // setup ADC in freerunning mode and do 1st conversion (25 clocks)
  //ADCSRB &=B11111000;
  // ADCSRB=(0<<ACME)|(0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0);
  // ADCSRA=(1<<ADATE);
  //ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<ADLAR)|(1<<ADATE)|(1<<ADIE)|(0<<ADPS2)|(0<<ADPS1)|(1<<ADPS0);// int takes +55 cycles

  //  ADCSRA=(1<<ADEN)|(1<<ADSC)|(1<ADLAR)|(0<<ADATE)|(0<<ADIE)|(0<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);
  //  ADCSRA=(1<<ADEN)|2;
  // ADCSRA |= (1<<ADSC); // start converting
  //sei();
  //nn=0;
  //do{++nn;}while(bit_is_set(ADCSRA,ADSC));



  //rez[13]=ADCSRA&(1<<ADSC);

  //for(int i=0;i<15;i++){lcd.print(rez[i]);lcd.print(" ");}

  //int aa=micros();
  //ADCv=0;
  //NOP;
  //e1=TCNT2;
  //NOP;
  //PORTA = 0b00000000; // select input channel 0 
  //c1=PINA;
  //d1=PINB;
  //e1=PINC;
  //c1=TCNT2-TCNT0;
  //cli();
  //r0(d1);// read TCNT0 (/64 clk)
  //mRawADC(nn,2);//3us


  //__asm__ __volatile__ ("in r24,0x26\n\t" "sts e1,r24\n\t"); //read TCNT0

    //d1=TCNT1;
  //NOP;
  //__asm__ __volatile__ ("ldi r25,0b11000001\n\t"); //PORTA = 0b11000001;// 1clock load port A opening mask
  //r0(d1);
  // select input channel 0 
  //__asm__ __volatile__ ("ldi r25,0b11000001\n\t" "out 0x02,r25\n\t"); //PORTA = 0b11000001;
  //__asm__ __volatile__ ("out 0x02,r25\n\t"); //PORTA = 0b11000001; // 1 clock
  //r2(e1);
  //PORTA |= 0b11000001; // select input channel 0 
  //c1=TCNT0;
  //NOP;
  //r02(e1);

  //sei();
  //for(int i=0;i<10000;i++){
  //  asm volatile ("lds r24,1 \n\t " :::); // 2 clocks
  // asm volatile ("1: sbiw r24,1 \n\t   brne 1b   \n\t" ::"r"(q1):); //breq 0
  // NOP;
  //__asm__ __volatile__ ("3:ldi r22,64 \n\t ldi r21,200 \n\t 2: ldi r20,243 \n\t 1: nop \n\t dec r20\n\t brne 1b\n\t dec r21\n\t brne 2b\n\t dec r22\n\t brne 3b\n\t"); // 1 1 1 2 +loop 5
  //delay(10);
  //delayMicroseconds(10); // 252 us actually?
  //int aa=ADCv;
  //~3us ADC conversion with ISR calling overhead
  //delayMicroseconds(1);//6 ADC conversions
  //NOP;//1
  //NOP;//1
  //NOP;//1
  //cli();
  //s1[0]=TCNT1L;s1[0]=s1[0]|(TCNT1H<<8);
  //s1[0]=TCNT1;

  //r2(a1);// double sts in macro!
  //#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 
  //mRawADC(nn,2);//3us
  //__asm__ __volatile__ ("ldi r24,0xC2 \n\t sts 0x007A,r24\n\t"); //ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;
  // adc conversion has started. can monitor analog comparator while converting

  //__asm__ __volatile__ ("1: lds r24,0x007A\n\t sbrc r24,6\n\t rjmp 1b\n\t"); //do{}while(bit_is_set(ADCSRA,ADSC));
  //__asm__ __volatile__ ("lds r24,0x0078\n\t lds r25,0x0079\n\t sts d1,r24\n\t sts e1,r25\n\t "); //v=ADCV;
  //NOP;
  //delayMicroseconds(100);
  //s1[1]=TCNT1;
  //s1[1]-=8;//correction
  //e1=ACSR&&(1<<ACO);
  //e1=ACSR;//&&(1<<ACO);
  //s1[1]=TCNT1L;s1[1]=s1[1]|(TCNT1H<<8);
  //sei();

  //NOP;
  //r2(b1);
  //r2(c1);
  //r2(d1);
  //r2(e1);
  //sei();
  //delta=64017;
  //a1=



  //PORTC |= 0b01000000; // select channel 1  ldi and out
  //PORTA |= 0b01000000; // select channel 1  ldi and out
  //PORTB |= 0b01000000; // select channel 0  sbi
  //TCNT1=0;
  __asm__ __volatile__ ( 
  //  "in r24, 0x6\n\t" //PORTC(8) PORTB(5) PORTA(2)
  //  "in r24, 0x30\n\t" //ACSR
  "cli \n\t"
    "lds r18,flash_duration\n\t"
    "lds r19,flash_duration+1\n\t"

    // storing to array
  "ldi r30,lo8(sc)\n\t" // load sc offset (low part)
  "ldi r31,hi8(sc)\n\t" // load sc offset (high part)

  "rjmp 1f\n\t"
    "2:\n\t" // analog comparator trigger

  "ldi r24,200 \n\t" "sts d1,r24\n\t" // some test code


  "rjmp 3f\n\t"
    "1:\n\t"
    "clr r20\n\t""sts 0x0085,r20\n\t""sts 0x0084,r20\n\t" // TCNT=0
  "lds r20,0x0084\n\t""lds r21,0x0085\n\t" // load 16bit TCNT1 value

  "lds r25,flash_start_mask\n\t" // "sts e1,r25\n\t"
    "out 0x02,r25\n\t" // START flash //pinc(6) pinb(3) pina(2)
  "5:\n\t"// next adc
  "ldi r24,0xC2 \n\t sts 0x007A,r24\n\t" //ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// adc conversion has started.

  // storing to array
  //  "ldi r30,lo8(s1+2)\n\t" // load s1 offset (low part)
  //  "ldi r31,hi8(s1+2)\n\t" // load s1 offset (high part)
  //"st Z+,r28\n\t" // 2 bytes
  //  "st Z+,r29\n\t"



    "4:\n\t"
    // compare timer r19:r18 with r23:r22
  "lds r22,0x0084\n\t""lds r23,0x0085\n\t" // load 16bit TCNT1 value
  "cp r18,r22 \n\t"//; compare lower bytes
  "cpc r19,r23 \n\t"// ; compare upper bytes

  "brcs 3f\n\t"//If the carry flag is set now, R1:R2 is smaller than R3:R4.


  "in r24, 0x30\n\t" //ACSR (can use "sbic 0..31,5\n\t" for 1st 32 ports)
  "sts e1,r24\n\t" 
    //  "sbrs r24,5\n\t" // skip next instruction if bit 5(ACO) is 1
  //  "sbrc r24,5\n\t" // skip next instruction if bit 5(ACO) is 0
  //  "rjmp 2b\n\t"
  "lds r25,0x007A\n\t"// read ADCSRA
  "sbrc r25,6\n\t rjmp 4b\n\t" //skip jump if bit 6(ADSC) is 0 // do{}while(bit_is_set(ADCSRA,ADSC));

  "lds r24,0x0078\n\t lds r25,0x0079\n\t" //read ADCW & store it plus check
  //  "sts a1,r24\n\t sts b1,r25\n\t "
  "st Z+,r24\n\t"
    "rjmp 5b\n\t"

    // "sts c1,r25\n\t"
  //  "sts d1,r24\n\t"

    //  "ldi r24,200 \n\t"
  //  "sts d1,r24\n\t"
  //  "nop \n\t"
  //  "ldi r30,0xb2\n\t"
  //  "ldi r31,0x0\n\t"
  //  "ld r25,Z \n\t"
  //  "nop\n\t"
  //  "nop\n\t"
  //  "ld r24,Z \n\t"

    "3:\n\t"
    //  "lds r25,flash_stop_mask\n\t" // "sts e1,r25\n\t"
  //  "out 0x02,r25\n\t" // STOP flash //pinc(6) pinb(3) pina(2)
  "sts st1,r20\n\t""sts st1+1,r21\n\t"
    "sts st2,r22\n\t""sts st2+1,r23\n\t"
    //"clr r24 \n\t sts 0x007A,r24\n\t" //ADCSRA=0;// stop adc (1st conversion will be long)
  "sei\n\t"
    //  "sts [a1],r25\n\t"
  //  "sts [b1],r24\n\t"
  //  "nop\n\t"
  //  "ldi r21,200 \n\t "
  //"ld r24,Z \n\t"
  //"st  %[c1], r24 \n\t" 
  //: [c1] "=r" (c1),[d1] "=r" (d1)// : [_PINB] "I"  (_SFR_IO_ADDR(PINB)) 
  );

  if(st2>st1){
    delta=(uint16_t)(st2-st1-4);
  }
  else{
    delta=st2+65536-st1-4;
  }//delta-=4;//correction

  //__asm__ __volatile__ ("ld r24,Z \n\t nop \n\t nop \n\t nop \n\t ld r23,Z \n\t sub r23,r24\n\t sts %[c1],%%r23\n\t":"=r"(c1)); // 1 1 1 2 +loop 5


  //__asm__ __volatile__ ("ldi r21,200 \n\t 2: ldi r20,250 \n\t 1: nop\n\t dec r20\n\t brne 1b\n\t dec r21\n\t brne 2b\n\t"); // 1 1 1 2 +loop 5
  //__asm__ __volatile__ ("ldi r21,200 \n\t 2: ldi r20,250 \n\t 1: nop\n\t dec r20\n\t brne 1b\n\t dec r21\n\t brne 2b\n\t"); // 1 1 1 2 +loop 5
  //delay(10);
  //}


  delay(3000);
  PORTB=0;
  //delayMicroseconds(10);// 6 ADC interrupts (cannot do any lower. nops don't work)
  //NOP;NOP;
  //nn=ADCv;
  //int vv=ADCval;

  //int bb=ADCv;
  //b1=TCNT2;

  //int bb=micros();
  //int cc=TCCR0A;
  /*
lcd.clear();
   lcd.setCursor(0,0);
   
   lcd.print("delta=");
   lcd.print(delta);
   
   lcd.print(" d=");
   lcd.print(b1-a1);
   
   lcd.print(" H=");
   lcd.print(delta>>8);
   lcd.print(" L=");
   lcd.print((delta<<8)>>8);
   
   lcd.print(" st1=");
   lcd.print(st1);
   lcd.print(" st2=");
   lcd.print(st2);
   
   uint16_t number_to_print=s1[0];
   //Divide number by a series of 10s
   for(mo = 4 ; number_to_print > 0 ; mo--)
   {
   //              tempo = number_to_print % (uint16_t)10;
   //                decimal_output[mo] = tempo;
   lcd.print((number_to_print % (uint16_t)10));
   number_to_print = number_to_print / (uint16_t)10;               
   
   }
   */

  //lcd.print(" a1=");
  //lcd.print(a1);
  //lcd.print(" b1=");
  //lcd.print(b1);
  //lcd.print(" c1=");
  //lcd.print(c1);
  //lcd.print(" d1=");
  //lcd.print(d1);
  //lcd.print(" e1=");
  //lcd.print(e1);

  //delay(100);

  //__asm__ ("nop\n\t");


  //delay(1000);
  //lcd.setCursor(0,0);
  //for(int i=0;i<10;i++){lcd.print(" ");lcd.print(sc[i]);}
  //delay(1500);

  //digitalWrite(10,LOW); //reset?

  //ADCSRA &=(1<<ADIF);
  //do{
  //if(nn++>100){break;}
  //if(ADCv){break;}
  //}while(1);
  //do{++nn;}while(ADCv==0);

  //for(int i=0;i<30;i++){rez[i]=ADCval;lcd.print(rez[i]);lcd.print(" ");}
  //for(int i=0;i<30;i++){lcd.print(rez[i]);lcd.print(" ");}

  //snprintf(tp,256,"[!] %d %d %d %d %d %d",ADCv,rez[0],rez[1],rez[2],rez[3],rez[4]);
  //lcd.print(tp);
  //lcd.print("uuu");

  //delay(10);


  //cli();
  //  PORTA = 0b11000001; // select input channel 1
  //PORTA = 0b00000000; // select input channel 0 
  //wait500ns;//220(0.5us)
  // wait1us;wait1us;wait1us;//wait 3us for MOSFETs to fully open
  //wait1us;//28-48(1us)
  //wait250ns;//7-15(1.25us)
  //wait250ns;//1-3(1.5us)
  //wait1us;wait1us;//wait1us;//wait1us;//wait1us;wait1us;wait1us;wait1us;wait1us;
  //delayMicroseconds(100);
  //wait1us;wait1us;wait1us;wait1us;wait1us;wait1us;wait1us;wait1us;wait1us;wait1us;
  //wait1us;wait1us;wait1us;wait1us;wait1us;wait1us;wait1us;wait1us;wait1us;wait1us;
  //for (int i=0;i<10;i++){mRawADC(rez[i],2);//delayMicroseconds(2);
  //}

  //PORTA = 0b11000000; // select input channel 1
  //for (int i=12;i<10;i++){mRawADC(rez[i],2);//if(rez[i]>60){break;}
  //}  // 1 adc(2) ~ 3us
  //for (int i=0;i<2;i++){mRawADC(rez[i],2);}  
  /*q=RawADC(2);//analogRead(8); //3-7 0 3 62 2 156
   qq=RawADC(2);//analogRead(8); //3-7 0 3 62 2 156
   qqq=RawADC(2);//analogRead(8); //3-7 0 3 62 2 156
   qqqq=RawADC(2);//analogRead(8); //3-7 0 3 62 2 156
   qqqqq=RawADC(2);//analogRead(8); //3-7 0 3 62 2 156
   */
  //delay(100);

  /*mRawADC(q,2);
   mRawADC(qq,2);
   mRawADC(qqq,2);
   mRawADC(qqqq,2);
   mRawADC(qqqqq,2);*/

  sei();
  //delay(1);
  //PORTB = 0b00000000; // select input channel 0 
  //sei();

  //delay(10);
}


void Flash1B(void)
{
  TCNT2=0;
// if(TCNT2>=33){v++;}
  __asm__ __volatile__(
  "FlashB:\n\t"
  "ldi r24,0xC2\n\t"
  "out 5,r25\n\t"
  "sts 0x007A,r24\n\t"
//  "ldi r25,69\n\t"
  "1:\n\t"
//  "dec r25\n\t"
//  "brne 1b\n\t"
  
  
  "lds r24,0x00B2\n\t" // load TCNT2
  "cpi r24,69\n\t" // jump if < 69
  "brcs 1b\n\t"

//  "lds r24,0x007A\n\t"
//  "sbrc r24,6\n\t"
//  "rjmp 1b\n\t"
/*
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t"//"nop\n\t""nop\n\t"//"nop\n\t""nop\n\t""nop\n\t""nop\n\t"
*/
  "out 5,r1\n\t"
  "ret\n\t"
  );
}

void Flash_B(byte mask)
{
PORTB=mask;
ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
//Pin2HIGH(PORTD,1); //digitalWrite(9,HIGH);//
        //    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|1;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   // NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //    delayMicroseconds(1);  
        // Pin2LOW(PORTD,1); //digitalWrite(9,LOW//   

//do{}while(bit_is_set(ADCSRA,ADSC));
  __asm__ __volatile__ ("nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  "nop\n\t"
  );
//          NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   NOP;  NOP; NOP; NOP; NOP; NOP;
        //    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        //    delayMicroseconds(10);  
        //  Pin2HIGH(PORTD,1); //digitalWrite(9,HIGH);//
        //   ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        // do{}while(bit_is_set(ADCSRA,ADSC));
        // v3=ADCW; 
        NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;


        //    delayMicroseconds(1);  
PORTB=0;
}
void Flash_C(byte mask)
{
PORTC=mask;
//Pin2HIGH(PORTD,1); //digitalWrite(9,HIGH);//
ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|1;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   // NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //    delayMicroseconds(1);  
        // Pin2LOW(PORTD,1); //digitalWrite(9,LOW//   

do{}while(bit_is_set(ADCSRA,ADSC));

        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   
        //    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        //    delayMicroseconds(10);  
        //  Pin2HIGH(PORTD,1); //digitalWrite(9,HIGH);//
        //   ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        // do{}while(bit_is_set(ADCSRA,ADSC));
        // v3=ADCW; 

/*        NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
*/

        NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;


        //    delayMicroseconds(1);  
PORTC=0;
}
void Flash_D(byte mask)
{
PORTD=mask;
//Pin2HIGH(PORTD,1); //digitalWrite(9,HIGH);//
ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|1;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   // NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //    delayMicroseconds(1);  
        // Pin2LOW(PORTD,1); //digitalWrite(9,LOW//   

do{}while(bit_is_set(ADCSRA,ADSC));

        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   
        //    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        //    delayMicroseconds(10);  
        //  Pin2HIGH(PORTD,1); //digitalWrite(9,HIGH);//
        //   ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        // do{}while(bit_is_set(ADCSRA,ADSC));
        // v3=ADCW; 

        NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        //    delayMicroseconds(1);  
PORTD=0;
}


//byte pinmask,prt;
//word max0=0,max1=0,max01=0,max10=0;

word VccN;
word VccH=1023;
word VccL=0;


void FlashTest(void) // #2 pin used as test
{
  word ch0,ch1;
  
  DrawBox(16,0,159,127,0x00,0x00,0x00);    // очистка экрана
  
  setAddrWindow(14,0,14+7,127);ta("Тестовый режим / TEST");
  setAddrWindow(22,0,22+7,127);ta("---------------------");


  setAddrWindow(30,0,30+7,127);ta("Сейчас на вывод TEST ");
  setAddrWindow(40,0,40+7,127);ta("будет подан постоян-"); 
  setAddrWindow(50,0,50+7,127);ta("ный ток. Используй");
  setAddrWindow(60,0,60+7,127);ta("подстроечный резистор");
  setAddrWindow(70,0,70+7,127);ta("для его ограничения.");
  setAddrWindow(80,0,80+7,127);ta("Максимум 700mA");

  setAddrWindow(95,0,95+7,127);ta("The continious cur-");
  setAddrWindow(105,0,105+7,127);ta("rent will be applied"); 
  setAddrWindow(115,0,115+7,127);ta("to TEST output now.");
  setAddrWindow(125,0,125+7,127);ta("Use adjustment resis-");
  setAddrWindow(135,0,135+7,127);ta("tor to limit it. Do");
  setAddrWindow(145,0,145+7,127);ta("not go beyond 700mA.");  
  
  delay(3000);

  
  
  Pin2Output(DDRD,7);Pin2HIGH(PORTD,7);  // G stop light
  Pin2Output(DDRD,5);Pin2HIGH(PORTD,5);  // SRCLR to HIGH. when it is LOW all regs are cleared
  Pin2Output(DDRB,1);
  Pin2Output(DDRD,6); 

    Pin2HIGH(PORTB,1);  // DATAPIN to HIGH  
    Pin2HIGH(PORTD,6);Pin2LOW(PORTD,6); // clock pulse 
    Pin2LOW(PORTB,1); // next data are zeroes
    Pin2HIGH(PORTD,6);Pin2LOW(PORTD,6); // clock pulse 
    Pin2HIGH(PORTD,6);Pin2LOW(PORTD,6); // clock pulse 

//    TSL2561_ON;

  
for(byte n=20;n>0;n--)
{

  LCD_OFF();
for(word z=0;z<30;z++)
{
                if(!ERR){TSLstart();} // 192us

  //    if (tsl.begin()) 
//  Wire.begin();

 // Initialise I2C
//  Wire.beginTransmission(TSL2561_ADDR_LOW);
//  Wire.write(TSL2561_REGISTER_ID);
//  Wire.endTransmission();
//  Wire.requestFrom(TSL2561_ADDR_LOW, 1);
//  byte x = Wire.read();
//  if (x==0x0A)
//  if (x & 0x0A )// FF подходит в случае отсутствия датчика
  //{
        //tsl.setGain(TSL2561_GAIN_0X);      // set 16x gain (for dim situations)
//        tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  
        //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  
    //    write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,  TSL2561_INTEGRATIONTIME_101MS |TSL2561_GAIN_0X);    //  // Set integration time and gain
        //write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,  TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_0X);    //  // Set integration time and gain
      //  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);  // enable();


for(byte q=0;q<103;q++)
{
cli();
TCNT1=0;
Pin2LOW(PORTD,7);// start lighting
GetVcc();
do{}while(TCNT1<1000);
Pin2HIGH(PORTD,7);//digitalWrite(G,HIGH); // stop light
sei();
}

  //              if(!ERR){delay(103);TSLstop();}//322us
                if(!ERR){TSLstop();}//322us
  
//  ch1 = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
//  ch0 = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);

//  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);  // disable();
//  if (ch0>max0){max0=ch0;max01=ch1;}
//  if (ch1>max1){max1=ch1;max10=ch0;}

//    }      // if begin


    }// for 100

      LCD_ON();

  setAddrWindow(152,120,152+7,127);tn(10,n);
      
  setAddrWindow(2,0,2+7,127);
      ta("ch0=");tn(10000,Light.W[0]);ta("ch1=");tn(10000,Light.W[1]);

  setAddrWindow(12,0,12+7,127);
    ta("Vcc");tn(100,VccN);ta(" L");tn(100,VccL);ta(" H");tn(100,VccH);
  setAddrWindow(22,0,22+7,127);    
    ta("Vcc");tn(100,114000L/VccN);ta(" L");tn(100,114000L/VccL);ta(" H");tn(100,114000L/VccH); // 1125300L
    
    delay(5000);

} //for n 10
 //   TSL2561_OFF;

  Pin2LOW(PORTD,5);Pin2Input(DDRD,5);  // SRCLR to LOW. Clear regs
  Pin2Input(DDRB,1);  // DATAPIN
  Pin2Input(DDRD,6); // CLK&LATCH 
  Pin2LOW(PORTD,7);Pin2Input(DDRD,7);  // detach G control pin (it is pull upped by resistor) --- no so really

  VccH=1023;VccL=0;
}

word Flashes=0; // число вспышек в секунду
long FlasheS=0; // общее число вспышек

void Flash(byte lamps,byte Duration)
{
  
  Pin2Output(DDRD,7);Pin2HIGH(PORTD,7);  // G stop light
  Pin2Output(DDRD,5);Pin2HIGH(PORTD,5);  // SRCLR to HIGH. when it is LOW all regs are cleared
  Pin2Output(DDRB,1);
  Pin2Output(DDRD,6); 
  
  for(byte n=0;n<Duration;n++)
 {
    Pin2HIGH(PORTB,1);  // DATAPIN to HIGH  
    Pin2HIGH(PORTD,6);Pin2LOW(PORTD,6); // clock pulse 
    Pin2LOW(PORTB,1); // next data are zeroes

//  Pin2Output(DDRB,6);Pin2LOW(PORTB,6);  // test latch


//  delayMicroseconds(5);



// shift register is cleared
// what is in storage register we don't care because G is HIGH and all outputs are OFF

//  delayMicroseconds(2);

//t=0;

//  if((z&7)==0){Pin2HIGH(PORTD,1);} // 1st bit is "1"
//  Pin2HIGH(PORTD,1); // 1st bit is "1"

for(byte z=0;z<lamps;z++)// serie of flashes
{
//  if((z&7)==0){Pin2HIGH(PORTD,1);} // 1st bit is "1"
  Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6); // clock pulse 
//  Pin2LOW(PORTD,1); // next data are zeroes (just delay)

//  Pin2HIGH(PORTB,6);  Pin2LOW(PORTB,6); // latch

//  delayMicroseconds(2);

//  if((z&7)==0){Pin2LOW(PORTD,1);} // next 7 are zeroes
//  Pin2LOW(PORTD,1); // next data are zeroes
  
//  delayMicroseconds(2);
// latch
//  Pin2HIGH(PORTD,6);//digitalWrite(LATCHPIN,HIGH);
//  delayMicroseconds(2);
//  Pin2LOW(PORTD,6);//digitalWrite(LATCHPIN,LOW);
//  delayMicroseconds(2);
  
//  if((it==0x0200)&&(z==0)){SetADC(0,14,100);}
  
//flash itself
//for(byte x=0;x<3;x++)// flashes
//{
//Pin2LOW(PORTD,5);delayMicroseconds(30);Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // start/stop light
//}
/*
if(it==1021)
{
  ADCSRA|=(1<<ADEN);  // start ADC
  ADMUX=(0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|14;// input 14 (Vcc) internalRef Vcc
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start 1st conversion
}
if(it==1022)// measure Vcc and stop ADC
{
if(z==0){Vcc1=ADCW;
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start conversion
  do{}while(bit_is_set(ADCSRA,ADSC));Vcc1=ADCW; //repeat
}// read 1st conversion
cli();


  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start conversion
  do{}while(bit_is_set(ADCSRA,ADSC));VccN[z][0]=ADCW; 

//enter critical section
//Pin2Input(DDRD,0); // important set it 2 input (high impedance state)
Pin2HIGH(PORTD,0);// open reset mosfet
//TCNT1=0;
Pin2LOW(PORTD,5);// start lighting
// do 3 ADC reading (27us)
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start conversion
  do{}while(bit_is_set(ADCSRA,ADSC));VccN[z][1]=ADCW; 
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start conversion
  do{}while(bit_is_set(ADCSRA,ADSC));VccN[z][2]=ADCW; 
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start conversion
  do{}while(bit_is_set(ADCSRA,ADSC));VccN[z][3]=ADCW; 
//delayMicroseconds(14);

Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // start/stop light
//tim=TCNT1;    // 27us

Pin2LOW(PORTD,0);// close reset mosfet

  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start conversion
  do{}while(bit_is_set(ADCSRA,ADSC));VccN[z][4]=ADCW; 
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start conversion
  do{}while(bit_is_set(ADCSRA,ADSC));VccN[z][5]=ADCW; 
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start conversion
  do{}while(bit_is_set(ADCSRA,ADSC));VccN[z][6]=ADCW; 
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start conversion
  do{}while(bit_is_set(ADCSRA,ADSC));VccN[z][7]=ADCW; 

}
else{
*/
//  if(it==15000){
//pinMode(A4,OUTPUT);
//pinMode(A5,OUTPUT);
//delay(1);
  // prepare light sensor
//if (tsl.begin()) {  
      //    write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);  //  enable();
//  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,  TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_16X);    //  // Set integration time and gain

//    tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
//  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
//  tsl.enable(); // will init if need to

//lvv+=0x100000L;  

//}// begin

//}
//lvv++;
  
cli();
//enter critical section
//Pin2Input(DDRD,0); // important set it 2 input (high impedance state)
//Pin2HIGH(PORTD,0);// open reset mosfet
//TCNT1=0;
Pin2LOW(PORTD,7);// start lighting

NOP;
//do{}while(TCNT1<Duration);

Pin2HIGH(PORTD,7);//digitalWrite(G,HIGH); // stop light

sei();

}// for
}

  Pin2LOW(PORTD,5);Pin2Input(DDRD,5);  // SRCLR to LOW. Clear regs
  Pin2Input(DDRB,1);  // DATAPIN
  Pin2Input(DDRD,6); // CLK&LATCH 
  Pin2LOW(PORTD,7);Pin2Input(DDRD,7);  // detach G control pin (it is pull upped by resistor) --- no so really
//  if(!TFT_IS_ON){Pin2LOW(PORTB,0);Pin2Input(DDRB,0);}

//if(it==1022){ADCSRA&=~(1<<ADEN); // stop ADC
//ADCSRA=0;
//}
    Flashes++;
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

boolean volatile UpdateRTC=true; // Update RTC at least once (to reset it if needed)
boolean volatile Update500=true; // Update once in 500ms

ISR (PCINT1_vect)  // A3
{ 
    if(++ticks>=172800){ticks=0;reboot;}
    if((ticks&0x1)==0){UpdateRTC=true;}// update clock every second
    Update500=true;
    
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

            WDhappen=0;
            WDsleep=1;// notify WD that we are sleeping (to avoid reboot)
        sleeps=0;
      do{
        sleep_enable();
        sleep_cpu();
//wake up here
// check if it us or not
        sleep_disable();
        if(WDhappen){break;}else{sleeps++;}
      }while(1);
}

void longnap_old(void)
{
//?????????????????
//PORTC=0;
//PORTB=0;
//PORTD=0;
//PORTD=0b00000100; //except 2nd pin (INT0)


     // cnt1=0;
      //tc1=TCNT1;
//        set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //// in r24,0x33// andi r24,0xF1// ori r24,0x04// out 0x33,r24
//      set_sleep_mode (SLEEP_MODE_IDLE);//// in r24,0x33// andi r24,0xF1// out 0x33,r24

//Pin2Output(DDRD,0);// sleep 2n7000 control
//Pin2HIGH(PORTD,0);// switch ON sleep control mosfet

//Pin2Output(DDRD,3);Pin2HIGH(PORTD,3);// charge cap


// all pins to  high imp. except some
//PORTC=0;
//PORTB=0;
//PORTD=0;
//DDRC=0;
//DDRB=0;
//DDRD=0;
// every pin must  be in determined state
/*
    Pin2Input(DDRB,2);Pin2LOW(PORTB,2); // SPI pins
    Pin2Input(DDRB,3);Pin2LOW(PORTB,3);
    Pin2Input(DDRD,4);Pin2LOW(PORTD,4);    
    Pin2Input(DDRB,5);Pin2LOW(PORTB,5);
*/
// не нравится...
//.. по кнопке пробуем

// instead of charging cap we must set internal pullup resistor to get SQW from DS1307
    Pin2Input(DDRD,3);Pin2HIGH(PORTD,3);// pinMode(3,INPUT_PULLUP);      attachInterrupt(1, pin3_isr, RISING);


//            WDhappen=0;
        sleeps=0;
//      TCNT1=0;
      do{

            cli();
      pin3_interrupt_flag=0;
      sleep_enable();
//      attachInterrupt(1, pin3_isr, LOW);
      attachInterrupt(1, pin3_isr, RISING);

//      Pin2LOW(PORTD,3);Pin2Input(DDRD,3); // controlled charging (very impurtant set it 2 input (high impedance state))
        sei();
        sleep_cpu();
//wake up here
// check if it us or not
        sleep_disable();
  //      if(pin3_interrupt_flag||WDhappen){break;}else{sleeps++;}
  // check if there is a ob to do or sleep further
  // 4kHz gives us 246us sleeping time
        if(pin3_interrupt_flag){break;}else{sleeps++;}
      }while(1);


/*
    Pin2Output(DDRB,2);Pin2HIGH(PORTB,2); // SPI pins
    Pin2Output(DDRB,3);Pin2LOW(PORTB,3);
//    Pin2Input(DDRD,4);Pin2LOW(PORTD,4);    
    Pin2Output(DDRB,5);Pin2LOW(PORTB,5);
*/

//Pin2Output(DDRD,0);// sreset mosfet 2n7000 control
//Pin2Output(DDRD,1);// 3.3/5v mosfet 2n7000 control
//Pin2Output(DDRD,2);// INT0 line

//Pin2Output(DDRD,3);// INT1 line /TFT CS
//Pin2Output(DDRD,4);// TFT DC
//Pin2Output(DDRB,3);// MOSI
//Pin2Output(DDRB,5);// CLK


//Pin2Output(DDRB,0); // CLOCKPIN 8
//Pin2Output(DDRD,1); // DATAPIN 9
//Pin2Output(DDRB,2);
//Pin2Output(DDRB,6);

//Pin2Output(DDRD,5); // pin 5 G
//Pin2Output(DDRD,6); // pin 6 LATCH
//Pin2Output(DDRD,7); // pin 7 SRCLR

//  pinMode(DATAPIN,OUTPUT);
//  pinMode(CLOCKPIN,OUTPUT);
//  pinMode(LATCHPIN,OUTPUT);


  
  //cli();t1111=TCNT1;sei();//atomic read
}

void nextnap(void)
{

//Pin2Output(DDRD,2);Pin2HIGH(PORTD,2);// charge cap
//delayMicroseconds(2);
//Pin2Input(DDRC,0);Pin2HIGH(PORTC,0); // pull up on A0
//PCICR |= 1<<PCIE1;
//PCMSK1 = 1<<PCINT8; // A0
Pin2Input(DDRC,3);Pin2HIGH(PORTC,3); // pull up on A0
PCICR |= 1<<PCIE1;
PCMSK1 = 1<<PCINT11; // A3


      sleeps=0;
      TCNT1=0;
//      do{
    //        cli();
            pin0_interrupt_flag=0;
            sleep_enable();
//            attachInterrupt(0, pin2_isr, LOW);

  //          Pin2LOW(PORTD,2);Pin2Input(DDRD,2); // controlled charging (very impurtant set it 2 input (high impedance state))
      //      sei();
            sleep_cpu();
//wake up here
            sleep_disable();
//break;  
//      if(pin3_interrupt_flag||WDhappen){break;}else{sleeps++;}
//            if(pin0_interrupt_flag||WDhappen){break;}else{sleeps++;} // WD is important in case  RC sleeper went off
  //        }while(1);
//  cli();t1111=TCNT1;sei();//atomic read
PCICR&=~(1<<PCIE1);
Pin2LOW(PORTC,0); // pull up off A0
}

void b7nap(void) // дрыхнет богатырским сном!
{

Pin2Output(DDRB,7);Pin2HIGH(PORTB,7);// charge cap
//delayMicroseconds(2);
//Pin2Input(DDRB,7);Pin2HIGH(PORTB,7); // pull up on B7
PCICR |= 1<<PCIE0;
PCMSK0 = 1<<PCINT6; // B7


      sleeps=0;
      TCNT1=0;
    //  do{
            cli();
            pin7_interrupt_flag=0;
            sleep_enable();
//            attachInterrupt(0, pin2_isr, LOW);

            Pin2LOW(PORTB,7);Pin2Input(DDRB,7); // controlled charging (very impurtant set it 2 input (high impedance state))
      //      sei();
            sleep_cpu();
//wake up here
            sleep_disable();
//break;  
//      if(pin3_interrupt_flag||WDhappen){break;}else{sleeps++;}
  //          if(pin7_interrupt_flag||WDhappen){break;}else{sleeps++;} // WD is important in case  RC sleeper went off
//          }while(1);
//  cli();t1111=TCNT1;sei();//atomic read
PCICR&=~(1<<PCIE0);
//Pin2LOW(PORTC,0); // pull up off A0
}

void fastnap(void)
{

Pin2Output(DDRD,2);Pin2HIGH(PORTD,2);// charge cap
//delayMicroseconds(2);

      WDhappen=0;
      sleeps=0;
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
            if(pin2_interrupt_flag||WDhappen){break;}else{sleeps++;} // WD is important in case  RC sleeper went off
          }while(1);
//  cli();t1111=TCNT1;sei();//atomic read
}
void longnap(void)
{
      Pin2Output(DDRD,3);Pin2HIGH(PORTD,3);// charge cap

      WDhappen=0;
      sleeps=0;
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
            if(pin3_interrupt_flag||WDhappen){break;}else{sleeps++;} // WD is important in case  RC sleeper went off
          }while(1);
//  cli();t1111=TCNT1;sei();//atomic read
}


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

void ShowBars(void)
{
  byte r,g,b;

//  setAddrWindow(70,4,77,123);
  //ta("hr:");t3(hr);

  

  
  // m max=11
  //16
  // *15/m
  
  // gradient background
//  setAddrWindow(0,4,15,123);
NiceBack(0,0,128,15);
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
  for(byte i=0;i<16;i++)
  {
//    gg=(Intensity[i]*14)/mi;if(!gg){gg=1;} 
    byte gg=Intensity[i];//if(!gg){gg=1;} 

    if(i!=HR){r=0x8c;g=0xac;b=0x8c;}else{r=0x8c;g=0xfc;b=0x4c;}
    DrawBox(16-gg,i*8,15,i*8+6,r,g,b);    
  }
    DrawBox(17,HR*8,17,HR*8+6,0x8c,0xfc,0x4c);    

}

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
  //  __asm__ __volatile__("wdr\n\t");//  wdt_reset(); // to avoid WD fire first
    longnap();
    if (pin3_interrupt_flag){longnaptime=cnt1;}else { ERR=ERR_BROKEN_SLEEP; }

//    WDsleep=1;// notify WD that we are sleeping (to avoid reboot)
    //__asm__ __volatile__("wdr\n\t");//  wdt_reset(); // to avoid WD fire first
  //  nextnap();
    //if (pin0_interrupt_flag){nextnaptime=cnt1;}else { ERR=ERR_BROKEN_SLEEP; }
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

boolean Touched(void)
{
    byte x,e=2;

    for (x=0;x<3;x++){ delay(700); Cherry(15+x*36,100); }
    do        
    {
        for (x=0;x<4;x++){ if (TouchSensor()>Etouch){ return true; } delay(200); }
        DrawBox(100,15+e*30,123,15+23+e*36,0x00,0x00,0x00);    
         if(e==0){break;}
         e--;
     }while(1);
     return false;
}

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
}


#define LONG_TOUCH_THRESHOLD 4 // критерий длинного нажатия в  секундах

  volatile long lvv,lmv,lm2;// luminous
  volatile long mlvv,mlmv,mlm2;// luminous
  volatile word lasttouch,ltp,CurrentTouch;
  byte LongTouch=0;
word rtcl;
long uptime=0; // uptime в секундах

void UpdateScreen(void)
{
//    DrawBox(0,0,159,127,0x00,0x00,0x00);    // очистка экрана

    setAddrWindow(20,0,27,127);
  ta("ERR");th(ERR);
    ta(" CT");tn(1000,CurrentTouch);
    ta(" Ti");tn(100000,ticks);
    ta(" H");tn(10,HR);
//    th(rtc8);ta(" fn");tn(10000,fastnaptime);ta(" ln");tn(10000,longnaptime);
    setAddrWindow(30,0,37,127);
    
    ta("Et");tn(1000,Etouch);ta(" rtcl");tn(10000,rtcl);ta(" Fi");th(FlashIntensity);

    setAddrWindow(40,0,47,127);
    ta(" it");tn(10000,it);ta(" LT");th(LongTouch);   ta(" sleeps");tn(10,sleeps); 

    setAddrWindow(50,0,57,127);
    //ta("sss=");tn(100000,sss);
    for(byte w=1;w<9;w++){th(TimeS[w]);ta(":");}
    

//3.61v - 319 3.93v -291     5v~190

    setAddrWindow(60,0,67,127);
    
//    ta(" sss=");tn(1000,sss);    
//    ta(" bbb=");tn(1000,bbb);    
    ta(" if=");th((pin0_interrupt_flag<<5)|(pin2_interrupt_flag<2)|(pin3_interrupt_flag<<1)|(WDhappen));
    ta("Vcc");tn(100,VccN);ta(" L");tn(100,VccL);ta(" H");tn(100,VccH);
    
    
    setAddrWindow(70,0,77,127);

TCNT1=0;
    ta("Vcc");tn(100,114000L/VccN);ta(" L");tn(100,114000L/VccL);ta(" H");tn(100,114000L/VccH); // 1125300L
rtcl=TCNT1; 
    

    setAddrWindow(80,0,87,127);

ta(" f");tn(10000,fastnaptime);ta(" l");tn(10000,longnaptime);   ta(" n");tn(10000,nextnaptime);   

    setAddrWindow(90,0,97,127);
//    ta("ch0=");tn(1000,chan0);ta("ch1=");tn(1000,chan1);
    ta("ch0=");tn(1000,Light.W[0]);ta("ch1=");tn(1000,Light.W[1]);

    setAddrWindow(100,0,107,127);
  
   ta("grr ");tn(100,grr);  ta(" ");th(grr);ta(" ");th(grr2);
  //  ta("FCPU ");tn(1000000,I2C_CPUFREQ);
//    ta("DC ");lh(I2C_DELAY_COUNTER);

    setAddrWindow(110,0,117,127);
    ta("uptime");tn(10000,uptime);
    setAddrWindow(120,0,127,127);
    ta(" fps");tn(1000,Flashes);
    ta(" ");tn(1000000000,FlasheS);


    Cherry(90,90);
    


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
    
    

    setAddrWindow(152,0,159,127);ta("-----==<АУРА>==-----3");

}

void GetVcc(void){ VccN=Vcc(); if (VccN<VccH){VccH=VccN;} if (VccN>VccL){VccL=VccN;} } //280us

// the loop routine runs over and over again forever:
void loop() {
  long now;
//  long oldnow=millis();
  word t,t1,n;
  word Temp;

  
  do{
//        __asm__ __volatile__("wdr\n\t");//  wdt_reset();
        if(it==0){SleepTime();}

//        Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // stop light outputs
//        Pin2HIGH(PORTB,6);//power supply to tpic6a595  (add caps?)
  //      delayMicroseconds(11);// wait for rise. 10 minimum to avoid nasty bugs

//Pin2Output(DDRB,7);Pin2HIGH(PORTB,7);// включаем питание  дисплея

//Pin2Input(DDRD,0);Pin2HIGH(PORTD,0); // pull up on D0
//PCICR |= 1<<PCIE2;
//PCMSK2 = 1<<PCINT16; // D0

  if (Update500) // every 500ms
  {
      Update500=false;
      GetVcc();  
   
    
  }

  if (UpdateRTC) // every second
  {
      UpdateRTC=false;
      uptime++;
      RTC(); // 500us


      CurrentTouch=TouchSensor();  //337-440us
      
      // update screen?
      if(LCD)
      {
           if (LCD<=ticks){LCD_OFF();}
           else
           {    
//                if(!ERR){TSLstart();} // 192us
  //              if(!ERR){delay(103);TSLstop();}//322us

                UpdateScreen();          
           }
       }
       else
       {

               if (CurrentTouch>=Etouch){
         LCD=ticks+30;// 15sec
      LCD_ON();
     
     // draw backgrounds
     ShowBars();
               }
       }

      if (CurrentTouch>=Etouch){ if ((++LongTouch==LONG_TOUCH_THRESHOLD)&&(LCD)){LCD+=900;Settings();LCD=ticks+20;} }
      else{ LongTouch=0; TouchSample(); }

      FlasheS+=Flashes;Flashes=0;
 }



/*

if(ticks>=NextTouch)
{
TCNT1=0;
    CurrentTouch=TouchSensor();  //337-440us
rtcl=TCNT1; 

//    if (CurrentTouch<Etouch)
  //  {
        Etouch=TouchT(); //7us
    //}



  if (CurrentTouch>=Etouch)
  {
      if ((++LongTouch==LONG_TOUCH_THRESHOLD)&&(LCD)){Settings();LCD+=20;} 
  }
  else
  {
      LongTouch=0;
  }

if (!LCD)
{
  if (CurrentTouch>=Etouch)
  {
      LCD=ticks+30;// 15sec
      LCD_ON();
     NextScreen=ticks+1; 
     
     // draw backgrounds
     ShowBars();
     
  }

    if(ticks>=NextTouchSample){TouchSample();NextTouchSample=ticks+100;}// подстройка раз в 50 секунд при выключенном экране
}


    NextTouch=ticks+1;
}
*/


//    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //// in r24,0x33// andi r24,0xF1// ori r24,0x04// out 0x33,r24
//    set_sleep_mode(SLEEP_MODE_IDLE);  //// in r24,0x33// andi r24,0xF1// ori r24,0x04// out 0x33,r24
  //  sss=0;nextnap();sssn=sss;delay(100);sssb=sss;

//        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//       wdt_disable();
     set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
 //   set_sleep_mode(SLEEP_MODE_IDLE);  //// in r24,0x33// andi r24,0xF1// ori r24,0x04// out 0x33,r24

  TCNT1=0;
//  longnap();//4845us 3 sleeps
  fastnap();//957us - 
//b7nap();//522us

//delay(1000);

//unap(T1S);

//unap();// 66 sleeps IDLE
//unap();// 17 sleeps PWR_DOWN
   //   wdt_disable();
      
// nextnap();// 16-52us  (wakeups from power down also)
//delay(5000); // 10 interrupts (every 1/2 second) - 
  //rtcl=TCNT1;

//detachInterrupt(0);  Pin2LOW(PORTD,2);//
        
       // fastnap();
       // sssn=sss;
//        nextnap(); // never wakeup (RTC is OFF  >> SQW is present
     //   sssb=bbb;

//pinMode(6,OUTPUT);//analogWrite(6,10);

//if(!TFT_IS_ON){

  //   set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
//unap();
//analogWrite(6,0);
          
//          continue;
        
  //          if (CurrentTouch>Etouch){LongTouch++;if(TFT_IS_ON){TFT_IS_ON+=7;} if(LongTouch==LONG_TOUCH_THRESHOLD){Settings();} }else{LongTouch=0;}
    //        if(TFT_IS_ON) { if(LongTouch<2){if(--TFT_IS_ON==0){TFT_OFF();}else{UpdateScreen();}}}  // если дисплей включен то проверим не пора ли его выключить
      //      else {ltp=lasttouch;lasttouch=CurrentTouch; if (lasttouch>Etouch){TFT_ON(15);ShowBars(HR);}else{TouchD[((TouchPos++)&3)]=lasttouch;}}
         // }
   //}
 
  
  //RTC_OFF();  // переводим лапки часиков в высокоомное состояние  и отключаем питание
//  rtcl=TCNT1;
  
//  I2C_ON(DDRC,1,2);
  //if(!Check_TSL(16)){ERR=WHERE_IS_THE_TSL2561;} 
//  else{

//TSL2561_START(TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_16X);//
//TSL2561_START(TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_0X);////
//delay(14);
//lvv=TSL2561_STOP();

//}
  //I2C_OFF(DDRC,PORTC,1,2);
  
//}

/*
*/


// если есть ошибки
if (ERR)
{
  //  RTC_ON();
    TFT_ON(3);
//    fillScreen(0x000000);
    DrawBox(0,0,159,127,0x00,0x00,0xfc);    // очистка экрана

    word cycles=TouchSensor();
    setAddrWindow(0,0,7,119);
    ta("ERR");wh(ERR);ta(" fn");tn(10000,fastnaptime);ta(" ln");tn(10000,longnaptime);
    setAddrWindow(10,0,17,119);
    ta("cycles:");tn(10000,cycles);

    delay(5500);
//    RTC_OFF();  // переводим лапки часиков в высокоомное состояние 
    TFT_OFF(); // close rtft/rtc mosfet
    reboot;  // This will call location zero and cause a reboot.
}

//continue;
//  if(!FlashIntensity){unap();continue;} // определяем продолжительность пыхи в данном часе и спим 8s если нечего делать

//if(((it&0x3FF)==0)&&TFT_IS_ON)// once in 1k
//{

// check that we have sensor responding
/*
byte vv=_readRegisterT(TSL2561_REGISTER_ID); // works

_writeRegisterT(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
_writeRegisterT(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,  TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_16X);   

delay(14);

long lm=_readRegisterT(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN1_HIGH);
  lm <<= 8;
lm|=_readRegisterT(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN1_LOW);
  lm <<= 8;
lm|=_readRegisterT(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN0_HIGH);
  lm <<= 8;
lm|=_readRegisterT(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN0_LOW);

// lm = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
  //lm <<= 16;
//  lm |= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);

_writeRegisterT(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
*/
/*
    I2C_ON(DDRC,4,5); // не работает вместе с wire.h точно
    if(Check_TSL(16))
    {  
//      lvv=Check_TSL(16); //16
      
//    TSL2561_START(TSL2561_INTEGRATIONTIME_101MS |TSL2561_GAIN_0X); 
    TSL2561_START(TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_0X); 
     delay(120);
      lvv=TSL2561_STOP();
      if (lvv>mlvv) mlvv=lvv;
      
    }
    I2C_OFF(DDRC,PORTC,4,5);*/

/* // измерялка фотонов
    if (tsl.begin()) {
    tsl.setGain(TSL2561_GAIN_0X);      // set 16x gain (for dim situations)
      tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
  
  delay(120);
  
  lmv = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
  lmv <<= 16;
  lmv |= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
  if (lmv>mlmv)mlmv=lmv;

    }

    if (tsl.begin()) {
    tsl.setGain(TSL2561_GAIN_0X);      // set 16x gain (for dim situations)
      tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
  
  delay(14);
  
  lm2 = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
  lm2 <<= 16;
  lm2 |= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
    if (lm2>mlm2)mlm2=lm2;

    }
*/    

//}

//if((it&0x3FF)==0)// once in 1024
//{

  //  t=0;  

  //once in 65536
 // if(it==0xFFFF){flashes++;}


if(FlashIntensity)
{
//    Flash(8,FlashIntensity);
    Flash(16,FlashIntensity);
//    Flash(16,1);

//    Flash(9,FlashIntensity);
/*    
//  if(it==1000){lmv=FlashM(8,FlashIntensity);}// измерение 0x0D пых=12us
  if(it==1000){lmv=0;lvv=0;lm2=0;
  

  if (tsl.begin()) {
    tsl.setGain(TSL2561_GAIN_0X);      // set 16x gain (for dim situations)
      tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // shortest integration time (bright light)
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
  
    delay(2);
    TCNT1=0;
    for( word i=0;i<50;i++){FlashZ(1,16);} 
    tt1=TCNT1;    
     delay(100);
  
  lvv = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
  lvv <<= 16;
  lvv |= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);

  } 
  
delay(1000);

if (tsl.begin()) {
    tsl.setGain(TSL2561_GAIN_0X);      // set 16x gain (for dim situations)
      tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // shortest integration time (bright light)
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
  
    delay(2);
    TCNT1=0;
    for( word i=0;i<200;i++){FlashZ(1,4);} 
    tt1=TCNT1;    
     delay(100);
  
  lmv = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
  lmv <<= 16;
  lmv |= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);

  } 
  delay(1000);
  if (tsl.begin()) {
    tsl.setGain(TSL2561_GAIN_0X);      // set 16x gain (for dim situations)
      tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // shortest integration time (bright light)
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
  
    delay(2);
    TCNT1=0;
    for( word i=0;i<800;i++){FlashZ(1,1);} 
    tt1=TCNT1;    
     delay(100);
  
  lm2 = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
  lm2 <<= 16;
  lm2 |= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
  } 
*/  
  
    
} // пыхнем


  
  //
// for(int i=0;i<10;i++){
  
  //digitalWrite(5,LOW);
  
  
//  for(int i=0;i<100;i++) {
//  cli();

//Pin2LOW(PORTD,6);//digitalWrite(LATCHPIN,LOW);///already low

  //delay(2000);


  //for(long i=0;i<3000000;i++){NOP;}//~2100ms delay
  //  pinMode(A5, INPUT); //LCD led backlight
  //digitalWrite(A5,LOW);
  //  pinMode(A5, OUTPUT); //LCD led backlight
  //-------------------------------------------------------------------------------------------------[ power saving wait state ]
  // digitalWrite(RST,LOW);
  // digitalWrite(RST,HIGH);

  //    Pin2LOW(PORTB,2); //digitalWrite(10,LOW);//
  // Pin2HIGH(PORTB,2);//digitalWrite(10,HIGH//

  //  Pin2LOW(PORTD,1); //digitalWrite(9,LOW//
  //    Pin2Input(DDRD,1);//  pinMode(9,INPUT);

//  Pin2LOW(PORTB,5); //SPI SCK pin low
 // Pin2LOW(PORTB,3); //SPI MOSI pin low

  //    Pin2LOW(PORTB,5); //SPI SCK pin low
 // Pin2LOW(PORTB,2); //SPI SS pin low


  //Pin2LOW(PORTD,0); //vcc A to low

  //  PORTC=0; // switch off PORTC control pins
  //    Pin2LOW(PORTC,3); //digitalWrite(1A3HIGH);//  power to current sensor

  //PORTC=0x7; // select channel 7 (some unused channel for debug)




//      pinMode(2,OUTPUT);digitalWrite(2,HIGH);delayMicroseconds(65);digitalWrite(2,LOW);pinMode(2,INPUT);// controlled charging(~100us)

//  SPCR&=~(1<<SPE); //  SPI.end();
//  ADCSRA&=~(1<<ADEN); //turn off ADC 
//  ACSR = (1<<ACD); // turn off analog comparator


  //set_sleep_mode (SLEEP_MODE_IDLE);// 29.6ma - don't work
  //set_sleep_mode (SLEEP_MODE_ADC);// 6.3ma clock is off
  //set_sleep_mode (SLEEP_MODE_PWR_SAVE);// 3.7ma
  //set_sleep_mode (SLEEP_MODE_STANDBY);// 2.7ma 
  //set_sleep_mode (SLEEP_MODE_PWR_DOWN);// 0.76ma




/*
NOP;NOP;
//tim=TCNT1;    //291
//rnd=(word(word(rand()*109)+89))%251;//rnd=rand(); // ~210us????????????????????????
//rnd=rand();// still long time
rnd=55;//if((it&0xF)==0){rnd=10;}
// ADC for rnd?
NOP;NOP;

    */


//if(rnd==0){break;}// long sleep. 8s?
/*
if((it&0xFFFF)==0)
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //// in r24,0x33// andi r24,0xF1// ori r24,0x04// out 0x33,r24
if((it==55555)&&(!TFT_IS_ON))// ultra long nap 8s once  in somewhere  2 minutes
{
  unap();
  // reprogram the WDT  back to 16ms
  cli();
//  NOP;
  __asm__ __volatile__("wdr\n\t");//  wdt_reset();
//  NOP;
  MCUSR &= ~(1<<WDRF);  // Clear the reset flag. 
  WDTCSR |= (1<<WDCE) | (1<<WDE); //  In order to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles).
 WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//15ms (16280us)
 //  WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//30ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (0<<WDP0);//60ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0);//120ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0);//240ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);//480ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);//960ms
 // WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);//2s
  // WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//4s
//    WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s
    
  sei();

}
else{ln++;longnap();fn++;fastnap();}  
//else {fn++;fastnap();} 
//else {sss=0;unap();} 
//if ((it&0x3)==0){ln++;longnap();}else {fn++;fastnap();} // 1:3

}*/


/*  
        Pin2Output(DDRD,2);
      Pin2HIGH(PORTD,2);// start charging timeout capacitor
      cnt1=0;
      tc1=TCNT1;
      TCNT1=0;
  
      do{

            cli();
            WDhappen=0;
      pin2_interrupt_flag=0;
      sleep_enable();
      attachInterrupt(0, pin2_isr, LOW);
      ticks=0;
      set_sleep_mode (SLEEP_MODE_IDLE);
     //   set_sleep_mode(SLEEP_MODE_PWR_DOWN);  


      // power saving
//      PORTB=0;
  //    PORTC=0;
    //  PORTD=0;
//      SPCR&=~(1<<SPE); //  SPI.end();
  //    ADCSRA&=~(1<<ADEN); //turn off ADC 
    //  ACSR = (1<<ACD); // turn off analog comparator


      Pin2LOW(PORTD,2);
      Pin2Input(DDRD,2); // controlled charging (very impurtant set it 2 input (high impedance state))

        sei();
        sleep_cpu();
//wake up here
// check if it us or not
        sleep_disable();
        if(pin2_interrupt_flag||WDhappen){break;}
      }while(1);

  */
  
  
  //    Pin2Output(DDRD,2);Pin2HIGH(PORTD,2);// start charging timeout capacitor (default state)

  

  // in r24,0x33
// andi r24,0xFE
// out 0x33,r24


//  NOP;

//  wdt_disable();// some serious stuff (its role in hangups prevention?) why disable?
  __asm__ __volatile__("wdr\n\t");//  wdt_reset(); why disable just reset WDT


//Pin2Output(DDRD,0);// sreset mosfet 2n7000 control
//Pin2Output(DDRD,1);// 3.3/5v mosfet 2n7000 control
//Pin2Output(DDRD,2);// INT0 line

//Pin2Output(DDRD,3);// INT1 line /TFT CS
//Pin2Output(DDRD,4);// TFT DC
//Pin2Output(DDRB,3);// MOSI
//Pin2Output(DDRB,5);// CLK




//Pin2Output(DDRB,0); // CLOCKPIN 8
//Pin2Output(DDRD,1); // DATAPIN 9
//Pin2Output(DDRB,2);
//Pin2Output(DDRB,6);


//Pin2Output(DDRC,2);
//Pin2Output(DDRC,3);
//Pin2Output(DDRC,4);
//Pin2Output(DDRC,5);

//Pin2Output(DDRD,5); // pin 5 G
//Pin2Output(DDRD,6); // pin 6 LATCH
//Pin2Output(DDRD,7); // pin 7 SRCLR



//  pinMode(DATAPIN,OUTPUT);
//  pinMode(CLOCKPIN,OUTPUT);
//  pinMode(LATCHPIN,OUTPUT);

//NOP;
it++;
//oldnow=now;
  //delay(1000);               // wait for a second
    }while(1); // loop
}


