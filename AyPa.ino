//#define __AVR_ATmega328P__ 
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <Arduino.h>



#include <SPI.h>


//#include <SoftI2CMaster.h>

#include "AyPa_m.h"
#include "AyPa_fonts.h"
#include "AyPa_n.h"
//#include "AyPa_TFT.h"
//#include "AyPa_i2c.h"
//#include "AyPa_rtc.h"

//#include "DHT.h"
//#define DHTPIN A0     // what pin we're connected to
// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

//DHT dht(DHTPIN, DHTTYPE);

void (* reboot) (void) = 0; //declare reset function @ address 0

/*
#include <Wire.h>
//#include "TSL2561.h"
//TSL2561 tsl(TSL2561_ADDR_LOW); 

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


#define SetADC(bandgap,input,us){ ADCSRA|=(1<<ADEN);delayMicroseconds(2);ADMUX=(bandgap<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input;delayMicroseconds(us);} // input (0..7,8,14) (bg/vcc analogReference )
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
/*
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
*/
/*
long LCD;

void LCD_ON(void){
    Pin2Output(DDRB,0);Pin2HIGH(PORTB,0); 
//    Pin2Output(DDRB,0);Pin2LOW(PORTB,0); 
  
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
    Pin2LOW(PORTB,0); // sink charge first  then to input
    Pin2HIGH(PORTB,0); // sink charge first  then to input
//    Pin2LOW(PORTD,4); 
//    Pin2LOW(PORTD,1); 
    Pin2LOW(PORTB,2); Pin2Input(DDRB,2); 
    Pin2LOW(PORTB,3); Pin2Input(DDRB,3); 
    Pin2LOW(PORTB,5); Pin2Input(DDRB,5); // need to close all  connected pins before sinking  remaining charge. otherwise it will be suck current from them :)

    LCD=0; 

//    delayMicroseconds(100);
//     Pin2Input(DDRD,1); 
//     Pin2Input(DDRD,4); 
//    Pin2LOW(PORTB,0); // sink charge first  then to input
     Pin2Input(DDRB,0); 
  //  Pin2Input(DDRB,0); // continue to sink. D4 has only 0.7v then
}// вЫключаем питание  дисплея
*/
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

//#define FLASH_CYCLE 95 // microseconds (+~5)
//#define FLASH_CYCLE 65 // microseconds (+~5)

//byte Intensity[24] = {0,0,0,0,0,0, 1,2,3,3,3,3, 3,3,3,3,3,3, 3,3,3,3,2,1}; // почасовая интенсивность 
byte Intensity[24] = {4,0,0,0,0,0, 1,2,3,4,4,4, 4,4,4,4,4,4, 4,4,4,3,2,1}; // почасовая интенсивность 
byte FlashIntensity=0;

byte volatile ticks=0; // 1/2с
byte HR=0;
//byte MN;
//byte SC;

byte __attribute__ ((noinline)) unBCD(byte bcd){return (((bcd>>4)*10)+(bcd&0xF)); }

void RTC(void)
{  
    Pin2Input(DDRC,4);Pin2Input(DDRC,5);Pin2HIGH(PORTC,4);Pin2HIGH(PORTC,5);   // activate internal pullups for twi.
// delayMicroseconds(100);
    RequestFrom((0x68<<1),7);
grr=TWSR;
    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);         // proceed with reading + send ack
    wait4int();//    if (TWSR!=0x50){ ERR=ERR_I2C; return;} // ack sent
grr2=TWSR;

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
    else{ ERR=ERR_WHERE_IS_THE_CLOCK; SetTime(); }
    if(!ERR) { TWCR = (1<<TWEN) |  (1<<TWINT) | (1<<TWSTO); }       // send stop and forget     // wait for stop condition to be exectued on bus  (TWINT is not set after a stop condition!)

    Pin2LOW(PORTC,4);Pin2LOW(PORTC,5);Pin2Input(DDRC,4);Pin2Input(DDRC,5); // если STOP не успевает - поставь задержку
}

typedef union{byte B[4];word W[2];long L[1];} Data4;
Data4 Light;
Data4 LightMax0;
Data4 LightMax1;

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

void TSLstart(byte IntGain)
{    
  
    Pin2Input(DDRC,4);Pin2Input(DDRC,5);Pin2HIGH(PORTC,4);Pin2HIGH(PORTC,5);   // activate internal pullups for twi.  
    WriteByte((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING), IntGain );
    WriteByte((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL), TSL2561_CONTROL_POWERON);
    if (TWSR!=0x28){ ERR=ERR_WHERE_IS_THE_TSL2561; } 
    TWCR = (1<<TWEN) |  (1<<TWINT) | (1<<TWSTO);   // send stop condition
    
//    Wire.begin();        write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON,0x29);  //  enable();
//  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,  IntGain ,0x29);    //  // Set default integration time and gain

}

void TSLstop(void)
{
/*
  grr=read8(TSL2561_REGISTER_ID,0x29);
       Light.W[1]= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW,0x29);
     Light.W[0]= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW,0x29);
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF,0x29);
*/
      

    RequestFrom((0x29<<1),(TSL2561_REGISTER_ID));
    
    TWCR = (1<<TWEN) | (1<<TWINT) | (0<<TWEA);         // proceed with reading + ack
    wait4int();//    if (TWSR!=0x50){return false;} // ack sent
    if ((TWDR&0xF)!=0xA){ ERR=ERR_WHERE_IS_THE_TSL2561; } 

/*
   RequestFrom((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN1_LOW));

   TWCR = (1<<TWEN) | (1<<TWINT) | (0<<TWEA);         // proceed with reading + ack
   wait4int();//    if (TWSR!=0x50){return false;} // ack sent
   grr2=TWDR;
   Light.W[1]=TWDR;

   RequestFrom((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN1_HIGH));

   TWCR = (1<<TWEN) | (1<<TWINT) | (0<<TWEA);         // proceed with reading + ack
   wait4int();//    if (TWSR!=0x50){return false;} // ack sent
   grr=TWDR;
   Light.W[1]|=(TWDR<<8);

   RequestFrom((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN0_LOW));

   TWCR = (1<<TWEN) | (1<<TWINT) | (0<<TWEA);         // proceed with reading + ack
   wait4int();//    if (TWSR!=0x50){return false;} // ack sent
  // grr2=TWDR;
   Light.W[0]=TWDR;

   RequestFrom((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN0_HIGH));

   TWCR = (1<<TWEN) | (1<<TWINT) | (0<<TWEA);         // proceed with reading + ack
   wait4int();//    if (TWSR!=0x50){return false;} // ack sent
//   grr=TWDR;
   Light.W[0]|=(TWDR<<8);
   */

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


WriteByte((0x29<<1),(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL), TSL2561_CONTROL_POWEROFF);  ///  глючит ну и ладно
//  if (TWSR!=0x28){ ERR=ERR_WHERE_IS_THE_TSL2561; return;} 
grr2=TWSR;
  
    TWCR = (1<<TWEN) |  (1<<TWINT) | (1<<TWSTO);   // send stop condition
    Pin2LOW(PORTC,4);Pin2LOW(PORTC,5);Pin2Input(DDRC,4);Pin2Input(DDRC,5); // если STOP не успевает - поставь задержку

}

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
 
//#define system_sleep(mode,flags) { set_sleep_mode(mode); sleeps=0;do{sleep_enable();sleep_cpu();sleep_disable();if(flags){break;}else{sleeps++;}}while(1);}
 
// Watchdog timeout values
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms, 6=1sec, 7=2sec, 8=4sec, 9=8sec
#define setup_watchdog(timeout){cli(); __asm__ __volatile__("wdr\n\t"); MCUSR&=~(1<<WDRF);WDTCSR|=(1<<WDCE)|(1<<WDE);WDTCSR=((1<<WDIE)|timeout);sei();}
 
// Turn off the Watchdog
// Watchdog sleep function - combines all the above functions into one
//#define watchdogSleep(mode,timeout){setup_watchdog(timeout);system_sleep(mode,WDhappen);wdt_disable();}
 
 //volatile word  cnt1; // vars updated in ISR should be declared as volatile and accessed with cli()/sei() ie atomic

ISR(WDT_vect) // Watchdog timer interrupt.
{ 
  //reboot();  //reboot
  //  r2=TCNT1;
 // if(WDsleep)
  //{
//  cnt1=TCNT1;
 // WDhappen=1;
  //WDsleep=0;
  //}
  //else{
//  SaveUptime(); 
  reboot(); //}// This will call location zero and cause a reboot.
}

byte DHTdata[5];

byte DHT_ReadData(void) 
{ 
   byte i; 
   byte v = 0; 
   word d;

   for(i=0; i<8; i++) 
   { 
//      d=0;while (!digitalRead(A0)){if(--d==0){return 0xBA;}}; 
      d=10000;while (!(PINB&(1<<1))) {if(--d==0){return 0xBA;}}; 
      delayMicroseconds(30);
//      if (digitalRead(A0)) 
      if (PINB&(1<<1)) 
      { 
          v = v |(1<<(7 - i)); 
//          d=0;while (digitalRead(A0)){if(--d==0){return 0xBA;}}; 
          d=10000;while (PINB&(1<<1)){if(--d==0){return 0xBA;}}; 
      } 
   } 
   return v;    
} 

//#define TIMEOUT (F_CPU/1600);
#define TIMEOUT (8000000L/400);
//#define TIMEOUT (8000000L/1200);
#define DHTLIB_OK				0
#define DHTLIB_ERROR_CHECKSUM	-1
#define DHTLIB_ERROR_TIMEOUT	-2
#define DHTLIB_INVALID_VALUE	-999

extern unsigned long timer0_millis;
extern unsigned long timer0_overflow_count;

unsigned long mmicros(void)
{
  unsigned long m;
//  long m;
  uint8_t t;
  
  m=timer0_overflow_count;
  t=TCNT0;
  
  return ((m<<8)+t)*64/8; // 8 clocks per microsecond
}

// TCNT0 8bit 256values with 1/64 prescaler  1value is 8microseconds - 40us=5 TCNT0 steps


int dhtread(void) //PB1 (9)
{
    // INIT BUFFERVAR TO RECEIVE DATA
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


boolean DHTread(void) {
  byte i;
  word d;
  boolean res=false;
  
  Pin2Input(DDRB,1);Pin2HIGH(PORTB,1);//  digitalWrite(A0, HIGH);  // internal pull up
  delay(500);
Pin2Output(DDRB,1);//  pinMode(A0, OUTPUT);
Pin2LOW(PORTB,1);//  digitalWrite(A0, LOW);
  delay(25);
Pin2HIGH(PORTB,1);//  digitalWrite(A0, HIGH);
  delayMicroseconds(30);
Pin2Input(DDRB,1);//  pinMode(A0, INPUT);

  

//  d=0;while (!digitalRead(A0)){if(--d==0){return false;}}; // Here we wait while the DHT_IO pin remains low..... 
//  d=0;while (digitalRead(A0)){if(--d==0){return false;}};    // Here we wait while the DHT_IO pin remains high..... 
  d=10000;while (!(PINB&(1<<1))){if(--d==0){Pin2Input(DDRB,1);Pin2HIGH(PORTB,1);return false;}}; // Here we wait while the DHT_IO pin remains low..... 
  d=60000;while (PINB&(1<<1)){if(--d==0){Pin2Input(DDRB,1);Pin2HIGH(PORTB,1);return false;}};    // Here we wait while the DHT_IO pin remains high..... 
  
 // res=true;
  for (i=0; i<5; i++){ DHTdata[i] = DHT_ReadData(); } 
  if  (DHTdata[4] == ((DHTdata[0] + DHTdata[1] + DHTdata[2] + DHTdata[3]) & 0xFF)) {res=true;}  // checksum  check

Pin2Input(DDRB,1);
Pin2HIGH(PORTB,1);

  return res;
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
//  return res;
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

word ADCresult;
//byte volatile ADCready=0;
byte volatile ADCfree=1;

#define moisture_input A0
#define divider_top A2
#define divider_bottom A1
//int moisture; // analogical value obtained from the experiment

word NextSoilMoistureCheck=0;
word NextTmpHumCheck=0;
word volatile uptime=0; // uptime в секундах

word MCUtemp;

word moisture;
void SoilMoisture(){
//  int reading;
  // set driver pins to outputs
  pinMode(divider_top,OUTPUT);
  pinMode(divider_bottom,OUTPUT);

  // drive a current through the divider in one direction
  digitalWrite(divider_top,LOW);
  digitalWrite(divider_bottom,HIGH);

  // wait a moment for capacitance effects to settle
  delay(300);

  while(!ADCfree);
  ADCfree=0;// to avoid collision with Vcc  
//  while(!ADCready);// у нас примерно 125 раз в секунду измеряется напряжение питание. здесь мы пожертвуем одним измерением ради датчика влажности почвы
//  cli();
//TCNT1=0;
//    SetADC(1,0,500); // A0
  //     mRawADC(reading,2);
  moisture=analogRead(moisture_input);  // take a reading
  
      SetADC(1,8,500);  //  select temperature sensor 352 (need calibration)  
       mRawADC(MCUtemp,2); // прочитаем заодно и этот датчик
  
      SetADC(0,14,1); // restore vcc adc channel
   ADCfree=1;


//  reading=TCNT1;
//  ADCready=0;
  //sei();

  // reverse the current
  digitalWrite(divider_top,HIGH);
  digitalWrite(divider_bottom,LOW);

  // give as much time in 'reverse' as in 'forward'
  delay(300);

  // stop the current
  digitalWrite(divider_bottom,LOW);


// back to high imp state
  pinMode(divider_top,INPUT);
  pinMode(divider_bottom,INPUT);
  digitalWrite(divider_top,LOW);
  digitalWrite(divider_bottom,LOW);

//  return reading;
}

//byte pp[10]={
//  5,5,5,5,5,5,5,5,5,5}; //port
//byte pb[10]={
//  1,0,0,0,0,0,0,0,0,0}; // bit in port
// the setup routine runs once when you press reset:
byte v;
byte   extreset;

//word uptimeS;
word bts=0; // timestamp

//byte setupdone=0;
byte portbmask;
byte ist=0;

//long ca,cb,cc;

void setup() {                
//  byte pmask,idx;

  wdt_disable();

//if (setupdone){  LcdInit();LcdClear();ta("2nd run of setup?"); delay(10000);}
     // Pin2Output(DDRD,2);Pin2HIGH(PORTD,2);// start charging timeout capacitor (default state)

  extreset=MCUSR;
  
    Pin2Input(DDRC,3);Pin2HIGH(PORTC,3); // pull up on A3 (user button)
  //delay(100);// pull up settle time
  
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

    PCMSK1 = 1<<PCINT11; // setup pin change interrupt on A3 pin 
    PCICR |= 1<<PCIE1; 

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
  TCCR1A=0x00;
  //  TCCR1B=(1 << WGM12)|(0<<CS22)|(0<<CS21)|(1<<CS20); // /no prescaler;
  TCCR1B=(1<<WGM12)|(0<<CS22)|(1<<CS21)|(0<<CS20); // /8; 1us clock
  TCNT1H=0x00; TCNT1L=0x00;
  ICR1H=0x00; ICR1L=0x00;
  OCR1AH=0xFF; OCR1AL=0xFF;  //OCR1AH=0x9C;  //OCR1AL=0x40; // 40000//  OCR1AH=0x4E;//  OCR1AL=0x20; // 20000//  OCR1AH=0x03;//  OCR1AL=0xE8; // 1000

  //  TCCR1B |= (1 << WGM12);  //  TCCR1B |= (1 << CS10);
  // TIMSK1 |= (1 << OCIE1A);// no interrupts just counting 1 tick is 1 microsecond

// setup timer2 

  TCNT2=0;
  TIMSK2=0; 
  TCCR2B=(0<<WGM22)|(0<<CS22)|(0<<CS21)|(1<<CS20); // 1 xxxxx000 32 microseconds
//  TCCR2B=(0<<WGM22)|(0<<CS22)|(1<<CS21)|(0<<CS20); // 8
  TCCR2A=(1<<WGM21);//ctc mode |(1<<CS22)|(0<<CS21)|(0<<CS20); // 256?
  //TIMSK2 = (1 << OCIE2A); // Enable CTC interrupt 
  OCR2A = 0xFF; // Set CTC compare value 
  //18 min to humtmp work?

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
 NextSoilMoistureCheck=NextTmpHumCheck=uptime;

    Pin2Output(DDRB,6);
    Pin2Output(DDRB,7);
}

byte FanTimeout=0; // время отдыха вентилятора после выключения
byte RunningFan=0; // время действующего вентилятора
word LastTimeFan=0; //  время последнего включения вентилятора

//ISR (PCINT1_vect){ if (PINC&(1<<3)==0){ HR++; HR&=0xF; FlashIntensity=Intensity[HR]; }}  // A3 user button handler

ISR (PCINT1_vect){ if(uptime!=bts){//A3 is low here

//if (uptime<uptimeS){uptime=0; eeprom_update_byte((byte*)1,0);eeprom_update_byte((byte*)2,0); reboot(); }
//else{
HR++; uptime+=(60*60/2); if(HR==24){HR=0;uptime=0;};  FlashIntensity=Intensity[HR]; LastTimeFan=uptime;
bts=uptime;
}}  // A3 user button handler

/*
ISR(TIMER1_OVF_vect)
 {
 //Toggle pin PD0 every second
 //    PIND=(1<<PD0);
 t1ovf++;
 }
 */
byte volatile t2ovf=0;
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


word Flashes=0; // число вспышек в секунду

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

void fastnap(void)
{
      word prev;

Pin2Output(DDRD,2);Pin2HIGH(PORTD,2);// charge cap
//delayMicroseconds(2);

      WDhappen=0;
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
            if(pin2_interrupt_flag||WDhappen){break;}else{sleeps++;} // WD is important in case  RC sleeper went off
          }while(1);
          TCNT1+=prev;
//  cli();t1111=TCNT1;sei();//atomic read
}
void longnap(void)
{
      word prev;
      
      Pin2Output(DDRD,3);Pin2HIGH(PORTD,3);// charge cap

      WDhappen=0;
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
            if(pin3_interrupt_flag||WDhappen){break;}else{sleeps++;} // WD is important in case  RC sleeper went off
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
  byte r,g,b;

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
  for(byte i=0;i<16;i++)
  {
//    gg=(Intensity[i]*14)/mi;if(!gg){gg=1;} 
    byte gg=Intensity[i];//if(!gg){gg=1;} 

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


#define LONG_TOUCH_THRESHOLD 4 // критерий длинного нажатия в  секундах

  volatile long lvv,lmv,lm2;// luminous
  volatile long mlvv,mlmv,mlm2;// luminous
  volatile word lasttouch,ltp,CurrentTouch;
  byte LongTouch=0;
word rtcl;
word FT[5];
/*
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
    ta(" if=");th((pin0_interrupt_flag<<5)|(pin2_interrupt_flag<2)|(pin3_interrupt_flag<<1)|(WDhappen));
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


//void GetVcc(void){ VccN=Vcc(); if (VccN<VccH){VccH=VccN;} if (VccN>VccL){VccL=VccN;} } //280us

void LcdBack(void)
{
    LcdInit();
    LcdClear();

    LcdSetPos(7,0);ta(":");    LcdSetPos(18,0);ta(":");
    LcdSetPos(12*5,0);ta("АуРа");
    LcdSetPos(0,1);ta("Ч"); LcdSetPos(14,1);ta("Инт"); LcdSetPos(41,1);ta("t"); LcdSetPos(60,1);ta("Вл");
    LcdSetPos(0,2);ta("U");   
    LcdSetPos(0,3);ta("Пыхи ");     
    LcdSetPos(0,4);ta("Влажность");
    LcdSetPos(0,5);ta("Температура");
}

//void SaveUptime(void){eeprom_update_byte((byte*)1,(uptime>>8));eeprom_update_byte((byte*)2,(uptime&0xFF));}



//word co1=0;

void FanON(byte d){Pin2Output(DDRB,0);Pin2HIGH(PORTB,0);RunningFan=d;}
void FanOFF(byte t){Pin2LOW(PORTB,0);Pin2Input(DDRB,0);FanTimeout=t;}



void Flash(void)
{
    cli();
    PORTB|=(1<<6);  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;   PORTB&=~(1<<6); 
    PORTB|=(1<<7);  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;   PORTB&=~(1<<7);   
    sei();
}

//void*() df=Delay1;
//void eeprom_update_byte(byte*addr,byte v){ if(eeprom_read_byte((byte*)addr)!=v){eeprom_write_byte((byte*)addr,v);}}

// the loop routine runs over and over again forever:
void loop() {
  byte ra;
byte nn;
//  long now;
//  long oldnow=millis();
  //word t,t1,n;
//  word Temp;

while(1){

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

//NOP;
//  if ((timer0_overflow_count&0x1FF)==0){ uptime++;UpdateS=true; } 

//check with interrupts disabled
  if ((timer0_overflow_count&0x1FF)==0) // do something 2ms
//  if (UpdateS) // every 1/2s
  {
        TCNT1=0;
  //    UpdateS=false;
      __asm__ __volatile__("wdr\n\t");//  wdt_reset();

//      if ((uptime>=43200)||((PINC&(1<<3))==0)){eeprom_update_byte((byte*)1,0);eeprom_update_byte((byte*)2,0);reboot();} // reboot every 24h or when reset button (A3) is pressed
//      if (uptime>=43200){uptime=0;SaveUptime();reboot();} // reboot every 24h
//      if (uptime>=43200){reboot();} // reboot every 24h
long hh=timer0_overflow_count>>9;
//     if((uptime&7)==0){
//       LcdSetPos(0,0);tn(10000,uptime);

//       LcdSetPos(0,0);tn(1000000,timer0_overflow_count);
      HR=hh/3600; sei(); if (HR==24){reboot();} // reboot every 24h
//      HR=timer0_overflow_count/(60*512); sei(); if (HR==24){reboot();} // reboot every 24h

//long ll=(timer0_overflow_count-HR*7200000);
  //    MN=ll/120000;
    //  SC=
//      MN=(timer0_overflow_count%7200000)/120000;
//      SC=timer0_overflow_count-HR*7200000-MN*60
      
      FlashIntensity=Intensity[HR];
//HR=uptime/(60*60/2);      FlashIntensity=Intensity[HR];
//byte mn=(uptime-HR*60*60/2)/30;
//byte sc=(uptime-HR*60*60/2-mn*30)*2;
       LcdSetPos(0,0);tn(10,HR); tn(10,FlashIntensity);     // LcdSetPos(11,0);tn(10,MN);LcdSetPos(22,0);tn(10,SC);
       
       tn(10,nn);

  //long mm=millis(); wctr=1000000L;while(--wctr>0);  long ww=millis();tn(10000,ww-mm);
  
 //}// every 16s
//  LcdSetPos(0,0);tn(10000,uptime);// 963us

  if(uptime>=NextTmpHumCheck)
  {
        NextTmpHumCheck=uptime+16; // +32s
 // LcdSetPos(18,2);ta("N");tn(100,co1);co1=0;
      
  //    LcdSetPos(6,1);tn(10,HR);
    //  LcdSetPos(32,1);tn(10,FlashIntensity);
      LcdSetPos(47,1);tn(100,MCUtemp);
     // LcdSetPos(6,2);tn(100,ADCresult);


    

//          LcdBack();
   if (DHTreadAll()) 
  {
      LcdSetPos(70,4);ta("%");tn(10,(DHThum+5)/10);
      LcdSetPos(70,5); char* cc; if (DHTdata[2] & 0x80){cc="-";}else{cc="+";}
      ta(cc);
      LcdSetPos(76,5);
      DHTtmp=(DHTtmp+5)/10;
      tn(10,DHTtmp);
      // иногда нужен ветерок CО2 свежего подкачать (c 6 утра)
      if ((!FanTimeout)&&(!RunningFan)){if ((DHTtmp>=28)||(uptime-LastTimeFan)>1800){if(DHTtmp>=31){delay(60000);}else if(HR>=6){FanON(32);}}}
   }
     if (uptime>=NextSoilMoistureCheck){ NextSoilMoistureCheck=uptime+1000; SoilMoisture(); LcdSetPos(72,1); tn(100,moisture); }// hide this here
  }// next humtmp check


//if(uptime&1){
  //Flashes/=2;
 // LcdSetPos(60,3);tn(10000,Flashes);
 
   //word ll=1000000L/Flashes;
//  LcdSetPos(5*6,3);tn(10000,ll);   
//}
//Flashes=0;


      
      if(FanTimeout){FanTimeout--;}else if(RunningFan){if((--RunningFan)==0){FanOFF(32);LastTimeFan=uptime;}}

delay(2);
       //tn(10000,TCNT1);

//       LcdSetPos(37,0);tn(100000,timer0_overflow_count);
       LcdSetPos(36,0);tn(100000,Flashes);Flashes=0;

      
  }// UpdateS
  
//  if ((timer0_overflow_count&0xFF)==0){ uptime++;UpdateS=true; }


//FlashIntensity=4;//debug

// 0,1,2,3,4
//4 8us windows

//4: x x x x
//3: x x x o
//2: x o x o
//1: x o o o
//0: o o o o
//NOP;NOP;
//word tt=TCNT1;

//NOP;
//cli();
//  ra=0;
  //uptime=TCNT2;
//nn=0;do{nn++;}
//  ra=TCNT0;  while(TCNT0==ra);  // catch initial sync
//cli();
//  if(FlashIntensity){
//NOP;

      __asm__ __volatile__(
      "in r20,0x26\n\t" //TCNT0
      "3:\n\t"
      "in r21,0x26\n\t" //TCNT0
      "cp r21,r20\n\t"
      "breq 3b\n\t"
      
      "lds r26,FlashIntensity\n\t"
      "mov r25,r1\n\t" // clear r25
      "or r26,r26\n\t"
      "breq 2f\n\t" // если FlashIntensity==0 то r25=0 иначе 01000000
      "ldi r25,0b01000000\n\t" // r25: 01000000
      "2:\n\t"

      "in r24,3\n\t" // PINB (bits 6&7 are 0)
      "or r24,r25\n\t" // r24: bit6 ON
      
      "cli\n\t"
      
      "out 5,r24\n\t" // set pin 6 ON
      
      "ldi r23,6\n\t"
      "1:\n\t"
      "dec r23\n\t" // 1 clk
      "brne 1b\n\t"// 2 clk if true (1clk if false) so 5 gives us: ldi(1) 5*dec(1)+4*jump(2)+last(not jump)(1)===15clk   10: 1+10+18+1=30  9: 1+9+16+1=27  8:1+8+14+1=24  7:21
      "add r24,r25\n\t"      // выключаем 6 включаем 7

      "out 5,r24\n\t" // set pin 6 OFF pin7 ON
      "cbr r24, 0b11000000\n\t" // bits 6&7 are OFF

      "ldi r23,6\n\t"
      "1:\n\t"
      "dec r23\n\t" // 1 clk
      "brne 1b\n\t"// 2 clk if true (1clk if false) so 5 gives us: ldi(1) 5*dec(1)+4*jump(2)+last(not jump)(1)===15clk   10: 1+10+18+1=30  9: 1+9+16+1=27  8:1+8+14+1=24

      "out 5,r24\n\t" // set pin 6 OFF pin 7 OFF
      "sei\n\t"    
      );
//  nn=PINB;


//  PORTB|=(1<<6);  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP; PORTB&=~(1<<6); 
//  PORTB|=(1<<7);  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP; PORTB&=~(1<<7);   
//  }
  //ra=0;
  // just use nops!

//  ra=TCNT0;  while(TCNT0==ra);  // catch sync
/*  if(FlashIntensity>=3){
 // while(TCNT2&0b00011111);  // catch sync
//    Flash();
  PORTB|=(1<<6);  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP; PORTB&=~(1<<6); 
  PORTB|=(1<<7);  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP; PORTB&=~(1<<7);   
  }
*/
      __asm__ __volatile__(
          "3:\n\t"
      "in r20,0x26\n\t" //TCNT0
      "cp r21,r20\n\t"
      "breq 3b\n\t"
      
  
//      "lds r26,FlashIntensity\n\t"
      "mov r25,r1\n\t" // clear r25
      "cpi r26,3\n\t"
//      "or r26,r26\n\t"
      "brlo 2f\n\t"// Branch if r23 < 3 (unsigned)
//      "breq 2f\n\t" // если FlashIntensity==0 то r25=0 иначе 01000000
      "ldi r25,0b01000000\n\t" // r25: 01000000
      "2:\n\t"

      "in r24,3\n\t" // PINB (bits 6&7 are 0)
      "or r24,r25\n\t" // r24: bit6 ON
      
      "cli\n\t"
      
      "out 5,r24\n\t" // set pin 6 ON
      
      "ldi r23,6\n\t"
      "1:\n\t"
      "dec r23\n\t" // 1 clk
      "brne 1b\n\t"// 2 clk if true (1clk if false) so 5 gives us: ldi(1) 5*dec(1)+4*jump(2)+last(not jump)(1)===15clk   10: 1+10+18+1=30  9: 1+9+16+1=27  8:1+8+14+1=24  7:21
      "add r24,r25\n\t"      // выключаем 6 включаем 7

      "out 5,r24\n\t" // set pin 6 OFF pin7 ON
      "cbr r24, 0b11000000\n\t" // bits 6&7 are OFF

      "ldi r23,6\n\t"
      "1:\n\t"
      "dec r23\n\t" // 1 clk
      "brne 1b\n\t"// 2 clk if true (1clk if false) so 5 gives us: ldi(1) 5*dec(1)+4*jump(2)+last(not jump)(1)===15clk   10: 1+10+18+1=30  9: 1+9+16+1=27  8:1+8+14+1=24

      "out 5,r24\n\t" // set pin 6 OFF pin 7 OFF
      "sei\n\t"
      );


      __asm__ __volatile__(
          "3:\n\t"
      "in r21,0x26\n\t" //TCNT0
      "cp r21,r20\n\t"
      "breq 3b\n\t"
      
  
//      "lds r26,FlashIntensity\n\t"
      "mov r25,r1\n\t" // clear r25
      "cpi r26,2\n\t"
//      "or r26,r26\n\t"
      "brlo 2f\n\t"// Branch if r23 < 2 (unsigned)
//      "breq 2f\n\t" // если FlashIntensity==0 то r25=0 иначе 01000000
      "ldi r25,0b01000000\n\t" // r25: 01000000
      "2:\n\t"

      "in r24,3\n\t" // PINB (bits 6&7 are 0)
      "or r24,r25\n\t" // r24: bit6 ON
  "cli\n\t"
      "out 5,r24\n\t" // set pin 6 ON
      
      "ldi r23,6\n\t"
      "1:\n\t"
      "dec r23\n\t" // 1 clk
      "brne 1b\n\t"// 2 clk if true (1clk if false) so 5 gives us: ldi(1) 5*dec(1)+4*jump(2)+last(not jump)(1)===15clk   10: 1+10+18+1=30  9: 1+9+16+1=27  8:1+8+14+1=24  7:21
      "add r24,r25\n\t"      // выключаем 6 включаем 7

      "out 5,r24\n\t" // set pin 6 OFF pin7 ON
      "cbr r24, 0b11000000\n\t" // bits 6&7 are OFF

      "ldi r23,6\n\t"
      "1:\n\t"
      "dec r23\n\t" // 1 clk
      "brne 1b\n\t"// 2 clk if true (1clk if false) so 5 gives us: ldi(1) 5*dec(1)+4*jump(2)+last(not jump)(1)===15clk   10: 1+10+18+1=30  9: 1+9+16+1=27  8:1+8+14+1=24

      "out 5,r24\n\t" // set pin 6 OFF pin 7 OFF
      "sei\n\t"
      );

//  ra=0;if(FlashIntensity>=2){ra=1;}
//  ra=TCNT0;  while(TCNT0==ra);  // catch sync
 // while(TCNT2&0b00011111);  // catch sync
// if(FlashIntensity>=2){
//    Flash();
  //PORTB|=(1<<6);  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP; PORTB&=~(1<<6); 
//  PORTB|=(1<<7);  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP; PORTB&=~(1<<7);   
// }
//  ra=0;if(FlashIntensity>=4){ra=1;}
//  ra=TCNT0;  while(TCNT0==ra);  // catch sync
//  if(FlashIntensity>=4){
 // while(TCNT2&0b00011111);  // catch sync
//    Flash();
//  PORTB|=(1<<6);  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP; PORTB&=~(1<<6); 
  //PORTB|=(1<<7);  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;  NOP;NOP;NOP;NOP; PORTB&=~(1<<7);   
//  }
      __asm__ __volatile__(
          "3:\n\t"
      "in r20,0x26\n\t" //TCNT0
      "cp r21,r20\n\t"
      "breq 3b\n\t"
      
  
//      "lds r26,FlashIntensity\n\t"
      "mov r25,r1\n\t" // clear r25
      "cpi r26,4\n\t"
//      "or r26,r26\n\t"
      "brlo 2f\n\t"// Branch if r23 < 4 (unsigned)
//      "breq 2f\n\t" // если FlashIntensity==0 то r25=0 иначе 01000000
      "ldi r25,0b01000000\n\t" // r25: 01000000
      "2:\n\t"

      "in r24,3\n\t" // PINB (bits 6&7 are 0)
      "or r24,r25\n\t" // r24: bit6 ON
"cli\n\t"
      "out 5,r24\n\t" // set pin 6 ON
      
      "ldi r23,2\n\t"
      "1:\n\t"
      "dec r23\n\t" // 1 clk
      "brne 1b\n\t"// 2 clk if true (1clk if false) so 5 gives us: ldi(1) 5*dec(1)+4*jump(2)+last(not jump)(1)===15clk   10: 1+10+18+1=30  9: 1+9+16+1=27  8:1+8+14+1=24  7:21
      "add r24,r25\n\t"      // выключаем 6 включаем 7

      "out 5,r24\n\t" // set pin 6 OFF pin7 ON
      "cbr r24, 0b11000000\n\t" // bits 6&7 are OFF

      "ldi r23,2\n\t" // сокращение последней вспышки
      "1:\n\t"
      "dec r23\n\t" // 1 clk
      "brne 1b\n\t"// 2 clk if true (1clk if false) so 5 gives us: ldi(1) 5*dec(1)+4*jump(2)+last(not jump)(1)===15clk   10: 1+10+18+1=30  9: 1+9+16+1=27  8:1+8+14+1=24

      "out 5,r24\n\t" // set pin 6 OFF pin 7 OFF
      "sei\n\t"
      );

//  TCNT1=tt;

Flashes++;
//sei();
//if(FlashIntensity)
//{
  
//  PORTD&=~(1<<5);   PORTD&=~(1<<6); 
//  PORTD|=(1<<5);   PORTD|=(1<<6); PORTD|=(1<<7); 
//    PORTB|=(1<<6);    PORTB|=(1<<7); 
//delayMicroseconds(20);
//PORTB&=~(1<<6);  PORTB&=~(1<<7); 
  //Pin2Input(DDRB,6);  Pin2Input(DDRB,7);

//  PORTD&=~(1<<5);   PORTD&=~(1<<6);  PORTD&=~(1<<7); 

//        cli();
//        TCNT1=0; PORTB|=(1<<6);  while(TCNT1<3); PORTB&=~(1<<6); 
  //      TCNT1=0; PORTB|=(1<<7);  while(TCNT1<3); PORTB&=~(1<<7); 
    //    sei();


//   if(FlashIntensity==3)// 8x3 24us of light 53us 45%
  // {
      //for(byte i=0;i<100;i++)
  //    {
        

    
    //        TCNT1=0; PORTB|=(1<<6);  while(TCNT1<FlashIntensity); PORTB&=~(1<<6); 
  //      TCNT1=0; PORTB|=(1<<7);  while(TCNT1<FlashIntensity); PORTB&=~(1<<7); 
//  NOP;
//  portbmask=PINB&(~((1<<6)|(1<<7))); 
/*
        __asm__ __volatile__(        
        "in r21,3\n\t" // PINB
        "mov r20,r21\n\t"  // bits 6&7 cleared
        "ori r21,0b01000000\n\t" // bit6set
        "sts 0x0084,r1\n\t" // low(TCNT1)=0
        "out 5,r21\n\t" // 6 ON
        "mov r22,r20\n\t"
        "ori r22,0b10000000\n\t" // bit7set

        "1:\n\t""lds r24,0x0084\n\t""subi r24,3\n\t""brcs 1b\n\t"
        "out 5,r22\n\t" // 7 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,6\n\t""brcs 1b\n\t"
        "out 5,r21\n\t" // 6 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,9\n\t""brcs 1b\n\t"
        "out 5,r22\n\t" // 7 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,12\n\t""brcs 1b\n\t"
        "out 5,r21\n\t" // 6 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,15\n\t""brcs 1b\n\t"
        "out 5,r22\n\t" // 7 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,18\n\t""brcs 1b\n\t"
        "out 5,r21\n\t" // 6 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,21\n\t""brcs 1b\n\t"
        "out 5,r22\n\t" // 7 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,24\n\t""brcs 1b\n\t"

        "out 5,r21\n\t" // 6 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,27\n\t""brcs 1b\n\t"
        "out 5,r22\n\t" // 7 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,30\n\t""brcs 1b\n\t"
        "out 5,r21\n\t" // 6 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,33\n\t""brcs 1b\n\t"
        "out 5,r22\n\t" // 7 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,36\n\t""brcs 1b\n\t"
        "out 5,r21\n\t" // 6 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,39\n\t""brcs 1b\n\t"
        "out 5,r22\n\t" // 7 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,42\n\t""brcs 1b\n\t"
        "out 5,r21\n\t" // 6 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,45\n\t""brcs 1b\n\t"
        "out 5,r22\n\t" // 7 ON
        "1:\n\t""lds r24,0x0084\n\t""subi r24,48\n\t""brcs 1b\n\t"
        */
  //      "out 5,r20\n\t" // 6&7 OFF        
    //    ); //PINB
        
        
  /*      
        TCNT1=0; 
        __asm__ __volatile__(        
        "in r20,3\n\t" // PINB
        "mov r21,r20\n\t"  // bits 6&7 cleared
        "mov r22,r20\n\t"
        "ori r21,0b01000000\n\t" // bit6set
        "ori r22,0b10000000\n\t" // bit7set
        );


        __asm__ __volatile__("out 5,r21\n\t");  __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,3\n\t""brcs 1b\n\t"); __asm__ __volatile__("out 5,r22\n\t"); __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,6\n\t""brcs 1b\n\t"); //PORTB=portbmask; 
        __asm__ __volatile__("out 5,r21\n\t");  __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,9\n\t""brcs 1b\n\t"); __asm__ __volatile__("out 5,r22\n\t"); __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,12\n\t""brcs 1b\n\t");//PORTB=portbmask; 
        __asm__ __volatile__("out 5,r21\n\t");  __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,15\n\t""brcs 1b\n\t"); __asm__ __volatile__("out 5,r22\n\t"); __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,18\n\t""brcs 1b\n\t");//PORTB=portbmask; 
        __asm__ __volatile__("out 5,r21\n\t");  __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,21\n\t""brcs 1b\n\t"); __asm__ __volatile__("out 5,r22\n\t"); __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,24\n\t""brcs 1b\n\t");//PORTB=portbmask; 
        
        __asm__ __volatile__("out 5,r21\n\t");  __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,27\n\t""brcs 1b\n\t"); __asm__ __volatile__("out 5,r22\n\t"); __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,30\n\t""brcs 1b\n\t");//PORTB=portbmask; 
        __asm__ __volatile__("out 5,r21\n\t");  __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,33\n\t""brcs 1b\n\t"); __asm__ __volatile__("out 5,r22\n\t"); __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,36\n\t""brcs 1b\n\t"); //PORTB=portbmask; 
        __asm__ __volatile__("out 5,r21\n\t");  __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,39\n\t""brcs 1b\n\t"); __asm__ __volatile__("out 5,r22\n\t"); __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,42\n\t""brcs 1b\n\t"); //PORTB=portbmask; 
        __asm__ __volatile__("out 5,r21\n\t");  __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,45\n\t""brcs 1b\n\t"); __asm__ __volatile__("out 5,r22\n\t"); __asm__ __volatile__("1:\n\t""lds r24,0x0084\n\t""subi r24,48\n\t""brcs 1b\n\t"); __asm__ __volatile__("out 5,r20\n\t");  
*/
/*
        TCNT1=0; PORTB=portbmask|(1<<6);  while(TCNT1<3); PORTB=portbmask|(1<<7); while(TCNT1<6); //PORTB=portbmask; 
        PORTB=portbmask|(1<<6);  while(TCNT1<9); PORTB=portbmask|(1<<7); while(TCNT1<12); //PORTB=portbmask; 
        PORTB=portbmask|(1<<6);  while(TCNT1<15); PORTB=portbmask|(1<<7); while(TCNT1<18); //PORTB=portbmask; 
        PORTB=portbmask|(1<<6);  while(TCNT1<21); PORTB=portbmask|(1<<7); while(TCNT1<24); //PORTB=portbmask; 
        
        PORTB=portbmask|(1<<6);  while(TCNT1<27); PORTB=portbmask|(1<<7); while(TCNT1<30); //PORTB=portbmask; 
        PORTB=portbmask|(1<<6);  while(TCNT1<33); PORTB=portbmask|(1<<7); while(TCNT1<36); //PORTB=portbmask; 
        PORTB=portbmask|(1<<6);  while(TCNT1<39); PORTB=portbmask|(1<<7); while(TCNT1<42); //PORTB=portbmask; 
        PORTB=portbmask|(1<<6);  while(TCNT1<45); PORTB=portbmask|(1<<7); while(TCNT1<48); PORTB=portbmask; 
         cli();
        TCNT1=0; PORTB|=(1<<6);  while(TCNT1<3); PORTB&=~(1<<6); 
        TCNT1=0; PORTB|=(1<<7);  while(TCNT1<3); PORTB&=~(1<<7); 
        sei();
          TCNT1=0;while(TCNT1<6);
        cli();
        TCNT1=0; PORTB|=(1<<6);  while(TCNT1<3); PORTB&=~(1<<6); 
        TCNT1=0; PORTB|=(1<<7);  while(TCNT1<3); PORTB&=~(1<<7); 
        sei();
          TCNT1=0;while(TCNT1<6);
        cli();
        TCNT1=0; PORTB|=(1<<6);  while(TCNT1<3); PORTB&=~(1<<6); 
        TCNT1=0; PORTB|=(1<<7);  while(TCNT1<3); PORTB&=~(1<<7); 
        sei();

          TCNT1=0;while(TCNT1<3);
*/          
//          TCNT1=0;while(TCNT1<20); // 50us total
    //  }
  // }
//   else if(FlashIntensity==2)
  // {
//3-0 2-3 1-6  
//    byte till=(6-FlashIntensity*2); 
  //  for(byte i=0;i<100;i++)
    //{
//        portbmask=PINB&(~((1<<6)|(1<<7))); 
        //Pin2Output(DDRB,6);
      //  TCNT1=0; PORTB|=(1<<6);  while(TCNT1<FlashIntensity); PORTB&=~(1<<6); //Pin2Input(DDRB,6); 
        //Pin2Output(DDRB,7);
        //TCNT1=0; PORTB|=(1<<7);  while(TCNT1<FlashIntensity); PORTB&=~(1<<7); //Pin2Input(DDRB,7); 
  //      cli();
    //    TCNT1=0; PORTB=portbmask|(1<<6);  while(TCNT1<3); PORTB=portbmask|(1<<7); while(TCNT1<6); PORTB=portbmask; 
    /*    cli();
        TCNT1=0; PORTB|=(1<<6);  while(TCNT1<3); PORTB&=~(1<<6); 
        TCNT1=0; PORTB|=(1<<7);  while(TCNT1<3); PORTB&=~(1<<7); 
        sei();
          TCNT1=0;while(TCNT1<10);
        cli();
        TCNT1=0; PORTB|=(1<<6);  while(TCNT1<3); PORTB&=~(1<<6); 
        TCNT1=0; PORTB|=(1<<7);  while(TCNT1<3); PORTB&=~(1<<7); 
        sei();
        */
//        if((i&3)==3){TCNT1=0;while(TCNT1<19);} // после каждой 4й перерыв (9-9-9-30) ?50
//    }
//   }
  // else if(FlashIntensity==1)
   //{
//3-0 2-3 1-6  
//    byte till=(6-FlashIntensity*2); 
  //  for(byte i=0;i<100;i++)
    //{
//        cli();
  //      TCNT1=0; PORTB|=(1<<6);  while(TCNT1<3); PORTB&=~(1<<6); 
    //    TCNT1=0; PORTB|=(1<<7);  while(TCNT1<3); PORTB&=~(1<<7); 
      //  sei();
        //  TCNT1=0;while(TCNT1<20);
//        if((i&3)==3){TCNT1=0;while(TCNT1<19);} // после каждой 4й перерыв (9-9-9-30) ?50
//    }
//   }

//    Flashes++;  

//} // пыхнем

    } // eternal loop
}


