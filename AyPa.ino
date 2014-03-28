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

#define reboot {__asm__ __volatile__ ("rcall 0\n\t" );} //void(* resetFunc) (void) = 0; //declare reset function @ address 0
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
#define FLASH_CYCLE 65 // microseconds (+~5)

byte Intensity[16] = {20,5,2,2, 2,2,2,2, 5,10,15,20, 25,30,30,25}; // интенсивность яркости (90min)
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
//  cnt1=TCNT1;
  WDhappen=1;
  WDsleep=0;
  }
  //debug
  else{ reboot; }// This will call location zero and cause a reboot.
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

boolean DHTread(void) {
  byte i;
  word d;
  boolean res=false;
  
  
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

//Pin2Input(DDRB,2);//  pinMode(A0, OUTPUT);
Pin2Input(DDRB,1);
Pin2HIGH(PORTB,1);//  digitalWrite(A0, HIGH);

//  pinMode(A0, OUTPUT);  digitalWrite(A0, HIGH); // left A0 in HIGH state
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

LcdInit();LcdClear();
LcdSetPos(12*5,0);
ta("АуРа");


//  dht.begin();
//pinMode(A0, INPUT);
//digitalWrite(A0, HIGH);  
Pin2Input(DDRB,1);
//Pin2Output(DDRB,2);
Pin2HIGH(PORTB,1);//  digitalWrite(A0, HIGH);  // internal pull up



setup_watchdog(T2S); // если в течении 2s не сбросить сторожевого пса то перезагрузка. (защита от зависаний)

    Pin2Input(DDRC,3);Pin2HIGH(PORTC,3); // pull up on A0
    PCMSK1 = 1<<PCINT11; // setup pin change interrupt on A3 pin (SQuareWave from RTC)
    PCICR |= 1<<PCIE1; 

Pin2Output(DDRD,5);
Pin2Output(DDRD,6);
Pin2Output(DDRD,7);

    
/*
      LCD_ON();
  DrawBox(16,0,159,127,0x00,0x00,0x00);    // очистка экрана
      
  setAddrWindow(2,0,2+7,127);ta("АУРА");
    
    delay(5000);
    
LCD_OFF();


LCD=ticks+300;
*/
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



//byte pinmask,prt;
//word max0=0,max1=0,max01=0,max10=0;

word VccN;
word VccH=1023;
word VccL=0;


void FlashTest(void) // #2 pin used as test
{
  word max0,max1;
  word maxx0,maxx1;
  
//  DrawBox(16,0,159,127,0x00,0x00,0x00);    // очистка экрана
  
//  setAddrWindow(140,0,140+7,127);ta("Тестовый режим / TEST");
//  setAddrWindow(22,0,22+7,127);ta("---------------------");

/*
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
  setAddrWindow(145,0,145+7,127);ta("not go beyond 700mA.");  */
  
  delay(3000);

  
  
//  Pin2Output(DDRD,7);Pin2HIGH(PORTD,7);  // G stop light
  Pin2Output(DDRD,5);Pin2HIGH(PORTD,5);  // SRCLR to HIGH. when it is LOW all regs are cleared
  Pin2Output(DDRB,1);
  Pin2Output(DDRD,6); 

    Pin2HIGH(PORTB,1);  // DATAPIN to HIGH  
    Pin2HIGH(PORTD,6);Pin2LOW(PORTD,6); // clock pulse 
    Pin2LOW(PORTB,1); // next data are zeroes
    Pin2HIGH(PORTD,6);Pin2LOW(PORTD,6); // clock pulse 
    Pin2HIGH(PORTD,6);Pin2LOW(PORTD,6); // clock pulse 

//    TSL2561_ON;

  maxx0=0;maxx1=0;
for(byte n=20;n>0;n--)
{

  //LCD_OFF();
  max0=0;max1=0;
for(word z=0;z<25;z++)
{
                if(!ERR){TSLstart(TSL2561_INTEGRATIONTIME_101MS |TSL2561_GAIN_0X);} // 192us
//                if(!ERR){TSLstart(TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_0X);} // 192us

Pin2LOW(PORTD,7);// start lighting

for(byte q=0;q<11;q++)
{
//cli();
//TCNT1=0;
GetVcc();
delay(10);
//do{}while(TCNT1<10000);
//sei();
}

Pin2HIGH(PORTD,7);//digitalWrite(G,HIGH); // stop light

  //              if(!ERR){delay(103);TSLstop();}//322us
                if(!ERR){TSLstop();}//322us
  
          if(Light.W[0]>max0){max0=Light.W[0];max1=Light.W[1];}
          if(Light.W[0]>maxx0){maxx0=Light.W[0];maxx1=Light.W[1];}

    }// for 100

     // LCD_ON();

  /*setAddrWindow(152,120,152+7,127);tn(10,n);
      
  setAddrWindow(2,0,2+7,127);
      ta("CH0 ");tn(10000,max0);ta(" CH1 ");tn(10000,max1);
//      ta(" ");tn(100,Light.W[0]);ta(" ");tn(100,Light.W[1]);
  setAddrWindow(12,0,12+7,127);
      ta("CH0 ");tn(10000,maxx0);ta(" CH1 ");tn(10000,maxx1);

  setAddrWindow(32,0,32+7,127);
    ta("Vcc");tn(100,VccN);ta(" L");tn(100,VccL);ta(" H");tn(100,VccH);ta(" ERR ");th(ERR);
  setAddrWindow(42,0,42+7,127);    
    ta("Vcc");tn(100,114000L/VccN);ta(" L");tn(100,114000L/VccL);ta(" H");tn(100,114000L/VccH); // 1125300L
    */
    delay(5000);

} //for n 

  Pin2LOW(PORTD,5);Pin2Input(DDRD,5);  // SRCLR to LOW. Clear regs
  Pin2Input(DDRB,1);  // DATAPIN
  Pin2Input(DDRD,6); // CLK&LATCH 
//  Pin2LOW(PORTD,7);Pin2Input(DDRD,7);  // detach G control pin (it is pull upped by resistor) --- no so really

  VccH=1023;VccL=0;
}

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

/*
PORTD=0x70; delayMicroseconds(10); PORTD=0x30; //PORTD=0xB0;//NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(10); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(10); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(10); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(10); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(10); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(10); PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; delayMicroseconds(10); PORTD=0x30;// PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0xB0;// restore pin7 high pin6 low
*/
/*
PORTD=0x70; PORTD=0x30; //PORTD=0xB0;//NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; PORTD=0x30; //PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; PORTD=0x30;// PORTD=0xB0;// NOP;//PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0xB0;// restore pin7 high pin6 low
*/
/*
PORTD=0x70; NOP; PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//pin0
PORTD=0x70; NOP; PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//1
PORTD=0x70; NOP; PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6);//2
PORTD=0x70; NOP; PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6); //3
PORTD=0x70; NOP; PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6); //4
PORTD=0x70; NOP; PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6); //pin5
PORTD=0x70; NOP; PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6); //pin6
PORTD=0x70; NOP; PORTD=0xB0;//PORTD=0xF0;PORTD=0xB0;  //Pin2HIGH(PORTD,6);  Pin2LOW(PORTD,6); ////Pin2LOW(PORTD,7); NOP;  Pin2HIGH(PORTD,7); // pin7
*/

//PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
//PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
//PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
//PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
//PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
//PORTD=0x30; NOP; PORTD=0x30; PORTD=0xB0;

/*
PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
PORTD=0x30; NOP; PORTD=0x70; PORTD=0xB0;
PORTD=0x30; NOP; PORTD=0x30; PORTD=0xB0;
*/

//PORTD=0xB0; //4,5,7


// Pin2LOW(PORTD,7);  PORTB=pb;  Pin2HIGH(PORTD,7); 
// Pin2LOW(PORTD,7);  PORTB=pb;  Pin2HIGH(PORTD,7); 
// Pin2LOW(PORTD,7);  PORTB=pb;  Pin2HIGH(PORTD,7); 


sei();
/*
cli();
if(Duration==5){ __asm__ __volatile__("cbi 0x0B,7\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""sbi 0x0B,7\n\t"); }
else if(Duration==4){ __asm__ __volatile__("cbi 0x0B,7\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""sbi 0x0B,7\n\t"); }
else if(Duration==3){ __asm__ __volatile__("cbi 0x0B,7\n\t""nop\n\t""nop\n\t""nop\n\t""sbi 0x0B,7\n\t"); }
else if(Duration==2){ __asm__ __volatile__("cbi 0x0B,7\n\t""nop\n\t""nop\n\t""sbi 0x0B,7\n\t"); }
else if(Duration==1){ __asm__ __volatile__("cbi 0x0B,7\n\t""nop\n\t""sbi 0x0B,7\n\t"); }
sei();
*/


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
}
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
word uptime=0; // uptime в секундах
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
  if (DHTread()) {
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

void GetVcc(void){ VccN=Vcc(); if (VccN<VccH){VccH=VccN;} if (VccN>VccL){VccL=VccN;} } //280us

// the loop routine runs over and over again forever:
void loop() {
  long now;
//  long oldnow=millis();
  word t,t1,n;
  word Temp;

  
  do{
        __asm__ __volatile__("wdr\n\t");//  wdt_reset();


//Pin2Input(DDRD,0);Pin2HIGH(PORTD,0); // pull up on D0
//PCICR |= 1<<PCIE2;
//PCMSK2 = 1<<PCINT16; // D0



//  if (Update500)   {      Update500=false;      }// every 500ms

  if (UpdateRTC) // every second
  {
      UpdateRTC=false;
//      GetVcc();  
      if(++uptime==12*60*60){reboot;} // перезагрузка каждые 12 часов
      RTC(); // 500us
      LcdSetPos(5*5,0);tn(100000,uptime);
      
      if ((uptime&0xFF)==3){ // раз в 256 секунд

      LcdSetPos(0,1);ta("HR ");tn(10,HR);ta(" Int ");tn(10,FlashIntensity);

  
  if (DHTreadAll()) 
  {
      LcdSetPos(0,4);
      ta("Влажность  %");
      LcdSetPos(76,4);
      tn(10,(DHThum+5)/10);
      LcdSetPos(0,5);
      ta("Температура");
      if (DHTdata[2] & 0x80){ta("-");}else{ta("+");}
      LcdSetPos(76,5);
      tn(10,(DHTtmp+5)/10);
   }
   
   
   
 }// UpdateRTC   
      
  //    CurrentTouch=TouchSensor();  //337-440us
    //  TouchD[(uptime&3)]=CurrentTouch;Etouch=TouchT();
      /*
    if(!LCD)
    {
        if (CurrentTouch>=Etouch)
        {
            SleepTime(); // measure SleepTime
            LCD=ticks+20;
            LCD_ON();
            // draw backgrounds
            ShowBars();
        }//else{TouchD[(uptime&3)]=CurrentTouch;Etouch=TouchT();} // add sample
    }*/
//    if(LCD){            FlasheS+=Flashes;Flashes=0;            UpdateScreen();      if(LCD<=ticks){LCD_OFF();}
//}

  }

//if(ERR==0x10){ERR=0;}// ignore TSL mising
// если есть ошибки
if (ERR)
{
  //  RTC_ON();
    //LCD_ON();
//    fillScreen(0x000000);
ta("ERR:");tn(100,ERR);
/*    DrawBox(0,0,159,127,0x00,0x00,0xfc);    // очистка экрана

    word cycles=TouchSensor();
    setAddrWindow(0,0,7,119);
    ta("ERR");wh(ERR);ta(" fn");tn(10000,fastnaptime);ta(" ln");tn(10000,longnaptime);
    setAddrWindow(10,0,17,119);
    ta("cycles:");tn(10000,cycles);
*/
    delay(5500);
    //SPCR&=~(1<<SPE); //  SPI.end(); // turn off SPI ????
   // LCD_OFF(); 
    reboot;  // This will call location zero and cause a reboot.
}

//FlashIntensity=10; // debug


if(FlashIntensity)
{

PORTD|=(1<<5); delayMicroseconds(FlashIntensity); PORTD&=~(1<<5); // pd5 start stop 
PORTD|=(1<<6); delayMicroseconds(FlashIntensity); PORTD&=~(1<<6); // pd6 start stop
//PORTD|=(1<<7); delayMicroseconds(FlashIntensity); PORTD&=~(1<<7); // pd7 start stop

word dd=FLASH_CYCLE-FlashIntensity*2;
if(dd){delayMicroseconds(dd);}
  /*
long base=micros();
  //long m1=millis();
  FT[0]=0;



//Pin2Output(DDRD,1);
Pin2Output(DDRD,5);
Pin2Output(DDRD,6);
Pin2Output(DDRD,7);


  
                                  if(!ERR){TSLstart(TSL2561_INTEGRATIONTIME_101MS |TSL2561_GAIN_0X);} // 192us
//                                  if(!ERR){TSLstart(TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_0X);} // 192us
while((micros()-base)<110000L)
//while((micros()-base)<14000L)
{

PORTD|=(1<<5); delayMicroseconds(FlashIntensity); PORTD&=~(1<<5); // pd5 start stop
PORTD|=(1<<6); delayMicroseconds(FlashIntensity); PORTD&=~(1<<6);  // pd6 start 
PORTD|=(1<<7); delayMicroseconds(FlashIntensity); PORTD&=~(1<<7);  // pd7 start 

  //  Flash(1,FlashIntensity); //84us    
//    Flash(1,FlashIntensity); //84us    
//    Flash(1,FlashIntensity); //84us    
//    delayMicroseconds(120);   
//    Flash(1,FlashIntensity); //84us    
//    delayMicroseconds(1120);   
    FT[0]++;
}
//long m2=millis();
                  if(!ERR){TSLstop();}//322us
                  
//  Pin2LOW(PORTD,5);Pin2Input(DDRD,5);  // SRCLR to LOW. Clear regs
//  Pin2Input(DDRB,1);  // DATAPIN
//  Pin2Input(DDRD,6); // CLK&LATCH 
//  Pin2LOW(PORTD,7);Pin2Input(DDRD,7);  // detach G control pin (it is pull upped by resistor) --- no so really

  //                FT[3]=ERR;
  //                FT[4]=m2-m1;
  
                if(Light.W[0]>LightMax0.W[0]){LightMax0.W[0]=Light.W[0];LightMax0.W[1]=Light.W[1];}
                if(Light.W[1]>LightMax1.W[1]){LightMax1.W[0]=Light.W[0];LightMax1.W[1]=Light.W[1];}


//    Flashes++;  
*/    
} // пыхнем
//else { set_sleep_mode(SLEEP_MODE_PWR_DOWN); unap(T1S); } // а иначе выходной :)


  
//  NOP;

//  wdt_disable();// some serious stuff (its role in hangups prevention?) why disable?
  


//NOP;
//it++;
//oldnow=now;
  //delay(1000);               // wait for a second
    }while(1); // loop
}


