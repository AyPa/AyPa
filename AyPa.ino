//#define __AVR_ATmega328P__ 
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>


#include <SPI.h> // < declaration ShiftOut etc problem



#include "AyPa_m.h"
#include "AyPa_fonts.h"
//#include "AyPa_n.h"
#include "AyPa_TFT.h"
#include "AyPa_i2c.h"
//#include "AyPa_rtc.h"

//#include <Wire.h>
//#include "TSL2561.h"
//TSL2561 tsl(TSL2561_ADDR_LOW); 


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

//DS1302_RAM ramBuffer;
//DS1302 rtc(A0,A1,A2);//ce data clk
//DS1307_RAM ramBuffer;
//DS1307 rtc(a1,a2);



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
word VccN[8][8];
word Vcc1;



volatile byte WDhappen;
volatile word  t1111; // vars updated in ISR should be declared as volatile and accessed with cli()/sei() ie atomic
volatile boolean WDsleep=false;

ISR(WDT_vect) // Watchdog timer interrupt.
{ 
  //resetFunc();  //reboot
  //  r2=TCNT1;
  if(WDsleep)
  {
  t1111=TCNT1;
  WDhappen=1;
  WDsleep=0;
  }
  //debug
//  else{    resetFunc();     }// This will call location zero and cause a reboot.
}

uint32_t ticks=0;
uint8_t seconds=0;
uint8_t minutes=0;
uint8_t hours=0;
uint8_t days=0;

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

ISR (TIMER1_COMPA_vect){
  ticks++;
  T1happen=1;
}


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

#define ERR_WHERE_IS_THE_CLOCK   0b00000001;
#define ERR_STRANGE_CLOCK_DATA  0b00000010;
#define ERR_WHERE_IS_THE_TSL2561 0b00000100;
#define ERR_BROKEN_SLEEP              0b00001000;
byte ERR=0; // ошибки

byte TFT_IS_ON=0;

void TFT_ON(byte duration){TFT_IS_ON=duration;
Pin2Output(DDRD,6);Pin2HIGH(PORTD,6); 
Pin2Output(DDRB,7); Pin2HIGH(PORTB,7); 
    Pin2Output(DDRD,4);Pin2LOW(PORTD,4); 
    Pin2Output(DDRB,2);Pin2HIGH(PORTB,2); // SS(SPI)
    Pin2Output(DDRB,3);Pin2LOW(PORTB,3); 
    Pin2Input(DDRB,4);Pin2LOW(PORTB,4); 
    Pin2Output(DDRB,5);Pin2LOW(PORTB,5);
delay(5); // time for power pin to stabilize  
} // включаем питание  дисплея
    
void TFT_OFF(void){ TFT_IS_ON=0; writecommand(ST7735_SLPIN); 
    Pin2LOW(PORTD,4); Pin2Input(DDRD,4); 
    Pin2LOW(PORTB,2); Pin2Input(DDRB,2); 
    Pin2LOW(PORTB,3); Pin2Input(DDRB,3); 
    Pin2LOW(PORTB,4); Pin2Input(DDRB,4); 
    Pin2LOW(PORTD,6);Pin2Input(DDRD,6);
    Pin2LOW(PORTB,7); Pin2Input(DDRB,7); 
    Pin2LOW(PORTB,5); Pin2Input(DDRB,5);
}// вЫключаем питание  дисплея

#define I2C_ON(DRC,p1,p2) {Pin2Output(DRC,p1);Pin2Output(DRC,p2);}
#define I2C_OFF(DRC,PORT,p1,p2) {Pin2Input(DRC,p1);Pin2LOW(PORT,p1);Pin2Input(DRC,p2);Pin2LOW(PORT,p2);}

byte rtc8;

// каждый раз перед обращением к часам проверяем их вменяемость
byte Check_RTC(byte attempts){for(byte n=attempts;n>0;n--){Save_I2C(DS1307_ADDR_W,8,'A',A1,A2);
rtc8=Read_I2C(DS1307_ADDR_W,8,A1,A2);
if(Read_I2C(DS1307_ADDR_W,8,A1,A2)=='A'){ERR&=~ERR_WHERE_IS_THE_CLOCK;return n;}}return 0;}
byte Check_TSL(byte attempts){for(byte n=attempts;n>0;n--){if(Read_I2C(TSL2561_ADDR_LOW_W,TSL2561_REGISTER_ID ,A4,A5)==0x0A){return n;}}return 0;}

word TouchSensor(void)
{
      word cycles=30000;
      Pin2Output(DDRC,3);Pin2LOW(PORTC,3); // discharge sensor  pin
      delay(1);//???
      Pin2Input(DDRC,3);
      for(int i=0;i<cycles;i++){if (PINC&0b00001000){cycles=i;break;}}
      Pin2Output(DDRC,3);Pin2LOW(PORTC,3); // discharge sensor  pin
      Pin2Input(DDRC,3);Pin2LOW(PORTC,3); // sensor pin
      return cycles;
}


word TouchD[4];
byte TouchPos=0;
word Etouch;
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
  char tstr[7];// строка даты и времени


byte CS; // текущая секунда
byte PS=0xFF; // предыдущая секунда
byte PH=0xFF; // предыдущий час
byte HR; // текущий час (0..23)
// интенсивность/продолжительность пыхи в микросекундах 0..255
//byte mi=25; 
byte mi=0;// максимальная интенсивность или 16 если меньше 
byte Intensity[24] = {0,0,0,0,0,0, 3,5,7,9,12,14, 15,16,16,16,15,14, 12,9,7,5,3,0};
byte FlashDuration=0;



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


  sei();




for(byte i=0;i<24;i++){if(mi<Intensity[i]){mi=Intensity[i];}} if(mi<16){mi=16;}// множитель для столбиков
for(byte i=0;i<4;i++){TouchD[i]=TouchSensor();} // calibrate touch sensor

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

  // check setup function
  PORTC=0x3;//  set pins 0123 HIGH (internal pullups) //digitalWrite(A1,HIGH);digitalWrite(A2,HIGH);digitalWrite(A3,HIGH);digitalWrite(A4,HIGH);
  //  pinMode(A1,INPUT_PULLUP); pinMode(A2,INPUT_PULLUP);  pinMode(A3,INPUT_PULLUP);  pinMode(A4,INPUT_PULLUP);   same same
  delay(1);
  v=(~PINC)&0x03;

  if (v>0){// 0 all are OFF // 1 1st is ON // 4 3rd is ON // C 3&4 are ON // F all are ON  
    //    setup function v (1..0xF) is called
    //idx=0; // lamp index
    //pmask=pp[0];//read port mask



  /*  Pin2Output(DDRB,pb[0]);
    if(v==1)
    {

    }// if 1
   */
    

    __asm__ __volatile__ ("nop\n\t");

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

  PORTC=0;//digitalWrite(A1,LOW);digitalWrite(A2,LOW);digitalWrite(A3,LOW);digitalWrite(A4,LOW);
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


Pin2Output(DDRD,0);// sreset mosfet 2n7000 control
Pin2Output(DDRD,1);// 3.3/5v mosfet 2n7000 control
Pin2Output(DDRD,2);// INT0 line

Pin2Output(DDRD,3);// INT1 line /TFT CS
Pin2Output(DDRD,4);// TFT DC




Pin2Output(DDRB,0); // CLOCKPIN 8
Pin2Output(DDRB,1); // DATAPIN 9
Pin2Output(DDRB,2);
Pin2Output(DDRB,6);


//Pin2Output(DDRC,2);
//Pin2Output(DDRC,3);
//Pin2Output(DDRC,4);
//Pin2Output(DDRC,5);

Pin2Output(DDRD,5); // pin 5 G
//Pin2Output(DDRD,6); // pin 6 LATCH
Pin2Output(DDRD,7); // pin 7 SRCLR



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







  // Setup the WDT  (16ms or reboot)
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
  sei();

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
//Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
        //    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|1;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   // NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //    delayMicroseconds(1);  
        // Pin2LOW(PORTB,1); //digitalWrite(9,LOW//   

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
        //  Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
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
//Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|1;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   // NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //    delayMicroseconds(1);  
        // Pin2LOW(PORTB,1); //digitalWrite(9,LOW//   

do{}while(bit_is_set(ADCSRA,ADSC));

        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   
        //    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        //    delayMicroseconds(10);  
        //  Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
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
//Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|1;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   // NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //    delayMicroseconds(1);  
        // Pin2LOW(PORTB,1); //digitalWrite(9,LOW//   

do{}while(bit_is_set(ADCSRA,ADSC));

        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   
        //    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        //    delayMicroseconds(10);  
        //  Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
        //   ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        // do{}while(bit_is_set(ADCSRA,ADSC));
        // v3=ADCW; 

        NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        //    delayMicroseconds(1);  
PORTD=0;
}


byte pinmask,prt;


long FlashM(byte lamps,byte Duration, byte cmd)// измерение интенсивности пыхи
{
    long  res;
    
    I2C_ON(DDRC,1,2);
    if(!Check_TSL(16)){ERR=ERR_WHERE_IS_THE_TSL2561;return 0;} 

    TSL2561_START(cmd); //
    delay(2);

    Pin2HIGH(PORTD,7);//digitalWrite(SRCLR,HIGH); // can write to 595 (it is cleared now)
    Pin2HIGH(PORTB,1);//DATAPIN to HIGH

    for(byte z=0;z<lamps;z++)// serie of flashes
    {
        Pin2HIGH(PORTB,0);Pin2LOW(PORTB,0); // clock pulse 
        Pin2LOW(PORTB,1); // next are zeroes
//        Pin2HIGH(PORTD,6);//digitalWrite(LATCHPIN,HIGH);
//        Pin2LOW(PORTD,6);//digitalWrite(LATCHPIN,LOW);
  
         cli();
         TCNT1=0;
         TCNT2=0;
         Pin2LOW(PORTD,5);// start lighting
         do{}while(TCNT2<Duration); 
//         do{}while(TCNT1<Duration); 
         Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // stop light
         sei();
     }// for
     
     delay(12);
      res=TSL2561_STOP();
      I2C_OFF(DDRC,PORTC,1,2);
      return res;
}

void FlashZ(byte lamps,byte Duration)
{
    Pin2HIGH(PORTD,7);//digitalWrite(SRCLR,HIGH); // can write to 595 (it is cleared now)
    Pin2HIGH(PORTB,1);//DATAPIN to HIGH

    for(byte z=0;z<lamps;z++)// serie of flashes
    {
        Pin2HIGH(PORTB,0);Pin2LOW(PORTB,0); // clock pulse 
        Pin2LOW(PORTB,1); // next are zeroes
//        Pin2HIGH(PORTD,6);//digitalWrite(LATCHPIN,HIGH);
  //      Pin2LOW(PORTD,6);//digitalWrite(LATCHPIN,LOW);
  
         cli();
         TCNT2=0;
         Pin2LOW(PORTD,5);// start lighting
         do{}while(TCNT2<Duration); 
         Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // stop light
         sei();
     }// for
    Pin2LOW(PORTD,7);//digitalWrite(SRCLR,LOW) // can not write to 595 (cleared now)
}

void FlashTest(word Duration) // #2 pin used as test
{
  Pin2Output(DDRD,6);Pin2HIGH(PORTD,6);  // switch on mosfet
  Pin2Output(DDRD,5);Pin2HIGH(PORTD,5);  // SRCLR to HIGH. when it is LOW all regs are cleared
  Pin2Output(DDRD,7);Pin2HIGH(PORTD,7);  // G stop light
  Pin2Output(DDRB,1);Pin2HIGH(PORTB,1);  // DATAPIN to HIGH
  Pin2Output(DDRB,0);  // CLK&LATCH
  delayMicroseconds(2);

  Pin2HIGH(PORTB,0);Pin2LOW(PORTB,0); // clock pulse 
  Pin2LOW(PORTB,1); // next data are zeroes
  Pin2HIGH(PORTB,0);Pin2LOW(PORTB,0); // clock pulse 

////for(byte z=0;z<=1;z++)// serie of flashes
//{
  Pin2HIGH(PORTB,0);Pin2LOW(PORTB,0); // clock pulse 
//  Pin2LOW(PORTB,1); // next data are zeroes
cli();
TCNT1=0;
Pin2LOW(PORTD,7);// start lighting
do{}while(TCNT1<Duration);
Pin2HIGH(PORTD,7);//digitalWrite(G,HIGH); // stop light
sei();

//}// for

  Pin2LOW(PORTD,5);Pin2Input(DDRD,5);  // SRCLR to LOW. Clear regs
  Pin2LOW(PORTB,1);Pin2Input(DDRB,1);  // DATAPIN
  Pin2Input(DDRB,1); // CLK&LATCH 
  Pin2LOW(PORTD,7);Pin2Input(DDRD,7);  // detach G control pin (it is pull upped by resistor)
  if(TFT_IS_ON==0){Pin2LOW(PORTD,6);Pin2Input(DDRD,6);}

}



void Flash(byte lamps,byte Duration)
{
  Pin2Output(DDRD,6);Pin2HIGH(PORTD,6);  // switch on mosfet
  Pin2Output(DDRD,5);Pin2HIGH(PORTD,5);  // SRCLR to HIGH. when it is LOW all regs are cleared
  Pin2Output(DDRD,7);Pin2HIGH(PORTD,7);  // G stop light
  Pin2Output(DDRB,1);Pin2HIGH(PORTB,1);  // DATAPIN to HIGH
  Pin2Output(DDRB,0);  // CLK&LATCH

  delayMicroseconds(2);

  Pin2HIGH(PORTB,0);Pin2LOW(PORTB,0); // clock pulse 
  Pin2LOW(PORTB,1); // next data are zeroes

// shift register is cleared
// what is in storage register we don't care because G is HIGH and all outputs are OFF

//  delayMicroseconds(2);

//t=0;

//  if((z&7)==0){Pin2HIGH(PORTB,1);} // 1st bit is "1"
//  Pin2HIGH(PORTB,1); // 1st bit is "1"


for(byte z=0;z<lamps;z++)// serie of flashes
{
//  if((z&7)==0){Pin2HIGH(PORTB,1);} // 1st bit is "1"
  Pin2HIGH(PORTB,0);  Pin2LOW(PORTB,0); // clock pulse 
//  delayMicroseconds(2);

//  if((z&7)==0){Pin2LOW(PORTB,1);} // next 7 are zeroes
//  Pin2LOW(PORTB,1); // next data are zeroes
  
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
TCNT1=0;
Pin2LOW(PORTD,7);// start lighting
//Pin2LOW(PORTD,0);// start lighting

//NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
//NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
//delayMicroseconds(5);//10us
/*
if((it==0x1000)&&(z==0)){   
Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // start/stop light
  SetADC(0,4,500); // pin A4 
Pin2LOW(PORTD,5); //start light

  
  mRawADC(VccN[0],2);    
  mRawADC(VccN[1],2);    
  mRawADC(VccN[2],2);    
  mRawADC(VccN[3],2);    
  mRawADC(VccN[4],2);    
Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // stop light
  mRawADC(VccN[5],2);    
  mRawADC(VccN[6],2);    
  delayMicroseconds(5);//10
  mRawADC(VccN[7],2);    
      ADCoff;
      t=1;
}
else if((it==0x0200)&&(z==0)){   
Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // start/stop light

  SetADC(0,14,500);
//  SetADC(1,8,50);
Pin2LOW(PORTD,5);

  
  mRawADC(VccN2[0],2);    
  mRawADC(VccN2[1],2);    
  mRawADC(VccN2[2],2);    
  mRawADC(VccN2[3],2);    
  mRawADC(VccN2[4],2);    
Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // start/stop light
  mRawADC(VccN2[5],2);    
  mRawADC(VccN2[6],2);    
  delayMicroseconds(5);//10
  mRawADC(VccN2[7],2);    
      ADCoff;
      t=1;
}
else {*/
//delayMicroseconds(14);// 28 us
//  if(it==15000)
//{
//  delayMicroseconds(20);// 40 us
//}  
//else{

  //delayMicroseconds(9);// 18 us
//}
//}//30us

do{}while(TCNT1<Duration);


//if(it==6000){delayMicroseconds(400);} // ~800us

Pin2HIGH(PORTD,7);//digitalWrite(G,HIGH); // stop light
//tim=TCNT1;    // 29-30us

//exit critical section >> if mcu haven't got here within 1-2ms then it will be rebooted
//Pin2Output(DDRD,0);Pin2HIGH(PORTD,0);// start charging timeout capacitor (default state)// internal pull up?
/*
if(it==5000){delayMicroseconds(50);} // 100us
if(it==10000){delayMicroseconds(100);} // 200us
if(it==15000){delayMicroseconds(150);} // 300us
if(it==20000){delayMicroseconds(200);} // 400us (both USB & battery powered) (R=20k)
if(it==25000){delayMicroseconds(250);} // 500us
if(it==30000){delayMicroseconds(300);} // 600us
if(it==35000){delayMicroseconds(350);} // 700us
if(it==40000){delayMicroseconds(400);} // 800us
if(it==45000){delayMicroseconds(450);} // 900us
if(it==50000){delayMicroseconds(500);} // 1000us
if(it==55000){delayMicroseconds(550);} // 1100us
if(it==60000){delayMicroseconds(600);} // 1200us
*/

//Pin2LOW(PORTD,0);// close reset mosfet
//}//if

sei();

//  if(it==15000)
  //{
  
//  delay(7); // flash time
// lvv = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
//  lvv <<= 16;
//  lvv |= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);

//  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);  //  disable();//  tsl.disable();

//}

//NOP;//sei();delay(2000);cli();
//if((it==0x0200)&&(z<8)){    mRawADC(VccN[z],2);    mRawADC(VccN2[z],2);}
//else
//{
//Pin2LOW(PORTD,0);// close reset mosfet

//}

//nap();
  //if((it==0x0200)&&(z==7)){ADCoff;t=1;}

}// for

  Pin2LOW(PORTD,5);Pin2Input(DDRD,5);  // SRCLR to LOW. Clear regs
  Pin2LOW(PORTB,1);Pin2Input(DDRB,1);  // DATAPIN
  Pin2Input(DDRB,1); // CLK&LATCH 
  Pin2LOW(PORTD,7);Pin2Input(DDRD,7);  // detach G control pin (it is pull upped by resistor)
  if(!TFT_IS_ON){Pin2LOW(PORTD,6);Pin2Input(DDRD,6);}

//if(it==1022){ADCSRA&=~(1<<ADEN); // stop ADC
//ADCSRA=0;
//}

}



// charging capacitor  on D2 pin. when it is discharged   to logic "0" INT0 on low level is fired
//===========================================================================================

byte pin2_interrupt_flag=0;
byte pin3_interrupt_flag=0;

volatile word  cnt1; // vars updated in ISR should be declared as volatile and accessed with cli()/sei() ie atomic

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
word sleeps;
uint16_t t1,t2,tt1,tt2,tt3,ttt1,ttt2;

word it=0;
byte rnd;
long flashes=0;
long ln=0;
long fn=0;
word tim;
word fnt,lnt; // fast/long nap time

void unap(void)
{
//PORTC=0;
//PORTB=0;
//PORTD=0;

  // Setup the WDT 
  cli();
  __asm__ __volatile__("wdr\n\t");//  wdt_reset();
  MCUSR &= ~(1<<WDRF);  // Clear the reset flag. 
  WDTCSR |= (1<<WDCE) | (1<<WDE); //  In order to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles).
  WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s
  sei();

            WDhappen=0;
            WDsleep=1;// notify WD that we are sleeping (to avoid rebbot)
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

void longnap(void)
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
    Pin2Input(DDRB,4);Pin2LOW(PORTB,4);    
    Pin2Input(DDRB,5);Pin2LOW(PORTB,5);
*/
// не нравится...
//.. по кнопке пробуем

// instead of charging cap we must set internal pullup resistor to get SQW from DS1307
    Pin2Input(DDRD,3);Pin2HIGH(PORTD,3);// pinMode(3,INPUT_PULLUP);

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
//    Pin2Input(DDRB,4);Pin2LOW(PORTB,4);    
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
//Pin2Output(DDRB,1); // DATAPIN 9
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


void NiceBack(byte x1,byte y1,byte x2,byte y2)
{
  byte r,g,b;

  setAddrWindow(y1,x1,y2,x2);
  
  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1);

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
  Pin2HIGH(PORTD,1);
  
  setAddrWindow(144,0,167,127);
  
  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1);

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
  Pin2HIGH(PORTD,1);
  
//  fillRect(16,32,48,64,0);
//  fillRect(0,16,128,96,0);

}


void ShowBars(byte hr)
{
  byte r,g,b,gg;

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
  for(byte i=0;i<24;i++)
  {
    gg=(Intensity[i]*14)/mi;if(!gg){gg=1;}

    if(i!=hr){r=0x8c;g=0xac;b=0x8c;}else{r=0x8c;g=0xfc;b=0x4c;}
    
    setAddrWindow(0+16-gg,4+i*5,15,4+i*5+4);

  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1);

//  for(byte j=0;j<4;j++)
  //{

  for(byte k=0;k<(gg*4);k++)
  {
      spiwrite(r);
      spiwrite(g);
      spiwrite(b);
  }  
           
//    }
  Pin2HIGH(PORTD,1);
  }
  
}

word naptime;

word SleepTime(void)
{
    word res=0;

    set_sleep_mode(SLEEP_MODE_IDLE);
    WDsleep=1;// notify WD that we are sleeping (to avoid reboot)
    __asm__ __volatile__("wdr\n\t");//  wdt_reset(); // to avoid WD fire first
    fastnap();
    if (pin2_interrupt_flag){res=cnt1;}else { ERR=ERR_BROKEN_SLEEP; }
    return res;
}


  volatile long lvv,lmv,lm2;// luminous
  volatile long mlvv,mlmv,mlm2;// luminous
  volatile word lasttouch,ltp;

// the loop routine runs over and over again forever:
void loop() {
  word t,t1,n;
  word Temp,rtcl;
  char tmps[32];
  
  
  do{
    
    if(it==0){naptime=SleepTime();}
    /*
if(TFT_IS_ON)
{
Pin2Output(DDRD,6);Pin2HIGH(PORTD,6);// charge mosfet
delayMicroseconds(5);
cli();
TCNT1=0;
TCNT2=0;
Pin2LOW(PORTD,6);Pin2Input(DDRD,6);
do{
if((PORTD&0b01000000)==0){break;}
}while(1);
tt1=TCNT2;
tt2=TCNT1;
sei();
Pin2Output(DDRD,6);Pin2HIGH(PORTD,6);
}  */

  
//if(it>7000){
  //    Pin2LOW(PORTD,2);Pin2Input(DDRD,2); // controlled charging (very impurtant set it 2 input (high impedance state))
//delay(1);//delayMicroseconds(1000);
//        Pin2Output(DDRD,2);Pin2HIGH(PORTD,2);// start charging timeout capacitor (default state)// internal pull up?
//}


//  __asm__ __volatile__("wdr\n\t");//  wdt_reset();


        Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // stop light outputs
//        Pin2HIGH(PORTB,6);//power supply to tpic6a595  (add caps?)
  //      delayMicroseconds(11);// wait for rise. 10 minimum to avoid nasty bugs

//Pin2Output(DDRB,7);Pin2HIGH(PORTB,7);// включаем питание  дисплея


TCNT1=0;

if((it&0xFF)==0) // 1 из 256
{
  I2C_ON(DDRC,1,2); // need to open mosfet also!!!
  Pin2Output(DDRD,6);Pin2HIGH(PORTD,6); // open rtc/tft mosfet
  if(Check_RTC(16)==0){ERR=ERR_WHERE_IS_THE_CLOCK;} 
  else
  {
//    if(Read_I2C(DS1307_ADDR_W,2,A1,A2)!=0x15) {Save_I2C(DS1307_ADDR_W,2,0x15,A1,A2);Save_I2C(DS1307_ADDR_W,1,0x57,A1,A2);Save_I2C(DS1307_ADDR_W,0,0x00,A1,A2);}//set time
    if(TFT_IS_ON) {CS=Read_I2C(DS1307_ADDR_W,0,A1,A2); if(CS!=PS){PS=CS; if(--TFT_IS_ON==0){TFT_OFF();}}}// если дисплей включен то проверим не пора ли его выключить
    else {ltp=lasttouch;lasttouch=TouchSensor();Etouch=TouchT(); if (lasttouch>Etouch){TFT_ON(15);InitTFT();ShowBars(HR);} else{TouchD[TouchPos]=lasttouch;(++TouchPos)&=3;}}
  
    HR=Read_I2C(DS1307_ADDR_W,2,A1,A2);
    if(HR>=0x20){HR-=12;}else if(HR>=0x10){HR-=6;} //  читаем текущий час и конветируем из упакованного BCD
    if(HR>23) {ERR=ERR_STRANGE_CLOCK_DATA; } // не пойми что с часов пришло
    else{ FlashDuration=Intensity[HR]; if(HR!=PH){PH=HR;} } // новый час
 
  }
  if(TFT_IS_ON==0){Pin2LOW(PORTD,6);Pin2Input(DDRD,6);} //close rtc/tft mosfet if it is not in use by TFT
  I2C_OFF(DDRC,PORTC,1,2);  // переводим лапки часиков в высокоомное состояние 
  rtcl=TCNT1;
  
//  I2C_ON(DDRC,1,2);
  //if(!Check_TSL(16)){ERR=WHERE_IS_THE_TSL2561;} 
//  else{

//TSL2561_START(TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_16X);//
//TSL2561_START(TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_0X);////
//delay(14);
//lvv=TSL2561_STOP();

//}
  //I2C_OFF(DDRC,PORTC,1,2);
  
}

/*
*/


// если есть ошибки
if (ERR)
{
    I2C_ON(DDRC,1,2);
    TFT_ON(3);
    InitTFT();  
    fillScreen(0x000000);

    word cycles=TouchSensor();
    setAddrWindow(0,0,7,119);
    byte rtc=Check_RTC(16);
    ta("ERR");th(ERR);ta(" RTC:");tn(10,rtc);th(rtc8);ta(" n");tn(10000,naptime);
    setAddrWindow(10,01,7,119);
    ta("cycles:");tn(10000,cycles);

    delay(2500);
    TFT_OFF(); // close rtft/rtc mosfet
    I2C_OFF(DDRC,PORTC,1,2);  // переводим лапки часиков в высокоомное состояние 
    resetFunc();  // This will call location zero and cause a reboot.
}

//  if(!FlashDuration){unap();continue;} // определяем продолжительность пыхи в данном часе и спим 8s если нечего делать

if(((it&0x3FF)==0)&&TFT_IS_ON)// once in 1k
{

//  rtc.poke(10,100);
//byte  val=rtcpeek(15);
//byte  val=rtc.peek(8);
//byte val=0;

//	pinMode(_scl_pin, OUTPUT);

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


//TFT_ON(3); 

// 33ms!!! draw them only once in an hour
//TCNT1=0;
//if(HR!=PH){ShowBars(HR); PH=HR;}
//ShowBars(HR); // every time when display is ON
//word ttt=TCNT1;

  setAddrWindow(60,0,67,127);
tn(10000,it);ta(" nap");tn(10000,naptime);
  setAddrWindow(70,0,77,127);
lh(lvv);ta(" ");lh(lmv);ta(" ");lh(lm2);//t3(val);th('A');th(vv);th(v2);
  setAddrWindow(80,0,87,127);
lh(mlvv);ta(" ");lh(mlmv);ta(" ");lh(mlm2);//t3(val);th('A');th(vv);th(v2);
  setAddrWindow(90,0,97,127);
ta(" 1 ");tn(10000,tt1);ta(" 2 ");tn(10000,tt2);ta(" 3 ");tn(10000,tt3);

  setAddrWindow(20,0,27,127);
  ta("Сон"); tn(1000,fnt);  ta(" Пых");tn(100,FlashDuration);ta(" E");th(ERR);ta(" T");wh(Etouch);

//  setAddrWindow(142,0,149,127);  tn(100000000,123456789);
/*
setAddrWindow(142,0,149,127);  

for(byte e=0;e<4;e++){wh(TouchD[e]);}

ta(" mi");tn(10,mi);
*/
  setAddrWindow(152,0,159,127);

//th(tstr[2]);ta(":");th(tstr[1]);ta(".");th(tstr[0]);ta(" ");th(tstr[4]);ta("-");th(tstr[5]);ta("-");th(tstr[6]);ta(" ");
tn(10,HR);ta(" ");th(FlashDuration);th(TFT_IS_ON);ta(" lt");wh(lasttouch);ta(" ");wh(ltp);ta(" l:");wh(rtcl);


  

}

//if((it&0x3FF)==0)// once in 1024
//{

  //  t=0;  

  //once in 65536
  if(it==0xFFFF){flashes++;}


if(FlashDuration)
{
//    Flash(8,FlashDuration);

//    Flash(16,FlashDuration);
FlashTest(10000);

//    Flash(9,FlashDuration);
/*    
//  if(it==1000){lmv=FlashM(8,FlashDuration);}// измерение 0x0D пых=12us
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
  
    /*
    I2C_ON(DDRC,1,2);
    if(Check_TSL(16))
    {  
    TSL2561_START(TSL2561_INTEGRATIONTIME_101MS |TSL2561_GAIN_0X); 
    delay(2);
    TCNT1=0;
    for( word i=0;i<50;i++){FlashZ(1,16);} 
    tt1=TCNT1;    
     delay(100);
      lvv=TSL2561_STOP();
    }
    I2C_OFF(DDRC,PORTC,1,2);


delay(3000);

    I2C_ON(DDRC,1,2);
    if(Check_TSL(16))
    {  
    TSL2561_START(TSL2561_INTEGRATIONTIME_101MS |TSL2561_GAIN_0X); 
    delay(2);
    TCNT1=0;
    for( word i=0;i<200;i++){FlashZ(1,4);} 
    tt2=TCNT1;    
     delay(100);
      lmv=TSL2561_STOP();
    }
    I2C_OFF(DDRC,PORTC,1,2);

delay(3000);

    I2C_ON(DDRC,1,2);
    if(Check_TSL(16))
    {  
    TSL2561_START(TSL2561_INTEGRATIONTIME_101MS |TSL2561_GAIN_0X); 
    delay(2);
    TCNT1=0;
    for( word i=0;i<800;i++){FlashZ(1,1);} 
    tt3=TCNT1;    
     delay(100);
      lm2=TSL2561_STOP();
    }
    I2C_OFF(DDRC,PORTC,1,2);
*/
//delay(2000);

//}// измерение
    
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

  //  Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
  //    Pin2Input(DDRB,1);//  pinMode(9,INPUT);

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
//else{ln++;longnap();}  
else {fn++;fastnap();} 
//if ((it&0x3)==0){ln++;longnap();}else {fn++;fastnap();} // 1:3

}
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
//Pin2Output(DDRB,1); // DATAPIN 9
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

  //delay(1000);               // wait for a second
    }while(1); // loop
}


