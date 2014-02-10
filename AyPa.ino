//#define __AVR_ATmega328P__ 
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>


#include <SPI.h> // < declaration ShiftOut etc problem

#define TSL2561_ADDR_LOW_W 82
#define TSL2561_ADDR_LOW_R  83 //(0x29<<1)+1

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
  TSL2561_INTEGRATIONTIME_402MS     = 0x02     // 402ms
}
tsl2561IntegrationTime_t;

typedef enum
{
  TSL2561_GAIN_0X                   = 0x00,    // No gain
  TSL2561_GAIN_16X                  = 0x10,    // 16x gain
}
tsl2561Gain_t;


#include "AyPa_m.h"
#include "AyPa_fonts.h"
#include "AyPa_n.h"
#include "AyPa_TFT.h"
//#include "AyPa_rtc.h"



//#include <DS1302.h>// cannot sit on SPI pins (leaves pin in input state)
//#include <DS1307.h>// cannot sit on SPI pins (leaves pin in input state)

//#define DS1307_ADDR 0xD0
#define DS1307_ADDR_R	209
#define DS1307_ADDR_W	208

#define DS1307_SQW_RATE_1		0x10
#define DS1307_SQW_RATE_4K		0x11
#define DS1307_SQW_RATE_8K		0x12
#define DS1307_SQW_RATE_32K  	0x13

#define _sda_pin A4
#define _scl_pin A5



void	_sendStart(byte addr)
{
	pinMode(_sda_pin, OUTPUT);
	digitalWrite(_sda_pin, HIGH);
	digitalWrite(_scl_pin, HIGH);
	digitalWrite(_sda_pin, LOW);
	digitalWrite(_scl_pin, LOW);
	shiftOut(_sda_pin, _scl_pin, MSBFIRST, addr);
}

void	_sendStop()
{
	pinMode(_sda_pin, OUTPUT);
	digitalWrite(_sda_pin, LOW);
	digitalWrite(_scl_pin, HIGH);
	digitalWrite(_sda_pin, HIGH);
	pinMode(_sda_pin, INPUT);
}

void	_sendNack()
{
	pinMode(_sda_pin, OUTPUT);
	digitalWrite(_scl_pin, LOW);
	digitalWrite(_sda_pin, HIGH);
	digitalWrite(_scl_pin, HIGH);
	digitalWrite(_scl_pin, LOW);
	pinMode(_sda_pin, INPUT);
}

void	_sendAck()
{
	pinMode(_sda_pin, OUTPUT);
	digitalWrite(_scl_pin, LOW);
	digitalWrite(_sda_pin, LOW);
	digitalWrite(_scl_pin, HIGH);
	digitalWrite(_scl_pin, LOW);
	pinMode(_sda_pin, INPUT);
}

void	_waitForAck()
{
	pinMode(_sda_pin, INPUT);
	digitalWrite(_scl_pin, HIGH);
TCNT1=0;
	while (_sda_pin==LOW) {if(TCNT1>100){break;}}
	digitalWrite(_scl_pin, LOW);
}

uint8_t _readByte()
{
	pinMode(_sda_pin, INPUT);

	uint8_t value = 0;
	uint8_t currentBit = 0;

	for (int i = 0; i < 8; ++i)
	{
		digitalWrite(_scl_pin, HIGH);
		currentBit = digitalRead(_sda_pin);
		value |= (currentBit << 7-i);
		delayMicroseconds(1);
		digitalWrite(_scl_pin, LOW);
	}
	return value;
}
void _writeByte(uint8_t value)
{
	pinMode(_sda_pin, OUTPUT);
	shiftOut(_sda_pin, _scl_pin, MSBFIRST, value);
}

uint8_t _readRegister(uint8_t reg)
{
	uint8_t	readValue=0;

	_sendStart(DS1307_ADDR_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_sendStop();
	_sendStart(DS1307_ADDR_R);
	_waitForAck();
	readValue = _readByte();
	_sendNack();
	_sendStop();
	return readValue;
}

void _writeRegister(uint8_t reg, uint8_t value)
{
	_sendStart(DS1307_ADDR_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_writeByte(value);
	_waitForAck();
	_sendStop();
}

void _writeRegisterT(uint8_t reg, uint8_t value)
{
	_sendStart(TSL2561_ADDR_LOW_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_writeByte(value);
	_waitForAck();
	_sendStop();
}

uint8_t _readRegisterT(uint8_t reg)
{
	uint8_t	readValue=0;

	_sendStart(TSL2561_ADDR_LOW_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_sendStop();
	_sendStart(TSL2561_ADDR_LOW_R);
	_waitForAck();
	readValue = _readByte();
	_sendNack();
	_sendStop();
	return readValue;
}

/*
uint16_t _readRegister16(uint8_t reg)// not tested
{
	uint16_t	readValue=0;

	_sendStart(TSL2561_ADDR_LOW_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_sendStop();
	_sendStart(TSL2561_ADDR_LOW_R);
	_waitForAck();
        _writeByte(2);
	_waitForAck();
	readValue = _readByte();
	_sendNack();
        readValue <<= 8;
	readValue |= _readByte();
	_sendNack();
	_sendStop();
	return readValue;
}*/

/*
void _burstRead()
{
	_sendStart(DS1307_ADDR_W);
	_waitForAck();
	_writeByte(0);
	_waitForAck();
	_sendStop();
	_sendStart(DS1307_ADDR_R);
	_waitForAck();

	for (int i=0; i<8; i++)
	{
		_burstArray[i] = _readByte();
		if (i<7)
			_sendAck();
		else
			_sendNack();
	}
	_sendStop();
}*/

void poke2(uint8_t addr, uint8_t value)
{
//	if ((addr >=0) && (addr<=55+8))
//	{
//		addr += 8;
		_sendStart(DS1307_ADDR_W);
		_waitForAck();
		_writeByte(addr);
		_waitForAck();
		_writeByte(value);
		_waitForAck();
		_sendStop();
//	}
}

//CLKrtc IOrtc
#define CLKrtc 5
#define IOrtc 4

void ShiftOutRTC(byte val)
{
//PORTC&=~(1<<CLKrtc);//clk low (already)

//PORTC|=(1<<CLKrtc);// tick clk    //8
//if(PINC&(1<<IOrtc))val|=(1<<5);   //6
//PORTC&=~(1<<CLKrtc);//clk low

/*
__asm__ __volatile__(
"out 0x08,r1\n\t" // clear data&clk
"sbrc %0,0\n\t"  // if (val&(1<<(0)))PORTC|=(1<<IOrtc);// set it if needed
"sbi 0x08,2\n\t" // set data bit
"sbi 0x08,3\n\t" // tick clk

"out 0x08,r1\n\t" // clear data&clk
"sbrc %0,1\n\t"  // if (val&(1<<(0)))PORTC|=(1<<IOrtc);// set it if needed
"sbi 0x08,2\n\t" // set data bit
"sbi 0x08,3\n\t" // tick clk

"out 0x08,r1\n\t" // clear data&clk
"sbrc %0,2\n\t"  // if (val&(1<<(0)))PORTC|=(1<<IOrtc);// set it if needed
"sbi 0x08,2\n\t" // set data bit
"sbi 0x08,3\n\t" // tick clk

"out 0x08,r1\n\t" // clear data&clk
"sbrc %0,3\n\t"  // if (val&(1<<(0)))PORTC|=(1<<IOrtc);// set it if needed
"sbi 0x08,2\n\t" // set data bit
"sbi 0x08,3\n\t" // tick clk

"out 0x08,r1\n\t" // clear data&clk
"sbrc %0,4\n\t"  // if (val&(1<<(0)))PORTC|=(1<<IOrtc);// set it if needed
"sbi 0x08,2\n\t" // set data bit
"sbi 0x08,3\n\t" // tick clk

"out 0x08,r1\n\t" // clear data&clk
"sbrc %0,5\n\t"  // if (val&(1<<(0)))PORTC|=(1<<IOrtc);// set it if needed
"sbi 0x08,2\n\t" // set data bit
"sbi 0x08,3\n\t" // tick clk

"out 0x08,r1\n\t" // clear data&clk
"sbrc %0,6\n\t"  // if (val&(1<<(0)))PORTC|=(1<<IOrtc);// set it if needed
"sbi 0x08,2\n\t" // set data bit
"sbi 0x08,3\n\t" // tick clk

"out 0x08,r1\n\t" // clear data&clk
"sbrc %0,7\n\t"  // if (val&(1<<(0)))PORTC|=(1<<IOrtc);// set it if needed
"sbi 0x08,2\n\t" // set data bit
"sbi 0x08,3\n\t" // tick clk

"out 0x08,r1\n\t" // clear data&clk

:: "r" (val):);*/

PORTC&=~(1<<IOrtc);//clear data bit
//__asm__ __volatile__ ( "out 0x08,r1\n\t"        :::);
//__asm__ __volatile__ ( "cbi 0x08,2\n\t"        :::);

if (val&(1<<(0)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

PORTC&=~(1<<CLKrtc);//clk low
PORTC&=~(1<<IOrtc);//clear data bit
//__asm__ __volatile__ ( "clr r24\n\t""out 0x08,r24\n\t"        :::);

//PORTC=~((1<<IOrtc)|(1<<CLKrtc));//clk low +  clear data bit
if (val&(1<<(1)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

PORTC&=~(1<<CLKrtc);//clk low
PORTC&=~(1<<IOrtc);//clear data bit
//PORTC=~((1<<IOrtc)|(1<<CLKrtc));//clk low +  clear data bit
if (val&(1<<(2)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

PORTC&=~(1<<CLKrtc);//clk low
PORTC&=~(1<<IOrtc);//clear data bit
//PORTC=~((1<<IOrtc)|(1<<CLKrtc));//clk low +  clear data bit

if (val&(1<<(3)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

PORTC&=~(1<<CLKrtc);//clk low
PORTC&=~(1<<IOrtc);//clear data bit
if (val&(1<<(4)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

PORTC&=~(1<<CLKrtc);//clk low
PORTC&=~(1<<IOrtc);//clear data bit
if (val&(1<<(5)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

PORTC&=~(1<<CLKrtc);//clk low
PORTC&=~(1<<IOrtc);//clear data bit
if (val&(1<<(6)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

PORTC&=~(1<<CLKrtc);//clk low
PORTC&=~(1<<IOrtc);//clear data bit
if (val&(1<<(7)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

//PORTC&=~(1<<CLKrtc);//clk low (needed)
//"=r" (result): "I" (val)

}

void	I2C_START(byte addr)
{
//	pinMode(_sda_pin, OUTPUT);
        Pin2HIGH(PORTC,4);//	digitalWrite(_sda_pin, HIGH);
// get port from pin//portOutputRegister(digitalPinToPort(cs);
	Pin2HIGH(PORTC,5);//digitalWrite(_scl_pin, HIGH);
        Pin2LOW(PORTC,4);//	digitalWrite(_sda_pin, LOW);
        Pin2LOW(PORTC,5);//	digitalWrite(_scl_pin, LOW);
//	shiftOut(_sda_pin, _scl_pin, MSBFIRST, addr);
	ShiftOutRTC(addr);
}

void Save_I2C(byte addr,byte reg,byte val)
{
//  		_sendStart(addr);
  I2C_START(addr);
		_waitForAck();
		_writeByte(reg);
		_waitForAck();
		_writeByte(val);
		_waitForAck();
		_sendStop();
}

byte Read_I2C(byte addr,byte reg)
{
  byte val;
	_sendStart(addr);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_sendStop();
	_sendStart(addr|1);
	_waitForAck();
	val = _readByte();
	_sendNack();
	_sendStop();
	return val;
}

uint8_t peek2(uint8_t addr)
{
//	if ((addr >=0) && (addr<=55+8))
//	{
		uint8_t readValue;

	//	addr += 8;
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

		return readValue;
//	}
//	else
//		return 0;
}


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
//DS1307 rtc(A4,A5);



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
#define DATAPIN 9
#define CLOCKPIN 8
#define LATCHPIN 6


#define RTC_ON(DRC,p1,p2) {Pin2Output(DRC,p1);Pin2Output(DRC,p2);}
#define RTC_OFF(DRC,PORT,p1,p2) {Pin2Input(DRC,p1);Pin2LOW(PORT,p1);Pin2Input(DRC,p2);Pin2LOW(PORT,p2);}

// каждый раз перед обращением к часам проверяем их вменяемость
byte Check_RTC(byte attempts){for(byte n=attempts;n>0;n--){Save_I2C(DS1307_ADDR_W,8,'A');if(Read_I2C(DS1307_ADDR_W,8)=='A'){return n;}}return 0;}

//#define DS1307_I2C_ADDRESS 0x68 // read and write address is different for DS1307 

// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively TSL2561_ADDR_FLOAT)
//TSL2561 tsl(TSL2561_ADDR_LOW); 

// port B: 5 port C: 8 port D:11
//byte cports[10]={PORTB1,PORTB2,PORTB1,PORTB1,PORTB1,PORTB1,PORTB1,PORTB1,PORTB1,PORTB1};
  char tstr[7];// строка даты и времени
//  boolean DS1307here=false; // часики работают




byte pp[10]={
  5,5,5,5,5,5,5,5,5,5}; //port
byte pb[10]={
  1,0,0,0,0,0,0,0,0,0}; // bit in port
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
  sei();



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

Pin2Output(DDRB,7);Pin2HIGH(PORTB,7);// включаем питание  дисплея

//Pin2Output(DDRC,4);Pin2Output(DDRC,5);
RTC_ON(DDRC,4,5);
//delay(10);

TCNT1=0;
//  rtc.poke(8,'A');
byte val=Check_RTC(16);


//if  (RTC_Here(10)==0)
//if  (RTC_Here(10)==0)
//{
  // complain rtc is not here
  //do{}while(1);
//}

//  poke2(8,'A'); 
//Save_I2C(DS1307_ADDR_W,8,'A');

//  word vv=TCNT1;
//sei();
//byte val=0;
//byte  val=rtcpeek(15);
//byte  
//val=rtc.peek(8);

//byte  val=peek2(8);
//byte vv2=peek2(8);
TCNT1=0;
byte vv2=Check_RTC(3);
  word vv=TCNT1;
//byte val=Read_I2C(DS1307_ADDR_W,8);
//vv=Read_I2C(DS1307_ADDR_W,8);



tstr[4]=peek2(4);
tstr[5]=peek2(5);
tstr[6]=peek2(6);
tstr[3]=peek2(3);
tstr[0]=peek2(2);
tstr[1]=peek2(1);
tstr[3]=peek2(0);


/*
 byte imm=peek2(6);
 if(imm<0x14||imm>0x30)// check correct date
 {
   poke2(6,0x14);
   poke2(5,0x02);
   poke2(4,0x10);
   poke2(3,0x01);
   poke2(2,0x12);
   poke2(1,0x49);
   poke2(0,0x00);
 }*/

RTC_OFF(DDRC,PORTC,4,5);// переводим лапки часиков в высокоомное состояние чтобы они не сливали ток через подтягивательные резисторы на шине I2C

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



    Pin2Output(DDRB,pb[0]);
    if(v==1)
    {

    }// if 1
   
    

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


  // setup timer2 


  /*
  cli();
   //Disable timer2 interrupts
   TIMSK2  = 0;
   //Enable asynchronous mode
   ASSR  = (1<<AS2);
   //set initial counter value
   TCNT2=0;
   //set prescaller 128
   //    TCCR2B |= (1<<CS22)|(1<<CS00);
   TCCR2B=(0<<CS22)|(0<<CS21)|(1<<CS20); // /no prescaler
   //wait for registers update
   while (!(ASSR & ((1<<TCN2UB)|(1<<TCR2BUB))));
   //clear interrupt flags
   TIFR2  = (1<<TOV2);
   //enable TOV2 interrupt
   TIMSK2  = (1<<TOIE2);
   sei();*/
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
Pin2Output(DDRB,3);// MOSI
Pin2Output(DDRB,5);// CLK




Pin2Output(DDRB,0); // CLOCKPIN 8
Pin2Output(DDRB,1); // DATAPIN 9
Pin2Output(DDRB,2);
Pin2Output(DDRB,6);


//Pin2Output(DDRC,2);
//Pin2Output(DDRC,3);
//Pin2Output(DDRC,4);
//Pin2Output(DDRC,5);

Pin2Output(DDRD,5); // pin 5 G
Pin2Output(DDRD,6); // pin 6 LATCH
Pin2Output(DDRD,7); // pin 7 SRCLR



  pinMode(DATAPIN,OUTPUT);
  pinMode(CLOCKPIN,OUTPUT);
  pinMode(LATCHPIN,OUTPUT);


PORTC=0;
PORTB=0;
PORTD=0;


// test LCD

//Pin2Output(DDRC,4);
//Pin2HIGH(PORTC,4);
//delay(1);


  initR();   // initialize a ST7735S chip, black tab


//  fillRect(0,0,27,91,0xfcfcfc);

//fillScreen(0xfcfc00);


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

  uint16_t time = millis();
  fillScreen(0x000000);// black (64ms)
//  fillScreen(0x005000);// green

  time = millis() - time;

 //setAddrWindow(0,0,7,120);
//wh(time);ta("TIME");

//delay(1500);
  // a single pixel
//  tft.drawPixel(tft.width()/2, tft.height()/2, ST7735_GREEN);
//fillRect(5,5,10,100,0xE400fc);


      char nam[16]="_";

word entries=0;
byte flag=0;

word x,xf,xi;
long lm;
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
  



  setAddrWindow(20,0,27,127);
ta("val:");t3(val);ta(" ");t3(val);ta(" ");th(val);ta(" ");wh(vv2);ta("t");wh(vv);

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

  setAddrWindow(80,10,87,120);
ta("Русский");

delay(2800);



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
void flash(void)
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

// charging capacitor  on D2 pin. when it is discharged   to logic "0" INT0 on low level is fired
//===========================================================================================

byte pin2_interrupt_flag=0;
byte pin3_interrupt_flag=0;

volatile word  cnt1; // vars updated in ISR should be declared as volatile and accessed with cli()/sei() ie atomic

void pin2_isr()
{
  cnt1=TCNT1;
  detachInterrupt(0);  //INT 0
  sleep_disable();// a bit later
  pin2_interrupt_flag = 1;
}
void pin3_isr()
{
  cnt1=TCNT1;
  detachInterrupt(1); //INT 1
  sleep_disable();// a bit later
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
uint16_t t1,t2,tt1,tt2,ttt1,ttt2;

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


Pin2Output(DDRB,0); // CLOCKPIN 8
Pin2Output(DDRB,1); // DATAPIN 9
Pin2Output(DDRB,2);
Pin2Output(DDRB,6);

Pin2Output(DDRD,5); // pin 5 G
Pin2Output(DDRD,6); // pin 6 LATCH
Pin2Output(DDRD,7); // pin 7 SRCLR

  pinMode(DATAPIN,OUTPUT);
  pinMode(CLOCKPIN,OUTPUT);
  pinMode(LATCHPIN,OUTPUT);


  
  //cli();t1111=TCNT1;sei();//atomic read
}
void fastnap(void)
{
//?????????????????

//PORTC=0;
//PORTB=0;
//PORTD=0;
//DDRB=0;// some needed pins are not put to output after wakeup
//DDRD=0;
//DDRC=0;// all pins as inputs and low (high impedance state)
//PORTD=0b00000100; //except 2nd pin (INT0)


     // cnt1=0;
      //tc1=TCNT1;
//        set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //// in r24,0x33// andi r24,0xF1// ori r24,0x04// out 0x33,r24
//      set_sleep_mode (SLEEP_MODE_IDLE);//// in r24,0x33// andi r24,0xF1// out 0x33,r24

//Pin2Output(DDRD,0);// sleep 2n7000 control
//Pin2HIGH(PORTD,0);// switch ON sleep control mosfet

Pin2Output(DDRD,2);Pin2HIGH(PORTD,2);// charge cap
delayMicroseconds(2);

//            WDhappen=0;
        sleeps=0;
      //  tim=TCNT1;
    //  TCNT1=0;
      do{

            cli();
      pin2_interrupt_flag=0;
      sleep_enable();
      attachInterrupt(0, pin2_isr, LOW);

      Pin2LOW(PORTD,2);Pin2Input(DDRD,2); // controlled charging (very impurtant set it 2 input (high impedance state))
        sei();
        sleep_cpu();
//wake up here
// check if it us or not
        sleep_disable();
  //      if(pin3_interrupt_flag||WDhappen){break;}else{sleeps++;}
        if(pin2_interrupt_flag){break;}else{sleeps++;}
      }while(1);
  
//  cli();t1111=TCNT1;sei();//atomic read
}

/*
uint16_t read16(uint8_t reg)
{
  uint16_t x; uint16_t t;

  Wire.beginTransmission(TSL2561_ADDR_LOW);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(TSL2561_ADDR_LOW, 2);
  t = Wire.read();
  x = Wire.read();
  x <<= 8;
  x |= t;
  return x;
}

void write8(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(TSL2561_ADDR_LOW);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t read88(uint8_t reg)
{
  uint8_t x;

  Wire.beginTransmission(DS1307_ADDR_W);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDR_R, 1);
  x = Wire.read();
  return x;
}

void write88(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(DS1307_ADDR_W);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();  
}

byte rtcpeek2(byte reg)
{
  byte bb;
  Wire.begin();
  bb=read88(8);
  return bb;
}

void rtcpoke2(byte reg,byte val)
{
  Wire.begin();
  write88(reg,val);
}
*/
/*    

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

/*
uint8_t DS1307::_readRegister(uint8_t reg)
{
	uint8_t	readValue=0;

	_sendStart(DS1307_ADDR_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_sendStop();
	_sendStart(DS1307_ADDR_R);
	_waitForAck();
	readValue = _readByte();
	_sendNack();
	_sendStop();
	return readValue;
}
*/

  volatile long lvv;// luminous

// the loop routine runs over and over again forever:
void loop() {
  word t,t1,n;
  word Temp;
  char tmps[32];
  
  
//if(it>7000){
  //    Pin2LOW(PORTD,2);Pin2Input(DDRD,2); // controlled charging (very impurtant set it 2 input (high impedance state))
//delay(1);//delayMicroseconds(1000);
//        Pin2Output(DDRD,2);Pin2HIGH(PORTD,2);// start charging timeout capacitor (default state)// internal pull up?
//}


//  __asm__ __volatile__("wdr\n\t");//  wdt_reset();


        Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // stop light outputs
//        Pin2HIGH(PORTB,6);//power supply to tpic6a595  (add caps?)
  //      delayMicroseconds(11);// wait for rise. 10 minimum to avoid nasty bugs

Pin2Output(DDRB,7);Pin2HIGH(PORTB,7);// включаем питание  дисплея


TCNT1=0;

if((it&0xFFF)==0)// once in 4k
{

//  rtc.poke(10,100);
//byte  val=rtcpeek(15);
//byte  val=rtc.peek(8);
//byte val=0;

//	pinMode(_scl_pin, OUTPUT);
RTC_ON(DDRC,4,5);


//byte  val=peek2(8);
byte val=Read_I2C(DS1307_ADDR_W,8);

byte vv=0,v2=0;
long lm=0;

if(val=='A')
{

    byte imm=peek2(6);
 if(imm<0x14||imm>0x30)// check correct date
 {
   poke2(6,0x14);
   poke2(5,0x02);
   poke2(4,0x10);
   poke2(3,0x01);
   poke2(2,0x13);
   poke2(1,0x02);
   poke2(0,0x00);
 }


tstr[4]=peek2(4);
tstr[5]=peek2(5);
tstr[6]=peek2(6);
tstr[3]=peek2(3);
tstr[0]=peek2(0);
tstr[1]=peek2(1);
tstr[2]=peek2(2);

RTC_OFF(DDRC,PORTC,4,5);

// check that we have sensor responding
vv=_readRegisterT(TSL2561_REGISTER_ID); // works

_writeRegisterT(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
_writeRegisterT(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,  TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_16X);   

delay(14);

lm=_readRegisterT(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN1_HIGH);
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

}

    initR();   // initialize a ST7735S chip, black tab

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

//  word time = millis();
//  fillScreen((it&0xfc)<<8);

//  time = millis() - time;

  // a single pixel
//  tft.drawPixel(tft.width()/2, tft.height()/2, ST7735_GREEN);
//fillRect(5,5,10,100,0xE400fc);
//sprintf(tmps, "%d %d %d", millis(),word(time),word(it));


  setAddrWindow(60,0,67,127);
wh(it);ta("LV:");lh(lvv);t3(val);th('A');th(vv);th(v2);


  setAddrWindow(0,0,7,119);
  ta("f:");t3(fnt);  ta(" l:");t3(lnt);

  setAddrWindow(152,0,159,127);

th(tstr[2]);ta(":");th(tstr[1]);ta(".");th(tstr[0]);ta(" ");th(tstr[4]);ta("-");th(tstr[5]);ta("-");th(tstr[6]);ta(" ");th(tstr[3]);

//long lm;
//TSL2561

//  if (tsl.begin()) {
    
//        write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);  //  enable();
  //write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,  TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_16X);    //  // Set integration time and gain


  
  //  tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
//  tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
//  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);  // longest integration time (dim light)
  
  // Now we're ready to get readings!
  
    // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 13-402 milliseconds! Uncomment whichever of the following you want to read
//  TCNT1=0;
//   lm = tsl.getFullLuminosity();
  
//if (!_initialized) begin();

  // Enable the device by setting the control bit to 0x03
//  tsl.enable(); // will init if need to
//  uint32_t x;
//delay(14); // flash time

// lm = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
  //lm <<= 16;
//  lm |= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);

//  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);  //  disable();//  tsl.disable();
//   word lmm=TCNT1;

//   x = tsl.getLuminosity(TSL2561_VISIBLE);     
//  xf = tsl.getLuminosity(TSL2561_FULLSPECTRUM);
//  xi = tsl.getLuminosity(TSL2561_INFRARED);

//  long lum = tsl.getFullLuminosity();
  word ir, full,lx;
  ir = lm >> 16;
  full = lm & 0xFFFF;
  setAddrWindow(9,0,16,127);
  //ta("I");th((ir>>8));th((ir&0xFF));
//  ta(" F");th((full>>8));th((full&0xFF));
//  ta(" V");th(((full-ir)>>8));th(((full-ir)&0xFF));
    ta("LM:");lh(lm);
  //lx=tsl.calculateLux(full,ir);
//  ta("L");wh(lx);
  //ta(" ");
  wh(lm);
  ta(" ");lh(lvv);

//  setAddrWindow(18,0,26,127);
  //ta("lv:");lh(lv);
  

}

//if((it&0x3FF)==0)// once in 1024
//{

  //  t=0;  

  //once in 65536
  if(it==0xFFFF){flashes++;

//    t=Vcc();    // measure with ADC inner voltage

}

/*
  LcdInit();

  LcdSet(0,0);
  sh(flashes>>24);
  sh((flashes>>16)&0xff);
  sh((flashes>>8)&0xff);
  sh(flashes&0xff);sa(" <F");
  sh(extreset);if(extreset&0b00000010){sa("R");}else{sa(" ");}   extreset=MCUSR;// check external reset flag
  sw(it);
  
  sh(ln>>24);
  sh((ln>>16)&0xff);
  sh((ln>>8)&0xff);
  sh(ln&0xff);sa(" <L");

  sh(fn>>24);
  sh((fn>>16)&0xff);
  sh((fn>>8)&0xff);
  sh(fn&0xff);sa(" <F");
  
  s3(tim);
  sa("<tim");
  s3(Vcc1);
  sa("Vcc1");
  
  sa("F");sw(fnt);sa("L");sw(lnt);sa(" S");s3(sleeps);sa(" ");
  
  for(byte d=0;d<8;d++){
  if(it==(1024+d*7*1024)){LcdSet(3,4);s2(d);for(byte e=0;e<8;e++){sa(" ");s3(VccN[d][e]);}}
  }

  
/*
  if(t)
  {
    LcdSet(0,1);
//    n=1125300L/t;//(1100L*1023) //roughly calibrated value    
    sa("V:");s3(t);
    t=0;
  }
*/

/*      Pin2Output(DDRD,3);
      Pin2HIGH(PORTD,3);// start charging timeout capacitor (INT1)
      cli();
      TCNT1=0;
      pin3_interrupt_flag=0;
      attachInterrupt(1, pin3_isr, LOW);
      Pin2LOW(PORTD,3);
     Pin2Input(DDRD,3); // controlled charging (very impurtant set it 2 input (high impedance state))
    sei();
do{
if (pin3_interrupt_flag){t=TCNT1;break;}
}while(1);
sw(t);
*/



  
//  SPCR&=~(1<<SPE); //  SPI.end(); // turn off SPI ????
//  }
//word VccN[8];
//word VccN2[8];

//

//  Serial.println("H");

  //pinMode(2,INPUT);

  //PORTD|=(1<<CErtc);//digitalWrite(CErtc,HIGH); //D7

  //write byte addr
  //XTAL pins are useable!
  // DDRB|=(1<<IOrtc);//(9clocks vs 1)//set bit IOrtc in DDRC//pinMode(IOrtc,OUTPUT);
  //DDRB|=(1<<CLKrtc);//(9clocks vs 1)//set bit IOrtc in DDRC//pinMode(IOrtc,OUTPUT);

  //PORTB|=(1<<IOrtc);//digitalWrite(CErtc,HIGH); 
  //PORTB|=(1<<CLKrtc);//digitalWrite(CErtc,HIGH); 
  //PORTB&=~(1<<IOrtc);//digitalWrite(CErtc,LOW); 
  //PORTB&=~(1<<CLKrtc);//digitalWrite(CErtc,LOW); 


  //pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);pinMode(A2,OUTPUT);pinMode(A3,OUTPUT);



//  Pin2Input(DDRC,5); //pinMode(A5,INPUT);
//  Pin2Input(DDRD,7); //D7 AIN1

  //Pin2HIGH(PORTB,2); //SPI SS pin high


  //analogReference(INTERNAL);// just for analogRead (SetADCinputChannel set it up for mRawADC)
  // setup analog comparator&ADC
//  ADCSRA|=(1<<ADEN); //turn on ADC    
//  DIDR0=(1<<ADC0D);// disable digital input on A0 pin
//  ADCSRB = 0;
//  DIDR1 = (1<<AIN1D); // AIN1 goes to analog comparator negative's input    so switch off digital input (+1.1 bandgap voltage is on positive input)
//  ACSR = (1<<ACBG); //bandgap instead of AIN0  + turn on analog comparator
  //  ACSR&=~(1<<ACD); //turn on analog comparator
//  SetADC(1,8,500);  //  select temperature sensor 352 (need calibration)

  //  SetADCinputChannel(0,500);  //  delay(1); // to avoid false interrupt due bandgap voltage settle

  //  ACSR = (1<<ACI)|(1<<ACBG)|(1<<ACIE)|(1<<ACIS1)|(0<<ACIS0); //bandgap instead of AIN0 int on falling edge 
  //  ACSR = (1<<ACBG)|(1<<ACIE)|(0<<ACIS1)|(0<<ACIS0); //bandgap instead of AIN0 int on toggle
  //  ACSR = (1<<ACBG); //bandgap instead of AIN0 int on toggle

//  mRawADC(Temp,2);
//  mRawADC(Temp,2);

  // SetADC(1,5,500); // select A5

  // mRawADC(t,2);





 // Pin2Output(DDRB,1);//  pinMode(9,OUTPUT);


//  SetADC(1,5,500); // select A5 (1.1v reference)


  //SPI.begin();//  InitSPI();

//  LcdInit();



  // Pin2HIGH(PORTD,0); //vcc to A


//  byte val=0;

  //word bb,aa;
  //  mRawADC(bb,2);//1023 (>1.1v)
  //  mRawADC(bb,2);


  //PORTC=0x7; // select channel 7
  //PORTC=0b0000001; // select channel 1
  //PORTC=0b0000000; // select channel 0
  //   Pin2HIGH(PORTC,3); //digitalWrite(1A3HIGH);//  power to current sensor

  //delay(10);
  //mRawADC(aa,2); // 688 imm 711--718 after 10ms
  //mRawADC(aa,2); // after powering up current sensor 

  /*
// SPI 595
   // digitalWrite (53, LOW);
   Pin2LOW(PORTB,0);//latch
   SPSR = (1 << SPI2X);
   SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first
   // polarity and phase = 0
   // clock = fosc / 2
   // SPDR = 0b10101001;// start transfer
   SPDR = 0b00000001;// start transfer
   while(!(SPSR&(1<<SPIF)));// interrupt also can!
   //SPI.transfer(0b10101001);
   Pin2HIGH(PORTB,0);//latch
   Pin2LOW(PORTB,0);//latch
   */



  //PORTC=0;
  //  cli();TCNT1=0;
//  sa("Тестовая АуРа!"); 
  //sa("Тестовая АуРа!1234567890+-*~=============="); 
  //  t=TCNT1;  sei();

  /*
 if (!card.init(SPI_HALF_SPEED, chipSelect)) {
   sa("SD failed");
   } else {
   sa("SD Ok");
   }
   */

  //.for(int i=0;i<strlen(buf2);i++){sprintf(buf,"%d %d %c %d %d]",i,buf2[i],buf2[i],Rus[(buf2[i]-32)*5],Rus[(buf2[i]-32)*5+1]);sa(buf);}
  //sprintf( buf+strlen(buf), ",%s:%04i", sensorCode, sensorValue );

  //  DDRC=0;// all as inputs  (already)
  //   PORTC=0x1E;//  set pins 0123 HIGH (internal pullups) //digitalWrite(A1,HIGH);digitalWrite(A2,HIGH);digitalWrite(A3,HIGH);digitalWrite(A4,HIGH);
  //  pinMode(A1,INPUT_PULLUP); pinMode(A2,INPUT_PULLUP);  pinMode(A3,INPUT_PULLUP);  pinMode(A4,INPUT_PULLUP);   same same

  //digitalWrite(A1,LOW);digitalWrite(A2,LOW);digitalWrite(A3,LOW);digitalWrite(A4,LOW);

  //delay(1);

  //  val=digitalRead(A0);
  //val=((~PINC)>>1)&0x0F;

  // 0 all are OFF
  // 1 1st is ON
  // 4 3rd is ON
  // C 3&4 are ON
  // F all are ON
/*
  sh(v);
  sa("_");
  //PORTC=0;//digitalWrite(A1,LOW);digitalWrite(A2,LOW);digitalWrite(A3,LOW);digitalWrite(A4,LOW);
  //DDRC=0x1E;//  pinMode(A1,OUTPUT);  pinMode(A2,OUTPUT);  pinMode(A3,OUTPUT);  pinMode(A4,OUTPUT);
  sw(t1111);
  sa(" ");
  sw(freeRam());
  extern int _etext;  //32
  extern int _edata; //8224 -256
  sw(_etext+(_edata-256));
  //  s3(_etext);sw(_edata);
  //  sw(FLASHEND);//32767
  //  sw(RAMEND);//2303
  //  sw(XRAMEND); //2303
  //  sw(E2END); //1023
  sprintf(buf,"  W%d ",WDhappen);
  sa(buf);
  sw(sleeps);
  sa("s ");
  sw(aa);
  sa(" ");

*/


  //cli();TCNT1=0;

  // use PB6&PB7 bits

  //rtcwriteprotect(true);//20us(cannot pair with false)
  //rtcgettime(7);//rtcgettime(8); 7 is enough


  //t=TCNT1;sei();
  //digitalWrite(CE,LOW);
  //PORTD&=~(1<<CErtc);//digitalWrite(CErtc,LOW);

  //PORTD|=(1<<CErtc);//digitalWrite(CErtc,HIGH);
  //rtcpoke(15,0xAC);
  //val=rtcpeek(15);
/*

  sh(buf[2]);
  sh(buf[1]);
  sh(buf[0]);
  sa(" ");
  sh(buf[3]);
  sh(buf[4]);
  sh(0x20);
  sh(buf[6]);
  sh(buf[5]);
  sa(" ");
  sh(val);
  s3(val);

  t=ACSR;
  word a0,a1,a2,a3;
  cli();
  TCNT1=0;
  mRawADC(a0,2);
  val=TCNT1;
  sei();
  mRawADC(a1,2);
  mRawADC(a2,2);
  mRawADC(a3,2);

  //    mRawADC(,1)  807..824   5us but 864..896 800ma
  //    mRawADC(,2)  807..8124  8us 864 800ma stable <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,
  //    mRawADC(,3)  807..818 14us 856..864 800ma



  LcdSet(0,4);
  s3(a0);
  sa(" ");
  s3(a1);
  sa(" ");
  s3(a2);
  sa(" ");
  s3(a3);
  sa("T");
  s3(Temp);
  s3(val);
  sa(" ");
  sh(t);        
  sh(t&(1<<ACO));    
  //    if((t&(1<<ACO))==0){sa(" I too high!");}// if ACO bit is set then the current is withing limits
  if((t&(1<<ACO))==0){
    sa("*");
  }// if ACO bit is set then the current is withing limits

*/
  //vcc read============================================================================


  //   analogReference(DEFAULT);
  //  analogRead(6);
  // ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR); // input (0..7) (default VCC reference)
  // ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(1<<MUX3); // input (0..7) (default VCC reference)
  //   ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(1<<MUX3)|(1<<MUX2)|(1<<MUX1); // input (14) (1.1V)
  //bitSet(ADMUX,3);
  //  delayMicroseconds(250);
  //    bitSet(ADCSRA,ADSC);while(bit_is_set(ADCSRA,ADSC));a1=ADCW;
  //    a2=(1100L*1023)/a1;
  /*
  SetADC(0,14,500);

  for(int e=0;e<10;e++)
  {
    mRawADC(a1,2);

    LcdSet(6,5);

    a2=1103817L/a1;//(1079L*1023) //roughly calibrated value
    sa(" ");
    s3(a1);
    sa(" ");
    sw(a2); //227 4957 5V   230 4892   231 4957   344 3271 3.3v
  }  
  */
  
 
  
  //
// for(int i=0;i<10;i++){
  
  //digitalWrite(5,LOW);
  
  
//  for(int i=0;i<100;i++) {
//  cli();

//Pin2LOW(PORTD,6);//digitalWrite(LATCHPIN,LOW);///already low
Pin2HIGH(PORTD,7);//digitalWrite(SRCLR,HIGH); // can write to 595 (it is cleared now)
// shift register is cleared
// what is in storage register we don't care because G is HIGH and all outputs are OFF

/*
digitalWrite(DATAPIN,HIGH);
//digitalWrite(DATAPIN,LOW);

digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);

//digitalWrite(DATAPIN,LOW);

digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
*/
//  shiftOut(DATAPIN,CLOCKPIN,MSBFIRST,0b11111111);


Pin2HIGH(PORTB,1);//DATAPIN to HIGH

//t=0;

//  if((z&7)==0){Pin2HIGH(PORTB,1);} // 1st bit is "1"
//  Pin2HIGH(PORTB,1); // 1st bit is "1"


for(byte z=0;z<8;z++)// serie of flashes
{
//  if((z&7)==0){Pin2HIGH(PORTB,1);} // 1st bit is "1"
  Pin2HIGH(PORTB,0);Pin2LOW(PORTB,0); // clock pulse 
//  if((z&7)==0){Pin2LOW(PORTB,1);} // next 7 are zeroes
  Pin2LOW(PORTB,1); // next 7 are zeroes
// latch
  Pin2HIGH(PORTD,6);//digitalWrite(LATCHPIN,HIGH);
  Pin2LOW(PORTD,6);//digitalWrite(LATCHPIN,LOW);
  
//  if((it==0x0200)&&(z==0)){SetADC(0,14,100);}
  
//flash itself
//for(byte x=0;x<3;x++)// flashes
//{
//Pin2LOW(PORTD,5);delayMicroseconds(30);Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // start/stop light
//}

if(it==1021)
{
  ADCSRA|=(1<<ADEN);  // start ADC
  ADMUX=(0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|14;// input 14 (Vcc) internalRef Vcc
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;// start 1st conversion
  
/*
#define SetADC(bandgap,input,us){ ADCSRA|=(1<<ADEN);delayMicroseconds(2);ADMUX=(bandgap<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input;delayMicroseconds((us>>1));} // input (0..7,8,14) (bg/vcc analogReference )
#define ADCoff{ ADCSRA&=~(1<<ADEN); }
#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 
*/

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

  if(it==15000)
{
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

}
//lvv++;
  
cli();
//enter critical section
//Pin2Input(DDRD,0); // important set it 2 input (high impedance state)
Pin2HIGH(PORTD,0);// open reset mosfet
//TCNT1=0;
Pin2LOW(PORTD,5);// start lighting
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
delayMicroseconds(9);// 18 us
//}
//}//30us



//if(it==6000){delayMicroseconds(400);} // ~800us

Pin2HIGH(PORTD,5);//digitalWrite(G,HIGH); // start/stop light
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

Pin2LOW(PORTD,0);// close reset mosfet
}//if

sei();

  if(it==15000)
  {
  
//  delay(7); // flash time
// lvv = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
//  lvv <<= 16;
//  lvv |= read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);

//  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);  //  disable();//  tsl.disable();


}

//NOP;//sei();delay(2000);cli();
//if((it==0x0200)&&(z<8)){    mRawADC(VccN[z],2);    mRawADC(VccN2[z],2);}
//else
//{
//Pin2LOW(PORTD,0);// close reset mosfet

//}

//nap();
  //if((it==0x0200)&&(z==7)){ADCoff;t=1;}

}// for

if(it==1022){ADCSRA&=~(1<<ADEN); // stop ADC
}
/*
if(t)
{
  // show 
  LcdInit();
    LcdSet(0,2);
    sa("VN:");
    for(byte z=0;z<8;z++)
    {
//    n=1125300L/t;//(1100L*1023) //roughly calibrated value    
    s3(VccN[z]);sa(" ");
    }
    LcdSet(0,4);
    sa("V2:");
    for(byte z=0;z<8;z++)
    {
//    n=1125300L/t;//(1100L*1023) //roughly calibrated value    
    s3(VccN2[z]);sa(" ");
    }
    
    
  SPCR&=~(1<<SPE); //  SPI.end(); // turn off SPI ????
    t=0;
}*/


// flash duration
//0 - 1028 ~1us
//1 - 1400
//2 - 1683
//3 - 2070
//4 - 2340
//5 - 2610
//6 - 2880
//7 - 3130
//8 - 3380 ~2us

//16 - 5190 ~3us
//32 - 8450 ~5us

//Pin2LOW(PORTD,6);//digitalWrite(LATCHPIN,LOW);
//Pin2LOW(PORTB,1);//digitalWrite(DATAPIN,LOW);
//Pin2LOW(PORTB,0);//digitalWrite(CLOCKPIN,LOW);

/*
Pin2LOW(PORTB,0);//digitalWrite(CLOCKPIN,LOW);
Pin2HIGH(PORTB,0);//digitalWrite(CLOCKPIN,HIGH);
Pin2LOW(PORTB,0);//digitalWrite(CLOCKPIN,LOW);
Pin2HIGH(PORTB,0);//digitalWrite(CLOCKPIN,HIGH);
Pin2LOW(PORTB,0);//digitalWrite(CLOCKPIN,LOW);
Pin2HIGH(PORTB,0);//digitalWrite(CLOCKPIN,HIGH);
Pin2LOW(PORTB,0);//digitalWrite(CLOCKPIN,LOW);
Pin2HIGH(PORTB,0);//digitalWrite(CLOCKPIN,HIGH);

Pin2LOW(PORTB,0);//digitalWrite(CLOCKPIN,LOW);
Pin2HIGH(PORTB,0);//digitalWrite(CLOCKPIN,HIGH);
Pin2LOW(PORTB,0);//digitalWrite(CLOCKPIN,LOW);
Pin2HIGH(PORTB,0);//digitalWrite(CLOCKPIN,HIGH);
Pin2LOW(PORTB,0);//digitalWrite(CLOCKPIN,LOW);
Pin2HIGH(PORTB,0);//digitalWrite(CLOCKPIN,HIGH);
Pin2LOW(PORTB,0);//digitalWrite(CLOCKPIN,LOW);
Pin2HIGH(PORTB,0);//digitalWrite(CLOCKPIN,HIGH);
*/
//  shiftOut(DATAPIN,CLOCKPIN,MSBFIRST,0b00000000);
    
  //Pin2HIGH(PORTD,6);//digitalWrite(LATCHPIN,HIGH);

//Pin2LOW(PORTB,6);//power supply OFF  was tpic595
//delayMicroseconds(100);
//sei();
/*
digitalWrite(LATCHPIN,LOW);

digitalWrite(DATAPIN,LOW);

digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);

//digitalWrite(DATAPIN,LOW);

digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);
digitalWrite(CLOCKPIN,LOW);digitalWrite(CLOCKPIN,HIGH);

//  shiftOut(DATAPIN,CLOCKPIN,MSBFIRST,0b11111111);
digitalWrite(LATCHPIN,HIGH);
*/
//digitalWrite(LATCHPIN,LOW);
//  shiftOut(DATAPIN,CLOCKPIN,MSBFIRST,0b00000000);
//digitalWrite(LATCHPIN,HIGH);

//  pinMode(5,INPUT);
// }

//  }//for
//sa("xxx");

 //delay(2000);

  /*
//  NOP;
   //  v=TCNT2;
   //NOP;  
   do{
   __asm__ __volatile__ (
   //  "lds r16,0x00B2\n\t"
   //
   "ldi r25,0\n\t" // off
   "ldi r24,0b0000010\n\t" // bit mask
   //"in r25,5\n\t" // read portb
   //"mov r23,r25\n\t"// r23 OFF mask
   //"or r25,r24\n\t" // ON mask
   //"ldi r22,0\n\t"  // 256 cycles
   "ldi r22,255\n\t"  // 255 cycles
   "cli\n\t"
   "1:\n\t"
   
   "out 5,r25\n\t"// ON
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
   "nop\n\t"
   "nop\n\t"
   
   "out 5,r23\n\t"// OFF
   
   "dec r22\n\t"// +1
   "brne 1b\n\t"// +3
   
   //  "lds r17,0x00B2\n\t" // TCNT2
   //  "lds r16,0x00B2\n\t" // TCNT2
   // "sts v,r16\n\t"
   //"sts v2,r17\n\t"
   
   "sei\n\t"
   ::"r"(idx));
   }while(1);
   // 1x1 24 pairs ON/OFF x256 (6144)
   
   */


  //++n;
  //  sei();
  //  flash_duration=100;//6.25us
  //  flash_duration=400;//25us
  //  flash_duration=500;//31us  //don't go beyond 65500:  give it some clocks: close to 65535 has bug (runs forever)
  // flash_start_mask=0b11000001;
  // flash_start_mask=0b11000010;
  // flash_stop_mask =0b00000111;// park 4051 to unused channel
  // flash();


  // 1/8 250ns 50%    0.3A
  /*
cli();
   for(int k=0;k<10000;k++)
   {
   TCNT2=0;
   Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
   //  NOP;  
   // NOP;  
   //{}while(TCNT2<1);
   
   //TCNT2=0;
   {}while(TCNT2<8); // cycle+tcnt2=0 (4)
   Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
   }
   sei();
   */
  /*
  LcdSet(15,5);sa("^");
   
   // 1/8 250ns 50%   0.7A
   for(int k=0;k<10000;k++)
   {
   cli();
   TCNT2=0;
   Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
   NOP;  
   //{}while(TCNT2<1);
   Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
   //TCNT2=0;
   sei();
   {}while(TCNT2<8);
   }
   LcdSet(15,5);sa("0");
   
   // 2/4 250ns 50%
   for(int k=0;k<20000;k++)
   {
   cli();
   TCNT2=0;
   Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
   {}while(TCNT2<2);
   Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
   //TCNT2=0;
   sei();
   {}while(TCNT2<4);
   }
   
   LcdSet(15,5);sa("1");
   
   // 2/8 500ns 25% 360mA
   for(int k=0;k<25000;k++)
   {
   cli();
   TCNT2=0;
   Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
   {}while(TCNT2<2);
   Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
   //TCNT2=0;
   sei();
   {}while(TCNT2<8);
   }
   
   LcdSet(15,5);sa("2");
   
   
   // 8x8 1us 50% 840mA
   for(int k=0;k<15000;k++)
   {
   cli();
   TCNT2=0;
   Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
   {}while(TCNT2<8);
   Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
   //TCNT2=0;
   sei();
   {}while(TCNT2<16);
   }
   
   LcdSet(15,5);sa("3");*/

  //#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 

/*
  // 100% ~1.7A
  sei();

  word t1,t2,v1,v2,v3,v4,v5;
  long nn=0;

  word tq[10];


  do{

    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
    do{}while(bit_is_set(ADCSRA,ADSC));
    v1=ADCW;
    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
    do{}while(bit_is_set(ADCSRA,ADSC));
    v2=ADCW; 

    if (v2>237){LcdSet(0,5);sa("LOW BATTERY!!!");delay(2000);break;}

    for(long jj=0;jj<100000;jj++){  
      cli();

//Flash_B(0b10000011); // PB0,PB1,PB7
//TCNT2=0;
//__asm__ __volatile__ ("ldi r24,0b10000000\n\t""call FlashB\n\t");

//69us

Flash_B(0b01000000); // PB6
//tq[0]=ADCSRA;
      tq[0]=ADCW; 
//tqq=TCNT2;
//__asm__ __volatile__ ("ldi r24,0b00000010\n\t""call FlashB\n\t");
Flash_B(0b00000010); // PB1
//tq[1]=ADCSRA;
      tq[1]=ADCW; 
//__asm__ __volatile__ ("ldi r24,0b00000001\n\t""call FlashB\n\t");
Flash_B(0b00000001); // PB0
      tq[2]=ADCW; 
//Flash_C(0b00111100); // PC2,PC3,PC4,PC5
//Flash_C(0b00001100); // PC2&PC3
  //    tq[3]=ADCW; 

Flash_C(0b00000100); // PC2
      tq[3]=ADCW; 
Flash_C(0b00001000); // PC3
      tq[4]=ADCW; 
Flash_C(0b00010000); // PC4
      tq[5]=ADCW; 
Flash_C(0b00100000); // PC5
      tq[6]=ADCW; 
//Flash_D(0b11100000); // PD5,PD6,PD7
Flash_D(0b00100000); // PD5
      tq[7]=ADCW; 
Flash_D(0b01000000); // PD6
      tq[8]=ADCW; 
Flash_D(0b10000000); // PD7
      tq[9]=ADCW; 
sei();
//NOP;
//      for(long j=0;j<10;j++){
  //      pinmask=0b00000000;
        
    //    if(j==0){pinmask=0b00000010;}
      //  else if(j==1){pinmask=0b00000100;}
//NOP;
//NOP;
//NOP;
        //    Pin2Output(DDRD,0);Pin2HIGH(PORTD,0);  delayMicroseconds(20);         
        */
//        for each port it's own version
  /*      
__asm__ __volatile__(
"lds r20,pinmask \n\t"
"lds r21,0xC2 \n\t"
"lds r22,0x00 \n\t"// r1
"out 0x5,r20\n\t"// pin 2 high on portb
//        ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2; //0xC2
"sts 0x007A,r21\n\t"
"1:\n\t"
"lds r24,0x007A\n\t"
"sbrc r24,6\n\t"
"rjmp 1b\n\t"
"out 0x5,r22\n\t"// pin 2 low on portb (all portb actually)

//"lds r24,0x0078\n\t"
//"lds r25,0x0079\n\t"
);*/
        //Pin2LOW(PORTD,0);

        //    TCNT1=0;
        //  TCNT2=0;


//bit_set(PORTB,1);
//PORTB=pinmask;

//Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
//ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|1;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   // NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //    delayMicroseconds(1);  
        // Pin2LOW(PORTB,1); //digitalWrite(9,LOW//   

//do{}while(bit_is_set(ADCSRA,ADSC));

        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   
        //    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        //    delayMicroseconds(10);  
        //  Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
        //   ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

        // do{}while(bit_is_set(ADCSRA,ADSC));
        // v3=ADCW; 


        //    delayMicroseconds(1);  


//PORTB=0;
//Pin2LOW(PORTB,1); //digitalWrite(9,LOW//

        //Pin2Input(DDRD,0);Pin2HIGH(PORTD,0);

  //      tq[j]=ADCW; 


        //    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //  do{}while(bit_is_set(ADCSRA,ADSC));
        //v3=ADCW; 
        //   ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        // do{}while(bit_is_set(ADCSRA,ADSC));
        //  v4=ADCW; 
        //  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
        //  do{}while(bit_is_set(ADCSRA,ADSC));
        //  v5=ADCW; 

        //    t2=TCNT2;
        //t1=TCNT1;

        // 1us   
        // NOP;    NOP;    NOP;    NOP;    //NOP;    NOP;    NOP;    NOP;   

/*
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;    
        NOP;
*/        //   NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
        //    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;



        //    delayMicroseconds(1000);          //???
        //delayMicroseconds(100);  // give batteryies some time for recovery (fire each lamp at 1/10 interval)
        //delayMicroseconds(1);  // give batteryies some time for recovery (fire each lamp at 1/10 interval)
  //    }//for 10
      //sei();

      //Pin2Output(DDRD,0);
      //Pin2HIGH(PORTD,0);

      //    delayMicroseconds(25);         

      //Pin2HIGH(PORTD,0);//Pin2Input(DDRD,0);

      //Pin2Output(DDRD,0);Pin2HIGH(PORTD,0);

      //  delayMicroseconds(10);         
      //  pinMode(0,INPUT);
      //pinMode(0,OUTPUT);

      //Pin2LOW(PORTD,0);


      //    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
      //    do{}while(bit_is_set(ADCSRA,ADSC));
      //   v=ADCW;
      //  delayMicroseconds(1);         


      //Pin2Input(DDRD,0);Pin2HIGH(PORTD,0);

      //  pinMode(0,INPUT_PULLUP); // too long...

      //Pin2HIGH(PORTD,0);

      //    delayMicroseconds(900);          //deep sleep

      //pinMode(2,OUTPUT);digitalWrite(2,HIGH);delayMicroseconds(65);digitalWrite(2,LOW);pinMode(2,INPUT);// controlled charging(~100us)






  //17211..17226 ~120clocks each 17ms sleep inaccuracy
  /*cli();  // disable all interrupts
   TCNT1=0; //17202..17263x8 17249.5 137996 clocks with 8000000 17.2ms
   wdt_reset(); // reset the WDT timer
   MCUSR &= ~(1<<WDRF);  // because the data sheet said to
   // Enter Watchdog Configuration mode:
   WDTCSR = (1<<WDCE) | (1<<WDE);
   // Set Watchdog settings: interrupte enable, 0110 for timer
   //   WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//15ms
   WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s
   WDhappen=0;
   sei();
   */

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

if((it&0xFFFF)==0) // measure nap time
{
      set_sleep_mode (SLEEP_MODE_IDLE);//// in r24,0x33// andi r24,0xF1// out 0x33,r24
TCNT1=0;
      fastnap();
fnt=TCNT1;      
//      set_sleep_mode (SLEEP_MODE_IDLE);//// in r24,0x33// andi r24,0xF1// out 0x33,r24
//TCNT1=0;
//      longnap();
//lnt=TCNT1;      
}
else
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //// in r24,0x33// andi r24,0xF1// ori r24,0x04// out 0x33,r24
if(it==55555)// ultra long nap 8s once  in somewhere  2 minutes
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


Pin2Output(DDRD,0);// sreset mosfet 2n7000 control
Pin2Output(DDRD,1);// 3.3/5v mosfet 2n7000 control
Pin2Output(DDRD,2);// INT0 line

Pin2Output(DDRD,3);// INT1 line /TFT CS
Pin2Output(DDRD,4);// TFT DC
Pin2Output(DDRB,3);// MOSI
Pin2Output(DDRB,5);// CLK




Pin2Output(DDRB,0); // CLOCKPIN 8
Pin2Output(DDRB,1); // DATAPIN 9
Pin2Output(DDRB,2);
Pin2Output(DDRB,6);


//Pin2Output(DDRC,2);
//Pin2Output(DDRC,3);
//Pin2Output(DDRC,4);
//Pin2Output(DDRC,5);

Pin2Output(DDRD,5); // pin 5 G
Pin2Output(DDRD,6); // pin 6 LATCH
Pin2Output(DDRD,7); // pin 7 SRCLR



  pinMode(DATAPIN,OUTPUT);
  pinMode(CLOCKPIN,OUTPUT);
  pinMode(LATCHPIN,OUTPUT);

//NOP;
it++;

  //delay(1000);               // wait for a second
}


