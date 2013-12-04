#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include <DS1302.h>
#include <SPI.h>
#include "AyPa_fonts.h"
  

// nokia 5110 pins

#define RST 6
#define CE 7
#define DC 5
#define DIN 11
#define CLK 13



//LiquidCrystalFast lcd(2,3,4,A1,A2,A3,A4);//4bit mode
//DS1302 rtc(6,7,8);//ce data clk
//DS1302_RAM ramBuffer;


/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
//int led = 9;

#define NOP __asm__ __volatile__ ("nop\n\t")
#define wait1us NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP; // 1 microsecond delay on a 20MHz Arduino
//#define wait500ns NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP; // 500ns delay on a 16MHz Arduino
//#define wait250ns NOP;NOP;NOP;NOP; // 250ns delay on a 16MHz Arduino
//#define wait125ns NOP;NOP; // 125ns delay on a 16MHz Arduino
//#define wait63ns NOP; // 63ns delay on a 16MHz Arduino


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

void LcdSet(byte x,byte y)
{
  LcdWriteCmd(0b10000000|x*5);//set X (0..83)
  LcdWriteCmd(0b01000000|y);//set Y (0..5)
}

// 7774 clocks original
// 7637
// 6624
// 1100
// 1057
// 952
void sa(char *st)
{
  byte i=0;
  byte ch;
  
  do{
//    LcdWriteData(0);//space  (start with it - while it is sending can calc address)
  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);

SPDR = 0;// start transfer with space (while it is sending can calc address)
//calcs
    ch=(st[i++]-32)*5;

while(!(SPSR&(1<<SPIF)));

//  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  

//-----------------------------------------------------
//  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
//  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);
//    LcdWriteData(Rus[ch]);
SPDR = Rus[ch++];
while(!(SPSR&(1<<SPIF)));
//  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  
//----------------------------------------------------  
//  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
//  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);
//    LcdWriteData(Rus[ch+1]);
SPDR = Rus[ch++];
while(!(SPSR&(1<<SPIF)));
//  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  
//----------------------------------------------------  
//  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
//  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);
//    LcdWriteData(Rus[ch+2]);
SPDR = Rus[ch++];
while(!(SPSR&(1<<SPIF)));
//  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  
//----------------------------------------------------  
//  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
//  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);
//    LcdWriteData(Rus[ch+3]);
SPDR = Rus[ch++];
while(!(SPSR&(1<<SPIF)));
//  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  
//----------------------------------------------------  
//  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
//  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);
//    LcdWriteData(Rus[ch+4]);
SPDR = Rus[ch];
while(!(SPSR&(1<<SPIF)));
  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  
//----------------------------------------------------  

  }while (st[i]!=0);
  
}

void SendStr(char *st)
{
  byte i=0;
  byte ch;
  
  do{
    ch=(st[i++]-32)*5;
    LcdWriteData(~Rus[ch]);
    LcdWriteData(~Rus[ch+1]);
    LcdWriteData(~Rus[ch+2]);
    LcdWriteData(~Rus[ch+3]);
    LcdWriteData(~Rus[ch+4]);
    LcdWriteData(0xff);//space  
  }while (st[i]!=0);
  
}

void SendChar(byte ch)
{
LcdWriteData(0);//space
LcdWriteData(Rus[ch*5]);
LcdWriteData(Rus[ch*5+1]);
LcdWriteData(Rus[ch*5+2]);
LcdWriteData(Rus[ch*5+3]);
LcdWriteData(Rus[ch*5+4]);
}

void LcdWriteCmd(byte cmd)
{
//SPI.setDataMode(SPI_MODE0);//default
//SPI.setBitOrder(MSBFIRST);// maybe
//SPI.setClockDivider(SPI_CLOCK_DIV2);//max
SPSR = (1 << SPI2X);//2
//SPSR = (0 << SPI2X); //4
//SPCR = (1 << MSTR) | (1 << SPE) |(1<<SPR0);      // enable, master, msb first
SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first

  digitalWrite(DC,LOW);
  digitalWrite(CE,LOW);

SPDR = cmd;// start transfer
while(!(SPSR&(1<<SPIF)));// interrupt also can!
  
  digitalWrite(CE,HIGH);  
}

void LcdWriteData(byte cmd)
{
//SPI.setDataMode(SPI_MODE0);//default
//SPI.setBitOrder(MSBFIRST);// maybe
//SPI.setClockDivider(SPI_CLOCK_DIV2);//max
SPSR = (1 << SPI2X);//2
//SPSR = (0 << SPI2X); //4
//SPCR = (1 << MSTR) | (1 << SPE) |(1<<SPR0);      // enable, master, msb first
SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first
  
  digitalWrite(DC,HIGH);
  digitalWrite(CE,LOW);

SPDR = cmd;// start transfer
while(!(SPSR&(1<<SPIF)));// interrupt also can!

  digitalWrite(CE,HIGH);  
}

void LcdWriteCmdold(byte cmd)
{
  digitalWrite(DC,LOW);
  digitalWrite(CE,LOW);
  shiftOut(DIN,CLK,MSBFIRST,cmd);
  digitalWrite(CE,HIGH);  
}

void LcdWriteDataold(byte cmd)
{
  digitalWrite(DC,HIGH);
  digitalWrite(CE,LOW);
  shiftOut(DIN,CLK,MSBFIRST,cmd);
  digitalWrite(CE,HIGH);  
}

unsigned long resetTime = 0;
#define TIMEOUTPERIOD 100             // You can make this time as long as you want,
                                       // it's not limited to 8 seconds like the normal
                                       // watchdog
#define doggieTickle() resetTime = millis();  // This macro will reset the timer
void(* resetFunc) (void) = 0; //declare reset function @ address 0

void watchdogSetup()
{
cli();  // disable all interrupts
wdt_reset(); // reset the WDT timer
MCUSR &= ~(1<<WDRF);  // because the data sheet said to
/*
WDTCSR configuration:
WDIE = 1 :Interrupt Enable
WDE = 1  :Reset Enable - I won't be using this on the 2560
WDP3 = 0 :For 1000ms Time-out
WDP2 = 1 :bit pattern is 
WDP1 = 1 :0110  change this for a different
WDP0 = 0 :timeout period.
*/
// Enter Watchdog Configuration mode:
WDTCSR = (1<<WDCE) | (1<<WDE);
// Set Watchdog settings: interrupte enable, 0110 for timer
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//15ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//30ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (0<<WDP0);//60ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0);//120ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0);//240ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);//480ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);//960ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);//2s
//WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//4s
WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s
sei();
//Serial.println("finished watchdog setup");  // just here for testing
}

//byte odd=0;
long r1=0,r2;

ISR(WDT_vect) // Watchdog timer interrupt.
{ 
//resetFunc();  //reboot
r2=TCNT1;
//if(odd){odd=0;r1=TCNT1;}
//else{odd=1;r2=TCNT1;}
 // if(millis() - resetTime > TIMEOUTPERIOD){
//    Serial.println("This is where it would have rebooted");  // just here for testing
//    lcd.print("R");  // just here for testing
  //  doggieTickle();                                          // take these lines out
//  resetFunc();     // This will call location zero and cause a reboot.
//  }
  //else                                                       // these lines should
//    lcd.print("H");                                 // be removed also
}

uint16_t ticks=0;
uint8_t seconds=0;
uint8_t minutes=0;
uint8_t hours=0;
uint8_t days=0;

ISR (TIMER1_COMPA_vect){ticks++;
if(ticks==500){ticks=0;seconds++;
if(seconds==60){seconds=0;minutes++;
if(minutes==60){minutes=0;hours++;
if(hours==24){hours=0;days++;}}}}}


void SetADCinputChannel(uint8_t input,uint16_t us)
{
      ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input; // input (0..7)
    ADCSRB = 0; // mux5 bit
//    if(stime) delay(stime); // Wait for input channel to settle
    if(us)delayMicroseconds(us); // Wait for input channel to settle (300us)

}
//#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
//  pinMode(led, OUTPUT);  
//  digitalWrite(A5,HIGH); pullup resistor
 // pinMode(9, OUTPUT); //ACS712 module
  
   // digitalWrite(9,HIGH);// lcd+acs712 

  //lcd.begin(20,4);
  
 // pinMode(A5, OUTPUT); //LCD led backlight
//  digitalWrite(A5,HIGH);



  //watchdogSetup();


//setup timer 1
cli();
TCCR1A=0x00;
TCCR1B=(1 << WGM12)|(0<<CS22)|(0<<CS21)|(1<<CS20); // /no prescaler;
TCCR1B=(1 << WGM12)|(0<<CS22)|(1<<CS21)|(0<<CS20); // /8;
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x9C;
OCR1AL=0x40; // 40000
//OCR1AH=0x4E;
//OCR1AL=0x20; // 20000
//OCR1AH=0xFF;
//OCR1AL=0xFF;

//  TCCR1B |= (1 << WGM12);
//  TCCR1B |= (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  
sei();



/*
	//======define charset
	uint8_t bell[8] = {0x4,0xe,0xe,0xe,0x1f,0x0,0x4};
	uint8_t clock[8] = {0x0,0xe,0x15,0x17,0x11,0xe,0x0};
	uint8_t heart[8] = {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0};
	uint8_t duck[8] = {0x0,0xc,0x1d,0xf,0xf,0x6,0x0};
	uint8_t check[8] = {0x0,0x1,0x3,0x16,0x1c,0x8,0x0};
	uint8_t cross[8] = {0x0,0x1b,0xe,0x4,0xe,0x1b,0x0};
	uint8_t retarrow[8] = { 0x1,0x1,0x5,0x9,0x1f,0x8,0x4};
	uint8_t box[8] = { 0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f};
		
	lcd.createChar(0, bell);
	lcd.createChar(1, box);
	lcd.createChar(2, clock);
	lcd.createChar(3, heart);
	lcd.createChar(4, duck);
	lcd.createChar(5, check);
	lcd.createChar(6, cross);
	lcd.createChar(7, retarrow);

	lcd.setCursor(0,1);for (int j=0; j<8; j++) {lcd.write(j);}
	delay(2000);*/

//  Serial.begin(9600);
//  Serial.print(",dsndvmdjkvhj");

//pinMode(7,OUTPUT);
//pinMode(7,INPUT);
//digitalWrite(7,LOW);
  // setup analog comparator
  cli();
  ADCSRB = 0;
  ACSR = (1<<ACBG); //bandgap instead of AIN0 int on falling edge 
//  delay(1); // to avoid false interrupt due bandgap voltage settle

//  ACSR = (1<<ACI)|(1<<ACBG)|(1<<ACIE)|(1<<ACIS1)|(0<<ACIS0); //bandgap instead of AIN0 int on falling edge 
//  ACSR = (1<<ACBG)|(1<<ACIE)|(0<<ACIS1)|(0<<ACIS0); //bandgap instead of AIN0 int on toggle
//  ACSR = (1<<ACBG); //bandgap instead of AIN0 int on toggle
  DIDR1 = (1<<AIN0D); // it will be bandgap on AIN0  
  sei();


//TIMSK0 &= ~_BV(TOIE0); // disable timer0 overflow interrupt
// when it is not needed

/*
  pinMode(6,OUTPUT);pinMode(7,OUTPUT);pinMode(8,OUTPUT);

  rtc.halt(false);
  rtc.writeProtect(false);
  rtc.setDOW(SATURDAY);        // Set Day-of-Week to FRIDAY
  rtc.setTime(15, 04, 0);     // Set the time to 12:00:00 (24hr format)
  rtc.setDate(30, 11, 2013);   // Set the date to August 6th, 2010
//  rtc.writeProtect(true);
*/
}

word sc[16];
word mn=5555,mx=5000;

char buf[128];
uint16_t t1,t2,tt1,tt2,ttt1,ttt2;
// the loop routine runs over and over again forever:
void loop() {

  pinMode(9,OUTPUT);
  //pinMode(10,OUTPUT);
  
    digitalWrite(9,HIGH);// acs712 module + lcd
  //  delay(1000);
 //   digitalWrite(10,HIGH);// lcd

  SPI.begin();

 pinMode(RST,OUTPUT);
 pinMode(CE,OUTPUT);
 pinMode(DC,OUTPUT);
 pinMode(DIN,OUTPUT);
 pinMode(CLK,OUTPUT);
 digitalWrite(RST,LOW);
 digitalWrite(RST,HIGH);

LcdWriteCmd(0x21);
LcdWriteCmd(0xB8);
LcdWriteCmd(0x07);
LcdWriteCmd(0x14);//bias(best)
//LcdWriteCmd(0x20); // extended instruction set control (H=0)
//LcdWriteCmd(0x09); // all display segments on
//LcdWriteCmd(0x20); // extended instruction set control (H=0)
//LcdWriteCmd(0x08); // all segments off - need to clear all ram on start
//LcdWriteCmd(0x20);
//LcdWriteCmd(0x0D); // invert
LcdWriteCmd(0x20); // extended instruction set control (H=0)
LcdWriteCmd(0x0c); // LCD in normal mode (0x0d = inverse mode)

LcdSet(0,0);for(byte i=0;i<84;i++){SendChar(0);} // clear ram manually

cli();
TCNT1=0;
sa("MARINOCHKA!!!a1234567890+-*~=============="); //ascii
int t=TCNT1;
sei();

//sprintf( buf+strlen(buf), ",%s:%04i", sensorCode, sensorValue );
sprintf(buf,"TCNT1=%d",t);
sa(buf);

  delay(7000);


//for(byte i=0;i<84;i++){SendChar(i);}
  //delay(7000);
//for(byte i=84;i<168;i++){SendChar(i);}
  //delay(7000);

//for(byte i=0;i<84;i++){SendStr(ord(i));}
  //delay(7000);
//sprintf( buf+strlen(buf), ",%s:%04i", sensorCode, sensorValue );
//SendStr("MARINOCHKA!!!a1234567890");
  //delay(7000);

 digitalWrite(RST,LOW);
// digitalWrite(RST,HIGH);
 

//lcd
 pinMode(A0,INPUT);
  pinMode(6,OUTPUT);pinMode(7,OUTPUT);pinMode(8,OUTPUT);//rtc
 
  pinMode(A1,OUTPUT);pinMode(A2,OUTPUT);pinMode(A3,OUTPUT);pinMode(A4,OUTPUT);pinMode(2,OUTPUT);pinMode(3,OUTPUT);pinMode(4,OUTPUT);
 // lcd.begin(20,4);  lcd.setCursor(0,0); //lcd.print("Marinochka lapochka!"); delay(1000);

  SetADCinputChannel(0,500);
  word i;mRawADC(i,2);
  //digitalWrite(A5,HIGH);
  



//  lcd.clear();

//rtc

 // for (int i=0; i<31; i++)ramBuffer.cell[i]=i;

//  comment("Writing buffer to RAM...");
 // rtc.writeBuffer(ramBuffer);
 // bufferDump();
  
//    comment("Setting byte 15 (0x0F) to value 160 (0xA0)...");
 // rtc.poke(15,160);

 // ramBuffer=rtc.readBuffer();
 // bufferDump();

//  comment("Reading address 18 (0x12). This should return 18, 0x12.");
 // r1 = rtc.peek(18);
 // r2 = rtc.peek(15);


   // Send Day-of-Week
/*  lcd.print(rtc.getDOWStr());
  lcd.print(" ");
  
  // Send date
  lcd.print(rtc.getDateStr());
  lcd.print(" -- ");

  // Send time
  lcd.println(rtc.getTimeStr());
  
  // Wait one second before repeating :)
  delay (3000);
*/

   // Serial.print("Hi! ");delay(100);

 // digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
 // lcd.setCursor(0,2); lcd.print("HIGH!");


for(byte i=0;i<12;i++){
  ttt1=millis();
  cli();tt1=ticks;t1=TCNT1;sei();//atomic read
  /*
  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;
  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;
  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;
  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;
  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;
  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;
  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;
  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;
  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;
  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;  wait1us;*/
  //t1=TCNT1;delayMicroseconds(1);t2=TCNT1;
  //lcd.print(t2-t1);
 
 //delayMicroseconds(100);
 //wait1us;
//   delay(100);
 
//  delay(1000);
  cli();
  t2=TCNT1;tt2=ticks;sei();
  ttt2=millis();
//  delay(1000);
//  lcd.print(" ");lcd.print(ttt2-ttt1);lcd.print(" ");lcd.print(tt2-tt1);lcd.print(" ");lcd.print(t2-t1-4); 
wdt_reset();
//  lcd.clear();
}
  //delay(10000);               // wait for a second
  
//  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
 // lcd.setCursor(0,3); lcd.print("LOW!");
cli();
t1=TCNT1;
//for(int j=0;j<16;j++){sc[j]=analogRead(0);}
for(int j=0;j<16;j++){mRawADC(sc[j],2);}
t2=TCNT1;
sei();

//lcd.setCursor(0,0);
long z=0;

//for(int j=0;j<16;j++){z+=sc[j]*10;
//lcd.print(" ");lcd.print(sc[j]);}

/*
//17211..17226 ~120clocks each 17ms sleep inaccuracy
cli();  // disable all interrupts
TCNT1=0; //17202..17263x8 17249.5 137996 clocks with 8000000 17.2ms
wdt_reset(); // reset the WDT timer
MCUSR &= ~(1<<WDRF);  // because the data sheet said to
// Enter Watchdog Configuration mode:
WDTCSR = (1<<WDCE) | (1<<WDE);
// Set Watchdog settings: interrupte enable, 0110 for timer
WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//15ms
//WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s
sei();
delay(24);*/
/*
lcd.setCursor(0,0);
 // lcd.print(" d");lcd.print(days);lcd.print("h");lcd.print(hours);lcd.print(":");lcd.print(minutes);lcd.print(":");lcd.print(seconds);

lcd.print("[");
//lcd.print(z);lcd.print(" ");
z=z>>4;// /16
if(z<mn){mn=z;}if(z>mx){mx=z;}
lcd.print(z);
lcd.print(" ");
lcd.print(mn);
lcd.print(" ");
lcd.print(mx);
lcd.print(" ");
lcd.print(t2-t1);
lcd.print(" r1 ");
lcd.print(r1);
lcd.print(" ");
lcd.print(r2);

lcd.print("]");*/

for(long i=0;i<3000000;i++){NOP;}//~2100ms delay
//  pinMode(A5, INPUT); //LCD led backlight
//digitalWrite(A5,LOW);
//  pinMode(A5, OUTPUT); //LCD led backlight
//-------------------------------------------------------------------------------------------------[ power saving wait state ]
SPI.end();

ADCSRA=0;// switch off ADC
ACSR = (1<<ACD); // switch off analog comparator
//digitalWrite(10,LOW);// lcd 
digitalWrite(9,LOW);// acs712 module 
//digitalWrite(A5,LOW);// LCD display backlight off
pinMode(A1,INPUT);pinMode(A2,INPUT);pinMode(A3,INPUT);pinMode(A4,INPUT);pinMode(2,INPUT);pinMode(3,INPUT);pinMode(4,INPUT);
pinMode(9,INPUT);pinMode(10,INPUT);pinMode(6,INPUT);pinMode(7,INPUT);pinMode(8,INPUT);




cli();  // disable all interrupts
wdt_reset(); // reset the WDT timer
MCUSR &= ~(1<<WDRF);  // because the data sheet said to
/*
WDTCSR configuration:
WDIE = 1 :Interrupt Enable
WDE = 1  :Reset Enable - I won't be using this on the 2560
WDP3 = 0 :For 1000ms Time-out
WDP2 = 1 :bit pattern is 
WDP1 = 1 :0110  change this for a different
WDP0 = 0 :timeout period.
*/
// Enter Watchdog Configuration mode:
WDTCSR = (1<<WDCE) | (1<<WDE);
// Set Watchdog settings: interrupte enable, 0110 for timer
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//15ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//30ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (0<<WDP0);//60ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0);//120ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0);//240ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);//480ms
//WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);//960ms
WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);//2s
//WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//4s
//WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s
sei();

//set_sleep_mode (SLEEP_MODE_IDLE);// 29.6ma - don't work
//set_sleep_mode (SLEEP_MODE_ADC);// 6.3ma clock is off
//set_sleep_mode (SLEEP_MODE_PWR_SAVE);// 3.7ma
//set_sleep_mode (SLEEP_MODE_STANDBY);// 2.7ma 
set_sleep_mode (SLEEP_MODE_PWR_DOWN);// 0.76ma

/*SLEEP_MODE_IDLE: 15 mA
SLEEP_MODE_ADC: 6.5 mA
SLEEP_MODE_PWR_SAVE: 1.62 mA
SLEEP_MODE_EXT_STANDBY: 1.62 mA
SLEEP_MODE_STANDBY : 0.84 mA
SLEEP_MODE_PWR_DOWN : 0.36 mA
*/

/*Power Reduction Register (PRR)
The next thing to experiment with is the Power Reduction Register (PRR). This lets you "turn off" various things inside the processor.
The various bits in this register turn off internal devices, as follows:

Bit 7 - PRTWI: Power Reduction TWI
Bit 6 - PRTIM2: Power Reduction Timer/Counter2
Bit 5 - PRTIM0: Power Reduction Timer/Counter0
Bit 4 - Res: Reserved bit
Bit 3 - PRTIM1: Power Reduction Timer/Counter1
Bit 2 - PRSPI: Power Reduction Serial Peripheral Interface
Bit 1 - PRUSART0: Power Reduction USART0
Bit 0 - PRADC: Power Reduction ADC
*/

//PRR = 0xFF; // not working

  sleep_enable();
    // turn off brown-out enable in software
 // MCUCR = bit (BODS) | bit (BODSE);  // turn on brown-out enable select
 // MCUCR = bit (BODS);        // this must be done within 4 clock cycles of above
  sleep_cpu(); 
  
  
  sleep_disable();//wakeup

//for(long i=0;i<6000000;i++){NOP;}//~4200ms delay


  //delay(1000);               // wait for a second
  }
