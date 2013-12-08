#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

#include <DS1302.h>
#include <SPI.h>
#include "AyPa_fonts.h"


// nokia 5110 pins layout 

#define RST 3
#define CE 4
#define DC 5
#define DIN 11
#define CLK 13

// rtc 1302 pins


#define CErtc 4 //CE of DS1302 and of Nokia3110 are compliment each other nicely!
#define CLKrtc 7
#define IOrtc 6


//7216 bytes
DS1302 rtc(4,6,7);//ce data clk
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


// 89 clocks
// integer 3 digits representation 
void s3(word v)
{
  byte c,ch;

  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);

  SPDR = 0;// start transfer with space  
  ch=v/100;
  v-=ch*100;
  ch*=3;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = 0;// start transfer with space
  ch=v/10;
  v-=ch*10;
  ch*=3;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = 0;// start transfer with space
  ch=v*3;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));

  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);      
}

//51 us
// integer 2 digits representation 
void s2(word v)
{
  byte c,ch;

  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);

  SPDR = 0;// start transfer with space
  ch=v/10;
  v-=ch*10;
  ch*=3;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = 0;// start transfer with space
  ch=v*3;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));

  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);      
}

// 164 clocks
// integer word representation 
void sw(word v)
{
  byte c,ch;

  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);

  SPDR = 0;// start transfer with space  
  ch=v/10000;
  v-=ch*10000;
  ch*=3; 
  c = pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c; 
  c = pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c; 
  c = pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c; 
  c = pgm_read_byte(&(Dig[ch++]));//c=Dig[ch];
  while(!(SPSR&(1<<SPIF)));
  SPDR = 0;// start transfer with space
  ch=v/1000;
  v-=ch*1000;
  ch*=3; 
  c = pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c; 
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = 0;// start transfer with space
  ch=v/100;
  v-=ch*100;
  ch*=3;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = 0;// start transfer with space
  ch=v/10;
  v-=ch*10;
  ch*=3;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = 0;// start transfer with space
  ch=v*3;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));

  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);      
}

// 27 clocks
//hex byte representation
void sh(byte v)
{
  byte c,ch;

  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);

  //do{
  SPDR = 0;// start transfer with space
  ch=(v>>4)*3;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = 0;// start transfer with space
  ch=(v&0xF)*3;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  SPDR = c;
  c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
  while(!(SPSR&(1<<SPIF)));
  //}while(i!=0xff);

  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);      
}


//114-115 clocks
//binary byte representation
void sb(byte v)
{
  byte c,ch,i=7;

  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);

  do{
    SPDR = 0;// start transfer with space
    ch=0;
    if(v&(1<<i--))ch=3;
    c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
    while(!(SPSR&(1<<SPIF)));
    SPDR = c;
    c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
    while(!(SPSR&(1<<SPIF)));
    SPDR = c;
    c=pgm_read_byte(&(Dig[ch++]));//c=Dig[ch++];
    while(!(SPSR&(1<<SPIF)));
    SPDR = c;
    c=pgm_read_byte(&(Dig[ch]));//c=Dig[ch++];
    while(!(SPSR&(1<<SPIF)));
  }
  while(i!=0xff);

  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);      
}

// clocks delay with 42 chars in string (Atmega328 with internal 8MHz oscillator)
// 7774 clocks original
// 7637
// 6624
// 1100
// 1057
// 952
// 946 
// 799
// 784..811 all ascii / all 16bit unicode  STRANGE... this must be ofsetted by SPI transfer. maybe volatile asm provide this better
void sa(char *st) // send ASCII string to display at current position
{
  byte i=0,c;
  word ch;

  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);

  do{
    //    LcdWriteData(0);//space  (start with it - while it is sending can calc address)

    SPDR = 0;// start transfer with space (while it is sending can calc address)
    //calcs
    c=st[i++];
    if (c>127){
      c=st[i++];
    }// 16bit code
    ch=(c-32)*5;
    //    c=Rus[ch++];//preload next char
    c=pgm_read_byte(&(Rus[ch++]));

    while(!(SPSR&(1<<SPIF)));

    //  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  

    //-----------------------------------------------------
    //  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
    //  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);
    //    LcdWriteData(Rus[ch]);
    SPDR = c;//Rus[ch++];
    //    c=Rus[ch++];
    c=pgm_read_byte(&(Rus[ch++]));

    while(!(SPSR&(1<<SPIF)));
    //  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  
    //----------------------------------------------------  
    //  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
    //  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);
    //    LcdWriteData(Rus[ch+1]);
    SPDR = c;
    //    c=Rus[ch++];
    c=pgm_read_byte(&(Rus[ch++]));
    while(!(SPSR&(1<<SPIF)));
    //  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  
    //----------------------------------------------------  
    //  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
    //  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);
    //    LcdWriteData(Rus[ch+2]);
    SPDR = c;
    c=pgm_read_byte(&(Rus[ch++]));
    //    c=Rus[ch++];
    while(!(SPSR&(1<<SPIF)));

    //  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  
    //----------------------------------------------------  
    //  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
    //  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);
    //    LcdWriteData(Rus[ch+3]);
    SPDR = c;
    c=pgm_read_byte(&(Rus[ch++]));
    //    c=Rus[ch++];
    while(!(SPSR&(1<<SPIF)));

    //  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);  
    //----------------------------------------------------  
    //  PORTD|=(1<<DC);//  digitalWrite(DC,HIGH); //port commands!!! DC-D5 CE-D7
    //  PORTD&=~(1<<CE);  //  digitalWrite(CE,LOW);
    //    LcdWriteData(Rus[ch+4]);
    SPDR = c;
    while(!(SPSR&(1<<SPIF)));
    //----------------------------------------------------  
    //if(st[i]==0){break;}
    //  }while (1);
  }
  while (st[i]!=0);//same same

  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);    
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
  }
  while (st[i]!=0);

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

//1828us (clocks with /8 prescaler)
void LcdClear(void)
{
  LcdSet(0,0);for(byte i=0;i<84;i++){sa(" ");} // clear ram manually (1828us)
  //for(byte i=0;i<6;i++){sa("              ");} // (1614us)
}

void LcdWriteCmd(byte cmd)
{

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


ISR (TIMER1_COMPA_vect){
  ticks++;
}

void SetADCinputChannel(uint8_t input,uint16_t us)
{
  ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input; // input (0..7)
  ADCSRB = 0; // mux5 bit
  //    if(stime) delay(stime); // Wait for input channel to settle
  if(us)delayMicroseconds(us); // Wait for input channel to settle (300us)

}
//#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 



/*
typedef struct { 
 char c1; 
 char c2; 
 uint8_t b1; 
 uint8_t b2; 
 } sager_type; 
 
 static sager_type forecast[] PROGMEM = 
 { 
 {'C', 'U', '8', '8'}, 
 {'C', 'U', '8', '8'}, 
 {'C', 'U', '8', '8'}, 
 {'C', 'U', '8', '8'}, 
 {'W', 'U', '8', '8'}, 
 {'A', 'U', '8', '8'}, 
 {'A', 'U', '8', '8'} 
 }; 
 
 sager_type ram_struct; */
byte pin2_interrupt_flag=0;
long timv=0;
long timl=0;
long t2ovf=0;
long t1ovf=0;

word t1111;

void pin2_isr()
{
  //timl=timv;
  t1111=TCNT1;//timl=t2ovf;
  sleep_disable();
  detachInterrupt(0);
  pin2_interrupt_flag = 1;
}


// the setup routine runs once when you press reset:
void setup() {                

  //  memcpy_P(&ram_struct, &forecast[4], sizeof(ram_struct)); 


  // initialize the digital pin as an output.
  //  pinMode(led, OUTPUT);  
  //  digitalWrite(A5,HIGH); pullup resistor
  // pinMode(9, OUTPUT); //ACS712 module

  // digitalWrite(9,HIGH);// lcd+acs712 

  //lcd.begin(20,4);

  // pinMode(A5, OUTPUT); //LCD led backlight
  //  digitalWrite(A5,HIGH);

  //memcpy_P(&Dig, &Dig, sizeof(Dig)); 

  //watchdogSetup();


  //setup timer1
  cli();
  TCCR1A=0x00;
//  TCCR1B=(1 << WGM12)|(0<<CS22)|(0<<CS21)|(1<<CS20); // /no prescaler;
  TCCR1B=(1 << WGM12)|(0<<CS22)|(1<<CS21)|(0<<CS20); // /8; 1us clock
  TCNT1H=0x00;
  TCNT1L=0x00;
  ICR1H=0x00;
  ICR1L=0x00;
  //OCR1AH=0x9C;
  //OCR1AL=0x40; // 40000
  //OCR1AH=0x4E;
  //OCR1AL=0x20; // 20000
  OCR1AH=0xFF;
  OCR1AL=0xFF;

  //  TCCR1B |= (1 << WGM12);
  //  TCCR1B |= (1 << CS10);
  // TIMSK1 |= (1 << OCIE1A);//no need for interrupt just using for profiling code
   TIMSK1 |= (1 << OCIE1A);
  sei();

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
  cli();
  ADCSRB = 0;
  ACSR = (1<<ACBG); //bandgap instead of AIN0 int on falling edge 
  //  delay(1); // to avoid false interrupt due bandgap voltage settle

  //  ACSR = (1<<ACI)|(1<<ACBG)|(1<<ACIE)|(1<<ACIS1)|(0<<ACIS0); //bandgap instead of AIN0 int on falling edge 
  //  ACSR = (1<<ACBG)|(1<<ACIE)|(0<<ACIS1)|(0<<ACIS0); //bandgap instead of AIN0 int on toggle
  //  ACSR = (1<<ACBG); //bandgap instead of AIN0 int on toggle
//  DIDR1 = (1<<AIN0D); // it will be bandgap on AIN0  (debug rtc)
  sei();


  //TIMSK0 &= ~_BV(TOIE0); // disable timer0 overflow interrupt
  // when it is not needed

  /*
  pinMode(4,OUTPUT);pinMode(6,OUTPUT);pinMode(7,OUTPUT);
   
   rtc.halt(false);
   rtc.writeProtect(false);
   rtc.setDOW(SUNDAY);        // Set Day-of-Week to FRIDAY
   rtc.setTime(0, 30, 0);     // Set the time to 12:00:00 (24hr format)
   rtc.setDate(8, 12, 2013);   // Set the date to August 6th, 2010
   //  rtc.writeProtect(true);
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


/*

void shiftOut5(byte val,byte vl2)
{
  byte imm;
  
    imm=PORTLMASK|0b10000000; if(val&(1<<(7)))imm|=0b01000000;if(vl2&(1<<(7)))imm|=0b00010000;PORTL=imm;
      PORTL = 0b00000000|PORTLMASK;
    imm=PORTLMASK|0b10000000;if(val&(1<<(6)))imm|=0b01000000;if(vl2&(1<<(6)))imm|=0b00010000;PORTL=imm;
      PORTL = 0b00000000|PORTLMASK;
    imm=PORTLMASK|0b10000000;if(val&(1<<(5)))imm|=0b01000000;if(vl2&(1<<(5)))imm|=0b00010000;PORTL=imm;
      PORTL = 0b00000000|PORTLMASK;
    imm=PORTLMASK|0b10000000;if(val&(1<<(4)))imm|=0b01000000;if(vl2&(1<<(4)))imm|=0b00010000;PORTL=imm;
      PORTL = 0b00000000|PORTLMASK;
    imm=PORTLMASK|0b10000000;if(val&(1<<(3)))imm|=0b01000000;if(vl2&(1<<(3)))imm|=0b00010000;PORTL=imm;
      PORTL = 0b00000000|PORTLMASK;
    imm=PORTLMASK|0b10000000;if(val&(1<<(2)))imm|=0b01000000;if(vl2&(1<<(2)))imm|=0b00010000;PORTL=imm;
      PORTL = 0b00000000|PORTLMASK;
    imm=PORTLMASK|0b10000000;if(val&(1<<(1)))imm|=0b01000000;if(vl2&(1<<(1)))imm|=0b00010000;PORTL=imm;
      PORTL = 0b00000000|PORTLMASK;
    imm=PORTLMASK|0b10000000;if(val&(1<<(0)))imm|=0b01000000;if(vl2&(1<<(0)))imm|=0b00010000;PORTL=imm;
      PORTL = PORTLMASK|0b00100000; // set master latch HIGH
      PORTL = PORTLMASK|0b00001000; // set secondary latch HIGH (allow ~50ns propagation delay)
}
*/

/*

void shiftOut6(byte val,byte vl2)
{
  byte imm;
  byte PORTDMASK=PORTD&0b00000111;
  
    imm=PORTDMASK|0b10000000; if(val&(1<<(7)))imm|=0b01000000;if(vl2&(1<<(7)))imm|=0b00010000;PORTD=imm;
      PORTD = 0b00000000|PORTDMASK;
    imm=PORTDMASK|0b10000000;if(val&(1<<(6)))imm|=0b01000000;if(vl2&(1<<(6)))imm|=0b00010000;PORTD=imm;
      PORTD = 0b00000000|PORTDMASK;
    imm=PORTDMASK|0b10000000;if(val&(1<<(5)))imm|=0b01000000;if(vl2&(1<<(5)))imm|=0b00010000;PORTD=imm;
      PORTD = 0b00000000|PORTDMASK;
    imm=PORTDMASK|0b10000000;if(val&(1<<(4)))imm|=0b01000000;if(vl2&(1<<(4)))imm|=0b00010000;PORTD=imm;
      PORTD = 0b00000000|PORTDMASK;
    imm=PORTDMASK|0b10000000;if(val&(1<<(3)))imm|=0b01000000;if(vl2&(1<<(3)))imm|=0b00010000;PORTD=imm;
      PORTD = 0b00000000|PORTDMASK;
    imm=PORTDMASK|0b10000000;if(val&(1<<(2)))imm|=0b01000000;if(vl2&(1<<(2)))imm|=0b00010000;PORTD=imm;
      PORTD = 0b00000000|PORTDMASK;
    imm=PORTDMASK|0b10000000;if(val&(1<<(1)))imm|=0b01000000;if(vl2&(1<<(1)))imm|=0b00010000;PORTD=imm;
      PORTD = 0b00000000|PORTDMASK;
    imm=PORTDMASK|0b10000000;if(val&(1<<(0)))imm|=0b01000000;if(vl2&(1<<(0)))imm|=0b00010000;PORTD=imm;
      PORTD = PORTDMASK|0b00000000; // no latches but set CLK to LOW
//      PORTL = PORTLMASK|0b00000000; // set secondary latch HIGH (allow ~50ns propagation delay)
}*/


//CLKrtc IOrtc
void ShiftOut(byte val)
{
//PORTD&=~(1<<CLKrtc);//clk low (already)

PORTD&=~(1<<IOrtc);//clear data bit
if (val&(1<<(0)))PORTD|=(1<<IOrtc);// set it if needed
PORTD|=(1<<CLKrtc);// tick clk

PORTD&=~(1<<CLKrtc);//clk low
PORTD&=~(1<<IOrtc);//clear data bit
if (val&(1<<(1)))PORTD|=(1<<IOrtc);// set it if needed
PORTD|=(1<<CLKrtc);// tick clk

PORTD&=~(1<<CLKrtc);//clk low
PORTD&=~(1<<IOrtc);//clear data bit
if (val&(1<<(2)))PORTD|=(1<<IOrtc);// set it if needed
PORTD|=(1<<CLKrtc);// tick clk

PORTD&=~(1<<CLKrtc);//clk low
PORTD&=~(1<<IOrtc);//clear data bit
if (val&(1<<(3)))PORTD|=(1<<IOrtc);// set it if needed
PORTD|=(1<<CLKrtc);// tick clk

PORTD&=~(1<<CLKrtc);//clk low
PORTD&=~(1<<IOrtc);//clear data bit
if (val&(1<<(4)))PORTD|=(1<<IOrtc);// set it if needed
PORTD|=(1<<CLKrtc);// tick clk

PORTD&=~(1<<CLKrtc);//clk low
PORTD&=~(1<<IOrtc);//clear data bit
if (val&(1<<(5)))PORTD|=(1<<IOrtc);// set it if needed
PORTD|=(1<<CLKrtc);// tick clk

PORTD&=~(1<<CLKrtc);//clk low
PORTD&=~(1<<IOrtc);//clear data bit
if (val&(1<<(6)))PORTD|=(1<<IOrtc);// set it if needed
PORTD|=(1<<CLKrtc);// tick clk

PORTD&=~(1<<CLKrtc);//clk low
PORTD&=~(1<<IOrtc);//clear data bit
if (val&(1<<(7)))PORTD|=(1<<IOrtc);// set it if needed
PORTD|=(1<<CLKrtc);// tick clk

PORTD&=~(1<<CLKrtc);//clk low (needed)

//"=r" (result): "I" (val)

}

/*
Register Usage
r0 This can be used as a temporary register. If you assigned a value to this
register and are calling code generated by the compiler, you’ll need to save
r0, since the compiler may use it. Interrupt routines generated with the
compiler save and restore this register.
r1 The compiler assumes that this register contains zero. If you use this register
in your assembly code, be sure to clear it before returning to compiler generated
code (use ”clr r1”). Interrupt routines generated with the compiler
save and restore this register, too.
r2–r17, r28, r29 These registers are used by the compiler for storage. If your assembly
code is called by compiler generated code, you need to save and restore any
of these registers that you use. (r29:r28 is the Y index register and is used
for pointing to the function’s stack frame, if necessary.)
r18–r27, r30, r31 These registers are up for grabs. If you use any of these
registers you need to save its contents if you call any compiler generated code.

Function calls use the register pairs r25:r24, r23:r22, r21:r20 down to r9:r8. Additional arguments are passed on the stack.
Return values are passed in r25:r24. The register pairs r27:r26, r29:r28 and r31:r30 can be used to indirectly address memory.
This is very important because clever array addressing loops are the main way to optimize C code for space.

Return Values
8-bit values are returned in r24. 16-bit values are returned in r25:r24.
32-bit values are returned in r25:r24:r23:r22. 64-bit values are returned in r25:-
r24:r23:r22:r21:r20:r19:r18.
*/

#define loop_until_bit_is_clear(port, bitn)__asm__ __volatile__ ("L_%=: " "sbic %0, %1" "\n\t""rjmp L_%=": /* no outputs */: "I" ((uint8_t)(port)),"I" ((uint8_t)(bitn)))


//CLKrtc IOrtc
byte ShiftIn(void)
{
byte val=0;
  //PORTD&=~(1<<CLKrtc);//clk low (already)
/*
PORTD|=(1<<CLKrtc);// tick clk

val=0;imm=PIND&(1<<IOrtc);
//if(PIND&(1<<IOrtc))val=(1<<0);
if(imm)val=(1<<0);
//val=(PIND&(1<<IOrtc));

PORTD&=~(1<<CLKrtc);//clk low

PORTD|=(1<<CLKrtc);// tick clk
if(PIND&(1<<IOrtc))val|=(1<<1);
PORTD&=~(1<<CLKrtc);//clk low

PORTD|=(1<<CLKrtc);// tick clk
if(PIND&(1<<IOrtc))val|=(1<<2);
PORTD&=~(1<<CLKrtc);//clk low

PORTD|=(1<<CLKrtc);// tick clk
if(PIND&(1<<IOrtc))val|=(1<<3);
PORTD&=~(1<<CLKrtc);//clk low

PORTD|=(1<<CLKrtc);// tick clk
if(PIND&(1<<IOrtc))val|=(1<<4);
PORTD&=~(1<<CLKrtc);//clk low

PORTD|=(1<<CLKrtc);// tick clk
if(PIND&(1<<IOrtc))val|=(1<<5);
PORTD&=~(1<<CLKrtc);//clk low

PORTD|=(1<<CLKrtc);// tick clk
if(PIND&(1<<IOrtc))val|=(1<<6);
PORTD&=~(1<<CLKrtc);//clk low

PORTD|=(1<<CLKrtc);// tick clk
if(PIND&(1<<IOrtc))val|=(1<<7);

PORTD&=~(1<<CLKrtc);//clk low (needed)
return val;

*/
// sbic - skip if bit in io register cleared
__asm__ __volatile__(
"clr r24\n\t"

"sbi 0x0b,7\n\t"
"sbic 0x09,6\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x01\n\t"
"cbi 0x0b,7\n\t"

"sbi 0x0b,7\n\t"
"sbic 0x09,6\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x02\n\t"
"cbi 0x0b,7\n\t"

"sbi 0x0b,7\n\t"
"sbic 0x09,6\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x04\n\t"
"cbi 0x0b,7\n\t"

"sbi 0x0b,7\n\t"
"sbic 0x09,6\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x08\n\t"
"cbi 0x0b,7\n\t"

"sbi 0x0b,7\n\t"
"sbic 0x09,6\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x10\n\t"
"cbi 0x0b,7\n\t"

"sbi 0x0b,7\n\t"
"sbic 0x09,6\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x20\n\t"
"cbi 0x0b,7\n\t"

"sbi 0x0b,7\n\t"
"sbic 0x09,6\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x40\n\t"
"cbi 0x0b,7\n\t"

"sbi 0x0b,7\n\t"
"sbic 0x09,6\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x80\n\t"
"cbi 0x0b,7\n\t"

: "=r" (val)::);

return val;
}

//23us
/*
void rtcpoke(byte addr,byte val)//7574
{
addr=addr+addr+192;
PORTD&=~(1<<CLKrtc);//digitalWrite(CLKrtc,LOW); //4
PORTD|=(1<<CErtc);//digitalWrite(CErtc,HIGH); //D7
 DDRD|=(1<<IOrtc);//(9clocks vs 1)//set bit IOrtc in DDRC//pinMode(IOrtc,OUTPUT);
ShiftOut(addr);//shiftOut(IOrtc,CLKrtc,LSBFIRST,addr);//253us
ShiftOut(val);//shiftOut(IOrtc,CLKrtc,LSBFIRST,val);//253us
 DDRD&=~(1<<IOrtc);//(9clocks vs 1)// clear bit IOrtc in DDRC//pinMode(IOrtc,INPUT);
PORTD&=~(1<<CErtc);//digitalWrite(CErtc,LOW);//D7  
}*/

//21us as a macro (20 bytes)//7556(18 bytes less than function if called once)

#define rtcpoke(addr,val)\
PORTD&=~(1<<CLKrtc);\
PORTD|=(1<<CErtc);\
 DDRD|=(1<<IOrtc);\
ShiftOut(addr+addr+192);\
ShiftOut(val);\
 DDRD&=~(1<<IOrtc);\
PORTD&=~(1<<CErtc);\


//500
//373
//322
//277
//272
//271
//252
//19us after switch to ShiftOut

byte rtcpeek(byte addr)
{
byte val;

addr=addr+addr+193;
PORTD&=~(1<<CLKrtc);//digitalWrite(CLKrtc,LOW); 
PORTD|=(1<<CErtc);//digitalWrite(CErtc,HIGH);
DDRD|=(1<<IOrtc);//(9clocks vs 1)//set bit IOrtc in DDRC//pinMode(IOrtc,OUTPUT);
ShiftOut(addr);//shiftOut(IOrtc,CLKrtc,LSBFIRST,addr);
DDRD&=~(1<<IOrtc);//(9clocks vs 1)// clear bit IOrtc in DDRC//pinMode(IOrtc,INPUT);
val=ShiftIn();
PORTD&=~(1<<CErtc);//digitalWrite(CErtc,LOW);// acts as reset otherwise next command is broken
return val;
}

int freeRam(void)
{
  extern unsigned int __heap_start;
  extern void *__brkval;
  int free_memory,stack_here;
  if (__brkval == 0) { free_memory = (int) &stack_here - (int) &__heap_start;}
  else{free_memory = (int) &stack_here - (int) __brkval;}
  return (free_memory);
}

word sc[16];
word mn=5555,mx=5000;

char buf[64]; // carefully with long vars. 2k ram only
uint16_t t1,t2,tt1,tt2,ttt1,ttt2;

// the loop routine runs over and over again forever:
void loop() {
  word t,n;

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


  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(4,OUTPUT);//rtc CE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! important
  pinMode(7,OUTPUT);//rtc CLK
  pinMode(6,OUTPUT);//rtc IO
  

  digitalWrite(9,HIGH);// acs712 module + lcd
  digitalWrite(10,HIGH);// 5v mosfet control

  SPI.begin();
  InitSPI();

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

  LcdClear();

//  cli();TCNT1=0;
 sa("Тестовая АуРа!"); 
  //sa("Тестовая АуРа!1234567890+-*~=============="); 
  //sa("Marinochka!!!a1234567890+-*~=============="); 
  //sa("Мариночка!!!!Мариночка!!!!!Мариночка!!!!!");
//  t=TCNT1;  sei();

  //.for(int i=0;i<strlen(buf2);i++){sprintf(buf,"%d %d %c %d %d]",i,buf2[i],buf2[i],Rus[(buf2[i]-32)*5],Rus[(buf2[i]-32)*5+1]);sa(buf);}
  //sprintf( buf+strlen(buf), ",%s:%04i", sensorCode, sensorValue );
  sprintf(buf," INT0=%d ",pin2_interrupt_flag);
  sw(t1111);
  sa(" ");
  sw(freeRam());
  sa(buf);



//  cli();TCNT1=0;
  //sb(n);
  //sh(0x01);
  //sh(0xED);
  //sw(65535);
  //sw(t);
//  s3(t);
//  s2(t);

  //t=TCNT1;  sei();
//  sprintf(buf," CNT1=%d",t); sa(buf);



  //for(byte i=0;i<84;i++){SendChar(i);}
  //delay(7000);
  //for(byte i=84;i<168;i++){SendChar(i);}
  //delay(7000);

  //for(byte i=0;i<84;i++){SendStr(ord(i));}
  //delay(7000);
  //sprintf( buf+strlen(buf), ",%s:%04i", sensorCode, sensorValue );
  //SendStr("MARINOCHKA!!!a1234567890");
  //delay(7000);



  //lcd
//  pinMode(A0,INPUT);
//  pinMode(6,OUTPUT);
  //pinMode(7,OUTPUT);
//  pinMode(8,OUTPUT);//rtc

  //pinMode(A1,OUTPUT);
 // pinMode(A2,OUTPUT);
 // pinMode(A3,OUTPUT);
 // pinMode(A4,OUTPUT);
  //pinMode(2,OUTPUT);
 // pinMode(3,OUTPUT);
 // pinMode(4,OUTPUT);
  // lcd.begin(20,4);  lcd.setCursor(0,0); //lcd.print("Marinochka lapochka!"); delay(1000);

  SetADCinputChannel(0,500);
  //  SetADCinputChannel(5,500);
  word i;
  mRawADC(i,2);
  //digitalWrite(A5,HIGH);

  //A0 playground
  //-------------------------------------------

  // stage1
  //--------------------------
  //     +Vcc
  //      |
  //     ||| 20-100K
  //      |
  // ---------zzzz220R----A0
  //      |
  //     ===
  //      |
  //      G

  //A0=102 of 1023

  //n=digitalRead(A0);// 1
  //n=PINC&1; // 1
  //byte c=DDRC;//1E 0
  //n=digitalRead(A0);// 1

  //DDRC|=(1<<0);//pinMode(A0,OUTPUT);
  //PORTC&=~(1<<0);//digitalWrite(A0,LOW);
  //n=PINC&1; // 0

    //NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
  //NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;

  //A0=96 of 1023
  //A0=44..48 of 1023 with //digitalWrite(A0,LOW);
  //A0=63..64 of 1023 with PORTC&=~(1<<0);


 // delayMicroseconds(50);// time for capacitor to discharge A0=12;
  //delayMicroseconds(30);// time for capacitor to discharge A0=28;
  //delayMicroseconds(20);// time for capacitor to discharge A0=39 of 1023;





  //cli();TCNT1=0;
 // byte c=DDRC;//1F 1
 // DDRC&=~(1<<0);//pinMode(A0,INPUT);//(9clocks vs 1)// clear bit A0 in DDRC
 // byte d=DDRC;//1E 0
  //t=TCNT1;sei();

  // here the capacitor is charging

  // A0=143 of 1023

  //cli();TCNT1=0;mRawADC(i,2);t=TCNT1;sei();

  //delayMicroseconds(10);// time for capacitor to discharge

  //i=PINC;//0
  //n=0;while(digitalRead(A0)==LOW){if(++n==3000){break;}}//n=114
  //n=0;while((PINC&1)==LOW){NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;if(++n==32000){break;}}//n=419 with ~1us delay (8x0.125)
  //n=0;while((PINC&1)==LOW){NOP;NOP;NOP;NOP;if(++n==32000){break;}}//n=541 with ~0.5us delay (4x0.125)
  //n=0;while((PINC&1)==LOW){if(++n==32000){break;}}//n=755..778 with 2/8 us delay (2x0.125) 0.1microfarad +20k pullup +220r to A0

  //pinMode(2,INPUT);
  //digitalWrite(2,HIGH);delay(100);
  //digitalWrite(2,LOW);

/*
  cli();
  TCNT1=0;
  n=0;
//  while((PINC&1)==LOW){
  while((PIND&0b00000100)==LOW){
    if(++n==32000){
      break;
    }
  }//n=419 with ~1us delay (8x0.125)
  //n=0;while((PINC&1)==LOW){if(++n==32000){break;}}//n=4502..4746 with 2/8 us delay (2x0.125) 0.1microfarad +100k pullup +220r to A0
  t=TCNT1;
  sei();*/

//  n=digitalRead(2);
  //i=PINC;//1

  //A0=519 of 1023

  //cli();TCNT1=0;mRawADC(i,2);t=TCNT1;sei();

  // stage2 - capacitor is discharging to ground very fast. maybe connect it to closed mosfet instead of ground???
  // nope. different current circuits. don't mix them
  //--------------------------
  // ---------zzzz220R----A0
  //     +|
  //     ===
  //      |
  //      G

  // charge the capacitor by output logic HIGH to pin
  //cli();TCNT1=0;mRawADC(i,2);t=TCNT1;sei(); //A0=67 of 1023 here

  //DDRC|=(1<<PINC0);//pinMode(A0,OUTPUT);
  //PORTC|=(1<<PINC0);//digitalWrite(A0,HIGH);
  //PORTC|=(1<<PINC0);// in asm this is supposedly right to avoid logic 1 during transitions? need extended check though
  //DDRC|=(1<<PINC0);

  //n=PINC;
  //n=PIND2;//2

  //delayMicroseconds(100);// time for capacitor to charge (1022)
  //delayMicroseconds(50);// time for capacitor to charge (1012)
  //delayMicroseconds(10);// time for capacitor to charge (960)
  //delayMicroseconds(5);// time for capacitor to charge (944)

  //cli();TCNT1=0;mRawADC(i,2);t=TCNT1;sei();//924
  // now disconnect pin

  //n=PINC&1;

  //PORTC&=~(1<<PINC0);//digitalWrite(A0,LOW);// more rapid discharge if just write low
  //DDRC&=~(1<<PINC0);//pinMode(A0,INPUT);// probably this will be during sleeping 17..22

  //n=PINC&1;

  //cli();TCNT1=0;mRawADC(i,2);t=TCNT1;sei();//82

  //cli();TCNT1=0;
  //n=0;while((PINC&1)==HIGH){if(++n==32000){break;}}//n=3805..4374 with 2/8 us delay (2x0.125) 0.1microfarad +100k pullup +220r to A0
  //t=TCNT1;sei();

/*
  sprintf(buf,"     TCNT1=%d i=%d",t,i);
  sa(buf);
  LcdSet(0,4);

  TCNT2=0;
  //delay(2);
  //delayMicroseconds(1000);//4 ticks on 8mhz
  word c1=TCNT1;
  long dl=t2ovf;
  sprintf(buf,"n%d c%d d%d    ",n,c1,dl);
  sa(buf);
*/

//  LcdSet(0,5);
  /*
cli();TCNT1=0;mRawADC(i,2);t=TCNT1;sei();
   //sprintf(buf,"b%d",i);sa(buf);
   
   pinMode(A5,OUTPUT);
   digitalWrite(A5,HIGH);// charge capacitor
   delay(1);
   int i1,i2,i3,i4,i5,i6;
   cli();mRawADC(i1,2);sei();
   //i4=digitalRead(A5);
   
   TCNT1=0;
   //PORTC&=~(1<<5);//digitalWrite(A5,LOW);
   digitalWrite(A5,LOW);
   pinMode(A5,INPUT);
   //DDRC&=~(1<<5);//pinMode(A5,INPUT);//(9clocks vs 1)// clear bit A5 in DDRC
   
   i3=digitalRead(A5);
   n=0;while((PINC&0b00100000)==HIGH){if(++n==32000){break;}}//n=
   
   cli();mRawADC(i2,2);sei();
   //cli();mRawADC(i3,2);sei();
   //cli();mRawADC(i4,2);sei();
   //cli();mRawADC(i5,2);sei();
   //cli();mRawADC(i6,2);sei();
   
   t=TCNT1;
   i3=n;
   
   //TCNT1=0;
   sprintf(buf,"%d %d %d %d %d %d ",i1,i2,i3,i4,i5,i6);
   //t=TCNT1;
   sa(buf);
   sprintf(buf,"t=%d",t);sa(buf);
   */

  


  //  lcd.clear();

  //rtc

  //for (int i=0; i<31; i++)ramBuffer.cell[i]=i;

  //  comment("Writing buffer to RAM...");
  //rtc.writeBuffer(ramBuffer);
  // bufferDump();

  //    comment("Setting byte 15 (0x0F) to value 160 (0xA0)...");
  // rtc.poke(15,160);
  //cli();TCNT1=0;
//rtc.writeBuffer(ramBuffer);//7896us
 // ramBuffer=rtc.readBuffer();//8075us
// c=rtc.peek(11);//520us
// rtc.poke(15,255);//517us
//t=TCNT1;sei();
  // bufferDump();

  //  comment("Reading address 18 (0x12). This should return 18, 0x12.");
  // r1 = rtc.peek(18);
  // r2 = rtc.peek(15);

//cli();TCNT1=0;
Time tim= rtc.getTime();//2292us
//t=TCNT1;sei();


//sprintf(buf,"  [  %d %d-%d-%d %d:%d:%d]",tim.dow,tim.date,tim.mon,tim.year,tim.hour,tim.min,tim.sec);sa(buf);
s2(tim.hour);
sa(":");
s2(tim.min);
sa(":");
s2(tim.sec);

//Time tim= rtc.getTime();//2292us

//start working with rtcclock. CErtc high
//PORTD|=(1<<CErtc);//digitalWrite(CErtc,HIGH);


//rtc.poke(15,0xE1);
byte val=0;
cli();TCNT1=0;

// use PB6&PB7 bits
for(int z=0;z<256;z++)
{
//rtcpoke(15,0x7A);
rtcpoke(15,z);if(rtcpeek(15)!=z){sh(z);}
}


t=TCNT1;sei();
//digitalWrite(CE,LOW);
//PORTD&=~(1<<CErtc);//digitalWrite(CErtc,LOW);

//PORTD|=(1<<CErtc);//digitalWrite(CErtc,HIGH);

val=rtcpeek(15);


//PORTD&=~(1<<CErtc);//digitalWrite(CErtc,LOW);
//stop working with rtcclock. CErtc low

//for(byte i=10;i<31;i++)
//{
//rtc.poke(i,i);
sa(">");
  sh(val);
  //sh(rtc.peek(15));
  sa("     ");
  s3(t);
//  s3(val);
//  sh(rtc.peek(15));
//  sh(rtc.peek(19));
 

  
  
  delay (400);

  /*
cli();
   t1=TCNT1;
   //for(int j=0;j<16;j++){sc[j]=analogRead(0);}
   for(int j=0;j<16;j++){mRawADC(sc[j],2);}
   t2=TCNT1;
   sei();
   */
  //lcd.setCursor(0,0);
 // long z=0;

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
 



//TCNT1=0;
 // cli();pinMode(2,OUTPUT);digitalWrite(2,HIGH);delayMicroseconds(65);digitalWrite(2,LOW);pinMode(2,INPUT);sei();// controlled charging(~99us)
//t=TCNT1;
//sw(t);delay(700);


  //for(long i=0;i<3000000;i++){NOP;}//~2100ms delay
  //  pinMode(A5, INPUT); //LCD led backlight
  //digitalWrite(A5,LOW);
  //  pinMode(A5, OUTPUT); //LCD led backlight
  //-------------------------------------------------------------------------------------------------[ power saving wait state ]
  digitalWrite(RST,LOW);
  // digitalWrite(RST,HIGH);

  SPI.end();

  ADCSRA=0;// switch off ADC
  ACSR = (1<<ACD); // switch off analog comparator
  digitalWrite(10,LOW);// lcd 
  digitalWrite(9,LOW);// acs712 module 
  //digitalWrite(A5,LOW);// LCD display backlight off
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(9,INPUT);
  pinMode(10,INPUT);
  pinMode(6,INPUT);
  pinMode(7,INPUT);
  pinMode(8,INPUT);

  //digitalWrite(2,LOW);//pinMode(2,INPUT);



  /*
cli();  // disable all interrupts
   wdt_reset(); // reset the WDT timer
   MCUSR &= ~(1<<WDRF);  // because the data sheet said to
   
   //WDTCSR configuration:
   //WDIE = 1 :Interrupt Enable
   //WDE = 1  :Reset Enable - I won't be using this on the 2560
   //WDP3 = 0 :For 1000ms Time-out
   //WDP2 = 1 :bit pattern is 
   //WDP1 = 1 :0110  change this for a different
   //WDP0 = 0 :timeout period.
   
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
   */

  //set_sleep_mode (SLEEP_MODE_IDLE);// 29.6ma - don't work
  //set_sleep_mode (SLEEP_MODE_ADC);// 6.3ma clock is off
  //set_sleep_mode (SLEEP_MODE_PWR_SAVE);// 3.7ma
  //set_sleep_mode (SLEEP_MODE_STANDBY);// 2.7ma 
  //set_sleep_mode (SLEEP_MODE_PWR_DOWN);// 0.76ma

  pinMode(2,OUTPUT);digitalWrite(2,HIGH);delayMicroseconds(65);digitalWrite(2,LOW);pinMode(2,INPUT);// controlled charging(~100us)

//                                       -----51K----
// D2(INT0)---------220R---|----+||-----|--------G   ~100us charging at 5V gives 1.7ms sleeping time (with 100K 3.2ms 680K 19ms 1M 32ms    without R 35.8ms)
//                                            0.1uF
//

cli();
  pin2_interrupt_flag=0;
  sleep_enable();
  attachInterrupt(0, pin2_isr, LOW);
  ticks=0;
  TCNT1=0;
  set_sleep_mode (SLEEP_MODE_IDLE);
  
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //set_sleep_mode (SLEEP_MODE_PWR_SAVE);// timer2 but it is not in async mode yet
  //cli();
  //sleep_bod_disable();
  //sei();
  sei();
  sleep_cpu();
  // wake up here
  sleep_disable();



  /*
attachInterrupt(0, pin2_isr, LOW);
   set_sleep_mode(SLEEP_MODE_PWR_DOWN);
   cli();
   sleep_enable();
   sleep_bod_disable();
   sei();
   sleep_cpu();
   // wake up here 
   sleep_disable();
   */
  /*power_adc_disable(),power_spi_disable(),power_timer0_disable(), power_timer1_disable(),power_timer2_disable(),power_twi_disable()*/
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

  //  sleep_enable();
  //sleep_cpu(); 


  //  sleep_disable();//wakeup

  //for(long i=0;i<6000000;i++){NOP;}//~4200ms delay


  //delay(1000);               // wait for a second
}

