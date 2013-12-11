//#define __AVR_ATmega328P__ 
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

//#include <DS1302.h>// cannot sit on SPI pins (leaves pin in input state)
#include <SPI.h> // < declaration ShiftOut etc problem
#include "AyPa_m.h"
#include "AyPa_fonts.h"
#include "AyPa_rtc.h"

int freeRam(void)
{
  extern unsigned int __heap_start;
  extern void *__brkval;
  int free_memory,stack_here;
  if (__brkval == 0) { free_memory = (int) &stack_here - (int) &__heap_start;}
  else{free_memory = (int) &stack_here - (int) __brkval;}
  return (free_memory);
}



// nokia 5110 pins layout 

#define RST 3
#define CE 4 // don't go along with CErtc but works on PIN4!
#define DC 5
#define DIN 11
#define CLK 13

// rtc 1302 pins


//#define CErtc 4 //CE of DS1302 and of Nokia3110 are compliment each other nicely!
//#define CLKrtc 7
//#define IOrtc 6


//7216 bytes
//DS1302 rtc(A4,A2,A3);//ce data clk


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

#define SetupWD(timeout){cli();wdt_reset();MCUSR&=~(1<<WDRF);WDTCSR=(1<<WDCE)|(1<<WDE);WDTCSR=(1<<WDIE)|timeout;WDhappen;sei();}

//byte odd=0;
//long r1=0,r2;

volatile byte WDhappen;
volatile word  t1111; // vars updated in ISR should be declared as volatile and accessed with cli()/sei() ie atomic


ISR(WDT_vect) // Watchdog timer interrupt.
{ 
  //resetFunc();  //reboot
//  r2=TCNT1;
  t1111=TCNT1;
  WDhappen=1;
  
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



// the setup routine runs once when you press reset:
void setup() {                
  wdt_disable();

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

  //delay(2000);
  //set date/time
//rtcwriteprotect(false);//20us
//rtc.writeProtect(false);
//rtc.setTime(22,49,0);
//   rtcsettime(0x15,0x59,0x35);
  // rtcsetdate(0x09,0x12,0x13);
   //rtcsetDOW(1);
   
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



word sc[16];
word mn=5555,mx=5000;
word sleeps;
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



    Pin2Output(DDRB,2);  //pinMode(10,OUTPUT);
    Pin2Output(DDRB,1);//  pinMode(9,OUTPUT);

//  pinMode(A4,OUTPUT);//rtc CE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! important
//  pinMode(A3,OUTPUT);//rtc CLK
//  pinMode(A2,OUTPUT);//rtc IO
  Pin2Output(DDRC,2);
  Pin2Output(DDRC,3);
  Pin2Output(DDRC,4);
  

  digitalWrite(9,HIGH);// acs712 module + lcd
  digitalWrite(10,HIGH);// 5v mosfet control

  //SPI.begin();//  InitSPI();
  
  Pin2HIGH(PORTB,2); //set SS high
  Pin2Output(DDRB,2); //SS pin

  //SPSR = (0 << SPI2X); //4
  //SPCR = (1 << MSTR) | (1 << SPE) |(1<<SPR0);      // enable, master, msb first
  SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first (lcd)
//  SPCR = (1 << MSTR) | (1 << SPE) | (1<<DORD);      // enable, master, lsb first (rtc)
  SPSR = (1 << SPI2X);// 1/2clk
  Pin2Output(DDRB,3); //MOSI pin
  Pin2Output(DDRB,5); //SCK pin


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
  sprintf(buf,"  WD=%d n=%d ",WDhappen,sleeps);
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
/*
//cli();TCNT1=0;
//Time tim= rtc.getTime();//2292us
//t=TCNT1;sei();


//sprintf(buf,"  [  %d %d-%d-%d %d:%d:%d]",tim.dow,tim.date,tim.mon,tim.year,tim.hour,tim.min,tim.sec);sa(buf);
s2(tim.hour);
sa(":");
s2(tim.min);
sa(":");
s2(tim.sec);
*/

//Time tim= rtc.getTime();//2292us

Pin2Output(DDRC,2);
  Pin2Output(DDRC,3);
  Pin2Output(DDRC,4);

//Pin2Output(DDRB,3);
//Pin2Output(DDRB,5); //this SCK/CLK pin DS1302 library has switched to INPUT mode
//start working with rtcclock. CErtc high
//PORTD|=(1<<CErtc);//digitalWrite(CErtc,HIGH);


byte val=0;
//cli();TCNT1=0;

// use PB6&PB7 bits

//rtcwriteprotect(true);//20us(cannot pair with false)
rtcgettime(7);//rtcgettime(8); 7 is enough


//t=TCNT1;sei();
//digitalWrite(CE,LOW);
//PORTD&=~(1<<CErtc);//digitalWrite(CErtc,LOW);

//PORTD|=(1<<CErtc);//digitalWrite(CErtc,HIGH);
//rtc.poke(15,0x1C);//returns 1E
rtcpoke(15,0x1C);//returns 1E
//rtcpoke(15,0xcc);

val=rtcpeek(15);



//PORTD&=~(1<<CErtc);//digitalWrite(CErtc,LOW);
//stop working with rtcclock. CErtc low

//for(byte i=10;i<31;i++)
//{
//rtc.poke(i,i);
//s2(tim.hour);
//s2(tim.min);
//s2(tim.sec);
//sa(" >");
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
  //sh(rtc.peek(15));
  sa(" ");
  s3(t);
//loop_until_bit_is_clear(PORTD,7);
  /*
  __asm__ __volatile__("nop\n\t"
  "in r24,%1\n\t" 
  "sts [val],r24\n\t"
  :[val]"=&w"(val)
  :[PORT]"I"(_SFR_I08(PORTD)):);
  */
//  sa(PORTD);


//  s3(val);
//  sh(rtc.peek(15));
//  sh(rtc.peek(19));
/*
WDhappen=0;
  sa("WD");
  s2(WDhappen);
  NOP;
//SetupWD(WDTO_15MS);
//wdt_enable(WDTO_15MS);
//wdt_reset();
TCNT1=0;
//  delay (2000);
  //wdt_disable();


  sa("WD");
  s2(WDhappen);
  sa("~");
  sw(t1111);
  */
  delay(1500);
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

  SPCR&=~(1<<SPE); //  SPI.end();

  ADCSRA=0;// switch off ADC
  ACSR = (1<<ACD); // switch off analog comparator
//  digitalWrite(10,LOW);// lcd 
//  digitalWrite(9,LOW);// acs712 module 
  //digitalWrite(A5,LOW);// LCD display backlight off
/*  pinMode(A0,INPUT);
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
*/
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

//  SetupWD(1);


  
 /*** Setup the WDT ***/
  cli();
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
//  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
//  WDTCSR = 0; /* 15ms */
//  WDTCSR = 1; /* 30ms */
//  WDTCSR = 2; /* 60ms */
  //WDTCSR = 3; /* 120ms */
//  WDTCSR = 9; /* 8s */

   WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//15ms
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//30ms
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (0<<WDP0);//60ms
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0);//120ms
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0);//240ms
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);//480ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);//960ms
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);//2s
   //WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//4s
  // WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s
  sei();
  /* Enable the WD interrupt (note no reset). */
//  WDTCSR |= _BV(WDIE);
  set_sleep_mode (SLEEP_MODE_IDLE);
//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  WDhappen=0;
 sleeps=0;
 TCNT1=0;

do{  
  /* Now enter sleep mode. */
//  sleep_mode();// sleep_enable+sleep_cpu+sleep_disable
  sleep_cpu();
  sleeps++;
  /* The program will continue from here after the WDT timeout*/
  //sleep_disable(); /* First thing to do is disable sleep. */
  
  
  /*
  wdt_enable(WDTO_30MS);
  wdt_reset();

  set_sleep_mode (SLEEP_MODE_IDLE);
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  TCNT1=0;
  sei();
  sleep_cpu();
  sei();
*/
if(WDhappen){break;}
}while(1);
  // wake up here
  t1111=TCNT1;
  sleep_disable();
  wdt_disable();
  
  //digitalWrite(RST,LOW);
//  digitalWrite(RST,HIGH);

 // sa("wakeup");delay(3000);

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

