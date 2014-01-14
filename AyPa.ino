//#define __AVR_ATmega328P__ 
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>



//#include <SD.h>



//#include <DS1302.h>// cannot sit on SPI pins (leaves pin in input state)

#include <SPI.h> // < declaration ShiftOut etc problem

//#include <TFT.h> // Hardware-specific library

//#define CS   0
//#define DC   1
//#define RESET  8  

//TFT myScreen = TFT(CS, DC, RESET);
//TFT myScreen = TFT(0, 1, 8);


#include "AyPa_m.h"
#include "AyPa_fonts.h"
#include "AyPa_n.h"
#include "AyPa_rtc.h"


// set up variables using the SD utility library functions:
//Sd2Card card;
//SdVolume volume;
//SdFile root;
// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
const int chipSelect = 4;    


int freeRam(void)
{
  extern unsigned int __heap_start;
  extern void *__brkval;
  int v;
  return (int)&v-(__brkval==0?(int)&__heap_start:(int)__brkval);
}


//DS1302 rtc(A4,A2,A3);//ce data clk




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


#define SetADC(bandgap,input,us){ADMUX=(bandgap<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input;delayMicroseconds(us);} // input (0..7,8,14) (bg/vcc analogReference )

/*
void SetADCinputChannel(boolean REFS1bit,uint8_t input,uint16_t us)
{
//  ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input; // input (0..7) (default VCC reference)
  ADMUX = (REFS1bit<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input; // input (0..7) (1.1v analogReference reference)
  if(us)delayMicroseconds(us); // Wait for input channel to settle (300us)
}*/
#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 


#define tRST 8
#define tCE 2 // don't go along with CErtc but works on PIN4!
#define tDC 5
#define tDIN 11
#define tCLK 13



// port B: 5 port C: 8 port D:11
byte pp[10]={5,5,5,5,5,5,5,5,5,5}; //port
byte pb[10]={1,0,0,0,0,0,0,0,0,0}; // bit in port
// the setup routine runs once when you press reset:
byte v;
void setup() {                
  byte pmask,idx;
  
  wdt_disable();


//  myScreen.begin();  
  //myScreen.background(0,0,0);  // clear the screen with black
  //delay(2000);  // pause for dramatic effect

// check setup function
   PORTC=0xF;//  set pins 0123 HIGH (internal pullups) //digitalWrite(A1,HIGH);digitalWrite(A2,HIGH);digitalWrite(A3,HIGH);digitalWrite(A4,HIGH);
//  pinMode(A1,INPUT_PULLUP); pinMode(A2,INPUT_PULLUP);  pinMode(A3,INPUT_PULLUP);  pinMode(A4,INPUT_PULLUP);   same same
delay(1);v=(~PINC)&0x0F;

  if (v>0){// 0 all are OFF // 1 1st is ON // 4 3rd is ON // C 3&4 are ON // F all are ON  
//    setup function v (1..0xF) is called
//idx=0; // lamp index
//pmask=pp[0];//read port mask
    Pin2Output(DDRB,pb[0]);
 if(v==1)
{

}// if 1
else if (v==2)
{

  do{
__asm__ __volatile__ (
"ldi r24,0b0000010\n\t" // bit mask
"in r25,5\n\t" // read portb
"mov r23,r25\n\t"// r23 OFF mask
"or r25,r24\n\t" // ON mask
"ldi r22,0\n\t"  // 256 cycles
"cli\n\t"
"1:\n\t"

"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF
"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF


"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF
"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF


"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF
"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF


"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF
"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF


"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF
"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF


"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF
"out 5,r25\n\t"// ON
"out 5,r25\n\t"// ON
"out 5,r23\n\t"// OFF
"out 5,r23\n\t"// OFF


"dec r22\n\t"
"brne 1b\n\t"
"sei\n\t"
::"r"(idx));
}while(1);

}

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
DDRC=0xF;//  pinMode(A1,OUTPUT);  pinMode(A2,OUTPUT);  pinMode(A3,OUTPUT);  pinMode(A4,OUTPUT);
//DDRC=0; //input



  // initialize the digital pin as an output.
  //  pinMode(led, OUTPUT);  
  //  digitalWrite(A5,HIGH); pullup resistor
  // pinMode(9, OUTPUT); //ACS712 module

  // digitalWrite(9,HIGH);// lcd+acs712 

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

if(st2>st1){delta=(uint16_t)(st2-st1-4);}else{delta=st2+65536-st1-4;}//delta-=4;//correction

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


word sc[16];
word mn=5555,mx=5000;
word sleeps;
uint16_t t1,t2,tt1,tt2,ttt1,ttt2;

// the loop routine runs over and over again forever:
void loop() {
  word t,n;
word Temp;

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



    Pin2Input(DDRC,5); //pinMode(A5,INPUT);
    Pin2Input(DDRD,7); //D7 AIN1



  //analogReference(INTERNAL);// just for analogRead (SetADCinputChannel set it up for mRawADC)
    // setup analog comparator&ADC
  ADCSRA|=(1<<ADEN); //turn on ADC    
  DIDR0=(1<<ADC0D);// disable digital input on A0 pin
  ADCSRB = 0;
  DIDR1 = (1<<AIN1D); // AIN1 goes to analog comparator negative's input    so switch off digital input (+1.1 bandgap voltage is on positive input)
  ACSR = (1<<ACBG); //bandgap instead of AIN0  + turn on analog comparator
//  ACSR&=~(1<<ACD); //turn on analog comparator
  SetADC(1,8,500);  //  select temperature sensor 352 (need calibration)

//  SetADCinputChannel(0,500);  //  delay(1); // to avoid false interrupt due bandgap voltage settle

  //  ACSR = (1<<ACI)|(1<<ACBG)|(1<<ACIE)|(1<<ACIS1)|(0<<ACIS0); //bandgap instead of AIN0 int on falling edge 
  //  ACSR = (1<<ACBG)|(1<<ACIE)|(0<<ACIS1)|(0<<ACIS0); //bandgap instead of AIN0 int on toggle
  //  ACSR = (1<<ACBG); //bandgap instead of AIN0 int on toggle

  mRawADC(Temp,2);
  mRawADC(Temp,2);
  
 // SetADC(1,5,500); // select A5

 // mRawADC(t,2);





    Pin2Output(DDRB,1);//  pinMode(9,OUTPUT);


  SetADC(1,5,500); // select A5 (1.1v reference)


  //SPI.begin();//  InitSPI();
  
  LcdInit();



 // Pin2HIGH(PORTD,0); //vcc to A


byte val=0;

word bb,aa;
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
 sa("Тестовая АуРа!"); 
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
  sh(v);sa("_");
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
  sw(sleeps);sa("s ");
  sw(aa);sa(" ");




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
cli();TCNT1=0;
    mRawADC(a0,2);
    val=TCNT1;sei();
    mRawADC(a1,2);
    mRawADC(a2,2);
    mRawADC(a3,2);

//    mRawADC(,1)  807..824   5us but 864..896 800ma
//    mRawADC(,2)  807..8124  8us 864 800ma stable <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,
//    mRawADC(,3)  807..818 14us 856..864 800ma



LcdSet(0,4);
    s3(a0);sa(" ");s3(a1);sa(" ");s3(a2);sa(" ");s3(a3);sa("T");s3(Temp);s3(val);sa(" ");
    sh(t);        sh(t&(1<<ACO));    
//    if((t&(1<<ACO))==0){sa(" I too high!");}// if ACO bit is set then the current is withing limits
    if((t&(1<<ACO))==0){sa("*");}// if ACO bit is set then the current is withing limits
    

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
  SetADC(0,14,500);

for(int e=0;e<10;e++)
{
  mRawADC(a1,2);

LcdSet(6,5);

    a2=1103817L/a1;//(1079L*1023) //roughly calibrated value
    sa(" ");s3(a1);sa(" ");sw(a2); //227 4957 5V   230 4892   231 4957   344 3271 3.3v
}  

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

for(long jj=0;jj<100;jj++){  
for(long j=0;j<10;j++){
cli();
//    TCNT1=0;
  //  TCNT2=0;
    Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
    ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;   // NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
    //    delayMicroseconds(1);  
   // Pin2LOW(PORTB,1); //digitalWrite(9,LOW//   
    do{}while(bit_is_set(ADCSRA,ADSC));
    
//    delayMicroseconds(10);  
  //  Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
 //   ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;
  //  NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;
    //NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;    NOP;

   // do{}while(bit_is_set(ADCSRA,ADSC));
  // v3=ADCW; 


//    delayMicroseconds(1);  



    Pin2LOW(PORTB,1); //digitalWrite(9,LOW//

    tq[j]=ADCW; 
    

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

sei();
    delayMicroseconds(1000);      
    
}//for 10
}//for jj



LcdSet(0,0);
sa(" vb:");s3(v1);s3(v2);
LcdSet(0,1);
for(byte o=0;o<10;o++){s3(tq[o]);sa(" ");}
LcdSet(0,3);
sa(">");sw(1103817L/v2);sa(" 0 ");sw(1103817L/tq[0]);sa(" 9 ");sw(1103817L/tq[9]);sa("<   ");
LcdSet(0,5);
sh(nn>>24);
sh((nn>>16)&0xff);
sh((nn>>8)&0xff);
sh(nn&0xff);

nn++;
}while(1);


/*
for(long j=0;j<10000;j++){
cli();    
    Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
    delayMicroseconds(4);  
    Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
//    delayMicroseconds(10);  
//    Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
//    delayMicroseconds(4);  
//    Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
sei();
    delayMicroseconds(1000);  
}


for(long j=0;j<10000;j++){
cli();
    Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
    delayMicroseconds(12);  
    Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
    delayMicroseconds(10);  
    Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
    delayMicroseconds(12);  
    Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
sei();
    delayMicroseconds(1000);  
}*/






// inner T
// bandgap vs vcc


    // 5V    
    //0ma     816..824    ~820
    //200ma                   ~831 (+11)
    //400ma                   ~842 (+22)
    //600ma                   ~853 (+33)
    //800ma 864..867    ~864 (+44)
    
    // 3.3V khaos.......... ACS712 cannot operate. 4.5V minimum as per datasheet
    // 800ma 847..851 
    // 0ma 825..839 860..865

 //   Pin2LOW(PORTB,2); //digitalWrite(10,LOW);//work with 4051


 // LcdSet(15,5);sa("4");

  
  delay(4000);
  LcdSet(15,5);sa("z");




  
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

    Pin2LOW(PORTB,5); //SPI SCK pin low
    Pin2LOW(PORTB,3); //SPI MOSI pin low

//    Pin2LOW(PORTB,5); //SPI SCK pin low
    Pin2LOW(PORTB,2); //SPI SS pin low

    //Pin2LOW(PORTD,0); //vcc A to low
    
  //  PORTC=0; // switch off PORTC control pins
//    Pin2LOW(PORTC,3); //digitalWrite(1A3HIGH);//  power to current sensor

//PORTC=0x7; // select channel 7 (some unused channel for debug)


  SPCR&=~(1<<SPE); //  SPI.end();
  ADCSRA&=~(1<<ADEN); //turn off ADC 
  ACSR = (1<<ACD); // turn off analog comparator
  

  //set_sleep_mode (SLEEP_MODE_IDLE);// 29.6ma - don't work
  //set_sleep_mode (SLEEP_MODE_ADC);// 6.3ma clock is off
  //set_sleep_mode (SLEEP_MODE_PWR_SAVE);// 3.7ma
  //set_sleep_mode (SLEEP_MODE_STANDBY);// 2.7ma 
  //set_sleep_mode (SLEEP_MODE_PWR_DOWN);// 0.76ma


  
 // Setup the WDT 
  cli();wdt_reset();
   MCUSR &= ~(1<<WDRF);  // Clear the reset flag. 
   WDTCSR |= (1<<WDCE) | (1<<WDE); //  In order to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles).
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//15ms (16280us)
  // WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//30ms
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (0<<WDP0);//60ms
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0);//120ms
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0);//240ms
   //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);//480ms
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);//960ms
   WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);//2s
  // WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);//4s
//   WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);//8s
  sei();
  set_sleep_mode (SLEEP_MODE_IDLE);
//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  WDhappen=0;sleeps=0;TCNT1=0;do{sleep_cpu();sleeps++;if(WDhappen){break;}}while(1); // 9 times within 16ms
  // wake up here
  cli();t1111=TCNT1;sei();
  sleep_disable();
  wdt_disable();
  
  
  //delay(1000);               // wait for a second
}

