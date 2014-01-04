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


// the setup routine runs once when you press reset:
byte v;
void setup() {                
//  byte v;
  wdt_disable();

// check setup function
   PORTC=0xF;//  set pins 0123 HIGH (internal pullups) //digitalWrite(A1,HIGH);digitalWrite(A2,HIGH);digitalWrite(A3,HIGH);digitalWrite(A4,HIGH);
//  pinMode(A1,INPUT_PULLUP); pinMode(A2,INPUT_PULLUP);  pinMode(A3,INPUT_PULLUP);  pinMode(A4,INPUT_PULLUP);   same same
delay(1);v=(~PINC)&0x0F;

  if (v>0){// 0 all are OFF // 1 1st is ON // 4 3rd is ON // C 3&4 are ON // F all are ON  
//    setup function v (1..0xF) is called
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


    Pin2Output(DDRB,0); //latch 595


    Pin2Output(DDRD,0); //+vcc A


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
  
  SetADC(1,5,500); // select A5

  mRawADC(t,2);





    Pin2Output(DDRB,1);//  pinMode(9,OUTPUT);

    //Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//

  //digitalWrite(9,HIGH);// acs712 module + lcd
  //digitalWrite(10,HIGH);// 5v mosfet control


// rtc clock pins
//  Pin2Output(DDRC,2);
  //Pin2Output(DDRC,3);
  //Pin2Output(DDRC,4);
  

  //SPI.begin();//  InitSPI();
  
  Pin2HIGH(PORTB,2); //set SS (10) high (also CE 4051)
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



  Pin2HIGH(PORTD,0); //vcc to A


byte val=0;

word bb,aa;
  SetADC(1,5,500); // select A5 (1.1v reference)
//  mRawADC(bb,2);//1023 (>1.1v)
//  mRawADC(bb,2);


//PORTC=0x7; // select channel 7
//PORTC=0b0000001; // select channel 1
PORTC=0b0000000; // select channel 0
    Pin2HIGH(PORTC,3); //digitalWrite(1A3HIGH);//  power to current sensor

delay(10);
  mRawADC(aa,2); // 688 imm 711--718 after 10ms
  mRawADC(aa,2); // after powering up current sensor 

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

// 5x5
for(int k=0;k<15000;k++)
{
    cli();
    TCNT2=0;
    Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
    {}while(TCNT2<5);
    Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
    //TCNT2=0;
    sei();
    {}while(TCNT2<10);
}

//5x15
for(int k=0;k<15000;k++)
{
    cli();
    TCNT2=0;
    Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
    {}while(TCNT2<5);
    Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
    //TCNT2=0;
    sei();
    {}while(TCNT2<20);
}


//5x55
for(int k=0;k<15000;k++)
{
    cli();
    TCNT2=0;
    Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
    {}while(TCNT2<5);
    Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
    //TCNT2=0;
    sei();
    {}while(TCNT2<55);
}

//5x240
for(int k=0;k<1500;k++)
{
    cli();
    TCNT2=0;
    Pin2HIGH(PORTB,1); //digitalWrite(9,HIGH);//
    {}while(TCNT2<5);
    Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
    //TCNT2=0;
    sei();
    {}while(TCNT2<245);
}

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

    Pin2LOW(PORTB,2); //digitalWrite(10,LOW);//work with 4051



  
  delay(3000);
sa("z");



  
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
    Pin2HIGH(PORTB,2);//digitalWrite(10,HIGH//
  
  //  Pin2LOW(PORTB,1); //digitalWrite(9,LOW//
//    Pin2Input(DDRB,1);//  pinMode(9,INPUT);

    Pin2LOW(PORTB,5); //SPI SCK pin low
    Pin2LOW(PORTB,3); //SPI MOSI pin low

    Pin2LOW(PORTD,0); //vcc A to low
    
    PORTC=0; // switch off PORTC control pins
//    Pin2LOW(PORTC,3); //digitalWrite(1A3HIGH);//  power to current sensor

PORTC=0x7; // select channel 7 (some unused channel for debug)


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

