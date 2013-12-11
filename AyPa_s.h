
// various handy staff

// chrging capacitor  on D2 pin. when it is discharged   to logic "0" INT0 on low level is fired
//===========================================================================================

byte pin2_interrupt_flag=0;

volatile word  t1111; // vars updated in ISR should be declared as volatile and accessed with cli()/sei() ie atomic

void pin2_isr()
{
  t1111=TCNT1;
  sleep_disable();
  detachInterrupt(0);
  pin2_interrupt_flag = 1;
}


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
  
  //=================================================================================
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

//=========================================================================================

// struct of defined type in progmem

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
//==========================================================================================
#define loop_until_bit_is_clear(port, bitn)\
__asm__ __volatile__ ("nop\n\t""nop\n\t"\
"L_%=: " "sbic %0, %1" "\n\t"\
"rjmp L_%="\
: /* no outputs */\
: "I" ((uint8_t)(port)),\
"I" ((uint8_t)(bitn))\
)


  

