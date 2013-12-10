

#define CErtc 4 //CE of DS1302 and of Nokia3110 are compliment each other nicely!
#define CLKrtc 3//13//7
#define IOrtc 2//12//11 //6

//CLKrtc IOrtc
void ShiftOut(byte val)
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

PORTC&=~(1<<CLKrtc);//clk low (needed)
//"=r" (result): "I" (val)

}
/*
void ShiftOutOld(byte val)
{
//PORTC&=~(1<<CLKrtc);//clk low (already)

//PORTC|=(1<<CLKrtc);// tick clk    //8
//if(PINC&(1<<IOrtc))val|=(1<<5);   //6
//PORTC&=~(1<<CLKrtc);//clk low

PORTC&=~(1<<IOrtc);//clear data bit
if (val&(1<<(0)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

PORTC&=~(1<<CLKrtc);//clk low
PORTC&=~(1<<IOrtc);//clear data bit
if (val&(1<<(1)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

PORTC&=~(1<<CLKrtc);//clk low
PORTC&=~(1<<IOrtc);//clear data bit
if (val&(1<<(2)))PORTC|=(1<<IOrtc);// set it if needed
PORTC|=(1<<CLKrtc);// tick clk

PORTC&=~(1<<CLKrtc);//clk low
PORTC&=~(1<<IOrtc);//clear data bit
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

PORTC&=~(1<<CLKrtc);//clk low (needed)

//"=r" (result): "I" (val)
}*/

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

//encode as sbi cbi asm directly. those macro are so buggy!!!!!!!!!!!  
  //Pin2Input(DDRB,5);
//  Pin2Output(DDRB,3);
//  Pin2Input(DDRD,6);
//  Pin2Output(DDRD,7);
  
// sbic - skip if bit in io register cleared
__asm__ __volatile__(
"clr r24\n\t"

//"ldi r25,2\n\t""1:\n\t""dec r25\n\t""brne 1b\n\t"

//PORTC|=(1<<CLKrtc);// tick clk
//if(PINC&(1<<IOrtc))val|=(1<<5);
//PORTC&=~(1<<CLKrtc);//clk low

"sbi 0x08,3\n\t"
"sbic 0x06,2\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x01\n\t"
"cbi 0x08,3\n\t"

"sbi 0x08,3\n\t"
"sbic 0x06,2\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x02\n\t"
"cbi 0x08,3\n\t"

"sbi 0x08,3\n\t"
"sbic 0x06,2\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x04\n\t"
"cbi 0x08,3\n\t"

"sbi 0x08,3\n\t"
"sbic 0x06,2\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x08\n\t"
"cbi 0x08,3\n\t"

"sbi 0x08,3\n\t"
"sbic 0x06,2\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x10\n\t"
"cbi 0x08,3\n\t"

"sbi 0x08,3\n\t"
"sbic 0x06,2\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x20\n\t"
"cbi 0x08,3\n\t"

"sbi 0x08,3\n\t"
"sbic 0x06,2\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x40\n\t"
"cbi 0x08,3\n\t"

"sbi 0x08,3\n\t"
"sbic 0x06,2\n\t"//"in r24,9\n\t"//"sbrc r24,3\n\t"
"ori r24,0x80\n\t"
"cbi 0x08,3\n\t"

: "=r" (val)::);

return val;
}

//23us
/*
void rtcpoke(byte addr,byte val)//7574
{
addr=addr+addr+192;
PORTC&=~(1<<CLKrtc);//digitalWrite(CLKrtc,LOW); //4
PORTC|=(1<<CErtc);//digitalWrite(CErtc,HIGH); //D7
 DDRC|=(1<<IOrtc);//(9clocks vs 1)//set bit IOrtc in DDRC//pinMode(IOrtc,OUTPUT);
ShiftOut(addr);//shiftOut(IOrtc,CLKrtc,LSBFIRST,addr);//253us
ShiftOut(val);//shiftOut(IOrtc,CLKrtc,LSBFIRST,val);//253us
 DDRC&=~(1<<IOrtc);//(9clocks vs 1)// clear bit IOrtc in DDRC//pinMode(IOrtc,INPUT);
PORTC&=~(1<<CErtc);//digitalWrite(CErtc,LOW);//D7  
}*/

//21us as a macro (20 bytes)//7556(18 bytes less than function if called once)

#define rtcpoke(addr,val)\
PORTC&=~(1<<CLKrtc);\
PORTC|=(1<<CErtc);\
ShiftOut(addr+addr+192);\
ShiftOut(val);\
PORTC&=~(1<<CErtc);\

// DDRB|=(1<<IOrtc);\
// DDRB&=~(1<<IOrtc);\

/*Question: When I do the following:
asm volatile("sbi 0x15,0x07;");
everything is OK! But when I do the same but replacing the address of the port by its label, like:
asm volatile("sbi PORTB,0x07;");
I get a compilation error: "Error: constant value required".
asm volatile("sbi PORTB,0x07;");
Answer: PORTB is a precompiler definition included in the processor specific file included in io-avr.h.
As you may know, the precompiler will not touch strings and PORTB is passed to the assembler. One
way to avoid this problem is:
asm volatile("sbi %0, 0x07" : "I" ((unsigned short)(PORTB)):);*/

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
PORTC&=~(1<<CLKrtc);//digitalWrite(CLKrtc,LOW); 
PORTC|=(1<<CErtc);//digitalWrite(CErtc,HIGH);
ShiftOut(addr);//shiftOut(IOrtc,CLKrtc,LSBFIRST,addr);
DDRC&=~(1<<IOrtc);//(9clocks vs 1)// clear bit IOrtc in DDRC//pinMode(IOrtc,INPUT);
val=ShiftIn();
DDRC|=(1<<IOrtc);//(9clocks vs 1)//set bit IOrtc in DDRC//pinMode(IOrtc,OUTPUT);
PORTC&=~(1<<CErtc);//digitalWrite(CErtc,LOW);// acts as reset otherwise next command is broken
return val;
}

char buf[64]; // carefully with long vars. 2k ram only

/*
void rtcgettime(byte n)
{
PORTC&=~(1<<CLKrtc);//digitalWrite(CLKrtc,LOW); 
PORTB|=(1<<CErtc);//digitalWrite(CErtc,HIGH);
DDRB|=(1<<IOrtc);//(9clocks vs 1)//set bit IOrtc in DDRC//pinMode(IOrtc,OUTPUT);
ShiftOut(191);
DDRB&=~(1<<IOrtc);//(9clocks vs 1)// clear bit IOrtc in DDRC//pinMode(IOrtc,INPUT);
for(byte i=0;i<n;i++){buf[i]=ShiftIn();}
PORTB&=~(1<<CErtc);//digitalWrite(CErtc,LOW);// acts as reset otherwise next command is broken  
}*/

//#define rtcgettime(n){PORTC&=~(1<<CLKrtc);PORTB|=(1<<CErtc);DDRB|=(1<<IOrtc);ShiftOut(191);DDRB&=~(1<<IOrtc);for(byte i=0;i<n;i++){buf[i]=ShiftIn();}PORTB&=~(1<<CErtc);}
#define rtcgettime(n){PORTC&=~(1<<CLKrtc);PORTC|=(1<<CErtc);ShiftOut(191);DDRC&=~(1<<IOrtc);for(byte i=0;i<n;i++){buf[i]=ShiftIn();}DDRC|=(1<<IOrtc);PORTC&=~(1<<CErtc);}
//#define rtcsettime(n){PORTC&=~(1<<CLKrtc);PORTB|=(1<<CErtc);DDRB|=(1<<IOrtc);ShiftOut(192);for(byte i=0;i<n;i++){ShiftOut(buf[i]);};DDRB&=~(1<<IOrtc);PORTB&=~(1<<CErtc);}

/*
   rtc.halt(false);
   rtc.writeProtect(false);
   rtc.setDOW(SUNDAY);        // Set Day-of-Week to FRIDAY
   rtc.setTime(0, 30, 0);     // Set the time to 12:00:00 (24hr format)
   rtc.setDate(8, 12, 2013);   // Set the date to August 6th, 2010
*/

#define rtcwriteregister(r,v){PORTC&=~(1<<CLKrtc);PORTC|=(1<<CErtc);ShiftOut(r);ShiftOut(v);PORTC&=~(1<<CErtc);}
#define rtcwriteprotect(t){rtcwriteregister(0x8E,(t<<7))}//reg_wp=0x8E
#define rtcsetDOW(v){rtcwriteregister(0x8A,v)}//reg_dow=0x8A
#define rtcsettime(h,m,s){rtcwriteregister(0x84,h) rtcwriteregister(0x82,m) rtcwriteregister(0x80,s)}
#define rtcsetdate(d,m,y){rtcwriteregister(0x86,d) rtcwriteregister(0x88,m) rtcwriteregister(0x8C,y)}

