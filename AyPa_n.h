//nokia 5110 display

// pins layout 

//#define RST 3 (to arduino's RST)
#define CE 0 
#define DC 4
#define DIN 11
#define CLK 13



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

/*
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
}*/

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

/*
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
}*/

//1828us (clocks with /8 prescaler)
void LcdClear(void)
{
  LcdSet(0,0);//for(byte i=0;i<84;i++){sa(" ");} // clear ram manually (1828us)

//for(byte i=0;i<(84);i++){SPDR = 0;while(!(SPSR&(1<<SPIF)));}// 361us - not working..
  //for(byte i=0;i<6;i++){sa("              ");} // (1614us)
}

void LcdInit(void)
{
    Pin2Output(DDRB,2); //SS pin  (SPI depends on this pin)
  Pin2HIGH(PORTB,2); //set SS (10) high (also CE 4051)

  //SPSR = (0 << SPI2X); //4
  //SPCR = (1 << MSTR) | (1 << SPE) |(1<<SPR0);      // enable, master, msb first
  SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first (lcd)
//  SPCR = (1 << MSTR) | (1 << SPE) | (1<<DORD);      // enable, master, lsb first (rtc)
  SPSR = (1 << SPI2X);// 1/2clk
  Pin2Output(DDRB,3); //MOSI pin
  Pin2Output(DDRB,5); //SCK pin
  Pin2Output(DDRD,0);
  Pin2Output(DDRD,4);


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

//cli();
//TCNT1=0;
//  LcdClear();
 // word rr=TCNT1;
//sei();
//delay(2000);//??????????????????????????????????????
//sa("clear=");sw(rr);

}
