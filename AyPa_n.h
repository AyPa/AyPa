//nokia 3110/5110 display

// pins layout 

//#define RST 3 (to arduino's RST)
#define CE 1 
#define DC 4
#define DIN 11
#define CLK 13

/*
void LcdWriteCmd1(byte cmd)
{

  digitalWrite(DC,LOW);
  digitalWrite(CE,LOW);

  for(byte r=0;r<8;r++)
  {
      digitalWrite(DIN,LOW);
      if(cmd&(1<<(7-r)))      digitalWrite(DIN,HIGH);

    
  digitalWrite(CLK,LOW);
  digitalWrite(CLK,HIGH);
    
  }
  

  digitalWrite(CE,HIGH);  
}*/

#define spiwrite(c){  SPDR = c;  while(!(SPSR&(1<<SPIF)));}

void LcdWriteCmd(uint8_t cmd)
{

    
  Pin2LOW(PORTD,DC);//digitalWrite(DC,LOW);
  Pin2LOW(PORTD,CE);//  digitalWrite(CE,LOW);

  spiwrite(cmd);
//  SPDR = cmd;// start transfer
  //while(!(SPSR&(1<<SPIF)));// interrupt also can!

  Pin2HIGH(PORTD,CE);//  digitalWrite(CE,HIGH);  
}
/*
void LcdWriteData1(byte cmd)
{

  digitalWrite(DC,HIGH);
  digitalWrite(CE,LOW);

  for(byte r=0;r<8;r++)
  {
      digitalWrite(DIN,LOW);
      if(cmd&(1<<(7-r)))      digitalWrite(DIN,HIGH);

    
  digitalWrite(CLK,LOW);
  digitalWrite(CLK,HIGH);
    
  }
  

  digitalWrite(CE,HIGH);  
}*/

void LcdWriteData(uint8_t cmd)
{
  //SPI.setDataMode(SPI_MODE0);//default
  //SPI.setBitOrder(MSBFIRST);// maybe
  //SPI.setClockDivider(SPI_CLOCK_DIV2);//max
//  SPSR = (1 << SPI2X);//2
  //SPSR = (0 << SPI2X); //4
  //SPCR = (1 << MSTR) | (1 << SPE) |(1<<SPR0);      // enable, master, msb first
//  SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first

  Pin2HIGH(PORTD,DC);//  digitalWrite(DC,HIGH);
  Pin2LOW(PORTD,CE);///  digitalWrite(CE,LOW);

  spiwrite(cmd);
//  SPDR = cmd;// start transfer
//  while(!(SPSR&(1<<SPIF)));// interrupt also can!

  Pin2HIGH(PORTD,CE);///  digitalWrite(CE,HIGH);  
}


void dd(uint8_t ch)
{
  uint8_t c;
  
  //for(byte j=0;j<24;j++)
  spiwrite(0x00);// 1st space

  for(uint8_t j=0;j<3;j++)  // display digit
  {
    c=pgm_read_byte(&(Dig[ch++]));
    spiwrite(c);
  }
}

/*
void dcn(byte ch)
{
  byte c;
  
  //for(byte j=0;j<24;j++)
//  spiwrite(0x00);// 1st space

  for(byte j=0;j<3;j++)  // display char
  {
    c=pgm_read_byte(&(Dig[ch++]));
    spiwrite(c);
  }
}*/

void ts(uint8_t ch)
{
  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1);
  spiwrite(ch);
  Pin2HIGH(PORTD,1);
}

void tc(uint8_t ch)
{
  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1);
  dd(ch+ch+ch);
  Pin2HIGH(PORTD,1);
}

void tn(long s, long v)
{
  uint8_t c,ch;
  long vv=v;  
  
  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1); ///digitalWrite(ce LOW);//3
  
  for(long n=s;n>0;n/=10){ch=vv/n;vv-=ch*n;dd(ch+ch+ch);}
    
  Pin2HIGH(PORTD,1);//    digitalWrite(ce,HIGH);
}

void tf(long s, long v,uint8_t d)
{
  uint8_t c,ch;
  uint8_t p=0;
  long vv=v;  
  
  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1); 
  
//  for(long n=s;n>0;n/=10){ch=vv/n;vv-=ch*n;dd(ch+ch+ch);if(p){p=0;spiwrite(0x00);spiwrite(0x60);spiwrite(0x60);}}
  for(long n=s;n>0;n/=10){ch=vv/n;vv-=ch*n;dd(ch+ch+ch);if(++p==d){spiwrite(0x00);spiwrite(0x40);}}
    
  Pin2HIGH(PORTD,1);
}

void ta(char *st)
{
  uint8_t l=0,c;
  uint16_t ch;
  
  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1); ///digitalWrite(cs, LOW);//3
  
    do{
    //    LcdWriteData(0);//space  (start with it - while it is sending can calc address)

   //   for(byte j=0;j<24;j++)s
   spiwrite(0x00);// 1st space
//    SPDR = 0;// start transfer with space (while it is sending can calc address)
    //calcs
    c=st[l++];if (c>127){c=st[l++];}// 16bit code
    ch=(c-32)*5;
    //    c=Rus[ch++];//preload next char

for(uint8_t j=0;j<5;j++)  // display char
{
    c=pgm_read_byte(&(Rus[ch++]));
    spiwrite(c);
//  for(byte i=0;i<8;i++)
  //{
//    if(c&0x01){      spiwrite(0xFC);spiwrite(0xFC);spiwrite(0xFC);}
//    else{      spiwrite(0x00);spiwrite(0x00);spiwrite(0x00);}
//    c=c>>1;
//  }
}



    

  }
  while (st[l]!=0);//same same

  Pin2HIGH(PORTD,1); // digitalWrite(CE,HIGH);    
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

//void LcdSet(byte x,byte y)
//{
  //LcdWriteCmd(0b10000000|x*5);//set X (0..83)
//  LcdWriteCmd(0b01000000|y);//set Y (0..5)
//}
void LcdSetPos(uint8_t x,uint8_t y)
{
  LcdWriteCmd(0b10000000|x);//set X (0..83)
  LcdWriteCmd(0b01000000|y);//set Y (0..5)
}

// 89 clocks
// integer 3 digits representation 
void s3(uint16_t v)
{
  uint8_t c,ch;

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
void s2(uint16_t v)
{
  uint8_t c,ch;

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


void th(uint8_t v)
{
  uint8_t c,ch;

  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1); ///digitalWrite(cs, LOW);//3


  ch=(v>>4)*3;
//  for(byte j=0;j<24;j++)
spiwrite(0x00);// 1st space

for(uint8_t j=0;j<3;j++)  // display digit
{
  c=pgm_read_byte(&(Dig[ch++]));
spiwrite(c);
//  for(byte i=0;i<8;i++)
//  {
//    if(c&0x01){      spiwrite(0xFC);spiwrite(0xFC);spiwrite(0xFC);}
//    else{      spiwrite(0x00);spiwrite(0x00);spiwrite(0x00);}
//    c=c>>1;
 // }
}

//  for(byte j=0;j<24;j++)
spiwrite(0x00);// 1st space

  ch=(v&0xF)*3;

for(uint8_t j=0;j<3;j++)  // display digit
{
  c=pgm_read_byte(&(Dig[ch++]));
spiwrite(c);
//  for(byte i=0;i<8;i++)
  //{
    //if(c&0x01){      spiwrite(0xFC);spiwrite(0xFC);spiwrite(0xFC);}
   // else{      spiwrite(0x00);spiwrite(0x00);spiwrite(0x00);}
   // c=c>>1;
  //}
}

  Pin2HIGH(PORTD,1);//    digitalWrite(cs,HIGH);
}
void lh(long v)
{
  th(v>>24);
  th((v>>16)&0xFF);
  th((v>>8)&0xFF);
  th(v&0xFF);
}
void wh(uint16_t v)
{
  th((v>>8)&0xFF);
  th(v&0xFF);
}

// 164 clocks
// integer word representation 
void sw(uint16_t v)
{
  uint8_t c,ch;

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
void sh(uint8_t v)
{
  uint8_t c,ch;

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
void sb(uint8_t v)
{
  uint8_t c,ch,i=7;

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
  uint8_t i=0,c;
  uint16_t ch;

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
  LcdSetPos(0,0);//for(byte i=0;i<84;i++){sa(" ");} // clear ram manually (1828us)



//for(byte i=0;i<(84);i++){SPDR = 0;while(!(SPSR&(1<<SPIF)));}// 361us - not working..
  for(uint8_t i=0;i<6;i++){ta("              ");} // (1614us)
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
  Pin2Output(DDRD,1);
  Pin2Output(DDRD,4);


  //LcdWriteCmd(0x21);
//  LcdWriteCmd(0xB8);
  //LcdWriteCmd(0x07);
//  LcdWriteCmd(0x14);//bias(best)
  //LcdWriteCmd(0x20); // extended instruction set control (H=0)
  //LcdWriteCmd(0x09); // all display segments on
  //LcdWriteCmd(0x20); // extended instruction set control (H=0)
  //LcdWriteCmd(0x08); // all segments off - need to clear all ram on start
  //LcdWriteCmd(0x20);
  //LcdWriteCmd(0x0D); // invert
  
  //LcdWriteCmd(0x20); // extended instruction set control (H=0)
//  LcdWriteCmd(0x0c); // LCD in normal mode (0x0d = inverse mode)

  LcdWriteCmd(0x21);//0x21, //switch to extended commands
    LcdWriteCmd(0xB8);
      LcdWriteCmd(0x04); //set temperature coefficient
        LcdWriteCmd(0x14); //set bias mode to 1:48.
          LcdWriteCmd(0x20); //switch back to regular commands
            LcdWriteCmd(0x0C);//enable normal display (dark on light), horizontal addressing


//cli();
//TCNT1=0;
//  LcdClear();
 // word rr=TCNT1;
//sei();
//delay(2000);//??????????????????????????????????????
//sa("clear=");sw(rr);

}

/*
void LcdInitN(void)
{
  uint8_t i;
    Pin2Output(DDRB,2); //SS pin  (SPI depends on this pin)
  Pin2HIGH(PORTB,2); //set SS (10) high (also CE 4051)

  //SPSR = (0 << SPI2X); //4
  //SPCR = (1 << MSTR) | (1 << SPE) |(1<<SPR0);      // enable, master, msb first
  SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first (lcd)
//  SPCR = (1 << MSTR) | (1 << SPE) | (1<<DORD);      // enable, master, lsb first (rtc)
  SPSR = (1 << SPI2X);// 1/2clk
  Pin2Output(DDRB,3); //MOSI pin
  Pin2Output(DDRB,5); //SCK pin
  Pin2Output(DDRD,1);
  Pin2Output(DDRD,4);


  LcdWriteCmd(0x21);
  
  
  LcdWriteCmd(0xB9);//SETEXTC
	LcdWriteData(0xFF);
	LcdWriteData(0x83);
	LcdWriteData(0x53);

LcdWriteCmd(0xB0);//RADJ
        LcdWriteData(0x3C);
        LcdWriteData(0x01);

	
LcdWriteCmd(0xB6);//VCOM
	LcdWriteData(0x94);

	LcdWriteData(0x6C);

	LcdWriteData(0x50);   

	

	LcdWriteCmd(0xB1);//PWR

	LcdWriteData(0x00);

	LcdWriteData(0x01);

	LcdWriteData(0x1B);

	LcdWriteData(0x03);

	LcdWriteData(0x01);

	LcdWriteData(0x08);

	LcdWriteData(0x77);

	LcdWriteData(0x89);

	

	LcdWriteCmd(0xE0); //Gamma setting for tpo Panel

	LcdWriteData(0x50);

	LcdWriteData(0x77);

	LcdWriteData(0x40);

	LcdWriteData(0x08);

	LcdWriteData(0xBF);

	LcdWriteData(0x00);

	LcdWriteData(0x03);

	LcdWriteData(0x0F);

	LcdWriteData(0x00);

	LcdWriteData(0x01);

	LcdWriteData(0x73);

	LcdWriteData(0x00);

	LcdWriteData(0x72);

	LcdWriteData(0x03);

	LcdWriteData(0xB0);

	LcdWriteData(0x0F);

	LcdWriteData(0x08);

	LcdWriteData(0x00);

	LcdWriteData(0x0F);

		

	LcdWriteCmd(0x3A);   

	LcdWriteData(0x05);  //05 

	LcdWriteCmd(0x36);    

	LcdWriteData(0xC0); //83  //0B 

		

	LcdWriteCmd(0x11); // SLPOUT  

	delay(150);	

	LcdWriteCmd(0x29);    // display on



	delay(150);

	LcdWriteCmd(0x2D);  //Look up table

	for(i=0;i<32;i++)

	 {LcdWriteData(2*i);} //Red

	for(i=0;i<64;i++)

	 {LcdWriteData(1*i);} //Green

	for(i=0;i<32;i++)

	 {LcdWriteData(2*i);} //Blue 

	

	LcdWriteCmd(0x2c);  

	delay(150);	

//fill rect
//setaddrwindow
	LcdWriteCmd(0x2a);  // caset
	LcdWriteData(0x00);  
	LcdWriteData(0x00);  // x start
	LcdWriteData(0x00);  
	LcdWriteData(0x32);  // x end

	LcdWriteCmd(0x2b);  // raset
	LcdWriteData(0x00);  
	LcdWriteData(0x00);  // y start
	LcdWriteData(0x00);  
	LcdWriteData(0x48);  // y end

	LcdWriteCmd(0x2c);  // ramwr


for(i=0;i<120;i++)
{
	LcdWriteData(0x50);  
}

delay(1000);

}*/
