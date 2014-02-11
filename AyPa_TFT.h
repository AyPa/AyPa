/***************************************************
  This is an example sketch for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// For the breakout, you can use any (4 or) 5 pins
//#define sclk 4
//#define mosi 5
//#define cs   6
//#define dc   7
//#define rst  8  // you can also connect this to the Arduino reset

//Use these pins for the shield!
#define sclk 13
#define mosi 11
#define cs   1
#define dc   4
#define rst  10  // you can also connect this to the Arduino reset
// не работает без reset


//#include <SPI.h>
#define spiwrite(c){  SPDR = c;  while(!(SPSR&(1<<SPIF)));}
void spiwritefunc(byte c)
{
  
//  pinMode(10,OUTPUT);
  //digitalWrite(10,HIGH);
//      Pin2Output(DDRB,2); //SS pin  (SPI depends on this pin)
//  Pin2HIGH(PORTB,2); //set SS (10) high (also CE 4051)

    //SPI.setDataMode(SPI_MODE0);//default
  //SPI.setBitOrder(MSBFIRST);// maybe
  //SPI.setClockDivider(SPI_CLOCK_DIV2);//max
//  SPSR = (1 << SPI2X);//2
  //SPSR = (0 << SPI2X); //4
  //SPCR = (1 << MSTR) | (1 << SPE) |(1<<SPR0);      // enable, master, msb first
//  SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first


  SPDR = c;// start transfer
  while(!(SPSR&(1<<SPIF)));// interrupt also can!
/*
  //  SPI.transfer(c);


      // Fast SPI bitbang swiped from LPD8806 library
    for(byte bit = 0x80; bit; bit >>= 1) {
      if(c & bit) digitalWrite(mosi,HIGH);else digitalWrite(mosi,LOW);
      digitalWrite(sclk,HIGH);
      digitalWrite(sclk,LOW);
    }*/
/*      // Fast SPI bitbang swiped from LPD8806 library
    for(byte bit = 0x80; bit; bit >>= 1) {
      if(c & bit) PORTB |=0b00001000;else PORTB &= 0b11110111;
      PORTB |=0b00100000;
      PORTB &= 0b11011111;
//      digitalWrite(sclk,HIGH);
  //    digitalWrite(sclk,LOW);
    }*/
    
    
   
}
//void writecommand(byte c) {
#define writecommand(c) {Pin2LOW(PORTD,1);Pin2LOW(PORTD,4);spiwrite(c);Pin2HIGH(PORTD,1);}
//#define writecommand(c) {PORTD&=0b11101101;spiwrite(c);PORTD|=0b00000010;}
//void writedata(byte c) {
//#define writedata(c) {PORTD|=0b00010000;  PORTD&=0b11110101;  spiwrite(c);  PORTD|=0b00000010;}
#define writedata(c) {Pin2LOW(PORTD,1);Pin2HIGH(PORTD,4);  spiwrite(c); Pin2HIGH(PORTD,1);}
  // set C/D вынести за скобки

// some flags for initR() :(
#define INITR_GREENTAB 0x0
#define INITR_REDTAB   0x1
#define INITR_BLACKTAB   0x2

#define ST7735_TFTWIDTH  128
#define ST7735_TFTHEIGHT 160

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions
#define	ST7735_BLACK   0x0000
#define	ST7735_BLUE    0x001F
#define	ST7735_RED     0xF800
#define	ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0  
#define ST7735_WHITE   0xFFFF

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t PROGMEM
//    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    9,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      5,                    //   5ms was 150 ms delay
//      150,                    //   5ms was 150 ms delay
    0xB9,3,0xFF,0x83,0x53,
    0xB0,2,0x3C,0x01,
    0xB6,3,0x94,0x6C,0x50,
    0xB1,8,0x00,0x01,0x1B,0x03,0x01,0x08,0x77,0x89,
    0xE0,19,0x50,0x77,0x40,0x08,
    0xBF,0x00,0x03,0xFF,
    0x00,0x01,0x73,0x00,
    0x72,0x03,0xB0,0x0F,
    0x08,0x00,0x0F,
    0x3A,1,0x06,
    0x36,1,0x20, // MADCTL: MXMY=0 MV=1 ML=0 was 0xC0   000  001 010 011 60 100 80 101 A0 110 C0 111 E0
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      5,                    //     was 255 500 ms delay
//      255,                    //     was 255 500 ms delay
//    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
//      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
//    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
//      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
//    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
//      0x01, 0x2C, 0x2D,       //     Dot inversion mode
//      0x01, 0x2C, 0x2D,       //     Line inversion mode
//    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
//      0x07,                   //     No inversion
//    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
//      0xA2,
//      0x02,                   //     -4.6V
//      0x84,                   //     AUTO mode
//    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
//      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
//    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
//      0x0A,                   //     Opamp current small
//      0x00,                   //     Boost frequency
//    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
//      0x8A,                   //     BCLK/2, Opamp current small & Medium low
//      0x2A,  
//    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
//      0x8A, 0xEE,
//    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
//      0x0E,
//    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
//    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
//      0xC8,                   //     row addr/col addr, bottom to top refresh
  //    0xC0,
//    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
//      0x05 // 16bit
//      0x06// 18bit
//      0x29,DELAY,150
      },                 //     16-bit color

  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      1,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
//      10 };                  //     100 ms delay
      5 };                  //     100 ms delay

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void commandList(const byte *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}

word colstart,rowstart;

void commonInit(const uint8_t *cmdList) {
  colstart  = rowstart = 0; // May be overridden in init func

  pinMode(dc, OUTPUT);
  pinMode(cs, OUTPUT);
    pinMode(sclk, OUTPUT);
    pinMode(mosi , OUTPUT);
      digitalWrite(sclk,LOW);
        digitalWrite(mosi,LOW);

  SPSR = (1 << SPI2X);//max speed
//  SPSR = (0 << SPI2X);//max speed/2
  SPCR = (1 << MSTR) | (1 << SPE);      // enable, master, msb first

  // toggle RST low to reset; CS low so it'll listen to us

  digitalWrite(cs,LOW);
  if (rst) {
    pinMode(rst, OUTPUT);
    digitalWrite(rst, HIGH);
    delay(1);
    digitalWrite(rst, LOW);
    delay(1);
    digitalWrite(rst, HIGH);
    delay(1);
  }

  if(cmdList) commandList(cmdList);

}

void InitTFT() {
  commonInit(Rcmd1);
    commandList(Rcmd2red);
  commandList(Rcmd3);
}

void setAddrWindow(byte x0, byte y0, byte x1,byte y1) {

  writecommand(ST7735_CASET); // Column addr set
  writedata(0x00);
  writedata(x0);     // XSTART 
  writedata(0x00);
  writedata(x1);     // XEND

  writecommand(ST7735_RASET); // Row addr set
  writedata(0x00);
  writedata(y0);     // YSTART
  writedata(0x00);
  writedata(y1);     // YEND

  writecommand(ST7735_RAMWR); // write to RAM
}

void ta(char *st)
{
  byte l=0,c;
  word ch;
  
  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1); ///digitalWrite(cs, LOW);//3
  
    do{
    //    LcdWriteData(0);//space  (start with it - while it is sending can calc address)

      for(byte j=0;j<24;j++)spiwrite(0x00);// 1st space
//    SPDR = 0;// start transfer with space (while it is sending can calc address)
    //calcs
    c=st[l++];if (c>127){c=st[l++];}// 16bit code
    ch=(c-32)*5;
    //    c=Rus[ch++];//preload next char

for(byte j=0;j<5;j++)  // display char
{
    c=pgm_read_byte(&(Rus[ch++]));
  for(byte i=0;i<8;i++)
  {
    if(c&0x01){      spiwrite(0xFC);spiwrite(0xFC);spiwrite(0xFC);}
    else{      spiwrite(0x00);spiwrite(0x00);spiwrite(0x00);}
    c=c>>1;
  }
}



    

  }
  while (st[l]!=0);//same same

  PORTD|=(1<<CE);// digitalWrite(CE,HIGH);    
}

void pushColor(byte r,byte g,byte b)
{
    Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1); ///digitalWrite(cs, LOW);//3

spiwrite(r);
spiwrite(g);
spiwrite(b);

  Pin2HIGH(PORTD,1);//    digitalWrite(cs,HIGH);

}


void th(byte v)
{
  byte c,ch;

  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1); ///digitalWrite(cs, LOW);//3


  ch=(v>>4)*3;
  for(byte j=0;j<24;j++)spiwrite(0x00);// 1st space

for(byte j=0;j<3;j++)  // display digit
{
  c=pgm_read_byte(&(Dig[ch++]));
  for(byte i=0;i<8;i++)
  {
    if(c&0x01){      spiwrite(0xFC);spiwrite(0xFC);spiwrite(0xFC);}
    else{      spiwrite(0x00);spiwrite(0x00);spiwrite(0x00);}
    c=c>>1;
  }
}

  for(byte j=0;j<24;j++)spiwrite(0x00);// 1st space

  ch=(v&0xF)*3;

for(byte j=0;j<3;j++)  // display digit
{
  c=pgm_read_byte(&(Dig[ch++]));
  for(byte i=0;i<8;i++)
  {
    if(c&0x01){      spiwrite(0xFC);spiwrite(0xFC);spiwrite(0xFC);}
    else{      spiwrite(0x00);spiwrite(0x00);spiwrite(0x00);}
    c=c>>1;
  }
}

  Pin2HIGH(PORTD,1);//    digitalWrite(cs,HIGH);
}

void dd(byte ch)
{
  byte c;
  
  for(byte j=0;j<24;j++)spiwrite(0x00);// 1st space
  for(byte j=0;j<3;j++)  // display digit
  {
    c=pgm_read_byte(&(Dig[ch++]));
    for(byte i=0;i<8;i++)
    {
      if(c&0x01){      spiwrite(0xFC);spiwrite(0xFC);spiwrite(0xFC);}
      else{      spiwrite(0x00);spiwrite(0x00);spiwrite(0x00);}
      c=c>>1;
    }
  }
}

void tn(long s, long v)
{
  byte c,ch;
  long vv=v;  
  
  Pin2HIGH(PORTD,4); 
  Pin2LOW(PORTD,1); ///digitalWrite(cs, LOW);//3
  
  for(long n=s;n>0;n/=10){ch=vv/n;vv-=ch*n;dd(ch+ch+ch);}
    
  Pin2HIGH(PORTD,1);//    digitalWrite(cs,HIGH);
}


void lh(long v)
{
  th(v>>24);
  th((v>>16)&0xFF);
  th((v>>8)&0xFF);
  th(v&0xFF);
}
void wh(word v)
{
  th((v>>8)&0xFF);
  th(v&0xFF);
}


/* fill rect ? no2
// square
void SQ(byte x0, byte y0, byte x1,byte y1,long color) {
  byte uh,hi,lo,n;
  
  setAddrWindow(x0,y0,x1,y1);
  uh=color>>24;
  hi=((color>>16)&0xff);
  lo=(color&0xff);
  n=(x1-x0)*(y1-y0);
  

Pin2HIGH(PORTD,4); //    digitalWrite(dc, HIGH);//4
Pin2LOW(PORTD,1); ///digitalWrite(cs, LOW);//3

  for(byte i=0;i<n;i++)
  {
  spiwrite(uh);
  spiwrite(hi);
  spiwrite(lo);
  }
  
Pin2HIGH(PORTD,1);//    digitalWrite(cs,HIGH);
  
}*/

/*
void LcdSet(byte x,byte y)
{
  LcdWriteCmd(0b10000000|x*5);//set X (0..83)
  LcdWriteCmd(0b01000000|y);//set Y (0..5)
}*/
//CASET x start x end
//PASET y start y end

void drawPixel(word x, word y, long color) {

//  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

    digitalWrite(dc, HIGH);
    digitalWrite(cs, LOW);

  
  spiwrite(color >>16);
  spiwrite((color >> 8)&0xff);
  spiwrite(color&0xff);

    digitalWrite(cs,HIGH);
}


// fill a rectangle
//void fillRect(word x, word y, word w, word h,long color) {
void fillRect(word y, word x, word h,word w,long color) {

  setAddrWindow(x, y, x+w-1, y+h-1);

  byte hi = (color >> 8)&0xff, lo = color&0xff,uh=color>>16;

Pin2HIGH(PORTD,4); //    digitalWrite(dc, HIGH);//4
Pin2LOW(PORTD,1); ///digitalWrite(cs, LOW);//3

  for(y=h; y>0; y--) {
//    uh+=0x4;
    
    for(x=w; x>0; x--) {
      spiwrite(uh);
      spiwrite(hi);
      spiwrite(lo);
    }
  }

Pin2HIGH(PORTD,1);//    digitalWrite(cs,HIGH);
}
void fillScreen(long color) {
  fillRect(0, 0,  128, 160, color);
//  fillRect(0, 0,  160, 128, color);
}


