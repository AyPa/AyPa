
#define TSL2561_ADDR_LOW_W 82
#define TSL2561_ADDR_LOW_R  83 //(0x29<<1)+1

#define TSL2561_READBIT           (0x01)
#define TSL2561_COMMAND_BIT       (0x80)    // Must be 1
#define TSL2561_CLEAR_BIT         (0x40)    // Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT          (0x20)    // 1 = read/write word (rather than byte)
#define TSL2561_BLOCK_BIT         (0x10)    // 1 = using block read/write
#define TSL2561_CONTROL_POWERON   (0x03)
#define TSL2561_CONTROL_POWEROFF  (0x00)

enum
{
  TSL2561_REGISTER_CONTROL          = 0x00,
  TSL2561_REGISTER_TIMING           = 0x01,
  TSL2561_REGISTER_THRESHHOLDL_LOW  = 0x02,
  TSL2561_REGISTER_THRESHHOLDL_HIGH = 0x03,
  TSL2561_REGISTER_THRESHHOLDH_LOW  = 0x04,
  TSL2561_REGISTER_THRESHHOLDH_HIGH = 0x05,
  TSL2561_REGISTER_INTERRUPT        = 0x06,
  TSL2561_REGISTER_CRC              = 0x08,
  TSL2561_REGISTER_ID               = 0x0A,
  TSL2561_REGISTER_CHAN0_LOW        = 0x0C,
  TSL2561_REGISTER_CHAN0_HIGH       = 0x0D,
  TSL2561_REGISTER_CHAN1_LOW        = 0x0E,
  TSL2561_REGISTER_CHAN1_HIGH       = 0x0F
};

typedef enum
{
  TSL2561_INTEGRATIONTIME_13MS      = 0x00,    // 13.7ms
  TSL2561_INTEGRATIONTIME_101MS     = 0x01,    // 101ms
  TSL2561_INTEGRATIONTIME_402MS     = 0x02     // 402ms
}
tsl2561IntegrationTime_t;

typedef enum
{
  TSL2561_GAIN_0X                   = 0x00,    // No gain
  TSL2561_GAIN_16X                  = 0x10,    // 16x gain
}
tsl2561Gain_t;

//#include <DS1302.h>// cannot sit on SPI pins (leaves pin in input state)
//#include <DS1307.h>// cannot sit on SPI pins (leaves pin in input state)

//#define DS1307_ADDR 0xD0
#define DS1307_ADDR_R	209
#define DS1307_ADDR_W	208

#define DS1307_SQW_RATE_1		0x10
#define DS1307_SQW_RATE_4K		0x11
#define DS1307_SQW_RATE_8K		0x12
#define DS1307_SQW_RATE_32K  	0x13

#define _sda_pin A4
#define _scl_pin A5



void	_sendStart(byte addr)
{
	pinMode(_sda_pin, OUTPUT);
	digitalWrite(_sda_pin, HIGH);
	digitalWrite(_scl_pin, HIGH);
	digitalWrite(_sda_pin, LOW);
	digitalWrite(_scl_pin, LOW);
	shiftOut(_sda_pin, _scl_pin, MSBFIRST, addr);
}

void	_sendStop()
{
	pinMode(_sda_pin, OUTPUT);
	digitalWrite(_sda_pin, LOW);
	digitalWrite(_scl_pin, HIGH);
	digitalWrite(_sda_pin, HIGH);
	pinMode(_sda_pin, INPUT);
}

void	_sendNack()
{
	pinMode(_sda_pin, OUTPUT);
	digitalWrite(_scl_pin, LOW);
	digitalWrite(_sda_pin, HIGH);
	digitalWrite(_scl_pin, HIGH);
	digitalWrite(_scl_pin, LOW);
	pinMode(_sda_pin, INPUT);
}

void	_sendAck()
{
	pinMode(_sda_pin, OUTPUT);
	digitalWrite(_scl_pin, LOW);
	digitalWrite(_sda_pin, LOW);
	digitalWrite(_scl_pin, HIGH);
	digitalWrite(_scl_pin, LOW);
	pinMode(_sda_pin, INPUT);
}

void	_waitForAck()
{
	pinMode(_sda_pin, INPUT);
	digitalWrite(_scl_pin, HIGH);
TCNT1=0;
	while (_sda_pin==LOW) {if(TCNT1>100){break;}}
	digitalWrite(_scl_pin, LOW);
}

uint8_t _readByte()
{
	pinMode(_sda_pin, INPUT);

	uint8_t value = 0;
	uint8_t currentBit = 0;

	for (int i = 0; i < 8; ++i)
	{
		digitalWrite(_scl_pin, HIGH);
		currentBit = digitalRead(_sda_pin);
		value |= (currentBit << 7-i);
		delayMicroseconds(1);
		digitalWrite(_scl_pin, LOW);
	}
	return value;
}
void _writeByte(uint8_t value)
{
	pinMode(_sda_pin, OUTPUT);
	shiftOut(_sda_pin, _scl_pin, MSBFIRST, value);
}

uint8_t _readRegister(uint8_t reg)
{
	uint8_t	readValue=0;

	_sendStart(DS1307_ADDR_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_sendStop();
	_sendStart(DS1307_ADDR_R);
	_waitForAck();
	readValue = _readByte();
	_sendNack();
	_sendStop();
	return readValue;
}

void _writeRegister(uint8_t reg, uint8_t value)
{
	_sendStart(DS1307_ADDR_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_writeByte(value);
	_waitForAck();
	_sendStop();
}

void _writeRegisterT(uint8_t reg, uint8_t value)
{
	_sendStart(TSL2561_ADDR_LOW_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_writeByte(value);
	_waitForAck();
	_sendStop();
}

uint8_t _readRegisterT(uint8_t reg)
{
	uint8_t	readValue=0;

	_sendStart(TSL2561_ADDR_LOW_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_sendStop();
	_sendStart(TSL2561_ADDR_LOW_R);
	_waitForAck();
	readValue = _readByte();
	_sendNack();
	_sendStop();
	return readValue;
}

/*
uint16_t _readRegister16(uint8_t reg)// not tested
{
	uint16_t	readValue=0;

	_sendStart(TSL2561_ADDR_LOW_W);
	_waitForAck();
	_writeByte(reg);
	_waitForAck();
	_sendStop();
	_sendStart(TSL2561_ADDR_LOW_R);
	_waitForAck();
        _writeByte(2);
	_waitForAck();
	readValue = _readByte();
	_sendNack();
        readValue <<= 8;
	readValue |= _readByte();
	_sendNack();
	_sendStop();
	return readValue;
}*/

/*
void _burstRead()
{
	_sendStart(DS1307_ADDR_W);
	_waitForAck();
	_writeByte(0);
	_waitForAck();
	_sendStop();
	_sendStart(DS1307_ADDR_R);
	_waitForAck();

	for (int i=0; i<8; i++)
	{
		_burstArray[i] = _readByte();
		if (i<7)
			_sendAck();
		else
			_sendNack();
	}
	_sendStop();
}*/

void poke2(uint8_t addr, uint8_t value)
{
//	if ((addr >=0) && (addr<=55+8))
//	{
//		addr += 8;
		_sendStart(DS1307_ADDR_W);
		_waitForAck();
		_writeByte(addr);
		_waitForAck();
		_writeByte(value);
		_waitForAck();
		_sendStop();
//	}
}

//CLKrtc IOrtc
//#define CLKrtc 5
//#define IOrtc 4

void ShiftOutM(byte val,byte clkmask,byte datamask)
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

//PORTC&=~(1<<sda);//clear data bit
PORTC&=~(datamask);//clear data bit
//__asm__ __volatile__ ( "out 0x08,r1\n\t"        :::);
//__asm__ __volatile__ ( "cbi 0x08,2\n\t"        :::);

if (val&(1<<(0)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk

PORTC&=~(clkmask);//clk low
PORTC&=~(datamask);//clear data bit
//__asm__ __volatile__ ( "clr r24\n\t""out 0x08,r24\n\t"        :::);

//PORTC=~((1<<IOrtc)|(1<<CLKrtc));//clk low +  clear data bit
if (val&(1<<(1)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk

PORTC&=~(clkmask);//clk low
PORTC&=~(1<<datamask);//clear data bit
//PORTC=~((1<<IOrtc)|(1<<CLKrtc));//clk low +  clear data bit
if (val&(1<<(2)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk

PORTC&=~(clkmask);//clk low
PORTC&=~(datamask);//clear data bit
//PORTC=~((1<<IOrtc)|(1<<CLKrtc));//clk low +  clear data bit

if (val&(1<<(3)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk

PORTC&=~(clkmask);//clk low
PORTC&=~(datamask);//clear data bit
if (val&(1<<(4)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk

PORTC&=~(clkmask);//clk low
PORTC&=~(datamask);//clear data bit
if (val&(1<<(5)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk

PORTC&=~(clkmask);//clk low
PORTC&=~(datamask);//clear data bit
if (val&(1<<(6)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk

PORTC&=~(clkmask);//clk low
PORTC&=~(datamask);//clear data bit
if (val&(1<<(7)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk

//PORTC&=~(1<<CLKrtc);//clk low (needed)
//"=r" (result): "I" (val)

}

void	I2C_START(byte addr,byte clkpin,byte datapin)
{
        Pin2Output(DDRC,datapin);//	pinMode(_sda_pin, OUTPUT);
        Pin2HIGH(PORTC,datapin);//	digitalWrite(_sda_pin, HIGH);
// get port from pin//portOutputRegister(digitalPinToPort(cs);???
	Pin2HIGH(PORTC,clkpin);//digitalWrite(_scl_pin, HIGH);
        Pin2LOW(PORTC,datapin);//	digitalWrite(_sda_pin, LOW);
        Pin2LOW(PORTC,clkpin);//	digitalWrite(_scl_pin, LOW);
//	shiftOut(_sda_pin, _scl_pin, MSBFIRST, addr);
	ShiftOutM(addr,(1<<clkpin),(1<<datapin));
}

void I2C_STOP(byte clkpin,byte datapin)
{
        Pin2Output(DDRC,datapin);//	pinMode(_sda_pin, OUTPUT);
        Pin2LOW(PORTC,datapin);//	digitalWrite(_sda_pin, LOW);
	Pin2HIGH(PORTC,clkpin);//digitalWrite(_scl_pin, HIGH);
        Pin2HIGH(PORTC,datapin);//	digitalWrite(_sda_pin, HIGH);
        Pin2Input(DDRC,datapin);//	pinMode(_sda_pin, INPUT);
}

/* too fast???
void I2C_SendNack(byte clkpin,byte datapin)
{
        Pin2Output(DDRC,datapin);//	pinMode(_sda_pin, OUTPUT);
        Pin2LOW(PORTC,clkpin);//		digitalWrite(_scl_pin, LOW);
        Pin2HIGH(PORTC,datapin);//	digitalWrite(_sda_pin, HIGH);
	Pin2HIGH(PORTC,clkpin);//digitalWrite(_scl_pin, HIGH);
        Pin2LOW(PORTC,clkpin);//		digitalWrite(_scl_pin, LOW);
        Pin2Input(DDRC,datapin);//	pinMode(_sda_pin, INPUT);
}

void I2C_SendAck(byte clkpin,byte datapin)
{
        Pin2Output(DDRC,datapin);//	pinMode(_sda_pin, OUTPUT);
        Pin2LOW(PORTC,clkpin);//		digitalWrite(_scl_pin, LOW);
        Pin2LOW(PORTC,datapin);//	digitalWrite(_sda_pin, LOW);
	Pin2HIGH(PORTC,clkpin);//digitalWrite(_scl_pin, HIGH);
        Pin2LOW(PORTC,clkpin);//		digitalWrite(_scl_pin, LOW);
        Pin2Input(DDRC,datapin);//	pinMode(_sda_pin, INPUT);
}

void I2C_WaitForAck(byte clkpin,byte datapin)
{
      word maxcycles=1000;
      
        Pin2Input(DDRC,datapin);//	pinMode(_sda_pin, INPUT);
	Pin2HIGH(PORTC,clkpin);//digitalWrite(_scl_pin, HIGH);
	while (PORTC&(1<<clkpin)==0) {if(maxcycles--==0){break;}}
        Pin2LOW(PORTC,clkpin);//		digitalWrite(_scl_pin, LOW);
}*/

void Save_I2C(byte addr,byte reg,byte val)
{
  		_sendStart(addr);
//  I2C_START(addr,5,4);
		_waitForAck();
//I2C_WaitForAck(5,4);
		_writeByte(reg);
//I2C_WaitForAck(5,4);
		_waitForAck();
		_writeByte(val);
//I2C_WaitForAck(5,4);
		_waitForAck();
//  I2C_STOP(5,4);
		_sendStop();
}

byte Read_I2C(byte addr,byte reg)
{
  byte val;
//  I2C_START(addr,5,4);//
	_sendStart(addr);
//I2C_WaitForAck(5,4);
	_waitForAck();
	_writeByte(reg);
//I2C_WaitForAck(5,4);
	_waitForAck();
  //I2C_STOP(5,4);//	_sendStop();//
  	_sendStop();
//  I2C_START(addr|1,5,4);//	
_sendStart(addr|1);
//I2C_WaitForAck(5,4);
	_waitForAck();
	val = _readByte();
//I2C_SendNack(5,4);
	_sendNack();
//  I2C_STOP(5,4);
	_sendStop();
	return val;
}

uint8_t peek2(uint8_t addr)
{
//	if ((addr >=0) && (addr<=55+8))
//	{
		uint8_t readValue;

	//	addr += 8;
		_sendStart(DS1307_ADDR_W);
		_waitForAck();
		_writeByte(addr);
		_waitForAck();
		_sendStop();
		_sendStart(DS1307_ADDR_R);
		_waitForAck();
		readValue = _readByte();
		_sendNack();
		_sendStop();

		return readValue;
//	}
//	else
//		return 0;
}

