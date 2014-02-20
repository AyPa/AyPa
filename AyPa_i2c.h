#define TSL2561_ADDR_LOW_W 82
#define TSL2561_ADDR_LOW_R  83 //(0x29<<1)+1



#define TSL2561_ADDR_LOW 0x29


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

//#define _sda_pin A4
//#define _scl_pin A5



void	sendStart(byte addr,byte sda,byte scl)
{
	pinMode(sda, OUTPUT);
	digitalWrite(sda, HIGH);
	digitalWrite(scl, HIGH);
	digitalWrite(sda, LOW);
	digitalWrite(scl, LOW);
	shiftOut(sda, scl, MSBFIRST, addr);
}

// слишком быстро.... результат - ошибки
void	sendStart12(byte addr)
{
        Pin2Output(DDRC,1);//	pinMode(sda, OUTPUT);
        Pin2Output(DDRC,2);//	pinMode(scl, OUTPUT);
	Pin2HIGH(PORTC,1);// digitalWrite(sda, HIGH);
        Pin2HIGH(PORTC,2);//digitalWrite(scl, HIGH);
	Pin2LOW(PORTC,1); //	digitalWrite(sda, LOW);
	Pin2LOW(PORTC,2);//	digitalWrite(scl, LOW);
	shiftOut(A1,A2, MSBFIRST, addr);
}

void	sendStop(byte sda,byte scl)
{
	pinMode(sda, OUTPUT);
	digitalWrite(sda, LOW);
	digitalWrite(scl, HIGH);
	digitalWrite(sda, HIGH);
	pinMode(sda, INPUT);
}
void	sendStop12(void)
{
        Pin2Output(DDRC,1);//	pinMode(sda, OUTPUT);
        Pin2Output(DDRC,2);//	pinMode(scl, OUTPUT);

	Pin2LOW(PORTC,1); //	digitalWrite(sda, LOW);
        Pin2HIGH(PORTC,2);//digitalWrite(scl, HIGH);
	Pin2HIGH(PORTC,1);// digitalWrite(sda, HIGH);
      Pin2Input(DDRC,1);//	pinMode(sda, INPUT);
}

void	sendNack(byte sda,byte scl)
{
	pinMode(sda, OUTPUT);
	digitalWrite(scl, LOW);
	digitalWrite(sda, HIGH);
	digitalWrite(scl, HIGH);
	digitalWrite(scl, LOW);
	pinMode(sda, INPUT);
}

void	sendAck(byte sda,byte scl)
{
	pinMode(sda, OUTPUT);
	digitalWrite(scl, LOW);
	digitalWrite(sda, LOW);
	digitalWrite(scl, HIGH);
	digitalWrite(scl, LOW);
	pinMode(sda, INPUT);
}

void	waitForAck(byte sda,byte scl)
{
  word n=0;
  	pinMode(sda, INPUT);
	digitalWrite(scl, HIGH);
	while (sda==LOW) {n++;if(!n){break;}}
	digitalWrite(scl, LOW);
}

uint8_t readByte(byte sda,byte scl)
{
	pinMode(sda, INPUT);

	uint8_t value = 0;
	uint8_t currentBit = 0;

	for (int i = 0; i < 8; ++i)
	{
		digitalWrite(scl, HIGH);
		currentBit = digitalRead(sda);
		value |= (currentBit << 7-i);
		delayMicroseconds(1);
		digitalWrite(scl, LOW);
	}
	return value;
}
void writeByte(uint8_t value,byte sda,byte scl)
{
	pinMode(sda, OUTPUT);
	shiftOut(sda, scl, MSBFIRST, value);
}

uint8_t readRegister(uint8_t reg,byte sda,byte scl)
{
	uint8_t	readValue=0;

	sendStart(DS1307_ADDR_W,sda,scl);
	waitForAck(sda,scl);
	writeByte(reg,sda,scl);
	waitForAck(sda,scl);
	sendStop(sda,scl);
	sendStart(DS1307_ADDR_R,sda,scl);
	waitForAck(sda,scl);
	readValue = readByte(sda,scl);
	sendNack(sda,scl);
	sendStop(sda,scl);
	return readValue;
}

void writeRegister(uint8_t reg, uint8_t value,byte sda,byte scl)
{
	sendStart(DS1307_ADDR_W,sda,scl);
	waitForAck(sda,scl);
	writeByte(reg,sda,scl);
	waitForAck(sda,scl);
	writeByte(value,sda,scl);
	waitForAck(sda,scl);
	sendStop(sda,scl);
}

/*
void _writeRegisterT(uint8_t reg, uint8_t value,byte sda,byte scl)
{
	sendStart(TSL2561_ADDR_LOW_W,sda,scl);
	waitForAck(sda,scl);
	writeByte(reg,sda,scl);
	waitForAck(sda,scl);
	writeByte(value,sda,scl);
	waitForAck(sda,scl);
	sendStop(sda,scl);
}

uint8_t _readRegisterT(uint8_t reg,byte sda,byte scl)
{
	uint8_t	readValue=0;

	sendStart(TSL2561_ADDR_LOW_W,sda,scl);
	waitForAck(sda,scl);
	writeByte(reg,sda,scl);
	waitForAck(sda,scl);
	sendStop(sda,scl);
	sendStart(TSL2561_ADDR_LOW_R,sda,scl);
	waitForAck(sda,scl);
	readValue = readByte(sda,scl);
	sendNack(sda,scl);
	sendStop(sda,scl);
	return readValue;
}*/

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
		sendStart(DS1307_ADDR_W,A4,A5);
		waitForAck(A4,A5);
		writeByte(addr,A4,A5);
		waitForAck(A4,A5);
		writeByte(value,A4,A5);
		waitForAck(A4,A5);
		sendStop(A4,A5);
//	}
}

//CLKrtc IOrtc
//#define CLKrtc 5
//#define IOrtc 4
/*
void ShiftOutM(byte val,byte clkmask,byte datamask)
{
PORTC&=~(datamask);//clear data bit

if (val&(1<<(0)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk
PORTC&=~(clkmask);//clk low
PORTC&=~(datamask);//clear data bit
if (val&(1<<(1)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk
PORTC&=~(clkmask);//clk low
PORTC&=~(1<<datamask);//clear data bit
if (val&(1<<(2)))PORTC|=(datamask);// set it if needed
PORTC|=(clkmask);// tick clk
PORTC&=~(clkmask);//clk low
PORTC&=~(datamask);//clear data bit
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
//	shiftOut(datapin, clkpin, MSBFIRST, addr);
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
*/
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

void Save_I2C(byte addr,byte reg,byte val,byte dp,byte cp)
{
  		sendStart(addr,dp,cp);
//  		sendStart12(addr);
		waitForAck(dp,cp);
		writeByte(reg,dp,cp);
		waitForAck(dp,cp);
		writeByte(val,dp,cp);
		waitForAck(dp,cp);
		sendStop(dp,cp);
//		sendStop12();
}

byte Read_I2C(byte addr,byte reg,byte dp,byte cp)
{
  byte val;
	sendStart(addr,dp,cp);
//	sendStart12(addr);
	waitForAck(dp,cp);
	writeByte(reg,dp,cp);
	waitForAck(dp,cp);
  	sendStop(dp,cp);
//  	sendStop12();
        sendStart(addr|1,dp,cp);
//        sendStart12(addr|1);
	waitForAck(dp,cp);
	val = readByte(dp,cp);
	sendNack(dp,cp);
	sendStop(dp,cp);
  //	sendStop12();
	return val;
}

uint8_t peek2(uint8_t addr)
{
//	if ((addr >=0) && (addr<=55+8))
//	{
		uint8_t readValue;

	//	addr += 8;
	        sendStart(DS1307_ADDR_W,A4,A5);
		waitForAck(A4,A5);
		writeByte(addr,A4,A5);
		waitForAck(A4,A5);
		sendStop(A4,A5);
		sendStart(DS1307_ADDR_R,A4,A5);
		waitForAck(A4,A5);
		readValue = readByte(A4,A5);
		sendNack(A4,A5);
		sendStop(A4,A5);

		return readValue;
//	}
//	else
//		return 0;
}
/*
//TSL2561_START(TSL2561_INTEGRATIONTIME_13MS |TSL2561_GAIN_16X);//delay(14);//res=TSL2561_STOP();
void  TSL2561_START(byte val)
{
  Save_I2C(TSL2561_ADDR_LOW_W,(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL),TSL2561_CONTROL_POWERON,A4,A5); // enable
  Save_I2C(TSL2561_ADDR_LOW_W,(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING),val,A4,A5);   // set gain and integration time
}

long  TSL2561_STOP(void)
{
    long res;
    res=Read_I2C(TSL2561_ADDR_LOW_W,(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN1_HIGH),A4,A5);
    res <<= 8;
    res|=Read_I2C(TSL2561_ADDR_LOW_W,(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN1_LOW),A4,A5);
    res <<= 8;
    res|=Read_I2C(TSL2561_ADDR_LOW_W,(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN0_HIGH),A4,A5);
    res <<= 8;
    res|=Read_I2C(TSL2561_ADDR_LOW_W,(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN0_LOW),A4,A5);


//res=Read_I2C(TSL2561_ADDR_LOW_W,TSL2561_REGISTER_ID ,A1,A2);// 09
//res=Read_I2C(TSL2561_ADDR_LOW_W,TSL2561_COMMAND_BIT | TSL2561_REGISTER_ID ,A1,A2); // 0x11

//    if (Read_I2C(TSL2561_ADDR_LOW_W,TSL2561_REGISTER_ID ,A1,A2)!=0x0A){ res=0; }  // дополнительная проверка
    Save_I2C(TSL2561_ADDR_LOW_W,(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL),TSL2561_CONTROL_POWEROFF,A4,A5); // disable
    return res;
}*/

#define TWI_SDA_PIN 1
#define TWI_SCL_PIN 2
//#define EEPROM_ADDR 0x50
#define I2C_DELAY_USEC 8
#define I2C_READ 1
#define I2C_WRITE 0
 
// Initialize SCL/SDA pins and set the bus high
void SoftI2cMasterInit(void) {
  DDRC |= (1<<TWI_SDA_PIN);
  PORTC |= (1<<TWI_SDA_PIN);
  DDRC |= (1<<TWI_SCL_PIN);
  PORTC |= (1<<TWI_SCL_PIN);
}
 
// De-initialize SCL/SDA pins and set the bus low
void SoftI2cMasterDeInit(void) {
  PORTC &= ~(1<<TWI_SDA_PIN);
  DDRC &= ~(1<<TWI_SDA_PIN);
  PORTC &= ~(1<<TWI_SCL_PIN);
  DDRC &= ~(1<<TWI_SCL_PIN);
}
 
// Read a byte from I2C and send Ack if more reads follow else Nak to terminate read
uint8_t SoftI2cMasterRead(uint8_t last) {
  uint8_t b = 0;
  // Make sure pull-up enabled
//  PORTC |= (1<<TWI_SDA_PIN);
//  DDRC &= ~(1<<TWI_SDA_PIN);
  // Read byte
  for (uint8_t i = 0; i < 8; i++) {
    // Don't change this loop unless you verify the change with a scope
    b <<= 1;
    delayMicroseconds(I2C_DELAY_USEC);
    PORTC |= (1<<TWI_SCL_PIN);
    if (bit_is_set(PINC, TWI_SDA_PIN)) b |= 1;
    PORTC &= ~(1<<TWI_SCL_PIN);
  }
  // Send Ack or Nak
  DDRC |= (1<<TWI_SDA_PIN);
  if (last) {
    PORTC |= (1<<TWI_SDA_PIN);
  }
  else {
    PORTC &= ~(1<<TWI_SDA_PIN);
  }
  PORTC |= (1<<TWI_SCL_PIN);
  delayMicroseconds(I2C_DELAY_USEC);
  PORTC &= ~(1<<TWI_SCL_PIN);
  PORTC &= ~(1<<TWI_SDA_PIN);
  return b;
}
 
// Write a byte to I2C
bool SoftI2cMasterWrite(uint8_t data) {
  // Write byte
  for (uint8_t m = 0x80; m != 0; m >>= 1) {
    // Don't change this loop unless you verify the change with a scope
    if (m & data) {
      PORTC |= (1<<TWI_SDA_PIN);
    }
    else {
      PORTC &= ~(1<<TWI_SDA_PIN);
    }
    PORTC |= (1<<TWI_SCL_PIN);
    delayMicroseconds(I2C_DELAY_USEC);
    PORTC &= ~(1<<TWI_SCL_PIN);
  }
  // get Ack or Nak
//  DDRC &= ~(1<<TWI_SDA_PIN);
  // Enable pullup
//  PORTC |= (1<<TWI_SDA_PIN);
//  PORTC |= (1<<TWI_SCL_PIN);
  uint8_t rtn = bit_is_set(PINC, TWI_SDA_PIN);
  PORTC &= ~(1<<TWI_SCL_PIN);
  DDRC |= (1<<TWI_SDA_PIN);
  PORTC &= ~(1<<TWI_SDA_PIN);
  return rtn == 0;
}
 
// Issue a start condition
bool SoftI2cMasterStart(uint8_t addressRW) {
  PORTC &= ~(1<<TWI_SDA_PIN);
  delayMicroseconds(I2C_DELAY_USEC);
  PORTC &= ~(1<<TWI_SCL_PIN);
  return SoftI2cMasterWrite(addressRW);
}
 
// Issue a restart condition
bool SoftI2cMasterRestart(uint8_t addressRW) {
  PORTC |= (1<<TWI_SDA_PIN);
  PORTC |= (1<<TWI_SCL_PIN);
  delayMicroseconds(I2C_DELAY_USEC);
  return SoftI2cMasterStart(addressRW);
}
 
// Issue a stop condition
void SoftI2cMasterStop(void) {
  PORTC &= ~(1<<TWI_SDA_PIN);
  delayMicroseconds(I2C_DELAY_USEC);
  PORTC |= (1<<TWI_SCL_PIN);
  delayMicroseconds(I2C_DELAY_USEC);
  PORTC |= (1<<TWI_SDA_PIN);
  delayMicroseconds(I2C_DELAY_USEC);
}
 
// Read 1 byte from the device and return it
uint8_t soft_i2c_read_byte(uint8_t deviceAddr, uint16_t readAddress) {
  uint8_t byteRead = 0;
 
  // Issue a start condition, send device address and write direction bit
  if (!SoftI2cMasterStart((deviceAddr<<1) | I2C_WRITE)) return false;
 
  // Send the address to read, 8 bit or 16 bit
  if (readAddress > 255) {
    if (!SoftI2cMasterWrite((readAddress >> 8))) return false; // MSB
    if (!SoftI2cMasterWrite((readAddress & 0xFF))) return false; // LSB
  }
  else {
    if (!SoftI2cMasterWrite(readAddress)) return false; // 8 bit
  }
 
  // Issue a repeated start condition, send device address and read direction bit
  if (!SoftI2cMasterRestart((deviceAddr<<1) | I2C_READ)) return false;
 
  // Read the byte
  byteRead = SoftI2cMasterRead(1);
 
  // Issue a stop condition
  SoftI2cMasterStop();
 
  return byteRead;
}
 
// Write 1 byte to the device
bool soft_i2c_write_byte(uint8_t deviceAddr, uint16_t writeAddress, byte writeByte) {
 
  // Issue a start condition, send device address and write direction bit
  if (!SoftI2cMasterStart((deviceAddr<<1) | I2C_WRITE)) return false;
 
  // Send the address to write, 8 bit or 16 bit
  if ( writeAddress > 255) {
    if (!SoftI2cMasterWrite((writeAddress >> 8))) return false; // MSB
    if (!SoftI2cMasterWrite((writeAddress & 0xFF))) return false; // LSB
  }
  else {
    if (!SoftI2cMasterWrite(writeAddress)) return false; // 8 bit
  }
 
  // Write the byte
  if (!SoftI2cMasterWrite(writeByte)) return false;
 
  // Issue a stop condition
  SoftI2cMasterStop();
 
  // Delay 10ms for the write to complete, depends on the EEPROM you use
//  _delay_ms(10);
 
  return true;
}

