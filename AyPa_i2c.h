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




//============================= monumental lib

/* Arduino SoftI2C library. 
 *
 * This is a very fast and very light-weight software I2C-master library 
 * written in assembler. It is based on Peter Fleury's I2C software
 * library: http://homepage.hispeed.ch/peterfleury/avr-software.html
 *
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino I2cMaster Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/* In order to use the library, you need to define SDA_PIN, SCL_PIN,
 * SDA_PORT and SCL_PORT before including this file.  Have a look at
 * http://www.arduino.cc/en/Reference/PortManipulation for finding out
 * which values to use. For example, if you use digital pin 3 for
 * SDA and digital pin 13 for SCL you have to use the following
 * definitions: 
 * #define SDA_PIN 3 
 * #define SDA_PORT PORTB 
 * #define SCL_PIN 5
 * #define SCL_PORT PORTB
 *
 * You can also define the following constants (see also below):
 * - I2C_CPUFREQ, when changing CPU clock frequency dynamically
 * - I2C_FASTMODE = 1 meaning that the I2C bus allows speeds up to 400 kHz
 * - I2C_SLOWMODE = 1 meaning that the I2C bus will allow only up to 25 kHz 
 * - I2C_NOINTERRUPT = 1 in order to prohibit interrupts while 
 *   communicating (see below). This can be useful if you use the library 
 *   for communicationg with SMbus devices, which have timeouts.
 *   Note, however, that interrupts are disabledfrom issuing a start condition
 *   until issuing a stop condition. So use this option with care!
 * - I2C_TIMEOUT = 0..10000 mssec in order to return from the I2C functions
 *   in case of a I2C bus lockup (i.e., SCL constantly low). 0 means no timeout
 */

/* Changelog:
 * Version 1.1: 
 * - removed I2C_CLOCK_STRETCHING
 * - added I2C_TIMEOUT time in msec (0..10000) until timeout or 0 if no timeout
 * - changed i2c_init to return true iff both SDA and SCL are high
 * - changed interrupt disabling so that the previous IRQ state is retored
 * Version 1.0: basic functionality
 */
#include <avr/io.h>
#include <Arduino.h>

#ifndef _SOFTI2C_H
#define _SOFTI2C_H   1

// Init function. Needs to be called once in the beginning.
// Returns false if SDA or SCL are low, which probably means 
// a I2C bus lockup or that the lines are not pulled up.
boolean __attribute__ ((noinline)) i2c_init(void);

// Start transfer function: <addr> is the 8-bit I2C address (including the R/W
// bit). 
// Return: true if the slave replies with an "acknowledge", false otherwise
bool __attribute__ ((noinline)) i2c_start(uint8_t addr); 

// Similar to start function, but wait for an ACK! Be careful, this can 
// result in an infinite loop!
void  __attribute__ ((noinline)) i2c_start_wait(uint8_t addr);

// Repeated start function: After having claimed the bus with a start condition,
// you can address another or the same chip again without an intervening 
// stop condition.
// Return: true if the slave replies with an "acknowledge", false otherwise
bool __attribute__ ((noinline)) i2c_rep_start(uint8_t addr);

// Issue a stop condition, freeing the bus.
void __attribute__ ((noinline)) i2c_stop(void) asm("ass_i2c_stop");

// Write one byte to the slave chip that had been addressed
// by the previous start call. <value> is the byte to be sent.
// Return: true if the slave replies with an "acknowledge", false otherwise
bool __attribute__ ((noinline)) i2c_write(uint8_t value) asm("ass_i2c_write");

// Read one byte. If <last> is true, we send a NAK after having received 
// the byte in order to terminate the read sequence. 
uint8_t __attribute__ ((noinline)) i2c_read(bool last);

// You can set I2C_CPUFREQ independently of F_CPU if you 
// change the CPU frequency on the fly. If do not define it,
// it will use the value of F_CPU
#ifndef I2C_CPUFREQ
#define I2C_CPUFREQ F_CPU
#endif

// If I2C_FASTMODE is set to 1, then the highest possible frequency below 400kHz
// is selected. Be aware that not all slave chips may be able to deal with that!
#ifndef I2C_FASTMODE
#define I2C_FASTMODE 0
#endif

// If I2C_FASTMODE is not defined or defined to be 0, then you can set
// I2C_SLOWMODE to 1. In this case, the I2C frequency will not be higher 
// than 25KHz. This could be useful for problematic buses.
#ifndef I2C_SLOWMODE
#define I2C_SLOWMODE 0
#endif

// if I2C_NOINTERRUPT is 1, then the I2C routines are not interruptable.
// This is most probably only necessary if you are using a 1MHz system clock,
// you are communicating with a SMBus device, and you want to avoid timeouts.
// Be aware that the interrupt bit is enabled after each call. So the
// I2C functions should not be called in interrupt routines or critical regions.
#ifndef I2C_NOINTERRUPT
#define I2C_NOINTERRUPT 0
#endif

// I2C_TIMEOUT can be set to a value between 1 and 10000.
// If it is defined and nonzero, it leads to a timeout if the
// SCL is low longer than I2C_TIMEOUT milliseconds, i.e., max timeout is 10 sec
#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT 0
#else 
#if I2C_TIMEOUT > 10000
#error I2C_TIMEOUT is too large
#endif
#endif

#define I2C_TIMEOUT_DELAY_LOOPS (I2C_CPUFREQ/1000UL)*I2C_TIMEOUT/4000UL
#if I2C_TIMEOUT_DELAY_LOOPS < 1
#define I2C_MAX_STRETCH 1
#else
#if I2C_TIMEOUT_DELAY_LOOPS > 60000UL
#define I2C_MAX_STRETCH 60000UL
#else
#define I2C_MAX_STRETCH I2C_TIMEOUT_DELAY_LOOPS
#endif
#endif

#if I2C_FASTMODE
#define I2C_DELAY_COUNTER (((I2C_CPUFREQ/400000L)/2-19)/3)
#else
#if I2C_SLOWMODE
#define I2C_DELAY_COUNTER (((I2C_CPUFREQ/25000L)/2-19)/3)
#else
#define I2C_DELAY_COUNTER (((I2C_CPUFREQ/100000L)/2-19)/3)
#endif
#endif

// Table of I2C bus speed in kbit/sec:
// CPU clock:           1MHz   2MHz    4MHz   8MHz   16MHz   20MHz
// Fast I2C mode          40     80     150    300     400     400
// Standard I2C mode      40     80     100    100     100     100
// Slow I2C mode          25     25      25     25      25      25     

// constants for reading & writing
#define I2C_READ    1
#define I2C_WRITE   0

// map the IO register back into the IO address space
#define SDA_DDR       	(_SFR_IO_ADDR(SDA_PORT) - 1)
#define SCL_DDR       	(_SFR_IO_ADDR(SCL_PORT) - 1)
#define SDA_OUT       	_SFR_IO_ADDR(SDA_PORT)
#define SCL_OUT       	_SFR_IO_ADDR(SCL_PORT)
#define SDA_IN		(_SFR_IO_ADDR(SDA_PORT) - 2)
#define SCL_IN		(_SFR_IO_ADDR(SCL_PORT) - 2)

#ifndef __tmp_reg__
#define __tmp_reg__ 0
#endif

 
// Internal delay functions.
void __attribute__ ((noinline)) i2c_delay_half(void) asm("ass_i2c_delay_half");
void __attribute__ ((noinline)) i2c_wait_scl_high(void) asm("ass_i2c_wait_scl_high");

void  i2c_delay_half(void)
{ // function call 3 cycles => 3C
#if I2C_DELAY_COUNTER < 1
  __asm__ __volatile__ (" ret");
  // 7 cycles for call and return
#else
  __asm__ __volatile__ 
    (
     " ldi      r25, %[DELAY]           ;load delay constant   ;; 4C \n\t"
     "_Lidelay: \n\t"
     " dec r25                          ;decrement counter     ;; 4C+xC \n\t"
     " brne _Lidelay                                           ;;5C+(x-1)2C+xC\n\t"
     " ret                                                     ;; 9C+(x-1)2C+xC = 7C+xC" 
     : : [DELAY] "M" I2C_DELAY_COUNTER : "r25");
  // 7 cycles + 3 times x cycles
#endif
}

void i2c_wait_scl_high(void)
{
#if I2C_TIMEOUT <= 0
  __asm__ __volatile__ 
    ("_Li2c_wait_stretch: \n\t"
     " sbis	%[SCLIN],%[SCLPIN]	;wait for SCL high \n\t" 
     " rjmp	_Li2c_wait_stretch \n\t"
     " cln                              ;signal: no timeout \n\t"
     " ret "
     : : [SCLIN] "I" (SCL_IN), [SCLPIN] "I" (SCL_PIN));
#else
  __asm__ __volatile__ 
    ( " ldi     r27, %[HISTRETCH]       ;load delay counter \n\t"
      " ldi     r26, %[LOSTRETCH] \n\t"
      "_Lwait_stretch: \n\t"
      " clr     __tmp_reg__             ;do next loop 255 times \n\t"
      "_Lwait_stretch_inner_loop: \n\t"
      " rcall   _Lcheck_scl_level       ;call check function   ;; 12C \n\t"
      " brpl    _Lstretch_done          ;done if N=0           ;; +1 = 13C\n\t"
      " dec     __tmp_reg__             ;dec inner loop counter;; +1 = 14C\n\t"
      " brne    _Lwait_stretch_inner_loop                      ;; +2 = 16C\n\t"
      " sbiw    r26,1                   ;dec outer loop counter \n\t"
      " brne    _Lwait_stretch          ;continue with outer loop \n\t"
      " sen                             ;timeout -> set N-bit=1 \n\t"
      " rjmp _Lwait_return              ;and return with N=1\n\t"
      "_Lstretch_done:                  ;SCL=1 sensed \n\t"            
      " cln                             ;OK -> clear N-bit \n\t"
      " rjmp _Lwait_return              ; and return with N=0 \n\t"

      "_Lcheck_scl_level:                                      ;; call = 3C\n\t"
      " cln                                                    ;; +1C = 4C \n\t"
      " sbic	%[SCLIN],%[SCLPIN]      ;skip if SCL still low ;; +2C = 6C \n\t"
      " rjmp    _Lscl_high                                     ;; +0C = 6C \n\t"
      " sen                                                    ;; +1 = 7C\n\t "
      "_Lscl_high: "
      " nop                                                    ;; +1C = 8C \n\t"
      " ret                             ;return N-Bit=1 if low ;; +4 = 12C\n\t"

      "_Lwait_return:"
      : : [SCLIN] "I" (SCL_IN), [SCLPIN] "I" (SCL_PIN), 
	[HISTRETCH] "M" (I2C_MAX_STRETCH>>8), 
	[LOSTRETCH] "M" (I2C_MAX_STRETCH&0xFF)
      : "r26", "r27");
#endif
}


boolean i2c_init(void)
{
  __asm__ __volatile__ 
    (" cbi      %[SDADDR],%[SDAPIN]     ;release SDA \n\t" 
     " cbi      %[SCLDDR],%[SCLPIN]     ;release SCL \n\t" 
     " cbi      %[SDAOUT],%[SDAPIN]     ;clear SDA output value \n\t" 
     " cbi      %[SCLOUT],%[SCLPIN]     ;clear SCL output value \n\t" 
     " clr      r24                     ;set return value to false \n\t"
     " clr      r25                     ;set return value to false \n\t"
     " sbis     %[SDAIN],%[SDAPIN]      ;check for SDA high\n\t"
     " ret                              ;if low return with false \n\t"  
     " sbis     %[SCLIN],%[SCLPIN]      ;check for SCL high \n\t"
     " ret                              ;if low return with false \n\t" 
     " ldi      r24,1                   ;set return value to true \n\t"
     " ret "
     : :
       [SCLDDR] "I"  (SCL_DDR), [SCLPIN] "I" (SCL_PIN), 
       [SCLIN] "I" (SCL_IN), [SCLOUT] "I" (SCL_OUT),
       [SDADDR] "I"  (SDA_DDR), [SDAPIN] "I" (SDA_PIN), 
       [SDAIN] "I" (SDA_IN), [SDAOUT] "I" (SDA_OUT)); 
  return true;
}

bool  i2c_start(uint8_t addr)
{
  __asm__ __volatile__ 
    (
#if I2C_NOINTERRUPT
     " cli                              ;clear IRQ bit \n\t"
#endif
     " sbis     %[SCLIN],%[SCLPIN]      ;check for clock stretching slave\n\t"
     " rcall    ass_i2c_wait_scl_high   ;wait until SCL=H\n\t" 
     " sbi      %[SDADDR],%[SDAPIN]     ;force SDA low  \n\t" 
     " rcall    ass_i2c_delay_half      ;wait T/2 \n\t"
     " rcall    ass_i2c_write           ;now write address \n\t"
     " ret"
     : : [SDADDR] "I"  (SDA_DDR), [SDAPIN] "I" (SDA_PIN),
       [SCLIN] "I" (SCL_IN),[SCLPIN] "I" (SCL_PIN)); 
  return true; // we never return here!
}

bool  i2c_rep_start(uint8_t addr)
{
  __asm__ __volatile__ 

    (
#if I2C_NOINTERRUPT
     " cli \n\t"
#endif
     " sbi	%[SCLDDR],%[SCLPIN]	;force SCL low \n\t" 
     " rcall 	ass_i2c_delay_half	;delay  T/2 \n\t" 
     " cbi	%[SDADDR],%[SDAPIN]	;release SDA \n\t" 
     " rcall	ass_i2c_delay_half	;delay T/2 \n\t" 
     " cbi	%[SCLDDR],%[SCLPIN]	;release SCL \n\t" 
     " rcall 	ass_i2c_delay_half	;delay  T/2 \n\t" 
     " sbis     %[SCLIN],%[SCLPIN]      ;check for clock stretching slave\n\t"
     " rcall    ass_i2c_wait_scl_high   ;wait until SCL=H\n\t" 
     " sbi 	%[SDADDR],%[SDAPIN]	;force SDA low \n\t" 
     " rcall 	ass_i2c_delay_half	;delay	T/2 \n\t" 
     " rcall    ass_i2c_write       \n\t"
     " ret"
     : : [SCLDDR] "I"  (SCL_DDR), [SCLPIN] "I" (SCL_PIN),[SCLIN] "I" (SCL_IN),
         [SDADDR] "I"  (SDA_DDR), [SDAPIN] "I" (SDA_PIN)); 
  return true; // just to fool the compiler
}

void  i2c_start_wait(uint8_t addr)
{
 __asm__ __volatile__ 
   (
    " push	r24                     ;save original parameter \n\t"
    "_Li2c_start_wait1: \n\t"
    " pop       r24                     ;restore original parameter\n\t"
    " push      r24                     ;and save again \n\t"
#if I2C_NOINTERRUPT
    " cli                               ;disable interrupts \n\t"
#endif
    " sbis     %[SCLIN],%[SCLPIN]      ;check for clock stretching slave\n\t"
    " rcall    ass_i2c_wait_scl_high   ;wait until SCL=H\n\t" 
    " sbi 	%[SDADDR],%[SDAPIN]	;force SDA low \n\t" 
    " rcall 	ass_i2c_delay_half	;delay T/2 \n\t" 
    " rcall 	ass_i2c_write	        ;write address \n\t" 
    " tst	r24		        ;if device not busy -> done \n\t" 
    " brne	_Li2c_start_wait_done \n\t" 
    " rcall	ass_i2c_stop	        ;terminate write & enable IRQ \n\t" 
    " rjmp	_Li2c_start_wait1	;device busy, poll ack again \n\t" 
    "_Li2c_start_wait_done: \n\t"
    " pop       __tmp_reg__             ;pop off orig argument \n\t"
    " ret "
    : : [SDADDR] "I"  (SDA_DDR), [SDAPIN] "I" (SDA_PIN),
      [SCLIN] "I" (SCL_IN),[SCLPIN] "I" (SCL_PIN)); 
}

void  i2c_stop(void)
{
  __asm__ __volatile__ 
    (
     " sbi      %[SCLDDR],%[SCLPIN]     ;force SCL low \n\t" 
     " sbi      %[SDADDR],%[SDAPIN]     ;force SDA low \n\t" 
     " rcall    ass_i2c_delay_half      ;T/2 delay \n\t"
     " cbi      %[SCLDDR],%[SCLPIN]     ;release SCL \n\t" 
     " rcall    ass_i2c_delay_half      ;T/2 delay \n\t"
     " sbis     %[SCLIN],%[SCLPIN]      ;check for clock stretching slave\n\t"
     " rcall    ass_i2c_wait_scl_high   ;wait until SCL=H\n\t" 
     " cbi      %[SDADDR],%[SDAPIN]     ;release SDA \n\t" 
     " rcall    ass_i2c_delay_half \n\t"
#if I2C_NOINTERRUPT
     " sei                              ;enable interrupts again!\n\t"
#endif
     : : [SCLDDR] "I"  (SCL_DDR), [SCLPIN] "I" (SCL_PIN), [SCLIN] "I" (SCL_IN),
         [SDADDR] "I"  (SDA_DDR), [SDAPIN] "I" (SDA_PIN)); 
}

bool i2c_write(uint8_t value)
{
  __asm__ __volatile__ 
    (
     " sec                              ;set carry flag \n\t"
     " rol      r24                     ;shift in carry and shift out MSB \n\t"
     " rjmp _Li2c_write_first \n\t"
     "_Li2c_write_bit:\n\t"
     " lsl      r24                     ;left shift into carry ;; 1C\n\t"
     "_Li2c_write_first:\n\t"
     " breq     _Li2c_get_ack           ;jump if TXreg is empty;; +1 = 2C \n\t"
     " sbi      %[SCLDDR],%[SCLPIN]     ;force SCL low         ;; +2 = 4C \n\t"
     " nop \n\t"
     " nop \n\t"
     " nop \n\t"
     " brcc     _Li2c_write_low                                ;;+1/+2=5/6C\n\t"
     " nop                                                     ;; +1 = 7C \n\t"
     " cbi %[SDADDR],%[SDAPIN]	        ;release SDA           ;; +2 = 9C \n\t"
     " rjmp      _Li2c_write_high                              ;; +2 = 11C \n\t"
     "_Li2c_write_low: \n\t"
     " sbi	%[SDADDR],%[SDAPIN]	;force SDA low         ;; +2 = 9C \n\t"
     " rjmp	_Li2c_write_high                               ;;+2 = 11C \n\t"
     "_Li2c_write_high: \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall 	ass_i2c_delay_half	;delay T/2             ;;+X = 11C+X\n\t"
#endif
     " cbi	%[SCLDDR],%[SCLPIN]	;release SCL           ;;+2 = 13C+X\n\t"
     " cln                              ;clear N-bit           ;;+1 = 14C+X\n\t"
     " nop \n\t"
     " nop \n\t"
     " nop \n\t"
     " sbis	%[SCLIN],%[SCLPIN]	;check for SCL high    ;;+2 = 16C+X\n\t"
     " rcall    ass_i2c_wait_scl_high \n\t"
     " brpl     _Ldelay_scl_high                              ;;+2 = 18C+X\n\t"
     "_Li2c_write_return_false: \n\t"
     " clr      r24                     ; return false because of timeout \n\t"
     " rjmp     _Li2c_write_return \n\t"
     "_Ldelay_scl_high: \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half	;delay T/2             ;;+X= 18C+2X\n\t"
#endif
     " rjmp	_Li2c_write_bit \n\t"
     "              ;; +2 = 20C +2X for one bit-loop \n\t"
     "_Li2c_get_ack: \n\t"
     " sbi	%[SCLDDR],%[SCLPIN]	;force SCL low ;; +2 = 5C \n\t"
     " nop \n\t"
     " nop \n\t"
     " cbi	%[SDADDR],%[SDAPIN]	;release SDA ;;+2 = 7C \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half	;delay T/2 ;; +X = 7C+X \n\t"
#endif
     " clr	r25                                            ;; 17C+2X \n\t"
     " clr	r24		        ;return 0              ;; 14C + X \n\t"
     " cbi	%[SCLDDR],%[SCLPIN]	;release SCL ;; +2 = 9C+X\n\t"
     "_Li2c_ack_wait: \n\t"
     " cln                              ; clear N-bit          ;; 10C + X\n\t" 
     " nop \n\t"
     " sbis	%[SCLIN],%[SCLPIN]	;wait SCL high         ;; 12C + X \n\t"
     " rcall    ass_i2c_wait_scl_high \n\t"
     " brmi     _Li2c_write_return_false                       ;; 13C + X \n\t "
     " sbis	%[SDAIN],%[SDAPIN]      ;if SDA hi -> return 0 ;; 15C + X \n\t"
     " ldi	r24,1                   ;return true           ;; 16C + X \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half	;delay T/2             ;; 16C + 2X \n\t"
#endif
     "_Li2c_write_return: \n\t"
     " nop \n\t "
     " nop \n\t "
     " sbi	%[SCLDDR],%[SCLPIN]	;force SCL low so SCL=H is short\n\t"
     " ret \n\t"
     "              ;; + 4 = 17C + 2X for acknowldge bit"
     ::
      [SCLDDR] "I"  (SCL_DDR), [SCLPIN] "I" (SCL_PIN), [SCLIN] "I" (SCL_IN),
      [SDADDR] "I"  (SDA_DDR), [SDAPIN] "I" (SDA_PIN), [SDAIN] "I" (SDA_IN)); 
  return true; // fooling the compiler
}

uint8_t i2c_read(bool last)
{
  __asm__ __volatile__ 
    (
     " ldi	r23,0x01 \n\t"
     "_Li2c_read_bit: \n\t"
     " sbi	%[SCLDDR],%[SCLPIN]	;force SCL low         ;; 2C \n\t" 
     " cbi	%[SDADDR],%[SDAPIN]	;release SDA(prev. ACK);; 4C \n\t" 
     " nop \n\t"
     " nop \n\t"
     " nop \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half	;delay T/2             ;; 4C+X \n\t" 
#endif
     " cbi	%[SCLDDR],%[SCLPIN]	;release SCL           ;; 6C + X \n\t" 
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half	;delay T/2             ;; 6C + 2X \n\t" 
#endif
     " cln                              ; clear N-bit          ;; 7C + 2X \n\t"
     " nop \n\t "
     " nop \n\t "
     " nop \n\t "
     " sbis     %[SCLIN], %[SCLPIN]     ;check for SCL high    ;; 9C +2X \n\t" 
     " rcall    ass_i2c_wait_scl_high \n\t"
     " brmi     _Li2c_read_return       ;return if timeout     ;; 10C + 2X\n\t"
     " clc		  	        ;clear carry flag      ;; 11C + 2X\n\t" 
     " sbic	%[SDAIN],%[SDAPIN]	;if SDA is high        ;; 11C + 2X\n\t" 
     " sec			        ;set carry flag        ;; 12C + 2X\n\t" 
     " rol	r23		        ;store bit             ;; 13C + 2X\n\t" 
     " brcc	_Li2c_read_bit	        ;while receiv reg not full \n\t"
     "                         ;; 15C + 2X for one bit loop \n\t" 
     
     "_Li2c_put_ack: \n\t" 
     " sbi	%[SCLDDR],%[SCLPIN]	;force SCL low         ;; 2C \n\t" 
     " cpi	r24,0                                          ;; 3C \n\t" 
     " breq	_Li2c_put_ack_low	;if (ack=0) ;; 5C \n\t" 
     " cbi	%[SDADDR],%[SDAPIN]	;release SDA \n\t" 
     " rjmp	_Li2c_put_ack_high \n\t" 
     "_Li2c_put_ack_low:                ;else \n\t" 
     " sbi	%[SDADDR],%[SDAPIN]	;force SDA low         ;; 7C \n\t" 
     "_Li2c_put_ack_high: \n\t" 
     " nop \n\t "
     " nop \n\t "
     " nop \n\t "
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half	;delay T/2             ;; 7C + X \n\t" 
#endif
     " cbi	%[SCLDDR],%[SCLPIN]	;release SCL           ;; 9C +X \n\t" 
     " cln                              ;clear N               ;; +1 = 10C\n\t"
     " nop \n\t "
     " nop \n\t "
     " sbis	%[SCLIN],%[SCLPIN]	;wait SCL high         ;; 12C + X\n\t" 
     " rcall    ass_i2c_wait_scl_high \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half	;delay T/2             ;; 11C + 2X\n\t" 
#endif
     "_Li2c_read_return: \n\t"
     " nop \n\t "
     " nop \n\t "
     "sbi	%[SCLDDR],%[SCLPIN]	;force SCL low so SCL=H is short\n\t"
     " mov	r24,r23                                        ;; 12C + 2X \n\t"
     " clr	r25                                            ;; 13 C + 2X\n\t"
     " ret                                                     ;; 17C + X"
     ::
      [SCLDDR] "I"  (SCL_DDR), [SCLPIN] "I" (SCL_PIN), [SCLIN] "I" (SCL_IN),
      [SDADDR] "I"  (SDA_DDR), [SDAPIN] "I" (SDA_PIN), [SDAIN] "I" (SDA_IN) 
     ); 
  return ' '; // fool the compiler!
}

#endif

#ifndef _TSL2561_H_
#define _TSL2561_H_

#define TSL2561_VISIBLE 2                   // channel 0 - channel 1
#define TSL2561_INFRARED 1                  // channel 1
#define TSL2561_FULLSPECTRUM 0              // channel 0

#define TSL2561_LUX_LUXSCALE      (14)      // Scale by 2^14
#define TSL2561_LUX_RATIOSCALE    (9)       // Scale ratio by 2^9
#define TSL2561_LUX_CHSCALE       (10)      // Scale channel values by 2^10
#define TSL2561_LUX_CHSCALE_TINT0 (0x7517)  // 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_LUX_CHSCALE_TINT1 (0x0FE7)  // 322/81 * 2^TSL2561_LUX_CHSCALE

// T, FN and CL package values
#define TSL2561_LUX_K1T           (0x0040)  // 0.125 * 2^RATIO_SCALE
#define TSL2561_LUX_B1T           (0x01f2)  // 0.0304 * 2^LUX_SCALE
#define TSL2561_LUX_M1T           (0x01be)  // 0.0272 * 2^LUX_SCALE
#define TSL2561_LUX_K2T           (0x0080)  // 0.250 * 2^RATIO_SCALE
#define TSL2561_LUX_B2T           (0x0214)  // 0.0325 * 2^LUX_SCALE
#define TSL2561_LUX_M2T           (0x02d1)  // 0.0440 * 2^LUX_SCALE
#define TSL2561_LUX_K3T           (0x00c0)  // 0.375 * 2^RATIO_SCALE
#define TSL2561_LUX_B3T           (0x023f)  // 0.0351 * 2^LUX_SCALE
#define TSL2561_LUX_M3T           (0x037b)  // 0.0544 * 2^LUX_SCALE
#define TSL2561_LUX_K4T           (0x0100)  // 0.50 * 2^RATIO_SCALE
#define TSL2561_LUX_B4T           (0x0270)  // 0.0381 * 2^LUX_SCALE
#define TSL2561_LUX_M4T           (0x03fe)  // 0.0624 * 2^LUX_SCALE
#define TSL2561_LUX_K5T           (0x0138)  // 0.61 * 2^RATIO_SCALE
#define TSL2561_LUX_B5T           (0x016f)  // 0.0224 * 2^LUX_SCALE
#define TSL2561_LUX_M5T           (0x01fc)  // 0.0310 * 2^LUX_SCALE
#define TSL2561_LUX_K6T           (0x019a)  // 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6T           (0x00d2)  // 0.0128 * 2^LUX_SCALE
#define TSL2561_LUX_M6T           (0x00fb)  // 0.0153 * 2^LUX_SCALE
#define TSL2561_LUX_K7T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7T           (0x0018)  // 0.00146 * 2^LUX_SCALE
#define TSL2561_LUX_M7T           (0x0012)  // 0.00112 * 2^LUX_SCALE
#define TSL2561_LUX_K8T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8T           (0x0000)  // 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8T           (0x0000)  // 0.000 * 2^LUX_SCALE

// Auto-gain thresholds
#define TSL2561_AGC_THI_13MS      (4850)    // Max value at Ti 13ms = 5047
#define TSL2561_AGC_TLO_13MS      (100)
#define TSL2561_AGC_THI_101MS     (36000)   // Max value at Ti 101ms = 37177
#define TSL2561_AGC_TLO_101MS     (200)
#define TSL2561_AGC_THI_402MS     (63000)   // Max value at Ti 402ms = 65535
#define TSL2561_AGC_TLO_402MS     (500)

// Clipping thresholds
#define TSL2561_CLIPPING_13MS     (4900)
#define TSL2561_CLIPPING_101MS    (37000)
#define TSL2561_CLIPPING_402MS    (65000)

#endif

/*
// Sketch to explore the luminosity sensor TSL2561 (breakout board by Adafruit)

#define SDA_PORT PORTD
#define SDA_PIN 3
#define SCL_PORT PORTD
#define SCL_PIN 5

#include <SoftI2CMaster.h>
#include "TSL2561Soft.h"

#define ADDR 0x72

//------------------------------------------------------------------------------
unsigned long computeLux(unsigned long channel0, unsigned long channel1){
  
  // Make sure the sensor isn't saturated! 
  uint16_t clipThreshold = TSL2561_CLIPPING_402MS;;

  // Return 0 lux if the sensor is saturated 
  if ((channel0 > clipThreshold) || (channel1 > clipThreshold))
  {
    Serial.println(F("Sensor is saturated"));
    return 32000;
  }

  // Find the ratio of the channel values (Channel1/Channel0) 
  unsigned long ratio1 = 0;
  if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

  // round the ratio value 
  unsigned long ratio = (ratio1 + 1) >> 1;

  unsigned int b, m;

  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
    {b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
  else if (ratio <= TSL2561_LUX_K2T)
    {b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
  else if (ratio <= TSL2561_LUX_K3T)
    {b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
  else if (ratio <= TSL2561_LUX_K4T)
    {b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
  else if (ratio <= TSL2561_LUX_K5T)
    {b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
  else if (ratio <= TSL2561_LUX_K6T)
    {b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
  else if (ratio <= TSL2561_LUX_K7T)
    {b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
  else if (ratio > TSL2561_LUX_K8T)
    {b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}

  unsigned long temp;
  temp = ((channel0 * b) - (channel1 * m));

  // Do not allow negative lux value 
  if (temp < 0) temp = 0;

  // Round lsb (2^(LUX_SCALE-1)) 
  temp += (1 << (TSL2561_LUX_LUXSCALE-1));

  // Strip off fractional portion 
  uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

  return lux;
}

void setup(void) {

  Serial.begin(19200);
  Serial.println("Initializing ...");
  i2c_init();

  if (!i2c_start(ADDR | I2C_WRITE)) Serial.println(F("Device does not respond"));
  if (!i2c_write(0x80)) Serial.println(F("Cannot address reg 0"));
  if (!i2c_write(0x03)) Serial.println(F("Cannot wake up"));
  i2c_stop();
}  

void loop (void) {
  unsigned int low0, high0, low1, high1;
  unsigned int chan0, chan1;
  unsigned int lux;

  delay(1000);
  i2c_start(ADDR | I2C_WRITE);
  i2c_write(0x8C);
  i2c_rep_start(ADDR | I2C_READ);
  low0 = i2c_read(false);
  high0 = i2c_read(false);
  low1 = i2c_read(false);
  high1 = i2c_read(true);
  i2c_stop();
  Serial.print(F("Raw values: chan0="));
  Serial.print(chan0=(low0+(high0<<8)));
  Serial.print(F(" / chan1="));
  Serial.println(chan1=(low1+(high1<<8)));
  lux = computeLux(chan0,chan1);
  Serial.print(F("Lux value="));
  Serial.println(lux);
}
*/






