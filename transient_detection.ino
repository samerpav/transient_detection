/*
derived from the MMA8452Q example by Jim Lindblom @ SparkFun

Hardware setup:
 MMA8452 Breakout ------------ Arduino
 3.3V --------------------- 3.3V
 SDA ----------------------- sda (10k pull-up)
 SCL ----------------------- sda (10k pull-up)
 INT1 ---------------------- digital pin 4
 GND ---------------------- GND
 
 LED ----- digital pin 13
*/
 
#include "i2c.h"  // not the wire library, can't use pull-ups

#define MMA8452_ADDRESS 0x1D  // 0x1D is default 

int int1Pin = 4;  //  cannot use pin 2 or 3 on Leonardo

void setup()
{
  byte c;

  Serial.begin(57600);

  // set up the interrupt pin (active high, push-pull)
  pinMode(int1Pin, INPUT);
  digitalWrite(int1Pin, LOW);

  // Read the WHO_AM_I register, this is a good test of communication
  c = readRegister(0x0D);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {  
    initMMA8452();
    //while (!Serial); Serial.println("MMA8452Q is online...");
  }
  else
  {
    //while (!Serial);
    Serial.print("Could not connect to MMA8452Q: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
  static byte source;

  if (digitalRead(int1Pin)==1)  
  {
    source = readRegister(0x0C);
    if ((source & 0x20) == 0x20)
      motionDetectHandler();
  }
}

void motionDetectHandler()
{
  byte source = readRegister(0x1E);
  long startClock = millis();
  
  while ( (millis()-startClock) < 2000 )
  {
    digitalWrite(13, HIGH);
    delay(100);              
    digitalWrite(13, LOW);
    delay(100);
  }
}

void initMMA8452()
{
  writeRegister(0x2A, 0x18);  // 100 Hz data rate, standby

  writeRegister(0x2C, 0x02);  // push-pull, active high
  
  writeRegister(0x1D, 0x1E);  // ELE,x,y,z enabled

  writeRegister(0x1F, 0x01);  // threshold     

  writeRegister(0x20, 0x05);  // debounce counter

  writeRegister(0x2D, 0x20);  // interrupt enable register desc, int 1

  writeRegister(0x2E, 0x20);  // interrupt config register desc, int 1

  MMA8452Active();  // Set to active to start reading  
}

void readAccelData(int * destination)
{
  byte rawData[6];  // x/y/z accel register data stored here

  readRegisters(0x01, 6, &rawData[0]);  // Read the six raw data registers into data array

  // Loop to calculate 12-bit ADC and g value for each axis
  for (int i=0; i<6; i+=2)
  {
    destination[i/2] = ((rawData[i] << 8) | rawData[i+1]) >> 4;  // Turn the MSB and LSB into a 12-bit value
    if (rawData[i] > 0x7F)
    {  
      // If the number is negative, we have to make it so manually (no 12-bit data type)
      destination[i/2] = ~destination[i/2] + 1;
      destination[i/2] *= -1;  // Transform into negative 2's complement #
    }
  }
}

// Sets the MMA8452 to active mode.
// Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(0x2A);
  writeRegister(0x2A, c | 0x01);
}

// Read i registers sequentially, starting at address into the dest byte array
void readRegisters(byte address, int i, byte * dest)
{
  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS<<1)); // write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address);	// write register address
  i2cWaitForComplete();

  i2cSendStart();
  i2cSendByte((MMA8452_ADDRESS<<1)|0x01); // write 0xB5
  i2cWaitForComplete();
  for (int j=0; j<i; j++)
  {
    i2cReceiveByte(TRUE);
    i2cWaitForComplete();
    dest[j] = i2cGetReceivedByte(); // Get MSB result
  }
  i2cWaitForComplete();
  i2cSendStop();

  cbi(TWCR, TWEN); // Disable TWI
  sbi(TWCR, TWEN); // Enable TWI
}

// Read a single byte from address and return it as a byte
byte readRegister(uint8_t address)
{
  byte data;

  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address);	// Write register address
  i2cWaitForComplete();

  i2cSendStart();

  i2cSendByte((MMA8452_ADDRESS<<1)|0x01); // Write 0xB5
  i2cWaitForComplete();
  i2cReceiveByte(TRUE);
  i2cWaitForComplete();

  data = i2cGetReceivedByte();	// Get MSB result
  i2cWaitForComplete();
  i2cSendStop();

  cbi(TWCR, TWEN);	// Disable TWI
  sbi(TWCR, TWEN);	// Enable TWI

  return data;
}

// Writes a single byte (data) into address
void writeRegister(unsigned char address, unsigned char data)
{
  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address);	// Write register address
  i2cWaitForComplete();

  i2cSendByte(data);
  i2cWaitForComplete();

  i2cSendStop();
}
