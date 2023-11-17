#include "RegisterMappings.h"
#include "Arduino.h"
#include "Wire.h"

int loopC = 0;

uint16_t readChannel(byte address)
{
  uint8_t readingL; uint16_t readingH; uint16_t reading = 0;
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(COLOR16_DEVICE_ADDRESS, 2);

  if (2 <= Wire.available())
  {
    readingL = Wire.read();
    readingH = Wire.read();
    readingH = readingH << 8;
    reading = (readingH | readingL);
    return (reading);
  }
  else
  {
    return (0xFFFF); //Error
  }
}

byte readRegister(byte address)
{
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(COLOR16_DEVICE_ADDRESS, 1);

  if (Wire.available())
  {
    return (Wire.read());
  }
  else
  {
    return (0xFF); // Error
  }
}

void writeRegister(byte address, byte value)
{
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

void setup() {
  Wire.begin(); // Initialize I2C communication
  Serial.begin(115200);

  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    Serial.println("comm loop");
    delay(1);
  }

  // TURN SENSOR OFF FOR SETUP
  readRegister(COLOR16_REG_ENABLE);
  writeRegister(COLOR16_REG_ENABLE, 0x03);
  readRegister(COLOR16_REG_ENABLE);

  // SET SMUX TO (0: 6 Channel |1: Reserved; |2: Automatic 12 channel |3: Automatic 18 channel)
  byte currentValue = readRegister(COLOR16_REG_CFG_20);
  // Set bits 6:5 to 3 (binary 11)
  currentValue &= ~(0x60); // Clear bits 6 and 5
  currentValue |= (0x03 << 5); // Set bits 6:5 to 3
  writeRegister(COLOR16_REG_CFG_20, currentValue);
  readRegister(COLOR16_REG_CFG_20);

  // set LED on bit 7 and set current with other bits
  writeRegister(COLOR16_REG_LED, 0x82);
  readRegister(COLOR16_REG_LED);

  // as7341.setATIME(100);
  // as7341.setASTEP(999);
  // as7341.setGain(AS7341_GAIN_256X)

  writeRegister(COLOR16_REG_ATIME, 100); // set ATIME to 100
  readRegister(COLOR16_REG_ATIME);

  writeRegister(COLOR16_REG_ASTEP_LSB, 999); // set ASTEP to 999
  readRegister(COLOR16_REG_ASTEP_LSB);

  writeRegister(COLOR16_REG_AS7343_CFG1, 128); // set GAIN to 128
  readRegister(COLOR16_REG_AS7343_CFG1);

  // set bit 0 PON to 1 on ENABLE to turn on sensor
  writeRegister(COLOR16_REG_ENABLE, 0x01);
  readRegister(COLOR16_REG_ENABLE);

  // read REG_BANK ->  0x58 to 0x66 needs 1 else 0
  byte value = readRegister(COLOR16_CFG_0_REG_BANK);
  Serial.print("**REG_BANK**: ");
  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (value >> bit) & 0x01;
    Serial.print(bitValue, BIN);
  }

  // Start measurements - set bit 1 SP_EN of ENABLE REG to enable readings
  writeRegister(COLOR16_REG_ENABLE, 0x03);
  readRegister(COLOR16_REG_ENABLE);

  Serial.println("SETUP END:");
}

void loop() {

  Serial.println(); // Print a newline after all bits
  Serial.print("loop: ");
  Serial.println(++loopC);

  // read ENABLE REG
  int value = readRegister(COLOR16_REG_ENABLE);
  Serial.print("COLOR16_REG_ENABLE: ");
  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (value >> bit) & 0x01;
    Serial.print(bitValue, BIN);
  }
  Serial.println(); // Print a newline after all bits

  // Read LED REG
  value = readRegister(COLOR16_REG_LED);
  Serial.print("COLOR16_REG_LED: ");
  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (value >> bit) & 0x01;
    Serial.print(bitValue, BIN);
  }
  Serial.println(); // Print a newline after all bits

  // Read ATIME
  value = readRegister(COLOR16_REG_ATIME);
  Serial.print("COLOR16_REG_ATIME : ");
  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (value >> bit) & 0x01;
    Serial.print(bitValue, BIN);
  }
  Serial.println(); // Print a newline after all bits

  // Read ASTEP
  byte astepValue = readRegister(COLOR16_REG_ASTEP_LSB);
  Serial.print("COLOR16_REG_ASTEP_LSB : ");
  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (astepValue >> bit) & 0x01;
    Serial.print(bitValue, BIN);
  }
  Serial.println(); // Print a newline after all bits

  // Read GAIN
  byte gainValue = readRegister(COLOR16_REG_AS7343_CFG1);
  Serial.print("COLOR16_REG_AS7343_CFG1 : ");
  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (gainValue >> bit) & 0x01;
    Serial.print(bitValue, BIN);
  }
  Serial.println(); // Print a newline after all bits

  // Read ASTATUS to latch spectral channels
  byte astatusValue = readRegister(COLOR16_REG_ASTATUS);

  byte firstDataReg = 0x95;
  byte lastDataReg = 0xB7;
  const int totalChannels = (lastDataReg - firstDataReg) / 2 + 1;

  // Buffer to store spectral data
  uint16_t spectralData[totalChannels];

  for (int channel = 0; channel < totalChannels; channel++) {
      int regAddress = firstDataReg + 2 * channel;
      spectralData[channel] = readChannel(regAddress);

      // Print the channel number and its data
      Serial.print("Channel ");
      Serial.print(channel + 1);
      Serial.print(": ");
      Serial.println(spectralData[channel]);
  }

  // CH7-CH19 are 0
  // Delay before repeating (adjust as needed)

  delay(10000);
  Serial.println("");
}
