/* This example will read all channels from the AS7341 and print out reported values */

#include "RegisterMappings.h"
#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_AS7341.h>
#include <U8g2lib.h>

//U8g2 Contructor
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 16, /* clock=*/ 5, /* data=*/ 4);
// Alternative board version. Uncomment if above doesn't work.
// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 4, /* clock=*/ 14, /* data=*/ 2);

u8g2_uint_t offset;     // current offset for the scrolling text
u8g2_uint_t width;      // pixel width of the scrolling text (must be lesser than 128 unless U8G2_16BIT is defined
const char *text = " "; // scroll this text from right to left
Adafruit_AS7341 as7341;

int loopC = 0;

void setup() {
  Wire.begin(); // Initialize I2C communication
  Serial.begin(115200);

  // start display
  u8g2.begin();

  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    Serial.println("comm loop");
    delay(1);
  }
  
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  // set bit 0 PON to 1 on ENABLE to turn on sensor
  Wire.write(COLOR16_REG_ENABLE);
  Wire.write(0x01);
  Wire.endTransmission();

  // set LED on bit 7 and set current with other bits
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_LED);
  Wire.write(0x82);
  Wire.endTransmission();

  //as7341.setATIME(100);
  //as7341.setASTEP(999);
  //as7341.setGain(AS7341_GAIN_256X);
  
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_ATIME);
  Wire.write(0x64); // set ATIME to 100
  Wire.endTransmission();

  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_ASTEP_LSB);
  Wire.write(0x3E7); // set ASTEP to 999
  Wire.endTransmission();

  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_AS7343_CFG1);
  Wire.write(128); // set GAIN to 128
  Wire.endTransmission();

  Serial.println("SETUP END:");
}

void loop() {
  
  // read REG_BANK ->  0x58 to 0x66 needs 1 else 0
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_CFG_0_REG_BANK);
  Wire.endTransmission();
  Wire.requestFrom(COLOR16_DEVICE_ADDRESS, 1);

  if (Wire.available()) {
    byte value = Wire.read();
    Serial.print("**REG_BANK**: ");
    for (int bit = 7; bit >= 0; bit--) {
      byte bitValue = (value >> bit) & 0x01;
      Serial.print(bitValue, BIN);
    }
    Serial.println(); // Print a newline after all bits
  } else {
    Serial.println("Failed to read REG_BANK");
  }
  Serial.print("loop: ");
  Serial.println(++loopC);
  // Read all channels at the same time and store in as7341 object

  // read ENABLE REG 
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_ENABLE);
  Wire.endTransmission();
  Wire.requestFrom(COLOR16_DEVICE_ADDRESS, 1);

  if (Wire.available()) {
    byte value = Wire.read();
    Serial.print("COLOR16_REG_ENABLE: ");
    for (int bit = 7; bit >= 0; bit--) {
      byte bitValue = (value >> bit) & 0x01;
      Serial.print(bitValue, BIN);
    }
    Serial.println(); // Print a newline after all bits
  } else {
    Serial.println("Failed to read COLOR16_REG_ENABLE");
  }
  
  // Read LED REG
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_LED);
  Wire.endTransmission();
  Wire.requestFrom(COLOR16_DEVICE_ADDRESS, 1);

  if (Wire.available()) {
    byte value = Wire.read();
    Serial.print("COLOR16_REG_LED: ");
    for (int bit = 7; bit >= 0; bit--) {
      byte bitValue = (value >> bit) & 0x01;
      Serial.print(bitValue, BIN);
    }
    Serial.println(); // Print a newline after all bits
  } else {
    Serial.println("Failed to read COLOR16_REG_LED");
  }

  // Read ATIME
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_ATIME);
  Wire.endTransmission();
  Wire.requestFrom(COLOR16_DEVICE_ADDRESS, 1);

  if (Wire.available()) {
    byte value = Wire.read();
    Serial.print("COLOR16_REG_ATIME : ");
    for (int bit = 7; bit >= 0; bit--) {
      byte bitValue = (value >> bit) & 0x01;
      Serial.print(bitValue, BIN);
    }
    Serial.println(); // Print a newline after all bits
  } else {
    Serial.println("Failed to read COLOR16_REG_ATIME ");
  }

  // Read ASTEP
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_ASTEP_LSB);
  Wire.endTransmission();
  Wire.requestFrom(COLOR16_DEVICE_ADDRESS, 1);

  if (Wire.available()) {
    byte value = Wire.read();
    Serial.print("COLOR16_REG_ASTEP_LSB : ");
    for (int bit = 7; bit >= 0; bit--) {
      byte bitValue = (value >> bit) & 0x01;
      Serial.print(bitValue, BIN);
    }
    Serial.println(); // Print a newline after all bits
  } else {
    Serial.println("Failed to read COLOR16_REG_ASTEP_LSB ");
  }

  // Read GAIN
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_AS7343_CFG1);
  Wire.endTransmission();
  Wire.requestFrom(COLOR16_DEVICE_ADDRESS, 1);

  if (Wire.available()) {
    byte value = Wire.read();
    Serial.print("COLOR16_REG_AS7343_CFG1 : ");
    for (int bit = 7; bit >= 0; bit--) {
      byte bitValue = (value >> bit) & 0x01;
      Serial.print(bitValue, BIN);
    }
    Serial.println(); // Print a newline after all bits
  } else {
    Serial.println("Failed to read COLOR16_REG_AS7343_CFG1 ");
  }

  // READING CHANNELS - set bit 1 SP_EN of ENABLE REG to enable readings
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_ENABLE);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(COLOR16_DEVICE_ADDRESS, 1);
  
  // Read ASTATUS to latch spectral channels
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(COLOR16_REG_ASTATUS);
  Wire.endTransmission();

  byte startingReg = COLOR16_REG_ASTATUS;
  byte firstDataReg = 0x95;
  int numOfDataRegs = 36;

  // start reading spectral data registers
  Wire.beginTransmission(COLOR16_DEVICE_ADDRESS);
  Wire.write(firstDataReg);
  Wire.endTransmission();
  Wire.requestFrom(COLOR16_DEVICE_ADDRESS, 36 * 2); // 16 bit values = 2 bytes

  uint16_t spectralData[numOfDataRegs];
  for (int i = 0; i < numOfDataRegs; i++) {
    if (Wire.available() >= 2) {
      byte lowByte = Wire.read();
      byte highByte = Wire.read();
      spectralData[i] = (highByte << 8) | lowByte;
    } else {
      Serial.println("Failed to read spectral data");
      break;
    }
  }

  // Print the stored spectral data
  Serial.println("Spectral Data:");

    // Print the spectral data in a row-by-column format
    for (int i = 0; i < 36; i++) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(spectralData[i]);
      Serial.print("\t");

      if ((i + 1) % 6 == 0) {
        Serial.println(); // Start a new row after every 6 channels
      }
    }

  // SO FAR ONLY CHANNEL 3 IS RELABLY CHANGING WITH COVERING/UNCOVERING SENSOR
  // Delay before repeating (adjust as needed)

  u8g2.setFont(u8g2_font_cursor_tr);
  u8g2.setCursor(24,12);
  u8g2.drawGlyph(10, 15, 72);
  u8g2.setFont(u8g2_font_victoriamedium8_8u);

  u8g2.setCursor(24,12);
  u8g2.print("CH3:  ");
  u8g2.print(spectralData[2]);
  u8g2.setCursor(24,24);
  u8g2.print("CH4:  ");
  u8g2.print(spectralData[4]);
  u8g2.sendBuffer();

  delay(3000);
  Serial.println("");
  u8g2.clearDisplay();
}