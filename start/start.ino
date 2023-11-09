#include <U8g2lib.h>

//U8g2 Contructor
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 16, /* clock=*/ 5, /* data=*/ 4);
// Alternative board version. Uncomment if above doesn't work.
// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 4, /* clock=*/ 14, /* data=*/ 2);

u8g2_uint_t offset;     // current offset for the scrolling text
u8g2_uint_t width;      // pixel width of the scrolling text (must be lesser than 128 unless U8G2_16BIT is defined
const char *text = " "; // scroll this text from right to left

#include <Adafruit_AS7341.h>

#include <Wire.h>

#define ID 0xE0 // AS7343 ID (10000001)
Adafruit_AS7341 as7341;

void setup(void) {
    Wire.begin(); 
    Serial.begin(115200);
    u8g2.begin();
    u8g2.setFont(u8g2_font_cursor_tf);
    u8g2.setCursor(8,15);
    u8g2.print("^");
}


void loop(void) {
  u8g2.setFont(u8g2_font_victoriamedium8_8u);
  Serial.print("test");
  u8g2.setCursor(24,12);
  u8g2.print("BPM:  ");
  u8g2.print(80);
  u8g2.setCursor(24,24);
  u8g2.print("SPO2:  ");
  u8g2.print(100);
  Serial.print("\nRegister data: ");

  byte AS343_DEVICE_ID = 0x39; // Device Addr
  byte enable = 0x80; // Reg Addr
  byte REG_BANK = 0xBF; //enables access to other Regs

  // Send the data to the I2C device
  Wire.beginTransmission(AS343_DEVICE_ID);

  Wire.write(enable);
  Wire.write(0x03); // ~ 00000011 bits 0 and 1 ON
  Wire.endTransmission();

  u8g2.sendBuffer();
  delay(1000);
}