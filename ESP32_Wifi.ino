// Ernest Spahiu April 08/2024
#include "RegisterMappings.h"
#include "Arduino.h"
#include "Wire.h"
#include <WiFi.h>
#include "arduino_secrets.h"
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h" //if not recognized reupload the sketch


#define BUTTON_RED 18
#define BUTTON_BLACK 23
int measurementReq = 0; //if 0 continuous measurementReq, 1 discrete measurementReq
int lastState = HIGH; // the previous state from the input pin
int currentState;     // the current reading from the input pin

int loopC = 0;
AsyncWebServer server(80);
char formattedStrings[9][50]; 
String allStrings;

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
    Serial.begin(115200);
    Wire.begin(); // Initialize I2C communication
    const char* ssid = "COGECO-47D8";
    const char* password = "242345061234";
  
    delay(10);
    Serial.println("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
  
    pinMode(BUTTON_RED, INPUT_PULLUP);
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    
    // set bit 0 PON to 1 on ENABLE to turn on sensor // everything else OFF for setup
    readRegister(COLOR16_REG_ENABLE);
    writeRegister(COLOR16_REG_ENABLE, 0x01);
    readRegister(COLOR16_REG_ENABLE);

    writeRegister(0xe2, 0xf2); // CFG 10
    byte val = readRegister(COLOR16_REG_CFG_20);
    byte newVal = val | 0x60;
    writeRegister(COLOR16_REG_CFG_20, newVal); // SMUX

    writeRegister(COLOR16_REG_AZ_CONFIG, 0xff);
    writeRegister(COLOR16_REG_AGC_GAIN_MAX, 0x99);

    writeRegister(0xdf, 0xa1); //CFG 0
    writeRegister(0xe0, 0x67); // CFG 1
    writeRegister(0xe1, 0x64); // CFG 2
    writeRegister(0xe2, 0x21); // CFG 3

    // set LED on bit 7 and set current with other bits
    writeRegister(COLOR16_REG_LED, 0x82);
    readRegister(COLOR16_REG_LED);

    // as7341.setATIME(1);
    // as7341.setASTEP(999); 
    // as7341.setGain(AS7341_GAIN_256X)

    writeRegister(COLOR16_REG_ATIME, 0); // set ATIME to 0
    readRegister(COLOR16_REG_ATIME);

    writeRegister(COLOR16_REG_ASTEP_LSB, 0xA11A); // set ASTEP to 0xA11A
    readRegister(COLOR16_REG_ASTEP_LSB);

    writeRegister(COLOR16_REG_AS7343_CFG1, 0x0C); // set GAIN to 0x0C
    readRegister(COLOR16_REG_AS7343_CFG1);

    // turn on Spectral measurements and enable SMUX
    writeRegister(COLOR16_REG_ENABLE, 0x03);
    readRegister(COLOR16_REG_ENABLE);

    // read REG_BANK ->  0x58 to 0x66 needs 1 else 0
    byte value = readRegister(COLOR16_CFG_0_REG_BANK);

    byte registerValue = 0; // Initialize with the current value of the register
    byte bitmask = 0b00000011; // Bits 0, 1, and 4 are set to 1 in binary

    // Set the bits by performing a bitwise OR operation
    registerValue |= bitmask;
    // Start measurements - set bit 1 SP_EN of ENABLE REG to enable readings
    writeRegister(COLOR16_REG_ENABLE, registerValue);
    value = readRegister(COLOR16_REG_ENABLE);
    for (int bit = 7; bit >= 0; bit--) {
      byte bitValue = (value >> bit) & 0x01;
    }
  
    // Initialize SPIFFS
    if(!SPIFFS.begin()){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }

    // Print ESP32 Local IP Address
    Serial.println(WiFi.localIP());

    //route to load html page
    server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/index.html");
    });
    //route to load css
    server.on("/main.css", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/main.css", "text/css");
    });
    // image
    server.on("/teamlogoT.png", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/teamlogoT.png", "image/png");
    });
    //jquery 
    server.on("/breakpoints.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/breakpoints.min.js", "text/javascript");
    });
    server.on("/browser.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/browser.min.js", "text/javascript");
    });
    server.on("/jquery.dropotron.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/jquery.dropotron.min.js", "text/javascript");
    });
    server.on("/jquery.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/jquery.min.js", "text/javascript");
    });
    server.on("/main.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/main.js", "text/javascript");
    });
    server.on("/util.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/util.js", "text/javascript");
    });
    

    server.on("/bluechannel", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain",  allStrings.c_str());
    });
    server.on("/measurementReq", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", String(measurementReq).c_str());
    });
      //start server
    server.begin();
}

void loop() {
  // read ENABLE REG
  int value = readRegister(COLOR16_REG_ENABLE);
  
  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (value >> bit) & 0x01;
    
  } // Print a newline after all bits

  // Read LED REG
  value = readRegister(COLOR16_REG_LED);
  
  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (value >> bit) & 0x01;
    
  }
  Serial.println(); // Print a newline after all bits

  // Read ATIME
  value = readRegister(COLOR16_REG_ATIME);

  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (value >> bit) & 0x01;
    
  }; // Print a newline after all bits

  // Read ASTEP
  byte astepValue = readRegister(COLOR16_REG_ASTEP_LSB);
  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (astepValue >> bit) & 0x01;
  }
 // Print a newline after all bits

  // Read GAIN
  byte gainValue = readRegister(COLOR16_REG_AS7343_CFG1);

  for (int bit = 7; bit >= 0; bit--) {
    byte bitValue = (gainValue >> bit) & 0x01;
    
  }
 // Print a newline after all bits

  // Read ASTATUS to latch spectral channels
  byte astatusValue = readRegister(COLOR16_REG_ASTATUS);

  byte firstDataReg = 0x95;
  byte lastDataReg = 0xB8;
  const int totalChannels = (lastDataReg - firstDataReg) / 2 + 1;

  // Buffer to store spectral data
  uint16_t spectralData[totalChannels];

  for (int channel = 0; channel < totalChannels; channel++) {
      int regAddress = firstDataReg + 2 * channel;
      spectralData[channel] = readChannel(regAddress);


      /** 
        Cycle 1: FZ, FY, FXL, NIR, 2xVIS, FD
        Cycle 2: F2, F3, F4, F6, 2xVIS, FD
        Cycle 3: F1, F7, F8, F5, 2xVIS, FD
      */
  }
  
    // Read the state of the switch/button:
  currentState = digitalRead(BUTTON_RED);

  // Check if the button is pressed (i.e., if the current state is LOW,
  // and the last state was HIGH, meaning the button state has changed to pressed)
  if (currentState == LOW && lastState == HIGH) {
    // Toggle measurementReq
    measurementReq = !measurementReq; // This toggles the measurementReq between 0 and 1.
    delay(50); // Debounce delay. Adjust as necessary for your specific button.

    if (measurementReq == 1) {
      Serial.println("Discrete measurementReq");
    } else {
      Serial.println("Continuous measurementReq");
    }
  }
    // Update lastState to the current state
  lastState = currentState;

    if (measurementReq == 1) {
    // Actions for discrete measurementReq
    // Note: Depending on your requirements, you might want to move or remove
    // this block to prevent "Discrete measurementReq" from being printed continuously.
    } else {
      // Actions for continuous mode
      // Note: As above, you may want to adjust this behavior based on your needs.
    }
      // Clear or re-initialize allStrings before starting to concatenate new data
      allStrings = ""; // Reset allStrings to empty

      // Format strings with channel names and values
      sprintf(formattedStrings[0], "F4 Blue:, %u", spectralData[8]);
      sprintf(formattedStrings[1], "F7 Red:, %u", spectralData[13]);
      sprintf(formattedStrings[2], "FY Green:, %u", spectralData[1]);
      sprintf(formattedStrings[3], "FXL Orange:, %u", spectralData[2]);
      sprintf(formattedStrings[4], "F1 Violet:, %u", spectralData[12]);
      sprintf(formattedStrings[5], "F5 Lime:, %u", spectralData[15]);
      sprintf(formattedStrings[6], "F8 Dark Red:, %u", spectralData[14]);
      sprintf(formattedStrings[7], "F6 Brown:, %u", spectralData[9]);
      sprintf(formattedStrings[8], "F3 Cyan:, %u", spectralData[7]);

      // Concatenate formatted strings into allStrings
      for (int i = 0; i < 9; i++) {
          allStrings += formattedStrings[i]; // Concatenate each string
          if (i < 8) { // If not the last string, add a separator
              allStrings += ", ";
          }
      }

      delay(50); // Your existing delay or other logic
}
