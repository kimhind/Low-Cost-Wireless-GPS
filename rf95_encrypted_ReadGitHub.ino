/*
 Wireless GPS - Read

 This arduino code is for the Adafruit LoRa Feather M0 combined. 
 This unit is designed to read the data sent from a similar unit comprising of Adafruit LoRa Feather M0 combined with the Adafruit GPS Featherwing and 8 switch DIP switch. 

 12 Feb 2019
 By Kim Hind
 
*/


//  LoRa simple server with encrypted communications 
// In order for this to compile you MUST uncomment the #define RH_ENABLE_ENCRYPTION_MODULE line
// at the bottom of RadioHead.h, AND you MUST have installed the Crypto directory from arduinolibs:
// http://rweather.github.io/arduinolibs/index.html
//  Philippe.Rochat'at'gmail.com
//  06.07.2017

#include <RH_RF95.h> //http://www.airspayce.com/mikem/arduino/RadioHead/
#include <RHEncryptedDriver.h>  //http://www.airspayce.com/mikem/arduino/RadioHead/
#include <Speck.h>

RH_RF95 rf95(8,3);      // Instance of a LoRa driver - The pinouts for the Adafruit LoRa Feather are 8 and 3 respectively for Chip Select and IRQ
Speck myCipher;   // Instance of a Speck block ciphering
RHEncryptedDriver myDriver(rf95, myCipher); // Instance of the driver with RF95 and Encryption

float frequency = 868.0; // Change the frequency here. 
//unsigned char encryptkey[16]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16}; // The very secret key 
unsigned char encryptkey[16]={"16charEncrypKey"}; // The very secret key 

void setup() {
  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  Serial.println("LoRa Simple_Encrypted Server");
  if (!rf95.init())
    Serial.println("LoRa init failed");
  // Setup ISM frequency
  rf95.setFrequency(frequency);
  // Setup Power,dBm
  rf95.setTxPower(13);
  myCipher.setKey(encryptkey, 16);
  delay(4000);
  Serial.println("Setup completed");
}

void loop() {
  if (myDriver.available()) {
    // Should be a message for us now   
    uint8_t buf[myDriver.maxMessageLength()];
    uint8_t len = sizeof(buf);
    if (myDriver.recv(buf, &len)) {
//      Serial.print("Received: ");
      Serial.println((char *)&buf);
    }
    else
    {
        Serial.println("recv failed");
    }
  }
}
