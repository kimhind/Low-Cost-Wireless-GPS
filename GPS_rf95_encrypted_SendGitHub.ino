// The Adafruit GPS libraries are available from http://www.github.com/adafruit/Adafruit_GPS

#include <Adafruit_GPS.h> 
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false

// The following library info is from http://www.airspayce.com/mikem/arduino/RadioHead/
// LoRa Simple Hello World Client with encrypted communications 
// In order for this to compile you MUST uncomment the #define RH_ENABLE_ENCRYPTION_MODULE line
// at the bottom of RadioHead.h, AND you MUST have installed the Crypto directory from arduinolibs:
// http://rweather.github.io/arduinolibs/index.html
//  Philippe.Rochat'at'gmail.com
//  06.07.2017
#include <RH_RF95.h> //http://www.airspayce.com/mikem/arduino/RadioHead/
#include <RHEncryptedDriver.h> //http://www.airspayce.com/mikem/arduino/RadioHead/
#include <Speck.h>

RH_RF95 rf95(8, 3);    // Instance of a LoRa driver - The pinouts for the Adafruit LoRa Feather are 8 and 3 respectively for Chip Select and IRQ
Speck myCipher;   // Instance of a Speck block ciphering
RHEncryptedDriver myDriver(rf95, myCipher); // Instance of the driver with RF95 and Encryption

float frequency = 868.0; // Change the frequency here.

unsigned char encryptkey[16] = {"16charEncrypKey"}; // The Encryption Key
unsigned char HexChars[] = {"0123456789ABCDEF"};

#define OUT_State_ON LOW
#define OUT_State_OFF HIGH
uint32_t timer = millis();
int DIP1[] = {5, 6, 9, 10}; //Enter the Pin numbers corresponding to the individual DIP switches
int DIP2[] = {11, 12, 18, 19}; //Enter the Pin numbers corresponding to the individual DIP switches

uint8_t DIPStatusA;
uint8_t DIPStatusB;

void setup() {
  Serial.begin(9600);
  Serial.println("LoRa Simple_Encrypted Server");
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix dataToSend) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" dataToSend
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // For parsing dataToSend, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the dataToSend, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  for (int i = 0; i < 4; i++)
  {
    pinMode(DIP1[i], INPUT);
    pinMode(DIP2[i], INPUT);
  }


  if (!rf95.init())
    Serial.println("LoRa init failed");
  // Setup ISM frequency
  rf95.setFrequency(frequency);
  // Setup Power,dBm
  rf95.setTxPower(20);
  myCipher.setKey(encryptkey, 16);
  delay(4000);
  Serial.println("Setup completed");
}


void loop() {
  int Transmit_Interval = 20;
  char* radiopacket;
  uint8_t MessageLen;
  uint8_t dataToSend[100 + 1];
  char c = GPS.read();
  String GPSPlus;
  if (GPSECHO)
    if (c) Serial.print(c);
  DIPStatusA = get_Switch_status(DIP1);
  DIPStatusB = get_Switch_status(DIP2);

  if (GPS.newNMEAreceived()) {
    //  myDriver.send(dataToSendGPS, sizeof(dataToSendGPS)); // Send out ID + Sensor dataToSend to LoRa gateway
    // a tricky thing here is if we print the NMEA sentence, or dataToSend
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLdataToSend and trytng to print out dataToSend
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;
 

    if ((GPS.seconds % Transmit_Interval) == DIPStatusA)
    {

      radiopacket = GPS.lastNMEA();
      Serial.println("Sending...");
      delay(10);

      MessageLen = strlen(radiopacket) + 2;

      dataToSend[0] = (char)HexChars[DIPStatusA]; //Add the Status of DIP Switch A at the beginning of the telegram
      dataToSend[1] = (char)HexChars[DIPStatusB]; //Add the Status of DIP Switch B as the second character of the telegram

      for (uint8_t i = 2; i <= MessageLen; i++) dataToSend[i] = (uint8_t)radiopacket[i];
      myDriver.send(dataToSend, sizeof(dataToSend)); // Send out ID + Sensor dataToSend to LoRa gateway
      Serial.print("Sent: ");
      Serial.println((char *)&dataToSend);

    }
  }
}

uint8_t get_Switch_status(int DIPA[])
{
  uint8_t Status = 0;
  uint8_t BinaryVals[] = {1, 2, 4, 8};
  for (int i = 0; i < 4; i++)
  {
    if (digitalRead(DIPA[i]) == OUT_State_ON)  {
      Status = Status + BinaryVals[i];
    }
  }
  return (Status);
}


