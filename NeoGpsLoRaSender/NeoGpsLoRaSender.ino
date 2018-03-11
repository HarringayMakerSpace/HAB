/* ------------------------------------------------------------------
 * 
 * Sends GPS position over LoRa for near space balloon tracking
 * 
 * Written to try to be as simple and easy to understand as possible to make
 * it easy to get started with. There are other better trackers, for example:
 *  - https://github.com/daveake/FlexTrack
 *  - https://github.com/LoRaTracker
 * but they're quite complex and (I found) hard to work out what they do. 
 *  
 * Libraries used:
 * 
 * NeoGPS appears to be the best, most current GPS library
 *  - https://github.com/SlashDevin/NeoGPS
 *  
 * AltSoftSerial is the fastest and lowest overhead serial library
 *  - https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
 *  
 * Arduino Lora is a good library for the SX127x LoRa radio chips 
 *  - https://github.com/sandeepmistry/arduino-LoRa
 * 
 * Author: Ant Elder
 * License: Apache License v2
 * -----------------------------------------------------------------*/
//#include <SPI.h>
#include <LoRa.h>
#include <NMEAGPS.h> // NeoGPS

const String SENTENCE_ID = "HMS2";  // <---- change this to yours

#include <AltSoftSerial.h>
// this only uses GPS Rx not Tx as it doesn't send commands to the GPS 
// pins used depend on MCU: ATMEGA328 Rx=8 or 32u4 Rx=13
AltSoftSerial gpsPort; 
#define GPS_PORT_NAME "AltSoftSerial"
#define DEBUG_PORT Serial

// LoRa device pins
//// Wemos LoRa shield
//#define LORA_LORA_NSS_PIN 16
//#define LORA_RESET_PIN 16
// Feather 32u4 LoRa
//#define LORA_LORA_NSS_PIN 8
//#define LORA_RESET_PIN 4
// Electrodragon Loraduino
#define LORA_LORA_NSS_PIN 10
#define LORA_RESET_PIN 9

const long LORA_FREQUENCY = 433998500;
const int LORA_SPREADING_FACTOR = 9;
const long LORA_BANDWIDTH = 31.25E3;
const int LORA_CODING_RATE = 6;
const boolean LORA_IMPLICIT_HEADERS = false;
const boolean LORA_CRC = false;
//const boolean LORA_RATE_OPTIMIZATION = true;

NMEAGPS  gps; // This parses the GPS characters

float lastGoodLat, lastGoodLong;
int lastGoodAlt, gpsSatellites;
String gpsTime = "00:00:00";

int sentenceCount = 0;
unsigned long lastSendMillis;

void setup() {
  Serial.begin(115200); 
  //while (!Serial);
  Serial.println("LoRa Sender");

  gpsPort.begin(9600);  

  LoRa.setPins(LORA_LORA_NSS_PIN, LORA_RESET_PIN); 
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setTxPower(7);
  (LORA_CRC ? LoRa.enableCrc() : LoRa.disableCrc());
  //LoRA.setRateOptimization(LORA_RATE_OPTIMIZATION);  
}

void loop() {
  readGPS();

  if (millis() - lastSendMillis > 3000) {
    doSend();
    lastSendMillis = millis();
  }
}

void readGPS() {
  gps_fix  gpsFix;
  while (gps.available( gpsPort )) {
    gpsFix = gps.read();
  }
  if (gpsFix.valid.location) {
    lastGoodLat = gpsFix.latitude();
    lastGoodLong = gpsFix.longitude();
  }
  if (gpsFix.valid.altitude) {
    lastGoodAlt = gpsFix.altitude();
  }
  if (gpsFix.valid.satellites) {
    gpsSatellites = gpsFix.satellites;
  }
  if (gpsFix.valid.time) {
    gpsTime = 
      String((gpsFix.dateTime.hours < 10 ? "0" : "")) + gpsFix.dateTime.hours  + ":" +
      (gpsFix.dateTime.minutes < 10 ? "0" : "") + gpsFix.dateTime.minutes  + ":" + 
      (gpsFix.dateTime.seconds < 10 ? "0" : "") + gpsFix.dateTime.seconds;
      
  }
}

void doSend() {
  String sentence = makeSentance();
  Serial.print(sentence);
  LoRa.beginPacket(LORA_IMPLICIT_HEADERS);
  LoRa.print(sentence);
  LoRa.endPacket();
  Serial.println(" ...sent");
}

String makeSentance() {
  String sentence = SENTENCE_ID + ',' +
    sentenceCount++ + ',' +
    gpsSatellites + ',' +
    gpsTime + "," +  
    String(lastGoodLat, 6) + "," +   
    String(lastGoodLong, 6) + "," +   
    lastGoodAlt; 

  sentence += xorChecksum(sentence);
  return "$$" + sentence + "\n";
}

String xorChecksum(String s) {
  uint8_t b = s.charAt(0);
  for (int i=1; i<s.length(); i++) {
    b ^= s.charAt(i);
  }
  return String( ( b < 16 ? "*0" : "*" )) + String(b, HEX);
}
