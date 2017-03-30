/*  HAB Tracker
 *  
 *  *** unfinished, work in progress ***
 *  
 *  Broadcast GPS location over LORA.  
 *  Uses an Arduino Pro Mini micorcontroller with GPS and LORA transceiver to broadcast HAB telemtetry.
 *
 *  Wiring connections:
 *    Arduino - RFM98
 *    GND        GND
 *    VCC        VCC
 *    Pin8       NSS
 *    Pin11      MOSI
 *    Pin12      MIS0
 *    Pin13      SCK
 *    Pin2       DIO0
 *  
 * Ant Elder
 * License: Apache License v2
*/
#include <SPI.h>
#include <RH_RF95.h>

RH_RF95 rf95(8, 2); // NSS=pin8, DIO0=pin2
//RH_RF95 rf95(10, 3); // NSS=pin8, DIO0=pin2

#include <Wire.h>

#define GPS_I2C
#define POWERSAVING                        // Comment out to disable GPS power saving

struct TGPS {
  int Hours, Minutes, Seconds;
  unsigned long SecondsInDay;          // Time in seconds since midnight
  float Longitude, Latitude;
  long Altitude;
  unsigned int Satellites;
  int Speed;
  int Direction;
  byte GotTime;
  byte Lock;
  byte psm_status;
  float InternalTemperature;
  float BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte FlightMode;
  byte PowerMode;
} GPS;

//const uint8_t ID = 0x42; // random number to identify this balloon  
const String ID = "$$test1";

// LORA config
double frequency = 433.9985; 
byte spreadingFactor = 11;
byte bandwidth = 0x30; // 28k3
byte codingRate = 0x02; // 4_5
boolean implicitHeaders = false;
boolean rateOptimization = true;

int rf95Power = 10;

int txCounter = 0; 

void setup() {
  Serial.begin(115200); Serial.println();
  Serial.print(F("HABTracker")); Serial.println(F(", compiled: "  __DATE__  ", " __TIME__));

  while (!Serial){};

  initRF95();

  SetupGPS();
}

int txDelay = 2000;

void loop() {

  // spin around reading GPS
  unsigned long sm = millis();
  while (millis() < (sm+txDelay)) {
     CheckGPS();
  }

//  GPS.Latitude = readMcuTemp();

  // $$test1,1,01:23:45,51.58680343,-0.10234091,23*28\n

  String sentence = ID + "," + txCounter++ + "," + 
     (GPS.Hours < 10 ? "0" : "") + GPS.Hours + ":" + 
     (GPS.Minutes < 10 ? "0" : "") + GPS.Minutes + ":" + 
     (GPS.Seconds < 10 ? "0" : "") + GPS.Seconds + "," + 
     String(GPS.Latitude, 6) + "," + 
     String(GPS.Longitude, 6) + "," + 
     GPS.Altitude + "," + 
     rf95Power;  

  sentence += "*" + xorChecksum(sentence) + "\n";

  rf95.setTxPower(rf95Power, false); // max UK power?
  
  unsigned long startMillis = millis();
  rf95Send((uint8_t*)sentence.c_str(), sentence.length());
  rf95.waitPacketSent();
  unsigned long txTime = millis() - startMillis;

  Serial.print(F("sent in ")); Serial.print(txTime); Serial.print(" ms: "); Serial.println(sentence);

  if (txCounter % 10 == 0) {
     Serial.print("Sending again with bandwidth 0x70...");
     rf95.setTxPower(23, false); // max UK power?
     rf95Config(0x70, spreadingFactor, codingRate, implicitHeaders, rateOptimization); 
     rf95Send((uint8_t*)sentence.c_str(), sentence.length());
     rf95.waitPacketSent();
     rf95Config(bandwidth, spreadingFactor, codingRate, implicitHeaders, rateOptimization);     
     Serial.println("sent");
  }

  if (rf95Power == 10) {
    rf95Power = 15;
  } else if (rf95Power == 15) {
    rf95Power = 23;
  } else {
    rf95Power = 10;
  }

}

void initRF95() {
  if ( ! rf95.init()) {
    Serial.println(F("rf95 init failed, check wiring. Restarting ..."));
    // ESP.restart();
  }

  // rf95.setTxPower(23, false);
  rf95.setTxPower(rf95Power, false); // max UK power?
  
  rf95.setFrequency(frequency);
  rf95Config(bandwidth, spreadingFactor, codingRate, implicitHeaders, rateOptimization); 

  Serial.print(F("RF95 started on frequency ")); Serial.print(frequency, 4);
  Serial.print(F("(MHz), spreading factor ")); Serial.print(spreadingFactor);
  Serial.print(F(", bandwidth ")); Serial.print(bandwidth, HEX);
  Serial.print(F(", coding rate ")); Serial.print(codingRate, HEX);
  Serial.print(F(", headers are ")); Serial.print((implicitHeaders? "implicit" : "explicit"));
  Serial.print(F(", rate optimization ")); Serial.print(rateOptimization);
  Serial.println();
}

void rf95Config(byte bandwidth, byte spreadingFactor, byte codingRate, boolean implicitHeaders, boolean rateOptimisation) {
  RH_RF95::ModemConfig rf95Config;

  rf95Config.reg_1d = implicitHeaders | codingRate | bandwidth;
  rf95Config.reg_1e = (spreadingFactor * 16) | 0x04; // 0x04 sets CRC on
  rf95Config.reg_26 = 0x04 | (rateOptimisation ? 0x08 : 0x00); // 0x04 sets AGC on, rateOptimisation should be on for SF 11  and 12

  Serial.print("rf95 config registers 0x1d:"); Serial.print(rf95Config.reg_1d, HEX);
  Serial.print(", 0x1e:"); Serial.print(rf95Config.reg_1e, HEX);
  Serial.print(", 0x26:"); Serial.println(rf95Config.reg_26, HEX);

  rf95.setModeIdle();
  rf95.setModemRegisters(&rf95Config);
}

// returns the XOR checksum of a String
String xorChecksum(String s) {
  byte b = s.charAt(0);
  for (int i=1; i<s.length(); i++) {
    b = b ^ s.charAt(i);
  }
  String checksum = String(b, HEX);
  if (checksum.length() ==1) checksum = "0" + checksum; 
  return checksum;
}

// http://playground.arduino.cc/Main/InternalTemperatureSensor
int readMcuTemp() {

  //set the internal reference and mux
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));              
  ADCSRA |= _BV(ADEN);  //enable the ADC

  delay(20); // wait for voltages to become stable.

  // Start the ADC
  ADCSRA |= _BV(ADSC);                                        
  
  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC)); 

  // read the result, low and high
  unsigned int wADC = ADCW;                                                

  // if temperature reports high, increase this number
  const float kelvin_offset = 307.0;
  // if the temp changes too much or too little adjust this
  const float temp_conversion_slope = 1.15;
                          
  // temp in celsius
  int temp = (wADC - kelvin_offset) / temp_conversion_slope;   

  return temp;
}

// copied from the RadioHead RH_RF95.cpp send and modified to not send the headers
bool rf95Send(const uint8_t* data, uint8_t len) {
    if (len > RH_RF95_MAX_MESSAGE_LEN)
      return false;

    rf95.waitPacketSent(); // Make sure we dont interrupt an outgoing message
    rf95.setModeIdle();

    if (!rf95.waitCAD()) 
      return false;  // Check channel activity

    // Position at the beginning of the FIFO
    rf95.spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);

/* comment out - don't want to send the RadioHead headers
    // The headers
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderTo);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFrom);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderId);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFlags);
*/
    // The message data
    rf95.spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);
    rf95.spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len);
//    spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);

    rf95.setModeTx(); // Start the transmitter
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}

