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

const uint8_t ID = 0x42; // random number to identify this balloon  

// LORA config
double frequency = 433.9985; 
byte spreadingFactor = 11;
byte bandwidth = 0x30; // 125k
byte codingRate = 0x02; // 4_5
boolean explicitHeaders = false;
boolean rateOptimization = true;

// The LORA payload must be as small as possible. At high spreading factors transmission is very slow, 
// several seconds for even small payloads, so longer is more susceptible to interference, 
// and there are legal limits (link?) to permitted time on air.   
struct TBinaryPacket {
  uint8_t   id;
  uint16_t  counter;
  float     latitude;
  float     longitude;
  int32_t   alt;
}  __attribute__ ((packed));

int txCounter = 0; 

void setup() {
  Serial.begin(115200); Serial.println();
  Serial.print(F("HABTracker")); Serial.println(F(", compiled: "  __DATE__  ", " __TIME__));

  while (!Serial){};

  initRF95();

  SetupGPS();
}

void loop() {

  // spin around reading GPS
  unsigned long sm = millis();
  while (millis() < (sm+15000)) {
     CheckGPS();
  }

  TBinaryPacket payload;
  payload.id = ID;
  payload.counter = txCounter++;
  payload.latitude = GPS.Latitude;
  payload.longitude = GPS.Longitude;
  payload.alt = GPS.Altitude;

  unsigned long startSendMs = millis();
  rf95.send((uint8_t*)&payload, sizeof(payload));
  rf95.waitPacketSent();

  Serial.print(F("sent ")); Serial.print(sizeof(payload)); Serial.print(F(" bytes in:")); Serial.println(millis()-startSendMs); 
  Serial.print(F(": ")); Serial.print(payload.counter); Serial.print(F(","));  Serial.print(payload.latitude, 8);
  Serial.print(F(",")); Serial.print(payload.longitude, 8); Serial.print(F(",")); Serial.print(payload.alt); 
  Serial.print(F(",")); Serial.print(GPS.Satellites); Serial.print(F(", Lock=")); Serial.print(GPS.Lock);
  Serial.println();
}

void initRF95() {
  if ( ! rf95.init()) {
    Serial.println(F("rf95 init failed, check wiring. Restarting ..."));
    // ESP.restart();
  }

  // rf95.setTxPower(23, false);
  rf95.setTxPower(10, false); // max UK power?
  
  rf95.setFrequency(frequency);
  rf95Config(bandwidth, spreadingFactor, codingRate, explicitHeaders, rateOptimization); 

  Serial.print(F("RF95 started on frequency ")); Serial.print(frequency, 4);
  Serial.print(F("(MHz), spreading factor ")); Serial.print(spreadingFactor);
  Serial.print(F(", bandwidth ")); Serial.print(bandwidth, HEX);
  Serial.print(F(", coding rate ")); Serial.print(codingRate, HEX);
  Serial.print(F(", headers are ")); Serial.print((explicitHeaders? "explicit" : "implicit"));
  Serial.print(F(", rate optimization ")); Serial.print(rateOptimization);
  Serial.println();
}

void rf95Config(byte bandwidth, byte spreadingFactor, byte codeRate, boolean explicitHeaders, boolean rateOptimisation) {
  RH_RF95::ModemConfig rf95Config;
  rf95Config.reg_1d = bandwidth + codeRate + (explicitHeaders ? 1 : 0);
  rf95Config.reg_1e = (spreadingFactor * 16) + 7;
  rf95Config.reg_26 = (rateOptimisation ? 0x08 : 0x00);
  
  rf95.setModemRegisters(&rf95Config);
}

