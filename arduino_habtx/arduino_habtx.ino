/*

UKHASnet HAB Transmitter-only Code by Phil Crump M0DNY

*/

#include <SPI.h>
#include <string.h>
#include "RFM69Config.h"
#include "RFM69.h"
//#include "DallasTemperature.h"
#include <TinyGPS.h>
//#include "TinyGPSpp.h"
#include "NodeConfig.h"

//************* Misc Setup ****************/
//DeviceAddress ds_addr;
float battV=0.0;
uint8_t n;
unsigned long beacon_next = 0;
uint8_t data_count = 97; // 'a'
unsigned long debugcnt = 0;
char data[64];

// Singleton instance of the radio
RFM69 rf69(RFM_TEMP_FUDGE); // parameter: RFM temperature calibration offset (degrees as float)
//OneWire ow(9);  // on pin PB1 (arduino: 9)
//DallasTemperature sensors(&ow);
//TinyGPSPlus gps;
TinyGPS gps;

int32_t lat_int=0, lat_dec=0, lon_int=0, lon_dec=0;
int16_t gps_alt=0;
uint8_t gps_sats=0; 

#ifdef ENABLE_BATTV_SENSOR
float sampleBattv() {
  // External 5:1 Divider
  return ((float)analogRead(BATTV_PIN)*1.1*5*BATTV_FUDGE)/1023.0;
}
#endif

int gen_Data(){
Serial.println("gen_Data");
  sprintf(data, "%c%cL%ld.%05ld,%ld.%05ld,%dS%i", num_repeats, data_count, lat_int,lat_dec,lon_int,lon_dec,gps_alt,gps_sats);
 // sensors.setWaitForConversion(false);
 // sensors.requestTemperatures();
  delay(1000);
  
  float temp = 0;//sensors.getTempC(ds_addr);

  char* tempStr;
  char tempStrA[12]; //make buffer large enough for 7 digits
  tempStr = dtostrf(temp,6,1,tempStrA);
  while( (strlen(tempStr) > 0) && (tempStr[0] == 32) )
  {
     strcpy(tempStr,&tempStr[1]);
  }
  sprintf(data,"%sT%s",data,tempStr);
  
  #ifdef ENABLE_BATTV_SENSOR
  battV = sampleBattv();
  char* battStr;
  char tempStrB[14]; //make buffer large enough for 7 digits
  battStr = dtostrf(battV,7,2,tempStrB);
  while( (strlen(battStr) > 0) && (battStr[0] == 32) )
  {
     strcpy(battStr,&battStr[1]);
  }
  sprintf(data,"%sV%s",data,battStr);
  #endif
  
  return sprintf(data,"%s[%s]",data,id);
}

void setup() 
{
  analogReference(INTERNAL); // 1.1V ADC reference
  randomSeed(analogRead(6));
  delay(1000);
  
  Serial.begin(9600);
  
 // sensors.begin();
 // sensors.getAddress(ds_addr, 0);
 // sensors.setResolution(ds_addr, 12);
  
  while (!rf69.init()){
    delay(100);
  }
  
  int packet_len = gen_Data();
  rf69.send((uint8_t*)data, packet_len, rfm_power);
  
  rf69.setMode(RFM69_MODE_SLEEP);
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
    
    for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  
  
  if (newData)
  {
    float flat, flon;
    unsigned long age;
    
    lat_int = (int32_t)flat;
    lat_dec = (flat-lat_int)*100000;
    if(lat_dec<0) lat_dec*=-1;
    
    lon_int = (int32_t)flon;
    lon_dec = (flon-lon_int)*100000;
    if(lon_dec<0) lon_dec*=-1;
    
    gps_sats = gps.satellites();
    
    gps_alt = gps.f_altitude();
    
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  
  //process_gps();
    
    if(millis()>(beacon_next+BEACON_INTERVAL)) { // If we've had 2x battery intervals
      Serial.println("Bacon Interval ;)");
        beacon_next = millis()+BEACON_INTERVAL;
        data_count++;

        if(data_count > 122){
            data_count = 98; //'b'
        }
    
        int packet_len = gen_Data();
         Serial.print("data: ");
         Serial.println(data);
        rf69.send((uint8_t*)data, packet_len, rfm_power);
        rf69.setMode(RFM69_MODE_SLEEP);
    }
 Serial.println("Loop Ended");
}
 /*
void process_gps() {
  Serial.println("Processing GPS");
    if(gps.location.isValid()) {
        double lat = gps.location.lat();
        double lon = gps.location.lng();
        
        lat_int = (int32_t)lat;
        lat_dec = (lat-lat_int)*100000;
        if(lat_dec<0) lat_dec*=-1;
        
        lon_int = (int32_t)lon;
        lon_dec = (lon-lon_int)*100000;
        if(lon_dec<0) lon_dec*=-1;
    }
    
    if(gps.altitude.isValid()) gps_alt = (int16_t)gps.altitude.meters();
    
    if(gps.satellites.isValid()) gps_sats = (uint8_t)gps.satellites.value();
}
*/
