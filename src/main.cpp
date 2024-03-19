#include <Arduino.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Time.h>
#include "AsyncUDP.h"
#include <AHT20.h>
#include "SparkFun_ENS160.h"
#include "BME280_I2C.h"
#include "htmls.h"
#include <Wire.h>
#include "EEPROM.h"

#define TIMERDELAY 30    // Delay between measurements in seconds
#define TOTALENTRIES 57600  // Total entries to be stored in the EEPROM
#define DAILYENTRIES 144    // How many measurements expected to be done during a day. If every 24h/10min = 144
#define EEPROMMARGIN 128    // Bytes reserved in the beginning of the EEPROM, before the EEPROM space of the measurements
#define DATASIZE 16         // The sum of all enabled stored data types. 8 bytes time_t, plus 3 * int16_t and one uint_16 = 16 bytes

#define AHT21_ADDRESS 0x38
#define ENS160_ADDRESS 0x53
#define BMP280_ADDRESS 0x76

#define TIMEZONE "EET-2EEST,M3.5.0/3,M10.5.0/4"

DNSServer dnsServer;
AsyncWebServer server(80);
EEPROMClass STORAGE("storage");

uint32_t rows_sent = 0;
RTC_SLOW_ATTR unsigned long lastTime = 0;   
const unsigned long timerDelay = 30000;

RTC_SLOW_ATTR float * regtemp;
RTC_SLOW_ATTR float * reghum;
RTC_SLOW_ATTR float * regpres;
RTC_SLOW_ATTR uint16_t * regtvoc;
RTC_SLOW_ATTR uint16_t * regco2;
RTC_SLOW_ATTR time_t * regtime;

//RTC_SLOW_ATTR struct tm * timeinfo;
//RTC_SLOW_ATTR struct timeval tv;
RTC_SLOW_ATTR static unsigned long acquiredTime = 0;
RTC_SLOW_ATTR static unsigned long previousTime = 0;

RTC_SLOW_ATTR static short day_step = 0; // iterates over the day. Shouldn't be so important, but it's here just in case
RTC_SLOW_ATTR static uint8_t week_it = 0;    // this might be useless to track... but can also speed up the server access

  //placeholder values that will be replaced by any realistic measurement
RTC_FAST_ATTR float histtemperaturemax[7] = {-404.0, -404.0, -404.0, -404.0, -404.0, -404.0, -404.0};
RTC_FAST_ATTR float histtemperaturemin[7] = {404.0, 404.0, 404.0, 404.0, 404.0, 404.0, 404.0};
RTC_FAST_ATTR float histhumiditymax[7] = {-404.0, -404.0, -404.0, -404.0, -404.0, -404.0, -404.0};
RTC_FAST_ATTR float histhumiditymin[7] = {404.0, 404.0, 404.0, 404.0, 404.0, 404.0, 404.0};
RTC_FAST_ATTR float histpressure[7] = {4040.2, 4040.2, 4040.2, 4040.2, 4040.2, 4040.2, 4040.2};
RTC_FAST_ATTR uint16_t histtvoc[7] = {0};

  //these will all be used for the arrays
  /*
RTC_SLOW_ATTR time_t * d_time;
RTC_SLOW_ATTR float * d_temp;
RTC_SLOW_ATTR float * d_hum;
RTC_SLOW_ATTR float * d_pres;
RTC_SLOW_ATTR uint16_t * d_tvoc;
RTC_SLOW_ATTR uint16_t * d_co2;
  */
  //d_ arrays correspond to the daily measurements
RTC_SLOW_ATTR time_t d_time[DAILYENTRIES] = {0};
RTC_SLOW_ATTR float d_temp[DAILYENTRIES] = {0};
RTC_SLOW_ATTR float d_hum[DAILYENTRIES] = {0};
RTC_SLOW_ATTR float d_pres[DAILYENTRIES] = {0};
RTC_SLOW_ATTR uint16_t d_tvoc[DAILYENTRIES] = {0};
RTC_SLOW_ATTR uint16_t d_co2[DAILYENTRIES] = {0};

RTC_SLOW_ATTR uint32_t step = 0; //iterator for the regtab array. It keeps track of what's the next measurement to be stored.

bool static measurement_trigger = false;
bool static midnight_trigger = false;
bool static   wifi_on = false;

AHT20 aht20;

SparkFun_ENS160 ens160;

ESP32Time rtc;

float temperature;
float humidity;
float pressure;
uint16_t tvoc;
uint16_t co2;


BME280_I2C bmp280;

// LOW POWER FUNCTIONS HERE

/*
RTC_FAST_ATTR float weektemp_h;
RTC_FAST_ATTR float weektemp_l;
RTC_FAST_ATTR float weekhum_h;
RTC_FAST_ATTR float weekhum_l;
RTC_FAST_ATTR float weekpres;
RTC_FAST_ATTR uint32_t weektvoc;
RTC_FAST_ATTR unsigned long weektime;
*/

/*
inline void rtc_alloc(){
  d_temp = (float *) malloc (DAILYENTRIES * sizeof (float));
  d_hum = (float *) malloc (DAILYENTRIES * sizeof (float));
  d_pres = (float *) malloc (DAILYENTRIES * sizeof (float));
  d_tvoc = (uint16_t *) malloc (DAILYENTRIES * sizeof (uint16_t));
  d_co2 = (uint16_t *) malloc (DAILYENTRIES * sizeof (uint16_t));
  d_time = (time_t *) malloc (DAILYENTRIES * sizeof (time_t));
}
*/

// END OF NEW LOW POWER FUNCTIONS



//EEPROM FUNCTIONS

#define EEPROM_SIZE 3145728 //size of the flash memory to be reserved. 3145728 = 3MB

void store_measurement(uint32_t step){
  byte byteARR[DATASIZE] = {0xFF};

  for (uint8_t i=0; i<DAILYENTRIES; i++){             //reserving the first 128 bytes of EEPROM for reasons
    //DATASIZE*( step - DAILYENTRIES + i + 1 ) + EEPROMMARGIN -> for the start. This is called for the first time when the value of variable step is DAILYENTRIES.
    STORAGE.put(DATASIZE*(step+i-DAILYENTRIES+1)+EEPROMMARGIN, d_time[i]);   //first variable, 8 bytes (time_t) - UNIX time
    STORAGE.put(DATASIZE*(step+i-DAILYENTRIES+1)+EEPROMMARGIN+8, d_temp[i]);   //second variable, 2 bytes (int16_t) - temperature in degC with 0.01 degree
    STORAGE.put(DATASIZE*(step+i-DAILYENTRIES+1)+EEPROMMARGIN+10, d_hum[i]);    //second variable, 2 bytes (int16_t) - humidity in %
    STORAGE.put(DATASIZE*(step+i-DAILYENTRIES+1)+EEPROMMARGIN+12, d_pres[i]);   //second variable, 2 bytes (int16_t) - pressure in hPa
    STORAGE.put(DATASIZE*(step+i-DAILYENTRIES+1)+EEPROMMARGIN+14, d_tvoc[i]);    //second variable, 2 bytes (uint16_t) - TVOC in ppb
    //STORAGE.put(DATASIZE*(step+i-DAILYENTRIES+1)+EEPROMMARGIN+16, d_co2[i]);    //second variable, 2 bytes (uint16_t) - eCO2 in ppm --- disabled in this example
  };
}

class readEntry {
  public:
    time_t tim;
    float temp;
    float hum;
    float pres;
    uint16_t tvoc;
    //uint16_t co2;
    int16_t t;
    int16_t h;
    int16_t p;
  
  void get_measurement (uint32_t pas){
    STORAGE.get(DATASIZE*pas+EEPROMMARGIN, tim); //reserving the first 128 bytes of EEPROM for reasons
    STORAGE.get(DATASIZE*pas+EEPROMMARGIN+8, t);
    STORAGE.get(DATASIZE*pas+EEPROMMARGIN+10, h);
    STORAGE.get(DATASIZE*pas+EEPROMMARGIN+12, p);
    STORAGE.get(DATASIZE*pas+EEPROMMARGIN+14, tvoc);
    //STORAGE.get(DATASIZE*pas+EEPROMMARGIN+16, co2);
  }
};

// This function will find how many entries are already stored in the EEPROM and increment the "step" iterator.
// It is very important to preserve past measurements and resume properly if the system is powered down.
RTC_FAST_ATTR uint32_t counter = 0, value = 0;
void get_stored_data_length(){
  counter = 0, value = 0;
    while(true) {
        STORAGE.get(18*counter + EEPROMMARGIN, value);
        Serial.printf("Checked the block");
        Serial.println(counter);
        if (value != 0xffffffff) {break;}
        counter++;
    }
  step = counter;
}

RTC_SLOW_ATTR time_t now = time(nullptr);

void update_time(){

  //struct tm * timeinfo;
  //localtime_r(&now, timeinfo);

 //timeval tv;
 //     tv.tv_sec = (time_t)acquiredTime;  // epoch time (seconds)
 //     tv.tv_usec = 0;  

  rtc.setTime(acquiredTime);
  Serial.println("update time function called");
  week_it = rtc.getDayofWeek();
  Serial.printf("weekday: %u ", week_it);
	Serial.println();

  //Serial.println(acquiredTime);
  //uint8_t nWday = (&now/86400L + 4) % 7;
  //  if(!getLocalTime(&timeinfo)) {Serial.println("Failed to obtain time"); weekday = 0;}
  //  else{}
  //previousTime = tv.tv_tsec;  
  //tv.tv_sec = acquiredTime;
  //if(nWday < oldWday) nWday += 7;
  //organise_week(nWday-oldWday); //calls the function to 

}

/*
void organise_week(int offset){
float ow_tempmax, ow_tempmin, ow_hummax, ow_hummin, ow_pres;
uint16_t ow_tvoc, ow_co2;



}
*/


void save_entry(float val0, float val1, float val2, uint16_t val3){

  // This function saves entries to the next 

  regtemp[step] = val0;
  reghum[step] = val1;
  regpres[step] = val2;
  regtvoc[step] = val3;
  regtime[step] = millis();
  step++;
  if(step > TOTALENTRIES){step = 0;};
}


void store_week_data(){
  if (temperature > histtemperaturemax[0]) histtemperaturemax[0] = temperature;
  if (temperature < histtemperaturemin[0]) histtemperaturemin[0] = temperature;
  if (humidity > histhumiditymax[0]) histhumiditymax[0] = humidity;
  if (humidity < histhumiditymin[0]) histhumiditymin[0] = humidity;
  if (pressure < histpressure[0]) histpressure[0] = pressure;
  if (tvoc > histtvoc[0]) histtvoc[0] = tvoc;
}


void shift_week(){
  Serial.println("shift_week function executed."); //debug
  float ow_tempmax, ow_tempmin, ow_hummax, ow_hummin, ow_pres;
  uint16_t ow_tvoc, ow_co2;
  for (uint8_t i = 6; i>0; i--)
    {
    histtemperaturemax[i]=histtemperaturemax[i-1];
    histtemperaturemin[i]=histtemperaturemin[i-1];
    histhumiditymax[i]=histhumiditymax[i-1];
    histhumiditymin[i]=histhumiditymin[i-1];
    histpressure[i]=histpressure[i-1];
    histtvoc[i]=histtvoc[i-1];
    };
  histtemperaturemax[0], histtemperaturemin[0] = temperature;
  histhumiditymax[0], histhumiditymin[0] = humidity;
  histpressure[0] = pressure;
  histtvoc[0] = tvoc;
}


void update_params(){
  //digitalWrite(ENS_CS, LOW);
  //temperature = aht20.getTemperature();
  humidity = aht20.getHumidity();
  //ENS160.setTempAndHum(temperature, humidity);
  
    //ens160.setOperatingMode(0x02);
    ens160.checkDataStatus();

    Serial.print("Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up): ");
    Serial.println(ens160.getFlags());
	
		Serial.print("Air Quality Index (1-5) : ");
		Serial.println(ens160.getAQI());

    co2 = ens160.getECO2();
    tvoc = ens160.getTVOC();
    //ens160.setOperatingMode(0x00);
    
  //ENS160.setPWRMode(ENS160_SLEEP_MODE);
  //pressure = bmp280.readPressure();
  bmp280.read();
  temperature = bmp280.temperature;
  pressure = bmp280.pressure;

  Serial.println("New measurement");
  Serial.printf("Temperature = %.2f °C", temperature);
    Serial.println();
  Serial.printf("Humidity = %.2f %%", humidity);
    Serial.println();
  Serial.printf("Pressure = %.2f hPa", pressure);
    Serial.println();
  Serial.printf("Total Volatile Organic Compounds: = %u ppb", tvoc);
  Serial.println();
  	Serial.printf("CO2 concentration: %u ppm", co2);
	Serial.println();
    Serial.print("Time: ");
    Serial.println(rtc.getTime("RTC0: %A, %B %d %Y %H:%M:%S"));

  //save_entry(temperature, humidity, pressure, tvoc);

  //step++;     //increments the big counter
  Serial.println("variable step value = ");  Serial.println(step); //debug
  day_step++; //increments the daily counter
  Serial.println("variable day_step value = ");  Serial.println(day_step); //debug
  if (day_step >= DAILYENTRIES) {
        shift_week(); day_step=0;
    }
  else store_week_data();
}



String processor(const String& var){
  
  if(var == "TEM"){
    return String(temperature);
  }
  else if(var == "HUM"){
    return String(humidity);
  }
    else if(var == "PRES"){
    return String(pressure);
  }
    else if(var == "POLU"){
    return String(tvoc);
  }
    else if(var == "WDAY"){
    return String(week_it);
  }

    else if(var == "PARMS"){ //parameters order = max temperature, min temperature, max humidity, min humidity, average pressure, max TVOC.
      String combined = "";
      //char* combined = "";
      //char[384] combined;
      for(float i: histtemperaturemax){combined+= String(i, 1) + ",";};
      for(float i: histtemperaturemin){combined+= String(i, 1) + ",";};
      for(float i: histhumiditymax){combined+= String(i, 1) + ",";};
      for(float i: histhumiditymin){combined+= String(i, 1) + ",";};
      for(float i: histpressure){combined+= String(i, 1) + ",";};
      for(int i = 0; i<6; i++){combined+= String(histtvoc[i]) + ",";};
      combined+= String(histtvoc[6]);

      /*
      for (uint8_t i = 0; i<7; i++){
        combined += String(histtemperaturemax[i], 1) + ",";
        combined += String(histtemperaturemin[i], 1) + ",";
        combined += String(histhumiditymax[i], 1) + ",";
        combined += String(histhumiditymin[i], 1) + ",";
        combined += String(histpressure[i], 1) + ","
        combined += String(histtvoc[i]) + ",";
        combined.pop_back();
      };
      */
    return combined;
  }
 
  return String();
}

void getHistory(String& str){

    static char row[256];
    static time_t tim;
    static int16_t t;
    static int16_t h;
    static uint16_t p;
    static uint16_t vc;

    STORAGE.get(DATASIZE*rows_sent+EEPROMMARGIN, tim);
    STORAGE.get(DATASIZE*rows_sent+EEPROMMARGIN+8, t);
    STORAGE.get(DATASIZE*rows_sent+EEPROMMARGIN+10, h);
    STORAGE.get(DATASIZE*rows_sent+EEPROMMARGIN+12, p);
    STORAGE.get(DATASIZE*rows_sent+EEPROMMARGIN+14, vc);

    snprintf(row, 256, "%llu;%i.%i;%i.%i;%u.%u;%u\n", tim, t/100, t%100, h/100, h%100, p/100+500, p%100, tvoc);
    rows_sent++;
}


void sendHistory(AsyncWebServerRequest *request){
    //static size_t maxLen = 256;
    AsyncWebServerResponse *response = request->beginChunkedResponse("text/plain", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      static byte currentIndexForChunk = 0;

    static size_t dataLen = (step+1)*(DATASIZE*3+16);
    rows_sent = 0;

    //Write up to "maxLen" bytes into "buffer" and return the amount written.
    //index equals the amount of bytes that have been already sent
    //You will be asked for more data until 0 is returned
    //Keep in mind that you can not delay or yield waiting for more data!

    if (index == 0) {
      currentIndexForChunk = 0; 
      strncat((char*)buffer, "First line\n", 12);
      //for(int i=0; i<TOTALENTRIES; i++){} 
      return strlen((char*) buffer);
    } else buffer[0] = '\0';

    if (currentIndexForChunk == step) { // we are done, send the footer
      strncat((char*)buffer, "\nLast line", 11);
      currentIndexForChunk++;
      return strlen((char*) buffer);
    } else if (currentIndexForChunk > step) { // the footer has been sent, we close this request by sending a length of 0
      // but for the sake of the demo, we add something in the log to make it grow for next refresh
      return 0;
    }


    size_t initialBufferLength = strlen((char*) buffer);

 
      //String row = String(regtime[i])+";"+String(regtemp[i], 2)+";"+String(reghum[i], 2)+";"+String(regpres[i], 2)+";"+String(regpol[i])+"\n";
        //static String row = "\n";
        static char row[256];
        static time_t tim;
        static int16_t t;
        static int16_t h;
        static uint16_t p;
        static uint16_t vc;
        //static uint16_t co2;

        STORAGE.get(DATASIZE*rows_sent+EEPROMMARGIN, tim);
        Serial.println(tim);
        STORAGE.get(DATASIZE*rows_sent+EEPROMMARGIN+8, t);
        Serial.println(t);
        STORAGE.get(DATASIZE*rows_sent+EEPROMMARGIN+10, h);
        Serial.println(h);
        STORAGE.get(DATASIZE*rows_sent+EEPROMMARGIN+12, p);
        Serial.println(p);
        STORAGE.get(DATASIZE*rows_sent+EEPROMMARGIN+14, vc);
        Serial.println(vc);
        
        snprintf(row, 256, "%llu;%i.%i;%i.%i;%u.%u;%u\n", tim, t/100, t%100, h/100, h%100, (p/100)+500, p%100, vc);
        strcpy((char*)buffer, row);
        //memcpy (buffer, &row, row.length());
/*
        row += (String)tim + ";";
        row += (String)(t/100) + "." + (String)(t%100) + ";";
        row += (String)(h/100) + "." + (String)(h%100) + ";";
        row += (String)(p/100+500) + "." + (String)(p%100) + ";";
        row += (String)tvoc + ";";
*/
      Serial.println(row);
      rows_sent ++;
      currentIndexForChunk++;
      return strlen((char*) buffer);
    //return mySource.read(buffer, maxLen);
    });
    response->addHeader("Server","SunnyBreeze History");
    request->send(response);

}

class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {

  server.on("/time", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", settime_html);
    if (request->hasParam("settime")) {
      acquiredTime = request->getParam("settime")->value().toInt();
      update_time();
      Serial.printf("Time received: %lu \n", acquiredTime);
      Serial.println();
    }
  });

  server.on("/forcestore", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", "last measurement stored to STORAGE");
    Serial.println("Client called the forecestore function. The values to be stored as time, temp, hum, pres and tvoc are:");

    STORAGE.put(DATASIZE*(step+1)+EEPROMMARGIN, rtc.getEpoch());   //first variable, 8 bytes (time_t) - UNIX time
    Serial.println(rtc.getEpoch());
    STORAGE.put(DATASIZE*(step+1)+EEPROMMARGIN+8, (int16_t)(temperature*100));   //second variable, 2 bytes (int16_t) - temperature in degC with 0.01 degree
    Serial.println((int16_t)temperature*100);
    STORAGE.put(DATASIZE*(step+1)+EEPROMMARGIN+10, (int16_t)(humidity*100));    //second variable, 2 bytes (int16_t) - humidity in %
    Serial.println((int16_t)humidity*100);
    STORAGE.put(DATASIZE*(step+1)+EEPROMMARGIN+12, (int16_t)((pressure-500)*100));   //second variable, 2 bytes (int16_t) - pressure in hPa
    Serial.println((int16_t)(pressure-500)*100);
    STORAGE.put(DATASIZE*(step+1)+EEPROMMARGIN+14, (int16_t)tvoc);
    Serial.println((int16_t)tvoc);

    step++;

  });

  server.on("/history", HTTP_GET, sendHistory);
  }

  virtual ~CaptiveRequestHandler() {}


  bool canHandle(AsyncWebServerRequest *request) {
    //request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, processor);
  };
};


inline void disableWiFi() __attribute__((always_inline));
inline void disableWiFi(){
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
}


/*
inline void mode_normal() __attribute__((always_inline));
void mode_normal(){
  ens160.setOperatingMode(0x00);
  //schedule_time();
  ens160.setOperatingMode(0x02);
  dnsServer.processNextRequest();
  //events.send(String(pressure).c_str(),"pressure",millis());
}
*/


void setupServer() {
  
  server.on("/time", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", settime_html);
    if (request->hasParam("settime")) {
      acquiredTime = request->getParam("settime")->value().toInt();
      update_time();
      Serial.printf("Time received: %lu \n", acquiredTime);
    }
  });

  server.onNotFound([](AsyncWebServerRequest *request){request->send_P(200, "text/html", index_html, processor);});

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", index_html, processor);
  });
}

void setup() {
  setCpuFrequencyMhz(240);
  STORAGE.begin(EEPROM_SIZE);
  Serial.begin(115200);
  Wire.begin();

  //while (!Serial) {}; // wait for serial port to connect. Needed for native USB port only, easier debugging :P
  //delay(2000); //debug stuff
  /*
  regtemp = (float *) ps_malloc (TOTALENTRIES * sizeof (float));
  reghum = (float *) ps_malloc (TOTALENTRIES * sizeof (float));
  regpres = (float *) ps_malloc (TOTALENTRIES * sizeof (float));
  regtvoc = (uint16_t *) ps_malloc (TOTALENTRIES * sizeof (unsigned int));
  regtime = (unsigned long *) ps_malloc (TOTALENTRIES * sizeof (unsigned long));
  */

  if(psramInit()){
    Serial.println("\nPSRAM is correctly initialized");
  }else{
    Serial.println("PSRAM not available");
  }

  Serial.println("Beginning Flash Storage discovery");
  get_stored_data_length(); //discovers how many rows were saved in the STORAGE already, then continues from the first empty space.
  Serial.printf("A total of %u entries have been discovered in the Flash Storage\n", step);
  Serial.println();

  setCpuFrequencyMhz(80);

  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1); // Timezone set to Helsinki
  //tzset();
  bmp280.begin();
  if (aht20.begin() == false){Serial.println("AHT20 not detected. Please check wiring.");} else {Serial.println("AHT20 acknowledged.");}
  ens160.begin();
  if( !ens160.begin() ){Serial.println("Could not communicate with the ENS160, check wiring.");}
  ens160.setOperatingMode(SFE_ENS160_RESET);
  ens160.setOperatingMode(SFE_ENS160_STANDARD);
  ens160.setOperatingMode(0x02);

  update_params();
  step--; day_step--;


  WiFi.mode(WIFI_AP);
  WiFi.softAP("WeatherSpot");
  Serial.println("Starting DNS Server");
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);  //only when requested from AP

  setupServer();

  server.begin();
  //historyserver.begin();
}


void loop() {

  dnsServer.processNextRequest();

  //void mode_normal();

  if ((millis() - lastTime) > TIMERDELAY * 1000) {
    ens160.setOperatingMode(0x02);
    update_params();
    //store_week_data();
    Serial.println("...");
    Serial.println();
    
    lastTime = millis();
  }
  
}