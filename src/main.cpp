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
#include "SPIFFS.h"

#define TIMERDELAY 30       // Delay between measurements in seconds
#define TOTALENTRIES 115200 // Delay between measurements in seconds
#define DAILYENTRIES 144    // How many measurements expected to be done during a day. If every 24h/10min = 144
#define EEPROMMARGIN 128    // Bytes reserved in the beginning of the EEPROM, before the EEPROM space of the measurements
#define DATASIZE 10         // The sum of stored data types in a storage row. 2 bytes from second-pair in day, plus one int16_t and three uint_16 = 10 bytes
#define BUFFERSIZE 1448     // Size of the flash storing buffer in Bytes; Consider the amount of measurements per day and data size for this and add 8 for a daily rtc.getEpoch() time recording;
#define FILESIZE 1536       // File size in bytes, to be used. Consider keeping it a multiple of 256 and calculate a minimum of DAILYENTRIES * DATASIZE

#define SPIFFS_SIZE 3145728 //size of the flash memory to be reserved. 3145728 = 3MB

#define AHT21_ADDRESS 0x38
#define ENS160_ADDRESS 0x53
#define BMP280_ADDRESS 0x76

#define TIMEZONE "EET-2EEST,M3.5.0/3,M10.5.0/4"


//////////////////////////////////////////////////////

//////////////////////////////////////////////////////


DNSServer dnsServer;
AsyncWebServer server(80);

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
RTC_SLOW_ATTR uint32_t acquiredTime = 0;
RTC_SLOW_ATTR uint32_t previousTime = 0;
RTC_SLOW_ATTR uint16_t bootcount;

RTC_SLOW_ATTR static short day_step = 0; // iterates over the day. Shouldn't be so important, but it's here just in case
RTC_SLOW_ATTR static uint8_t week_it = 0;    // this might be useless to track... but can also speed up the server access

  //placeholder values that will be replaced by any realistic measurement
RTC_FAST_ATTR float histtemperaturemax[7] = {-404.0, -404.0, -404.0, -404.0, -404.0, -404.0, -404.0};
RTC_FAST_ATTR float histtemperaturemin[7] = {404.0, 404.0, 404.0, 404.0, 404.0, 404.0, 404.0};
RTC_FAST_ATTR float histhumiditymax[7] = {-404.0, -404.0, -404.0, -404.0, -404.0, -404.0, -404.0};
RTC_FAST_ATTR float histhumiditymin[7] = {404.0, 404.0, 404.0, 404.0, 404.0, 404.0, 404.0};
RTC_FAST_ATTR float histpressure[7] = {4040.2, 4040.2, 4040.2, 4040.2, 4040.2, 4040.2, 4040.2};
RTC_FAST_ATTR uint16_t histtvoc[7] = {0};

RTC_FAST_ATTR uint8_t d_data[DAILYENTRIES*DATASIZE];

  //d_ arrays correspond to the daily measurements
RTC_SLOW_ATTR uint16_t d_time[DAILYENTRIES];
RTC_SLOW_ATTR int16_t d_temp[DAILYENTRIES];
RTC_SLOW_ATTR uint16_t d_hum[DAILYENTRIES];
RTC_SLOW_ATTR uint16_t d_pres[DAILYENTRIES];
RTC_SLOW_ATTR uint16_t d_tvoc[DAILYENTRIES];
RTC_SLOW_ATTR uint16_t d_co2[DAILYENTRIES];

RTC_SLOW_ATTR uint8_t storingbuffer[BUFFERSIZE];

RTC_SLOW_ATTR uint16_t day_count = 0; //counts days logged in the system

bool measurement_trigger = false;
bool midnight_trigger = false;
bool wifi_on = false;

uint8_t readingBuffer[BUFFERSIZE];

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

// END OF NEW LOW POWER FUNCTIONS

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
    Serial.println("\r");
  Serial.printf("Humidity = %.2f %%", humidity);
    Serial.println("\r");
  Serial.printf("Pressure = %.2f hPa", pressure);
    Serial.println("\r");
  Serial.printf("Total Volatile Organic Compounds: = %u ppb", tvoc);
  Serial.println("\r");
  	Serial.printf("CO2 concentration: %u ppm", co2);
	Serial.println("\r");
    Serial.print("Time: ");
    Serial.println(rtc.getTime("RTC0: %A, %B %d %Y %H:%M:%S"));

  //save_entry(temperature, humidity, pressure, tvoc);

  day_step++; //increments the daily counter
  Serial.println("variable day_step value = ");  Serial.println(day_step); //debug

  storeData();
  if(day_step >= DAILYENTRIES) {
    shift_week(); day_step=0;
    storeTime();
    commitData();
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

    snprintf(row, 256, "%llu;%i.%i;%i.%i;%u.%u;%u\n", tim, t/100, t%100, h/100, h%100, p/100+500, p%100, tvoc);
    rows_sent++;
}


/////////////////////////////

uint16_t count_files() //this function returns how many entries have been saved in the SPIFFS
{
  static uint16_t count = 0;
  File root = SPIFFS.open("/data/");
  File file = root.openNextFile();
  while(file) count++ ;
  return count;
}


bool storeData(){
//static uint8_t* buffer = (uint8_t *)malloc(BUFFERSIZE);
/*
  float tempy, hummy, pressy;
  tempy = temperature*100;
  hummy = humidity*100;
  pressy = (pressure-500)*100;
*/
  for(uint16_t i = 0; i < day_step; i++){
    static uint16_t storingSeconds = (rtc.getSecond()/2) + (rtc.getMinute()*30) + (rtc.getHour(true)*1800); //storing the time of measurement in seconds of the day, divided by 2
    storingbuffer[i*DATASIZE] = storingSeconds >> 8;
    storingbuffer[i*DATASIZE+1] = storingSeconds;
    storingbuffer[i*DATASIZE+2] = (int16_t)(temperature*100) >> 8;
    storingbuffer[i*DATASIZE+3] = (int16_t)(temperature*100);
    storingbuffer[i*DATASIZE+4] = (uint16_t)(humidity*100) >> 8;
    storingbuffer[i*DATASIZE+5] = (uint16_t)(humidity*100);
    storingbuffer[i*DATASIZE+6] = (uint16_t)((pressure-500)*100) >> 8;
    storingbuffer[i*DATASIZE+7] = (uint16_t)((pressure-500)*100);
    storingbuffer[i*DATASIZE+8] = (uint16_t)(tvoc) >> 8;
    storingbuffer[i*DATASIZE+9] = (uint16_t)(tvoc);
    //storingbuffer[i*DATASIZE+10] = (uint16_t)(co2) >> 8;
    //storingbuffer[i*DATASIZE+11] = (uint16_t)(co2);
  }
  return true;
}

void storeTime(){
  static uint64_t timeToStore = rtc.getEpoch();
  storingbuffer[DAILYENTRIES*DATASIZE] = timeToStore >> 56;
  storingbuffer[DAILYENTRIES*DATASIZE+1] = timeToStore >> 48;
  storingbuffer[DAILYENTRIES*DATASIZE+2] = timeToStore >> 40;
  storingbuffer[DAILYENTRIES*DATASIZE+3] = timeToStore >> 32;
  storingbuffer[DAILYENTRIES*DATASIZE+4] = timeToStore >> 24;
  storingbuffer[DAILYENTRIES*DATASIZE+5] = timeToStore >> 16;
  storingbuffer[DAILYENTRIES*DATASIZE+6] = timeToStore >> 8;
  storingbuffer[DAILYENTRIES*DATASIZE+7] = timeToStore;
}

bool commitData(){
  File file = SPIFFS.open("/data/"+(String)day_count, "w");

    if(!file) return false;

  file.write(storingbuffer, sizeof(storingbuffer));
  for(uint8_t i : storingbuffer) i=0;
  file.close();
  return true;
}

uint16_t readData(uint16_t i){
  File file = SPIFFS.open("/data/"+(String)i, "r");
  size_t len = 0;

  if(file && !file.isDirectory()){
    len = file.size();
    size_t flen = len;
    Serial.print("- reading" );
    while(len){
        size_t toRead = len;
        if(toRead > 512){
            toRead = 512;
        }
        file.read(readingBuffer, toRead);
        len -= toRead;
    }
  }
  file.close();

  return 0;
}

void parseData(uint16_t i, char* ex_string){

  char row[64];

  float tempy, hummy, pressy;

  static uint16_t sr;
  static int16_t tr;
  static uint16_t hr;
  static uint16_t pr;
  static uint16_t vcr;
  //static uint16_t co2r;

  sr = readingBuffer[i*DATASIZE] << 8 | readingBuffer[i*DATASIZE+1];
  tr = readingBuffer[i*DATASIZE+2] << 8 | readingBuffer[i*DATASIZE+3];
  hr = readingBuffer[i*DATASIZE+4] << 8 | readingBuffer[i*DATASIZE+5];
  pr = readingBuffer[i*DATASIZE+6] << 8 | readingBuffer[i*DATASIZE+7];
  vcr = readingBuffer[i*DATASIZE+8] << 8 | readingBuffer[i*DATASIZE+9];
  //co2r = readingBuffer[i*DATASIZE+10] << 8 | readingBuffer[i*DATASIZE+11];

  snprintf(row, 64, "%llu;%f;%f;%f;%u\r\n", sr*2, (float)tr/100, (float)hr/100, (float)pr/100+500, vcr);

  for(int i=0; i < 64; i++) ex_string[i] = row[i];

}

////////////////////////////

void sendHistory(AsyncWebServerRequest *request){
    //static size_t maxLen = 256;
    AsyncWebServerResponse *response = request->beginChunkedResponse("text/plain", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      static byte currentIndexForChunk = 0;
      static File dir = SPIFFS.open("/data/");

    static size_t dataLen = (DAILYENTRIES*day_count)*(64);
    rows_sent = 0;

    //Write up to "maxLen" bytes into "buffer" and return the amount written.
    //index equals the amount of bytes that have been already sent
    //You will be asked for more data until 0 is returned
    //Keep in mind that you can not delay or yield waiting for more data!

    if (index == 0) {
      //buffer='\0';
      currentIndexForChunk = 0; 
      strncat((char*)buffer, "First line\n", 12);
      //for(int i=0; i<TOTALENTRIES; i++){} 
      return strlen((char*) buffer);
    } else buffer[0] = '\0';

    size_t initialBufferLength = strlen((char*) buffer);

    File file = dir.openNextFile();

    if (!file) { // we are done, send the footer
      strncat((char*)buffer, "\nLast line", 11);
      currentIndexForChunk++;
      return strlen((char*) buffer);
    } else if (currentIndexForChunk > day_step) { // the footer has been sent, we close this request by sending a length of 0
      // but for the sake of the demo, we add something in the log to make it grow for next refresh
      return 0;
    }

    while(file){
      char row[64];
        uint32_t ptr = 0;
        while(file.available()) {
            readingBuffer[ptr] = file.read();
            ptr++;
        }

        file.close();

        for(uint32_t i=0; i<DAILYENTRIES; i++){
            parseData(i, row);
            strcpy((char*)buffer, row);
            Serial.println(row);
        }
        
        for (size_t i = 0; i < 4; i++)
        {
            acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE] << 56;
            acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+1] << 48;
            acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+2] << 40;
            acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+3] << 32;
            acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+4] << 24;
            acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+5] << 16;
            acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+6] << 8;
            acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+7];

        }
        ;
        snprintf(row, 64, "\nTimestamp = %" );
        strcpy((char*)buffer, row);
        Serial.println(row);
    }

      rows_sent++;
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

    //storeData();

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
  Serial.begin(115200);
  Wire.begin();

  while (!Serial) {}; // wait for serial port to connect. Needed for native USB port only, easier debugging :P
  //delay(2000); //debug stuff

  if(psramInit()){
    Serial.println("\nPSRAM is correctly initialized");
  }else{
    Serial.println("PSRAM not available");
  }

  Serial.println("Beginning Flash Storage discovery");

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    delay(0xFFFFFFFF);
  }

  day_count = count_files();
  Serial.printf("A total of %u entries have been discovered in the Flash Storage\n", day_count);
  Serial.println();

  setCpuFrequencyMhz(160);

  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1); // Timezone set to Helsinki

  if ( !bmp280.begin() ) Serial.println("BMP280 not detected. Please check wiring."); else Serial.println("BMP280 check OK!.");
  if ( !aht20.begin() ) Serial.println("AHT20 not detected. Please check wiring."); else Serial.println("AHT20 check OK!.");
  if ( !ens160.begin() ) Serial.println("Could not communicate with the ENS160, check wiring."); else Serial.println("ENS160 check OK!");
  ens160.setOperatingMode(SFE_ENS160_RESET);
  delay(100);
  ens160.setOperatingMode(SFE_ENS160_STANDARD);
  delay(100);
  ens160.setOperatingMode(0x02);
  delay(100);
  update_params();
  day_step--;


  WiFi.mode(WIFI_AP);
  WiFi.softAP("WeatherSpot");
  Serial.println("Starting DNS Server");
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);  //only when requested from AP

  setupServer();
  server.begin();

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
