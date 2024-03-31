#include <Arduino.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Time.h>
#include "AsyncUDP.h"
#include <AHT20.h>
#include "SparkFun_ENS160.h"
//#include "BME280_I2C.h"
#include "htmls.h"
#include <Wire.h>
#include <FS.h>
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


////////////////////////////////////////////////////// -- BMP280 lib... because that thing isn't loading properly it seems


class BME280_I2C{
    
    public:

    const uint8_t ADDRESS = 0x76; //default address is 0x76

    BME280_I2C(){}

    BME280_I2C(const uint8_t& x) : ADDRESS(x) {}

    uint8_t osrs_t = 1;             //Temperature oversampling x 1
    uint8_t osrs_p = 1;             //Pressure oversampling x 1
    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t mode = 3;               //Normal mode
    uint8_t t_sb = 5;               //Tstandby 1000ms
    uint8_t filter = 0;             //Filter off 
    uint8_t spi3w_en = 0;           //3-wire SPI Disable
    
    unsigned long int hum_raw,temp_raw,pres_raw;
    signed long int t_fine;
    double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
    signed long int temp_cal;
    unsigned long int press_cal,hum_cal;

    float temperature {0};
    float pressure {0};
    float humidity {0};

    void read(){
        readData();
          temp_cal = calibration_T(temp_raw);
          temp_act = (double)temp_cal / 100.0;
          press_cal = calibration_P(pres_raw);
          press_act = (double)press_cal / 100.0;
          hum_cal = calibration_H(hum_raw);
          hum_act = (double)hum_cal / 1024.0;
          temperature = (float)temp_act;
          pressure = (float)press_act;
          humidity = (float)hum_act;
    }

    bool begin(){
        //Wire.begin();
        writeReg(0xF2,ctrl_hum_reg);
        writeReg(0xF4,ctrl_meas_reg);
        writeReg(0xF5,config_reg);
        
        //begin Trim
        uint8_t i=0;
        Wire.beginTransmission(ADDRESS);
        Wire.write(0x88);
        Wire.endTransmission();
        Wire.requestFrom(ADDRESS,24);
        while(Wire.available()){
            data[i] = Wire.read();
            i++;
        }
            
        Wire.beginTransmission(ADDRESS);
        Wire.write(0xA1);
        Wire.endTransmission();
        Wire.requestFrom(ADDRESS,1);
        data[i] = Wire.read();
        i++;
        
        Wire.beginTransmission(ADDRESS);
        Wire.write(0xE1);
        Wire.endTransmission();
        Wire.requestFrom(ADDRESS,7);
        while(Wire.available()){
            data[i] = Wire.read();
            i++;    
        }
        dig_T1 = (data[1] << 8) | data[0];
        dig_T2 = (data[3] << 8) | data[2];
        dig_T3 = (data[5] << 8) | data[4];
        dig_P1 = (data[7] << 8) | data[6];
        dig_P2 = (data[9] << 8) | data[8];
        dig_P3 = (data[11]<< 8) | data[10];
        dig_P4 = (data[13]<< 8) | data[12];
        dig_P5 = (data[15]<< 8) | data[14];
        dig_P6 = (data[17]<< 8) | data[16];
        dig_P7 = (data[19]<< 8) | data[18];
        dig_P8 = (data[21]<< 8) | data[20];
        dig_P9 = (data[23]<< 8) | data[22];
        dig_H1 = data[24];
        dig_H2 = (data[26]<< 8) | data[25];
        dig_H3 = data[27];
        dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
        dig_H5 = (data[30]<< 4) | ((data[29] >> 4) & 0x0F);
        dig_H6 = data[31];
        return true;
    }

    float getTemperature(){ //returns temperature in ºC
        readData();
        temp_cal = calibration_T(temp_raw);
        temp_act = (double)temp_cal / 100.0;
        return((float)temp_act);
    }

    float getPressure(){ //returns pressure in hPa
        readData();
        press_cal = calibration_P(pres_raw);
        press_act = (double)press_cal / 100.0;
        return((float)press_act);
    }

    float getHumidity(){ //returns relative atmospheric humidity in %
        readData();
        hum_cal = calibration_H(hum_raw);
        hum_act = (double)hum_cal / 1024.0;
        return((float)hum_act);
    }
    
    void readData(){
        int i = 0;
        uint32_t data[8];
        Wire.beginTransmission(ADDRESS);
        Wire.write(0xF7);
        Wire.endTransmission();
        Wire.requestFrom(ADDRESS,8);
        while(Wire.available()){
            data[i] = Wire.read();
            i++;
        }
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
        hum_raw  = (data[6] << 8) | data[7];
    }

    private:
        uint8_t data[32];

        uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
        uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
        uint8_t ctrl_hum_reg  = osrs_h;

        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;
        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
        int8_t  dig_H1;
        int16_t dig_H2;
        int8_t  dig_H3;
        int16_t dig_H4;
        int16_t dig_H5;
        int8_t  dig_H6;

        void writeReg(uint8_t reg_address, uint8_t data){
            Wire.beginTransmission(ADDRESS);
            Wire.write(reg_address);
            Wire.write(data);
            Wire.endTransmission();    
        }

    signed long int calibration_T(signed long int adc_T)    {        
        signed long int var1, var2, T;
        var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
        
        t_fine = var1 + var2;
        T = (t_fine * 5 + 128) >> 8;
        return T; 
    }

    unsigned long int calibration_P(signed long int adc_P)    {
        signed long int var1, var2;
        unsigned long int P;
        var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
        var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
        var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
        var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
        var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
        var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
        if (var1 == 0)
        {
            return 0;
        }    
        P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
        if(P<0x80000000)
        {
        P = (P << 1) / ((unsigned long int) var1);   
        }
        else
        {
            P = (P / (unsigned long int)var1) * 2;    
        }
        var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
        var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
        P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
        return P;
    }

    unsigned long int calibration_H(signed long int adc_H)    {
        signed long int v_x1;        
        v_x1 = (t_fine - ((signed long int)76800));
        v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) + 
        ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) * 
        (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
        ((signed long int) dig_H2) + 8192) >> 14));
        v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
        v_x1 = (v_x1 < 0 ? 0 : v_x1);
        v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
        return (unsigned long int)(v_x1 >> 12);   
    }
};

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

uint8_t readingBuffer[BUFFERSIZE];

AHT20 aht20;

SparkFun_ENS160 ens160;

ESP32Time rtc;

float temperature;
float humidity;
float pressure;
uint16_t tvoc;
uint16_t co2;
char timeStr[32];

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
  File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while(file){
      count++;
        Serial.print("  FILE: ");
        Serial.print(file.name());
        Serial.print("\tSIZE: ");
        Serial.println(file.size());

        file = root.openNextFile();
    }
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
  //storingbuffer[DAILYENTRIES*DATASIZE] = timeToStore >> 56;
  //storingbuffer[DAILYENTRIES*DATASIZE+1] = timeToStore >> 48;
  //storingbuffer[DAILYENTRIES*DATASIZE+2] = timeToStore >> 40;
  //storingbuffer[DAILYENTRIES*DATASIZE+3] = timeToStore >> 32;
  storingbuffer[DAILYENTRIES*DATASIZE+4] = timeToStore >> 24;
  storingbuffer[DAILYENTRIES*DATASIZE+5] = timeToStore >> 16;
  storingbuffer[DAILYENTRIES*DATASIZE+6] = timeToStore >> 8;
  storingbuffer[DAILYENTRIES*DATASIZE+7] = timeToStore;
}

bool commitData(){
  File file = SPIFFS.open("/data/"+(String)day_count, FILE_WRITE);

  file.write(storingbuffer, BUFFERSIZE);

    if(!file) return false;
  for(uint8_t i : storingbuffer) file.print(i);
  file.close();
  return true;
}

uint16_t readData(uint16_t index){
  File file = SPIFFS.open("/data/"+(String)index);
  static int j = 0;
  while (file.available() && j < BUFFERSIZE) {
    readingBuffer[j] = file.read();
    j++;
  }
/*
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
  */
  
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
      static size_t dataLen = (DAILYENTRIES*day_count)*(64);
      static size_t daysSent = 0;
      
      if (index == 0) {
        buffer[0]='\0';
        currentIndexForChunk = 0; 
        strcpy((char*)buffer, "First line\n");
        //return strlen((char*) buffer);
      } else buffer[0] = '\0';

      //static File dir = SPIFFS.open("/");
      static File file = SPIFFS.open("/data/"+(String)currentIndexForChunk);

      if (!file) {
        strcpy((char*)buffer, "\nEnd of function\n");
        currentIndexForChunk++;
        return strlen((char*) buffer);
      } else if (currentIndexForChunk > day_count) { // the footer has been sent, we close this request by sending a length of 0
        return 0;
      }

    //Write up to "maxLen" bytes into "buffer" and return the amount written.
    //index equals the amount of bytes that have been already sent
    //You will be asked for more data until 0 is returned
    //Keep in mind that you can not delay or yield waiting for more data!



    size_t initialBufferLength = strlen((char*) buffer);  


        uint32_t ptr = 0;
        while(file.available()) {
            readingBuffer[ptr] = file.read();
            ptr++;
        }
       
        static uint16_t sr;
        static int16_t tr;
        static uint16_t hr;
        static uint16_t pr;
        static uint16_t vcr;

        for (int i = 0; i < DAILYENTRIES; i++)
        {
          char row[64];

          sr = readingBuffer[i*DATASIZE] << 8 | readingBuffer[i*DATASIZE+1];
          tr = readingBuffer[i*DATASIZE+2] << 8 | readingBuffer[i*DATASIZE+3];
          hr = readingBuffer[i*DATASIZE+4] << 8 | readingBuffer[i*DATASIZE+5];
          pr = readingBuffer[i*DATASIZE+6] << 8 | readingBuffer[i*DATASIZE+7];
          vcr = readingBuffer[i*DATASIZE+8] << 8 | readingBuffer[i*DATASIZE+9];
          //co2r = readingBuffer[i*DATASIZE+10] << 8 | readingBuffer[i*DATASIZE+11];

          snprintf(row, 64, "%llu;%f;%f;%f;%u\r\n", sr*2, (float)tr/100, (float)hr/100, (float)pr/100+500, vcr);
          strncat((char*)buffer, "\nTimestamp: ", 64);
        }
          //acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE] << 56;
          //acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+1] << 48;
          //acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+2] << 40;
          //acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+3] << 32;
          acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+4] << 24;
          acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+5] << 16;
          acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+6] << 8;
          acquiredTime = readingBuffer[DAILYENTRIES*DATASIZE+7];
        
        snprintf(timeStr, 32, "%llu", acquiredTime);
        strncat((char*)buffer, "\nTimestamp: ", 13);
        strncat((char*)buffer, timeStr, sizeof(timeStr));
      
      currentIndexForChunk++;
      file.close();
      return strlen((char*) buffer);
    //return mySource.read(buffer, maxLen);

    });
    //response->addHeader("Server","SunnyBreeze History");
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
  Serial.print("variable day_step value = ");  Serial.println(day_step); //debug

  storeData();
  if(day_step >= DAILYENTRIES) {
    shift_week(); day_step=0;
    storeTime();
    commitData();
  }
  else store_week_data();

}




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
    return;
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


/////////////////////////////////////////////////////////////////////////////////



void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("- file renamed");
    } else {
        Serial.println("- rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    Serial.printf("Testing file I/O with %s\r\n", path);

    static uint8_t buf[512];
    size_t len = 0;
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }

    size_t i;
    Serial.print("- writing" );
    uint32_t start = millis();
    for(i=0; i<2048; i++){
        if ((i & 0x001F) == 0x001F){
          Serial.print(".");
        }
        file.write(buf, 512);
    }
    Serial.println("");
    uint32_t end = millis() - start;
    Serial.printf(" - %u bytes written in %lu ms\r\n", 2048 * 512, end);
    file.close();

    file = fs.open(path);
    start = millis();
    end = start;
    i = 0;
    if(file && !file.isDirectory()){
        len = file.size();
        size_t flen = len;
        start = millis();
        Serial.print("- reading" );
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            if ((i++ & 0x001F) == 0x001F){
              Serial.print(".");
            }
            len -= toRead;
        }
        Serial.println("");
        end = millis() - start;
        Serial.printf("- %u bytes read in %lu ms\r\n", flen, end);
        file.close();
    } else {
        Serial.println("- failed to open file for reading");
    }
}
