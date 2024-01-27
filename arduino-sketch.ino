#include <DNSServer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <time.h>
#include "AsyncUDP.h"
//#include <SPI.h>
#include <AHT20.h>
#include "SparkFun_ENS160.h"
#include <Wire.h>
#include <EEPROM.h>

#define TIMERDELAY 30    // Delay between measurements in seconds
#define TOTALENTRIES 57600  // Total entries to be stored in the EEPROM
#define DAILYENTRIES 144    // How many measurements expected to be done during a day. If every 24h/10min = 144

#define AHT21_ADDRESS 0x38
#define ENS160_ADDRESS 0x53
#define BMP280_ADDRESS 0x76

DNSServer dnsServer;
AsyncWebServer server(80);
//AsyncWebServer timeserver(80);
//AsyncWebServer historyserver(80);
AsyncEventSource events("/events");

RTC_SLOW_ATTR unsigned long lastTime = 0;   
const unsigned long timerDelay = 30000;

RTC_SLOW_ATTR float * regtemp;
RTC_SLOW_ATTR float * reghum;
RTC_SLOW_ATTR float * regpres;
RTC_SLOW_ATTR uint16_t * regpol;
RTC_SLOW_ATTR unsigned long * regtime;
//ps_malloc();
RTC_SLOW_ATTR uint32_t step = 0; //iterator for the regtab array. It keeps track of what's the next measurement to be stored.

bool measurement_trigger = false;
bool midnight_trigger = false;
bool wifi_on = false;

AHT20 aht20;

SparkFun_ENS160 ens160;

float temperature;
float humidity;
float pressure;
uint16_t pollution;



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

    void begin(){
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
        //Serial.println("readData() function called");
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


BME280_I2C bmp280;

/*
int week_f(int i){
  if (week_it + i <7){return week_it + i;}
  else {return week_it+i-7;}
}
*/


// LOW POWER FUNCTIONS HERE

RTC_SLOW_ATTR struct timeval tv;
RTC_SLOW_ATTR unsigned long acquiredTime = 0;
RTC_SLOW_ATTR unsigned long previousTime = 0;

  //placeholder values that will be replaced by any realistic measurement
RTC_FAST_ATTR float histtemperaturemax[7] = {-404.0, -404.0, -404.0, -404.0, -404.0, -404.0, -404.0};
RTC_FAST_ATTR float histtemperaturemin[7] = {404.0, 404.0, 404.0, 404.0, 404.0, 404.0, 404.0};
RTC_FAST_ATTR float histhumiditymax[7] = {-404.0, -404.0, -404.0, -404.0, -404.0, -404.0, -404.0};
RTC_FAST_ATTR float histhumiditymin[7] = {404.0, 404.0, 404.0, 404.0, 404.0, 404.0, 404.0};
RTC_FAST_ATTR float histpressure[7] = {4040.2, 4040.2, 4040.2, 4040.2, 4040.2, 4040.2, 4040.2};
RTC_FAST_ATTR uint16_t histpollution[7] = {0};

  //these will all be used for the arrays
RTC_SLOW_ATTR float * d_temp;
RTC_SLOW_ATTR float * d_hum;
RTC_SLOW_ATTR float * d_pres;
RTC_SLOW_ATTR uint16_t * d_pol;
RTC_SLOW_ATTR unsigned long * d_time;
RTC_SLOW_ATTR uint8_t d_10m_step = 0;

RTC_SLOW_ATTR uint8_t week_it = 0; // iterates over the 7-long "hist" arrays over the week
RTC_SLOW_ATTR uint8_t week_acq = 0; // learns the "day zero" for the iterator above

/*
RTC_FAST_ATTR float weektemp_h;
RTC_FAST_ATTR float weektemp_l;
RTC_FAST_ATTR float weekhum_h;
RTC_FAST_ATTR float weekhum_l;
RTC_FAST_ATTR float weekpres;
RTC_FAST_ATTR uint32_t weekpol;
RTC_FAST_ATTR unsigned long weektime;
*/

inline void rtc_alloc(){
  d_temp = (float *) malloc (DAILYENTRIES * sizeof (float));
  d_hum = (float *) malloc (DAILYENTRIES * sizeof (float));
  d_pres = (float *) malloc (DAILYENTRIES * sizeof (float));
  d_pol = (uint16_t *) malloc (DAILYENTRIES * sizeof (uint16_t));
  d_time = (unsigned long *) malloc (DAILYENTRIES * sizeof (unsigned long));

}

void store_week_data(float temp, float hum, float pres, uint16_t tvoc){
  if (temp > histtemperaturemax[week_it]) histtemperaturemax[week_it] = temp;
  if (temp < histtemperaturemin[week_it]) histtemperaturemin[week_it] = temp;
  if (hum > histhumiditymax[week_it]) histhumiditymax[week_it] = hum;
  if (hum < histhumiditymin[week_it]) histhumiditymin[week_it] = hum;
  if (pres < histpressure[week_it]) histpressure[week_it] = pres;
  if (tvoc > histpollution[week_it]) histpollution[week_it] = tvoc;  
}




// END OF NEW LOW POWER FUNCTIONS



//EEPROM FUNCTIONS

#define EEPROM_SIZE 3145728 //size of the flash memory to be reserved. 3145728 = 3MB

void store_measurement(uint16_t step){
  byte byteARR[18] = {0xFF};

  for (uint8_t i=0; i<DAILYENTRIES; i++){             //reserving the first 128 bytes of EEPROM for reasons
    //18*( step - DAILYENTRIES + i + 1 ) + 128 -> for the start. This is called for the first time when the value of variable step is 144.
    EEPROM.put(18*(step+i-DAILYENTRIES)+146, d_time[i]);   //first variable, 4 bytes (unsigned long) - UNIX time
    EEPROM.put(18*(step+i-DAILYENTRIES)+150, d_temp[i]);   //second variable, 4 bytes (float) - temperature in degC
    EEPROM.put(18*(step+i-DAILYENTRIES)+154, d_hum[i]);    //second variable, 4 bytes (float) - humidity in %
    EEPROM.put(18*(step+i-DAILYENTRIES)+158, d_pres[i]);   //second variable, 4 bytes (float) - pressure in hPa
    EEPROM.put(18*(step+i-DAILYENTRIES)+160, d_pol[i]);    //second variable, 4 bytes (uint16_t) - TVOC in ppb
  };
}

class readEntry {
  public:
    unsigned long tim;
    float temp;
    float hum;
    float pres;
    uint16_t pol;
  
  void get_measurement (uint16_t pas){
    EEPROM.get(18*pas+128, tim); //reserving the first 128 bytes of EEPROM for reasons
    EEPROM.get(18*pas+132, temp);
    EEPROM.get(18*pas+136, hum);
    EEPROM.get(18*pas+140, pres);
    EEPROM.get(18*pas+142, pol);
  }
};

// This function will find how many entries are already stored in the EEPROM and increment the "step" iterator.
// It is very important to preserve past measurements and resume properly if the system is powered down.
RTC_FAST_ATTR uint32_t counter = 0, value = 0;
void get_stored_data_length(){
  counter = 0, value = 0;
    while(true) {
        EEPROM.get(18*counter+128, value);
        Serial.printf("Checked the block");
        Serial.println(counter);
        if (value != 0xffffffff) {break;}
        counter++;
    }
  step = counter;
}

/*
void get_measurement (uint16_t pas){
    EEPROM.get(pas+128, unsigned long d_time[i]); //reserving the first 128 bytes of EEPROM for reasons
    EEPROM.get(pas+132, float d_temp[i]);
    EEPROM.get(pas+136, float d_hum[i]);
    EEPROM.get(pas+140, float d_pres[i]);
    EEPROM.get(pas+142, uint16_t d_pol[i]);
}
*/

/*
void send_db(){
WiFiClient client = WebServer.client();
client.print("HTTP/1.1 200 OK\r\n");
client.print("Content-Disposition: attachment; filename=config.txt\r\n");
client.print("Content-Type: application/octet-stream\r\n");
client.print("Content-Length: 2048\r\n");
client.print("Connection: close\r\n");
client.print("Access-Control-Allow-Origin: *\r\n");
client.print("\r\n");
client.write((const char*)data, 2048);
}
*/

void update_time(){
  if(previousTime = 0){
    previousTime = millis();
  };
  for(int i = 0; i < TOTALENTRIES; i++){
    regtime[i]+= (acquiredTime - previousTime);
  };
  tv.tv_sec = acquiredTime;
}



void save_entry(float val0, float val1, float val2, uint16_t val3){

  // This function saves entries to the next 

  regtemp[step] = val0;
  reghum[step] = val1;
  regpres[step] = val2;
  regpol[step] = val3;
  regtime[step] = millis();
  step++;
  if(step > TOTALENTRIES){step = 0;};
}

void update_params(){
  //digitalWrite(ENS_CS, LOW);
  //temperature = aht20.getTemperature();
  humidity = aht20.getHumidity();
  //ENS160.setTempAndHum(temperature, humidity);
  
    //ens160.setOperatingMode(0x02);
    ens160.checkDataStatus();
	
		Serial.print("Air Quality Index (1-5) : ");
		Serial.println(ens160.getAQI());

		Serial.print("Total Volatile Organic Compounds: ");
		Serial.print(ens160.getTVOC());
		Serial.println("ppb");

		Serial.print("CO2 concentration: ");
		Serial.print(ens160.getECO2());
		Serial.println("ppm");
    pollution = ens160.getTVOC();
    //ens160.setOperatingMode(0x00);
    
  //ENS160.setPWRMode(ENS160_SLEEP_MODE);
  //pressure = bmp280.readPressure();
  bmp280.read();
  temperature = bmp280.temperature;
  pressure = bmp280.pressure;

  Serial.println("New measurement");
  Serial.printf("Temperature = %.2f °C", temperature);
    Serial.println();
  Serial.printf("Humidity = %.2f %", humidity);
    Serial.println();
  Serial.printf("Pressure = %.2f %", pressure);
    Serial.println();
  Serial.printf("TVOC = %u %", pollution);
  Serial.println();
  //save_entry(temperature, humidity, pressure, pollution);



}

void schedule_time(){
  //Serial.printf("time(null) value: %d \n", time(NULL));//debug stuff
  //Serial.println(measurement_trigger == true); //debug stuff

  if((time(NULL) / TIMERDELAY) > 1 && measurement_trigger == true) {
    update_params();
    store_week_data(temperature, humidity, pressure, pollution);
    measurement_trigger = false;
  };
  if ((time(NULL) % TIMERDELAY) > 15 && (time(NULL) % TIMERDELAY) < 20 && measurement_trigger == false)
  {
    measurement_trigger = true;
  };
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
    return String(pollution);
  }
    else if(var == "WDAY"){
    return String(week_it);
  }

    else if(var == "PARMS"){ //parameters order = max temperature, min temperature, max humidity, min humidity, average pressure, max pollution.
      String combined = "";
      for(float i: histtemperaturemax){combined+= String(i, 1) + ",";};
      for(float i: histtemperaturemin){combined+= String(i, 1) + ",";};
      for(float i: histhumiditymax){combined+= String(i, 1) + ",";};
      for(float i: histhumiditymin){combined+= String(i, 1) + ",";};
      for(float i: histpressure){combined+= String(i, 1) + ",";};
      for(int i = 0; i<6; i++){combined+= String(histpollution[i]) + ",";};
      combined+= String(histpollution[6]);
    return combined;
  }
 
  return String();
}

const char index_html[] PROGMEM = R"rawliteral(

<!DOCTYPE HTML>
<html>
<head>
    <title>SunnyBreeze Sensor</title>
<style>
    h1 {font-size: 4rem;}
    h2 {font-size: 2.6rem;}
    h3 {font-size: 2rem;}
    p {font-size: 2.5rem;}
    
    html, body, .background{
        width: 100vw;
        margin: 0;
        padding: 0;
        font-family: 'Roboto', 'Palatino', 'Lucida Sans Unicode', sans-serif;
        color: #fff;
        background: #12022a;
    }
    
    body{
        background: #eee;
        background: linear-gradient(0deg, rgba(16, 58, 226, 0.2) , rgba(181, 222, 0, 0) );
    }
    
    a{ color: #C51; text-decoration: none;}
    a:hover{ color: #bb5;}
    
    .container
    {
        align-items: center;
            justify-content: center;
        text-align: center;
        margin: auto;
        padding: 1rem;
        width: 98vw;
        height: 95vh;
        overflow: auto;
        font-size: 2.5rem;
    
        display: grid;
        grid-template-columns: 1fr 1fr minmax(auto, 2.5fr);
        grid-template-rows: 1fr 2fr 2fr 1fr;
        gap: 0.2rem 0.2rem;
        grid-template-areas:
        "Ti Ti Hi"
        "Te Hu Hi"
        "At Ai Hi"
        "Cr Cr Hi";
        
        background: #eee;
        background: linear-gradient(240deg, rgba(165, 51, 51, 0.3) , rgba(240, 205, 7, 0)  );
    }
    
    .half {
    padding: 10px;
    }
    
    .boxed {
    content: "";    
    clear: both;
    display: grid;
    grid-template-columns: 20%% 3%% 20%% 3%% 20%% 17%% 17%%;
    font-size: 1.6rem;
    align-items: center;
    }
    
    @media (max-device-width: 1280px){
        .container{
            grid-template-columns: 1fr minmax(auto, 2.5fr);
            grid-template-rows: auto;
            grid-template-areas:
            "Ti Ti"
            "Te Hi"
            "Hu Hi"
            "At Hi"
            "Ai Hi"
            "Cr Cr";
            gap: 0.1rem 0.1rem;
            height: max(100vh, 100vw);
        }
        
        .boxed {
        content: "";    
        clear: both;
        display: grid;
        grid-template-columns: 20%% 3%% 20%% 3%% 20%% 17%% 17%%;
        }
        .Hi{height: 100%%;}
    }
    
    @media (max-device-width: 1050px){
        .container{grid-template-columns: 1fr minmax(auto, 3.5fr);}
        h1 {font-size: 48;}
        h2, .container {font-size: 36;}
        h3, .boxed {font-size: 24;}
        p {font-size: 32;}
        
    }
    
    @media (max-device-width: 800px){
        h1 {font-size: 40;}
        h2, .container {font-size: 28;}
        h3, .boxed {font-size: 24;}
        p {font-size: 20;}
    }
    
    .Tem { grid-area: Te; }
    .His { grid-area: Hi; min-width: 40%%;}
    .Hum { grid-area: Hu; }
    .Ti { grid-area: Ti;}
    .AtmP { grid-area: At;}
    .AirQ { grid-area: Ai;}
    .Cr { grid-area: Cr;}
    
    hr{
        border-top: dotted 1px;
    }

    </style>
</head>
<body>
<div class="container">
    <div class="His"><p></p></div>
    <div class="Ti"><h1>Sunny Breeze<br></h1><h2>weather monitor</h2></div>
    <div class="Hum">
        <svg width="5rem" height="10rem" viewBox="0 0 32 32" fill="white"><path d="M23.476 13.993 16.847 3.437a1.04 1.04 0 0 0-1.694 0L8.494 14.043A9.986 9.986 0 0 0 7 19a9 9 0 0 0 18 0 10.063 10.063 0 0 0-1.524-5.007ZM16 26a7.009 7.009 0 0 1-7-7 7.978 7.978 0 0 1 1.218-3.943l.935-1.49 10.074 10.074A6.977 6.977 0 0 1 16 26.001Z" stroke="white"/><path style="fill:none" d="M0 0h32v32H0z"/></svg>
        <br><span id="humf">%HUM%</span> &#37;</div>
    <div class="Tem">
        <svg width="5rem" height="10rem" viewBox="0 0 56 56" fill="white"><path d="M25.012 54.04c6.75 0 12.234-5.485 12.234-12.235 0-3.563-1.476-6.68-4.266-9.211-.515-.469-.632-.727-.632-1.43l.047-21.047c0-4.898-2.977-8.156-7.383-8.156-4.43 0-7.407 3.258-7.407 8.156l.024 21.047c0 .703-.117.961-.61 1.43-2.812 2.531-4.265 5.648-4.265 9.21 0 6.75 5.46 12.235 12.258 12.235Zm0-3.4c-4.875 0-8.836-3.984-8.836-8.835 0-2.93 1.383-5.578 3.867-7.242.727-.493 1.008-.938 1.008-1.899V10.258c0-2.953 1.617-4.852 3.96-4.852 2.321 0 3.915 1.899 3.915 4.852v22.406c0 .961.281 1.406 1.008 1.898 2.484 1.665 3.867 4.313 3.867 7.243 0 4.851-3.938 8.836-8.79 8.836Zm11.742-40.078h5.11a1.35 1.35 0 0 0 1.382-1.359 1.35 1.35 0 0 0-1.383-1.36h-5.11a1.35 1.35 0 0 0-1.382 1.36c0 .727.563 1.36 1.383 1.36Zm0 6.586h5.11a1.35 1.35 0 0 0 1.382-1.359 1.35 1.35 0 0 0-1.383-1.36h-5.11a1.35 1.35 0 0 0-1.382 1.36c0 .727.563 1.36 1.383 1.36ZM24.988 47.477a5.667 5.667 0 0 0 5.672-5.696c0-2.203-1.242-4.008-3.047-4.992-.75-.398-1.008-.68-1.008-1.828V22.516c0-1.22-.703-1.946-1.617-1.946-.89 0-1.617.727-1.617 1.946V34.96c0 1.148-.258 1.43-1.008 1.828-1.804.984-3.047 2.79-3.047 4.992a5.667 5.667 0 0 0 5.672 5.696Zm11.766-23.743h5.11a1.35 1.35 0 0 0 1.382-1.359 1.37 1.37 0 0 0-1.383-1.383h-5.11a1.37 1.37 0 0 0-1.382 1.383c0 .727.563 1.36 1.383 1.36Zm0 6.586h5.11a1.37 1.37 0 0 0 1.382-1.383 1.35 1.35 0 0 0-1.383-1.359h-5.11a1.35 1.35 0 0 0-1.382 1.36 1.37 1.37 0 0 0 1.383 1.382Z" stroke="white"/></svg>
        <br><span id="temf">%TEM%</span> &deg;C</div>
    <div class="AtmP">
        <svg width="5rem" height="10rem" viewBox="0 0 24 24" fill="none"><path d="M20.693 17.33a9 9 0 1 0-17.386 0" stroke="white" stroke-width="1.5" stroke-linecap="round"/><path d="M12.766 15.582c.487.71.144 1.792-.766 2.417-.91.626-2.043.558-2.53-.151-.52-.756-2.314-5.007-3.403-7.637-.205-.495.4-.911.79-.542 2.064 1.96 5.39 5.157 5.909 5.913Z" stroke="white" stroke-width="1.5"/><path d="M12 6v2m-6.364.636L7.05 10.05m11.314-1.414L16.95 10.05m3.743 7.28-1.931-.518m-15.455.518 1.931-.518" stroke="white" stroke-width="1.5" stroke-linecap="round"/></svg>
        <br><span id="prsf">%PRES%</span> hPa</div>
    <div class="AirQ">
        <svg width="5rem" height="10rem" viewBox="0 0 32 32" fill="white"><path d="M8 5a6 6 0 1 0 0 12h7.023c.913 1.208 2.347 2 3.977 2s3.065-.792 3.977-2H25.5a4.5 4.5 0 0 0 0-9c-.904 0-1.742.271-2.447.73A5.999 5.999 0 0 0 17.5 5c-1.936 0-3.653.92-4.75 2.344A5.984 5.984 0 0 0 8 5zM5 21v2h25v-2H5zm-3 4v2h7v-2H2zm9 0v2h16v-2H11z"/></svg>
        <br><span id="polf">%POLU%</span> ppb</div>
    <div class="Cr"><br><h3>Weather awareness project by Naervern,<br>Antonio Maximiano Mascarenhas Almeida</h3></div>
    <div class="His" id="hi_S">
    </div>
  </div>

<script>
if (!!window.EventSource) {
 var source = new EventSource('/events');
 
 source.addEventListener('open', function(e) {
  console.log("Events Connected");
 }, false);
 source.addEventListener('error', function(e) {
  if (e.target.readyState != EventSource.OPEN) {
    console.log("Events Disconnected");}
 }, false);
 
 source.addEventListener('message', function(e) {
  console.log("message", e.data);
 }, false);
 
 source.addEventListener('temperature', function(e) {
  document.getElementById("temf").innerHTML = e.data;
 }, false);
 
 source.addEventListener('humidity', function(e) {
  document.getElementById("humf").innerHTML = e.data;
 }, false);

  source.addEventListener('pressure', function(e) {
  document.getElementById("prsf").innerHTML = e.data;
 }, false);

  source.addEventListener('pollution', function(e) {
  document.getElementById("polf").innerHTML = e.data;
 }, false);
};

const wd = %WDAY%;
const wk = ["Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"];
const prs = [%PARMS%];

let hCont = "<div style=\"display: grid; grid-template-columns: 50%% 50%%; align-items: center;\"><div><h2>History</h2></div><div><a href=\"/history.html\">Download all data</a></div></div>";
for (let i = 0; i < 7; i++) {
 let j = 0;
 if(wd-i < 0){j = 7;};
 hCont += "<div class=\"boxed\">";
 if(i>0){hCont+="<div>"+wk[wd-i+j]+"</div>"};
 if(i===0){hCont+= "<div>Today<br>so far...</div>"};
 hCont += "<div><h2>&#127777;</h2></div><div class=\"half\"> &#9652;"+prs[wd-i+j]+"&deg;C<br>&#9662;"+prs[wd-i+j+7]+"&deg;C</div><div><h2>&#128167;</h2></div><div>&#9652;"+prs[wd-i+j+14]+"&#37;<br>&#9662;"+prs[wd-i+j+21]+"&#37; </div><div>"+prs[wd-i+j+28]+" hPa</div><div>"+prs[wd-i+j+35]+"ppb</div></div>";
if (i<6){hCont+= "<hr>";};
}

document.getElementById("hi_S").innerHTML = hCont;

</script>
</body></html>

)rawliteral";

const char settime_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
    <style>
        html{text-align: center;}
        button {
        font-size: 2rem;
        border-radius: 1vw;
        padding: 2vw;
        }
    </style>
<h1>Current millis</h1>
<h2 id="date"></h2><br>
<button onclick="sendTime()">Sync</button>
<script>
document.getElementById("date").innerHTML = new Date().getTime();
function sendTime(){
    var t = new Date().getTime();
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/time?set_millis="+ t);
    document.getElementById("date").innerHTML = t;
    xhr.send();
};
</script>
</html>
)rawliteral";

/*
void sendHistory(){
  server client = server.available();
  response->print("<!DOCTYPE html><html>");
  for(int i=0; i<TOTALENTRIES; i++){
    response->print(String(regtime[i])+";"+String(regtemp[i], 2)+";"+String(reghum[i], 2)+";"+String(regpres[i], 2)+";"+String(regpol[i])+"\n");
  }
  response->print("</body></html>");
  response->print();
}
*/

class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {

  server.on("/time", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", settime_html, processor);
    if (request->hasParam("set_millis=")) {
      acquiredTime = request->getParam("set_millis=")->value().toInt();
      update_time();
      Serial.printf("Millis received: %lu \n", acquiredTime);
    }
    request->send_P(200, "text/html", settime_html);
  });

  server.on("/history", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", "<html>History goes here</html>", processor);
  });

  }
  virtual ~CaptiveRequestHandler() {}


  bool canHandle(AsyncWebServerRequest *request) {
    //request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, processor);
    if (request->hasParam("set_millis=")) {
      acquiredTime = request->getParam("set_millis=")->value().toInt();
      update_time();
      Serial.printf("Millis received: %lu \n", acquiredTime);
    };
  };
};

inline void disableWiFi() __attribute__((always_inline));
inline void disableWiFi(){
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
}
inline void mode_normal() __attribute__((always_inline));
void mode_normal(){
  ens160.setOperatingMode(0x00);
  schedule_time();
  ens160.setOperatingMode(0x02);
  dnsServer.processNextRequest();
  //events.send(String(pressure).c_str(),"pressure",millis());
}

void setupServer() {
  /*
  timeserver.on("/time.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", settime_html, processor);
    if (request->hasParam("set_millis=")) {
      acquiredTime = request->getParam("set_millis=")->value().toInt();
      update_time();
      Serial.printf("Millis received: %lu \n", acquiredTime);
    }
    request->send_P(200, "text/html", settime_html);
  });
  historyserver.on("/history.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", "<html>History goes here</html>", processor);
  });
  */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", index_html, processor);
      if (request->hasParam("set_millis=")) {
      acquiredTime = request->getParam("set_millis=")->value().toInt();
      update_time();
      Serial.printf("Millis received: %lu \n", acquiredTime);
      }
  });
}

void setup() {
  setCpuFrequencyMhz(240);
  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(115200);
  Wire.begin();
  while (!Serial) {}; // wait for serial port to connect. Needed for native USB port only, easier debugging :P
  regtemp = (float *) ps_malloc (TOTALENTRIES * sizeof (float));
  reghum = (float *) ps_malloc (TOTALENTRIES * sizeof (float));
  regpres = (float *) ps_malloc (TOTALENTRIES * sizeof (float));
  regpol = (uint16_t *) ps_malloc (TOTALENTRIES * sizeof (unsigned int));
  regtime = (unsigned long *) ps_malloc (TOTALENTRIES * sizeof (unsigned long));
  if(psramInit()){
    Serial.println("\nPSRAM is correctly initialized");
  }else{
    Serial.println("PSRAM not available");
  }

  Serial.println("Beginning EEPROM discovery");
  get_stored_data_length(); //discovers how many rows were saved in the EEPROM already, then continues from the first empty space.
  Serial.printf("A total of %u entries have been discovered in the EEPROM\n", step);
  Serial.println();

  setCpuFrequencyMhz(80);

  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1); // Timezone set to Helsinki
  tzset();
  bmp280.begin();
  if (aht20.begin() == false){Serial.println("AHT20 not detected. Please check wiring.");} else {Serial.println("AHT20 acknowledged.");}
  if( !ens160.begin() ){Serial.println("Could not communicate with the ENS160, check wiring.");}
  ens160.setOperatingMode(SFE_ENS160_RESET);
  ens160.setOperatingMode(SFE_ENS160_STANDARD);
	

  update_params();


  WiFi.mode(WIFI_AP);
  WiFi.softAP("WeatherSpot");
  Serial.println("Starting DNS Server");
  dnsServer.start(53, "*", WiFi.softAPIP());



  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      //Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 1000);
  });

  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);  //only when requested from AP
  server.addHandler(&events);

  setupServer();

  server.begin();
  //timeserver.begin();
  //historyserver.begin();
}


void loop() {
  Serial.println();
  dnsServer.processNextRequest();

  //void mode_normal();

  if ((millis() - lastTime) > TIMERDELAY * 1000) {
    ens160.setOperatingMode(0x02);
    update_params();
    ens160.setOperatingMode(0x02);
    store_week_data(temperature, humidity, pressure, pollution);
    Serial.println("...");

    //Send Events to the Web Client with the Sensor Readings
    //events.send("ping",NULL,millis());
    //events.send(String(temperature).c_str(),"temperature",millis());
    //events.send(String(humidity).c_str(),"humidity",millis());
    //Serial.println("events sent:");
    //Serial.println();
    
    lastTime = millis();
  }
  
}