#include <DNSServer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <time.h>
#include "AsyncUDP.h"
#include <SPI.h>
#include <AHT20.h>
#include "SparkFun_ENS160.h"

#include <Wire.h>

#define TOTALENTRIES 57600

#define AHT21_ADDRESS 0x38
#define ENS160_ADDRESS 0x53
#define BMP280_ADDRESS 0x76

DNSServer dnsServer;
AsyncWebServer server(80);
//AsyncWebServer timeserver(80);
//AsyncWebServer historyserver(80);
AsyncEventSource events("/events");

unsigned long lastTime = 0;   
unsigned long timerDelay = 30000;

float * regtemp;
float * reghum;
float * regpres;
uint16_t * regpol;
unsigned long * regtime;
//ps_malloc();
uint16_t step = 0; //iterator for the regtab array. It keeps track of what's the next measurement to be stored.

bool measurement_trigger = false;
bool midnight_trigger = false;
bool wifi_on = false;

AHT20 aht20;

SparkFun_ENS160 ENS160;

float temperature;
float humidity;
float pressure;
uint16_t pollution;



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

RTC_FAST_ATTR float histtemperaturemax[7] = {0.0};
RTC_FAST_ATTR float histtemperaturemin[7] = {0.0};
RTC_FAST_ATTR float histhumiditymax[7] = {404.0, 404.1, 404.2, 404.3, 404.4, 404.5, 404.6};
RTC_FAST_ATTR float histhumiditymin[7] = {404.0, 404.1, 404.2, 404.3, 404.4, 404.5, 404.6};
RTC_FAST_ATTR float histpressure[7] = {1404.0, 1404.1, 1404.2, 1404.3, 1404.4, 1404.5, 1404.6};
RTC_FAST_ATTR uint16_t histpollution[7] = {0};


RTC_SLOW_ATTR float * d_temp;
RTC_SLOW_ATTR float * d_hum;
RTC_SLOW_ATTR float * d_pres;
RTC_SLOW_ATTR uint16_t * d_pol;
RTC_SLOW_ATTR unsigned long * d_time;
RTC_SLOW_ATTR uint8_t d_10m_step = 0;

RTC_SLOW_ATTR uint8_t week_it = 0;

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
  d_temp = (float *) malloc (144 * sizeof (float));
  d_hum = (float *) malloc (144 * sizeof (float));
  d_pres = (float *) malloc (144 * sizeof (float));
  d_pol = (uint16_t *) malloc (144 * sizeof (uint16_t));
  d_time = (unsigned long *) malloc (144 * sizeof (unsigned long));
}
















// END OF NEW LOW POWER FUNCTIONS



//EEPROM FUNCTIONS
#include <EEPROM.h>
#define EEPROM_SIZE 3145728 //size of the flash memory to be reserved. 3145728 = 3MB

void store_measurement(uint16_t step){
  byte byteARR[18] = {0xFF};
  for (uint8_t i=0; i<144; i++){
    EEPROM.put(step+i-16, d_time[i]); //reserving the first 128 bytes of EEPROM for reasons
    EEPROM.put(step+i-12, d_temp[i]);
    EEPROM.put(step+i-8, d_hum[i]);
    EEPROM.put(step+i-4, d_pres[i]);
    EEPROM.put(step+i, d_pol[i]);
    /*
    byteARR[18] = {
      d_temp[i] >> 24;
      d_temp[i] >> 16;
      d_temp[i] >> 8;
      d_temp[i] & 0xFF;
      d_hum[i] >> 24;
      d_hum[i] >> 16;
      d_hum[i] >> 8;
      d_hum[i] & 0xFF;
      d_pres[i] >> 24;
      d_pres[i] >> 16;
      d_pres[i] >> 8;
      d_pres[i] & 0xFF;
      d_pol[i] >> 8;
      d_pol[i] & 0xFF;
      d_time[i] >> 24;
      d_time[i] >> 16;
      d_time[i] >> 8;
      d_time[i] & 0xFF;
     };
    for (uint8_t j=0; j<18; j++){
      EEPROM.write(18*step+18*i+j, byteARR[j]);
      };
    */
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
    EEPROM.get(pas+128, tim); //reserving the first 128 bytes of EEPROM for reasons
    EEPROM.get(pas+132, temp);
    EEPROM.get(pas+136, hum);
    EEPROM.get(pas+140, pres);
    EEPROM.get(pas+142, pol);
  }
};

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
  if(step > 53999){step = 0;};
}

void update_params(){
  /**
   * Set power mode
   * mode Configurable power mode:
   *   ENS160_SLEEP_MODE: DEEP SLEEP mode (low power standby)
   *   ENS160_IDLE_MODE: IDLE mode (low-power)
   *   ENS160_STANDARD_MODE: STANDARD Gas Sensing Modes
   */
  //ENS160.setPWRMode(ENS160_STANDARD_MODE);
  //add any sensor measurement here
  //digitalWrite(ENS_CS, LOW);
  temperature = aht20.getTemperature();
  humidity = aht20.getHumidity();
  //ENS160.setTempAndHum(temperature, humidity);
  pollution = ENS160.getTVOC();
  //ENS160.setPWRMode(ENS160_SLEEP_MODE);
  pressure = bmp280.readPressure();
  Serial.println("New measurement");
  Serial.printf("Temperature = %.2f °C", temperature);
    Serial.println();
  Serial.printf("Humidity = %.2f %", humidity);
    Serial.println();
  Serial.printf("Pressure = %.2f %", pressure);
    Serial.println();
  Serial.printf("TVOC = %u %", (unsigned int)humidity);
  Serial.println();
  save_entry(temperature, humidity, pressure, pollution);



}

void schedule_time(){
  //Serial.printf("time(null) value: %d \n", time(NULL));//debug stuff
  //Serial.println(measurement_trigger == true); //debug stuff

  if((time(NULL) % 30) == 0 && measurement_trigger == true) {
    update_params();
    measurement_trigger = false;
    Serial.println("Attempting to send events"); //debug stuff
    events.send("ping",NULL,millis());
    events.send(String(temperature).c_str(),"temperature",millis());
    events.send(String(humidity).c_str(),"humidity",millis());
  };
  if ((time(NULL) % 30) > 15 && (time(NULL) % 30) < 20 && measurement_trigger == false)
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
  schedule_time();

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
  setCpuFrequencyMhz(80);
  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(115200);
  //pinMode(BMP_CS, OUTPUT);
  //pinMode(ENS_CS, OUTPUT);
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

  Serial.println();

  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1); // Timezone set to Helsinki
  tzset();
      Serial.println("Trying to set Wire and I2C modules");
  Wire.begin(); //Join I2C bus
  //Check if the AHT20 will acknowledge
  if (aht20.begin() == false){Serial.println("AHT20 not detected. Please check wiring.");} else {Serial.println("AHT20 acknowledged.");}
  
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


  //Setup for the BMP280 sensor
    unsigned status;
  
  //status = bmp280.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  /* When the BMP280 is around... */
  status = bmp280.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp280.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    //while (1) delay(10);
  }
  
  /* Default settings from datasheet. */
  //bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  //                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  //                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  //                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  //                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  
  setupServer();

  server.begin();
  //timeserver.begin();
  //historyserver.begin();
}


void loop() {

  dnsServer.processNextRequest();
  if ((millis() - lastTime) > timerDelay) {
    update_params();
    Serial.println("...");

    //Send Events to the Web Client with the Sensor Readings
    events.send("ping",NULL,millis());
    events.send(String(temperature).c_str(),"temperature",millis());
    events.send(String(humidity).c_str(),"humidity",millis());
    //Serial.println("events sent:");
    //Serial.println();
    
    lastTime = millis();
  }

}
