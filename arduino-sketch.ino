#include <DNSServer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AHT20.h>
#include <time.h>
#include "AsyncUDP.h"

//debug libraries
#include <Wire.h>

DNSServer dnsServer;
AsyncWebServer server(80);
AsyncEventSource events("/events");

unsigned long lastTime = 0;  
unsigned long timerDelay = 30000;

bool measurement_trigger = false;
bool wifi_on = false;

AHT20 aht20;
float temperature;
float humidity;

void update_params(){
  //add any sensor measurement here
  temperature = aht20.getTemperature();
  humidity = aht20.getHumidity();
  Serial.println("New measurement");
  Serial.printf("Temperature = %.2f ℃ \n", temperature);
  Serial.printf("Humidity = %.2f % \n", humidity);
  Serial.println();
}

String processor(const String& var){ //Stuff from Rui Santos
  
  if(var == "TEM"){
    return String(temperature);
  }
  else if(var == "HUM"){
    return String(humidity);
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
h2 {font-size: 2.5rem;}
h3 {font-size: 1.6rem;}
p {font-size: 2rem;}

html, body, .background{
	width: 100vw;
	height: 100vh;
	margin: 0;
	padding: 0;
	font-family: 'Roboto', 'Palatino', 'Lucida Sans Unicode', sans-serif;
    color: #fff;
}

html{
	background: #12022a;
}

body{
	background: #eee;
	background: linear-gradient(0deg, rgba(16, 58, 226, 0.2) , rgba(181, 222, 0, 0) );
}

.container
{
    align-items: center;
  	justify-content: center;
    text-align: center;
    margin: auto;
	  padding: 1rem;
	  height: min(95vh, 95vw);
    width: 95vw;
	  overflow: auto;
    font-size: 2.5rem;

    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
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

.Tem { grid-area: Te; }
.His { grid-area: Hi; }
.Hum { grid-area: Hu; }
.Ti { grid-area: Ti;}
.AtmP { grid-area: At;}
.AirQ { grid-area: Ai;}
.Cr { grid-area: Cr;}

</style>
</head>
<body>
<div class="container">
    <div class="His"><p></p></div>
    <div class="Ti"><h1>Sunny Breeze<br><h2>weather monitor</h2></h1></div>
    <div class="Hum">
        <svg width="5rem" height="10rem" viewBox="0 0 32 32" fill="white"><path d="M23.476 13.993 16.847 3.437a1.04 1.04 0 0 0-1.694 0L8.494 14.043A9.986 9.986 0 0 0 7 19a9 9 0 0 0 18 0 10.063 10.063 0 0 0-1.524-5.007ZM16 26a7.009 7.009 0 0 1-7-7 7.978 7.978 0 0 1 1.218-3.943l.935-1.49 10.074 10.074A6.977 6.977 0 0 1 16 26.001Z" stroke="white"/><path style="fill:none" d="M0 0h32v32H0z"/></svg>
        <br><span id="HumF">%HUM%</span> &percnt;</div>
    <div class="Tem">
        <svg width="5rem" height="10rem" viewBox="0 0 56 56" fill="white"><path d="M25.012 54.04c6.75 0 12.234-5.485 12.234-12.235 0-3.563-1.476-6.68-4.266-9.211-.515-.469-.632-.727-.632-1.43l.047-21.047c0-4.898-2.977-8.156-7.383-8.156-4.43 0-7.407 3.258-7.407 8.156l.024 21.047c0 .703-.117.961-.61 1.43-2.812 2.531-4.265 5.648-4.265 9.21 0 6.75 5.46 12.235 12.258 12.235Zm0-3.4c-4.875 0-8.836-3.984-8.836-8.835 0-2.93 1.383-5.578 3.867-7.242.727-.493 1.008-.938 1.008-1.899V10.258c0-2.953 1.617-4.852 3.96-4.852 2.321 0 3.915 1.899 3.915 4.852v22.406c0 .961.281 1.406 1.008 1.898 2.484 1.665 3.867 4.313 3.867 7.243 0 4.851-3.938 8.836-8.79 8.836Zm11.742-40.078h5.11a1.35 1.35 0 0 0 1.382-1.359 1.35 1.35 0 0 0-1.383-1.36h-5.11a1.35 1.35 0 0 0-1.382 1.36c0 .727.563 1.36 1.383 1.36Zm0 6.586h5.11a1.35 1.35 0 0 0 1.382-1.359 1.35 1.35 0 0 0-1.383-1.36h-5.11a1.35 1.35 0 0 0-1.382 1.36c0 .727.563 1.36 1.383 1.36ZM24.988 47.477a5.667 5.667 0 0 0 5.672-5.696c0-2.203-1.242-4.008-3.047-4.992-.75-.398-1.008-.68-1.008-1.828V22.516c0-1.22-.703-1.946-1.617-1.946-.89 0-1.617.727-1.617 1.946V34.96c0 1.148-.258 1.43-1.008 1.828-1.804.984-3.047 2.79-3.047 4.992a5.667 5.667 0 0 0 5.672 5.696Zm11.766-23.743h5.11a1.35 1.35 0 0 0 1.382-1.359 1.37 1.37 0 0 0-1.383-1.383h-5.11a1.37 1.37 0 0 0-1.382 1.383c0 .727.563 1.36 1.383 1.36Zm0 6.586h5.11a1.37 1.37 0 0 0 1.382-1.383 1.35 1.35 0 0 0-1.383-1.359h-5.11a1.35 1.35 0 0 0-1.382 1.36 1.37 1.37 0 0 0 1.383 1.382Z" stroke="white"/></svg>
        <br><span id="TemF">%TEM%</span>  &deg;C</div>
    <div class="AtmP">
        <svg width="5rem" height="10rem" viewBox="0 0 24 24" fill="none"><path d="M20.693 17.33a9 9 0 1 0-17.386 0" stroke="white" stroke-width="1.5" stroke-linecap="round"/><path d="M12.766 15.582c.487.71.144 1.792-.766 2.417-.91.626-2.043.558-2.53-.151-.52-.756-2.314-5.007-3.403-7.637-.205-.495.4-.911.79-.542 2.064 1.96 5.39 5.157 5.909 5.913Z" stroke="white" stroke-width="1.5"/><path d="M12 6v2m-6.364.636L7.05 10.05m11.314-1.414L16.95 10.05m3.743 7.28-1.931-.518m-15.455.518 1.931-.518" stroke="white" stroke-width="1.5" stroke-linecap="round"/></svg>
        <br><span id="pressure">%PRES%</span> hPa</div>
    <div class="AirQ">
        <svg width="5rem" height="10rem" viewBox="0 0 32 32" fill="white"><path d="M8 5a6 6 0 1 0 0 12h7.023c.913 1.208 2.347 2 3.977 2s3.065-.792 3.977-2H25.5a4.5 4.5 0 0 0 0-9c-.904 0-1.742.271-2.447.73A5.999 5.999 0 0 0 17.5 5c-1.936 0-3.653.92-4.75 2.344A5.984 5.984 0 0 0 8 5zM5 21v2h25v-2H5zm-3 4v2h7v-2H2zm9 0v2h16v-2H11z"/></svg>
        <br><span id="pressure">%AIRQ%</span> ppb</div>
    <div class="Cr"><br><h3>Weather awareness project by Naervern,<br>Antonio Maximiano Mascarenhas Almeida</h3></div>
  </div>

</body></html>

)rawliteral";


class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}


  bool canHandle(AsyncWebServerRequest *request) {
    //request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request) {

    //float temperature = aht20.getTemperature();
    //float humidity = aht20.getHumidity();

    request->send_P(200, "text/html", index_html, processor);

    events.send("ping",NULL,millis());
    events.send(String(temperature).c_str(),"temperature",millis());
    events.send(String(humidity).c_str(),"humidity",millis());
  }
};

void disableWiFi(){
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off

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

void mode_normal(){
  schedule_time();

  dnsServer.processNextRequest();
  //events.send(String(pressure).c_str(),"pressure",millis());
}



void setupServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, processor);
    Serial.println("Client Connected");
  });
}

void setup() {
  setCpuFrequencyMhz(80);

  Serial.begin(115200);

  //while (!Serial) {}; // wait for serial port to connect. Needed for native USB port only, easier debugging :P
  Serial.println();

  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1); // Timezone set to Helsinki
  tzset();
  
  Wire.begin(); //Join I2C bus
  //Check if the AHT20 will acknowledge
  if (aht20.begin() == false){Serial.println("AHT20 not detected. Please check wiring. Freezing.");while (1);}
  Serial.println("AHT20 acknowledged.");

  WiFi.mode(WIFI_AP);
  WiFi.softAP("WeatherSpot");
  Serial.println("Starting DNS Server");
  dnsServer.start(53, "*", WiFi.softAPIP());


  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });

  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);  //only when requested from AP
  server.addHandler(&events);
  
  setupServer();

  server.begin();
}


void loop() {

  dnsServer.processNextRequest();
  if ((millis() - lastTime) > timerDelay) {
    update_params();
    Serial.println("...");

    Send Events to the Web Client with the Sensor Readings
    events.send("ping",NULL,millis());
    events.send(String(temperature).c_str(),"temperature",millis());
    events.send(String(humidity).c_str(),"humidity",millis());
    //Serial.println("events sent:");
    //Serial.println();
    
    lastTime = millis();
  }

}
