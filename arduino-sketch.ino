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
  //Serial.println();
  Serial.printf("Humidity = %.2f % \n", humidity);
  //Serial.println();
}

String processor(const String& var){ //Stuff from Rui Santos
  //Serial.println(var);
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
	width: 100%;
	height: 100%;
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
    margin: auto;
	  padding: 1rem;
	  height: min(95vh, 95vw);
    width: 95%;
	  overflow: auto;

    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: 1fr 2fr 2fr;
    gap: 0.2rem 0.2rem;
    grid-auto-flow: row;
    grid-template-areas:
    "Ti Ti His"
    "Tem Hum His"
    "AtmP AirQ His";
	
	background: #eee;
	background: linear-gradient(240deg, rgba(165, 51, 51, 0.3) , rgba(240, 205, 7, 0) );
}

.Tem { grid-area: Tem; }
.His { grid-area: His; }
.Hum { grid-area: Hum; }
.Ti { grid-area: Ti;}
.AtmP { grid-area: AtmP;}
.AirQ { grid-area: AirQ;}

.Tem, .Hum, .Ti, .AtmP, .AirQ {margin: auto; text-align: center;}

</style>
</head>
<body>
<div class="container">
    <div class="His"><p></p></div>
    <div class="Ti"><h1>Sunny Breeze<br><h2>weather monitor</h2></h1></div>
    <div class="Hum"><p>Kosteus</p><h2><span id="humf">%HUM%</span> %</h2></div>
    <div class="Tem"><p>Lämpötila</p><h2><span id="temf">%TEM%</span> ℃</h2></div>
    <div class="AtmP"><p>Paine</p><p><span id="pressure">--</span> hPa</p></div>
    <div class="AirQ"><p>Saaste</p><p><span id="pressure">--</span> ppb</p></div>
  </div>
  <div class="Cr"><h3>Weather awareness project by Naervern,<br>Antonio Maximiano Mascarenhas Almeida</h3></div>
<script>
if (!!window.EventSource) {
  var source = new EventSource('/events');
  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);
  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);

  source.addEventListener('message', function(e) {
    console.log("message", e.data);
  }, false);

  source.addEventListener('temperature', function(e) {
    console.log("temperature", e.data);
    document.getElementById("temf").innerHTML = e.data;
  }, false);

  source.addEventListener('humidity', function(e) {
    console.log("humidity", e.data);
    document.getElementById("humf").innerHTML = e.data;
  }, false);
}
</script>
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

    request->send_P(200, "text/html", index_html);

    events.send("ping",NULL,millis());
    events.send(String(temperature).c_str(),"temperature",millis());
    events.send(String(humidity).c_str(),"humidity",millis());
  }
};

class LogEntry {
public:
  int index;
  float temperature;
  float humidity;
};

void streamUDP (){

}

void disableWiFi(){
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off

}

void mode_sleep(){
  esp_sleep_enable_timer_wakeup(59990000);
  esp_deep_sleep_start();
  esp_light_sleep_start();

}

void mode_eco(){
  
}

void mode_wake(int wake_time){

}

void mode_normal(){
  schedule_time();

  dnsServer.processNextRequest();
  //events.send(String(pressure).c_str(),"pressure",millis());
}

void schedule_time(){
  Serial.printf("time(null) value: %d \n", time(NULL));//debug stuff
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

void setupServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, processor);
    Serial.println("Client Connected");
  });
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  setCpuFrequencyMhz(80);

  Serial.begin(115200);

  while (!Serial) {}; // wait for serial port to connect. Needed for native USB port only, easier debugging :P
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
  setupServer();

  update_params();

  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);  //only when requested from AP

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 1000);
  });

  server.addHandler(&events);

  server.begin();
}


void loop() {
  
  mode_normal();
  //digitalWrite(LED_BUILTIN, HIGH);//debug stuff
  //Serial.printf("The measurement trigger is %s", measurement_trigger ? "true" : "false"); //debug stuff
  //Serial.println(measurement_trigger);
  //esp_sleep_enable_timer_wakeup(500000);
  //ret = esp_light_sleep_start();
  //Serial.printf("light_sleep: %d\n", ret);
  delay(200);
  //digitalWrite(LED_BUILTIN, LOW);
  delay(800);
  //esp_sleep_enable_timer_wakeup(500000);
  //ret = esp_light_sleep_start();
  //Serial.printf("light_sleep: %d\n", ret);

}
