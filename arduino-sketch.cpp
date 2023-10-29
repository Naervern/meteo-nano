#include <DNSServer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebSrv.h"
#include <AHT20.h>
#include <time.h>
//debug libraries - remove for optimised version
#include <Wire.h>

DNSServer dnsServer;
AsyncWebServer server(80);

float temperature;
float humidity;
bool show_history;
AHT20 aht20;

// Time to sleep
uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds
// sleep for 10 minutes = 600 seconds
uint64_t TIME_TO_SLEEP = 600;

//Change the HTML
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

html, body, .background
{
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

body
{
	background: #eee;
	background: linear-gradient(0deg, rgba(16, 58, 226, 0.2) , rgba(181, 222, 0, 0) );
}


.container
{
    align-items: center;
  	justify-content: center;
    margin: auto;
	padding: 1rem;
	height: 95%;
    width: 95%;
	overflow: auto;

    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: 2fr 2fr 1fr;
    gap: 0.2rem 0.2rem;
    grid-auto-flow: row;
    grid-template-areas:
    "Ti Ti His"
    "Tem Hum His"
    "Cr Cr His";
	
	background: #eee;
	background: linear-gradient(240deg, rgba(165, 51, 51, 0.3) , rgba(240, 205, 7, 0) );
}

.Tem { grid-area: Tem; }

.His { grid-area: His; }

.Hum { grid-area: Hum; }

.Ti { grid-area: Ti;}

.Cr { grid-area: Cr; }

.Tem, .Hum, .Ti {margin: auto; text-align: center;}


    </style>
</head>
<body>
<div class="container">
    <div class="Tem"><p>Lämpötila</p><h2>%f ℃</h2></div>
    <div class="His"><p></p></div>
    <div class="Hum"><p>Kosteus</p><h2>%f \%</h2></div>
    <div class="Ti"><h1>Sunny Breeze<br><h2>weather monitor</h2></h1></div>
    <div class="Cr"><h3>2023 - Naervern<br>Antonio Maximiano Mascarenhas Almeida</h3></div>
</div>
</body>
</html>

)rawliteral";


class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}


  bool canHandle(AsyncWebServerRequest *request){
    //request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request) {

    float temperature = aht20.getTemperature();
    float humidity = aht20.getHumidity();

    request->send_P(200, "text/html", index_html);
  }
};


class LogEntry {
  public:
    int index;
    float temperature;
    float humidity;
};


void setupServer(){
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", index_html);
      Serial.println("Client Connected");
  });
   
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
      String inputMessage;
      String inputParam;
 
      request->send(200, "text/html", "The values entered by you have been successfully sent to the device <br><a href=\"/\">Return to Home Page</a>");
  });
}


void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
<<<<<<< Updated upstream
  //your other setup stuff...
=======
>>>>>>> Stashed changes
  Serial.begin(115200);
  while (!Serial) {};
  // wait for serial port to connect. Needed for native USB port only
  Serial.println();
  Serial.println("Setting up AP Mode");
  WiFi.mode(WIFI_AP);
  WiFi.softAP("WeatherSpot");
  setupServer();


  // Timezone set to Helsinki
  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);
  tzset();

  Serial.println("Starting DNS Server");
  dnsServer.start(53, "*", WiFi.softAPIP());
<<<<<<< Updated upstream
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP
  //more handlers...
=======
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);
>>>>>>> Stashed changes
  server.begin();
  Serial.println("All Done!");
}


void loop(){
  dnsServer.processNextRequest();
}



 