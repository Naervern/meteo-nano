// based on J-M-L's code from the Arduino forum

#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

const char *ssid = "***";
const char *password = "***";

uint32_t  logNumber = 0;
const byte numberOfLogs = 200;
const byte numberOfCharsAtOneRow = 100;

struct t_log {
  uint32_t number;
  char     message[numberOfCharsAtOneRow];
};

static t_log logs[numberOfLogs];
const char * htmlHeader = "<HTML><BODY>\r\n<table><tr><th>INDEX</th><th>DATA</th></tr>\r\n";
const char * htmlFooter = "</table></BODY></HTML>\r\n";

void logThis(String& str) {
  logNumber++;
  for (int x = numberOfLogs - 1; x > 0; x--) logs[x] = logs[x - 1];
  logs[0].number = logNumber;
  str.toCharArray(logs[0].message, numberOfCharsAtOneRow);
}

void getLog(AsyncWebServerRequest *request) {
  AsyncWebServerResponse *response = request->beginChunkedResponse("text/html",
  [](uint8_t *buffer, size_t maxlen, size_t index) -> size_t {
    static byte currentIndexForChunk = 0;
    // is it a new request
    if (index == 0) {
      currentIndexForChunk = 0;             // reset the log line index when we get a new request
      strcpy((char*) buffer, htmlHeader);
    } else buffer[0] = '\0';

    if (currentIndexForChunk == numberOfLogs) { // we are done, send the footer
      strcpy((char*) buffer, htmlFooter);
      currentIndexForChunk++;
      return strlen((char*) buffer);
    } else if (currentIndexForChunk > numberOfLogs) { // the footer has been sent, we close this request by sending a length of 0
      // but for the sake of the demo, we add something in the log to make it grow for next refresh
      logThis(String("I've sent the page, millis was ") + String(millis()) + String(" ms"));
      return 0;
    }

    size_t initialBufferLength = strlen((char*) buffer);
    if (logs[currentIndexForChunk].number != 0)
      snprintf((char*) buffer + initialBufferLength, maxlen, "<tr><td>%u</td><td>%s</td></tr>\r\n", logs[currentIndexForChunk].number, logs[currentIndexForChunk].message);
    else {
      // we are done, send the footer (adding to the buffer in case the log was empty then we have the header already in the buffer)
      strncat((char*) buffer + initialBufferLength, htmlFooter,maxlen);
      currentIndexForChunk = numberOfLogs + 1; // pretend we were at the end of the array so next time we return 0 and terminate the request
      return strlen((char*) buffer);
    }

    currentIndexForChunk++;
    return strlen((char*) buffer);
  });

  request->send(response);
}


void setup() {
  Serial.begin(115200);

  String aMessage;
  aMessage = "I'll store here a lot of requests. Just update this page to see more"; logThis(aMessage);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    while (true) yield();
  }

  Serial.print("IP Address: "); Serial.println(WiFi.localIP());
  server.on("/", HTTP_GET, getLog);
  server.begin();
}


void loop() {}