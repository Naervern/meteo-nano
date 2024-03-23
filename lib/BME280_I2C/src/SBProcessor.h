String SBprocessor(const String& var){
  
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

        while(file.available()) {
            readingBuffer = file.read();
        }

        file.close();

        for(uint16_t i=0; i<DAILYENTRIES; i++){
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
        snprintf(row, 64, "\nTimestamp = %" )
        strcpy((char*)buffer, row);
        Serial.println(row);
    }

      //String row = String(regtime[i])+";"+String(regtemp[i], 2)+";"+String(reghum[i], 2)+";"+String(regpres[i], 2)+";"+String(regpol[i])+"\n";


      rows_sent++;
      currentIndexForChunk++;
      return strlen((char*) buffer);
    //return mySource.read(buffer, maxLen);

    });
    response->addHeader("Server","SunnyBreeze History");
    request->send(response);

}
