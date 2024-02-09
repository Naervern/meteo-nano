# Sunny Breeze
A tiny solar-powered esp32-s2 weather station with wifi capture portal access, capable of logging and displaying instantaneous measurements of temperature, relative humidity, atmospheric pressure and total volatile organic compounds (TVOC) updated once every 10 minutes, plus a daily registry for the current and past 6 days in maximum and minimum temperature and humidity, minimum pressure and maximum TVOC reading. It can also provide the raw data of all measurements since being powered up (maximum number of entries is configurable, stored in the EEPROM with a pointer for balancing usage).


Current dependencies:
- EspAsyncWebServer.h
- DNSServer.h
- WiFi.h
- AsyncTCP.h
- ESP32Time.h
- Wire.h
- AHT20.h
- SparkFun_ENS160.h

Features:
- Solar charging, supercapacitor operation
- Power saving operation under low light environment
- Temperature, relative humidity, atmospheric pressure and TVOC sensors
- One measurement every 10 minutes
- Database of entries stored can be configured so long as it does not exceed the EEPROM size
- Registry of 6 parameters (max, min temperature; max, min humidity; minimum pressure; peak TVOC) for the current and the 6 prior days
- Optionally, eCO2 can be logged as well, but since this parameter has a very low localised deviation, it was opted to leave out for this project.

The module clock can be adjusted by accessing the /time address then clicking the "Sync" button, sending the client device milliseconds since epoch by HTTP.
