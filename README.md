# Sunny Breeze
A tiny solar-powered esp32-s2 weather station with wifi capture portal access, capable of logging and displaying instantaneous measurements of temperature, relative humidity, atmospheric pressure and total volatile organic compounds (TVOC) updated once every 10 minutes, plus a daily registry for the current and past 6 days in maximum and minimum temperature and humidity, minimum pressure and maximum TVOC reading. It can also provide the raw data of all measurements since being powered up (maximum of 52600 entries).


Current dependencies:
- EspAsyncWebServer.h
- DNSServer.h
- WiFi.h
- AsyncTCP.h
- Time.h
- Wire.h
- SPI.h
- AHT20.h
- SparkFun_ENS160.h
- Adafruit_BMP280.h

Features:
- Solar charging, supercapacitor operation
- Power saving operation under low light environment
- Temperature, relative humidity, atmospheric pressure and TVOC sensors
- One measurement every 10 minutes
- Database of up to 52600 entries stored
- Registry of 6 parameters (max, min temperature; max, min humidity; average pressure; peak TVOC) for the current and the 6 prior days
