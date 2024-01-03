# Sunny Breeze
A tiny solar-powered esp32-s2 weather station with wifi capture portal access


Current dependencies:
- EspAsyncWebServer.h
- DNSServer.h
- WiFi.h
- AsyncTCP.h
- AHT20.h
- Time.h
- Wire.h
- SPI.h
- SparkFun_ENS160.h

Features:
- Solar charging, supercapacitor operation
- Power saving operation under low light environment
- Temperature, relative humidity, atmospheric pressure and TVOC sensors
- One measurement every 10 minutes
- Database of up to 52600 entries stored
- Registry of 6 parameters (max, min temperature; max, min humidity; average pressure; peak TVOC) for the current and the 6 prior days
