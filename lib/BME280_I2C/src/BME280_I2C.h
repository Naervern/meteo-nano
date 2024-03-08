/*
  BME280I2C.h library for both BME280 and BMP280 sensors in I2C mode.

    Copyright (c) 2024, Antonio Maximiano - <naervern@protonmail.com>
    
    I did some work so this library could be used more easily, but the
    bulk of information was originally found across helpful replies in
     the Arduino Forum (specifically thread 553030) and Bosch's datasheet.


    This library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#ifndef BME280_I2C_h
#define BME280_I2C_h
#include <Wire.h>
#include <Arduino.h>

class BME280_I2C
{
    public:


        BME280_I2C(const uint8_t& x) : ADDRESS(x){};
        ~BME280_I2C();

        const uint8_t ADDRESS = 0x76; //default address is 0x76

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

        void begin();               // Initialises the sensor in the I2C bus

        float getTemperature();     // returns temperature in ºC

        float getPressure();        // returns pressure in hPa

        float getHumidity();        // returns relative atmospheric humidity in %
        
        void readData();            // reads raw data from the sensor

        void read();                // calls readData() and then parses it to update the parameters of temperature, pressure and humidity stored in the sensor

    private:
        uint8_t data[32];

        uint8_t ctrl_meas_reg;
        uint8_t config_reg;
        uint8_t ctrl_hum_reg;

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

        void writeReg(uint8_t reg_address, uint8_t data);   // Communicates via the I2C bus;

        signed long int calibration_T(signed long int adc_T);   // Converts the byte array into an arithmetic-compatible data type;

        unsigned long int calibration_P(signed long int adc_P); // Converts the byte array into an arithmetic-compatible data type;

        unsigned long int calibration_H(signed long int adc_H); // Converts the byte array into an arithmetic-compatible data type;
};

#endif