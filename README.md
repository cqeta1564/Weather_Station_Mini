# Basic info
This code is based on [Weather_Station_Mini](https://github.com/LaskaKit/Weather_Station_Mini) code from [@laskakit](https://github.com/LaskaKit). 

## My upgrade
Added support for SHT40 and DS18B20. My code is pushing data to two [tmep.cz](https://tmep.cz/) "sensors". One of them is set up for public use (SHT40 humi, BMP280 temp and press) and the another one is just for some more info about the state of the board (DS18B20 temp, battery voltage and WiFi RSSI).

## How to use the code
Just clone this repository, unzip it, go to folder called "weather_mini" and open "weather_mini.ino". Don't forget to change "config_my.h" to "config.h". Fill your server names and WiFi login credentials.
