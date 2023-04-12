/* LaskaKit DIY Mini Weather Station. 
 * TMEP edition
 * Read Temperature, Humidity and pressure and send to TMEP.cz
 * 
 * For settings see config.h
 * 
 * Email:podpora@laskakit.cz
 * Web:laskakit.cz
 * Board: ESP32-C3 Dev Module
 * 
 */

// připojení knihoven
#include "config.h"            // change to config.h and fill the file.

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>      // BMP280 by Adafruit 
#include "Adafruit_SHT4x.h"       // SHT40
#include <OneWire.h>              // OneWire for DS18B20
#include <DallasTemperature.h>    // DS18B20
#include <ESP32AnalogRead.h>      // ESP32AnalogRead by madhephaestus https://github.com/madhephaestus/ESP32AnalogRead 
#include <WiFiManager.h>          // WiFi manager by tzapu https://github.com/tzapu/WiFiManager

#define BMP280_ADDRESS (0x76)     // (0x77) cut left and solder right pad on board
#define SLEEP_SEC 1*60           // Measurement interval (seconds)
#define ADC_PIN 0                 // ADC pin on LaskaKit Meteo mini
#define deviderRatio 1.7693877551 // Voltage devider ratio on ADC pin 1M + 1.3MOhm
#define ONE_WIRE_BUS_1 10         // Define DS18B20 pin

// Vytvoření instance | Instance creation
Adafruit_BMP280 bmp;
ESP32AnalogRead adc;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
OneWire oneWire_in(ONE_WIRE_BUS_1);
DallasTemperature sensor_inhouse(&oneWire_in);

float temperature;
float DStemperature;
float pressure;
float humidity;
float bat_voltage;

void postData(){

  if(WiFi.status()== WL_CONNECTED) {
    HTTPClient http;
      
    //hodnota teploty venku, pro vlhkost "humV", pro tlak "pressV"
    String serverPath = serverName + "" + "temp=" + temperature + "&humV=" + humidity + "&pressV=" + pressure; 
    //hodnota teploty uvnitr, pro napeti baterie "v"
    String serverPathDEV = serverNameDEV + "" + "temp=" + sensor_inhouse.getTempCByIndex(0) + "&V=" + bat_voltage + "&rssi=" + WiFi.RSSI(); 
      
    // zacatek http spojeni 1
    http.begin(serverPath.c_str());
      
    // http get request
    int httpResponseCode = http.GET();
      
    if (httpResponseCode>0) {
      Serial.print("HTTP odpoved: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println(payload);
    } else {
      Serial.print("Error kod: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();

    // zacatek http spojeni 2
    http.begin(serverPathDEV.c_str());
      
    // http get request
    int httpResponseCodeDEV = http.GET();
      
    if (httpResponseCodeDEV>0) {
      Serial.print("HTTP odpoved: ");
      Serial.println(httpResponseCodeDEV);
      String payloadDEV = http.getString();
      Serial.println(payloadDEV);
    } else {
      Serial.print("Error kod: ");
      Serial.println(httpResponseCodeDEV);
    }
    // Free resources
    http.end();
  } else 
      Serial.println("Wi-Fi odpojeno");

}

void GoToSleep(){
    delay(1);
  // ESP Deep Sleep 
  Serial.println("ESP in sleep mode");
  esp_sleep_enable_timer_wakeup(SLEEP_SEC * 1000000);
  esp_deep_sleep_start();
}

// pripojeni k WiFi | WiFi Connection
void WiFiConnection(){
    // Probudit WiFi | Wake up WiFi Modem
  WiFi.mode( WIFI_STA);

 //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;

    // reset settings - wipe stored credentials for testing
    // these are stored by the esp library
    //wm.resetSettings();

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result
    
    wm.setConfigPortalTimeout(180);   // set portal time to  3 min, then sleep.
    bool res;
    res = wm.autoConnect("LaskaKit AutoConnectAP","laskakit"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("Wi-Fi connected successfully");
    }
}

// Přečíst data z BMP280 | Read data from BMP280
void readBMP(){
  temperature = bmp.readTemperature();
  pressure    = bmp.readPressure() / 100.0F;  
  
  Serial.print("Temp: "); Serial.print(temperature); Serial.println("°C");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.println("hPa");
}

// Přečíst data z SHT40 | Read data from SHT40
void readSHT(){
//  float SHThumidity
  sensors_event_t SHThumidity, SHTtemp; // temperature and humidity variables

  sht4.getEvent(&SHThumidity, &SHTtemp);
  Serial.print("Temperature: "); Serial.print(SHTtemp.temperature); Serial.println(" degC");
  Serial.print("Humidity: "); Serial.print(SHThumidity.relative_humidity); Serial.println("% rH");

  humidity = SHThumidity.relative_humidity;
}

// Přečíst data z DS18B20 | Read data from DS18B20
void readDalas(){
  sensor_inhouse.requestTemperatures();
  Serial.print("DS18B20: "); Serial.print(sensor_inhouse.getTempCByIndex(0)); Serial.println("°C");
}

// Měření napětí baterie | Battery voltage measurement
void readBat(){
  bat_voltage = adc.readVoltage()*deviderRatio;
  Serial.print("Battery voltage " + String(bat_voltage) + "V");
}

void setup() {
  // Hned vypneme WiFi | disable WiFi, coming from DeepSleep, as we do not need it right away
  WiFi.mode( WIFI_OFF );
  delay( 1 );
  
  adc.attach(ADC_PIN);  

  Serial.begin(115200);
  while(!Serial) {} // Wait

  // initilizace BMP280 | BMP280 Initialization
  Wire.begin(19,18); // SDA SCL
  if (! bmp.begin(BMP280_ADDRESS)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  Serial.println("-- Weather Station Scenario --");
  Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
  Serial.println("filter off");
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X1, // temperature
                  Adafruit_BMP280::SAMPLING_X1, // pressure
                  Adafruit_BMP280::FILTER_OFF   );
  delay(10);

  // initilizace SHT40 | SHT40 Initialization
  if (! sht4.begin()) 
  {
    Serial.println("SHT4x not found");
    Serial.println("Check the connection");
    while (1) delay(1);
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION); // highest resolution
  sht4.setHeater(SHT4X_NO_HEATER); // no heater
  delay(10);

  // initilizace DS18B20 | DS18B20 Initialization
  sensor_inhouse.begin();

  readBMP();
  readSHT();
  readDalas();
  readBat();
  
  // Pripojeni k WiFi | Connect to WiFi
  WiFiConnection();
  postData();

  WiFi.disconnect(true);
  GoToSleep();
}

void loop(){
  // Nepotřebujeme loop | We dont use the loop
}