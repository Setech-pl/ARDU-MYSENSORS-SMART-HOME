/**
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.1 - Marcin Krzetowski
 *
 * DESCRIPTION
 * The EthernetGateway sends data received from sensors to the WiFi link.
 * The gateway also accepts input on ethernet interface, which is then sent out to the radio network.
 *
 *
 * Make sure to fill in your ssid and WiFi password below for ssid & pass.
 */

// Enable deobug prints to serial monitor
//#define MY_DEBUG

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
#define MY_BAUD_RATE 115200

// Enables and select radio type (if attached)
#define MY_RFM69_NEW_DRIVER
#define MY_RFM69_MAX_POWER_LEVEL_DBM (20u)
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RF69_IRQ_PIN D2
#define MY_RF69_IRQ_NUM MY_RF69_IRQ_PIN
#define MY_RFM69_CS_PIN D8
#define MY_RF69_IRQ_NUM MY_RF69_IRQ_PIN D2
#define MY_GATEWAY_ESP8266
#define MY_ESP8266_SSID "WIFI NETWORK SID"
#define MY_ESP8266_PASSWORD "*****************"
#define MY_SKETCH_NAME "WIFIgateway"
#define MY_SKETCH_VERSION "1.4"
#define MY_SKETCH_VENDOR "(c)2018 Marceli"


const char* ssid = MY_ESP8266_SSID;
const char* password = MY_ESP8266_PASSWORD;

// Enable UDP communication
//#define MY_USE_UDP  // If using UDP you need to set MY_CONTROLLER_IP_ADDRESS below

// Set the hostname for the WiFi Client. This is the hostname
// it will pass to the DHCP server if not static.
#define MY_ESP8266_HOSTNAME "sensorgateway"
#define MY_NODE_ID 1

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
#define MY_IP_ADDRESS 192,168,1,201

// If using static ip you can define Gateway and Subnet address as well
#define MY_IP_GATEWAY_ADDRESS 192,168,1,1
#define MY_IP_SUBNET_ADDRESS 255,255,255,0

// The port to keep open on node server mode
#define MY_PORT 5003
// How many clients should be able to connect to this gateway (default 1)
#define MY_GATEWAY_MAX_CLIENTS 4

// Controller ip address. Enables client mode (default is "server" mode).
// Also enable this if MY_USE_UDP is used and you want sensor data sent somewhere.
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 1, 200

// Enable inclusion mode
//#define MY_INCLUSION_MODE_FEATURE

// Enable Inclusion mode button on gateway
#define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3

// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Flash leds on rx/tx/err
// Led pins used if blinking feature is enabled above
#define MY_DEFAULT_ERR_LED_PIN 16  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  16  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  16  // the PCB, on board LED

#define DHT_DATA_PIN D1
#define SENSOR_TEMP_OFFSET -2.5
#define BUTTON_PIN D0

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <MySensors.h>
#include <Time.h>
#include <TimeLib.h>
#include <LiquidCrystal_I2C.h> 
#include <SPI.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>


// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
#define MAX_BACKLIGHT_TIME  90000
static const uint16_t UPDATE_INTERVAL = 10000; 
static const uint16_t FORCE_UPDATE_N_READS = 300;

//*****************************sensors attached to gw section

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_MOTION 2
#define CHILD_ID_LIGHT 3
#define CHILD_ID_BARO 4
#define CHILD_ID_FORECAST 5


//****************************end of sensors attached to gw section

// Global variables
float lastTemperature;
float minTemp;
float maxTemp;
float lastHum;
float lastPress;
float pressure;
float internalTemp;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
uint8_t nNoUpdatesPressure;
bool detected_motion = false;
bool last_button = false;
uint8_t nStageNumber = 0;
unsigned long button_time;
unsigned long display_time;
unsigned long time_start;
unsigned long forecast_time;
unsigned long loop_time;
bool display_backlight=false;
int time_refresh_day;

/*
 * Weather forecast section based by Mysensors example
 */
int gforecast;
const float ALTITUDE = 128; 
const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
const char *weather_pl[] = {"stabilna","slonecznie","niestabilna","burze deszcze","nieznana"};
enum FORECAST
{
    STABLE = 0,            // "Stable Weather Pattern"
    SUNNY = 1,            // "Slowly rising Good Weather", "Clear/Sunny "
    CLOUDY = 2,            // "Slowly falling L-Pressure ", "Cloudy/Rain "
    UNSTABLE = 3,        // "Quickly rising H-Press",     "Not Stable"
    THUNDERSTORM = 4,    // "Quickly falling L-Press",    "Thunderstorm"
    UNKNOWN = 5            // "Unknown (More Time needed)
};
const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];
int minuteCount = 0;
bool firstRound = true;
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;
float dP_dt;
//float lastPressure = -1;
int lastForecast = -1;
#define CONVERSION_FACTOR (1.0/10.0)
/*
 * 
 */
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgPressure(CHILD_ID_BARO, V_PRESSURE);
MyMessage msgMotion(CHILD_ID_MOTION, V_TRIPPED);
MyMessage msgForecast(CHILD_ID_FORECAST,V_FORECAST);
DHT dht;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_BMP085 bmp;



void setup()
{
  Wire.begin(2,0);
  Serial.begin(115200);
  lcd.init();   // initializing the LCD
  lcd.display();
  lcd.backlight(); // Enable or Turn On the backlight 
  display_backlight=true;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(MY_SKETCH_NAME); // Start Printing
  lcd.print(" ");
  lcd.print(MY_SKETCH_VERSION);
  lcd.setCursor(0, 1);
  lcd.print(MY_SKETCH_VENDOR);
  pinMode(BUTTON_PIN, INPUT);    
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL <= dht.getMinimumSamplingPeriod()) {
    #ifdef MY_DEBUG
      //Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
    #endif 
  }
  sleep(dht.getMinimumSamplingPeriod()+200);

  if (!bmp.begin())
  {
  #ifdef MY_DEBUG    
    //Serial.println("Nie odnaleziono czujnika BMP085 / BMP180");
  #endif  
    lcd.clear();
    lcd.println("Error BMP180 ");
    wait(1000);
  } 
  nNoUpdatesTemp=FORCE_UPDATE_N_READS ;
  nNoUpdatesHum=FORCE_UPDATE_N_READS ;
  nNoUpdatesPressure = FORCE_UPDATE_N_READS;
  wait(100);
  display_time=millis();
  time_start = display_time;  
  forecast_time = display_time;
  loop_time=millis();
  minTemp=100;
  maxTemp=-100;


ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.println("Aktualizacja " + type);
    wait(50);
  });
  ArduinoOTA.onEnd([]() {
    lcd.clear();
    lcd.println("End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    lcd.setCursor(0,1);
    lcd.printf("Postep: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    lcd.clear();
    if (error == OTA_AUTH_ERROR) {
      lcd.println("Blad autoryzacji");
    } else if (error == OTA_BEGIN_ERROR) {
      lcd.println("Blad OTA begin");
    } else if (error == OTA_CONNECT_ERROR) {
      lcd.println("Blad OTA connect");
    } else if (error == OTA_RECEIVE_ERROR) {
      lcd.println("Blad OTA receive");
    } else if (error == OTA_END_ERROR) {
      lcd.println("Blad OTA end");
    }
  });
  ArduinoOTA.begin();
}
 

void receiveTime(uint32_t ts){
  setTime(ts);
  time_refresh_day=day();
}

void presentation()
{
    sendSketchInfo(MY_SKETCH_NAME, MY_SKETCH_VERSION);
    present(CHILD_ID_HUM, S_HUM);
    present(CHILD_ID_TEMP, S_TEMP);
    present(CHILD_ID_MOTION, S_MOTION);    
    present(CHILD_ID_BARO, S_BARO);         
    requestTime();
}

void ButtonOnPress(){
  button_time=millis();
}

void ButtonOnRelease(){
  if (millis()-button_time>1000UL){
    if (!display_backlight){
      lcd.display();
      lcd.backlight();
      display_backlight = true;
    }
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print("Restart systemu...");  
    WiFi.forceSleepBegin(); 
    wdt_reset(); 
    ESP.restart(); 
    while(1)wdt_reset();
  }
  if (display_backlight==true){
     nStageNumber++;    
  }
  if (nStageNumber>4) {
    nStageNumber=0;
  }
  display_time=millis();
  if (!display_backlight){
    lcd.display();
    lcd.backlight(); // Enable or Turn On the backlight 
    display_backlight = true;    
  }
  lcdPrintDHT(lastTemperature,lastHum,lastPress,detected_motion );
  wait(460);
}


/* Commands are
 * #MOTIONON
 * #MOTIONOF
 * 
 */
 
void MotionFromSerial(){
  //String readString;
  char serial_buffer [16];
  int serial_cnt = 0;
  bool wasData = false;
  bool was_motion = false;
  while (Serial.available()) {
    delay(3);  
    if (Serial.available() >0 ) {
      wasData=true;
      char c = Serial.read();  //gets one byte from serial buffer
        serial_buffer[serial_cnt++] = c;
      if ((c == '\n') || (serial_cnt == sizeof(serial_buffer)-1))
       {
            serial_buffer[serial_cnt] = '\0';
            serial_cnt = 0;            
       }      
    } 
  }
  
  if (wasData){ 
  if (strncmp(serial_buffer,"#MOTIONON",9)==0){
    was_motion=true;
  }
  if (strncmp(serial_buffer,"#MOTIONOFF",9)==0){
    was_motion=false;
  }
  }  
  if (was_motion!=detected_motion && wasData){
    detected_motion=was_motion;
    if (detected_motion) {   
        msgMotion.set("1");
        send(msgMotion);
        lcdPrintDHT(lastTemperature,lastHum,lastPress,detected_motion);                        
    }else {   
        msgMotion.set("0");
        send(msgMotion);
        lcdPrintDHT(lastTemperature,lastHum,lastPress,detected_motion);                         
    }    
  }
}

void printDigits(byte digits){
 // utility function for digital clock display: prints preceding colon and leading 0
 lcd.print(":");
 if(digits < 10)
   lcd.print('0');
 lcd.print(digits,DEC);
}

void lcdPrintDHT(const float temperature,const float humidity, float pressure,const bool motion){
  lcd.clear();
  lcd.setCursor(0, 0); 
  unsigned long time_now;  
  time_now=millis()-time_start;
  switch(nStageNumber){
    case 0:     
      lcd.print(temperature);
      lcd.print("C");
      lcd.setCursor(9,0);
      lcd.print((int) pressure);
      lcd.print("hPa");
      lcd.setCursor(0,1);
      lcd.print(humidity);
      lcd.print("% ");
      lcd.setCursor(10,1);
      lcd.print(hour(),DEC);  
      printDigits(minute());  
      if (motion) {
        lcd.print("!");
      }    
    break;
    case 1:
      lcd.print("Prognoza pogody");
      lcd.setCursor(0,1);
      lcd.print(" ");
      lcd.print(weather_pl[gforecast]);
    break;
    case 2:
      lcd.print("Min temp: ");
      lcd.print(minTemp);
      lcd.print("C");
      lcd.setCursor(0,1);
      lcd.print("Max temp: ");
      lcd.print(maxTemp);
      lcd.print("C");
    break;
    case 3:
      lcd.print("Czas: ");      
      if (time_now>=1000 && time_now<60000){
        lcd.print((int) time_now/1000);
        lcd.print("(s)");
      }
      if (time_now>=60000 && time_now<3600000){
        lcd.print((int) time_now/60000);
        lcd.print("(m)");
      }
      if (time_now>=3600000 && time_now<86400000){
        lcd.print((int) time_now/3600000);
        lcd.print("(g)");
      }
      if (time_now>86400000){
        lcd.print((int) time_now/86400000);
        lcd.print("(d)");
      }      
      lcd.setCursor(0,1);
      lcd.print("Temp wewn: ");
      lcd.print(internalTemp);
    break;
    case 4:
      long rssi = WiFi.RSSI();
      lcd.print(MY_ESP8266_SSID);
      lcd.print("(");
      lcd.print(WiFi.RSSI());
      lcd.print(")");
      lcd.setCursor(0,1);
      if (WiFi.status() != WL_CONNECTED){
        lcd.print("WIFI Error!!"); 
      }else{
        lcd.print(WiFi.localIP());
      }
    break;
  }
}

void readFromSensors(){   
    dht.readSensor(true);
    float temperature = dht.getTemperature()+SENSOR_TEMP_OFFSET;
    //float pressure = bmp.readPressure()/100;
    float pressure = (bmp.readSealevelPressure(ALTITUDE) / 100.0)-3;
    
    internalTemp = bmp.readTemperature();
   // temperature=(temperature*3+tempBMP)/4;
    if (isnan(pressure)) {
      #ifdef MY_DEBUG
        //Serial.println("Failed reading pressure from BMP180!");
      #endif
    } else if (abs(pressure - lastPress)>1 || nNoUpdatesPressure >= FORCE_UPDATE_N_READS) {
      lastPress = pressure;
      nNoUpdatesPressure = 0;
      send(msgPressure.set(pressure,2));
      lcdPrintDHT(lastTemperature,lastHum,pressure, detected_motion );
    } else {
      nNoUpdatesPressure++;
    }
   
    if (isnan(temperature)) {
      #ifdef MY_DEBUG
        //Serial.println("Failed reading temperature from DHT!");
      #endif
    } else if (abs(temperature - lastTemperature)>0.5 || nNoUpdatesTemp >= FORCE_UPDATE_N_READS) {
      // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
      lastTemperature = temperature;
      // Reset no updates counter
      nNoUpdatesTemp = 0;
      if (minTemp>temperature){
        minTemp=temperature;
      }      
      if (maxTemp<temperature){
        maxTemp=temperature;
      }
      send(msgTemp.set(temperature,2));
      lcdPrintDHT(temperature,lastHum,lastPress,detected_motion );
    } else {
      // Increase no update counter if the temperature stayed the same
      nNoUpdatesTemp++;
    }
    // Get humidity from DHT library
    float humidity = dht.getHumidity();
    if (isnan(humidity)) {
      #ifdef MY_DEBUG
        //println("Failed reading humidity from DHT");
      #endif
    } else if (abs(humidity - lastHum)>1 || nNoUpdatesHum >= FORCE_UPDATE_N_READS) {
      // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
      lastHum = humidity;
      // Reset no updates counter
      nNoUpdatesHum = 0;
      send(msgHum.set(humidity,1));
      lcdPrintDHT(lastTemperature,humidity, lastPress,detected_motion);      
    } else {
      // Increase no update counter if the humidity stayed the same
      nNoUpdatesHum++;
    }
}


float getLastPressureSamplesAverage()
{
    float lastPressureSamplesAverage = 0;
    for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
    {
        lastPressureSamplesAverage += lastPressureSamples[i];
    }
    lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

    return lastPressureSamplesAverage;
}

// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
    // Calculate the average of the last n minutes.
    int index = minuteCount % LAST_SAMPLES_COUNT;
    lastPressureSamples[index] = pressure;

    minuteCount++;
    if (minuteCount > 185)
    {
        minuteCount = 6;
    }

    if (minuteCount == 5)
    {
        pressureAvg = getLastPressureSamplesAverage();
    }
    else if (minuteCount == 35)
    {
        float lastPressureAvg = getLastPressureSamplesAverage();
        float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
        if (firstRound) // first time initial 3 hour
        {
            dP_dt = change * 2; // note this is for t = 0.5hour
        }
        else
        {
            dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
        }
    }
    else if (minuteCount == 65)
    {
        float lastPressureAvg = getLastPressureSamplesAverage();
        float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
        if (firstRound) //first time initial 3 hour
        {
            dP_dt = change; //note this is for t = 1 hour
        }
        else
        {
            dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
        }
    }
    else if (minuteCount == 95)
    {
        float lastPressureAvg = getLastPressureSamplesAverage();
        float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
        if (firstRound) // first time initial 3 hour
        {
            dP_dt = change / 1.5; // note this is for t = 1.5 hour
        }
        else
        {
            dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
        }
    }
    else if (minuteCount == 125)
    {
        float lastPressureAvg = getLastPressureSamplesAverage();
        pressureAvg2 = lastPressureAvg; // store for later use.
        float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
        if (firstRound) // first time initial 3 hour
        {
            dP_dt = change / 2; // note this is for t = 2 hour
        }
        else
        {
            dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
        }
    }
    else if (minuteCount == 155)
    {
        float lastPressureAvg = getLastPressureSamplesAverage();
        float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
        if (firstRound) // first time initial 3 hour
        {
            dP_dt = change / 2.5; // note this is for t = 2.5 hour
        }
        else
        {
            dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
        }
    }
    else if (minuteCount == 185)
    {
        float lastPressureAvg = getLastPressureSamplesAverage();
        float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
        if (firstRound) // first time initial 3 hour
        {
            dP_dt = change / 3; // note this is for t = 3 hour
        }
        else
        {
            dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
        }
        pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
        firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
    }

    int forecast = UNKNOWN;
    if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
    {
        forecast = UNKNOWN;
    }
    else if (dP_dt < (-0.25))
    {
        forecast = THUNDERSTORM;
    }
    else if (dP_dt > 0.25)
    {
        forecast = UNSTABLE;
    }
    else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
    {
        forecast = CLOUDY;
    }
    else if ((dP_dt > 0.05) && (dP_dt < 0.25))
    {
        forecast = SUNNY;
    }
    else if ((dP_dt >(-0.05)) && (dP_dt < 0.05))
    {
        forecast = STABLE;
    }
    else
    {
        forecast = UNKNOWN;
    }

    // uncomment when debugging
   /*
    Serial.print(F("Forecast at minute "));
    Serial.print(minuteCount);
    Serial.print(F(" dP/dt = "));
    Serial.print(dP_dt);
    Serial.print(F("kPa/h --> "));
    Serial.println(weather[forecast]);
*/
    return forecast;
}

void calculaceForecast(){
    float fpressure = (bmp.readSealevelPressure(ALTITUDE) / 100.0)-3;
    gforecast = sample(fpressure);
    /*
    Serial.print("Pressure = ");
    Serial.print(fpressure);
    Serial.println(" hPa");
    Serial.print("Forecast = ");
    Serial.println(weather[gforecast]);
    
    if (temperature != lastTemp) 
    {
        lastTemp = temperature;
    }

    if (pressure != lastPressure) 
    {
        lastPressure = pressure;
    }
*/
    if (gforecast != lastForecast)
    {
        send(msgForecast.set(weather[gforecast]));
        lastForecast = gforecast;
    }
}


 
void loop()
{
  bool btnStatus = false;
  unsigned long current_time;
  current_time=millis();
  if (abs(current_time-display_time)>MAX_BACKLIGHT_TIME){
      lcd.noDisplay();
      lcd.noBacklight();
      nStageNumber=0;
      display_backlight=false;
    }
  
  if (current_time- forecast_time>60000){
    forecast_time=current_time;
    calculaceForecast();
  }
 btnStatus=digitalRead(BUTTON_PIN)==HIGH;
 if ((btnStatus==true) && (last_button==false)){
      last_button=true;    
      ButtonOnPress();
 }
 if ((btnStatus==false) && (last_button==true)){
      last_button=false;    
      ButtonOnRelease();
 }
  if ((loop_time + UPDATE_INTERVAL)<current_time){      
    loop_time=current_time;
    readFromSensors();
    if (time_refresh_day!=day()){
       requestTime();
       minTemp=100;
       maxTemp=-100;
       time_refresh_day=day();
    }
  }
  MotionFromSerial();
  ArduinoOTA.handle();
}




