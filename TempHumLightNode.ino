 
 /**
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.3 - Marcin Krzetowski
 * 
 */

static const unsigned long UPDATE_INTERVAL = 4080; // in microseconds
static const uint16_t FORCE_UPDATE_N_READS = 20; // number of sensor readings without sending message

#define MY_NODE_ID 23
// Enable debug prints to serial monitor
//#define MY_DEBUG


/*
 * Setting offset for sensors over radio network :
 * Send message C_SET with subtype: V_TEMP, V_HUM, V_LIGHT_LEVEL, V_VAR1 where payload is offset (signed float for temp & hum, signed int for light, int for loop delay)
 * 
 */

#define MY_BAUD_RATE 115200
// Enables and select radio type (if attached)
#define MY_RFM69_NEW_DRIVER
#define MY_RADIO_RFM69
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC
#define MY_RFM69_MAX_POWER_LEVEL_DBM 13
//#define MY_IS_RFM69HW
#define MY_SKETCH_NAME "TempHumLightNode"
#define MY_SKETCH_VERSION "1.3"
#define MY_SKETCH_VENDOR "(c)2018 Marceli"
#define HEARTBEAT_INTERVAL 3600000
#define DHT_DATA_PIN 4
#define DOOR_PIN 3

#include <MySensors.h>
#include <Time.h>
#include <TimeLib.h>
#include <SPI.h>
#include <DHT.h>
#include <BH1750.h>
#include <Wire.h>
#include <Bounce2.h>
#include <EEPROM.h>

//*****************************sensors attached to gw section
#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_DOOR 2
#define CHILD_ID_LIGHT 3
//****************************end of sensors attached to gw section

// Global variables
float lastTemperature;
float offsetTemperature=0;
float offsetHumidity=0;
uint16_t offsetLux=0;
float lastHum;
uint16_t lastLux;
uint16_t nNoUpdLux;
uint16_t nNoUpdatesTemp;
uint16_t nNoUpdatesHum;
uint16_t EEPROM_INTERVAL = 0;
unsigned long time_start;
unsigned long loop_time;
int time_refresh_day;
int doorStatus;
int lastDoorStatus;
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgDoor(CHILD_ID_DOOR, V_TRIPPED);
MyMessage msgLux(CHILD_ID_LIGHT, V_LIGHT_LEVEL);

DHT dht;
BH1750 lightSensor;
Bounce debouncer = Bounce();

void setup()
{
  Serial.begin(115200);
  lightSensor.begin();
  pinMode(DOOR_PIN, INPUT);    
  digitalWrite(DOOR_PIN,HIGH);
  debouncer.attach(DOOR_PIN);
  debouncer.interval(5);  
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL+1500 <= dht.getMinimumSamplingPeriod()) {
      Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
    
  }
  sleep(dht.getMinimumSamplingPeriod()+random(500,1500));  
  nNoUpdatesTemp=FORCE_UPDATE_N_READS;
  nNoUpdatesHum=FORCE_UPDATE_N_READS;
  nNoUpdLux=FORCE_UPDATE_N_READS;
  time_start = millis();    
  loop_time=random(10,100);
  doorStatus=0;
  lastDoorStatus=0;  
  lastLux=2000;
  lastTemperature=2000;
  lastHum=2000;
  EEPROM.get(0,offsetTemperature);
  EEPROM.get(4,offsetHumidity);
  EEPROM.get(8,offsetLux);
  EEPROM.get(10,EEPROM_INTERVAL);
  Serial.println("setup");
  Serial.println(sizeof(offsetLux));
  Serial.println("offsetTemperature");
  Serial.println(offsetTemperature);
  Serial.println("offsetHumidity");
  Serial.println(offsetHumidity);
  Serial.println("offsetLux");
  Serial.println(offsetLux);
  Serial.println("EEPROM_INTERVAL");
  Serial.println(EEPROM_INTERVAL);
  EEPROM_INTERVAL=UPDATE_INTERVAL+EEPROM_INTERVAL*100;  
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
    present(CHILD_ID_DOOR, S_DOOR);    
    present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);    
    requestTime();
}


void readFromSensors(){   
    Serial.println("Read from sensors");
    dht.readSensor(true);
    float temperature = dht.getTemperature()+offsetTemperature;
    if (isnan(temperature)) {
        Serial.println("Failed reading temperature from DHT!");
    } else if (abs(temperature - lastTemperature)>0.5 || nNoUpdatesTemp > FORCE_UPDATE_N_READS) {
      // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
      lastTemperature = temperature;
      // Reset no updates counter
      nNoUpdatesTemp = 0;
      send(msgTemp.set(temperature,2));      
    } else {
      // Increase no update counter if the temperature stayed the same
      nNoUpdatesTemp++;
    }
    float humidity = dht.getHumidity()+offsetHumidity ;
    if (isnan(humidity)) {
        Serial.println("Failed reading humidity from DHT");
    } else if (abs(humidity - lastHum)>2 || nNoUpdatesHum > FORCE_UPDATE_N_READS) {       
      lastHum = humidity;
      nNoUpdatesHum = 0;
      send(msgHum.set(humidity,1));  
    } else {
      nNoUpdatesHum++;
    }
    uint16_t lux = lightSensor.readLightLevel()+offsetLux;
    if ((abs(lux-lastLux)>10) || nNoUpdLux > FORCE_UPDATE_N_READS){      
      lastLux=lux;
      nNoUpdLux = 0;    
      send(msgLux.set(lux));
    }else{
      nNoUpdLux++;
    }
}

void receive (const MyMessage &message){ 
    if (message.getCommand()==1){
    if (message.type==0){
      EEPROM.put(0, message.getFloat());
      offsetTemperature=message.getFloat();
      nNoUpdatesTemp=FORCE_UPDATE_N_READS+1;
      readFromSensors();
    }
    if (message.type==1){
      EEPROM.put(4, message.getFloat());
      offsetHumidity=message.getFloat();
      nNoUpdatesHum=FORCE_UPDATE_N_READS+1;
      readFromSensors();   
    }    
    if (message.type==23){
      EEPROM.put(8, message.getInt());
      offsetLux=message.getInt();
      nNoUpdLux=FORCE_UPDATE_N_READS+1;
      readFromSensors();   
    }  
    if (message.type==24){
      EEPROM.put(10, message.getInt());
      EEPROM_INTERVAL=message.getInt();   
      EEPROM_INTERVAL=UPDATE_INTERVAL+message.getInt()*100;     
    }
    }
}

void doorSensor(){
 doorStatus = digitalRead(DOOR_PIN)== HIGH;
 
}

 
void loop()
{
    debouncer.update(); 
    doorStatus = debouncer.read();
    unsigned long current_time=millis();
    /*
      Serial.println("loop time+ UPDATE_INTERVAL");
      Serial.println(loop_time+ UPDATE_INTERVAL);
      Serial.println("current time");
      Serial.println(current_time);
      */
      
    if ((loop_time +  EEPROM_INTERVAL)<current_time){  
     // Serial.println("*************************************READ FROM SENSORS********************************************");
      loop_time=current_time;
      readFromSensors();
      if (time_refresh_day!=day()){
        requestTime();
        time_refresh_day=day();
      }
    }else{

    }

   // sleep(10);    
    if (doorStatus!=lastDoorStatus){
      send(msgDoor.set(doorStatus)); 
      lastDoorStatus=doorStatus;
    }
 
}






