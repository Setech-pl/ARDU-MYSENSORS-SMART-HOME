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
 * LED purposes:
 * - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs in your sketch, only the LEDs that is defined is used.
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error
 *
 * See http://www.mysensors.org/build/esp8266_gateway for wiring instructions.
 * nRF24L01+  ESP8266
 * VCC        VCC
 * CE         GPIO4
 * CSN/CS     GPIO15
 * SCK        GPIO14
 * MISO       GPIO12
 * MOSI       GPIO13
 * GND        GND
 *
 * Not all ESP8266 modules have all pins available on their external interface.
 * This code has been tested on an ESP-12 module.
 * The ESP8266 requires a certain pin configuration to download code, and another one to run code:
 * - Connect REST (reset) via 10K pullup resistor to VCC, and via switch to GND ('reset switch')
 * - Connect GPIO15 via 10K pulldown resistor to GND
 * - Connect CH_PD via 10K resistor to VCC
 * - Connect GPIO2 via 10K resistor to VCC
 * - Connect GPIO0 via 10K resistor to VCC, and via switch to GND ('bootload switch')
 *
  * Inclusion mode button:
 * - Connect GPIO5 via switch to GND ('inclusion switch')
 *
 * Hardware SHA204 signing is currently not supported!
 *
 * Make sure to fill in your ssid and WiFi password below for ssid & pass.
 */

// Enable debug prints to serial monitor
//#define MY_DEBUG

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
#define MY_BAUD_RATE 115200

// Enables and select radio type (if attached)
#define MY_RFM69_NEW_DRIVER
#define MY_RFM69_MAX_POWER_LEVEL_DBM (10u)
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RF69_IRQ_PIN D2
#define MY_RF69_IRQ_NUM MY_RF69_IRQ_PIN
#define MY_RFM69_CS_PIN D8
#define MY_RF69_IRQ_NUM MY_RF69_IRQ_PIN D2

#define MY_GATEWAY_ESP8266
#define MY_ESP8266_SSID "teletubisie"
#define MY_ESP8266_PASSWORD "****************************"
#define MY_SKETCH_NAME "WIFI gate "
#define MY_SKETCH_VERSION "1.2"
#define MY_SKETCH_VENDOR "(c)2018 Marceli"

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
#define MY_GATEWAY_MAX_CLIENTS 2

// Controller ip address. Enables client mode (default is "server" mode).
// Also enable this if MY_USE_UDP is used and you want sensor data sent somewhere.
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 1, 200

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE

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


#if defined(MY_USE_UDP)
  #include <WiFiUdp.h>
#endif

#define DHT_DATA_PIN D1
#define SENSOR_TEMP_OFFSET 0
//#define MOTION_DATA_PIN 10 //D4


#include <ESP8266WiFi.h>
#include <MySensors.h>
#include <Time.h>
#include <LiquidCrystal_I2C.h> 
//*****************************sensors attached to gw section

#include <SPI.h>
#include <DHT.h>
#include <Wire.h>

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint16_t UPDATE_INTERVAL = 500000; 

static const uint16_t FORCE_UPDATE_N_READS = 25;


#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_MOTION 2
#define CHILD_ID_LIGHT 3


// Global variables
float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
uint16_t nNoUpdates;
bool metric = true;
bool last_detected_move = false;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgMotion(CHILD_ID_MOTION, V_TRIPPED);



DHT dht;
/*
LiquidCrystal_I2C lcd(0x4E, 16, 2);
LiquidCrystal_I2C lcd2(0x27, 16, 2);
*/
LiquidCrystal_I2C lcd(0x27, 16, 2);

//****************************end of sensors attached to gw section


void setup()
{
 Wire.begin(2,0);
  lcd.init();   // initializing the LCD
  lcd.backlight(); // Enable or Turn On the backlight 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(MY_SKETCH_NAME); // Start Printing
  lcd.print(" v.");
  lcd.print(MY_SKETCH_VERSION);
  lcd.setCursor(0, 1);
  lcd.print(MY_SKETCH_VENDOR);
  //pinMode(MOTION_DATA_PIN, INPUT); 
  //attachInterrupt(digitalPinToInterrupt(MOTION_DATA_PIN), MotionSensorCallback, CHANGE);
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor

  if (UPDATE_INTERVAL <= dht.getMinimumSamplingPeriod()) {
    #ifdef MY_DEBUG
      Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
    #endif
  }
  sleep(dht.getMinimumSamplingPeriod()+1000);
  nNoUpdates=UPDATE_INTERVAL;
  nNoUpdatesTemp=FORCE_UPDATE_N_READS ;
  nNoUpdatesHum=FORCE_UPDATE_N_READS ;
  wait(2000);
  lcd.clear();
  }

void receiveTime(uint32_t ts){

//  setTime(ts);
}

void presentation()
{
    sendSketchInfo(MY_SKETCH_NAME, MY_SKETCH_VERSION);
    present(CHILD_ID_HUM, S_HUM);
    present(CHILD_ID_TEMP, S_TEMP);
    present(CHILD_ID_MOTION, S_MOTION);    
    metric = getControllerConfig().isMetric;  
    requestTime();
    
}

/*
void MotionSensorCallback(){
  bool detected_move = digitalRead(MOTION_DATA_PIN) == HIGH;
  if (detected_move) {
    
      Serial.println("Detected move, inside interrupt");  
      Serial.println(detected_move);    
      if (!last_detected_move){
        last_detected_move=detected_move;
        msgMotion.set("1");
        send(msgMotion);
      }
  }else {
    
      Serial.println("Detected no move");  
      Serial.println(detected_move);
    
    if (last_detected_move=detected_move){
      last_detected_move=detected_move;
      msgMotion.set("0");
      send(msgMotion);
    }    
  }
}
*/

void lcdPrintDHT(const float temperature,const float humidity){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("t:");
  lcd.print(temperature);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("h:");
  lcd.print(humidity);
  lcd.print("%");
}


void loop()
{
  if (nNoUpdates < UPDATE_INTERVAL){      
    nNoUpdates++;
  }else{
    nNoUpdates=0;
    dht.readSensor(true);
    float temperature = dht.getTemperature();
    if (isnan(temperature)) {
      #ifdef MY_DEBUG
        Serial.println("Failed reading temperature from DHT!");
      #endif
    } else if (abs(temperature - lastTemp)>0.2 || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {
      // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
      lastTemp = temperature;
      if (!metric) {
        temperature = dht.toFahrenheit(temperature);
      }
      // Reset no updates counter
      nNoUpdatesTemp = 0;
      temperature += SENSOR_TEMP_OFFSET;
      send(msgTemp.set(temperature,2));
      lcdPrintDHT(temperature,lastHum );
    } else {
      // Increase no update counter if the temperature stayed the same
      nNoUpdatesTemp++;
    }
    // Get humidity from DHT library
    float humidity = dht.getHumidity();
    if (isnan(humidity)) {
      #ifdef MY_DEBUG
        Serial.println("Failed reading humidity from DHT");
      #endif
    } else if (abs(humidity - lastHum)>0.2 || nNoUpdatesHum == FORCE_UPDATE_N_READS) {
      // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
      lastHum = humidity;
      // Reset no updates counter
      nNoUpdatesHum = 0;
      send(msgHum.set(humidity,1));
      lcdPrintDHT(lastTemp,humidity);      
    } else {
      // Increase no update counter if the humidity stayed the same
      nNoUpdatesHum++;
    }
    wait(1000); //one second delay
    }
}
void LCDprint(const char *firstRow, const char *secondRow){
}


