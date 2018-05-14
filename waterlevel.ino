//Define constans
#define I2C_ADDR          0x27        //Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN      3
#define En_pin             2
#define Rw_pin             1
#define Rs_pin             0
#define D4_pin             4
#define D5_pin             5
#define D6_pin             6
#define D7_pin             7




#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC
#define MY_RFM69_NEW_DRIVER
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_NODE_ID 102
#define MY_RFM69_MAX_POWER_LEVEL_DBM 10

#define MY_RF69_IRQ_PIN 2
#define MY_RF69_IRQ_NUM 2

#define MY_DEBUG
/*
#include <MySensors.h>
*/

/*
const int ECHO_PIN = 3;
const int TRIGGER_PIN = 2;
*/
const int ECHO_PIN = 6;
const int TRIGGER_PIN = 5;
const int MAX_DISTANCE = 350;

#define tankoffset 70
#define tankdepth 280
#define tankradius 80
#define MAX_NO_UPDATES_DIST 15
#include <NewPing.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h> 
unsigned int lastDistance=0;
unsigned long nNoUpdatesDist=0;


LiquidCrystal_I2C      lcd(I2C_ADDR, En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

void setup()
 {

    lcd.begin (20,4);
    
    //Switch on the backlight
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);
    lcd.setCursor(0,0);   
    lcd.print("Water level meter");
    lcd.setCursor(0,1);
    lcd.print("(c) 2018 Marceli");   
    delay(2000);
    lcd.clear();
 }



void loop(){
  //unsigned int cm = sonar.ping_cm();
  unsigned int cm = sonar.convert_cm(sonar.ping_median(15));
  if ((abs(lastDistance-cm)>1) || (nNoUpdatesDist>MAX_NO_UPDATES_DIST)){    
    lcd.setCursor(0,0);
    lcd.print("Dist:");
    lcd.setCursor(6,0);
    lcd.print("    ");
    lcd.setCursor(6,0);
    lcd.print((String) cm);
    lcd.setCursor(11,0);
    lcd.print("Depth:");
    lcd.setCursor(17,0);
    lcd.print("    ");
    lcd.setCursor(17,0);
    lcd.print(tankdepth+tankoffset-cm);
    lcd.setCursor(0,1);
    lcd.print("Vol :");
    lcd.setCursor(5,1);
    lcd.print("            ");
    lcd.setCursor(5,1);
    float volumel = 3.14*(tankdepth+tankoffset-cm)*(tankradius/2)*(tankradius/2)/1000;
    lcd.print((int) volumel);
    lcd.print("l");
    lcd.setCursor(10,1);
    lcd.print(",   ");
    lcd.print(volumel/1000);
    lcd.print("m3");
    lcd.setCursor(0,3);
    lcd.print("rad:");
    lcd.setCursor(5,3);
    lcd.print(tankradius);
    lcd.print(" tank: ");
    lcd.print(tankdepth);
    nNoUpdatesDist=0;
    lastDistance=cm;
  }else{
    nNoUpdatesDist++;  
    delay(30);
  }
  
  
  }
