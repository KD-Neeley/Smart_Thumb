/*
 * Project smart_Planter_KD-Neeley for the Smart Thumb : "Grow Your Life". A Smart Desk Lamp + Shelf + Planter
                                                        Keep your plants alive and stay on track
                                                        Includes Neopixel shelf lights w/brightness & color settings
 * Description: Utilize: 
 *                2N3906 Emitter Follower and Relay
 *                BME280
 *                Grove Dust Sensor
 *                Grove Air Quality Sensor
 *                Soil Moisture Sensor
 *                OLED Display
 * 
 *              Publish soil moisture and room environmental data
 *              Publish Data on Dashboard at Adafruit.io
 * 
 *              Automatically water the plant in 1/2 sec increments when the soil is too dry
 * 
 *              Install a button to manually water the plant
 * 
 * Author: Katie Neeley
 * Date: 03/19/2023
 */

#include <adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include <Air_Quality_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "Encoder.h"
#include "neopixel.h"
#include "kdsrainbows.h"
#include "credentials.h"
#include "IoTClassroom_CNM.h"
#include "neopixelLightSettings.h"


/****GLOBALS****/

//Constants
//OLED
int const OLED_RESET = D4;
int const TOP = 1;
int const LEFT = 1;
int const BOTTOM = 50;
int const RIGHT = 68;

//Capacitive Soil Moisture Sensor
int const CSMS = A0;

//Encoder
int const PINA = D3;
int const PINB = D2;
int const ENCBLUE = D7;
int const ENCGRN = D6;
int const ENCRED = D5;
int const ENCODERSWITCH = A5;
//Push Buttons
int const BLUEBTN = A3;
int const BLKBTN  = A4;
//Neopixels
int const PIXELPIN = D8;
int const PIXELCOUNT = 16;
//Dust Sensor
int const DUSTSENSORPIN = A2;
//Air Quality Sensor
int const AQSENSORPIN = A1;
//Pump / Emitter Follower and Relay
int const PUMPPIN = 11;
//BME280
const int BME280ADDRESS = 0x76;

/****GLOBAL VARIABLES****/

//encoder variables
int encPosA;
int encPosB;
int maxPos;
int minPos;
int position; 
int newPosition;
int encSwitch; 
int encSwitchBefore;

//neoPixel variables
int brightness;
int color;
byte buf[6];
int pixelLight;

//BME280 variables
bool status;
int tempC;
int tempC2;
int tempF;
float rtStartTime;
float rtSampleTime;

//Dust Sensor Variables
float duration;
float lowpulseoccupancy;
float ratio;
float concentration;
float concentration2;
float dsStartTime;
float dsSampleTime;

//Soil Moisture Sensor Variables
float moistureReading;
float moistureReading2;
float soilTime;
float waterIf;
float waterTime;
int dashBlueBtnStatus;
float smStartTime;
float smSampleTime;

//Black Button Variables
bool blkBtnState;
bool onOff;
bool prevBlkBtnState;

//Air Quality Variables
int airQualityQual; //Qualitative Air Quality Reading
int airQualityQual2; //Qualitative Air Quality Comparison
float aqStartTime;
float aqSampleTime;

//Timer variables
bool roomTempTime;

/****FUNCTION DECLARATIONS****/
void MQTT_connect();
bool MQTT_ping();
int pixelFill(int startpixel, int endPixel, int hexColor);

/****OBJECTS****/
//MQTT and TCP Client for cloud
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
//OLED
Adafruit_SSD1306 oled(OLED_RESET);
//Encoder
Encoder myEnc(PINA, PINB);
//Neopixel
Adafruit_NeoPixel pixel(PIXELCOUNT, PIXELPIN, WS2812B);
//Air Quality Sensor
AirQualitySensor aqSensor(AQSENSORPIN);
//BME280
Adafruit_BME280 bme280Sensor;
//Buttons
Button blueBtn (BLUEBTN);
Button blkBtn (BLKBTN);
Button encBtn (ENCODERSWITCH);
//Timer
IoTTimer stopWatch;
//NeoPixel Light Settings
smartPixelPresets smartPixelColor;
//Timers


/****PUBLISH****/
//Publish Air Quality Sensor Feed
Adafruit_MQTT_Publish airQualityFeed=Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME"/feeds/smartthumb.airqualityfeed");
//Publish Dust Sensor Feed
Adafruit_MQTT_Publish dustParticleFeed=Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME"/feeds/smartthumb.dustfeed"); 
//Publish Soil Moisture Sensor Feed
Adafruit_MQTT_Publish soilMoistureFeed=Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME"/feeds/smartthumb.soilmoisturefeed");
//Publish Water History Feed
Adafruit_MQTT_Publish waterHistoryFeed=Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME"/feeds/smartthumb.waterhistoryfeed");
//Publish BME280 Room Temp Feed
Adafruit_MQTT_Publish roomTempFeed=Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME"/feeds/smartthumb.roomtempfeed");

/****SUBSCRIBE****/
//Subscribe NeoPixel Color Selector
Adafruit_MQTT_Subscribe neoPixelColorFeed=Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/smartthumb.neopixelcolorfeed");
//Subscribe Air Quality Sensor Feed
Adafruit_MQTT_Subscribe airQualityFeedSub=Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/smartthumb.airqualityfeed");
//Subscribe Dust Sensor Feed (Concentration)
Adafruit_MQTT_Subscribe dustSensorFeedSub=Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/smartthumb.dustfeed");
//Subscribe Soil Moisture Feed
Adafruit_MQTT_Subscribe soilMoistureFeedSub=Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/smartthumb.soilmoisturefeed");
//Subscribe BME280 Environmental Sensor Room Temp Feed
Adafruit_MQTT_Subscribe roomTempFeedSub=Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/smartthumb.roomtempfeed");
//Subscribe to Dashbaord Blue Button (Green Toggle)
Adafruit_MQTT_Subscribe dashBlueBtnFeed=Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/smartthumb.bluebuttonfeed"); 


/****SYSTEM MODE****/
SYSTEM_MODE(SEMI_AUTOMATIC);

//BUGS:
//physical blue btn not turning on pump, possibly wire possibly not
//black button not turning on/off neopixels, neopixels not on, could be wiring but maybe not
//Encoder not lighting up, neopixels have no power, NONE of the encoder lights are on (could be wiring)
//Encoder not turning brightness or fluctuating lights
//Dust Sensor not reading anything
//the degree symbol on the oled is a question mark
//
//fixing
//need new statements for Encoder lights
//need new timer settings for OLED display rotating messages



/****SETUP****/
void setup() {

//INITIALIZE SERIAL MONITOR
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);

  //INITIALIZE Wi-Fi
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
  Serial.printf(".");
  }
  Serial.printf("\n\n");

  // INITIALIZE MQTT SUBSCRIPTIONS
  mqtt.subscribe(&neoPixelColorFeed); 
  mqtt.subscribe(&airQualityFeedSub); 
  mqtt.subscribe(&dustSensorFeedSub); 
  mqtt.subscribe(&soilMoistureFeedSub); 
  mqtt.subscribe(&roomTempFeedSub); 

  //INITIALIZE OLED
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();
  oled.setRotation(2);
  oled.setTextSize(1);
  oled.setTextColor(WHITE);

  //INITIALIZE NEOPIXELS
  pixel.begin();
  pixel.show();

  //INITIALIZE ENCODER
  pinMode(ENCODERSWITCH, INPUT_PULLDOWN);
  maxPos = 255;
  minPos = 0;  
  position = myEnc.read();

  //INITIALIZE BME280
  Wire.begin();
  status=bme280Sensor.begin(BME280ADDRESS);
  rtSampleTime = 30000;
  rtStartTime = millis();

  //INITIALIZE DUST SENSOR
  pinMode(DUSTSENSORPIN, INPUT);
  lowpulseoccupancy = 0;
  ratio = 0;
  concentration = 0;
  dsSampleTime = 30000;
  dsStartTime = millis();   

  //INITIALIZE AIR QUALITY SENSOR
  Serial.println("Waiting for Air Quality Sensor to Initiate...");
  delay(20000);
  if (aqSensor.init()) {
    Serial.println("Sensor ready.");
  }
  else {
    Serial.println("Sensor ERROR!");
  }
  delay(5000);
  aqSampleTime = 30000;
  aqStartTime = millis();

  //INITIALIZE WATER PUMP BASED ON SOIL MOISTURE
  waterIf = 2960;
  waterTime = 5000;
  pinMode(PUMPPIN, OUTPUT);
  smSampleTime = 30000;
  smStartTime = millis();

  //INITIALIZE PUSH BUTTONS
  pinMode(BLUEBTN, INPUT_PULLDOWN);
  pinMode(BLKBTN, INPUT_PULLDOWN);
}





/****LOOP****/
void loop() {
  //Connect and Ping MQTT
  MQTT_connect();
  MQTT_ping();

 Adafruit_MQTT_Subscribe *subscription; 
 subscription = mqtt.readSubscription(100);

  //GET ROOM TEMPERATURE FROM BME280
  if(status == false) {
    Serial.printf("BME280 at address 0x%02X failed to start", BME280ADDRESS);
  }
  tempC=bme280Sensor.readTemperature(); //deg Celsius
  tempF = map(tempC, 0,100,32,212);//CONVERT CELSIUS TO FARENHEIT
  if((millis()-rtStartTime) > rtSampleTime){
    rtStartTime = millis();
    roomTempFeed.publish(tempF);
    //PRINT TO SERIAL MONITOR
    Serial.printf("Room Temp %iF\n", tempF);
    //PRINT TO OLED
    oled.setCursor(LEFT,TOP);
    oled.printf("Room Temp%iF\n", tempF);
    oled.display();
  }
   
   
    
      
      
      
      

  


    //SOIL MOISTURE SENSOR
    //GET READING
    moistureReading = analogRead(CSMS);
   //publish every 30 seconds
    if((millis()-smStartTime) > smSampleTime){
      //PUBLISH TO DASHBOARD
      smStartTime = millis();
      soilMoistureFeed.publish(CSMS);
      //PRINT TO SERIAL MONITOR AND SET ENCODER LIGHTS
      if(moistureReading >= 2967) {
        Serial.printf("Soil is Dry: %f\n", moistureReading);
      }
      if(moistureReading > 1941 && moistureReading < 2967) {
        Serial.printf("Soil is Damp: %f\n", moistureReading);
      }
      if(moistureReading <= 1941) {
        Serial.printf("Soil is wet: %f\n", moistureReading);
      }
      //PRINT TO OLED
      oled.setCursor(LEFT,BOTTOM);
      if(moistureReading >= 2967) {
        oled.printf("Soil is Dry: %f", moistureReading);
      }
      if(moistureReading > 1941 && moistureReading < 2967) {
        oled.printf("Soil is Damp: %f", moistureReading);
      }
      if(moistureReading <= 1941) {
        oled.printf("Soil is wet: %f", moistureReading);
      }
      oled.display();
      delay(3000);
      oled.clearDisplay();
    }
  


    //TURN PUMP ON, AUTOMATICALLY WATER IF SOIL IS GETTING DRY
    //SET ENCODER LIGHTS FOR WATER STATUS
    if((millis()-soilTime > waterTime)) {
      soilTime = millis();
      if(moistureReading >= waterIf) {
        digitalWrite(PUMPPIN, HIGH);
        digitalWrite(ENCGRN, LOW);
        digitalWrite(ENCRED, LOW);
        digitalWrite(ENCBLUE, HIGH);
      }
      if((moistureReading < waterIf)) {
        digitalWrite(PUMPPIN, LOW);
        digitalWrite(ENCBLUE, LOW);
      }
    }
    //MANUALLY WATER IF BLUE BUTTON IS PUSHED
    //SET ENCODER LIGHTS FOR WATER STATUS
    if(blueBtn.isClicked()) {
      if((millis()-soilTime > waterTime)) {
        soilTime = millis();
        digitalWrite(PUMPPIN, HIGH);
        digitalWrite(ENCGRN, LOW);
        digitalWrite(ENCRED, LOW);
        digitalWrite(ENCBLUE, HIGH);
      }
      else {
        digitalWrite(PUMPPIN, LOW);
        digitalWrite(ENCBLUE, LOW);
      }
    }
    //MANUALLY WATER IF DASHBOARD BLUE BUTTON IS PUSHED
    //SET ENCODER LIGHTS FOR WATER STATUS
      if (subscription == &dashBlueBtnFeed) {
        dashBlueBtnStatus = atof((char *)dashBlueBtnFeed.lastread);
        //Check dash button status
        Serial.printf("Button Status = %i\n", dashBlueBtnStatus);
        //Water if Status == 1 (is ON "Water")
        if(dashBlueBtnStatus==1) {
          if((millis()-soilTime > waterTime)) {
            soilTime = millis();
            digitalWrite(PUMPPIN, HIGH);
            digitalWrite(ENCGRN, LOW);
            digitalWrite(ENCRED, LOW);
            digitalWrite(ENCBLUE, HIGH);
          }
          else {
            digitalWrite(PUMPPIN, LOW);
            digitalWrite(ENCBLUE, LOW);
          }
        }
      }
  

    //AIR QUALITY SENSOR
    //only publish every 30 seconds
    if((millis()-aqStartTime) > aqSampleTime) {
      aqStartTime=millis();
    //GET READING
      airQualityQual = aqSensor.slope();
      //Get Qualitative Value for Air Quality Sensor
      if (airQualityQual == AirQualitySensor::FORCE_SIGNAL) {
        Serial.printf("High pollution! Force signal active.\n");
      }
      else if (airQualityQual == AirQualitySensor::HIGH_POLLUTION) {
        Serial.printf("High pollution!\n");
      }
      else if (airQualityQual == AirQualitySensor::LOW_POLLUTION) {
        Serial.printf("Low pollution!\n");
      }
      else if (airQualityQual == AirQualitySensor::FRESH_AIR) {
        Serial.printf("Fresh air.\n");
      }
      airQualityFeed.publish(airQualityQual);
       //Display Air Quality on OLED
      oled.setCursor(LEFT,BOTTOM);
      if (airQualityQual == AirQualitySensor::FORCE_SIGNAL) {
        oled.printf("High pollution! Force signal active.");
      }
      else if (airQualityQual == AirQualitySensor::HIGH_POLLUTION) {
        oled.printf("High pollution!");
      }
      else if (airQualityQual == AirQualitySensor::LOW_POLLUTION) {
        oled.printf("Low pollution!");
      }
      else if (airQualityQual == AirQualitySensor::FRESH_AIR) {
        oled.printf("Fresh air.");
      }
      oled.display();
      delay(3000);
      oled.clearDisplay();
    }
  
    
   
   


    //DUST SENSOR
    duration = pulseIn(DUSTSENSORPIN, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;
    //only publish every 30 seconds
    if((millis()-dsStartTime) > dsSampleTime){
       dsStartTime = millis();
      //integer percentage 0=>100
      ratio=lowpulseoccupancy/(dsSampleTime * 10.0);
      //Using spec sheet curve by Christopher Nafis
      concentration = 1.1*pow(ratio, 3)*pow(ratio,2)+520+ratio+0.62;
      dustParticleFeed.publish(concentration);
      Serial.printf("Dust Reads %f", concentration);
      //reset
      lowpulseoccupancy =0;
      oled.setCursor(LEFT,BOTTOM);
      oled.printf("Particles: %f\n", concentration);
      delay(3000);
      oled.clearDisplay();
    }
 


    //NEOPIXEL SETTINGS
    //Set neopixel brightness using Encoder
    position=myEnc.read();
    brightness=map(position, 0, 95, minPos, maxPos);
    pixel.setBrightness(brightness);
    pixel.show();

   

    //toggle neopixels on/off with Black Button
    blkBtnState = digitalRead(BLKBTN);
    prevBlkBtnState = blkBtnState;
    blkBtnState = digitalRead(BLKBTN);
      if(blkBtnState != prevBlkBtnState) {
            if(blkBtnState) {
            onOff =!onOff;
            }
        if(onOff) {
         //on
        //  brightness = myEnc.read();
        //  pixel.setBrightness(brightness);
        //Cycle through array settings for neopixel colors
        // encSwitch = digitalRead(ENCODERSWITCH);
        // encSwitchBefore = encSwitch;
        // encSwitch = digitalRead(ENCODERSWITCH);
        // if (encSwitch != encSwitchBefore) {
        //   smartPixelColor.goToNext();
        // }
                        //Set Custom Color from Dashboard Color Picker
        //   if (subscription == &neoPixelColorFeed) {
        //     Serial.printf("The Color is = %s\n", (char *)neoPixelColorFeed.lastread);
        //     memcpy(buf, &neoPixelColorFeed.lastread[1], 6); //strip off the '#'
        //     Serial.printf("Buffer: %s\n", (char *)buf);
        //     color = strtol((char *)buf, NULL, 16); // convert string to int (hex)
        //     Serial.printf("Buffer: 0x%02X \n", color);
        //   for(int i=0; i<PIXELCOUNT; i++) {
        //     pixel.setPixelColor(i, color);
        //     pixel.show();
        //   }
        // }
        pixel.setBrightness(50);
        pixel.setPixelColor(0, fullred);
         pixel.show();
        }
        if(!onOff){
          //off
          pixel.clear();
          pixel.show();
          }
      }
      prevBlkBtnState = blkBtnState;

      //PUBLISH WATER HISTORY TO DASHBOARD COMING SOON
    
      //DATE / CLOCK / LOCATION SETTINGS COMING SOON
      //print time on OLED on right top
      //print the date under the time on the OLED

      //OUTSIDE TEMPERATURE SETTINGS COMING SOON
      //use location settings to set the weather feed

      //GROW LIGHT SETTINGS COMING SOON
      //use time settings to set the grow light to turn on at sunset
      //use time settings to set the grow light to turn off at sunrise

      //CALENDAR SETTINGS COMING SOON
      //need to learn how to import Google Calendar events for the current date
      //print events on OLED main screen, scrolling text
      //publish events to dashboard as text feed

      //WIRELESS CAMERA SETTINGS COMING SOON
      //need to learn how to display the camera feed on the dashboard



}





/****FUNCTION DEFINITIONS****/
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

int pixelFill(int startPixel, int endPixel, int hexColor) {
  for(int i=startPixel; i<endPixel+1; i++) {
    pixel.setPixelColor(i, hexColor);
    for (int n = hexColor; n<7; n++); {
      pixel.setPixelColor(i, hexColor);
    };  
  pixel.show();
  };
    Serial.printf("The hexColor is %i\n", hexColor);
  return(startPixel, endPixel, hexColor);
}