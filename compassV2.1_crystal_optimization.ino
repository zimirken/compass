/*
 * magic compass. uses gps and a magnetometer to point to destinations with neopixels
 * created by Rolphill
 */

//#define buttonDebug
//#define powerDebug
//#define pixelDebug
//#define pixelDebug2
//#define pixelDebug3
//#define hueDebug
//#define calOverride
//#define gpsDebug
//#define gpsDebug2
//#define reportDebug
#define watchDog
#define speedMode

#include <CapacitiveSensor.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SleepyDog.h>
#include <AsyncDelay.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <QMC5883LCompass.h>

                                                        //pin definitions
#define ringPin 2
#define buttonPin 3

#define power1Pin 4
#define power2Pin 5
#define power3Pin 6
#define gpsPower1Pin 12
#define gpsPower2Pin A1
#define gpsGndPin 11

#define sensorDrivePin 10
#define sensor1Pin 9
#define sensor2Pin 8
#define sensor3Pin 7

#define gpsTxPin 13
#define gpsRxPin A0

CapacitiveSensor   powerOn = CapacitiveSensor(sensorDrivePin,sensor1Pin);
CapacitiveSensor   Button1 = CapacitiveSensor(sensorDrivePin,sensor2Pin);
CapacitiveSensor   Button2 = CapacitiveSensor(sensorDrivePin,sensor3Pin);

SoftwareSerial mySerial(gpsRxPin, gpsTxPin);
Adafruit_GPS GPS(&mySerial);
QMC5883LCompass compass;

AsyncDelay delay100;
AsyncDelay delayRing;
AsyncDelay delaySparkle;
AsyncDelay delayFlip;
#ifdef reportDebug
AsyncDelay delayReport;
#endif

#define ringLength 8
Adafruit_NeoPixel ring = Adafruit_NeoPixel(12, ringPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel buttons = Adafruit_NeoPixel(1, buttonPin, NEO_GRB + NEO_KHZ800);
byte compassMatrix[8];
int compassMatrixVol[8];
unsigned int compassMatrixHue[8];
                                                                //constants
#define magneticDeclination 5
#define mDminutes 44
#define compassOffset 0
#define ringOffset -2

#define powerButtonSet 110
#define button1Set 110
#define button2Set 100
#define powerButtonPixel 0
#define button1Pixel 9
#define button2Pixel 10
#define button3Pixel 8

#define compassRise 31//depending on brightness setting, there is a minimum value for this to work properly
#define compassFall -15//minus doesnt seem to have the same problem
#define compassMin 0//min and max compass brightness
#define compassMax 255
#define centerPixel 11

#define homeHue 0
#define destination1Hue 33000
#define destination2Hue 54613
#define orientationHue 16000
                                                                //variables

byte powerState = 0;//set to 1 to default to on, set to 0 to default to off
int orientation;
byte calGet = 0;
//bool homeFound = 0;
int homeBearing = 0;
int destination1Bearing = 0;
int destination2Bearing = 0;
bool fadeDir = 1;
unsigned int centerPixelHue = 0;
unsigned long gpsUpdateMillis = 0;
//byte pixelInterval = 50;       // Pixel Interval (ms)
//byte pixelNumber = 12;  // Total Number of Pixels
int pixelCycle = 0;           // Pattern Pixel Cycle
//int pixelInterval2 = 50;
int pixelCycle2 = 0;
bool pixelDirection2 = true;
//unsigned long pixelPrevious = 0;        // Previous Pixel Millis
char c;
bool flip = 0;

                                                                //coordinates and calibration
//gps coordinates
//float homeLatitude = 0.0;
//float homeLongitude = 0.0;
float destination1Latitude = 0.0;
#define d1Aaddress 0
float destination1Longitude = 0.0;
#define d1Oaddress 4
float destination2Latitude = 0.0;
#define d2Aaddress 8
float destination2Longitude = 0.0;
#define d2Oaddress 12

//calibration addresses
#define c0 16
#define c1 20
#define c2 24
#define c3 28
#define c4 32
#define c5 36


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  
  EEPROM.get(d1Aaddress, destination1Latitude);
  EEPROM.get(d1Oaddress, destination1Longitude);
  EEPROM.get(d2Aaddress, destination2Latitude);
  EEPROM.get(d2Oaddress, destination2Longitude);
  
  delay100.start(250, AsyncDelay::MILLIS);
  delayRing.start(50, AsyncDelay::MILLIS);
  delaySparkle.start(500, AsyncDelay::MILLIS);
  delayFlip.start(5000, AsyncDelay::MILLIS);
  #ifdef reportDebug
  delayReport.start(10000, AsyncDelay::MILLIS);
  #endif
  
          
  //gps setup
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  //neopixel init
  
  ring.begin();
  ring.setBrightness(100);
  buttons.begin();
  buttons.setBrightness(150);
  //flash power light red on restart
   pixelPower(true);
   buttons.setPixelColor(powerButtonPixel, 255,0,0);
   buttons.show();
   delay(1000);
   buttons.setPixelColor(powerButtonPixel, 0, 0, 0);
   buttons.show();
   pixelPower(false);
   #ifdef watchDog
  Watchdog.disable();
  Watchdog.enable(8000);
  #endif
  
}
//uint32_t timer = millis();

void loop() {
                                                        //loop begin
       
  if(powerState == 0) {
    offRoutine();
  }
  if(powerState == 1) {
    wakeupRoutine();
  }
  if(powerState == 2) {
    runRoutine();
  }
  if(powerState == 3) {
    shutdownRoutine();   
  }
  #ifdef watchDog
  Watchdog.reset();
  #endif
}



void offRoutine() {                                   //handle powerup
  Watchdog.sleep(1000);                               //sleep first  
  if(powerButton() == true) {//if power button pressed
    byte flickerCount = 0;
    #define flickers 6
    pixelPower(true);                    
    while(powerButton() == true) {//power button must continue to be pressed
      if(flickerCount < flickers) {//begin flickers
        for(int i = 1;i<255;i++) {
          ring.setPixelColor(centerPixel, ring.ColorHSV(54000, 255, i));
          ring.show();
          #ifdef watchDog
          Watchdog.reset();
          #endif
          i=i+i/(6-flickerCount);
          delay(20);
        }
        for(int i = 1;i<255;i++) {
          ring.setPixelColor(1, ring.ColorHSV(54000, 64, i));
          ring.setPixelColor(0, ring.ColorHSV(54000, 64, i));
          ring.show();
          #ifdef watchDog
          Watchdog.reset();
          #endif
          i=i+i/(6-flickerCount);
          delay(20);
        }
        flickerCount++;
        buttons.setPixelColor(powerButtonPixel, buttons.ColorHSV(54000, 64, (flickerCount*42)));
        ring.setPixelColor(centerPixel, 0, 0, 0);
        ring.setPixelColor(0, 0, 0, 0);
        ring.setPixelColor(1, 0, 0, 0);
        ring.show();
        buttons.show();
        #ifdef watchDog
        Watchdog.reset();
        #endif
        delay(20);
        
      }
      if(flickerCount==flickers){//commence powerup
        powerState = 1;
        #ifdef powerDebug
        Serial.println("wakeup");
        #endif
      }
    }
    if(powerState == 0) {//if button released before finished
      buttons.setPixelColor(powerButtonPixel, 0, 0, 0);
      buttons.show();
      #ifdef watchDog
      Watchdog.reset();
      #endif
      ring.setPixelColor(centerPixel, 0, 0, 0);
      ring.setPixelColor(1, 0, 0, 0);
      ring.setPixelColor(0, 0, 0, 0);
      ring.show();
      pixelPower(false);
    }
    
  }

}

void wakeupRoutine() {                                      //wakeup routine
  //pixels are already awake, need to powerup GPS and compass, power crystal should already be lit
  pixelPower(true);
  buttons.setPixelColor(powerButtonPixel, buttons.ColorHSV(54000, 64, 255));
  buttons.show();
  for(int i=255;i>0;i--){
    ring.setPixelColor(0, ring.ColorHSV(54000, 64, 255-i));
    ring.show();
    delay(10);
    #ifdef watchDog
    Watchdog.reset();
    #endif
  }
  buttons.setPixelColor(powerButtonPixel, buttons.ColorHSV(54000, 64, 255));
  buttons.show();
  gpsPower(true);
  //everything is powered up, time for pretty
  #define pulseSource 0
  pulseWave(pulseSource, 54000, 64, 2, -1, 1);
  //add more powerup stuff here maybe?
  powerState = 2;
  fadeDir = 1;
  buttons.setPixelColor(powerButtonPixel, buttons.ColorHSV(54000, 64, 255));
  buttons.show();
}

void runRoutine() {                                                         //run routine
  //powerFade(powerButtonPixel);//fading effect on power button
  
  
  if(delay100.isExpired()) { //100ms interval, for input updating
    delay100.repeat();
    if(powerButton() == true){//check power button
      int t = 0;
      for(int i=0;i<64;i++){
        buttons.setPixelColor(powerButtonPixel, buttons.ColorHSV(54000, 64-i, 255));
        buttons.show();
        ring.setPixelColor(1, ring.ColorHSV(54000, 255, i*4));
        ring.setPixelColor(0, ring.ColorHSV(54000, 255, i*4));
        ring.show();
        delay(2);
      }
      #ifdef watchDog
      Watchdog.reset();
      #endif
      while(powerButton() == true && t<254) {
        buttons.setPixelColor(powerButtonPixel, buttons.ColorHSV(54000, 0, 255-t));
        buttons.show();
        ring.setPixelColor(centerPixel, ring.ColorHSV(54000, 64, t));
        ring.show();
        t+=5;
        if(t>254){
          powerState = 3;
          buttons.setPixelColor(powerButtonPixel, 0, 0, 0);
          buttons.show();
          for(int i=255;i>0;i--){
            ring.setPixelColor(centerPixel, ring.ColorHSV(54000, 64, i));
            ring.show();
            delay(20);
          }
          #ifdef powerDebug
          Serial.println("powerdown");
          #endif
        }
        #ifdef watchDog
        Watchdog.reset();
        #endif
      }
      if(powerState == 2) {//if power button not held for long enough
        while(t>0){
          buttons.setPixelColor(powerButtonPixel, buttons.ColorHSV(54000, 64-t, 255));
          buttons.show();
          t--;
          delay(10);
        }
        buttons.setPixelColor(powerButtonPixel, buttons.ColorHSV(54000, 64, 255));
        buttons.show();
      }
    }
    //handle other buttons
    bool b1 = button1();
    bool b2 = button2();
    if(b1 == true && b2 == false) {
      int y = 0;
      #ifdef gpsDebug
      if(false) {
      #else
      if(!GPS.fix){
      #endif
        for(int i=255;i>0;i--) {
          ring.setPixelColor(button1Pixel, i,0, 0);
          ring.show();
          delay(1);
        }
      }
      #ifdef gpsDebug
      if(true) {
      #else
      if(GPS.fix) {
      #endif        
        while(button1() == true && button2() == false) {

          if(y>253) {
            if(y==254){
              destination1Latitude = GPS.latitudeDegrees;
              destination1Longitude = GPS.longitudeDegrees;
              EEPROM.put(d1Aaddress, destination1Latitude);
              EEPROM.put(d1Oaddress, destination1Longitude);
              y=255;
            }
            ring.setPixelColor(button1Pixel, 255, 255, 255);
            ring.show();
          }
          else {
            y++;
            delay(10);
            ring.setPixelColor(button1Pixel, y/2, y/2, y);
            ring.show();
          }
        
        }
        
      }
    }

    if(b2 == true && b1 == false) {
      int y = 0;
      #ifdef gpsDebug
      if(false){
      #else
      if(!GPS.fix){
      #endif
        for(int i=255;i>0;i--) {
          ring.setPixelColor(button2Pixel, i,0, 0);
          ring.show();
          delay(1);
        }
      }
      #ifdef gpsDebug
      if(true){
      #else
      if(GPS.fix) {
      #endif
        
        while(button2() == true && button1() == false) {

          if(y>253) {
            if(y==254){
              destination2Latitude = GPS.latitudeDegrees;
              destination2Longitude = GPS.longitudeDegrees;
              EEPROM.put(d2Aaddress, destination2Latitude);
              EEPROM.put(d2Oaddress, destination2Longitude);
              y=255;
            }
            ring.setPixelColor(button2Pixel, 255, 255, 255);
            ring.show();
          }
          else {
            y++;
            delay(10);
            ring.setPixelColor(button2Pixel, y/2, y/2, y);
            ring.show();
          }
        
        }
        
      }
      //ring.setPixelColor(button2Pixel, 0, 0, 0);
      //ring.show();
    }

    if(b2 == true && b1 == true) {//calibrate
      pixelCycle = 0;
      delay(1000);
      #ifdef watchDog
      Watchdog.reset();
      #endif
      if(button1() == true && button2() == true) {
        rainbow(1);
        if(button1() == true || button2() == true) {
          #ifdef watchDog
          Watchdog.reset();
          #endif
          doCalibrate();
        }
      }
    }
    if(b1 == false && b2 == false) {//neither buttons are touched
      #ifdef speedMode
      if(GPS.fix) {
        int speedInt = GPS.speed*100.0;
        ring.setPixelColor(button1Pixel, ring.ColorHSV(constrain(map(speedInt,0,200,11000,4000),4000,11000), 255, constrain(map(speedInt, 0, 100, 0, 255),0, 255)));
        ring.setPixelColor(button2Pixel, ring.ColorHSV(constrain(map(speedInt,200,400,11000,4000),4000,11000), 255, constrain(map(speedInt, 200, 300, 0, 255),0, 255)));
        ring.setPixelColor(button2Pixel, ring.ColorHSV(constrain(map(speedInt,400,600,11000,4000),4000,11000), 255, constrain(map(speedInt, 400, 500, 0, 255),0, 255)));
      }
      else {
        ring.setPixelColor(button1Pixel, ring.gamma32(ring.ColorHSV(centerPixelHue, 255, 128)));
        ring.setPixelColor(button2Pixel, ring.gamma32(ring.ColorHSV(centerPixelHue, 255, 128)));
        ring.setPixelColor(button3Pixel, ring.gamma32(ring.ColorHSV(centerPixelHue, 255, 128)));
      }
      #else
      pixelCycle2 = map(abs(orientation-180), 0, 180, 44, 30);
      ring.setPixelColor(button1Pixel, DimColor(Wheel((pixelCycle2) & 255)));
      ring.setPixelColor(button2Pixel, DimColor(Wheel((pixelCycle2) & 255)));
      ring.setPixelColor(button3Pixel, DimColor(Wheel((pixelCycle2) & 255)));
      #endif
    }
    
    //update mag sensor
    compass.read();
    orientation = compass.getAzimuth()+180;
    //orientation = map(compass.getBearing(orientation), 0, 16, 0, 360);
    orientation = orientation+compassOffset;
    #ifdef gpsDebug
    if(true){
      homeBearing = 90;
      destination1Bearing = 180;
      destination2Bearing = 270;
    #else
    if(GPS.fix) {
      //homeBearing = getDirection(GPS.latitudeDegrees, GPS.longitudeDegrees, homeLatitude, homeLongitude);
      homeBearing = 0;
      destination1Bearing = getDirection(GPS.latitudeDegrees, GPS.longitudeDegrees, destination1Latitude, destination1Longitude);
      destination2Bearing = getDirection(GPS.latitudeDegrees, GPS.longitudeDegrees, destination2Latitude, destination2Longitude);
    #endif

    }
    else {
      homeBearing = 0;
      destination1Bearing = 0;
      destination2Bearing = 0;
    }
    
    updateCompass(orientation, homeBearing, destination1Bearing, destination2Bearing);
    
    //updateSerial();//check for new coordinates over serial
  
    

    
  }

  if(delayFlip.isExpired()){
    delayFlip.repeat();
    flip = !flip;
  }

  if(delayRing.isExpired()) {            
    delayRing.repeat();
    ringUpdate();
    checkGPS();
  }
  #ifdef debugReport
  if(delayReport.isExpired()) {
    delayReport.repeat(); 

    #ifdef pixelDebug
    for(int i=0;i<ringLength;i++){
      Serial.print(compassMatrix[i]);
      Serial.print(", ");
    }
    Serial.println("matrix");
  #endif
      Serial.print("Fix: "); Serial.print((int)GPS.fix);    
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 8); //Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees, 8); //Serial.println(GPS.lon);
      Serial.print("Speed kn: "); 
      Serial.println(GPS.speed);
    }
    Serial.print("Heading: "); 
    Serial.println(orientation);
  }
  #endif

 
  checkGPS();
//    #ifdef gpsDebug
//    if(homeFound == 0) {
//    #else
//    if ((int)GPS.fix == 1 && homeFound == 0) { //save coordinates as home if this is the first gps fix
//    #endif
//      homeFound = 1;
//      homeLatitude = GPS.latitudeDegrees;
//      homeLongitude = GPS.longitudeDegrees;
//    }
//  
//  
}

void shutdownRoutine(){                                        //shutdown routine
  gpsPower(false);
  pixelPower(false);
  powerState = 0;
  
}

void updateCompass(int heading, int d1, int d2, int d3) {
  heading = map(heading, 0, 360, 0, (ringLength*2));//map headings to twice the size of the ring
  d1 = map(d1, 0, 360, 0, (ringLength*2));
  d2 = map(d2, 0, 360, 0, (ringLength*2));
  d3 = map(d3, 0, 360, 0, (ringLength*2));
  for(int i=0;i<ringLength;i++) {//clear matrix, except flicker
    if(compassMatrix[i] != 10 && compassMatrix[i] != 11){
      compassMatrix[i] = 0;
    }
  }
  #ifdef gpsDebug
  if(true){
  #else
  if(GPS.fix) {//if gps fix
  #endif
    
    for(int m=0;m<ringLength;m++){
      if(m==(d1/2)){
        compassMatrix[m]=2;
        if((d1%2)!=0){//if odd
          compassMatrix[m]=3;
          compassMatrix[(m+1)%ringLength]=3;
        }
      }
      if(flip==true) {
        if(m==(d2/2)){
          compassMatrix[m]=4;
          if((d2%2)!=0){//if odd
            compassMatrix[m]=5;
            compassMatrix[(m+1)%ringLength]=5;
          }
        }
        if(m==(d3/2)){
          compassMatrix[m]=6;
          if((d3%2)!=0){//if odd
            compassMatrix[m]=7;
            compassMatrix[(m+1)%ringLength]=7;
          }
        }
      }
      else {
        if(m==(d3/2)){
          compassMatrix[m]=6;
          if((d3%2)!=0){//if odd
            compassMatrix[m]=7;
            compassMatrix[(m+1)%ringLength]=7;
          }
        }
        if(m==(d2/2)){
          compassMatrix[m]=4;
          if((d2%2)!=0){//if odd
            compassMatrix[m]=5;
            compassMatrix[(m+1)%ringLength]=5;
          }
        }
      }
      

    }//end filling compassMatrix with gps
    centerPixelHue = constrain(map(gpsUpdateMillis, 0, 10000, 54700, 0),0,54700);
    ring.setPixelColor(centerPixel, ring.gamma32(ring.ColorHSV(centerPixelHue)));
  }
  else{//if no gps fix
    if(delaySparkle.isExpired()) {
      delaySparkle.repeat();
      compassMatrix[random(0, ringLength-1)] = 10;
      #ifdef hueDebug
      Serial.print("Hue: ");
      Serial.println(centerPixelHue);
      #endif
    }
      centerPixelHue+=256;
  }

  for(int m=0;m<ringLength;m++){//map heading last, so it always overwrites
    if(m==(heading/2)){
        compassMatrix[m]=8;
        if((heading%2)!=0){//if odd
          compassMatrix[m]=9;
          compassMatrix[(m+1)%ringLength]=9;
        }
      }
  }


  
}

void ringUpdate() {
    //handle the pixels
  for(int p=0;p<ringLength;p++){
//    int red = Red(ring.getPixelColor(p));//extract current colors
//    int green = Green(ring.getPixelColor(p));
//    int blue = Blue(ring.getPixelColor(p));
    unsigned int targetHue = 0;//initialize targets
    int targetVol = 0;
    switch((compassMatrix[p])){
      case 0://fade away
        break;
      case 2://d1 single
        targetHue = homeHue;
        targetVol = 128;
        break;
      case 3://d1 double
        targetHue = homeHue;
        targetVol = 64;
        break;
      case 4://d2 single
        targetHue = destination1Hue;
        targetVol = 128;
        break;
      case 5://d2 double
        targetHue = destination1Hue;
        targetVol = 64;
        break;
      case 6://d3 single
        targetHue = destination2Hue;
        targetVol = 128;
        break;
      case 7://d3 double
        targetHue = destination2Hue;
        targetVol = 64;
        break;
      case 8://heading single
        targetHue = orientationHue;
        targetVol = 200;
        break;
      case 9://heading double
        targetHue = orientationHue;
        targetVol = 100;
        break;
      case 10://random flickers if no gps fix
        compassMatrixHue[p] = centerPixelHue;
        compassMatrixVol[p] = 128;
        ring.setPixelColor(centerPixel, ring.ColorHSV(centerPixelHue));
        break;
      case 11://end flicker
        compassMatrixHue[p] = centerPixelHue;
        compassMatrixVol[p] = 0;
        compassMatrix[p]=0;
        ring.setPixelColor(centerPixel, 0, 0, 0);
        break;
      default:
        break;
    }
    if(compassMatrix[p] == 10) {
      compassMatrix[p] = 11;
    }
    if(compassMatrixHue[p] < targetHue) {
      compassMatrixHue[p]+=1000;
    }
    if(compassMatrixHue[p] > targetHue){
      compassMatrixHue[p]-=1000;
    }
    if(compassMatrixVol[p] <= targetVol){
      compassMatrixVol[p]+=compassRise;
    }
    if(compassMatrixVol[p] > targetVol){
      compassMatrixVol[p]+=compassFall;
    }
    
    #ifdef pixelDebug2
    if(compassMatrix[p] > 0){
      
    }
    #endif
    ring.setPixelColor(p,ring.ColorHSV(compassMatrixHue[p], 255, constrain(compassMatrixVol[p],compassMin,compassMax)));
  }

  ring.show();//that's it for now
}

void pixelPower(bool upDown) {                                  //pixel power routine
  if(upDown == true) {//power up
    digitalWrite(power1Pin, HIGH);//set them high before enabling them to avoid crash
    digitalWrite(power2Pin, HIGH);
    digitalWrite(power3Pin, HIGH);
    pinMode(power1Pin, OUTPUT);
    pinMode(power2Pin, OUTPUT);
    pinMode(power3Pin, OUTPUT);
    delay(50);//let everything settle
    //ring.fill();//initialize pixels
    ring.show();
    //buttons.fill();
    buttons.show();
  }
  if(upDown == false) {//power down
    ring.fill();//turn off pixels
    ring.show();
    buttons.fill();
    buttons.show();
    delay(10);
    pinMode(power1Pin, INPUT);
    pinMode(power2Pin, INPUT);
    pinMode(power3Pin, INPUT);
    digitalWrite(power1Pin, LOW);
    digitalWrite(power2Pin, LOW);
    digitalWrite(power3Pin, LOW);
    delay(50);
  }
}

void gpsPower(bool upDown) {                                        //gps power routine
  if(upDown == true){//power up
    digitalWrite(gpsGndPin, LOW);
    digitalWrite(gpsPower1Pin, HIGH);
    digitalWrite(gpsPower2Pin, HIGH);
    pinMode(gpsGndPin, OUTPUT);
    pinMode(gpsPower1Pin, OUTPUT);
    pinMode(gpsPower2Pin, OUTPUT);
    delay(1000);//give the gps some extra time to settle
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate not sure if this is necessary
    compass.init();//initialize compass
    compass.setMagneticDeclination(magneticDeclination, mDminutes);
    compass.setSmoothing(10,true);
    
    float calVal1 = 0;
    float calVal2 = 0;
    float calVal3 = 0;
    EEPROM.get(c0, calVal1);
    EEPROM.get(c1, calVal2);
    EEPROM.get(c2, calVal3);
    compass.setCalibrationOffsets(calVal1, calVal2, calVal3);
    EEPROM.get(c3, calVal1);
    EEPROM.get(c4, calVal2);
    EEPROM.get(c5, calVal3);
    compass.setCalibrationScales(calVal1, calVal2, calVal3);
    #ifdef calOverride
    compass.setCalibrationOffsets(-118.00, 309.00, 427.00);
    compass.setCalibrationScales(1.15, 1.12, 0.81);
    #endif
  }
  if(upDown == false) {//power down
    //Wire.end();//shut down i2c
    pinMode(gpsPower1Pin, INPUT);//power down gps
    pinMode(gpsPower2Pin, INPUT);
    digitalWrite(gpsPower1Pin, LOW);
    digitalWrite(gpsPower2Pin, LOW);
    //i think that's it?
  }
}

//void powerFade(int buttonPixel) {                                              //power button fading effect
//  int red = Red(buttons.getPixelColor(buttonPixel));
//  int green = Green(buttons.getPixelColor(buttonPixel));
//  int blue = Blue(buttons.getPixelColor(buttonPixel));
//  #define maxx 254
//  #define minn 50
//  if(fadeDir == true) {
//    if(red != 0) {
//      red++;
//    }
//    if(green != 0) {
//      green++;
//    }
//    if(blue != 0) {
//      blue++;
//    }
//    if(red > maxx || green > maxx || blue > maxx) {
//      fadeDir = false;
//    }
//  }
//  if(fadeDir == false) {
//    if(red != 0) {
//      red--;
//    }
//    if(green != 0) {
//      green--;
//    }
//    if(blue != 0) {
//      blue--;
//    }
//    if(red == minn || green == minn || blue == minn) {
//      fadeDir = true;
//    }
//  }
//  buttons.setPixelColor(buttonPixel, red, green, blue);
//  buttons.show();
//  
//}

void doCalibrate() {                                              //calibration routine
    for(int i=0;i<255;i++){
      ring.fill(ring.Color(i/2,0,i));
      ring.show();
      delay(10);
    }
    delay(2000);
    ring.fill();
    ring.setPixelColor(centerPixel,128,0,255);
    ring.show();
    #ifdef watchDog
    Watchdog.reset();
    Watchdog.disable();
    #endif
    compass.calibrate();
    #ifdef watchDog
    Watchdog.enable(8000);
    #endif
    EEPROM.put(c0, compass.getCalibrationOffset(0));
    EEPROM.put(c1, compass.getCalibrationOffset(1));
    EEPROM.put(c2, compass.getCalibrationOffset(2));

    EEPROM.put(c3, compass.getCalibrationScale(0));
    EEPROM.put(c4, compass.getCalibrationScale(1));
    EEPROM.put(c5, compass.getCalibrationScale(2));
}

                                                            // Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    ring.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    ring.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {                                             //wheel
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return ring.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return ring.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return ring.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

bool powerButton() {                                            //read power button
  long total1 =  powerOn.capacitiveSensor(30);
  #ifdef buttonDebug
  Serial.print("PowerBtn: ");
  Serial.println(total1);
  #endif 
  if(total1 > powerButtonSet) {
    delay(50);
    total1 = powerOn.capacitiveSensor(30);
    if(total1 > powerButtonSet) {
      return true;
    }
    
  }
  else {
    return false;
  }
}

bool button1() {                                                //read button 1
  long total1 =  Button1.capacitiveSensor(30);
  #ifdef buttonDebug
  Serial.print("button1: ");
  Serial.println(total1);
  #endif 
  if(total1 > button1Set) {
    delay(50);
    total1 = Button1.capacitiveSensor(30);//debounce
    if(total1 > button1Set) {
      return true;
    }
    
  }
  else {
    return false;
  }
}

bool button2() {                                                  //read button 2
  long total1 =  Button2.capacitiveSensor(30);
  #ifdef buttonDebug
  Serial.print("button2: ");
  Serial.println(total1);
  #endif 
  if(total1 > button2Set) {
    delay(50);
    total1 = Button2.capacitiveSensor(30);//debounce
    if(total1 > button2Set) {
      return true;
    }
  }
  else {
    return false;
  }
}
/*
void updateSerial(){                                              //update serial
if(Serial.available() > 0){
        delay(200);
        float read1 = Serial.parseFloat();
        float read2 = Serial.parseFloat();
        Serial.print("Got: ");
        Serial.print(read1,6);
        Serial.print(" , ");
        Serial.println(read2,6);
      
        if(read1 == 0 || read2 == 0) {
          Serial.println("Error");
        }
        else{
          destination1Latitude = read1;
          destination1Longitude = read2;
          EEPROM.put(d1Aaddress, destination1Latitude);
          EEPROM.put(d1Oaddress, destination1Longitude);
        }
        while(Serial.available() > 0) {
          int a = Serial.read();
        } 
      }
}
*/
void checkGPS() {                                                               //check gps
//      #ifdef gpsDebug2
//      ring.setPixelColor(5, 255, 0, 0);
//      ring.show();
//      #endif
      
  c = GPS.read();
 // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    gpsUpdateMillis = millis();
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    
    
  }
//      #ifdef gpsDebug2
//      ring.setPixelColor(5, 0, 255, 0);
//      ring.show();
//      #endif
}

void pulseWave(int startPixel, //pixel the wave starts at
unsigned int Hue, //hue of the wave
byte Sat, //saturation of the wave
int rise, //rise amount per iteration
int fall, //fall amount per iteration, ratio of rise/fall determines how long the "tail" is on the wave
int wait) //wait time in between iterations
{
  //set up variables
  startPixel = constrain(startPixel,0,ringLength-1);
//  int colorMax = max(Red(color), Green(color));//figure out the max brightness of any color
//  colorMax = max(colorMax, Blue(color));
//  float redMult = (float)Red(color)/colorMax;//these multipliers allow color to stay consistent as brightness changes
//  float greenMult = (float)Green(color)/colorMax;
//  float blueMult = (float)Blue(color)/colorMax;
  int pixelMatrix[ringLength] ;//brightness of a pixel
  int pixelMatrixDirection[ringLength] ;//change in brightness of a pixel
    for(int i=0;i<ringLength;i++){ //clear variables after initialization
      pixelMatrix[i] = 0;
      pixelMatrixDirection[i]=0;
    }
  pixelMatrixDirection[startPixel] = rise;//gotta start the process
  pixelMatrix[startPixel] = 255;
  int stopPixel = (startPixel + ringLength/2)%ringLength;//find opposite side
  int stripAdvance = 0;
  int stripAdvanceOld = 0;

  while((pixelMatrix[stopPixel]>0 || pixelMatrixDirection[stopPixel]>-1)) {            //this loop actually does the pulse
    
      for(int a=0;a<ringLength;a++){           //increase or decrease the pixel brightness based on direction
          pixelMatrix[a] = pixelMatrix[a]+pixelMatrixDirection[a];
          if((pixelMatrixDirection[a]>0)&&pixelMatrix[a]>255){   //check if it's time to advance
            pixelMatrixDirection[a] = fall;
            stripAdvance++;
          }
        pixelMatrix[a]=constrain(pixelMatrix[a],0,255);//this keeps any weirdness out of the pixels
        //the multiplier is an easy way to make the color stay consistent while the brightness changes.
        ring.setPixelColor(a, ring.ColorHSV(Hue, Sat, pixelMatrix[a]));
      }

      
      if(stripAdvance > stripAdvanceOld) {                  //advance the wave
        //Serial.print("Advance: ");
        for(int u=0;u<ringLength;u++){//iterate through the pixels
          if(pixelMatrixDirection[u]==0) {//if pixel is untouched

             if(pixelMatrixDirection[(u+1)%(ringLength)]<0){
              pixelMatrixDirection[u]=rise;
             }
             if(pixelMatrixDirection[(u-1+ringLength)%(ringLength)]<0 && u!=stopPixel){
                pixelMatrixDirection[u]=rise;
             }

          }
          //Serial.print(pixelMatrixDirection[u]);
          //Serial.print(", ");
        }
        //Serial.println();
        stripAdvanceOld = stripAdvance;
      }
      
      ring.show();
      delay(wait);
    }
  
  //clear everything when done
  //ring.fill();
  ring.show();
}


    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
        return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
        return (color >> 8) & 0xFF;
    }

    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
        return color & 0xFF;
    }

// Return color, dimmed by 75% (used by scanner)
    uint32_t DimColor(uint32_t color)
    {
        uint32_t dimColor = ring.Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
        return dimColor;
    }
    
int getDirection(float latitude1, float longitude1, float latitude2, float longitude2) {
  float lat1 = toRadians(latitude1);
  float lat2 = toRadians(latitude2);
  float lng1 = toRadians(longitude1);
  float lng2 = toRadians(longitude2);
  float Y = sin(lng2 - lng1) * cos(lat2);
  float X = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lng2 - lng1);  
  float deg = toDegrees(atan2(Y, X));  
  // note that this implementation doesn't use the module, but angles lower than 0 get augmented by 360 only
  if (deg < 0) {
    deg = 360 + deg;
  }
  float angle = deg;
  int a = (int) (abs(angle) + (1 / 7200));
  return a;
}

float toRadians(float angle) {
  return (PI / 180) * angle;
}

float toDegrees(float rad) {
  return (rad * 180) / PI;
}
