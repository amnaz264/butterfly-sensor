#include <WiFi.h>
#include <ArduinoMqttClient.h> // https://github.com/arduino-libraries/ArduinoMqttClient

#include <Ultrasonic.h> // https://github.com/ErickSimoes/Ultrasonic
#include <ESP32Servo.h>
#include "secrets.h"

/*
* Pass as a parameter the trigger and echo pin, respectively,
* or only the signal pin (for sensors 3 pins), like:
* Ultrasonic ultrasonic(13);
*/
// wifi and mqtt clients
WiFiClient wifiClient;  // part of wifi.h
MqttClient mqttClient(wifiClient);

String sendMessage="";

#include <Glue.h>
Glue elmers;
String distanceTopic= "amna_butterfly_distance";

Servo myservoWhite;// create servo object to control a servo
// 16 servo objects can be created on the ESP32
int pos = 0;    // variable to store the servo position
int servoPinWhite = 14;

Ultrasonic ultrasonic(27, 33);  // trg = 27, echo = 33
int distance;

// send timer -- we should listen more than we send
unsigned long sendInterval = 500;
unsigned long sendStartTime = 0;
unsigned long currentTime = 0;

void setup() {


  pinMode(27,OUTPUT);
  pinMode(33,INPUT);
  Serial.begin(9600);

  elmers.begin();  // not attaching it to a stream

  initWiFi();

  if ( WiFi.status() == WL_CONNECTED)  { // if wifi connects attempt to connect to shiftr
    initMqtt();
  }

  // Allow allocation of all timers - not sure why
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);


  myservoWhite.setPeriodHertz(50); // standard 50 hz servo
  myservoWhite.attach(servoPinWhite, 500, 3000);// attaches the servo on pin 14

  // NOTE -- using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
}



void loop() {

  currentTime = millis();
  // Pass INC as a parameter to get the distance in inches
  distance = ultrasonic.read();
  Serial.println(distance);

  if (distance < 60 ){
    for (pos =0;pos<=180;pos ++)
    {
    myservoWhite.write(pos);
    delay(1);
  }
  delay(100);
  for (pos =180;pos>=0;pos --)
  {
    myservoWhite.write(pos);
    delay(1);
  }
  delay(100);

  if ((currentTime - sendStartTime) > sendInterval){
    buildDistanceMessage(distance);
    sendStartTime = currentTime;
    //debugSensors();
  }

}
}
void buildDistanceMessage(int value){
  // use elmers to add the parsing syntax around value -- [*,#]
  elmers.create();
  elmers.add(value);
  elmers.endPackage();
  String payload = elmers.getPackage();

  publishMqttMessage(distanceTopic, payload);
}
