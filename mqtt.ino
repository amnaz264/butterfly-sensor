// note mqttClient is globally defined in main tab

void initMqtt(){
 // https://github.com/arduino-libraries/ArduinoMqttClient/blob/master/examples/WiFiSimpleSender/WiFiSimpleSender.ino

  mqttClient.setId(id); // device name
  mqttClient.setUsernamePassword(user, mqttPW);

  Serial.print("\nconnecting to shiftr...");
  while (!mqttClient.connect(broker,port)) {
    Serial.print(".");
    delay(500);
  }

  Serial.print("\nconnected to shiftr @");
  Serial.println(user); // instacne name
} // end initMqtt


// generic huzzah32 MQTT send function
void publishMqttMessage(String topicString, String payload){
   mqttClient.beginMessage(topicString);
   mqttClient.print(payload);
   mqttClient.endMessage();
}
