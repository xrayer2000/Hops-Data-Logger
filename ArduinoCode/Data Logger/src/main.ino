#include <Arduino.h>
#include <BH1750.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <WiFi.h> //Home assistant
#include <PubSubClient.h> //Home assistant
#include "Privates.h" //Homeassistant 

BH1750 lightMeter;
BME280I2C bme;  
float temp(NAN), hum(NAN), pres(NAN), lux(NAN);

unsigned long previousTime = 0; 
unsigned long passedTime = 0;

//Homeassistant
//-----------------------------------------------------------------------
Privates privates; 
WiFiClient espClient; 
PubSubClient client(espClient); 
char messages[50]; 
volatile  bool TopicArrived = false;
const     int mqttpayloadSize = 10;
char mqttpayload [mqttpayloadSize] = {'\0'};
String mqtttopic;
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long mqttReconnectInterval = 5000; // 5 seconds
//-----------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  while(!Serial) {} 
  Wire.begin();
  lightMeter.begin();

  while(!bme.begin()){Serial.println("Could not find BME280 sensor!"); delay(1000);}

  setupWiFi(); //Home assistant
  client.setServer(privates.broker, 1883); //Home assistant
  client.setCallback(callback);
}

void loop()
{
  printBME280Data();
  //Homeassistant
  passedTime = millis() / 1000.0;
  if(passedTime - previousTime > 1.0)
  {
    publishMessage();
    previousTime = passedTime;
  }
}

void printBME280Data()
{
  
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);
  lux = lightMeter.readLightLevel();

  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
  Serial.print("\tHumidity: ");
  Serial.print(hum);
  Serial.print("% RH");
  Serial.print("\tPressure: ");
  Serial.print(pres);
  Serial.print("Pa, ");
  Serial.print("\tLight: ");
  Serial.print(lux);
  Serial.println(" lx");

   delay(1000);
}

void setupWiFi() //Homeassistant
{
  WiFi.begin(privates.ssid, privates.pass);
  Serial.print("\nConnecting to ");
  Serial.print(privates.ssid);
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  Serial.print("\nConnected to ");
  Serial.println(privates.ssid);
}

void reconnect() //Homeassistant
{
  // Only try to reconnect if enough time has passed
  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt > mqttReconnectInterval) {
    lastMqttReconnectAttempt = now;
    if (!client.connected()) {
      client.connect("boll", privates.brokerUser, privates.brokerPass);
    }
  }
}

void publishMessage() //Homeassistant
{
  if(!client.connected()){reconnect();}
  client.loop();
  //int "%d", float "%f", bool "%s", obs vikitgt!
  snprintf(messages, sizeof(messages), "%f", temp);
  client.publish(privates.topicTemp, messages);
  snprintf(messages, sizeof(messages), "%f", hum);
  client.publish(privates.topicHum, messages);
  snprintf(messages, sizeof(messages), "%f", pres); 
  client.publish(privates.topicPre, messages);
  snprintf(messages, sizeof(messages), "%f", lux);
  client.publish(privates.topicLux, messages);
}

void callback(char* topic, byte* payload, unsigned int length) //Homeassistant
{
  if ( !TopicArrived )
  {
    memset( mqttpayload, '\0', mqttpayloadSize ); // clear payload char buffer
    mqtttopic = ""; //clear topic string buffer
    mqtttopic = topic; //store new topic
    memcpy( mqttpayload, payload, length );
    TopicArrived = true;
  }
}

