#include <Arduino.h>
#include <BH1750.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <WiFi.h> //Home assistant
#include <PubSubClient.h> //Home assistant
#include "Privates.h" //Homeassistant 
#include <PressButton.h> //Interface
#include <RotaryEncoderAccel.h> //Interface
#include <U8g2lib.h> //oled
#include <EEPROM.h> //Save Settings
#include <math.h>
//-----------------------------------------------------------------------
// OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//-----------------------------------------------------------------------
#define SDA_PIN 21
#define SCL_PIN 22
#define confirmBtnPin 18 //input digitalRead()
//#define ledOnPin 33 //output digitalWrite()
#define pumpPin 4
#define moisturePin 36
//rotary encoder
#define outputA 16 //input digitalRead()
#define outputB 17 //input digitalRead()
//-----------------------------------------------------------------------
//Settings
#define DISP_ITEM_ROWS 4
#define DISP_CHAR_WIDTH 16
#define PACING_MC 30 //25
#define FLASH_RST_CNT 3 //30
#define SETTINGS_CHKVAL 3647 
#define CHAR_X SCREEN_WIDTH/DISP_CHAR_WIDTH // 128/16
#define CHAR_Y SCREEN_HEIGHT/DISP_ITEM_ROWS // 64/4
#define SHIFT_UP 0 //240/8
//-----------------------------------------------------------------------
//Varibles
BH1750 lightMeter;
BME280I2C bme;  
float temp(NAN), hum(NAN), pres(NAN), lux(NAN);
float moisture = 50.0;
int newMoisture = 0;
int rawMoisture = 0;
bool pump_status = false;
bool vattna = false;

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
//rotary encoder
RotaryEncoderAccel encoder(outputA, outputB);  // GPIO16 och GPIO17 på ESP32
// ISR för båda pins, som anropas vid ändring (rising/falling)
void IRAM_ATTR handleInterrupt() {
  encoder.tick();
}
//-----------------------------------------------------------------------
//Buttons
PressButton btnOk(confirmBtnPin);
//-----------------------------------------------------------------------
//Menu structure
enum pageType{
  MENU_ROOT,
};
enum pageType currPage = MENU_ROOT;
void page_MenuRoot();
//-----------------------------------------------------------------------
//Menu internals
boolean updateAllItems = true;
boolean updateItemValue;
uint8_t itemCnt;
int8_t cursorPos;
uint8_t saveCursorPos;
uint8_t dispOffset;
uint8_t saveDispOffset;
bool edditing = false;
bool detectedRotation = false;
uint8_t root_pntrPos = 1;
uint8_t root_dispOffset = 0;
uint8_t flashCntr;
boolean flashIsOn;
void initMenuPage(String title, uint8_t itemCount);
void captureButtonDownStates();
void incrementDecrementFloat(float *v, float amount, float min, float max);
void incrementDecrementDouble(double *v, double amount, double min, double max);
void doPointerNavigation();
bool menuItemPrintable(uint8_t xPos, uint8_t yPos);
//-----------------------------------------------------------------------
//Print tools
void printPointer();
void printOnOff(bool val);
void printUint32_tAtWidth(uint32_t value, uint8_t width, char c);
void printDoubleAtWidth(double value, uint8_t width, char c);
//-----------------------------------------------------------------------
//Settings

#pragma pack(1) //memory alignment
struct Mysettings{
  uint16_t targetMoisture = 70;
  boolean power = true;
  uint16_t dryMoistureCalli = 3200;
  uint16_t wetMoistureCalli = 1200;
  uint16_t overshoot = 3;
  uint16_t filterWeight = 95;
  uint16_t settingsCheckValue = SETTINGS_CHKVAL;
};

Mysettings settings;
Mysettings oldSettings;
void sets_SetDeafault();
void sets_Load();
void sets_Save();
//-----------------------------------------------------------------------
//Time
unsigned long previousTime = 0; 
unsigned long passedTime = 0;
unsigned long previousPassedTime1,previousPassedTime2 = 0;
unsigned long currentTime;
unsigned long previousSensorRead = 0;
int readInterval = 500;
const unsigned long saveInterval = 60000; // 60 000 ms = 60 sekunder
unsigned long lastEditTime = 0;
//-----------------------------------------------------------------------
// Oled - Adafruit_SSD1306
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
long timeLastTouched = 0;
const long timeBeforeDisable = 120000;
//-----------------------------------------------------------------------
bool initPage = true;
bool changeValue = false;
bool changeValues [10];
//-----------------------------------------------------------------------
//BME280
BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);
//-----------------------------------------------------------------------

void setup()
{
  //=================================================SETUP=======================================================
  Serial.begin(115200);
  while(!Serial) {} 
  Wire.begin(SDA_PIN, SCL_PIN);  // Initierar I2C på valfria pinnar
  lightMeter.begin();

  while(!bme.begin()){Serial.println("Could not find BME280 sensor!"); delay(1000);}

  currentTime = millis();

  setupWiFi(); //Home assistant
  client.setServer(privates.broker, 1883); //Home assistant
  client.setCallback(callback);

  pinMode(pumpPin, OUTPUT);
  pinMode(moisturePin, INPUT);

  pinMode(confirmBtnPin, INPUT_PULLDOWN);
  //pinMode(ledOnPin, OUTPUT);
  pinMode(outputA,INPUT_PULLUP);
  pinMode(outputB,INPUT_PULLUP);

  // Koppla interrupt på båda encoder-pins till samma ISR
  attachInterrupt(digitalPinToInterrupt(outputA), handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), handleInterrupt, CHANGE);

  Wire.begin();
  Wire.setClock(3400000 );      //3.4 MHz
  display.setBusClock(3400000 );  //3.4 MHz

  display.setI2CAddress(0x78); 
  display.begin();  
  display.setFont(u8g2_font_6x10_mf);	// choose a suitable font
  display.clearBuffer();					// clear the internal memory
  display.setCursor(0, 0);

  EEPROM.begin(sizeof(settings));
  sets_Load();
  oldSettings = settings;
}

void loop()
{
   //=================================================LOOP=======================================================
  passedTime = millis() / 1000.0;

  //print sensor data
  //printSensorData();

  if(millis() - timeLastTouched > timeBeforeDisable)
  {
    display.setPowerSave(1);
    display.setPowerSave(1);
  }
  else
  {
    display.setPowerSave(0);
    display.setPowerSave(0);
  }

  const float UPDATE_INTERVAL1 = 0.25;
  
  bool shouldUpdate1 = (passedTime - previousPassedTime1 >= UPDATE_INTERVAL1);
  bool shouldUpdate2 = (passedTime - previousPassedTime2 >= 0.01);
  updateSettings();

  if(shouldUpdate1)
  {
    updateSensorValues(); 
    previousPassedTime1 = passedTime;
  }

  page_MenuRoot();
  
  printPointer();         
  // Serial.print(updateAllItems);
  // Serial.print(" ");
  // Serial.println(updateItemValue);

  if ((updateAllItems || updateItemValue) && shouldUpdate2)
  {
    //Serial.println("Sending display buffer");
    display.sendBuffer();                        //välldigt långsam
    previousPassedTime2 = passedTime;
    updateAllItems = false;
    updateItemValue = false;
  } 
  //Capture button states
  captureButtonDownStates();

  //Homeassistant
  if(passedTime - previousTime > 1.0) {publishMessage(); previousTime = passedTime;}

  detectedRotation = encoder.getDirection() != RotaryEncoderAccel::Direction::NOROTATION;
  if(detectedRotation)
  {
    lastEditTime = millis();
  }
    
  //Save settings every 10 seconds if not rotated the encoder
  if (millis() - lastEditTime >= saveInterval) {
    sets_Save();
    lastEditTime = millis();
  }
}

void page_MenuRoot()
{
  //=================================================ROOT_MENU============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    changeValues[10];
    initMenuPage(F("MAIN MENU"), 8);
    initPage = false;
  }

  if(menuItemPrintable(1,1)){display.print(F("POWER              "));}
  if(menuItemPrintable(1,2)){display.print(F("TARGET MOIST       "));}
  if(menuItemPrintable(1,3)){display.print(F("SOIL MOISTURE      "));}
  if(menuItemPrintable(1,4)){display.print(F("RAW SOIL MOIST     "));}
  if(menuItemPrintable(1,5)){display.print(F("DRY RAW CALLIB     "));}
  if(menuItemPrintable(1,6)){display.print(F("WET RAW CALLIB     "));}
  if(menuItemPrintable(1,7)){display.print(F("FILTER WEIGHT      "));}
  if(menuItemPrintable(1,8)){display.print(F("HYSTERESIS         "));}

  if(menuItemPrintable(13,1)){printOnOff(settings.power);}
  if(menuItemPrintable(13,2)){printUint32_tAtWidth(settings.targetMoisture, 5, '%');}
  if(menuItemPrintable(13,3)){printUint32_tAtWidth(moisture, 5, '%');}
  if(menuItemPrintable(13,4)){printUint32_tAtWidth(rawMoisture, 5, '%');}
  if(menuItemPrintable(13,5)){printUint32_tAtWidth(settings.dryMoistureCalli, 5, ' ');}
  if(menuItemPrintable(13,6)){printUint32_tAtWidth(settings.wetMoistureCalli, 5, ' ');}
  if(menuItemPrintable(13,7)){printUint32_tAtWidth(settings.filterWeight, 5, ' ');}
  if(menuItemPrintable(13,8)){printUint32_tAtWidth(settings.overshoot, 5, ' ');}
      
  if(btnOk.PressReleased())
  {
    FlashPointer();
    switch (cursorPos)
    {
      case 0: changeValues [0] = !changeValues [0]; edditing = !edditing; break; 
      case 1: changeValues [1] = !changeValues [1]; edditing = !edditing; break; 
      case 4: changeValues [4] = !changeValues [4]; edditing = !edditing; break; 
      case 5: changeValues [5] = !changeValues [5]; edditing = !edditing; break;
      case 6: changeValues [6] = !changeValues [6]; edditing = !edditing; break;
      case 7: changeValues [7] = !changeValues [7]; edditing = !edditing; break;
    }

    if(edditing) // If edditing is true, save the current cursor position and display offset
    {   
      // Save current cursor position and display offset
      // and set the encoder position to the current cursor position
      saveCursorPos = encoder.getPosition();
      saveDispOffset = dispOffset;

      encoder.setAccel(150, 5);
    }
    else // If edditing is false, restore the cursor position and display offset
    {
      encoder.setPosition(saveCursorPos);
      dispOffset = saveDispOffset; 
    }
  }
       if(changeValues[0]){*&settings.power = !*&settings.power; changeValues [0] = false; updateItemValue = true;}
  else if(changeValues[1])incrementDecrementInt(&settings.targetMoisture, 1, 0, 100);
  else if(changeValues[4])incrementDecrementInt(&settings.dryMoistureCalli, 1, 2800, 3400);
  else if(changeValues[5])incrementDecrementInt(&settings.wetMoistureCalli, 1, 600, 1300);
  else if(changeValues[6])incrementDecrementInt(&settings.filterWeight, 1, 0, 100);
  else if(changeValues[7])incrementDecrementInt(&settings.overshoot, 1, 0, 5);
  else 
    doPointerNavigation(); 
}

void printSensorData()
{
  Serial.print("\tTemp: ");
  Serial.print(temp);
  Serial.print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
  Serial.print("\tHumidity: ");
  Serial.print(hum);
  Serial.print("% RH");
  Serial.print("\tPressure: ");
  Serial.print(pres);
  Serial.print(" Pa");
  Serial.print("\tLight: ");
  Serial.print(lux);
  Serial.print(" Lx");
  Serial.print("\t\tMoisture: ");
  Serial.print(moisture);
  Serial.println("%");
}

//======================================================TOOLS - menu Internals==================================================
void initMenuPage(String title, uint8_t itemCount){
  display.clearBuffer();
  printPointer();
  uint8_t fillCnt = (DISP_CHAR_WIDTH - title.length()) / 2;

  btnOk.ClearWasDown();
 
  itemCnt = itemCount;
  flashCntr = 0;
  flashIsOn = false;
  updateAllItems = true;
}
void captureButtonDownStates(){
  btnOk.CaptureDownState();
}

void doPointerNavigation() 
{
  int direction = round(encoder.getRPM());

  if (direction != 0) {
    int newCursorPos = cursorPos + direction;

    // Clamp
    if (newCursorPos < 0) newCursorPos = 0;
    if (newCursorPos > itemCnt - 1) newCursorPos = itemCnt - 1;

    if (newCursorPos != cursorPos) {
      cursorPos = newCursorPos;

      // Scroll logic
      if (cursorPos < dispOffset) {
        dispOffset = cursorPos;
        updateAllItems = true; // Force update of all items
      } else if (cursorPos >= dispOffset + DISP_ITEM_ROWS) {
        dispOffset = cursorPos - DISP_ITEM_ROWS + 1;
        updateAllItems = true; // Force update of all items
      }

      printPointer();  // Only redraw when view actually changes
      timeLastTouched = millis(); // Update last touched time
    }
  }
}

void incrementDecrementInt(uint16_t *v, uint16_t amount, uint16_t min, uint16_t max)
{
    int direction = encoder.getRPM();

    if (direction != 0) {
        int target = direction * amount;
        int newValue = *v + target;

        if (newValue >= min && newValue <= max) {
            *v = newValue;
        }
        else if (newValue < min) {
            *v = min;
        }
        else {
            *v = max;
        }

        updateItemValue = true;
        timeLastTouched = millis();
    }

    delayMicroseconds(5);
}


void incrementDecrementFloat(float *v, float amount, float min, float max)
{
  int enc = encoder.getPosition();
  if(enc != 0) {
    *v += (enc*amount);
    *v = constrain(*v,min,max);
    //Serial.println(enc*amount);
    updateItemValue = true;
  } 
  delayMicroseconds(5);
}
void incrementDecrementDouble(double *v, double amount, double min, double max)
{
  int enc = encoder.getPosition();
  if(enc != 0) {
    *v += (enc*amount);
    *v = constrain(*v,min,max);
    //Serial.println(enc*amount);
    updateItemValue = true;
  } 
  delayMicroseconds(5);
}

bool isFlashChanged(){
  if(flashCntr == 0){
    flashIsOn = !flashIsOn;
    flashCntr = FLASH_RST_CNT;
    return true;
  }
  else{flashCntr--; return false;}
}

bool menuItemPrintable(uint8_t xPos, uint8_t yPos){
  if(!(updateAllItems || (updateItemValue && cursorPos == yPos))){return false;}
  uint8_t yMaxOffset = 0;
  if(yPos > DISP_ITEM_ROWS) {yMaxOffset = yPos - DISP_ITEM_ROWS;}
  if(dispOffset <= (yPos) && dispOffset >= yMaxOffset){display.setCursor(CHAR_X*xPos, CHAR_Y*(yPos - dispOffset)); return true;}
  return false;
}

//======================================================TOOLS_display========================================================
void printPointer(){
  //Serial.println("printPointer");
  display.drawStr(0, 1*CHAR_Y, " ");
  display.drawStr(0, 2*CHAR_Y, " ");
  display.drawStr(0, 3*CHAR_Y, " ");
  display.drawStr(0, 4*CHAR_Y, " ");
  display.drawStr(0, (cursorPos - dispOffset + 1)*CHAR_Y, "*");
  display.sendBuffer();
}
void FlashPointer(){
  timeLastTouched = millis();
  display.drawStr(0, 1*CHAR_Y, " ");
  display.drawStr(0, 2*CHAR_Y, " ");
  display.drawStr(0, 3*CHAR_Y, " ");
  display.drawStr(0, 4*CHAR_Y, " ");
  display.sendBuffer();                  //nytt

  delay(100);
  //Serial.println("FlashPointer");
  display.drawStr(0, (cursorPos - dispOffset + 1)*CHAR_Y, "*");
  display.sendBuffer();                  //nytt
}

void printOnOff(bool val){
  if(val){display.print(F("ON    "));}
  else   {display.print(F("OFF   "));}
}
void printChars(uint8_t cnt, char c){
  if(cnt > 0){
    char cc[] = " "; cc[0] = c;
    for(u_int8_t i = 1; i < cnt; i++){display.print(cc);}
  }
}
uint8_t getUint32_tCharCnt(uint32_t value)
{
  if(value == 0){return 1;}
  uint32_t tensCalc = 10; int8_t cnt = 1;
  while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
  return cnt;
}
uint8_t getDoubleCharCnt(double value)
{
  if(value == 0){return 1;}
  uint32_t tensCalc = 10; int8_t cnt = 1;
  while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
  return cnt;
}
void printUint32_tAtWidth(uint32_t value, uint8_t width, char c){
  display.print(value);
  display.print(c);
  printChars(width-getUint32_tCharCnt(value), ' ');
}
void printDoubleAtWidth(double value, uint8_t width, char c){
  char buf[10];
  dtostrf(value, width-getDoubleCharCnt(value), 1, buf); // 1 decimal
  display.print(buf);
  display.print(c);
}
//======================================================TOOLS_settings======================================================
void sets_SetDeafault()
{
  Mysettings tempSets;
  memcpy(&settings, &tempSets, sizeof settings);
}

void sets_Load()
{
  EEPROM.get(0,settings);
  if(settings.settingsCheckValue != SETTINGS_CHKVAL){sets_SetDeafault();}
}
void sets_Save()
{
  if (memcmp(&settings, &oldSettings, sizeof(settings)) != 0) {
    EEPROM.put(0, settings);
    EEPROM.commit();
    oldSettings = settings; // uppdatera efter commit
  }
}


void updateSettings()
{
  if(settings.power)
  {
    //Pump control
    if (moisture < settings.targetMoisture - settings.overshoot) 
    { //
      vattna = true;
      pump_status = true;
      //Serial.print("Pump water ON:  ");
      digitalWrite(pumpPin, HIGH);
      delay(1);
    }
    else if (moisture > settings.targetMoisture + settings.overshoot) {
      vattna = false;
      pump_status = false;
      //Serial.print("Pump water OFF:  ");
      digitalWrite(pumpPin, LOW);
    }
    else {
      pump_status = vattna;
      //Serial.print("Pump water ");
      // if (vattna) Serial.print("ON:  ");
      // else Serial.print("OFF:  ");
      digitalWrite(pumpPin, vattna);
      //Serial.print("Pump water vattna:  ");
    }
    //------------------------------------------------------------------------
    updateAllItems = true;

    client.loop();
    if(TopicArrived)
    {
      // Serial.print("Message arrived: ");
      // Serial.println(mqttpayload);
      //settings.targetTemp = atof(mqttpayload);
      TopicArrived = false;
    }
  }
  else
  {
    //digitalWrite(ledOnPin, LOW);
  }
}

void updateSensorValues() {

  unsigned long currentTime = millis();
  if (currentTime - previousSensorRead >= readInterval)
  {
    previousSensorRead = currentTime;

    bool result = bme.read(pres, temp, hum, tempUnit, presUnit);
    lux = lightMeter.readLightLevel();

    // If sensor read fails, try to re-initialize
    if (!result || lux < 0) 
    {
      Serial.println("I2C sensor error, re-initializing...");
      Wire.end();
      delay(100);
      Wire.begin(SDA_PIN, SCL_PIN);
      bme.begin();
      lightMeter.begin();
      delay(100);
      return;
    }

    rawMoisture = analogRead(moisturePin);
    newMoisture = mapFloat(rawMoisture, settings.dryMoistureCalli, settings.wetMoistureCalli, 0.0, 100.0); // Map the analog value to percentage
    newMoisture = constrain(newMoisture, 0, 100); // Ensure moisture is within 0-100%
    moisture = Filter(newMoisture, moisture); // Apply filter to smooth the value
  }
}

float Filter(float New, float Current) //Moisture sensor
{
  return (1.0 - settings.filterWeight/100.0) * New + settings.filterWeight/100.0 * Current;
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
  snprintf(messages, sizeof(messages), "%f", moisture);
  client.publish(privates.topicMoisture, messages);
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

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


