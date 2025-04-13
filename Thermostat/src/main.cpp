#include <Arduino.h>
#include "secrets.h"
#include <EEPROM.h>
#include "sensors.h"
#include <bitset>
#include <cstring>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define LEDC_CHANNEL_BASE 0
#define LEDC_RESOLUTION 12 
#define LEDC_FREQUENCY 5000

#define EEPROM_SIZE 8

#define LEDAPIN 32
#define LEDBPIN 33
#define LEDCPIN 25
#define LEDDPIN 26
#define LEDEPIN 27
#define LEDFPIN 14
#define LEDGPIN 13
#define LEDPPIN 15
#define CONTROL1PIN 12
#define CONTROL2PIN 2
#define CONTROL3PIN 5
#define CONTROL4PIN 18
#define BUTTON1PIN 35
#define BUTTON2PIN 23
#define NTC_PIN 34
#define RELAYPIN 4

#define LDRPIN 36

#define NUMOUTPUTS 12

#define MAX_TEMP 30.0
#define MIN_TEMP 5.0

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

void display(void *pvParameters);
void buttons(void *pvParameters);
void LDR(void *pvParameters);
void turnOff(void *pvParameters);
void turnOn(void *pvParameters);
void relay(void *pvParameters);
void NTC(void *pvParameters);
void mqttLoopTask(void *pvParameters);

TaskHandle_t xDisplay;
TaskHandle_t xButtons;
TaskHandle_t xLDR;
TaskHandle_t xTurnOff;
TaskHandle_t xTurnOn;
TaskHandle_t xRelay;
TaskHandle_t xNTC;

SemaphoreHandle_t xMessageMutex;
SemaphoreHandle_t xSetPointMutex;

TaskHandle_t xSetPointMutexOwner = NULL;

uint8_t ledPins[9] = { LEDAPIN, LEDBPIN, LEDCPIN, LEDDPIN, LEDEPIN, LEDFPIN, LEDGPIN, LEDPPIN };
uint8_t controls[4] = {CONTROL1PIN, CONTROL2PIN, CONTROL3PIN, CONTROL4PIN};
float temp = 16.0;
unsigned int intensity = 4095;
unsigned int tempoVolta = 0;
unsigned int timerOff = 0;
float envTemp = 20.0;
bool isButtonActive = false;
unsigned int scrollIndex = 0;
bool status = true;
bool off = false;
bool on = false;


enum SevenSegmentChar {
    SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G, SEG_H, SEG_I, SEG_J,
    SEG_K, SEG_L, SEG_M, SEG_N, SEG_O, SEG_P, SEG_Q, SEG_R, SEG_S, SEG_T,
    SEG_U, SEG_V, SEG_W, SEG_X, SEG_Y, SEG_Z,
    SEG_COUNT 
};

uint8_t sevenSegmentDigits[10] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};
uint8_t sevenSegmentLetters[26] = {
  0b01110111, // A
  0b01111100, // B
  0b00111001, // C
  0b01011110, // D
  0b01111001, // E
  0b01110001, // F
  0b00111101, // G
  0b01110110, // H
  0b00010000, // I
  0b00011110, // J
  0b00111000, // K
  0b00111000, // L
  0b00110111, // M
  0b01010100, // N
  0b00111111, // O
  0b01110011, // P
  0b01100111, // Q
  0b01010000, // R
  0b01101101, // S
  0b01111000, // T
  0b00111110, // U
  0b00111110, // V
  0b00111110, // W
  0b00111110, // X
  0b01101110, // Y
  0b01011011  // Z
};
uint8_t message[4] = {0};
uint8_t scrollMessage[10] = {0};
uint8_t nullChar = 0b00000000;
uint8_t digits[3] = {0};

void getDigits(uint8_t *buff, float number) {
  int firstDigit = (number >= 10) ? (int)(number / 10) : 0;
  int secondDigit = (int)number % 10;
  int thirdDigit = (int)((number - (int)number) * 10);

  buff[0] = firstDigit;
  buff[1] = secondDigit;
  buff[2] = thirdDigit;
}

void setMessage(uint8_t char1, uint8_t char2, uint8_t char3, uint8_t char4){
  if(xSemaphoreTake(xMessageMutex, portMAX_DELAY) == pdTRUE){
    message[0] = char1;
    message[1] = char2;
    message[2] = char3;
    message[3] = char4;
    xSemaphoreGive(xMessageMutex);
  }
}

void updateSetPointDeviceShadow(char* buffer, size_t bufferSize, float setPoint){
  StaticJsonDocument<512> doc;
  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");
  JsonObject desired = state.createNestedObject("desired");
  reported["setPoint"] = setPoint;
  desired["setPoint"] = setPoint;
  serializeJson(doc, buffer, bufferSize);
}

void updateStatusDeviceShadow(char* buffer, size_t bufferSize, bool status){
  StaticJsonDocument<512> doc;
  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");
  JsonObject desired = state.createNestedObject("desired");
  if(status){
    reported["status"] = "On";
    desired["status"] = "On";
  }else{
    reported["status"] = "Off";
    desired["status"] = "Off";
  }
  
  serializeJson(doc, buffer, bufferSize);
}

void changeSetPoint(){
  EEPROM.put(0, temp);
  EEPROM.commit();

  char buffer[512];
  updateSetPointDeviceShadow(buffer, 512, temp);
  client.publish("$aws/things/testEsp/shadow/update", buffer);

  isButtonActive = false;
  setMessage(sevenSegmentLetters[SEG_S], sevenSegmentLetters[SEG_E], sevenSegmentLetters[SEG_T], nullChar);
  vTaskDelay(pdMS_TO_TICKS(2000));
  getDigits(digits, temp);
  uint8_t lastInt = sevenSegmentDigits[digits[1]];
  lastInt |= (1 << 7);
  setMessage(sevenSegmentDigits[digits[0]], lastInt, sevenSegmentDigits[digits[2]], sevenSegmentLetters[SEG_C]);
  vTaskDelay(pdMS_TO_TICKS(3000)); 
}

void messageHandler(char* topic, byte* payload, unsigned int length){
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.f_str());
    return;
  }

  if (doc.containsKey("state") && doc["state"].containsKey("setPoint")) {
    float newTemp = doc["state"]["setPoint"];
    Serial.print("New desired temperature: ");
    Serial.println(newTemp);
    if(xSemaphoreTake(xSetPointMutex, portMAX_DELAY) == pdTRUE){
      if(eTaskGetState(xNTC) == eRunning || eTaskGetState(xNTC) == eBlocked){
        vTaskSuspend(xNTC);
      }
      temp = newTemp;
      changeSetPoint();
      vTaskResume(xNTC);
      xSemaphoreGive(xSetPointMutex);
    }
  }

  if (doc.containsKey("state") && doc["state"].containsKey("status")) {
    if(doc["state"]["status"] == "Off" && status){
      off = true;
    }
    if(doc["state"]["status"] == "On" && !status){
      on = true;
    }
  }

}

void connectToWifi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("\nConnecting");

  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void connectToAWS(){
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);
 
  Serial.println("Connecting to AWS IOT");
  while (!client.connect(THINGNAME)){
    Serial.print(".");
    delay(100);
  }
  if (!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }
  client.loop();
  Serial.println("AWS IoT Connected!");
}

void resetControls(){
    digitalWrite(CONTROL1PIN, HIGH);
    digitalWrite(CONTROL2PIN, HIGH);
    digitalWrite(CONTROL3PIN, HIGH);
    digitalWrite(CONTROL4PIN, HIGH);
}

void showDisplay(uint8_t message[4]){
  for(int i = 0; i < 4;i++){
    for (int j = 0; j < 8; j++) {
      ledcWrite(LEDC_CHANNEL_BASE + j, ((message[i] >> j) & 1) * intensity);  
    }
    resetControls();
    digitalWrite(controls[i], LOW);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void resetLeds(){
  for (int i = 0; i < 7; i++) {
    ledcWrite(LEDC_CHANNEL_BASE + i, 0);  
  }
}

void scrollDisplay(uint8_t scrollMessage[], int tamanho){
  const int padding = 4;
  int totalSize = tamanho + padding * 2;
  uint8_t paddedScrollMessage[totalSize];
  memset(paddedScrollMessage, 0b00000000, totalSize);
  memcpy(&paddedScrollMessage[padding], scrollMessage, tamanho);

  if(xSemaphoreTake(xMessageMutex, portMAX_DELAY) == pdTRUE){
    while(scrollIndex < tamanho + 8){
      for(int i = 0;i < 30;i++){
        message[0] = paddedScrollMessage[scrollIndex];
        message[1] = paddedScrollMessage[scrollIndex + 1];
        message[2] = paddedScrollMessage[scrollIndex + 2];
        message[3] = paddedScrollMessage[scrollIndex + 3];
        showDisplay(message);
      }
      scrollIndex++;
    }
    scrollIndex = 0;
    xSemaphoreGive(xMessageMutex);
  }
}

void setup() {
  Serial.begin(115200);
  delay(10000);
  
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, temp);

  connectToWifi();
  connectToAWS();
  client.subscribe("$aws/things/testEsp/shadow/update/delta");

  if(isnan(temp) || temp < 5 || temp > 30){
    temp = 16;
  }
  for (int i = 0; i < NUMOUTPUTS - 4; i++) {
    ledcSetup(LEDC_CHANNEL_BASE + i, LEDC_FREQUENCY, LEDC_RESOLUTION);
    ledcAttachPin(ledPins[i], LEDC_CHANNEL_BASE + i);
  }
  pinMode(CONTROL1PIN, OUTPUT);
  pinMode(CONTROL2PIN, OUTPUT);
  pinMode(CONTROL3PIN, OUTPUT);
  pinMode(CONTROL4PIN, OUTPUT);
  pinMode(BUTTON1PIN, INPUT);
  pinMode(BUTTON2PIN, INPUT);
  pinMode(LDRPIN, INPUT);
  pinMode(RELAYPIN, OUTPUT);
  initSensors();

  xMessageMutex = xSemaphoreCreateMutex();
  xSetPointMutex = xSemaphoreCreateMutex();

  xTaskCreate(display, "Display", 2048, NULL, 0, &xDisplay);
  xTaskCreate(buttons, "Buttons", 3072, NULL, 0, &xButtons);
  xTaskCreate(LDR, "LDR", 2048, NULL, 1, &xLDR);
  xTaskCreate(turnOff, "TurnOff", 2048, NULL, 1, &xTurnOff);
  xTaskCreate(turnOn, "TurnOn", 2048, NULL, 1, &xTurnOn);
  vTaskSuspend(xTurnOn);
  xTaskCreate(relay, "Relay", 2048, NULL, 1, &xRelay);
  xTaskCreate(NTC, "NTC", 2048, NULL, 1, &xNTC);
  xTaskCreate(mqttLoopTask, "MQTTLoop", 3072, NULL, 1, NULL);
}

void loop() {}


void display(void *pvParameters) {
  uint8_t localMessage[4];
  while (1) {
    if(xSemaphoreTake(xMessageMutex, portMAX_DELAY) == pdTRUE){
      memcpy(localMessage, message, 4);
      xSemaphoreGive(xMessageMutex);
    }
    showDisplay(localMessage);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void buttons(void *pvParameters) {
  while(1){
    if(digitalRead(BUTTON1PIN) ^ digitalRead(BUTTON2PIN)){
      if(xSetPointMutexOwner == xButtons || xSemaphoreTake(xSetPointMutex, portMAX_DELAY) == pdTRUE){
        xSetPointMutexOwner = xTaskGetCurrentTaskHandle();
        isButtonActive = true;
        temp += (0.5 * digitalRead(BUTTON1PIN) + digitalRead(BUTTON2PIN) * -0.5);
        if(temp > MAX_TEMP){
          temp = 30.0;
        }
        if(temp < MIN_TEMP){
          temp = 5.0;
        }
        tempoVolta = millis();
        if(eTaskGetState(xNTC) == eRunning || eTaskGetState(xNTC) == eBlocked){
          vTaskSuspend(xNTC);
        }
        getDigits(digits, temp);
        uint8_t lastInt = sevenSegmentDigits[digits[1]];
        lastInt |= (1 << 7);
        setMessage(sevenSegmentDigits[digits[0]], lastInt, sevenSegmentDigits[digits[2]], sevenSegmentLetters[SEG_C]);
      }
    }    
    if(millis() - tempoVolta > 3000 && isButtonActive){
      changeSetPoint();
      xSemaphoreGive(xSetPointMutex);
      xSetPointMutexOwner = NULL;
      vTaskResume(xNTC);
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void LDR(void *pvParameters){
  while(1){
    intensity = analogRead(LDRPIN);
    if(intensity < 100){
      intensity = 100;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void turnOff(void *pvParameters){
  while(1){
    if((digitalRead(BUTTON1PIN) && digitalRead(BUTTON2PIN)) || off){
      if(millis() - timerOff > 3000){
        status = false;
        off = false;
        char buffer[512];
        updateStatusDeviceShadow(buffer, 512, false);
        if (client.connected()) {
          client.publish("$aws/things/testEsp/shadow/update", buffer);
        }
        vTaskSuspend(xNTC);
        vTaskSuspend(xButtons);
        vTaskSuspend(xLDR);
        vTaskSuspend(xRelay);
        setMessage(nullChar, sevenSegmentLetters[SEG_O], sevenSegmentLetters[SEG_F], sevenSegmentLetters[SEG_F]);
        vTaskDelay(pdMS_TO_TICKS(3000));
        vTaskSuspend(xDisplay);
        vTaskResume(xTurnOn);
        resetControls();
        resetLeds();

        timerOff = millis();
        vTaskSuspend(NULL);
      }
    }else{
      timerOff = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void turnOn(void *pvParameters){
  while(1){
    if((digitalRead(BUTTON1PIN) && digitalRead(BUTTON2PIN)) || on){
      if(millis() - timerOff > 3000){
        on = false;
        status = true;
        char buffer[512];
        updateStatusDeviceShadow(buffer, 512, true);
        client.publish("$aws/things/testEsp/shadow/update", buffer);

        setMessage(nullChar, sevenSegmentLetters[SEG_O], sevenSegmentLetters[SEG_N], nullChar);
        vTaskDelay(pdMS_TO_TICKS(10));
        vTaskResume(xDisplay);
        vTaskDelay(pdMS_TO_TICKS(1000));

        scrollMessage[0] = sevenSegmentLetters[SEG_H];
        scrollMessage[1] = sevenSegmentLetters[SEG_F];
        scrollMessage[2] = sevenSegmentDigits[1];
        scrollMessage[3] = sevenSegmentDigits[0];
        scrollMessage[4] = sevenSegmentDigits[0];
        scrollMessage[5] = sevenSegmentDigits[0];
        scrollDisplay(scrollMessage, 6);

        timerOff = millis();

        vTaskResume(xButtons);
        vTaskResume(xLDR);
        vTaskResume(xRelay);
        vTaskResume(xTurnOff);
        vTaskResume(xNTC);
        vTaskSuspend(NULL);
      }
    }else{
      timerOff = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void relay(void *pvParameters){
  while(1){
    if(temp > envTemp){
      digitalWrite(RELAYPIN, HIGH);
    }else{
      digitalWrite(RELAYPIN, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void NTC(void *pvParameters){
  while(1){
    envTemp = getSensorTemperature(false);
    getDigits(digits, envTemp);
    uint8_t lastInt = sevenSegmentDigits[digits[1]];
    lastInt |= (1 << 7);
    setMessage(sevenSegmentDigits[digits[0]], lastInt, sevenSegmentDigits[digits[2]], sevenSegmentLetters[SEG_C]);
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void mqttLoopTask(void *pvParameters) {
  while (1) {
    client.loop(); 
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}
