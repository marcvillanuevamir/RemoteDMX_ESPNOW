/*
  ESP-NOW Output - Remote DMX controller
  Se'ns moren les plantes
  Setembre 2024 - FiraTàrrega

  Hardware: Battery-powered SparkFun ESP32 Thing Plus + 1 button + 2 knobs
  Power issues:
  I am using a 3,7V 850mAh LIPO battery, which seems not to provide enough 
  current for the wifi sparks. Therefore, I am disabling brown out detection
  but the device keeps shutting down sometimes. A battery that can provide
  at least 2500mAh should be tested to ensure the stability of the system.
*/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <debounce.h>  // https://github.com/kimballa/button-debounce

// for disabling brown out detection
#include "soc/soc.h"
#include "soc/rtc.h"

// SERIAL ON/OFF
// I toggle serial debugging to save energy when running on battery
#define SERIAL_ON false
#define SERIAL \
  if (SERIAL_ON) Serial

// PINS
const int buttonPin = 12;   // digital GPIO
const int sensorPin = A4;   // ADC1 (ADC2 can not be used, since it is in use by Wifi)
const int sensorPin2 = A3;  // ADC1
const int ledPin = 13;      // digital GPIO

// VARIABLES
uint8_t buttonState = 1;
uint8_t lastButtonState = 1;
uint8_t sensorValue = 0;
uint8_t sensorValue2 = 0;
uint8_t lastSensorValue;
uint8_t lastSensorValue2;

// ANALOG SMOOTHING
const int numReadings = 10;
int readings[numReadings];   // the readings from the knob 1
int readings2[numReadings];  // the readings from the knob 2
int readIndex = 0;
int total = 0;
int readIndex2 = 0;
int total2 = 0;

// ESP-NOW
// Mac Address
uint8_t broadcastAddress[] = { 0xa0, 0xb7, 0x65, 0x60, 0x1c, 0xe8 };  // My ESP32 Thing Plus (redefine if other hardware is used)
// Message structure
typedef struct struct_message {
  int a;  // button data
  int b;  // sensor 1 data
  int c;  // sensor 2 data
} struct_message;
struct_message myData;  // Create a struct_message called myData
esp_now_peer_info_t peerInfo;
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  SERIAL.print("\r\nLast Packet Send Status:\t");
  SERIAL.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// ESP-NOW MESSAGES
const int idle = 0;
const int nextScene = 10;
const int previousScene = 20;
int sceneManagement = idle;
int lastTask;

// BUTTON DEBOUNCING
bool useShort;
// Short press button
static void shortPressHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    useShort = true;
    analogWrite(ledPin, 0);
  } else {
    // btnState == BTN_OPEN.
    if (useShort == true) {
      SERIAL.println("Next scene");
      sceneManagement = nextScene;
      analogWrite(ledPin, 200);
      return;
    }
  }
}
// Long press button
static void longPressHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    useShort = false;
    blinkLed(100);
  } else {
    // btnState == BTN_OPEN
    // Send a message after it has been held down a long time and released
    if (useShort == false) {
      SERIAL.println("Previous scene");
      sceneManagement = previousScene;
      analogWrite(ledPin, 200);
      return;
    }
  }
}
// Button definitions
static Button shortBtn(0, shortPressHandler);
static Button longBtn(1, longPressHandler);

void setup() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // disable brown out detection

#if SERIAL_ON == true
  SERIAL.begin(115200);
  SERIAL.println("ESP-NOW controller. Initialising...");
#endif

  // GPIO setup
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(sensorPin, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(ledPin, OUTPUT);
  analogReadResolution(8);
  longBtn.setPushDebounceInterval(1000);  // 1 second
  analogWrite(ledPin, 200);

  // WiFi setup
  WiFi.mode(WIFI_STA);
  delay(50);

  // ESP-NOW setup
  if (esp_now_init() != ESP_OK) {
    SERIAL.println("Error initializing ESP-NOW");
    return;
  } else if (esp_now_init() == ESP_OK) {
    blinkLed(50);
  }
  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  delay(50);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    SERIAL.println("Failed to add peer");
    //analogWrite(ledPin, 0);
    return;
  } else if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    blinkLed(50);
  }
  analogWrite(ledPin, 200);
  SERIAL.println("Ready");
  delay(50);
}

// function to read the button states
static void pollButtons() {
  uint8_t pinState = digitalRead(buttonPin);
  shortBtn.update(pinState);
  longBtn.update(pinState);
}

// function to smooth analog input
uint8_t smoothing(int pin) {
  if (pin == sensorPin) {
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = analogRead(pin);
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex++;
    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }
    // calculate the average:
    uint8_t average = total / numReadings;
    delay(5);
    return average;
  } else if (pin == sensorPin2) {
    total2 = total2 - readings2[readIndex2];
    readings2[readIndex2] = analogRead(pin);
    total2 = total2 + readings2[readIndex2];
    readIndex2++;
    if (readIndex2 >= numReadings) {
      readIndex2 = 0;
    }
    uint8_t average2 = total2 / numReadings;
    delay(5);
    return average2;
  }
}

// function to blink a debugging led
void blinkLed(int delayTime) {
  for (int i = 0; i < 4; i++) {
    if (i == 0 || i == 2) {
      analogWrite(ledPin, 200);
    } else if (i == 1 || i == 3) {
      analogWrite(ledPin, 0);
    }
    delay(delayTime);
  }
}

void loop() {

  // Read the button state and write the message:
  pollButtons();
  if (sceneManagement != lastTask) {
    myData.a = sceneManagement;
    lastTask = sceneManagement;
  } else {
    sceneManagement = idle;
    myData.a = sceneManagement;
  }

  // Read and smooth the values from the sensors:
  sensorValue = smoothing(sensorPin);
  sensorValue2 = smoothing(sensorPin2);

  if (sensorValue != lastSensorValue) {
    //SERIAL.print("Sensor value 1: ");
    //SERIAL.println(sensorValue);
    myData.b = sensorValue;
  }
  lastSensorValue = sensorValue;

  if (sensorValue2 != lastSensorValue2) {
    //SERIAL.print("Sensor value 2: ");
    //SERIAL.println(sensorValue2);
    myData.c = sensorValue2;
  }
  lastSensorValue2 = sensorValue2;

  // Print sent struct message
  SERIAL.print(myData.a);
  SERIAL.print("  ");
  SERIAL.print(myData.b);
  SERIAL.print("  ");
  SERIAL.println(myData.c);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  // Feedback for ESP-NOW message
  if (result == ESP_OK) {
    SERIAL.println("Sent with success");
  } else {
    SERIAL.println("Error sending the data");
  }
  delay(50);
}
