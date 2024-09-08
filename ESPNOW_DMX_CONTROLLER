/*
  DMX CONTROLLER (ESP-NOW Input - DMX Output)
  Se'ns moren les plantes - Posició fixa
  Setembre 2024 - FiraTàrrega

  Hardware: SparkFun ESP32 DMX shield + ESP32 Thing Plus
*/

#include <Arduino.h>
#include <esp_now.h>  // https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/
#include <WiFi.h>
#include <esp_dmx.h>  // https://github.com/someweisguy/esp_dmx
#include <debounce.h>

// GPIO PINS
const int buttonPin = 0;
const int ledPin = 13;

// VARIABLES
uint8_t buttonState = 1;
uint8_t lastButtonState = 1;

// DMX PINS
const int transmitPin = 17;
const int receivePin = 16;
const int enablePin = 21;

// DMX PORT: UART 1
dmx_port_t dmxPort = 1;

// DMX PACKET
byte data[DMX_PACKET_SIZE];

// DMX CHANNELS
const int parChannels = 8;
const int laserChannels = 9;
const int fogmachineChannels = 2;
const int dimmerChannels = 4;
const int num_bytes_to_send = 30;  // Number of channels to be sent in each message + 1 for break message

// DMX START ADDRESSES
const int par1StartAddress = 1;
const int par2StartAddress = 9;
const int fogmachineStartAddress = 17;
const int laserStartAddress = 19;
//const int dimmerStartAddress = 26;
const int strobe = 7;  // offset for strobe channel (channel 8), as per fixtures' specs

// PRESETS AND CONTROL
// Par: dimmer, red, green, blue, white, ambar, uv, strobe
uint8_t parPreset[parChannels] = { 0, 75, 0, 0, 255, 0, 127, 0 };
uint8_t fadeStep = 0;
//Laser: control, template, x, y, speed, segment speed, zoom, colour, segment colour
uint8_t laserLine[laserChannels] = { 150, 45, 0, 0, 0, 0, 0, 255, 0 };
uint8_t laserSlowLine[laserChannels] = { 150, 45, 0, 0, 255, 0, 0, 255, 0 };
uint8_t laserTunnel[laserChannels] = { 150, 1, 0, 250, 0, 0, 0, 255, 0 };
uint8_t laserSpiral[laserChannels] = { 150, 255, 0, 250, 0, 0, 0, 255, 0 };

// TIMING
unsigned long lastUpdate = millis();
unsigned long lastFadeUpdate = millis();
bool isCoolingDown = false;
bool isFadingIn = false;

// ESP-NOW MESSAGE STRUCTURE
typedef struct struct_message {
  int a;
  int b;
  int c;
} struct_message;
struct_message myData;

// ESP-NOW MESSAGES
const int idle = 0;
const int nextScene = 10;
const int previousScene = 20;

// SCENE MANAGEMENT
const int numScenes = 8;  // define
uint8_t scene = 0;        // counter
uint8_t lastScene;
uint8_t sceneManagement = idle;
uint8_t manualManagement = idle;

// ESP-NOW CALLBACK (this function will be executed when data is received)
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));  // actually stores the data in memory for further processing
  /*
  Serial.print("Bytes received: ");
  Serial.print(len);
  Serial.print("  ");
  Serial.print(myData.a);
  Serial.print("  ");
  Serial.print(myData.b);
  Serial.print("  ");
  Serial.println(myData.c);
  */
}

// BUTTON DEBOUNCING
bool useShort;
// Short press button
static void shortPressHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    useShort = true;
    analogWrite(ledPin, 200);
  } else {
    // btnState == BTN_OPEN.
    if (useShort == true) {
      manualManagement = nextScene;
      analogWrite(ledPin, 0);
      Serial.println("Next scene hitted manually");  // DEBUGGING
      return;
    }
  }
}
// Long press button
static void longPressHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    useShort = false;
    analogWrite(ledPin, 200);
    // Send a message after it has been held down a long time.
  } else {
    // btnState == BTN_OPEN.
    if (useShort == false) {
      Serial.println("Previous scene hitted manually");  // DEBUGGING
      manualManagement = previousScene;
      analogWrite(ledPin, 0);
      return;
    }
  }
}
// Long press and release
static void longPressAndReleaseHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    useShort = false;
    Serial.println("Fog machine manually armed.");
    analogWrite(ledPin, 0);
    data[fogmachineStartAddress] = 255;
    data[fogmachineStartAddress + 1] = 255;
    dmx_write(dmxPort, data, DMX_PACKET_SIZE);
  } else {
    // btnState == BTN_OPEN.
    // Shown when the button is released, but only if held down 3s first.
    Serial.println("Fog machine stopped.");
    data[fogmachineStartAddress] = 0;
    data[fogmachineStartAddress + 1] = 0;
    dmx_write(dmxPort, data, DMX_PACKET_SIZE);
  }
}
// Button definitions
static Button shortBtn(0, shortPressHandler);
static Button longBtn(1, longPressHandler);
static Button longPressReleaseBtn(2, longPressAndReleaseHandler);


void setup() {
  Serial.begin(115200);
  Serial.println("SparkFun DMX ESPNOW - Initializing...");
  pinMode(buttonPin, INPUT_PULLUP);
  longBtn.setPushDebounceInterval(1000);              // 1 second
  longPressReleaseBtn.setPushDebounceInterval(3000);  // 3 seconds
  pinMode(ledPin, OUTPUT);

  // DMX SETUP
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {};
  int personality_count = 0;
  dmx_driver_install(dmxPort, &config, personalities, personality_count);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);
  delay(20);

  // ESP-NOW SETUP
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {  // Init ESP-NOW
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  delay(20);
  Serial.println("Ready.");
  blackout();
}

static void pollButtons() {
  uint8_t pinState = digitalRead(buttonPin);
  shortBtn.update(pinState);
  longBtn.update(pinState);
}

void loop() {

  pollButtons();

  // SCENE MANAGER
  sceneManagement = myData.a;
  if (!isCoolingDown) {
    if (sceneManagement == nextScene | manualManagement == nextScene) {
      if (scene < numScenes) {
        scene++;
        Serial.print("Scene: ");
        Serial.println(scene);
        cooldown();
      } else {
        Serial.println("Scene maximum reached.");
        data[laserStartAddress] = 0;
        dmx_write(dmxPort, data, DMX_PACKET_SIZE);
        printAllChannels();
        //blinkLed();         // blocking function; debugging only
      }
    } else if (sceneManagement == previousScene | manualManagement == previousScene) {
      if (scene > 0) {
        scene--;
        Serial.print("Scene: ");
        Serial.println(scene);
        cooldown();
      } else {
        Serial.println("Scene minimum reached.");
        //blinkLed();
      }
    }
  }
  manualManagement = idle;

  // SCENE SWITCH
  // Make sure to set DMX values to the value they should have when calling the scene
  if (scene != lastScene) {  // only triggered on scene change
    switch (scene) {
      case 0:  // turn everything off
        blackout();
        break;
      case 1:  // turn lights on (fade in)
        fadeIn();
        break;
      case 2:  // laser effect: line
        for (int i = 0; i < laserChannels; i++) {
          data[i + laserStartAddress] = laserLine[i];
        }
        dmx_write(dmxPort, data, DMX_PACKET_SIZE);
        printAllChannels();
        break;
      case 3:  // laser effect: slower line
        data[laserStartAddress + 4] = 180;
        dmx_write(dmxPort, data, DMX_PACKET_SIZE);
        printAllChannels();
        break;
      case 4:  // laser tunnel
        for (int i = 0; i < laserChannels; i++) {
          data[i + laserStartAddress] = laserTunnel[i];
        }
        dmx_write(dmxPort, data, DMX_PACKET_SIZE);
        printAllChannels();
        break;
      case 5:  // laser effect: slower tunnel
        data[laserStartAddress + 4] = 255;
        data[laserStartAddress + 6] = 127;
        dmx_write(dmxPort, data, DMX_PACKET_SIZE);
        printAllChannels();
        break;
      case 6:  // laser effect: slow line
        for (int i = 0; i < laserChannels; i++) {
          data[i + laserStartAddress] = laserSlowLine[i];
        }
        dmx_write(dmxPort, data, DMX_PACKET_SIZE);
        printAllChannels();
        break;
      case 7:  // laser effect: slow line
        data[laserStartAddress + 3] = 127;
        data[laserStartAddress + 6] = 127;
        dmx_write(dmxPort, data, DMX_PACKET_SIZE);
        printAllChannels();
        break;
      case 8:  // laser spiral
        Serial.println("Laser Spiral");
        for (int i = 0; i < laserChannels; i++) {
          data[i + laserStartAddress] = laserSpiral[i];
        }
        dmx_write(dmxPort, data, DMX_PACKET_SIZE);
        printAllChannels();
        break;
    }
  }
  lastScene = scene;

  // FOG RELEASE (direct input from knob, bypasses scene count)
  data[fogmachineStartAddress] = myData.b / 3;  // fog machine's fan
  data[fogmachineStartAddress + 1] = myData.b;  // fog machine's fog

  // LASER TOGGLE (direct input from knob, bypasses scene count)
  if (myData.b > 200) {
    data[laserStartAddress] = 150;  // sets the channel either to on or to off
  } else {
    data[laserStartAddress] = 0;
  }

  // PAR STROBE (direct input from knob, bypasses scene count)
  if (scene != 0) {
    data[par1StartAddress + strobe] = myData.c;  // strobe
    data[par2StartAddress + strobe] = myData.c;  // strobe
  }

  // TIMING
  if (isCoolingDown) {
    unsigned long now = millis();
    if (now - lastUpdate >= 1000UL) {
      isCoolingDown = false;
      analogWrite(ledPin, 0);
    }
  }
  if (isFadingIn) {
    unsigned long nowFade = millis();
    if (nowFade - lastFadeUpdate >= 15UL) {
      if (fadeStep < 257) {
        parPreset[0] = fadeStep;                 // increment intensity step by step
        for (int j = 0; j < parChannels; j++) {  // store values in dmx buffer (for both pars)
          data[j + par1StartAddress] = parPreset[j];
          data[j + par2StartAddress] = parPreset[j];
        }
        dmx_write(dmxPort, data, DMX_PACKET_SIZE);
        dmx_send_num(dmxPort, num_bytes_to_send);
        dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK);
        fadeStep++;
        lastFadeUpdate = nowFade;
        if (fadeStep >= 255) {
          isFadingIn = false;
          fadeStep = 0;
          Serial.println();
          printAllChannels();
          Serial.println("Fade is complete");
        }
      }
    }
  }

  // DMX SEND
  dmx_write(dmxPort, data, DMX_PACKET_SIZE);
  dmx_send_num(dmxPort, num_bytes_to_send);
  dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK);
  delay(20);
}

// MANAGEMENT FUNCTIONS

void blackout() {
  for (int i = 0; i < num_bytes_to_send; i++) {
    data[i + 1] = 0;
  }
  dmx_write(dmxPort, data, DMX_PACKET_SIZE);
  printAllChannels();
}

void printAllChannels() {
  for (int i = 0; i < num_bytes_to_send; i++) {
    Serial.print(i + 1);
    Serial.print("\t");
  }
  Serial.println();
  for (int i = 0; i < num_bytes_to_send; i++) {
    Serial.print(data[i + 1]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.println();
}

void cooldown() {
  unsigned long currentMillis = millis();
  lastUpdate = currentMillis;
  isCoolingDown = true;
  analogWrite(ledPin, 255);
}

void fadeIn() {
  lastFadeUpdate = millis();
  isFadingIn = true;
}

void blinkLed() {  // used only for debugging
  for (int i = 0; i < 4; i++) {
    if (i == 0 || i == 2) {
      analogWrite(ledPin, 200);
    } else if (i == 1 || i == 3) {
      analogWrite(ledPin, 0);
    }
    delay(100);
  }
}
