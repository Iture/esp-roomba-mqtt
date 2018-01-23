#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP.h>
#include <ArduinoOTA.h>
#include <Roomba.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
extern "C" {
#include "user_interface.h"
}

// Remote debugging over telnet. Just run:
// telnet roomba.home
#if LOGGING
#include <RemoteDebug.h>
#define DLOG(msg, ...) if(Debug.isActive(Debug.DEBUG)){Debug.printf(msg, ##__VA_ARGS__);}
RemoteDebug Debug;
#else
#define DLOG(msg, ...)
#endif

// Roomba setup
Roomba roomba(&Serial, Roomba::Baud115200);

int distanceCumulative;

bool isActive=false;
bool isCleaning=false;

// Network setup
WiFiClient wifiClient;

// MQTT setup
PubSubClient mqttClient(wifiClient);
const PROGMEM char *commandTopic = MQTT_COMMAND_TOPIC;
const PROGMEM char *statusTopic = MQTT_STATE_TOPIC;

os_timer_t wakeupTimer;

void wakeup(){
  // TODO: There's got to be some black magic here that I'm missing to keep the
  // Roomba from going into deep sleep while on the dock. Thinking Cleaner
  // had a software solution, so it must be possible:
  // http://www.thinkingcleaner.com/setup/problem_solutions/
  // I've tried:
  // * Pulsing the BRC pin low at various intervals
  // * Switching to safe and full modes briefly
  // Maybe I could try:
  // * Switching on a motor or led very very briefly?
  // * Changing the baud rate
  // * Setting BRC to input (high impedence) instead of high
  // I have noticed sometimes I'll get a sensor packet while the BRC pin is
  // pulsed low but this is super unreliable.
  // Spin up a timer to bring the BRC_PIN back high again
  //os_timer_disarm(&wakeupTimer);


  //TODO: Too complicated, need to refactor and stay with working part (especially for verification enablig safe mode etc.)
  DLOG("Wakeup Roomba\n");
  
  pinMode(BRC_PIN, OUTPUT);
  digitalWrite(BRC_PIN, HIGH);

  delay(1000);
  ESP.wdtFeed();
  digitalWrite(BRC_PIN, LOW);
  delay(1000);
  ESP.wdtFeed();
  digitalWrite(BRC_PIN, HIGH);
  delay(250);
  roomba.start();

  ESP.wdtFeed();
  DLOG("Done waking up\n");
 }


void sendStatus(bool updateCountersOnly) {
  // Flush serial buffers
  while (Serial.available()) {
    Serial.read();
  }

  uint8_t sensors[] = {
    Roomba::SensorDistance, // 2 bytes, mm, signed
    Roomba::SensorChargingState, // 1 byte
    Roomba::SensorVoltage, // 2 bytes, mV, unsigned
    Roomba::SensorCurrent, // 2 bytes, mA, signed
    Roomba::SensorBatteryCharge, // 2 bytes, mAh, unsigned
    Roomba::SensorBatteryCapacity, // 2 bytes, mAh, unsigned
    Roomba::SensorOIMode // 1 byte
  };
  uint8_t values[12];

  int16_t distance = 0;
  uint8_t chargingState = 0;
  uint16_t voltage = 0;
  int16_t current = 0;
  uint16_t charge = 0;
  uint16_t capacity =0;
  uint8_t OIMode = 0;
  


  bool success = roomba.getSensorsList(sensors, sizeof(sensors), values, 12);
  if (!success) {
    DLOG("Failed to read sensor values from Roomba\n");
    isActive=false;
    //return;
  }else {
    distance = values[0] * 256 + values[1];
    chargingState = values[2];
    voltage = values[3] * 256 + values[4];
    current = values[5] * 256 + values[6];
    charge = values[7] * 256 + values[8];
    capacity = values[9] * 256 + values[10];
    OIMode = values[11];
    isActive = true;
    DLOG("Got sensor values Distance:%dmm DistanceC:%dmm ChargingState:%d Voltage:%dmV Current:%dmA Charge:%dmAh Capacity:%dmAh OIMode:%d\n", distance, distanceCumulative, chargingState, voltage, current, charge, capacity,OIMode);
  }


  bool cleaning = false;
  bool docked = false;

  String state;
  if (current < -300) {
    cleaning = true;
    isCleaning = true;
  } else if (current > -50) {
    docked = true;
    isCleaning=false;
  }
  distanceCumulative = distanceCumulative + abs(distance);
  if (updateCountersOnly == false)
  {
    
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["battery_level"] = (charge * 100)/capacity;
    root["cleaning"] = cleaning;
    root["docked"] = docked;
    root["charging"] = chargingState == Roomba::ChargeStateReconditioningCharging
    || chargingState == Roomba::ChargeStateFullChanrging
    || chargingState == Roomba::ChargeStateTrickleCharging;
    root["charging_state"] = chargingState;
    root["voltage"] = voltage;
    root["current"] = current;
    root["charge"] = charge;
    root["distance"] = distanceCumulative;
    String jsonStr; 
    root.printTo(jsonStr);
    DLOG("Sending report to MQTT");
    DLOG(jsonStr.c_str());
    mqttClient.publish(statusTopic, jsonStr.c_str());
    distanceCumulative=0;
  }
}



bool performCommand(const char *cmdchar) {
  //It is a lot of easier to wake Roomba every time
  if (!isCleaning)
  {
    wakeup();
  };
  
  // Char* string comparisons dont always work
  String cmd(cmdchar);

  //TODO: refactor this, remove unneccessary commands
  // MQTT protocol commands
  if (cmd == "turn_on") {
    if (!isCleaning)
    {
      DLOG("Turning on\n");
      roomba.cover();
    }
  } else if (cmd == "turn_off") {
    DLOG("Turning off\n");
    roomba.power();
  } else if (cmd == "wakeup") {
    DLOG("Waking up\n");
    wakeup();
  } else if (cmd == "stop") {
    DLOG("Stopping\n");
    roomba.cover();
  } else if (cmd == "clean_spot") {
    DLOG("Cleaning Spot\n");
    roomba.spot();
  } else if (cmd == "reset") {
    DLOG("Resetting\n");
    ESP.restart();
  } else if (cmd == "locate") {
    DLOG("Locating\n");
    // TODO: Locating roomba - play song
  } else if (cmd == "return_to_base") {
    DLOG("Returning to Base\n");
    roomba.dock();
    isCleaning=true;
  } else {
    return false;
  }

  return true;
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  DLOG("Received mqtt callback for topic %s\n", topic);
  if (strcmp(commandTopic, topic) == 0) {
    // turn payload into a null terminated string
    char *cmd = (char *)malloc(length + 1);
    memcpy(cmd, payload, length);
    cmd[length] = 0;

    if(!performCommand(cmd)) {
      DLOG("Unknown command %s\n", cmd);
    }
    free(cmd);
  }
}

void debugCallback() {
  String cmd = Debug.getLastCommand();

  // Debugging commands via telnet
  if (performCommand(cmd.c_str())) {
  } else if (cmd == "quit") {
    DLOG("Stopping Roomba\n");
    Serial.write(173);
  } else if (cmd == "rreset") {
    DLOG("Resetting Roomba\n");
    roomba.reset();
  } else if (cmd == "mqtthello") {
    mqttClient.publish("vacuum/hello", "hello there");
  } else if (cmd == "version") {
    const char compile_date[] = __DATE__ " " __TIME__;
    DLOG("Compiled on: %s\n", compile_date);
  } else {
    DLOG("Unknown command %s\n", cmd.c_str());
  }
}

void setup() {
  wdt_enable(2000);


  distanceCumulative = 0;
  // Set Hostname.
  String hostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostname);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    ESP.wdtFeed();
  }

  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();

  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(mqttCallback);

  #if LOGGING
  Debug.begin((const char *)hostname.c_str());
  Debug.setResetCmdEnabled(true);
  Debug.setCallBackProjectCmds(debugCallback);
  Debug.setSerialEnabled(false);
  #endif

  //FIXME: Song upload
  wakeup();
  uint8_t a[] = { 0, 9, 57, 30, 57, 30, 57, 30, 53, 20, 60, 10, 57, 30, 53, 20, 60, 10, 57, 45};
  roomba.song(0, a, sizeof(a));
  
}

void reconnect() {
  DLOG("Attempting MQTT connection...\n");
  // Attempt to connect
  //if (mqttClient.connect(HOSTNAME, MQTT_USER, MQTT_PASSWORD,'/lwt/roomba',1,true,'ON')) {
  if (mqttClient.connect(HOSTNAME, MQTT_USER, MQTT_PASSWORD)) {
    DLOG("MQTT connected\n");
    mqttClient.subscribe(commandTopic);
  } else {
    DLOG("MQTT failed rc=%d try again in 5 seconds\n", mqttClient.state());
  }
  ESP.wdtFeed();
}



int lastStateMsgTime = 0;
int lastStateUpdateTime = 0;
int lastWakeupTime = 0;

void loop() {
  ESP.wdtFeed();
  long now = millis();
  // If MQTT client can't connect to broker, then reconnect
  if (!mqttClient.connected()) {
    reconnect();
  } else {
    if (now - lastWakeupTime > TIMER_WAKEUP_INTERVAL) {
      lastWakeupTime = now;
      if (!isActive)
      {
        wakeup();
      }
    }
    if (now - lastStateUpdateTime > TIMER_READ_INTERVAL) {
      lastStateUpdateTime = now;
      sendStatus(true);
    }
    if (now - lastStateMsgTime > TIMER_SEND_INTERVAL) {
      lastStateMsgTime = now;
      sendStatus(false);
    }
  }

  ArduinoOTA.handle();
  yield();
  Debug.handle();
  mqttClient.loop();
}
