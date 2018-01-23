#include "secrets.h"

#define HOSTNAME "roomba" // e.g. roomba.local
#define BRC_PIN 16

#define MQTT_SERVER "172.22.0.56"
#define MQTT_USER ""
#define MQTT_COMMAND_TOPIC "/roomba/command"
#define MQTT_STATE_TOPIC "/roomba/state"

#define TIMER_READ_INTERVAL 500
#define TIMER_SEND_INTERVAL 60000
#define TIMER_WAKEUP_INTERVAL 900000
