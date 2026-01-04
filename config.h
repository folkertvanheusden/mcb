// MQTT settings
#define MQTT_TOPIC      "meshcore/bridge"
#define MQTT_SERVER     "vps001.vanheusden.com"
#define MQTT_SERVER_PORT 1883
// #define MQTT_USER       "mqtt-user"
// #define MQTT_PASSWORD   "mqtt-password"

// deduplicates only the payload for better loop detection
#define MESHCORE_MODE

// LoRa settings
#define CARRIER_FREQ 869.618
#define BANDWIDTH    62.5
#define SF           8
#define CR           8
#define SYNC_WORD    0x12
#define POWER        22
#define PREAMBLE     16
#define USE_CRC      true

// display settings
#define DISPLAY_TIMEOUT 2500  // in milliseconds

// WiFi settings
#define WIFI_PORTAL_TIMEOUT  60  // seconds
#define WIFI_CONNECT_TIMEOUT 15
