#include <atomic>
#include <condition_variable>
#include <mutex>
#include <PubSubClient.h>
#include <RadioLib.h>
#include <WiFiManager.h>

// MQTT settings
const char *const   mqtt_topic       = "meshcore/bridge";
const char *const   mqtt_server      = "vps001.vanheusden.com";
constexpr const int mqtt_server_port = 1883;
char               *mqtt_sub_topic   = nullptr;
char               *mqtt_pub_topic   = nullptr;

// LoRa settings
#define CARRIER_FREQ 869.618
#define BANDWIDTH    62.5
#define SF           8
#define CR           8
#define SYNC_WORD    0x12
#define POWER        22
#define PREAMBLE     16

// SX1262 radio setup
// NSS pin:   5
// DIO1 pin:  2
// NRST pin:  3
// BUSY pin:  4
#if defined(XIAO_HEADERS)
SX1262 radio = new Module(5, 2, 3, 4);
#elif defined(XIAO_CONNECTOR)
SX1262 radio = new Module(41, 39, 42, 40);
#elif defined(HELTEC_V3)
SX1262 radio = new Module(8, 14, 12, 13);
#else
#error please configure the ESP32 to SX1262 pins in mcb.ino
#endif

// WiFi settings
#define WIFI_PORTAL_TIMEOUT 60
#define WIFI_CONNECT_TIMEOUT 15
#define WIFI_NAME "LoRaBridge"

#define MAX_LORA_MSG_SIZE RADIOLIB_SX126X_MAX_PACKET_LENGTH

struct mqtt_entry {
  uint8_t  buffer[MAX_LORA_MSG_SIZE];
  unsigned n;
};

#define MAX_N_MQTT_ENTRIES 10
std::mutex mqtt_recv_lock;
std::array<mqtt_entry, MAX_N_MQTT_ENTRIES> mqtt_recv_entries;
int n_mqtt_recv_entries = 0;

std::mutex mqtt_send_lock;
std::array<mqtt_entry, MAX_N_MQTT_ENTRIES> mqtt_send_entries;
int n_mqtt_send_entries = 0;
std::condition_variable mqtt_send_cv;

uint8_t rf_buffer[MAX_LORA_MSG_SIZE];

#define MAX_N_DEDUP_HASHES 256
uint32_t dedup_hashes[MAX_N_DEDUP_HASHES];
int dedup_hash_index = 0;

char sys_id[9] { 0 };

// hash function
uint32_t adler32(const void *const buf, const size_t buflength) {
        const uint8_t *buffer = reinterpret_cast<const uint8_t *>(buf);

        uint32_t s1 = 1;
        uint32_t s2 = 0;

        for (size_t n = 0; n < buflength; n++) {
                s1 = (s1 + buffer[n]) % 65521;
                s2 = (s2 + s1) % 65521;
        }

        return (s2 << 16) | s1;
}

bool register_packet(const uint8_t *const pl, const size_t len) {
  uint32_t hash = adler32(pl, len);

  Serial.printf("[%08x]", hash);

  for(int i=0; i<MAX_N_DEDUP_HASHES; i++) {
    if (dedup_hashes[i] == hash) {
      Serial.print("DUP ");
      return false;
    }
  }

  Serial.print(" ");
  dedup_hashes[dedup_hash_index] = hash;
  dedup_hash_index = (dedup_hash_index + 1) % MAX_N_DEDUP_HASHES;

  return true;
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("MQTT "));
  Serial.print(topic);
  Serial.print(F(": "));
  if (length == 0) {
    Serial.println(F(", ignoring empty msg"));
    return;
  }

  std::unique_lock<std::mutex> lck(mqtt_recv_lock);
  if (n_mqtt_recv_entries >= MAX_N_MQTT_ENTRIES) {
    Serial.println(F(", recv buffer full"));
    return;
  }

  memcpy(mqtt_recv_entries[n_mqtt_recv_entries].buffer, payload, length);
  mqtt_recv_entries[n_mqtt_recv_entries].n = length;
  n_mqtt_recv_entries++;

  Serial.print(F(", msg size: "));
  Serial.println(length);
}

WiFiClient wifi_client;
PubSubClient *mqtt_client = nullptr;

std::atomic_bool rf_received { false };

void ICACHE_RAM_ATTR set_rf_recv_flag() {
  rf_received = true;
}

void start_rf_receive() {
  auto state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("failed, code: "));
    Serial.println(state);
    ESP.restart();
  }
}

void mqtt_transmit(const uint8_t *const pl, const size_t len) {
  digitalWrite(LED_BUILTIN, HIGH);
  mqtt_client->publish(mqtt_pub_topic, pl, len, false);
  digitalWrite(LED_BUILTIN, LOW);
}

void check_mqtt(void) {
  mqtt_client->loop();

  if (!mqtt_client->connected()) {
    Serial.println(F(" *** MQTT reconnect"));
    mqtt_client->disconnect();
    if (!mqtt_client->connect(sys_id)) {
      Serial.println(F("MQTT connect failed"));
      ESP.restart();
    }
    if (!mqtt_client->subscribe(mqtt_sub_topic)) {
      Serial.println(F("MQTT subscribe failed"));
      ESP.restart();
    }
  }
}

void mqtt_thread(void *) {
  mqtt_client = new PubSubClient(mqtt_server, mqtt_server_port, mqtt_callback, wifi_client);

  for(;;) {
    check_mqtt();

    std::unique_lock<std::mutex> lck(mqtt_send_lock);
    mqtt_send_cv.wait_for(lck, std::chrono::milliseconds(1));
    for(int i=0; i<n_mqtt_send_entries; i++)
        mqtt_transmit(mqtt_send_entries[i].buffer, mqtt_send_entries[i].n);
    n_mqtt_send_entries = 0;
  }
}

TaskHandle_t mqtt_handle;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print(F("[SX126x] Initializing ... "));
  auto state = radio.begin(CARRIER_FREQ, BANDWIDTH, SF, CR, SYNC_WORD, POWER, PREAMBLE);
  if (state == RADIOLIB_ERR_NONE)
    Serial.println(F("success!"));
  else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    ESP.restart();
  }

  if (radio.setCRC(true) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
    Serial.println(F("Selected CRC is invalid for this module!"));
    ESP.restart();
  }

  WiFiManager wm;
  wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT);
  wm.setConnectTimeout(WIFI_CONNECT_TIMEOUT);
  if (!wm.autoConnect(WIFI_NAME)) {
    Serial.print(F("WiFi failed, code "));
    Serial.println(state);
    ESP.restart();
  }

  radio.setPacketReceivedAction(set_rf_recv_flag);
  start_rf_receive();

  auto mac = WiFi.macAddress();
  uint8_t temp[4] { };
  for(int i=0; i<6; i++)
    temp[i % 4] ^= mac[i];
  sprintf(sys_id, "%08x", *reinterpret_cast<const uint32_t *>(temp));
  Serial.print(F("System ID: "));
  Serial.println(sys_id);

  asprintf(&mqtt_sub_topic, "%s/#",  mqtt_topic);
  asprintf(&mqtt_pub_topic, "%s/%s", mqtt_topic, sys_id);

  xTaskCreatePinnedToCore(mqtt_thread, "mqtt", 10000, nullptr, 0, &mqtt_handle, 0);
}

void rf_transmit(const uint8_t *const pl, const size_t len) {
  digitalWrite(LED_BUILTIN, HIGH);
  int state = radio.transmit(pl, len);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(F("packet transmitted via RF, data rate: "));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("rf packet too long!"));
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("rf timeout!"));
  }
  else {
    // some other error occurred
    Serial.print(F("rf transmission failed, code "));
    Serial.println(state);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (rf_received.exchange(false)) {
    int num_bytes = radio.getPacketLength();
    if (num_bytes == 0)
      Serial.println(F("RF ignoring empty msg"));
    else if (num_bytes > sizeof(rf_buffer)) {
      Serial.print(F("RF truncated: "));
      Serial.print(num_bytes - sizeof(rf_buffer));
      Serial.println(F(" too short"));
    }
    else {
      int state = radio.readData(rf_buffer, num_bytes);
      if (state == RADIOLIB_ERR_NONE) {
        if (register_packet(rf_buffer, num_bytes)) {
          std::unique_lock<std::mutex> lck(mqtt_send_lock);
          if (n_mqtt_send_entries >= MAX_N_MQTT_ENTRIES)
            Serial.println(F("MQTT send buffer full"));
          else {
            memcpy(mqtt_send_entries[n_mqtt_send_entries].buffer, rf_buffer, num_bytes);
            mqtt_send_entries[n_mqtt_send_entries].n = num_bytes;
            n_mqtt_send_entries++;
            mqtt_send_cv.notify_one();
          }
        }
        else {
          Serial.println(F("rf -> mqtt: dedupped"));
        }
      }
      else if (state == RADIOLIB_ERR_CRC_MISMATCH)
        Serial.println(F("CRC mismatch"));
      else {
        Serial.print(F("recv failed: "));
        Serial.println(state);
      }
    }
  }

  std::unique_lock<std::mutex> lck(mqtt_recv_lock);
  bool any_mqtt = n_mqtt_recv_entries > 0;
  for(int i=0; i<n_mqtt_recv_entries; i++) {
    if (register_packet(mqtt_recv_entries[i].buffer, mqtt_recv_entries[i].n))
      rf_transmit(mqtt_recv_entries[i].buffer, mqtt_recv_entries[i].n);
    else
      Serial.println(F("mqtt -> rf: dedupped"));
  }
  n_mqtt_recv_entries = 0;
  lck.unlock();

  if (any_mqtt)
    start_rf_receive();
}
