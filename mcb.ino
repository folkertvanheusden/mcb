#include <atomic>
#include <mutex>
#include <PubSubClient.h>
#include <RadioLib.h>
#include <WiFiManager.h>

// MQTT settings
const char *const   mqtt_topic       = "meshcore/bridge";
const char *const   mqtt_server      = "vps001.vanheusden.com";
constexpr const int mqtt_server_port = 1883;

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
SX1262 radio = new Module(5, 2, 3, 4);

#define MAX_LORA_MSG_SIZE 384

struct mqtt_entry {
  uint8_t  buffer[MAX_LORA_MSG_SIZE];
  unsigned n;
};

#define MAX_N_MQTT_ENTRIES 10
std::mutex mqtt_lock;
std::array<mqtt_entry, MAX_N_MQTT_ENTRIES> mqtt_entries;
int n_mqtt_entries = 0;

uint8_t rf_buffer[MAX_LORA_MSG_SIZE];

#define MAX_N_DEDUP_HASHES 16
uint32_t dedup_hashes[MAX_N_DEDUP_HASHES];
int dedup_hash_index = 0;

// hash function
uint32_t adler32(const void *buf, size_t buflength)
{
        const uint8_t *buffer = reinterpret_cast<const uint8_t *>(buf);

        uint32_t s1 = 1;
        uint32_t s2 = 0;

        for (size_t n = 0; n < buflength; n++) {
                s1 = (s1 + buffer[n]) % 65521;
                s2 = (s2 + s1) % 65521;
        }

        return (s2 << 16) | s1;
}

bool register_packet(const uint8_t *const pl, const size_t len)
{
  uint32_t hash = adler32(pl, len);

  for(int i=0; i<MAX_N_DEDUP_HASHES; i++) {
    if (dedup_hashes[i] == hash)
      return false;
  }

  dedup_hashes[dedup_hash_index] = hash;
  dedup_hash_index = (dedup_hash_index + 1) % MAX_N_DEDUP_HASHES;

  return true;
}

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  std::unique_lock<std::mutex> lck(mqtt_lock);
  if (n_mqtt_entries >= MAX_N_MQTT_ENTRIES) {
    Serial.println(F("MQTT recv buffer full"));
    return;
  }

  memcpy(mqtt_entries[n_mqtt_entries].buffer, payload, length);
  mqtt_entries[n_mqtt_entries].n = length;
  n_mqtt_entries++;
}

WiFiClient wifi_client;
PubSubClient mqtt_client(mqtt_server, mqtt_server_port, mqtt_callback, wifi_client);

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

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print(F("[SX126x] Initializing ... "));
  // carrier frequency:           869.618 MHz
  // bandwidth:                   62.5 kHz
  // spreading factor:            8
  // coding rate:                 8
  // sync word:                   0x12
  // output power:                22 dBm
  // preamble length:             16 symbols
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
  wm.setConfigPortalTimeout(60);
  if (!wm.autoConnect("LoRaBridge")) {
    Serial.print(F("WiFi failed, code "));
    Serial.println(state);
    ESP.restart();
  }

  radio.setPacketReceivedAction(set_rf_recv_flag);
  start_rf_receive();
}

void check_mqtt(void) {
  mqtt_client.loop();

  if (!mqtt_client.connected()) {
    if (!mqtt_client.connect("LoRaBridge")) {
      Serial.println(F("MQTT connect failed"));
      ESP.restart();
    }
    if (!mqtt_client.subscribe(mqtt_topic)) {
      Serial.println(F("MQTT subscribe failed"));
      ESP.restart();
    }
  }
}

void mqtt_transmit(const uint8_t *const pl, const size_t len) {
  digitalWrite(LED_BUILTIN, HIGH);
  mqtt_client.publish(mqtt_topic, pl, len, false);
  digitalWrite(LED_BUILTIN, LOW);
}

void rf_transmit(const uint8_t *const pl, const size_t len) {
  digitalWrite(LED_BUILTIN, HIGH);
  int state = radio.transmit(pl, len);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(F("packet transmitted via RF, data rate:"));
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
  check_mqtt();

  if (rf_received.exchange(false)) {
    int num_bytes = radio.getPacketLength();
    int state     = radio.readData(rf_buffer, num_bytes);
    if (num_bytes > sizeof(rf_buffer)) {
      Serial.print(F("Truncated: "));
      Serial.print(num_bytes - sizeof(rf_buffer));
      Serial.print(F(" too short"));
    }
    else if (state == RADIOLIB_ERR_NONE) {
      if (register_packet(rf_buffer, num_bytes))
        mqtt_transmit(rf_buffer, num_bytes);
      else
        Serial.println(F("rf -> mqtt: dedupped"));
    }
    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
      Serial.print(F("Recv CRC mismatch"));
    else {
      Serial.print(F("Recv failed: "));
      Serial.println(state);
    }
  }

  std::unique_lock<std::mutex> lck(mqtt_lock);
  bool any_mqtt = n_mqtt_entries > 0;
  for(int i=0; i<n_mqtt_entries; i++) {
    if (register_packet(mqtt_entries[i].buffer, mqtt_entries[i].n))
      rf_transmit(mqtt_entries[i].buffer, mqtt_entries[i].n);
    else
      Serial.println(F("mqtt -> rf: dedupped"));
  }
  n_mqtt_entries = 0;
  lck.unlock();

  if (any_mqtt)
    start_rf_receive();
}
