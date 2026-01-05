#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <atomic>
#include <condition_variable>
#include <esp_mac.h>
#include <ESPmDNS.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>  // included by wifimanager as well(?!)
#include <mutex>
#include <PubSubClient.h>
#include <RadioLib.h>
#include <SPI.h>

#include "config.h"


// MQTT
char *mqtt_sub_topic = nullptr;
char *mqtt_pub_topic = nullptr;

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
#define DISPTYPE_SSD1306
#define USER_BUTTON 0
#define PIN_SDA 17
#define PIN_SCL 18
#define PIN_RST_OLED 21

#elif defined(LILYGO_T3)
SX1276 radio = new Module(18, 26, 23);
#undef POWER
#define POWER        20

#elif defined(T_BEAM_1_2)
SX1276 radio = new Module(18, 26, 23);
#undef POWER
#define POWER        20

#elif defined(T_BEAM_SUPREME)
SPIClass    spi(HSPI);
SPISettings spi_settings(400000, MSBFIRST, SPI_MODE0);
SX1262      radio = new Module(10, 1, 5, 4, spi, spi_settings);
#define DISPTYPE_SH110x
#define PIN_SDA 17
#define PIN_SCL 18

#else
#error please configure the ESP32 to SX1262 pins in mcb.ino
#endif

#if defined(DISPTYPE_SSD1306)
#define HAS_DISPLAY
#define DISP_I2C_PORT 0x3c
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 64, &Wire, PIN_RST_OLED);
#endif

#if defined(DISPTYPE_SH110x)
#define HAS_DISPLAY
#define DISP_I2C_PORT 0x3c
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
Adafruit_SH1106G display(128, 64, &Wire, -1);
#endif

#if defined(HAS_DISPLAY)
// initialize with 'now' in case the display still has
// text from just before the reboot
std::atomic_uint32_t disp_since     { millis() + 1 };
TaskHandle_t         disp_handle    {              };
std::atomic_bool     button_pressed { false        };
bool                 disp_rc        { false        };
#endif

#define MAX_LORA_MSG_SIZE RADIOLIB_SX126X_MAX_PACKET_LENGTH

TaskHandle_t mqtt_handle;

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
std::mutex dedup_hash_lock;
uint32_t dedup_hashes[MAX_N_DEDUP_HASHES];
uint16_t dedup_hash_index = 0;

char sys_id[19] { 0 };

uint32_t prev_millis = 0;
uint32_t rf_n        = 0;
uint32_t mqtt_n      = 0;
uint32_t hash_ok     = 0;
uint32_t hash_dup    = 0;
uint32_t crc_errors  = 0;

#define HTTP_PORT 80
AsyncWebServer *http_server = nullptr;

void set_disp_state(const bool on) {
#if defined(HAS_DISPLAY)
#if defined(DISPTYPE_SSD1306)
  display.ssd1306_command(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
#else
  display.oled_command(on ? SH110X_DISPLAYON : SH110X_DISPLAYOFF);
#endif
#endif
}

void disp_text(const char *const txt) {
#if defined(HAS_DISPLAY)
  if (disp_rc) {
    disp_since = millis();
    set_disp_state(true);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.write(txt);
    display.display();
  }
#endif
}

void failed_reboot(const char *const txt) {
  Serial.println(txt);
  disp_text(txt);
  ESP.restart();
}

void disp_thread(void *) {
  for(;;) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
#if defined(HAS_DISPLAY)
    if (disp_since == 0)
      continue;

    uint32_t now = millis();
    if (now - disp_since >= DISPLAY_TIMEOUT) {
      if (disp_rc) {
        set_disp_state(false);
        display.clearDisplay();
        display.display();
      }
      disp_since = 0;
    }
#endif
  }
}

void button_clicked() {
#if defined(HAS_DISPLAY)
  button_pressed = true;
#endif
}

// hash function
uint32_t adler32(const void *const buf, const size_t n) {
  const uint8_t  *buffer = reinterpret_cast<const uint8_t *>(buf);
  uint32_t  s1     = 1;
  uint32_t  s2     = 0;
  for(size_t i = 0; i < n; i++) {
    s1 = (s1 + buffer[i]) % 65521;
    s2 = (s2 + s1) % 65521;
  }
  return (s2 << 16) | s1;
}

void emit_stats() {
  Serial.printf("[%.2f%%]", hash_dup * 100. / (hash_dup + hash_ok));
}

bool register_packet(const char *const source, const uint8_t *const pl, const size_t len) {
  uint32_t now      = millis();
#ifdef MESHCORE_MODE
  uint16_t path_len = pl[1];
  uint16_t offset   = 2 + path_len;
  if (offset >= len)
    return false;
  uint8_t route     = pl[0] & 3;
  if (route == 0 /* flood */ || route == 3 /* direct */)
    offset += 4;  // transport codes
  if (offset >= len)
    return false;
  uint32_t hash     = adler32(&pl[offset], len - offset);
  Serial.printf("[%08x](%d) %ld %s ", hash, path_len, now - prev_millis, source);
#else
  uint32_t hash     = adler32(pl, len);
  Serial.printf("[%08x] %ld %s ", hash, now - prev_millis, source);
#endif
  prev_millis = now;

  {
    std::unique_lock<std::mutex> lck(dedup_hash_lock);
    for(int i=0; i<MAX_N_DEDUP_HASHES; i++) {
      if (dedup_hashes[i] == hash) {
        lck.unlock();
        hash_dup++;
        emit_stats();
        Serial.println(F(" DUP"));
        return false;
      }
    }

    dedup_hashes[dedup_hash_index] = hash;
    dedup_hash_index = (dedup_hash_index + 1) % MAX_N_DEDUP_HASHES;
  }

  hash_ok++;
  emit_stats();
  Serial.println(F(" OK"));

  return true;
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  if (length == 0) {
    Serial.println(F("ignoring empty mqtt msg"));
    return;
  }

  if (register_packet("MQTT", payload, length)) {
    std::unique_lock<std::mutex> lck(mqtt_recv_lock);
    if (n_mqtt_recv_entries >= MAX_N_MQTT_ENTRIES) {
      Serial.println(F("mqtt recv buffer full"));
      return;
    }

    memcpy(mqtt_recv_entries[n_mqtt_recv_entries].buffer, payload, length);
    mqtt_recv_entries[n_mqtt_recv_entries].n = length;
    n_mqtt_recv_entries++;
    mqtt_n++;
  }
}

WiFiClient wifi_client;
PubSubClient *mqtt_client = nullptr;

std::atomic_bool rf_received { false };

void ICACHE_RAM_ATTR set_rf_recv_flag() {
  rf_received = true;
}

void start_rf_receive() {
  auto state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE)
    failed_reboot("radio recv failed");
}

void set_builtin_led(const byte state) {
#if defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, state);
#endif
}

void mqtt_transmit(const uint8_t *const pl, const size_t len) {
  set_builtin_led(HIGH);
  mqtt_client->publish(mqtt_pub_topic, pl, len, false);
  set_builtin_led(LOW);
}

void check_mqtt(void) {
  mqtt_client->loop();

  if (!mqtt_client->connected()) {
    Serial.println(F(" *** MQTT reconnect"));
    mqtt_client->disconnect();
#if defined(MQTT_USER)
    bool mqtt_rc = mqtt_client->connect(sys_id, MQTT_USER, MQTT_PASSWORD);
#else
    bool mqtt_rc = mqtt_client->connect(sys_id);
#endif
    if (mqtt_rc == false)
      failed_reboot("MQTT connect failed");
    if (!mqtt_client->subscribe(mqtt_sub_topic))
      failed_reboot("MQTT subscribe failed");
  }
}

void mqtt_thread(void *) {
  mqtt_client = new PubSubClient(MQTT_SERVER, MQTT_SERVER_PORT, mqtt_callback, wifi_client);
  mqtt_client->setBufferSize(MAX_LORA_MSG_SIZE + 128);  // 128 is maximum topic size (limit by this app)

  for(;;) {
    check_mqtt();

    std::unique_lock<std::mutex> lck(mqtt_send_lock);
    mqtt_send_cv.wait_for(lck, std::chrono::milliseconds(50));
    for(int i=0; i<n_mqtt_send_entries; i++) {
      if (register_packet("RF", mqtt_send_entries[i].buffer, mqtt_send_entries[i].n))
        mqtt_transmit(mqtt_send_entries[i].buffer, mqtt_send_entries[i].n);
    }
    n_mqtt_send_entries = 0;
  }
}

void setup_http_server() {
  http_server = new AsyncWebServer(HTTP_PORT);

  http_server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "text/text", "LoRa bridge: " SELBOARD " / " AUTO_VERSION " / " __DATE__ " " __TIME__);
    request->send(response);
  });

  http_server->on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response_text = "{ \"uptime\": " + String(millis()) + ", \"hash-ok\": " + String(hash_ok) + ", \"hash-dup\": " + String(hash_dup) + ", \"crc-errors\": " + String(crc_errors) + "\", \"RF-count\": " + String(rf_n) + ", \"MQTT-count\": " + String(mqtt_n) + " }";
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", response_text);
    request->send(response);
  });

  http_server->on("/version", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response_text = "{ \"board\": \"" + String(SELBOARD) + "\", \"GIT-hash\": \"" + String(AUTO_VERSION) + "\", \"build-on\": \"" + String(__DATE__ " " __TIME__) +  "\" }";
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", response_text);
    request->send(response);
  });

  http_server->begin();

  if (!MDNS.begin(sys_id))
    Serial.println(F("Failed initializing MDNS"));
  MDNS.addService("http", "tcp", 80);
}

void setupOTA() {
  ArduinoOTA.setHostname(sys_id);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
      Serial.println(F("Start"));
      });
  ArduinoOTA.onEnd([]() {
      Serial.println(F(""));
      Serial.println(F("End"));
      });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      });
  ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println(F("Auth Failed"));
      else if (error == OTA_BEGIN_ERROR) Serial.println(F("Begin Failed"));
      else if (error == OTA_CONNECT_ERROR) Serial.println(F("Connect Failed"));
      else if (error == OTA_RECEIVE_ERROR) Serial.println(F("Receive Failed"));
      else if (error == OTA_END_ERROR) Serial.println(F("End Failed"));
      });
  ArduinoOTA.begin();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println(F("Selected board: " SELBOARD));
  Serial.println(F("Git hash      : " AUTO_VERSION));
  Serial.println(F("Built on      : " __DATE__ " " __TIME__));

#if defined(HAS_DISPLAY)
  Wire.begin(PIN_SDA, PIN_SCL);
#if defined(DISPTYPE_SH110x)
  disp_rc = display.begin(DISP_I2C_PORT, true);
#else
  disp_rc = display.begin(SSD1306_SWITCHCAPVCC, DISP_I2C_PORT, true, false);
#endif
  if (disp_rc) {
    display.setTextSize(2);
#if defined(DISPTYPE_SH110x)
    display.setTextColor(SH110X_WHITE);
#else
    display.setTextColor(SSD1306_WHITE);
#endif
    display.cp437(true);
    disp_text("INIT");
  }
  else {
    Serial.print(F("Cannot initialize display"));
  }
#endif

#if defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  Serial.print(F("[SX12xx] Initializing... "));
#if defined(T_BEAM_SUPREME)
  spi.begin(12, 13, 11, 10);
#endif
  auto state = radio.begin(CARRIER_FREQ, BANDWIDTH, SF, CR, SYNC_WORD, POWER, PREAMBLE);
  if (state == RADIOLIB_ERR_NONE)
    Serial.println(F("success!"));
  else
    failed_reboot("radio err");

  if (radio.setCRC(USE_CRC) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
    failed_reboot("radio setup failed");

  uint8_t mac[8];
  esp_efuse_mac_get_default(mac);
  Serial.print(F("MAC: "));
  for(int i=0; i<6; i++) {
    if (i)
      Serial.print(':');
    Serial.print(mac[i], HEX);
  }
  snprintf(sys_id, sizeof sys_id, "LoRaBr-%02x%02x%02x", mac[3], mac[4], mac[5]);
  Serial.print(F(", system ID: "));
  Serial.println(sys_id);

  disp_text("WiFi");
  WiFiManager wm;
  wm.setHostname(sys_id);
  wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT);
  wm.setConnectTimeout(WIFI_CONNECT_TIMEOUT);
  if (!wm.autoConnect(sys_id))
    failed_reboot("WiFi start fail");

  setupOTA();

  setup_http_server();

  radio.setPacketReceivedAction(set_rf_recv_flag);
  start_rf_receive();

  asprintf(&mqtt_sub_topic, "%s/#",  MQTT_TOPIC);
  asprintf(&mqtt_pub_topic, "%s/%s", MQTT_TOPIC, sys_id);

  xTaskCreatePinnedToCore(mqtt_thread, "mqtt", 10000, nullptr, 0, &mqtt_handle, 0);
#if defined(HAS_DISPLAY)
  xTaskCreatePinnedToCore(disp_thread, "display", 2048, nullptr, 0, &disp_handle, 0);
#endif

#if defined(USER_BUTTON)
  pinMode(USER_BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(USER_BUTTON), button_clicked, RISING);
#endif

  disp_text("GO!");
}

void rf_transmit(const uint8_t *const pl, const size_t len) {
  set_builtin_led(HIGH);
  int state = radio.transmit(pl, len);
  if (state == RADIOLIB_ERR_NONE) {
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("rf packet too long"));
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("rf timeout"));
  }
  else {
    // some other error occurred
    Serial.print(F("rf transmission failed, code: "));
    Serial.println(state);
  }
  set_builtin_led(LOW);
}

void show_statistics() {
#if defined(HAS_DISPLAY)
  if (disp_rc) {
    disp_since = millis();
    set_disp_state(true);
    display.clearDisplay();
    display.setCursor(0, 0);
    char buffer[16] { };
    snprintf(buffer, sizeof buffer, "%d", millis() / 1000);
    display.println(buffer);
    snprintf(buffer, sizeof buffer, "%d", hash_ok);
    display.println(buffer);
    display.display();
  }
#endif
}

void loop() {
  if (rf_received.exchange(false)) {
    int num_bytes = radio.getPacketLength();
    int state     = radio.readData(rf_buffer, num_bytes);
    if (num_bytes == 0)
      Serial.println(F("RF ignoring empty msg"));
    else if (num_bytes > sizeof(rf_buffer)) {
      Serial.print(F("RF truncated: "));
      Serial.print(num_bytes - sizeof(rf_buffer));
      Serial.println(F(" too short"));
    }
    else {
      if (state == RADIOLIB_ERR_NONE) {
        // store in mqtt transmit queue
        std::unique_lock<std::mutex> lck(mqtt_send_lock);
        memcpy(mqtt_send_entries[n_mqtt_send_entries].buffer, rf_buffer, num_bytes);
        mqtt_send_entries[n_mqtt_send_entries].n = num_bytes;
        n_mqtt_send_entries++;
        mqtt_send_cv.notify_one();
        rf_n++;
      }
      else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        crc_errors++;
        Serial.println(F("CRC mismatch"));
      }
      else {
        Serial.print(F("recv failed: "));
        Serial.println(state);
      }
    }
  }

  // transmit any message that came in via mqtt
  std::unique_lock<std::mutex> lck(mqtt_recv_lock);
  bool any_mqtt = n_mqtt_recv_entries > 0;
  for(int i=0; i<n_mqtt_recv_entries; i++)
      rf_transmit(mqtt_recv_entries[i].buffer, mqtt_recv_entries[i].n);
  n_mqtt_recv_entries = 0;
  lck.unlock();

  if (any_mqtt)
    start_rf_receive();

#if defined(HAS_DISPLAY)
  if (button_pressed.exchange(false))
    show_statistics();
#endif

  ArduinoOTA.handle();
}
