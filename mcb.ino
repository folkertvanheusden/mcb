#include <RadioLib.h>

/*
spi:
  clk_pin: 7
  mosi_pin: 9
  miso_pin: 8
sx126x:
  busy_pin: GPIO4
  cs_pin: GPIO5
  dio1_pin: GPIO2
  rst_pin: GPIO3
*/

// SX1262 has the following connections:
// NSS pin:   10
// DIO1 pin:  2
// NRST pin:  3
// BUSY pin:  9
SX1262 radio = new Module(5, 2, 3, 4);

void setup() {
  Serial.begin(115200);
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  }
  else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    for(;;) {
    }
  }

  Serial.print(F("[SX126x] Initializing ... "));
  // carrier frequency:           915.0 MHz
  // bandwidth:                   500.0 kHz
  // spreading factor:            6
  // coding rate:                 5
  // sync word:                   0x34 (public network/LoRaWAN)
  // output power:                2 dBm
  // preamble length:             20 symbols
  state = radio.begin(869.618, 62.5, 8, 8, 0x12, 22, 16);
  if (state == RADIOLIB_ERR_NONE)
    Serial.println(F("success!"));
  else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    for(;;) {
    }
  }

  if (radio.setCRC(true) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
    Serial.println(F("Selected CRC is invalid for this module!"));
    for(;;) {
    }
  }
}

void loop() {
  byte byteArr[] = {0x01, 0x23, 0x45, 0x56, 0x78, 0xAB, 0xCD, 0xEF};
  int state = radio.transmit(byteArr, 8);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(F("[SX1262] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("timeout!"));
  }
  else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  delay(11000);
}
