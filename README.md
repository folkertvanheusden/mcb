what it is
----------
This program allows you to bridge a LoRa setup over a longer distance by using MQTT over e.g. an IP network (not neccessarily Internet).
Use-case: extending the reach of MeshCore by bridging it over HAM-Net.
It works transparently: you only need to know LoRa settings and then it should be able to bridge any protocol.


## required

* one of:
  * Seeed XIAO ESP32-S3 with seperate SX1262
  * Seeed XIAO ESP32-S3 with Wio-SX1262
  * Heltec v3
  * t-beam v1.2
  * t-beam supreme
  * Lilygo t3
* platformio


## configuration

* For it to work you must adjust `config.h` (e.g. MQTT- and LoRa settings).


## usage

* flash it:
```bash
pio run -t upload -e X
```
X shall either `xiao_headers` (sx1262 via dupont cables), `xiao_connector` (the XIAO with the Wio-SX1262), `heltec_v3`, `t_beam_v1_2`, `lilygo_t3` or `t_beam_supreme`.
* Connect to the WiFi station called "`LoRaBr-xxxxxx`" and configure its WiFi connection, that's it! (`xxxxxx` is the unique identifier of the ESP32 MCU)
* It will blink the LED when packets are seen (on MQTT or RF).
* You can press the button on an Heltec v3 to see some statistics.

* it runs a HTTP REST server that can be reached on the IP-address shown in the serial console
* REST endpoints:
  * /status  - uptime & traffic statistics
  * /version - board type, GIT hash, build timestamp


## see also

* https://github.com/folkertvanheusden/lora-pipe  (Raspberry Pi version)


## license

MIT


-- written by Folkert van Heusden <folkert@vanheusden.com>
