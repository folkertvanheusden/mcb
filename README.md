what it is
----------
This program allows you to bridge a LoRa setup over a longer distance by using MQTT over e.g. an IP network.
Use-case: extending the reach of MeshCore by bridging it over HAM-Net.
It works transparently: you only need to know LoRa settings and then it should be able to bridge any protocol.


## required

* an SX1262 radio connected to an ESP32-S3
* platformio


## configuration

* For it to work you must adjust mcb.ino (e.g. pin numbers, topic, mqtt server, LoRa settings).


## usage

* flash it:
```bash
pio run -t upload -e X
```
X shall be either xiao_headers (sx1262 via dupont cables) or xiao_connector (using the special connector on the xiao board: https://www.antratek.nl/xiao-esp32s3-for-meshtastic-lora).
* Connect to the wifi station called "LoRaBridge" and configure its WiFi connection, that's it!
It will blink the LED when packets are seen (on MQTT or RF).


## see also

* https://github.com/folkertvanheusden/lora-pipe  (Raspberry Pi version)


## license

MIT


-- written by Folkert van Heusden <folkert@vanheusden.com>
