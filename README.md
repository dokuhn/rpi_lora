# RaspberryPi LoRaWAN (SX1276) to MQTT Gateway  

This repository contains a simple proof-of-concept implementation of a single channel (receive or send at the same time) LoRaWAN gateway communicating with the IoT world via MQTT protocol.

It has been tested on the Raspberry Pi 3 platform, using a Semtech SX1276 transceiver (HopeRF RFM95W).

Maintainer: Dominik Kuhn <dominik.kuhn90@googlemail.com>

## ToDo

- development of pseudo "half-duplex" receive and transmit functions
- extension of the rudimentary program options with standardized linux like parameters like the logging-level, location of the log file etc.
