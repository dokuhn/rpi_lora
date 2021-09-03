# RaspberryPi LoRaWAN (SX1276) to MQTT Gateway  

This repository contains a simple proof-of-concept implementation of a single channel (receive or send at the same time) LoRaWAN gateway communicating with the IoT world via MQTT protocol.

It has been tested on the Raspberry Pi 3 platform, using a Semtech SX1276 transceiver (HopeRF RFM95W).

Maintainer: Dominik Kuhn <dominik.kuhn90@googlemail.com>

## ToDo

- development of pseudo "half-duplex" receive and transmit functions
- extension of the rudimentary program options with standardized linux like parameters like the logging-level, location of the log file etc.

# Installation
The following dependencies must be installed
    1. paho-mqtt-c
    2. paho-mqtt-cpp
    3. wiring-pi 
    4. boost-program-options
 
## Installing paho-mqtt-c
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON \
        -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON
cmake --build build/ --target install
 
## Installing paho-mqtt-cpp
git clone https://github.com/eclipse/paho.mqtt.cpp
cd paho.mqtt.cpp
cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON \
        -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE
cmake --build build/ --target install
 
## Installing wiring-pi
sudo apt get install wiringpi
 
## installing boost-program-options
Needs input