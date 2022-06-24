# RaspberryPi LoRa (SX1276) to MQTT Bridge  

This repository contains a simple proof-of-concept implementation of a single channel (receive or send at the same time) 
LoRa Bridge communicating with the IoT world via MQTT protocol. Packets received via LoRa are published on a specific MQTT topic,
in this example LoRa_test/receivedPacket/. At the same time, this bridge software subscribes to another topic to be able to send 
data via LoRa. Due to the used SX1276 chip only one channel can be demodulated and received at the same time, i.e. in the EU a 
frequency like 868.1 MHz as a band in the frequency range 863-870 MHz.

It has been tested on the Raspberry Pi 3 platform, using a Semtech SX1276 transceiver (HopeRF RFM95W).

Maintained: Dominik Kuhn <dominik.kuhn90@googlemail.com>

## ToDo

- development of pseudo "half-duplex" receive and transmit functions
- extension of the rudimentary program options with standardized linux like parameters like the logging-level, location of the log file etc.

## Installation

The following dependencies must be installed

1. paho-mqtt-c
2. paho-mqtt-cpp
3. wiring-pi
4. boost-program-options

### Installing paho-mqtt-c

```bash
git clone https://github.com/eclipse/paho.mqtt.c.git

cd paho.mqtt.c

cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON \
        -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON

cmake --build build/ --target install
```

### Installing paho-mqtt-cpp

```bash
git clone https://github.com/eclipse/paho.mqtt.cpp

cd paho.mqtt.cpp

cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON \
        -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE

cmake --build build/ --target install
```

### Installing wiring-pi

```bash
sudo apt get install wiringpi
```

### Installing boost-program-options

<pre>
Needs input, see <a href="https://github.com/boostorg/wiki/wiki/Getting-Started%3A-Overview">boost getting started</a>
</pre>
