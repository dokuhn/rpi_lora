# dragino lora testing
# Single lora testing app

CC = g++
CFLAGS = -Wall
INCLUDES = /home/pi/paho.mqtt.cpp/src
LDFLAGS = -lwiringPi -lpaho-mqttpp3 -lpaho-mqtt3as
OBJFILES = sx1276.o main.o
TARGET = dragino_lora_app

all: $(TARGET)

$(TARGET): $(OBJFILES)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJFILES) $(LDFLAGS)

# $(TARGET): $(OBJFILES)
# 	$(CC) -o $(TARGET) $(OBJFILES) $(LDFLAGS)

# sx1276.o: sx1276.cpp
# 	$(CC) -I $(INCLUDES) sx1276.cpp

# main.o: main.cpp
# 	$(CC) -I $(INCLUDES) main.cpp

clean:
	rm -f $(OBJFILES) $(TARGET) *~




