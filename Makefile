# dragino lora testing
# Single lora testing app

CC=gcc
CFLAGS=-Wall
LDFLAGS=-lwiringPi
OBJFILES = main.o sx1276.o
TARGET = dragino_lora_app

all: $(TARGET)

$(TARGET): $(OBJFILES)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJFILES) $(LDFLAGS)

clean:
	rm -f $(OBJFILES) $(TARGET) *~




