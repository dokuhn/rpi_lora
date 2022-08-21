/**
 * @file test_new_lib.cpp
 * @author Dominik Kuhn (dominik.kuhn90@googlemail.com)
 * @brief
 * @version 0.1
 * @date 2021-10-15
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>
#include <cstring>
#include <cstdio>

#include "sx1276.hpp"

extern "C" {

 //  #include <errno.h>
    #include <wiringPi.h>
    #include <wiringPiSPI.h>
}

using namespace std;

const auto TIMEOUT = std::chrono::seconds(1);

static sx1276 sx1276Inst;


#define RF_FREQUENCY                                868100000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#define LORA_BANDWIDTH                              0       // [0: 125 kHz,
                                                            //  1: 250 kHz,
                                                            //  2: 500 kHz,
                                                            //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12      // [SF7..SF12]
#define LORA_CODINGRATE                             1       // [1: 4/5,
                                                            //  2: 4/6,
                                                            //  3: 4/7,
                                                            //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8       // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         8       // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_FHSS_ENABLED                           true
#define LORA_NB_SYMB_HOP                            0xFF
#define LORA_IQ_INVERSION                           false
#define LORA_CRC_ENABLED                            true

#define RX_TIMEOUT_VALUE                                3500      // in ms
#define BUFFER_SIZE                                     128       // Define the payload size here




int main (int argc, char *argv[]) {

    uint8_t recvBuffer[BUFFER_SIZE];
    uint8_t receivedbytes;


    wiringPiSetup () ;
    pinMode(sx1276Inst.ssPin, OUTPUT);
    pinMode(sx1276Inst.dio0, INPUT);
    pinMode(sx1276Inst.RST, OUTPUT);
    wiringPiSPISetup(sx1276Inst.CHANNEL, 500000);

    sx1276Inst.radio_reset();

    std::printf("LORA chip with version %x found.\n", (int)sx1276Inst.read_register(REG_LR_VERSION) );

    sx1276Inst.init_radio();

    sx1276Inst.sleep();
    sx1276Inst.set_channel(RF_FREQUENCY);

    sx1276Inst.set_rx_config( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                            LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                            LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, BUFFER_SIZE,
                            LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                            LORA_IQ_INVERSION, true );

    sx1276Inst.set_public_network(true);
    sx1276Inst.receive();

    std::printf("Listening on %.6lf Mhz.\n", (double)RF_FREQUENCY/1000000);
    std::cout << "------------------" << endl;

    std::printf("operation mode: %x \n", sx1276Inst.read_register(REG_OPMODE));
    std::printf("LoRa Sync Word: %x \n", sx1276Inst.read_register(REG_LR_SYNCWORD));
    std::printf("RegModemConfig1: %x \t RegModemConfig2: %x \t RegModemConfig3: %x \n", 
                sx1276Inst.read_register(REG_LR_MODEMCONFIG1),
                sx1276Inst.read_register(REG_LR_MODEMCONFIG2),
                sx1276Inst.read_register(REG_LR_MODEMCONFIG3));

    std::printf("REG_FRFMSB: %x \t REG_FRFMID: %x \t REG_FRFLSB: %x \n",
                sx1276Inst.read_register(REG_FRFMSB),
                sx1276Inst.read_register(REG_FRFMID),
                sx1276Inst.read_register(REG_FRFLSB));

    std::printf("REG_LR_SYMBTIMEOUTLSB: %x \t REG_LR_PREAMBLEMSB: %x \t REG_LR_PREAMBLELSB: %x \n",
                sx1276Inst.read_register(REG_LR_SYMBTIMEOUTLSB),
                sx1276Inst.read_register(REG_LR_PREAMBLEMSB),
                sx1276Inst.read_register(REG_LR_PREAMBLELSB));



    while(1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if(digitalRead(sx1276Inst.dio0) == 1){

            int irqflags = sx1276Inst.read_register(REG_LR_IRQFLAGS);
            std::printf("irgflags: %x \n", irqflags );

            //  payload crc: 0x20
            if((irqflags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) != 0u)
            {
                printf("CRC error\n");
                fflush(stdout);
                // clear CRC error
                sx1276Inst.write_to_register(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK);
            }

            if((irqflags & RFLR_IRQFLAGS_RXDONE_MASK) != 0u)
            {

                // clear rxDone
                sx1276Inst.write_to_register(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE_MASK);

                uint8_t currentAddr = sx1276Inst.read_register(REG_LR_FIFORXCURRENTADDR);
                uint8_t receivedCount = sx1276Inst.read_register(REG_LR_RXNBBYTES);
                receivedbytes = receivedCount;

                std::printf("received bytes: %d \n", receivedbytes );
                std::printf("current addr: %x \n", currentAddr);

                sx1276Inst.write_to_register(REG_LR_FIFOADDRPTR, currentAddr);

                for(int i = 0; i < receivedCount; i++)
                {
                    recvBuffer[i] = sx1276Inst.read_register(REG_LR_FIFO);
                }

                recvBuffer[receivedCount+1] = '\0';

                std::printf("payload: %s \n", recvBuffer);

            }

            irqflags = sx1276Inst.read_register(REG_LR_IRQFLAGS);
            std::printf("irgflags: %x \n", irqflags );

            std::cout << "debug message !!!" << std::endl;




        }

    }

    return 0;
}
