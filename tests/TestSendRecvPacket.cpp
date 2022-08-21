/**
 * @file main.cpp
 * @author Dominik Kuhn (dominik.kuhn90@googlemail.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>
#include <cstring>
#include <cstdio>

#include "../MQTTDataStreamer.hpp"
#include "../sx1276.hpp"

#include <boost/program_options.hpp>
namespace po = boost::program_options;


extern "C" {

 //  #include <errno.h>
    #include <wiringPi.h>
    #include <wiringPiSPI.h>
}

using namespace std;

static sx1276 sx1276Inst;

const std::string DFLT_SERVER_ADDRESS	{ "tcp://localhost:1883" };
const std::string CLIENT_ID		{ "paho_cpp_async_publish" };
const std::string PERSIST_DIR		{ "./persist" };

const char* LWT_PAYLOAD = "Last will and testament.";

uint8_t QOS = 1;

const auto TIMEOUT = std::chrono::seconds(1);


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

#define RX_TIMEOUT_VALUE                            3500      // in ms
#define BUFFER_SIZE                                 128       // Define the payload size here


void handleTopics(std::shared_ptr<MQTTDataStreamer> streamer_obj,
        const std::vector<std::shared_ptr<TopicsToHandle>>& topics_to_handle,
        std::mutex* mut) {
    /*
     * This function runs in a separate thread. Here you can 
     * write down the logic to be executed when a message is sent over a 
     * subscribed topic. Subscription happens via Callback described in 
     * HelperClasses.hh file in MQTTDataStreamer library. 
     * Messages over subscribed topics are mainly supposed 
     * to act as a trigger. If you want to perform
     * actions on the sent msg, it is currently only possible in the HelperClasses.hh
     * This boundary is rather limiting and it require a rewrite of MQTTDataStreamer 
     * library(which is highly needed in my(Nirmal) opinion)
     */
    while(true) {
        for(const auto& topic : topics_to_handle) {
            if(topic->name == "LoRa_test/transmitPacket/") {
                if(topic->message_received){

                    topic->message_received = false;
                }
            }
            else {
                std::cout << "\tTopic '" << topic->name << "' not handled\n"; 
                exit(1);
            }
        }
    }
}



class DataTransmitTopic : public virtual TopicsToHandle {
    /*
     * This class is used provided as a means to do something
     * with the messages sent over subscribed messages. It requires
     * changing processMessage() to processMessage(const_message_ptr).
     * This can be theoretically done and boundary between MQTTDataStreamer
     * and main() can be broken by declaring static variables in this
     * Translation Unit. 
     */
public:
    DataTransmitTopic(const std::string& name,
            uint8_t QoS = 1) : 
        TopicsToHandle(name, QoS) {} 
    void processMessage(mqtt::const_message_ptr msg_) override {
        message_received = true;

        std::size_t msg_len = msg_->to_string().size();
        char * msg = new char[msg_len + 1];
        std::strcpy (msg, msg_->to_string().c_str());

	sx1276Inst.send( (unsigned char*)msg , (unsigned char)msg_len );

        std::cout << "sending packet ..." << std::endl;

	sx1276Inst.receive();


    }
};


int main (int argc, char *argv[]) {

    uint8_t recvBuffer[BUFFER_SIZE];
    uint8_t receivedbytes;


    /*
    sx1276Inst.SetupLoRa();

    sx1276Inst.opmodeLora();
    // enter standby mode (required for FIFO loading))
    sx1276Inst.opmode(OPMODE_STANDBY);

    sx1276Inst.writeReg(RegPaRamp, (sx1276Inst.readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

    sx1276Inst.configPower(23);
    */

    std::cout << "Initializing and connecting for server '" << DFLT_SERVER_ADDRESS << "'..." << std::endl;

    std::vector<std::shared_ptr<TopicsToHandle>> topics_to_handle;
    topics_to_handle.push_back(std::make_shared<DataTransmitTopic>(
                               "LoRa_test/transmitPacket/")); 

    auto mqtt_async_client = std::make_shared<mqtt::async_client>(
                              DFLT_SERVER_ADDRESS, CLIENT_ID);

    auto callback = std::make_shared<MqttCallback>
                    (mqtt_async_client, topics_to_handle);

    auto streamer_obj = std::make_shared<MQTTDataStreamer>(
                        std::make_tuple(mqtt_async_client, callback));

    std::mutex mut;
    std::cout << "  ...OK" << endl;


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


    sx1276Inst.set_tx_config( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                         LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                         LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP, 
                         LORA_IQ_INVERSION, 2000000 );

    sx1276Inst.set_public_network(true);

    sx1276Inst.receive();

    std::printf("Send and receive packets on %.6lf Mhz.\n", (double)RF_FREQUENCY/1000000);
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
        // sx1276Inst.receivepacket();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // sx1276Inst.send(sendBuffer, sizeof(sendBuffer));
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

       	        std::printf("payload: ");

		for(int i = 0; i < receivedCount; i++)
		{
		    std::printf("%02hhX", recvBuffer[i]);
		}
		std::printf("\n");


		std::string debug_info = "Sending messages ...";
                // std::size_t payload_len = strlen(recvBuffer);
                streamer_obj->publishMessage( recvBuffer, receivedCount, 
                            "LoRa_test/receivedPacket/", 1,  &mut, debug_info);



            }

            irqflags = sx1276Inst.read_register(REG_LR_IRQFLAGS);
            std::printf("irgflags: %x \n", irqflags );

            std::cout << "debug message !!!" << std::endl;


        }

    }

    return 0;

}

