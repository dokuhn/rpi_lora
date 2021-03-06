/**
 * @file main.cpp
 * @author Dominik Kuhn (dominik.kuhn90@googlemail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-23
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

#include "MQTTDataStreamer.hpp"

#include "sx1276_old.hpp"

#include <boost/program_options.hpp>
namespace po = boost::program_options;


extern "C" {

 //  #include <errno.h>
    #include <wiringPi.h>
    #include <wiringPiSPI.h>
}

using namespace std;

const std::string DFLT_SERVER_ADDRESS	{ "tcp://dompfaf:1883" };
const std::string CLIENT_ID				{ "paho_cpp_async_publish" };
const std::string PERSIST_DIR			{ "./persist" };


const std::string TOPIC { "/rpi_lora" };
const char* LWT_PAYLOAD = "Last will and testament.";

const std::string TEST_MSG {"Hallo Welt das ist ein fucking test !!!!"};

uint8_t QOS = 1;

const auto TIMEOUT = std::chrono::seconds(10);

static sx1276_old sx1276Inst;


/////////////////////////////////////////////////////////////////////////////


// int readline(int fd, char *buf, int nbytes)
// {
//     int numread = 0;
//     int returnval;
//     while (numread < nbytes - 1)
//     {
//         returnval = read(fd, buf + numread, 1);
//         if ((returnval == -1) && (errno == EINTR))
//             continue;
//         if ((returnval == 0) && (numread == 0))
//             return 0;
//         if (returnval == 0)
//             break;
//         if (returnval == -1)
//             return -1;
//         numread++;
//         if (buf[numread - 1] == '\n')
//         {
//             buf[numread] = '\0';
//             return numread;
//         }
//     }
//     errno = EINVAL;
//     return -1;
// }


extern "C" void isr_handler_wrapper(void)
{        
    
    sx1276Inst.isr_handler();

}

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
            if(topic->name == "/LoRa_test/transmitPacket/") {
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

        sx1276Inst.txlora( (unsigned char*)msg, (unsigned char)msg_len );




    }    
};


int main (int argc, char *argv[]) {

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("rec",  "setup receive mode")
        ("sender", "setup transmit mode")
        ("half-duplex", "setup half-duplex mode")
    ;


    po::variables_map vm;        
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    


    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }



    if (vm.count("sender")) {

        std::cout << "Initializing and connecting for server '" << DFLT_SERVER_ADDRESS << "'..." << std::endl;

        std::vector<std::shared_ptr<TopicsToHandle>> topics_to_handle;
        topics_to_handle.push_back(std::make_shared<DataTransmitTopic>(
                    "/LoRa_test/transmitPacket/")); 

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

        sx1276Inst.SetupLoRa();
        
        // enter standby mode (required for FIFO loading))
        sx1276Inst.opmode(OPMODE_STANDBY);

        sx1276Inst.writeReg(RegPaRamp, (sx1276Inst.readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

        sx1276Inst.configPower(23);

        std::printf("Send packets at SF%i on %.6lf Mhz.\n", sx1276Inst.sf, (double)sx1276Inst.freq/1000000);
        std::cout << "------------------" << endl;

        std::printf("operation mode: %x \n", sx1276Inst.readReg(REG_OPMODE));
        std::printf("LoRa Sync Word: %x \n", sx1276Inst.readReg(REG_SYNC_WORD));
        std::printf("REG_MODEM_CONFIG: %x \t REG_MODEM_CONFIG2: %x \t REG_MODEM_CONFIG3: %x \n", 
                sx1276Inst.readReg(REG_MODEM_CONFIG),
                sx1276Inst.readReg(REG_MODEM_CONFIG2),
                sx1276Inst.readReg(REG_MODEM_CONFIG3));

        printf("REG_FRF_MSB: %x \t REG_FRF_MID: %x \t REG_FRF_LSB: %x \n",
               sx1276Inst.readReg(REG_FRF_MSB),sx1276Inst.readReg(REG_FRF_MID),sx1276Inst.readReg(REG_FRF_LSB));


        std::printf("REG_SYMB_TIMEOUT_LSB: %x \t REG_PREAMBLEMSB: %x \t REG_PREAMBLELSB: %x \n",
                sx1276Inst.readReg(REG_SYMB_TIMEOUT_LSB),
                sx1276Inst.readReg(REG_PREAMBLEMSB),
                sx1276Inst.readReg(REG_PREAMBLELSB));


        
        std::thread handleTopicsThread(handleTopics, streamer_obj, topics_to_handle, &mut);

        while( 1 ){

            // sx1276Inst.txlora((byte*)line_buffer, (byte)std::strlen(line_buffer));

            std::this_thread::sleep_for(std::chrono::milliseconds(5000) );
        }


        

    } else if(vm.count("rec")) {

        std::cout << "Initializing and connecting for server '" << DFLT_SERVER_ADDRESS << "'..." << std::endl;

        std::vector<std::shared_ptr<TopicsToHandle>> topics_to_handle;
        topics_to_handle.push_back(std::make_shared<DataTransmitTopic>(
                    "/LoRa_test/transmitPacket/")); 

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
        
        sx1276Inst.SetupLoRa();

        sx1276Inst.init_streamer_obj(streamer_obj, &mut);

        // radio init
        sx1276Inst.opmodeLora();
        sx1276Inst.opmode(OPMODE_STANDBY);
        sx1276Inst.opmode(OPMODE_RX);
        std::printf("Listening at SF%i on %.6lf Mhz.\n", sx1276Inst.sf,(double)sx1276Inst.freq/1000000);
        std::cout << "------------------" << endl;
        

        std::printf("operation mode: %x \n", sx1276Inst.readReg(REG_OPMODE));
        std::printf("LoRa Sync Word: %x \n", sx1276Inst.readReg(REG_SYNC_WORD));
        std::printf("RegModemConfig1: %x \t RegModemConfig2: %x \t RegModemConfig3: %x \n", 
                sx1276Inst.readReg(REG_MODEM_CONFIG),
                sx1276Inst.readReg(REG_MODEM_CONFIG2),
                sx1276Inst.readReg(REG_MODEM_CONFIG3));

        printf("REG_FRF_MSB: %x \t REG_FRF_MID: %x \t REG_FRF_LSB: %x \n",
               sx1276Inst.readReg(REG_FRF_MSB),sx1276Inst.readReg(REG_FRF_MID),sx1276Inst.readReg(REG_FRF_LSB));


        std::printf("REG_LR_SYMBTIMEOUTLSB: %x \t REG_LR_PREAMBLEMSB: %x \t REG_LR_PREAMBLELSB: %x \n",
                sx1276Inst.readReg(REG_SYMB_TIMEOUT_LSB),
                sx1276Inst.readReg(REG_PREAMBLEMSB),
                sx1276Inst.readReg(REG_PREAMBLELSB));


        

	    void (*fun_ptr2isr_handler)(void) = &isr_handler_wrapper;

	    wiringPiISR(sx1276Inst.dio0, INT_EDGE_RISING, fun_ptr2isr_handler);


        while(1) {
            // sx1276Inst.receivepacket();
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }




    } else if(vm.count("half-duplex")) {

        std::cout << "half-duplex mode is not implemented yet" << std::endl;
        return 0;

    } else {
        std::cout << "nothing setup" << std::endl;
        return 1;
    }



    // int buffer_size = 256;
    // char line_buffer[buffer_size];

    // if (argc < 2) {
    //     std::cout << "Usage: argv[0] sender|rec [message]" << endl;
    //     exit(1);
    // }


    // callback cb;
    // mqtt_async_client->set_callback(cb);

    // auto connOpts = mqtt::connect_options_builder()
	// 	.clean_session()
	// 	.will(mqtt::message(TOPIC, LWT_PAYLOAD, QOS))
	// 	.finalize();

    
    
    // mutex object used to make the publishMessage() in MQTTDataStreamer
    // library thread safe.
  

    // cout << "\nConnecting..." << endl;
    // mqtt::token_ptr conntok = mqtt_async_client->connect(connOpts);
    // cout << "Waiting for the connection..." << endl;
    // conntok->wait();
    // cout << "  ...OK" << endl;


    // auto msg = mqtt::make_message(TOPIC, "Hallo Welt !!!");
    // msg->set_qos(QOS);

    // std::size_t buffer_len = TEST_MSG.size();

    // std::string debug_info = "Sending messages ...";

    

    // while(1) {
            
    //         streamer_obj->publishMessage(TEST_MSG.data(), buffer_len, 
    //                     TOPIC, 1,  &mut, debug_info);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // }

   

    // if (!strcmp("sender", argv[1])) {
        
	    
    //     //if (argc > 2)
    //     //    strncpy((char *)hello, argv[2], sizeof(hello));

    //     // while( readline(STDIN_FILENO, line_buffer, sizeof(line_buffer)) > 1 ){
        
        
    // } else {

        
    // }

    return 0;
}
