/*******************************************************************************
 *
 * 
 *
 * 
 *
 *******************************************************************************/

#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>
#include <cstring>
#include <cstdio>

#include "MQTTDataStreamer.hpp"

#include "sx1276.hpp"

extern "C" {

 //  #include <errno.h>
    #include <wiringPi.h>
    #include <wiringPiSPI.h>
}

using namespace std;

const std::string DFLT_SERVER_ADDRESS	{ "tcp://troubadix:1883" };
const std::string CLIENT_ID				{ "paho_cpp_async_publish" };
const std::string PERSIST_DIR			{ "./persist" };


const std::string TOPIC { "/rpi_lora" };
const char* LWT_PAYLOAD = "Last will and testament.";

const std::string TEST_MSG {"Hallo Welt das ist ein fucking test !!!!"};

uint8_t QOS = 1;

const auto TIMEOUT = std::chrono::seconds(10);

static sx1276 sx1276Inst;



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

class DataTransmitTopic : public virtual TopicsToHandle {
    /*
     * This class is used provided as a means to do something
     * with the messages sent over subscribed messages. It requires
     * changing processMessage() to processMessage(const_message_ptr).
     * This can be theoretically done and boundary between MQTTKameraStreamer
     * and main() can be broken by declaring static variables in this
     * Translation Unit. 
     */
public:
    DataTransmitTopic(const std::string& name,
            uint8_t QoS = 1) : 
        TopicsToHandle(name, QoS) {} 
    void processMessage() override {
        message_received = false;
    }    
};


int main (int argc, char *argv[]) {

    // int buffer_size = 256;
    // char line_buffer[buffer_size];

    if (argc < 2) {
        std::cout << "Usage: argv[0] sender|rec [message]" << endl;
        exit(1);
    }

    std::cout << "Initializing and connecting for server '" << DFLT_SERVER_ADDRESS << "'..." << endl;

    // callback cb;
    // mqtt_async_client->set_callback(cb);

    // auto connOpts = mqtt::connect_options_builder()
	// 	.clean_session()
	// 	.will(mqtt::message(TOPIC, LWT_PAYLOAD, QOS))
	// 	.finalize();

    std::vector<std::shared_ptr<TopicsToHandle>> topics_to_handle;
    topics_to_handle.push_back(std::make_shared<DataTransmitTopic>(
                TOPIC)); 

    auto mqtt_async_client = std::make_shared<mqtt::async_client>(
            DFLT_SERVER_ADDRESS, CLIENT_ID);

    auto callback = std::make_shared<MqttCallback>
            (mqtt_async_client, topics_to_handle);

    auto streamer_obj = std::make_shared<MQTTDataStreamer>(
                            std::make_tuple(mqtt_async_client, callback));

    
    // mutex object used to make the publishMessage() in MQTTDataStreamer
    // library thread safe.
    std::mutex mut;

    std::cout << "  ...OK" << endl;

    // cout << "\nConnecting..." << endl;
    // mqtt::token_ptr conntok = mqtt_async_client->connect(connOpts);
    // cout << "Waiting for the connection..." << endl;
    // conntok->wait();
    // cout << "  ...OK" << endl;


    // auto msg = mqtt::make_message(TOPIC, "Hallo Welt !!!");
    // msg->set_qos(QOS);

    std::size_t buffer_len = TEST_MSG.size();

    std::string debug_info = "Sending messages ...";

    // wiringPiSetup () ;
    // pinMode(sx1276Inst.ssPin, OUTPUT);
    // pinMode(sx1276Inst.dio0, INPUT);
    // pinMode(sx1276Inst.RST, OUTPUT);

    // wiringPiSPISetup(sx1276Inst.CHANNEL, 500000);

    // sx1276Inst.SetupLoRa();


    while(1) {
            
            streamer_obj->publishMessage(TEST_MSG.data(), buffer_len, 
                        TOPIC, 1,  &mut, debug_info);
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }


    // if (!strcmp("sender", argv[1])) {
    //     sx1276Inst.opmodeLora();
    //     // enter standby mode (required for FIFO loading))
    //     sx1276Inst.opmode(OPMODE_STANDBY);

    //     sx1276Inst.writeReg(RegPaRamp, (sx1276Inst.readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

    //     sx1276Inst.configPower(23);

    //     std::printf("Send packets at SF%i on %.6lf Mhz.\n", sx1276Inst.sf, (double)sx1276Inst.freq/1000000);
    //     std::cout << "------------------" << endl;
	    
    //     //if (argc > 2)
    //     //    strncpy((char *)hello, argv[2], sizeof(hello));

    //     // while( readline(STDIN_FILENO, line_buffer, sizeof(line_buffer)) > 1 ){
        
    //     while( 1 ){

    //         // sx1276Inst.txlora((byte*)line_buffer, (byte)std::strlen(line_buffer));

    //         std::this_thread::sleep_for(std::chrono::milliseconds(500) );
    //     }

    // } else {

    //     // radio init
    //     sx1276Inst.opmodeLora();
    //     sx1276Inst.opmode(OPMODE_STANDBY);
    //     sx1276Inst.opmode(OPMODE_RX);
    //     std::printf("Listening at SF%i on %.6lf Mhz.\n", sx1276Inst.sf,(double)sx1276Inst.freq/1000000);
    //     std::cout << "------------------" << endl;
        
	//     void (*fun_ptr2isr_handler)(void) = &isr_handler_wrapper;

	//     wiringPiISR(sx1276Inst.dio0, INT_EDGE_RISING, fun_ptr2isr_handler);

    //     while(1) {
    //         // sx1276Inst.receivepacket();
    //         streamer_obj->publishMessage(TEST_MSG, buffer_len, 
    //                     TOPIC , 1, &mut, debug_info);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    //     }

    // }

    return (0);
}
