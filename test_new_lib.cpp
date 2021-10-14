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

#include "sx1276.hpp"

#include <boost/program_options.hpp>
namespace po = boost::program_options;


extern "C" {

 //  #include <errno.h>
    #include <wiringPi.h>
    #include <wiringPiSPI.h>
}

using namespace std;

const auto TIMEOUT = std::chrono::seconds(1);

static sx1276 sx1276Inst;


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

        wiringPiSetup () ;
        pinMode(sx1276Inst.ssPin, OUTPUT);
        pinMode(sx1276Inst.dio0, INPUT);
        pinMode(sx1276Inst.RST, OUTPUT);

        wiringPiSPISetup(sx1276Inst.CHANNEL, 500000);

        /* 
        sx1276Inst.SetupLoRa();

        sx1276Inst.opmodeLora();
        // enter standby mode (required for FIFO loading))
        sx1276Inst.opmode(OPMODE_STANDBY);

        sx1276Inst.writeReg(RegPaRamp, (sx1276Inst.readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

        sx1276Inst.configPower(23); 
        */

        std::printf("Send packets on %.6lf Mhz.\n", (double)sx1276Inst.freq/1000000);
        std::cout << "------------------" << endl;


        while( 1 ){

            // sx1276Inst.txlora((byte*)line_buffer, (byte)std::strlen(line_buffer));

            std::this_thread::sleep_for(std::chrono::milliseconds(2000) );
        }


        

    } else if(vm.count("rec")) {


        wiringPiSetup () ;
        pinMode(sx1276Inst.ssPin, OUTPUT);
        pinMode(sx1276Inst.dio0, INPUT);
        pinMode(sx1276Inst.RST, OUTPUT);

        wiringPiSPISetup(sx1276Inst.CHANNEL, 500000);
        
        /*   
        sx1276Inst.SetupLoRa();

        sx1276Inst.init_streamer_obj(streamer_obj, &mut);

        // radio init
        sx1276Inst.opmodeLora();
        sx1276Inst.opmode(OPMODE_STANDBY);
        sx1276Inst.opmode(OPMODE_RX);
        */

        std::printf("Listening on %.6lf Mhz.\n", (double)sx1276Inst.freq/1000000);
        std::cout << "------------------" << endl;
        

        while(1) {
            // sx1276Inst.receivepacket();
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
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
