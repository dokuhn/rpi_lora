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
#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdio>

#include "mqtt/async_client.h"

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


const string TOPIC { "/rpi_lora" };
const char* LWT_PAYLOAD = "Last will and testament.";

const int  QOS = 1;

const auto TIMEOUT = std::chrono::seconds(10);

static sx1276 sx1276Inst;

mqtt::async_client client(DFLT_SERVER_ADDRESS, CLIENT_ID, PERSIST_DIR);



/////////////////////////////////////////////////////////////////////////////

/**
 * A callback class for use with the main MQTT client.
 */

class callback : public virtual mqtt::callback
{
public:
	void connection_lost(const string& cause) override {
		cout << "\nConnection lost" << endl;
		if (!cause.empty())
			cout << "\tcause: " << cause << endl;
	}

	void delivery_complete(mqtt::delivery_token_ptr tok) override {
		cout << "\tDelivery complete for token: "
			<< (tok ? tok->get_message_id() : -1) << endl;
	}
};

/////////////////////////////////////////////////////////////////////////////

/**
 * A base action listener.
 */

class action_listener : public virtual mqtt::iaction_listener
{
protected:
	void on_failure(const mqtt::token& tok) override {
		cout << "\tListener failure for token: "
			<< tok.get_message_id() << endl;
	}

	void on_success(const mqtt::token& tok) override {
		cout << "\tListener success for token: "
			<< tok.get_message_id() << endl;
	}
};

/////////////////////////////////////////////////////////////////////////////

/**
 * A derived action listener for publish events.
 */

class delivery_action_listener : public action_listener
{
	atomic<bool> done_;

	void on_failure(const mqtt::token& tok) override {
		action_listener::on_failure(tok);
		done_ = true;
	}

	void on_success(const mqtt::token& tok) override {
		action_listener::on_success(tok);
		done_ = true;
	}

public:
	delivery_action_listener() : done_(false) {}
	bool is_done() const { return done_; }
};

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
    
    // sx1276Inst.isr_handler();
    std::printf(  sx1276Inst.receivepacket() );

}



int main (int argc, char *argv[]) {

    int buffer_size = 256;
    char line_buffer[buffer_size];

    if (argc < 2) {
        std::cout << "Usage: argv[0] sender|rec [message]" << endl;
        exit(1);
    }

    std::cout << "Initializing for server '" << DFLT_SERVER_ADDRESS << "'..." << endl;

    callback cb;
    client.set_callback(cb);

    auto connOpts = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC, LWT_PAYLOAD, QOS))
		.finalize();

    cout << "  ...OK" << endl;

    cout << "\nConnecting..." << endl;
    mqtt::token_ptr conntok = client.connect(connOpts);
    cout << "Waiting for the connection..." << endl;
    conntok->wait();
    cout << "  ...OK" << endl;


    auto msg = mqtt::make_message(TOPIC, "Hallo Welt !!!");
    msg->set_qos(QOS);

    wiringPiSetup () ;
    pinMode(sx1276Inst.ssPin, OUTPUT);
    pinMode(sx1276Inst.dio0, INPUT);
    pinMode(sx1276Inst.RST, OUTPUT);

    wiringPiSPISetup(sx1276Inst.CHANNEL, 500000);

    sx1276Inst.SetupLoRa();


    if (!strcmp("sender", argv[1])) {
        sx1276Inst.opmodeLora();
        // enter standby mode (required for FIFO loading))
        sx1276Inst.opmode(OPMODE_STANDBY);

        sx1276Inst.writeReg(RegPaRamp, (sx1276Inst.readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

        sx1276Inst.configPower(23);

        std::printf("Send packets at SF%i on %.6lf Mhz.\n", sx1276Inst.sf, (double)sx1276Inst.freq/1000000);
        std::cout << "------------------" << endl;
	    
        //if (argc > 2)
        //    strncpy((char *)hello, argv[2], sizeof(hello));

        // while( readline(STDIN_FILENO, line_buffer, sizeof(line_buffer)) > 1 ){
        
        while( 1 ){

            // sx1276Inst.txlora((byte*)line_buffer, (byte)std::strlen(line_buffer));

            std::this_thread::sleep_for(std::chrono::milliseconds(500) );
        }

    } else {

        // radio init
        sx1276Inst.opmodeLora();
        sx1276Inst.opmode(OPMODE_STANDBY);
        sx1276Inst.opmode(OPMODE_RX);
        std::printf("Listening at SF%i on %.6lf Mhz.\n", sx1276Inst.sf,(double)sx1276Inst.freq/1000000);
        std::cout << "------------------" << endl;
        
	    void (*fun_ptr2isr_handler)(void) = &isr_handler_wrapper;

	    wiringPiISR(sx1276Inst.dio0, INT_EDGE_RISING, fun_ptr2isr_handler);

        while(1) {
            // sx1276Inst.receivepacket();
            client.publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }

    }

    return (0);
}
