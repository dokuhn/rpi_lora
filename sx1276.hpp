/**
 * @file sx1276.hpp
 * @author Dominik Kuhn (dominik.kuhn90@googlemail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __SX1276_H__
#define __SX1276_H__

// #include "sx1276_RegsLoRa.hpp"

extern "C" {

    # include <stdio.h>
    # include <sys/types.h>
    # include <string.h>
    # include <sys/time.h>
    # include <unistd.h>
    # include <stdlib.h>
    # include <stdbool.h>
    # include <stdint.h>

}


#include <memory>

#include "enums.hpp"
#include "sx1276_RegsFsk.hpp"
#include "sx1276_RegsLoRa.hpp"


#include "MQTTDataStreamer.hpp"


#define MAX_DATA_BUFFER_SIZE_SX1276  255
 

class sx1276 {
 
    public:

        const int CHANNEL = 0;

        // SX127X - Raspberry connections
        int ssPin = 6;
        int dio0  = 7;
        int RST   = 0;

        // Set center frequency
        uint32_t  freq = 868100000; // in Mhz! (868.1)

        void lock(void);

        void unlock(void);

        void init_radio();

        void radio_reset();

        uint8_t get_status(void);

        void set_channel(uint32_t freq);

        uint32_t random(void);

        void set_rx_config(RadioModems_t modem, uint32_t bandwidth,
                            uint32_t datarate, uint8_t coderate,
                            uint32_t bandwidth_afc,
                            uint16_t preamble_len,
                            uint16_t symb_timeout, bool fix_len,
                            uint8_t payload_len, bool crc_on,
                            bool freq_hop_on, uint8_t hop_period,
                            bool iq_inverted, bool rx_continuous);

        void set_tx_config(RadioModems_t modem, int8_t power,
                            uint32_t fdev, uint32_t bandwidth,
                            uint32_t datarate, uint8_t coderate,
                            uint16_t preamble_len, bool fix_len,
                            bool crc_on, bool freq_hop_on,
                            uint8_t hop_period, bool iq_inverted,
                            uint32_t timeout);


        uint32_t time_on_air(RadioModems_t modem, uint8_t pkt_len);

        void send(uint8_t *buffer, uint8_t size);

        void receive(void);


        bool perform_carrier_sense(RadioModems_t modem,
                                    uint32_t freq,
                                    int16_t rssi_threshold,
                                    uint32_t max_carrier_sense_time);

        void set_public_network(bool enable);

        void start_cad();

        /**
        *  Sets the maximum payload length
        *
        *  @param modem         Radio modem to be used [0: FSK, 1: LoRa]
        *  @param max           Maximum payload length in bytes
        */
        void set_max_payload_length(RadioModems_t modem, uint8_t max);

        void set_tx_continuous_wave(uint32_t freq, int8_t power,
                                    uint16_t time);

        void set_operation_mode(uint8_t mode);

        void set_modem(RadioModems_t modem);

        void setup_registers();

        uint8_t get_fsk_bw_reg_val(uint32_t bandwidth);

        uint8_t get_pa_conf_reg(uint32_t channel);

        void set_rf_tx_power(int8_t power);

        void transmit(uint32_t timeout);

        int16_t get_rssi(RadioModems_t modem);

        void set_low_power_mode();

        void set_antenna_switch(uint8_t mode);

        unsigned char read_register(unsigned char addr);

        void read_register(uint8_t addr, uint8_t *buffer, uint8_t size);

        void write_fifo(uint8_t *buffer, uint8_t size);

        void read_fifo(uint8_t *buffer, uint8_t size);

        void write_to_register(unsigned char addr, unsigned char value);

        void write_to_register(uint8_t addr, uint8_t *data, uint8_t size);
        
        void sleep(void);

        void standby(void);


    private:
    

        RadioSettings_t settings;


        // Structure containing all user and network specified settings
        // for radio module
        RadioSettings_t _rf_settings;

        // Data buffer used for both TX and RX
        // Size of this buffer is configurable via Mbed config system
        // Default is 255 bytes
        uint8_t _data_buffer[MAX_DATA_BUFFER_SIZE_SX1276];
       

        char message[256];

        std::shared_ptr<MQTTDataStreamer> streamer_obj;

        std::mutex mutex;


        std::mutex *mut;

        uint8_t radio_variant;

        /*******************************************************************************
        *
        * Configure these values!
        *
        *******************************************************************************/

        

};


#endif

