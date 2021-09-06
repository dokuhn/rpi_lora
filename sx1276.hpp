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
 
// #############################################
// #############################################
 
# define REG_FIFO                    0x00
# define REG_OPMODE                  0x01
# define REG_FIFO_ADDR_PTR           0x0D
# define REG_FIFO_TX_BASE_AD         0x0E
# define REG_FIFO_RX_BASE_AD         0x0F
# define REG_RX_NB_BYTES             0x13
# define REG_FIFO_RX_CURRENT_ADDR    0x10
# define REG_IRQ_FLAGS               0x12
# define REG_DIO_MAPPING_1           0x40
# define REG_DIO_MAPPING_2           0x41
# define REG_MODEM_CONFIG            0x1D
# define REG_MODEM_CONFIG2           0x1E
# define REG_MODEM_CONFIG3           0x26
# define REG_SYMB_TIMEOUT_LSB 	    0x1F
# define REG_PKT_SNR_VALUE	    0x19
# define REG_PAYLOAD_LENGTH          0x22
# define REG_IRQ_FLAGS_MASK          0x11
# define REG_MAX_PAYLOAD_LENGTH 	    0x23
# define REG_HOP_PERIOD              0x24
# define REG_SYNC_WORD		    0x39
# define REG_VERSION	  	    0x42
 
# define PAYLOAD_LENGTH              0x40
 
// LOW NOISE AMPLIFIER
# define REG_LNA                     0x0C
# define LNA_MAX_GAIN                0x23
# define LNA_OFF_GAIN                0x00
# define LNA_LOW_GAIN		    0x20
 
# define RegDioMapping1              0x40 // common
# define RegDioMapping2              0x41 // common
 
# define RegPaConfig                 0x09 // common
# define RegPaRamp                   0x0A // common
# define RegPaDac                    0x5A // common
 
# define SX72_MC2_FSK                0x00
# define SX72_MC2_SF7                0x70
# define SX72_MC2_SF8                0x80
# define SX72_MC2_SF9                0x90
# define SX72_MC2_SF10               0xA0
# define SX72_MC2_SF11               0xB0
# define SX72_MC2_SF12               0xC0
 
# define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12
 
// sx1276 RegModemConfig1
# define SX1276_MC1_BW_125                0x70
# define SX1276_MC1_BW_250                0x80
# define SX1276_MC1_BW_500                0x90
# define SX1276_MC1_CR_4_5            0x02
# define SX1276_MC1_CR_4_6            0x04
# define SX1276_MC1_CR_4_7            0x06
# define SX1276_MC1_CR_4_8            0x08
 
# define SX1276_MC1_IMPLICIT_HEADER_MODE_ON    0x01
 
// sx1276 RegModemConfig2
# define SX1276_MC2_RX_PAYLOAD_CRCON        0x04
 
// sx1276 RegModemConfig3
# define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
# define SX1276_MC3_AGCAUTO                 0x04
 
// preamble for lora networks (nibbles swapped)
# define LORA_MAC_PREAMBLE                  0x34
 
# define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
# ifdef LMIC_SX1276
# define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70
# elif LMIC_SX1272
# define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x74
# endif
 
// FRF
# define        REG_FRF_MSB              0x06
# define        REG_FRF_MID              0x07
# define        REG_FRF_LSB              0x08
 
# define        FRF_MSB                  0xD9 // 868.1 Mhz
# define        FRF_MID                  0x06
# define        FRF_LSB                  0x66
 
// ----------------------------------------
// Constants for radio registers
# define OPMODE_LORA            0x80
# define OPMODE_MASK            0x07
# define OPMODE_SLEEP           0x00
# define OPMODE_STANDBY         0x01
# define OPMODE_FSTX            0x02
# define OPMODE_TX              0x03
# define OPMODE_FSRX            0x04
# define OPMODE_RX              0x05
# define OPMODE_RX_SINGLE       0x06
# define OPMODE_CAD             0x07
 
// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
# define IRQ_LORA_RXTOUT_MASK   0x80
# define IRQ_LORA_RXDONE_MASK   0x40
# define IRQ_LORA_CRCERR_MASK   0x20
# define IRQ_LORA_HEADER_MASK   0x10
# define IRQ_LORA_TXDONE_MASK   0x08
# define IRQ_LORA_CDDONE_MASK   0x04
# define IRQ_LORA_FHSSCH_MASK   0x02
# define IRQ_LORA_CDDETD_MASK   0x01
 
// DIO function mappings                D0D1D2D3
# define MAP_DIO0_LORA_RXDONE   0x00  // 00------
# define MAP_DIO0_LORA_TXDONE   0x40  // 01------
# define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
# define MAP_DIO1_LORA_NOP      0x30  // --11----
# define MAP_DIO2_LORA_NOP      0xC0  // ----11--

// #############################################
// #############################################
//
typedef bool boolean;
typedef unsigned char byte;

typedef enum
{
    SF7 = 7,
    SF8,
    SF9,
    SF10,
    SF11,
    SF12
} sf_t;


class sx1276 {
 
    public:

        const int CHANNEL = 0;

        // SX127X - Raspberry connections
        int ssPin = 6;
        int dio0  = 7;
        int RST   = 0;

        // Set spreading factor (SF7 - SF12)
        sf_t sf = SF7;

        // Set center frequency
        uint32_t  freq = 868100000; // in Mhz! (868.1)


        void init_radio();

        void radio_reset();

        uint8_t get_status(void);

        void set_channel(uint32_t freq);

        uint32_t random(void);

        void set_operation_mode(uint8_t mode);

        void set_modem(RadioModems_t modem);

        void set_low_power_mode();

        void set_antenna_switch(uint8_t mode);

        unsigned char read_register(unsigned char addr);

        void write_to_register(unsigned char addr, unsigned char value);


        void sleep( void );


        // ################  old functions  ################ 
        // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv


        void die(const char *s);

        void selectreceiver(void);

        void unselectreceiver();

        byte readReg(byte addr);

        void writeReg(byte addr, byte value);

        void opmode (uint8_t mode);

        void opmodeLora();

        void SetupLoRa();

        boolean receive(char *payload);

        const char* receivepacket();

        void configPower (int8_t pw);

        void writeBuf(byte addr, byte *value, byte len);

        void txlora(byte *frame, byte datalen);

        void isr_handler(void);

        void init_streamer_obj(std::shared_ptr<MQTTDataStreamer> streamer_obj, std::mutex* mut);


    private:
    

        RadioSettings_t settings;


        // Structure containing all user and network specified settings
        // for radio module
        RadioSettings_t _rf_settings;
       

        char message[256];

        bool sx1272 = false;

        byte receivedbytes;

        std::shared_ptr<MQTTDataStreamer> streamer_obj;

        std::mutex *mut;

        /*******************************************************************************
        *
        * Configure these values!
        *
        *******************************************************************************/

        

};


#endif

