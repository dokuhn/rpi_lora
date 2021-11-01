/**
 * @file sx1276.cpp
 * @author Dominik Kuhn (dominik.kuhn90@googlemail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <thread>
#include <cassert>
#include <cmath>

#include <boost/timer.hpp>

extern "C" {
    #include <stdio.h>
    #include <sys/types.h>
    #include <string.h>
    #include <sys/time.h>
    #include <unistd.h>
    #include <stdlib.h>
    #include <stdbool.h>

    #include <wiringPi.h>
    #include <wiringPiSPI.h>
}

#include "sx1276.hpp"

/*!
 * Sync word for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12

/*!
 * Sync word for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x34

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164.0
#define RSSI_OFFSET_HF                              -157.0
#define RF_MID_BAND_THRESH                          525000000


/*!
 * FSK bandwidth definition
 */
typedef struct {
    uint32_t bandwidth;
    uint8_t  register_value;
} fsk_bw_t;

/*!
 * Radio registers definition
 */
typedef struct {
    RadioModems_t   modem;
    uint8_t         addr;
    uint8_t         value;
} radio_registers_t;


#define RADIO_INIT_REGISTERS_VALUE                \
{                                                 \
    { MODEM_FSK , REG_LNA                , 0x23 },\
    { MODEM_FSK , REG_RXCONFIG           , 0x1E },\
    { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
    { MODEM_FSK , REG_AFCFEI             , 0x01 },\
    { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
    { MODEM_FSK , REG_OSC                , 0x07 },\
    { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },\
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
    { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
    { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },\
    { MODEM_FSK , REG_IMAGECAL           , 0x02 },\
    { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
    { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
}

static const fsk_bw_t fsk_bandwidths[] = {
    { 2600, 0x17 },
    { 3100, 0x0F },
    { 3900, 0x07 },
    { 5200, 0x16 },
    { 6300, 0x0E },
    { 7800, 0x06 },
    { 10400, 0x15 },
    { 12500, 0x0D },
    { 15600, 0x05 },
    { 20800, 0x14 },
    { 25000, 0x0C },
    { 31300, 0x04 },
    { 41700, 0x13 },
    { 50000, 0x0B },
    { 62500, 0x03 },
    { 83333, 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid bandwidth
};

/**
 * Radio hardware registers initialization
 */
static const radio_registers_t radio_reg_init[] = RADIO_INIT_REGISTERS_VALUE;


/*****************************************************************************
 * Public APIs                                                               *
 ****************************************************************************/
/**
 * Acquire lock
 */
void sx1276::lock(void)
{
    mutex.lock();
}

/**
 * Release lock
 */
void sx1276::unlock(void)
{
    mutex.unlock();
}




/**
 * Initializes radio module
 */
void sx1276::init_radio()
{

    // Reset the radio transceiver
    radio_reset();

    // Setup radio variant type
    //set_sx1276_variant_type();

    // setup SPI frequency
    // default is 8MHz although, configurable through
    // SPI_FREQUENCY macro
    //setup_spi();

    // Calibrate radio receiver chain
    // rx_chain_calibration();

    // set radio mode to sleep
    set_operation_mode(RF_OPMODE_SLEEP);

    // Setup radio registers to defaults
    // setup_registers();

    // set modem type - defaults to FSK here
    set_modem(MODEM_FSK);

    // set state to be idle
    _rf_settings.State = RF_IDLE;

    // Setup interrupts on DIO pins
    //setup_interrupts();
}

/**
 * Can be used by application/stack or the driver itself
 */
void sx1276::radio_reset()
{
    digitalWrite(RST, LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    digitalWrite(RST, HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/**
 * Returns current status of the radio state machine
 */
uint8_t sx1276::get_status(void)
{
    return _rf_settings.State;
}

/**
 * Sets up carrier frequency
 */
void sx1276::set_channel(uint32_t freq)
{
    _rf_settings.Channel = freq;
    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    write_to_register(REG_FRFMSB, (uint8_t)((frf >> 16) & 0xFF));
    write_to_register(REG_FRFMID, (uint8_t)((frf >> 8) & 0xFF));
    write_to_register(REG_FRFLSB, (uint8_t)(frf & 0xFF));
}

/**
 * Generates 32 bit random number based upon RSSI monitoring
 * Used for various calculation by the stack for example dev nonce
 *
 * When this API is used modem is set in LoRa mode and all interrupts are
 * masked. If the user had been using FSK mode, it should be noted that a
 * change of mode is required again because the registers have changed.
 * In addition to that RX and TX configuration APIs should be called again in
 * order to have correct desires setup.
 */
uint32_t sx1276::random(void)
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    set_modem(MODEM_LORA);

    // Disable LoRa modem interrupts
    write_to_register(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                      RFLR_IRQFLAGS_RXDONE |
                      RFLR_IRQFLAGS_PAYLOADCRCERROR |
                      RFLR_IRQFLAGS_VALIDHEADER |
                      RFLR_IRQFLAGS_TXDONE |
                      RFLR_IRQFLAGS_CADDONE |
                      RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                      RFLR_IRQFLAGS_CADDETECTED);

    // Set radio in continuous reception
    set_operation_mode(RF_OPMODE_RECEIVER);

    for (i = 0; i < 32; i++) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ((uint32_t) read_register(REG_LR_RSSIWIDEBAND) & 0x01) << i;
    }

    sleep();

    return rnd;
}

/**
 * Sets up receiver related configurations
 *
 * Must be called before setting the radio in rx mode
 */
void sx1276::set_rx_config(RadioModems_t modem, uint32_t bandwidth,
                            uint32_t datarate, uint8_t coderate,
                            uint32_t bandwidth_afc,
                            uint16_t preamble_len,
                            uint16_t symb_timeout, bool fix_len,
                            uint8_t payload_len, bool crc_on,
                            bool freq_hop_on, uint8_t hop_period,
                            bool iq_inverted, bool rx_continuous)
{
    set_modem(modem);

    switch (modem) {
        case MODEM_FSK:
            _rf_settings.Fsk.Bandwidth = bandwidth;
            _rf_settings.Fsk.Datarate = datarate;
            _rf_settings.Fsk.BandwidthAfc = bandwidth_afc;
            _rf_settings.Fsk.FixLen = fix_len;
            _rf_settings.Fsk.PayloadLen = payload_len;
            _rf_settings.Fsk.CrcOn = crc_on;
            _rf_settings.Fsk.IqInverted = iq_inverted;
            _rf_settings.Fsk.RxContinuous = rx_continuous;
            _rf_settings.Fsk.PreambleLen = preamble_len;
            _rf_settings.Fsk.RxSingleTimeout = (symb_timeout + 1) / 2; // dividing by 2 as our detector size is 2 symbols (16 bytes)

            datarate = (uint16_t)((float) XTAL_FREQ / (float) datarate);
            write_to_register(REG_BITRATEMSB, (uint8_t)(datarate >> 8));
            write_to_register(REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

            write_to_register(REG_RXBW, get_fsk_bw_reg_val(bandwidth));
            write_to_register(REG_AFCBW, get_fsk_bw_reg_val(bandwidth_afc));

            write_to_register(REG_PREAMBLEMSB, (uint8_t)((preamble_len >> 8) & 0xFF));
            write_to_register(REG_PREAMBLELSB, (uint8_t)(preamble_len & 0xFF));

            if (fix_len == 1) {
                write_to_register(REG_PAYLOADLENGTH, payload_len);
            } else {
                write_to_register(REG_PAYLOADLENGTH, 0xFF); // Set payload length to the maximum
            }

            write_to_register(
                REG_PACKETCONFIG1,
                (read_register(REG_PACKETCONFIG1)
                 & RF_PACKETCONFIG1_CRC_MASK
                 & RF_PACKETCONFIG1_PACKETFORMAT_MASK)
                | ((fix_len == 1) ?
                   RF_PACKETCONFIG1_PACKETFORMAT_FIXED :
                   RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE)
                | (crc_on << 4));

            // TODO why packet mode 2 ?
            write_to_register(REG_PACKETCONFIG2, (read_register(REG_PACKETCONFIG2)
                                                  | RF_PACKETCONFIG2_DATAMODE_PACKET));

            break;

        case MODEM_LORA:

            if (bandwidth > 2) {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while (1)
                    ;
                // TODO Return a proper error from here
            }

            // stupid hack. TODO think something better
            bandwidth += 7;

            _rf_settings.LoRa.Bandwidth = bandwidth;
            _rf_settings.LoRa.Datarate = datarate;
            _rf_settings.LoRa.Coderate = coderate;
            _rf_settings.LoRa.PreambleLen = preamble_len;
            _rf_settings.LoRa.FixLen = fix_len;
            _rf_settings.LoRa.PayloadLen = payload_len;
            _rf_settings.LoRa.CrcOn = crc_on;
            _rf_settings.LoRa.FreqHopOn = freq_hop_on;
            _rf_settings.LoRa.HopPeriod = hop_period;
            _rf_settings.LoRa.IqInverted = iq_inverted;
            _rf_settings.LoRa.RxContinuous = rx_continuous;

            if (datarate > 12) {
                datarate = 12;
            } else if (datarate < 6) {
                datarate = 6;
            }

            if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12)))
                    || ((bandwidth == 8) && (datarate == 12))) {
                _rf_settings.LoRa.LowDatarateOptimize = 0x01;
            } else {
                _rf_settings.LoRa.LowDatarateOptimize = 0x00;
            }

            write_to_register(REG_LR_MODEMCONFIG1, (read_register(REG_LR_MODEMCONFIG1)
                                                    & RFLR_MODEMCONFIG1_BW_MASK
                                                    & RFLR_MODEMCONFIG1_CODINGRATE_MASK
                                                    & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK)
                              | (bandwidth << 4)
                              | (coderate << 1) | fix_len);

            write_to_register(REG_LR_MODEMCONFIG2, (read_register(REG_LR_MODEMCONFIG2)
                                                    & RFLR_MODEMCONFIG2_SF_MASK
                                                    & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK
                                                    & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK)
                              | (datarate << 4)
                              | (crc_on << 2)
                              | ((symb_timeout >> 8)
                                 & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

            write_to_register(REG_LR_MODEMCONFIG3, (read_register(REG_LR_MODEMCONFIG3)
                                                    & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK)
                              | (_rf_settings.LoRa.LowDatarateOptimize << 3));

            write_to_register(REG_LR_SYMBTIMEOUTLSB, (uint8_t)(symb_timeout & 0xFF));

            write_to_register(REG_LR_PREAMBLEMSB, (uint8_t)((preamble_len >> 8) & 0xFF));
            write_to_register(REG_LR_PREAMBLELSB, (uint8_t)(preamble_len & 0xFF));

            write_to_register(REG_LR_LNA, RFLR_LNA_GAIN_G1 | RFLR_LNA_BOOST_HF_ON);
            
            /*
            if (fix_len == 1) {
                write_to_register(REG_LR_PAYLOADLENGTH, payload_len);
            }
            */

            write_to_register(REG_LR_PAYLOADLENGTH, payload_len >> 1);
            write_to_register(REG_LR_PAYLOADMAXLENGTH, payload_len);

            if (_rf_settings.LoRa.FreqHopOn == true) {
                /*
                write_to_register(REG_LR_PLLHOP, (read_register(REG_LR_PLLHOP)
                                                  & RFLR_PLLHOP_FASTHOP_MASK)
                                  | RFLR_PLLHOP_FASTHOP_ON);     
                */            
                write_to_register(REG_LR_HOPPERIOD, _rf_settings.LoRa.HopPeriod);
            }
            /*
            if ((bandwidth == 9) && (_rf_settings.Channel > RF_MID_BAND_THRESH)) {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                write_to_register(REG_LR_TEST36, 0x02);
                write_to_register(REG_LR_TEST3A, 0x64);
            } else if (bandwidth == 9) {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                write_to_register(REG_LR_TEST36, 0x02);
                write_to_register(REG_LR_TEST3A, 0x7F);
            } else {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                write_to_register(REG_LR_TEST36, 0x03);
            }
            

            if (datarate == 6) {
                write_to_register(REG_LR_DETECTOPTIMIZE, (read_register(REG_LR_DETECTOPTIMIZE)
                                                          & RFLR_DETECTIONOPTIMIZE_MASK)
                                  | RFLR_DETECTIONOPTIMIZE_SF6);
                write_to_register(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
            } else {
                write_to_register(REG_LR_DETECTOPTIMIZE, (read_register(REG_LR_DETECTOPTIMIZE)
                                                          & RFLR_DETECTIONOPTIMIZE_MASK)
                                  | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
                write_to_register(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
            }
            */


            break;

        default:
            break;
    }
}

/**
 * Sets up transmitter related configuration
 *
 * Must be called before putting the radio module in Tx mode or trying
 * to send
 */
void sx1276::set_tx_config(RadioModems_t modem, int8_t power,
                                     uint32_t fdev, uint32_t bandwidth,
                                     uint32_t datarate, uint8_t coderate,
                                     uint16_t preamble_len, bool fix_len,
                                     bool crc_on, bool freq_hop_on,
                                     uint8_t hop_period, bool iq_inverted,
                                     uint32_t timeout)
{
    set_modem(modem);
    set_rf_tx_power(power);

    switch (modem) {
        case MODEM_FSK:
            _rf_settings.Fsk.Power = power;
            _rf_settings.Fsk.Fdev = fdev;
            _rf_settings.Fsk.Bandwidth = bandwidth;
            _rf_settings.Fsk.Datarate = datarate;
            _rf_settings.Fsk.PreambleLen = preamble_len;
            _rf_settings.Fsk.FixLen = fix_len;
            _rf_settings.Fsk.CrcOn = crc_on;
            _rf_settings.Fsk.IqInverted = iq_inverted;
            _rf_settings.Fsk.TxTimeout = timeout;

            fdev = (uint16_t)((float) fdev / (float) FREQ_STEP);
            write_to_register(REG_FDEVMSB, (uint8_t)(fdev >> 8));
            write_to_register(REG_FDEVLSB, (uint8_t)(fdev & 0xFF));

            datarate = (uint16_t)((float) XTAL_FREQ / (float) datarate);
            write_to_register(REG_BITRATEMSB, (uint8_t)(datarate >> 8));
            write_to_register(REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

            write_to_register(REG_PREAMBLEMSB, (preamble_len >> 8) & 0x00FF);
            write_to_register(REG_PREAMBLELSB, preamble_len & 0xFF);

            write_to_register(REG_PACKETCONFIG1,
                              (read_register(REG_PACKETCONFIG1) &
                               RF_PACKETCONFIG1_CRC_MASK &
                               RF_PACKETCONFIG1_PACKETFORMAT_MASK)
                              | ((fix_len == 1) ?
                                 RF_PACKETCONFIG1_PACKETFORMAT_FIXED :
                                 RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE)
                              | (crc_on << 4));
            write_to_register(REG_PACKETCONFIG2,
                              (read_register(REG_PACKETCONFIG2)
                               | RF_PACKETCONFIG2_DATAMODE_PACKET));

            break;

        case MODEM_LORA:
            _rf_settings.LoRa.Power = power;
            // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            assert(bandwidth <= 2);
            bandwidth += 7;
            _rf_settings.LoRa.Bandwidth = bandwidth;
            _rf_settings.LoRa.Datarate = datarate;
            _rf_settings.LoRa.Coderate = coderate;
            _rf_settings.LoRa.PreambleLen = preamble_len;
            _rf_settings.LoRa.FixLen = fix_len;
            _rf_settings.LoRa.FreqHopOn = freq_hop_on;
            _rf_settings.LoRa.HopPeriod = hop_period;
            _rf_settings.LoRa.CrcOn = crc_on;
            _rf_settings.LoRa.IqInverted = iq_inverted;
            _rf_settings.LoRa.TxTimeout = timeout;

            if (datarate > 12) {
                datarate = 12;
            } else if (datarate < 6) {
                datarate = 6;
            }
            if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12)))
                    || ((bandwidth == 8) && (datarate == 12))) {
                _rf_settings.LoRa.LowDatarateOptimize = 0x01;
            } else {
                _rf_settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if (_rf_settings.LoRa.FreqHopOn == true) {
                write_to_register(REG_LR_PLLHOP, (read_register(REG_LR_PLLHOP)
                                                  & RFLR_PLLHOP_FASTHOP_MASK)
                                  | RFLR_PLLHOP_FASTHOP_ON);
                write_to_register(REG_LR_HOPPERIOD, _rf_settings.LoRa.HopPeriod);
            }

            write_to_register(REG_LR_MODEMCONFIG1, (read_register(REG_LR_MODEMCONFIG1)
                                                    & RFLR_MODEMCONFIG1_BW_MASK
                                                    & RFLR_MODEMCONFIG1_CODINGRATE_MASK
                                                    & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) | (bandwidth << 4)
                              | (coderate << 1) | fix_len);

            write_to_register(REG_LR_MODEMCONFIG2, (read_register(REG_LR_MODEMCONFIG2)
                                                    & RFLR_MODEMCONFIG2_SF_MASK
                                                    & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK)
                              | (datarate << 4)
                              | (crc_on << 2));

            write_to_register(REG_LR_MODEMCONFIG3, (read_register(REG_LR_MODEMCONFIG3)
                                                    & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK)
                              | (_rf_settings.LoRa.LowDatarateOptimize << 3));

            write_to_register(REG_LR_PREAMBLEMSB, (preamble_len >> 8) & 0x00FF);
            write_to_register(REG_LR_PREAMBLELSB, preamble_len & 0xFF);

            if (datarate == 6) {
                write_to_register(REG_LR_DETECTOPTIMIZE, (read_register(REG_LR_DETECTOPTIMIZE)
                                                          & RFLR_DETECTIONOPTIMIZE_MASK) | RFLR_DETECTIONOPTIMIZE_SF6);
                write_to_register(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
            } else {
                write_to_register(REG_LR_DETECTOPTIMIZE, (read_register(REG_LR_DETECTOPTIMIZE)
                                                          & RFLR_DETECTIONOPTIMIZE_MASK) | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
                write_to_register(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
            }

            break;
    }
}

/**
 * Calculates time on Air i.e., dwell time for a single packet
 *
 * Crucial for the stack in order to calculate dwell time so as to control
 * duty cycling.
 */
uint32_t sx1276::time_on_air(RadioModems_t modem, uint8_t pkt_len)
{
    uint32_t airTime = 0;

    switch (modem) {
        case MODEM_FSK:
            airTime =
                std::rint((8 * (_rf_settings.Fsk.PreambleLen
                            + ((read_register(REG_SYNCCONFIG)
                                & ~RF_SYNCCONFIG_SYNCSIZE_MASK) + 1)
                            + ((_rf_settings.Fsk.FixLen == 0x01) ?
                                0.0f : 1.0f)
                            + (((read_register(REG_PACKETCONFIG1)
                                    & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK)
                                != 0x00) ? 1.0f : 0) + pkt_len
                            + ((_rf_settings.Fsk.CrcOn == 0x01) ?
                                2.0 : 0))
                        / _rf_settings.Fsk.Datarate) * 1000);

            break;
        case MODEM_LORA:
            float bw = 0.0f;
            // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch (_rf_settings.LoRa.Bandwidth) {
                //case 0: // 7.8 kHz
                //    bw = 78e2;
                //    break;
                //case 1: // 10.4 kHz
                //    bw = 104e2;
                //    break;
                //case 2: // 15.6 kHz
                //    bw = 156e2;
                //    break;
                //case 3: // 20.8 kHz
                //    bw = 208e2;
                //    break;
                //case 4: // 31.2 kHz
                //    bw = 312e2;
                //    break;
                //case 5: // 41.4 kHz
                //    bw = 414e2;
                //    break;
                //case 6: // 62.5 kHz
                //    bw = 625e2;
                //    break;
                case 7: // 125 kHz
                    bw = 125000;
                    break;
                case 8: // 250 kHz
                    bw = 250000;
                    break;
                case 9: // 500 kHz
                    bw = 500000;
                    break;
            }

            // Symbol rate : time for one symbol (secs)
            float rs = bw / (1 << _rf_settings.LoRa.Datarate);
            float ts = 1 / rs;
            // time of preamble
            float tPreamble = (_rf_settings.LoRa.PreambleLen + 4.25f) * ts;
            // Symbol length of payload and time
            float tmp = std::ceil((8 * pkt_len - 4 * _rf_settings.LoRa.Datarate + 28
                                + 16 * _rf_settings.LoRa.CrcOn
                                - (_rf_settings.LoRa.FixLen ? 20 : 0))
                                / (float)(4
                                        * (_rf_settings.LoRa.Datarate
                                            - ((_rf_settings.LoRa.LowDatarateOptimize > 0)
                                                ? 2 : 0))))
                            * (_rf_settings.LoRa.Coderate + 4);
            float nPayload = 8 + ((tmp > 0) ? tmp : 0);
            float tPayload = nPayload * ts;
            // Time on air
            float tOnAir = tPreamble + tPayload;
            // return ms secs
            airTime = std::floor(tOnAir * 1000 + 0.999f);

            break;
    }

    return airTime;
}

/**
 * Prepares and sends the radio packet out in the air
 */
void sx1276::send(uint8_t *buffer, uint8_t size)
{
    uint32_t tx_timeout = 0;

    switch (_rf_settings.Modem) {
        case MODEM_FSK:
            _rf_settings.FskPacketHandler.NbBytes = 0;
            _rf_settings.FskPacketHandler.Size = size;

            if (_rf_settings.Fsk.FixLen == false) {
                write_fifo((uint8_t *) &size, 1);
            } else {
                write_to_register(REG_PAYLOADLENGTH, size);
            }

            if ((size > 0) && (size <= 64)) {
                _rf_settings.FskPacketHandler.ChunkSize = size;
            } else {
                std::memcpy(_data_buffer, buffer, size);
                _rf_settings.FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            write_fifo(buffer, _rf_settings.FskPacketHandler.ChunkSize);
            _rf_settings.FskPacketHandler.NbBytes +=
                _rf_settings.FskPacketHandler.ChunkSize;
            tx_timeout = _rf_settings.Fsk.TxTimeout;

            break;

        case MODEM_LORA:
            if (_rf_settings.LoRa.IqInverted == true) {
                write_to_register(REG_LR_INVERTIQ, ((read_register(REG_LR_INVERTIQ)
                                                     & RFLR_INVERTIQ_TX_MASK
                                                     & RFLR_INVERTIQ_RX_MASK)
                                                    | RFLR_INVERTIQ_RX_OFF
                                                    | RFLR_INVERTIQ_TX_ON));
                write_to_register(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
            } else {
                write_to_register(REG_LR_INVERTIQ, ((read_register(REG_LR_INVERTIQ)
                                                     & RFLR_INVERTIQ_TX_MASK
                                                     & RFLR_INVERTIQ_RX_MASK)
                                                    | RFLR_INVERTIQ_RX_OFF
                                                    | RFLR_INVERTIQ_TX_OFF));
                write_to_register(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
            }

            _rf_settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            write_to_register(REG_LR_PAYLOADLENGTH, size);

            // Full buffer used for Tx
            write_to_register(REG_LR_FIFOTXBASEADDR, 0);
            write_to_register(REG_LR_FIFOADDRPTR, 0);

            // FIFO operations can not take place in Sleep mode
            if ((read_register(REG_OPMODE) & ~RF_OPMODE_MASK) == RF_OPMODE_SLEEP) {
                standby();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            // write_to_register payload buffer
            write_fifo(buffer, size);
            tx_timeout = _rf_settings.LoRa.TxTimeout;

            break;
    }

    transmit(tx_timeout);
}

/**
 * sets the radio module to sleep
 */

void sx1276::sleep( void )
{
    // txTimeoutTimer.detach( );
    // rxTimeoutTimer.detach( );

    set_operation_mode( RF_OPMODE_SLEEP );
    this->settings.State = RF_IDLE;
}

/**
 * Put radio in Standby mode
 */
void sx1276::standby(void)
{
    // tx_timeout_timer.detach();

    set_operation_mode(RF_OPMODE_STANDBY);
    _rf_settings.State = RF_IDLE;
}
 
/**
 * Sets the radio module in receive mode
 *
 * A DIO4 interrupt let's the state machine know that a preamble is detected
 * and finally a DIO0 interrupt let's the state machine know that a packet is
 * ready to be read from the FIFO
 */
void sx1276::receive(void)
{
    switch (_rf_settings.Modem) {
        case MODEM_FSK:

            // DIO0=PayloadReady/PacketSent
            // DIO1=FifoLevel
            // DIO2=RxTimeout
            // DIO3=FifoEmpty?
            // DIO4=PreambleDetect
            // DIO5=ModeReady?
            write_to_register(REG_DIOMAPPING1, (read_register(REG_DIOMAPPING1)
                                                & RF_DIOMAPPING1_DIO0_MASK
                                                & RF_DIOMAPPING1_DIO1_MASK
                                                & RF_DIOMAPPING1_DIO2_MASK)
                              | RF_DIOMAPPING1_DIO0_00
                              | RF_DIOMAPPING1_DIO1_00
                              | RF_DIOMAPPING1_DIO2_10);

            write_to_register(REG_DIOMAPPING2, (read_register(REG_DIOMAPPING2)
                                                & RF_DIOMAPPING2_DIO4_MASK
                                                & RF_DIOMAPPING2_MAP_MASK)
                              | RF_DIOMAPPING2_DIO4_11
                              | RF_DIOMAPPING2_MAP_PREAMBLEDETECT);

            _rf_settings.FskPacketHandler.FifoThresh =
                read_register(REG_FIFOTHRESH) & 0x3F;

            write_to_register(REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON
                              | RF_RXCONFIG_AGCAUTO_ON
                              | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);

            if (!_rf_settings.Fsk.RxContinuous) {
                // the value for rx timeout in symbols cannot be more than 255
                // as the preamble length is fixed. We assert here for quick
                // diagnostics
                assert(_rf_settings.Fsk.RxSingleTimeout <= 255);
                write_to_register(REG_RXTIMEOUT2, _rf_settings.Fsk.RxSingleTimeout);
                write_to_register(REG_RXTIMEOUT3, 0x00);
                write_to_register(REG_RXTIMEOUT1, 0x00);
            }

            _rf_settings.FskPacketHandler.PreambleDetected = 0;
            _rf_settings.FskPacketHandler.SyncWordDetected = 0;
            _rf_settings.FskPacketHandler.NbBytes = 0;
            _rf_settings.FskPacketHandler.Size = 0;

            break;

        case MODEM_LORA:

            if (_rf_settings.LoRa.IqInverted == true) {
                write_to_register(REG_LR_INVERTIQ, ((read_register(REG_LR_INVERTIQ)
                                                     & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK)
                                                    | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF));
                write_to_register(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
            } else {
                write_to_register(REG_LR_INVERTIQ, ((read_register(REG_LR_INVERTIQ)
                                                     & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK)
                                                    | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
                write_to_register(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
            }
            /*

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if (_rf_settings.LoRa.Bandwidth < 9) {
                write_to_register(REG_LR_DETECTOPTIMIZE,
                                  read_register(REG_LR_DETECTOPTIMIZE) & 0x7F);
                write_to_register(REG_LR_TEST30, 0x00);
                switch (_rf_settings.LoRa.Bandwidth) {
                    case 0: // 7.8 kHz
                        write_to_register(REG_LR_TEST2F, 0x48);
                        set_channel(_rf_settings.Channel + 7.81e3);
                        break;
                    case 1: // 10.4 kHz
                        write_to_register(REG_LR_TEST2F, 0x44);
                        set_channel(_rf_settings.Channel + 10.42e3);
                        break;
                    case 2: // 15.6 kHz
                        write_to_register(REG_LR_TEST2F, 0x44);
                        set_channel(_rf_settings.Channel + 15.62e3);
                        break;
                    case 3: // 20.8 kHz
                        write_to_register(REG_LR_TEST2F, 0x44);
                        set_channel(_rf_settings.Channel + 20.83e3);
                        break;
                    case 4: // 31.2 kHz
                        write_to_register(REG_LR_TEST2F, 0x44);
                        set_channel(_rf_settings.Channel + 31.25e3);
                        break;
                    case 5: // 41.4 kHz
                        write_to_register(REG_LR_TEST2F, 0x44);
                        set_channel(_rf_settings.Channel + 41.67e3);
                        break;
                    case 6: // 62.5 kHz
                        write_to_register(REG_LR_TEST2F, 0x40);
                        break;
                    case 7: // 125 kHz
                        write_to_register(REG_LR_TEST2F, 0x40);
                        break;
                    case 8: // 250 kHz
                        write_to_register(REG_LR_TEST2F, 0x40);
                        break;
                }
            } else {
                write_to_register(REG_LR_DETECTOPTIMIZE,
                                  read_register(REG_LR_DETECTOPTIMIZE) | 0x80);
            }

            */

            /*
            if (_rf_settings.LoRa.FreqHopOn == true) {
                write_to_register(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_VALIDHEADER
                                  | RFLR_IRQFLAGS_TXDONE
                                  | RFLR_IRQFLAGS_CADDONE
                                  | RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=RxDone, DIO2=FhssChangeChannel
                write_to_register(REG_DIOMAPPING1, (read_register(REG_DIOMAPPING1)
                                                    & RFLR_DIOMAPPING1_DIO0_MASK
                                                    & RFLR_DIOMAPPING1_DIO2_MASK)
                                  | RFLR_DIOMAPPING1_DIO0_00
                                  | RFLR_DIOMAPPING1_DIO2_00);
            } else {
            */
                write_to_register(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_VALIDHEADER
                                  | RFLR_IRQFLAGS_TXDONE
                                  | RFLR_IRQFLAGS_CADDONE
                                  | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL
                                  | RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=RxDone
                write_to_register(REG_DIOMAPPING1, (read_register(REG_DIOMAPPING1)
                                                    & RFLR_DIOMAPPING1_DIO0_MASK)
                                  | RFLR_DIOMAPPING1_DIO0_00);
            /*
            }
            */
            write_to_register(REG_LR_FIFORXBASEADDR, 0);
            write_to_register(REG_LR_FIFOADDRPTR, 0);

            break;
    }

    memset(_data_buffer, 0, (size_t) MAX_DATA_BUFFER_SIZE_SX1276);

    _rf_settings.State = RF_RX_RUNNING;

    if (_rf_settings.Modem == MODEM_FSK) {
        set_operation_mode(RF_OPMODE_RECEIVER);
        return;
    }

    // If mode is LoRa set mode
    if (_rf_settings.LoRa.RxContinuous == true) {
        set_operation_mode(RFLR_OPMODE_RECEIVER);
    } else {
        set_operation_mode(RFLR_OPMODE_RECEIVER_SINGLE);
    }
}



/**
 * Perform carrier sensing
 *
 * Checks for a certain time if the RSSI is above a given threshold.
 * This threshold determines if there is already a transmission going on
 * in the channel or not.
 *
 */
bool sx1276::perform_carrier_sense(RadioModems_t modem,
                                    uint32_t freq,
                                    int16_t rssi_threshold,
                                    uint32_t max_carrier_sense_time)
{
    bool status = true;
    int16_t rssi = 0;

    set_modem(modem);
    set_channel(freq);
    set_operation_mode(RF_OPMODE_RECEIVER);

    // hold on a bit, radio turn-around time
    std::this_thread::sleep_for(std::chrono::seconds(1));

    boost::timer timer;

    // Perform carrier sense for maxCarrierSenseTime
    while ((timer.elapsed()/1000) < (int)max_carrier_sense_time) {
        rssi = get_rssi(modem);

        if (rssi > rssi_threshold) {
            status = false;
            break;
        }
    }

    sleep();
    return status;
}

/**
 * TODO: Making sure if this API is valid only for LoRa modulation ?
 *
 * Indicates if the node is part of a private or public network
 */
void sx1276::set_public_network(bool enable)
{
    set_modem(MODEM_LORA);

    _rf_settings.LoRa.PublicNetwork = enable;
    if (enable == true) {
        // Change lora modem SyncWord
        write_to_register(REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD);
    } else {
        // Change lora modem SyncWord
        write_to_register(REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD);
    }
}

/**
 * Puts a limit on the size of payload the module can handle
 * By default it is MAX, i.e., 256 bytes
 */
void sx1276::set_max_payload_length(RadioModems_t modem, uint8_t max)
{
    set_modem(modem);

    switch (modem) {
        case MODEM_FSK:
            if (_rf_settings.Fsk.FixLen == false) {
                write_to_register(REG_PAYLOADLENGTH, max);
            }
            break;
        case MODEM_LORA:
            write_to_register(REG_LR_PAYLOADMAXLENGTH, max);
            break;
    }
}


/**
 * Channel Activity detection (can be done only in LoRa mode)
 *
 * If any activity on the channel is detected, an interrupt is asserted on
 * DIO3. A callback will be generated to the stack/application upon the
 * assertion of DIO3.
 */
void sx1276::start_cad()
{
    uint8_t reg_val;

    switch (_rf_settings.Modem) {
        case MODEM_FSK:
            break;
        case MODEM_LORA:
            write_to_register(REG_LR_IRQFLAGSMASK,
                              RFLR_IRQFLAGS_RXTIMEOUT |
                              RFLR_IRQFLAGS_RXDONE |
                              RFLR_IRQFLAGS_PAYLOADCRCERROR |
                              RFLR_IRQFLAGS_VALIDHEADER |
                              RFLR_IRQFLAGS_TXDONE |
                              RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

            // DIO3=CADDone
            reg_val = read_register(REG_DIOMAPPING1);
            write_to_register(REG_DIOMAPPING1, (reg_val &
                                                RFLR_DIOMAPPING1_DIO3_MASK) |
                              RFLR_DIOMAPPING1_DIO3_00);

            set_operation_mode(RFLR_OPMODE_CAD);

            _rf_settings.State = RF_CAD;

            break;
        default:
            break;
    }
}

/**
 * Set transmission in continuous wave mode
 */
void sx1276::set_tx_continuous_wave(uint32_t freq, int8_t power,
                                    uint16_t time)
{
    uint8_t reg_val;

    set_channel(freq);
    set_tx_config(MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, time * 1000);
    reg_val = read_register(REG_PACKETCONFIG2);

    write_to_register(REG_PACKETCONFIG2, (reg_val & RF_PACKETCONFIG2_DATAMODE_MASK));
    // Disable radio interrupts
    write_to_register(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11);
    write_to_register(REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10);

    _rf_settings.State = RF_TX_RUNNING;
    // tx_timeout_timer.attach_us(callback(this, &SX1276_LoRaRadio::timeout_irq_isr), time * 1000000);
    set_operation_mode(RF_OPMODE_TRANSMITTER);
}

/*****************************************************************************
 * Private APIs                                                              *
 ****************************************************************************/


void sx1276::selectreceiver()
{
    digitalWrite(ssPin, LOW);
}

void sx1276::unselectreceiver()
{
    digitalWrite(ssPin, HIGH);
}


/**
 * Writes a single byte to a given register
 */
void sx1276::write_to_register(unsigned char addr, unsigned char value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    unselectreceiver();
}

/**
 * Writes multiple bytes to a given register
 */
void sx1276::write_to_register(uint8_t addr, uint8_t *data, uint8_t size)
{
    unsigned char spibuf[256];

    // set write command
    spibuf[0] = addr | 0x80;

    // copies write data to internal data structure
    for (uint8_t i = 0; i < size; i++) {
        spibuf[i+1] = data[i];
    }

    // set chip-select low
    // digitalWrite(ssPin, LOW);
    selectreceiver();

    // write data
    wiringPiSPIDataRW(CHANNEL, spibuf, size+1);  

    // set chip-select high
    // digitalWrite(ssPin, HIGH);
    unselectreceiver();
}

/**
 * Reads the value of a single register
 */
unsigned char sx1276::read_register(unsigned char addr)
{
    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];

}

/**
 * Reads multiple values from a given register
 */
void sx1276::read_register(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    unsigned char spibuf[size+1];

    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;

    // copies internal buffer variable to return value
    for (uint8_t i = 0; i < (size - 1); i++) {
        buffer[i] = spibuf[i+1];
    }

    // set chip-select low
    digitalWrite(ssPin, LOW);

    // set read command and read buffers to internal variable
    wiringPiSPIDataRW(CHANNEL, spibuf, size+1);

    // set chip-select high
    digitalWrite(ssPin, HIGH);
}


/**
 * Writes to FIIO provided by the chip
 */
void sx1276::write_fifo(uint8_t *buffer, uint8_t size)
{
    write_to_register(0, buffer, size);
}

/**
 * Reads from the FIFO provided by the chip
 */
void sx1276::read_fifo(uint8_t *buffer, uint8_t size)
{
    read_register(0, buffer, size);
}


/**
 * Sets up operation mode
 */
void sx1276::set_operation_mode(uint8_t mode)
{
    if (mode == RF_OPMODE_SLEEP) {
        set_low_power_mode();
    } else {
        set_low_power_mode();
        set_antenna_switch(mode);
    }

    write_to_register(REG_OPMODE, (read_register(REG_OPMODE) & RF_OPMODE_MASK) | mode);

}

/**
 * Sets the modem type to use
 *
 * At initialization FSK is chosen. Later stack or application
 * can choose to change.
 */
void sx1276::set_modem(RadioModems_t modem)
{


    if ((read_register(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_ON) != 0) {
        _rf_settings.Modem = MODEM_LORA;
    } else {
        _rf_settings.Modem = MODEM_FSK;
    }

    if (_rf_settings.Modem == modem) {
        // if the modem is already set
        return;
    }    

    _rf_settings.Modem = modem;



    switch (_rf_settings.Modem) {
        default:
        case MODEM_FSK:
            // before changing modem mode, put the module to sleep
            sleep();
            write_to_register(REG_OPMODE, (read_register(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK)
                              | RFLR_OPMODE_LONGRANGEMODE_OFF);

            // Datasheet Tables 28, 29 DIO mapping
            write_to_register(REG_DIOMAPPING1, 0x00); // sets DIO0-DI03 in default mode
            write_to_register(REG_DIOMAPPING2, 0x30); // bits 4-5 are turned on i.e.,
            //  DIO5 and DIO4=ModeReady
            break;
        case MODEM_LORA:
            sleep();
            write_to_register(REG_OPMODE, (read_register(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK)
                              | RFLR_OPMODE_LONGRANGEMODE_ON);

            // Datasheet Tables 17 DIO mapping for LoRa
            // set to defaults
            write_to_register(REG_DIOMAPPING1, 0x00); // DIO0 - DIO3 defaults
            write_to_register(REG_DIOMAPPING2, 0x00); // DIO4 - DIO5 defaults

            break;
    }

}

/**
 * Set the radio module variant
 */
 /*
void sx1276::set_sx1276_variant_type()
{
    if (_rf_ctrls.ant_switch != NC) {
        _ant_switch.input();
        ThisThread::sleep_for(1);
        if (_ant_switch == 1) {
            radio_variant = SX1276MB1LAS;
        } else {
            radio_variant = SX1276MB1MAS;
        }
        _ant_switch.output();
        ThisThread::sleep_for(1);
    } else {
        radio_variant = MBED_CONF_SX1276_LORA_DRIVER_RADIO_VARIANT;
    }
}
*/

/**
 * Sets the radio registers to defaults
 */
void sx1276::setup_registers()
{
    for (unsigned int i = 0; i < sizeof(radio_reg_init) / sizeof(radio_registers_t); i++) {
        set_modem(radio_reg_init[i].modem);
        write_to_register(radio_reg_init[i].addr, radio_reg_init[i].value);
    }
}

/**
 * Performs the Rx chain calibration for LF and HF bands
 *
 * Must be called just after the reset so all registers are at their
 * default values.
 */
void sx1276::rx_chain_calibration(void)
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = read_register(REG_PACONFIG);
    initialFreq = (float)(((uint32_t) this->read_register(REG_FRFMSB) << 16) |
                          ((uint32_t) this->read_register(REG_FRFMID) << 8) |
                          ((uint32_t)this->read_register(REG_FRFLSB))) * (float) FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    write_to_register(REG_PACONFIG, 0x00);

    // Launch Rx chain calibration for LF band
    write_to_register(REG_IMAGECAL, (read_register(REG_IMAGECAL)
                                     & RF_IMAGECAL_IMAGECAL_MASK)
                      | RF_IMAGECAL_IMAGECAL_START);
    while ((read_register(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING)
            == RF_IMAGECAL_IMAGECAL_RUNNING) {
    }

    // Sets a Frequency in HF band
    set_channel(868000000);

    // Launch Rx chain calibration for HF band
    write_to_register(REG_IMAGECAL, (read_register(REG_IMAGECAL)
                                     & RF_IMAGECAL_IMAGECAL_MASK)
                      | RF_IMAGECAL_IMAGECAL_START);
    while ((read_register(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING)
            == RF_IMAGECAL_IMAGECAL_RUNNING) {
        // do nothing, just wait while rf image frequency calibration is done
    }

    // Restore context
    write_to_register(REG_PACONFIG, regPaConfigInitVal);
    set_channel(initialFreq);
}


/**
 * Gets FSK bandwidth values
 *
 * Gives either normal bandwidths or bandwidths for
 * AFC (auto frequency correction)
 */
uint8_t sx1276::get_fsk_bw_reg_val(uint32_t bandwidth)
{
    uint8_t i;

    for (i = 0; i < (sizeof(fsk_bandwidths) / sizeof(fsk_bw_t)) - 1; i++) {
        if ((bandwidth >= fsk_bandwidths[i].bandwidth)
                && (bandwidth < fsk_bandwidths[i + 1].bandwidth)) {
            return fsk_bandwidths[i].register_value;
        }
    }
    // ERROR: Value not found
    // This should never happen
    while (1);
}


uint8_t sx1276::get_pa_conf_reg(uint32_t channel)
{
    if (radio_variant == UNKNOWN) {
        return RF_PACONFIG_PASELECT_PABOOST;
    } else if (channel > RF_MID_BAND_THRESH) {
        if (radio_variant == SX1276MB1LAS) {
            return RF_PACONFIG_PASELECT_PABOOST;
        } else {
            return RF_PACONFIG_PASELECT_RFO;
        }
    } else {
        return RF_PACONFIG_PASELECT_RFO;
    }
}


/**
 * Sets the transmit power for the module
 */
void sx1276::set_rf_tx_power(int8_t power)
{

    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = read_register(REG_PACONFIG);
    paDac = read_register(REG_PADAC);

    paConfig = (paConfig & RF_PACONFIG_PASELECT_MASK) | get_pa_conf_reg(_rf_settings.Channel);
    paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK) | 0x70;

    if ((paConfig & RF_PACONFIG_PASELECT_PABOOST) == RF_PACONFIG_PASELECT_PABOOST) {
        if (power > 17) {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
        } else {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
        }
        if ((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON) {
            if (power < 5) {
                power = 5;
            }
            if (power > 20) {
                power = 20;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK)
                       | (uint8_t)((uint16_t)(power - 5) & 0x0F);
        } else {
            if (power < 2) {
                power = 2;
            }
            if (power > 17) {
                power = 17;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK)
                       | (uint8_t)((uint16_t)(power - 2) & 0x0F);
        }
    } else {
        if (power < -1) {
            power = -1;
        }
        if (power > 14) {
            power = 14;
        }
        paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK)
                   | (uint8_t)((uint16_t)(power + 1) & 0x0F);
    }
    write_to_register(REG_PACONFIG, paConfig);
    write_to_register(REG_PADAC, paDac);
}

/**
 * Actual TX - Transmit routine
 *
 * A DIO0 interrupt let the state machine know that a a packet is
 * successfully sent, otherwise a TxTimeout is invoked.
 * TxTimeout should never happen in normal circumstances as the radio should
 * be able to send a packet out in the air no matter what.
 */
void sx1276::transmit(uint32_t timeout)
{
    switch (_rf_settings.Modem) {

        case MODEM_FSK:
            // DIO0=PacketSent
            // DIO1=FifoEmpty
            // DIO2=FifoFull
            // DIO3=FifoEmpty
            // DIO4=LowBat
            // DIO5=ModeReady
            write_to_register(REG_DIOMAPPING1, (read_register(REG_DIOMAPPING1) &
                                                RF_DIOMAPPING1_DIO0_MASK &
                                                RF_DIOMAPPING1_DIO1_MASK &
                                                RF_DIOMAPPING1_DIO2_MASK) |
                              RF_DIOMAPPING1_DIO1_01);

            write_to_register(REG_DIOMAPPING2, (read_register(REG_DIOMAPPING2) &
                                                RF_DIOMAPPING2_DIO4_MASK &
                                                RF_DIOMAPPING2_MAP_MASK));
            _rf_settings.FskPacketHandler.FifoThresh =
                read_register(REG_FIFOTHRESH) & 0x3F;

            break;

        case MODEM_LORA:

            if (_rf_settings.LoRa.FreqHopOn == true) {
                write_to_register(REG_LR_IRQFLAGSMASK,
                                  RFLR_IRQFLAGS_RXTIMEOUT |
                                  RFLR_IRQFLAGS_RXDONE |
                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                  RFLR_IRQFLAGS_VALIDHEADER |
                                  RFLR_IRQFLAGS_CADDONE |
                                  RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=tx_done, DIO2=fhss_change_channel

                write_to_register(REG_DIOMAPPING1, (read_register(REG_DIOMAPPING1) &
                                                    RFLR_DIOMAPPING1_DIO0_MASK &
                                                    RFLR_DIOMAPPING1_DIO2_MASK) |
                                  RFLR_DIOMAPPING1_DIO0_01 |
                                  RFLR_DIOMAPPING1_DIO2_01);
            } else {
                write_to_register(REG_LR_IRQFLAGSMASK,
                                  RFLR_IRQFLAGS_RXTIMEOUT |
                                  RFLR_IRQFLAGS_RXDONE |
                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                  RFLR_IRQFLAGS_VALIDHEADER |
                                  RFLR_IRQFLAGS_CADDONE |
                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                  RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=tx_done
                write_to_register(REG_DIOMAPPING1, (read_register(REG_DIOMAPPING1) &
                                                    RFLR_DIOMAPPING1_DIO0_MASK) |
                                  RFLR_DIOMAPPING1_DIO0_01);
            }

            break;
    }

    _rf_settings.State = RF_TX_RUNNING;

    // tx_timeout_timer.attach_us(callback(this,
    //                                    &SX1276_LoRaRadio::timeout_irq_isr), timeout * 1000);

    set_operation_mode(RF_OPMODE_TRANSMITTER);
}

/**
 * Get RSSI from the module
 */
int16_t sx1276::get_rssi(RadioModems_t modem)
{
    int16_t rssi = 0;

    switch (modem) {
        case MODEM_FSK:
            rssi = -(read_register(REG_RSSIVALUE) >> 1);
            break;
        case MODEM_LORA:
            if (_rf_settings.Channel > RF_MID_BAND_THRESH) {
                rssi = RSSI_OFFSET_HF + read_register(REG_LR_RSSIVALUE);
            } else {
                rssi = RSSI_OFFSET_LF + read_register(REG_LR_RSSIVALUE);
            }
            break;
        default:
            rssi = -1;
            break;
    }
    return rssi;
}




/**
 * Sets the module in low power mode by disconnecting
 * TX and RX submodules, turning off power amplifier etc.
 */
void sx1276::set_low_power_mode()
{
    // does nothing at the moment, functionality will be implemented later
    for(int i = 0; i < 10; ++i)
	;
}


/**
 * Sets up radio latch position according to the
 * radio mode
 */
void sx1276::set_antenna_switch(uint8_t mode)
{
    // does nothing at the moment, functionality will be implemented later
    for(int i = 0; i < 10; ++i)
	;
}





