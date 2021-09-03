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
    //rx_chain_calibration();

    // set radio mode to sleep
    set_operation_mode(OPMODE_SLEEP);

    // Setup radio registers to defaults
    //setup_registers();

    // set modem type - defaults to FSK here
    set_modem(MODEM_FSK);

    // set state to be idle
    //_rf_settings.state = RF_IDLE;

    // Setup interrupts on DIO pins
    //setup_interrupts();
}

/**
 * Can be used by application/stack or the driver itself
 */
void sx1276::radio_reset()
{
    digitalWrite(RST, HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    digitalWrite(RST, LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}


/**
 * Sets up operation mode
 */
void sx1276::set_operation_mode(uint8_t mode)
{
    if (mode == OPMODE_SLEEP) {
        set_low_power_mode();
    } else {
        set_low_power_mode();
        set_antenna_switch(mode);
    }

    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & ~OPMODE_MASK) | mode);

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

unsigned char sx1276::read_register(unsigned char addr)
{
    unsigned char spibuf[2];

    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    digitalWrite(ssPin, LOW);
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    digitalWrite(ssPin, HIGH);

    return spibuf[1];

}

void sx1276::write_to_register(unsigned char addr, unsigned char value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    digitalWrite(ssPin, LOW);
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    digitalWrite(ssPin, HIGH);
}


void sx1276::sleep( void )
{
    // txTimeoutTimer.detach( );
    // rxTimeoutTimer.detach( );
 
    opmode( RF_OPMODE_SLEEP );
    this->settings.State = RF_IDLE;
}



// #########################################################################################################################
// #########################################################################################################################


void sx1276::die(const char *s)
{
    perror(s);
    exit(1);
}


void sx1276::selectreceiver()
{
    digitalWrite(ssPin, LOW);
}

void sx1276::unselectreceiver()
{
    digitalWrite(ssPin, HIGH);
}

byte sx1276::readReg(byte addr)
{
    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}

void sx1276::writeReg(byte addr, byte value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    unselectreceiver();
}

void sx1276::opmode (uint8_t mode) {
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & ~OPMODE_MASK) | mode);
}

void sx1276::opmodeLora() {
    uint8_t u = OPMODE_LORA;
    if (sx1272 == false)
        u |= 0x8;   // TBD: sx1276 high freq
    writeReg(REG_OPMODE, u);
}


void sx1276::SetupLoRa()
{
    
    digitalWrite(RST, HIGH);
    delay(100);
    digitalWrite(RST, LOW);
    delay(100);

    byte version = readReg(REG_VERSION);

    if (version == 0x22) {
        // sx1272
        printf("SX1272 detected, starting.\n");
	    fflush(stdout);
        sx1272 = true;
    } else {
        // sx1276?
        digitalWrite(RST, LOW);
        delay(100);
        digitalWrite(RST, HIGH);
        delay(100);
        version = readReg(REG_VERSION);
        if (version == 0x12) {
            // sx1276
            printf("sx1276 detected, starting.\n");
            fflush(stdout);
            sx1272 = false;
        } else {
            printf("Unrecognized transceiver.\n");
            fflush(stdout);
            //printf("Version: 0x%x\n",version);
            exit(1);
        }
    }

    opmode(OPMODE_SLEEP);

    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeReg(REG_FRF_MSB, (uint8_t)(frf>>16) );
    writeReg(REG_FRF_MID, (uint8_t)(frf>> 8) );
    writeReg(REG_FRF_LSB, (uint8_t)(frf>> 0) );

    writeReg(REG_SYNC_WORD, 0x34); // LoRaWAN public sync word

    if (sx1272) {
        if (sf == SF11 || sf == SF12) {
            writeReg(REG_MODEM_CONFIG,0x0B);
        } else {
            writeReg(REG_MODEM_CONFIG,0x0A);
        }
        writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    } else {
        if (sf == SF11 || sf == SF12) {
            writeReg(REG_MODEM_CONFIG3,0x0C);
        } else {
            writeReg(REG_MODEM_CONFIG3,0x04);
        }
        writeReg(REG_MODEM_CONFIG,0x72);
        writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    }

    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeReg(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeReg(REG_SYMB_TIMEOUT_LSB,0x08);
    } 
    writeReg(REG_MAX_PAYLOAD_LENGTH,0x80);
    writeReg(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
    writeReg(REG_HOP_PERIOD,0xFF);
    writeReg(REG_FIFO_ADDR_PTR, readReg(REG_FIFO_RX_BASE_AD));

    writeReg(REG_LNA, LNA_MAX_GAIN);

}

boolean sx1276::receive(char *payload) {
    // clear rxDone
    writeReg(REG_IRQ_FLAGS, IRQ_LORA_RXDONE_MASK);

    int irqflags = readReg(REG_IRQ_FLAGS);

    //  payload crc: 0x20
    if((irqflags & IRQ_LORA_CRCERR_MASK) != 0u)
    {
        printf("CRC error\n");
        fflush(stdout);
        writeReg(REG_IRQ_FLAGS, IRQ_LORA_CRCERR_MASK);
        return false;
    } else {

        byte currentAddr = readReg(REG_FIFO_RX_CURRENT_ADDR);
        byte receivedCount = readReg(REG_RX_NB_BYTES);
        receivedbytes = receivedCount;

        writeReg(REG_FIFO_ADDR_PTR, currentAddr);

        for(int i = 0; i < receivedCount; i++)
        {
            payload[i] = (char)readReg(REG_FIFO);
        }

	payload[receivedCount+1] = '\0';

    }
    return true;
}

const char* sx1276::receivepacket() {

    long int SNR;
    int rssicorr;

    if(digitalRead(dio0) == 1)
    {
        if(receive(message)) {
            byte value = readReg(REG_PKT_SNR_VALUE);
            if( value & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                SNR = -value;
            }
            else
            {
                // Divide by 4
                SNR = ( value & 0xFF ) >> 2;
            }
            
            if (sx1272) {
                rssicorr = 139;
            } else {
                rssicorr = 157;
            }

            printf("Packet RSSI: %d, ", readReg(0x1A)-rssicorr);
            printf("RSSI: %d, ", readReg(0x1B)-rssicorr);
            printf("SNR: %li, ", SNR);
            printf("Length: %i", (int)receivedbytes);
            printf("\n");
            printf("Payload: %s\n", message);
	        fflush(stdout);

            if( streamer_obj.get() != nullptr){
                std::string debug_info = "Sending messages ...";
                std::size_t payload_len = strlen(message);
                streamer_obj->publishMessage( message, payload_len, 
                            "/LoRA_Test/receivedPacket/", 1,  mut, debug_info);
            }

	    return (const char*)message;

        } // received a message

    } // dio0=1
}

void sx1276::configPower (int8_t pw) {
    if (sx1272 == false) {
        // no boost used for now
        if(pw >= 17) {
            pw = 15;
        } else if(pw < 2) {
            pw = 2;
        }
        // check board type for BOOST pin
        writeReg(RegPaConfig, (uint8_t)(0x80|(pw&0xf)));
        writeReg(RegPaDac, readReg(RegPaDac)|0x4);

    } else {
        // set PA config (2-17 dBm using PA_BOOST)
        if(pw > 17) {
            pw = 17;
        } else if(pw < 2) {
            pw = 2;
        }
        writeReg(RegPaConfig, (uint8_t)(0x80|(pw-2)));
    }
}

void sx1276::writeBuf(byte addr, byte *value, byte len) {                                                       
    unsigned char spibuf[256];                                                                          
    spibuf[0] = addr | 0x80;                                                                            
    for (int i = 0; i < len; i++) {                                                                         
        spibuf[i + 1] = value[i];                                                                       
    }                                                                                                   
    selectreceiver();                                                                                   
    wiringPiSPIDataRW(CHANNEL, spibuf, len + 1);                                                        
    unselectreceiver();                                                                                 
}

void sx1276::txlora(byte *frame, byte datalen) {

    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeReg(REG_IRQ_FLAGS, 0xFF);
    // mask all IRQs but TxDone
    writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_TXDONE_MASK);
 
    // initialize the payload size and address pointers
    writeReg(REG_FIFO_TX_BASE_AD, 0x00);
    writeReg(REG_FIFO_ADDR_PTR, 0x00);
    writeReg(REG_PAYLOAD_LENGTH, datalen);

    // download buffer to the radio FIFO
    writeBuf(REG_FIFO, frame, datalen);
    // now we actually start the transmission
    opmode(OPMODE_TX);

    printf("send: %s\n", frame);
    fflush(stdout);
}

void sx1276::isr_handler(void){


   const int irqflags = readReg(REG_IRQ_FLAGS);

   if( (irqflags &  IRQ_LORA_RXDONE_MASK)  != 0u ){

        receivepacket();
   }

   if( (irqflags & IRQ_LORA_TXDONE_MASK) != 0u ){
        
        // clear Irq
        writeReg(REG_IRQ_FLAGS, IRQ_LORA_TXDONE_MASK);

   }

}

void sx1276::init_streamer_obj(std::shared_ptr<MQTTDataStreamer> streamer_obj_, std::mutex* mut_){

    streamer_obj = streamer_obj_;

    mut = mut_;


}
