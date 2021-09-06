/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2014 Semtech
 
Description: SX1276 LoRa modem registers and bits definitions
 
License: Revised BSD License, see LICENSE.TXT file include in the project
 
Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __SX1276_REGS_LORA_HPP__
#define __SX1276_REGS_LORA_HPP__

/*!
 * ============================================================================
 * SX1276 Internal registers Address
 * ============================================================================
 */


#define REG_LR_IRQFLAGSMASK                         0x11
#define REG_LR_IRQFLAGS                             0x12

#define REG_LR_RSSIWIDEBAND                         0x2C


/*!
 * ============================================================================
 * SX1276 LoRa bits control definition
 * ============================================================================
 */
 
/*!
 * RegFifo
 */
 
/*!
 * RegOpMode
 */
#define RFLR_OPMODE_LONGRANGEMODE_MASK              0x7F 
#define RFLR_OPMODE_LONGRANGEMODE_OFF               0x00 // Default
#define RFLR_OPMODE_LONGRANGEMODE_ON                0x80 
 
#define RFLR_OPMODE_ACCESSSHAREDREG_MASK            0xBF 
#define RFLR_OPMODE_ACCESSSHAREDREG_ENABLE          0x40 
#define RFLR_OPMODE_ACCESSSHAREDREG_DISABLE         0x00 // Default
 
#define RFLR_OPMODE_FREQMODE_ACCESS_MASK            0xF7
#define RFLR_OPMODE_FREQMODE_ACCESS_LF              0x08 // Default
#define RFLR_OPMODE_FREQMODE_ACCESS_HF              0x00 
 
#define RFLR_OPMODE_MASK                            0xF8 
#define RFLR_OPMODE_SLEEP                           0x00 
#define RFLR_OPMODE_STANDBY                         0x01 // Default
#define RFLR_OPMODE_SYNTHESIZER_TX                  0x02 
#define RFLR_OPMODE_TRANSMITTER                     0x03 
#define RFLR_OPMODE_SYNTHESIZER_RX                  0x04 
#define RFLR_OPMODE_RECEIVER                        0x05 
// LoRa specific modes
#define RFLR_OPMODE_RECEIVER_SINGLE                 0x06 
#define RFLR_OPMODE_CAD                             0x07 


/*!
 * RegIrqFlagsMask
 */
#define RFLR_IRQFLAGS_RXTIMEOUT_MASK                0x80
#define RFLR_IRQFLAGS_RXDONE_MASK                   0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK          0x20
#define RFLR_IRQFLAGS_VALIDHEADER_MASK              0x10
#define RFLR_IRQFLAGS_TXDONE_MASK                   0x08
#define RFLR_IRQFLAGS_CADDONE_MASK                  0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK       0x02
#define RFLR_IRQFLAGS_CADDETECTED_MASK              0x01

/*!
 * RegIrqFlags
 */
#define RFLR_IRQFLAGS_RXTIMEOUT                     0x80
#define RFLR_IRQFLAGS_RXDONE                        0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR               0x20
#define RFLR_IRQFLAGS_VALIDHEADER                   0x10
#define RFLR_IRQFLAGS_TXDONE                        0x08
#define RFLR_IRQFLAGS_CADDONE                       0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL            0x02
#define RFLR_IRQFLAGS_CADDETECTED                   0x01



#endif // __SX1276_REGS_LORA_HPP__