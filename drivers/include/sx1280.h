/*
   ______                              _
   / _____)             _              | |
   ( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
   _____) ) ____| | | || |_| ____( (___| | | |
   (______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

   Description: Driver for SX1280 devices

   License: Revised BSD License, see LICENSE.TXT file include in the project

   Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
 */

/*
 * Copyright (C) 2020 Université Grenoble Alpes
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup    drivers_SX1280
 *
 * @file			sx1280.h
 * @brief      Driver for SX1280 devices
 *
 * @author     Julia CORREA REIS and Pierre Millot
 */

#ifndef SX1280_H
#define SX1280_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "net/netdev.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "xtimer.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Enables/disables driver debug features */
#define SX1280_DEBUG 0

/**Provides the frequency of the chip running on the radio and the frequency step
 *\remark These defines are used for computing the frequency divider to set the RF frequency
 *@{
 */
#define XTAL_FREQ 52000000
#define FREQ_STEP ((double)(XTAL_FREQ / pow(2.0, 18.0)))
/**@}*/

/** Compensation delay for SetAutoTx/Rx functions in microseconds */
#define AUTO_RX_TX_OFFSET 33

#define SX1280_TX_POWER (14U)       /**< Radio power in dBm */

#define SX1280_IRQ_DIO0 (1 << 0)    /**< DIO0 IRQ */
#define SX1280_IRQ_DIO1 (1 << 1)    /**< DIO1 IRQ */
#define SX1280_IRQ_DIO2 (1 << 2)    /**< DIO2 IRQ */
#define SX1280_IRQ_DIO3 (1 << 3)    /**< DIO3 IRQ */

#define SX1280_WAKEUP_TIME (1000U)  /**< In microseconds [us] */

#define SX1280_RADIO_TX_POWER (14U) /**< Radio power in dBm */

/**
 * @brief   GPIO mode of DIOx Pins.
 */
#ifndef SX1280_DIO_PULL_MODE
#define SX1280_DIO_PULL_MODE (GPIO_IN_PD)
#endif

/**Error and status return
 *@{
 */
enum {
    SX1280_INIT_OK = 0, /* all went as expected */
    SX1280_ERR_SPI,     /* error using SPI */
    SX1280_ERR_GPIOS,   /* error using the GPIO */
    SX1280_ERR_NODEV    /* no device with the configured address found on the bus */
};
/**@}*/

enum { NETOPT_STATE_STANDBY_RC = 7, NETOPT_STATE_STANDBY_XOSC, NETOPT_STATE_FS,
       NETOPT_STATE_CAD };

/** Represents the states of the radio
 *@{
 */
typedef enum {
    SX1280_RF_IDLE,         /* !< The radio is idle */
    SX1280_RF_RX_RUNNING,   /* !< The radio is in reception state */
    SX1280_RF_TX_RUNNING,   /* !< The radio is in transmission state */
    SX1280_RF_CAD           /* !< The radio is doing channel activity detection */
} SX1280_States_t;

/**
 * @brief Selector values to configure LNA regime
 */
typedef enum {
    SX1280_LNA_LOW_POWER_MODE,          /* !< Allow maximum efficiency of sx1280 (default) */
    SX1280_LNA_HIGH_SENSITIVITY_MODE,   /* !< Allow to use highest three steps of LNA gain and
                                           !< increase current consumption */
} SX1280_LnaSettings_t;
/**@}*/

/**
 * @brief Represents the length of the ID to check in ranging operation
 */
typedef enum {
    SX1280_RANGING_IDCHECK_LENGTH_08_BITS = 0x00,
    SX1280_RANGING_IDCHECK_LENGTH_16_BITS,
    SX1280_RANGING_IDCHECK_LENGTH_24_BITS,
    SX1280_RANGING_IDCHECK_LENGTH_32_BITS,
} SX1280_RangingIdCheckLengths_t;

/** Represents a sleep mode configuration
 *@{
 */
typedef struct {
    uint8_t WakeUpRTC : 1;                  //!< Get out of sleep mode if wakeup signal received from RTC
    uint8_t InstructionRamRetention : 1;    //!< InstructionRam is conserved during sleep
    uint8_t DataBufferRetention : 1;        //!< Data buffer is conserved during sleep
    uint8_t DataRamRetention : 1;           //!< Data ram is conserved during sleep
} sx1280_sleep_params_t;
/**@}*/

/** Represents the operating mode the radio is actually running
 *@{
 */
typedef enum {
    SX1280_MODE_SLEEP = 0x00,   //! The radio is in sleep mode
    SX1280_MODE_STDBY_RC,       //! The radio is in standby mode with RC oscillator
    SX1280_MODE_STDBY_XOSC,     //! The radio is in standby mode with XOSC oscillator
    SX1280_MODE_FS,             //! The radio is in frequency synthesis mode
    SX1280_MODE_TX,             //! The radio is in transmit mode
    SX1280_MODE_RX,             //! The radio is in receive mode
    SX1280_MODE_CAD             //! The radio is in channel activity detection mode
} SX1280_OperatingModes_t;

/*!
 * \brief Represents the possible spreading factor values in LORA packet types
 */
typedef enum {
    SX1280_LORA_SF5     = 0x50,
    SX1280_LORA_SF6     = 0x60,
    SX1280_LORA_SF7     = 0x70,
    SX1280_LORA_SF8     = 0x80,
    SX1280_LORA_SF9     = 0x90,
    SX1280_LORA_SF10    = 0xA0,
    SX1280_LORA_SF11    = 0xB0,
    SX1280_LORA_SF12    = 0xC0,
} SX1280_LoRaSpreadingFactors_t;

/*!
 * \brief Represents the coding rate values for LORA packet type
 */
typedef enum {
    SX1280_LORA_CR_4_5      = 0x01,
    SX1280_LORA_CR_4_6      = 0x02,
    SX1280_LORA_CR_4_7      = 0x03,
    SX1280_LORA_CR_4_8      = 0x04,
    SX1280_LORA_CR_LI_4_5   = 0x05,
    SX1280_LORA_CR_LI_4_6   = 0x06,
    SX1280_LORA_CR_LI_4_7   = 0x07,
} SX1280_LoRaCodingRates_t;

/**@}*/

/** Represents all possible opcode understood by the radio
 *@{
 */
typedef enum {
    SX1280_GET_STATUS               = 0xC0,
    SX1280_WRITE_REGISTER           = 0x18,
    SX1280_READ_REGISTER            = 0x19,
    SX1280_WRITE_BUFFER             = 0x1A,
    SX1280_READ_BUFFER              = 0x1B,
    SX1280_SET_SLEEP                = 0x84,
    SX1280_SET_STANDBY              = 0x80,
    SX1280_SET_FS                   = 0xC1,
    SX1280_SET_TX                   = 0x83,
    SX1280_SET_RX                   = 0x82,
    SX1280_SET_RXDUTYCYCLE          = 0x94,
    SX1280_SET_CAD                  = 0xC5,
    SX1280_SET_TXCONTINUOUSWAVE     = 0xD1,
    SX1280_SET_TXCONTINUOUSPREAMBLE = 0xD2,
    SX1280_SET_PACKETTYPE           = 0x8A,
    SX1280_GET_PACKETTYPE           = 0x03,
    SX1280_SET_RFFREQUENCY          = 0x86,
    SX1280_SET_TXPARAMS             = 0x8E,
    SX1280_SET_CADPARAMS            = 0x88,
    SX1280_SET_BUFFERBASEADDRESS    = 0x8F,
    SX1280_SET_MODULATIONPARAMS     = 0x8B,
    SX1280_SET_PACKETPARAMS         = 0x8C,
    SX1280_GET_RXBUFFERSTATUS       = 0x17,
    SX1280_GET_PACKETSTATUS         = 0x1D,
    SX1280_GET_RSSIINST             = 0x1F,
    SX1280_SET_DIOIRQPARAMS         = 0x8D,
    SX1280_GET_IRQSTATUS            = 0x15,
    SX1280_CLR_IRQSTATUS            = 0x97,
    SX1280_CALIBRATE                = 0x89,
    SX1280_SET_REGULATORMODE        = 0x96,
    SX1280_SET_SAVECONTEXT          = 0xD5,
    SX1280_SET_AUTOTX               = 0x98,
    SX1280_SET_AUTOFS               = 0x9E,
    SX1280_SET_LONGPREAMBLE         = 0x9B,
    SX1280_SET_UARTSPEED            = 0x9D,
    SX1280_SET_RANGING_ROLE         = 0xA3,
} SX1280_Commands_t;
/**@}*/

/*!
 * \brief Represents the interruption masks available for the radio
 *
 * \remark Note that not all these interruptions are available for all packet types
 */
typedef enum {
    SX1280_IRQ_NONE                             = 0x0000,
    SX1280_IRQ_TX_DONE                          = 0x0001,
    SX1280_IRQ_RX_DONE                          = 0x0002,
    SX1280_IRQ_SYNCWORD_VALID                   = 0x0004,
    SX1280_IRQ_SYNCWORD_ERROR                   = 0x0008,
    SX1280_IRQ_HEADER_VALID                     = 0x0010,
    SX1280_IRQ_HEADER_ERROR                     = 0x0020,
    SX1280_IRQ_CRC_ERROR                        = 0x0040,
    SX1280_IRQ_RANGING_SLAVE_RESPONSE_DONE      = 0x0080,
    SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARDED  = 0x0100,
    SX1280_IRQ_RANGING_MASTER_RESULT_VALID      = 0x0200,
    SX1280_IRQ_RANGING_MASTER_TIMEOUT           = 0x0400,
    SX1280_IRQ_RANGING_SLAVE_REQUEST_VALID      = 0x0800,
    SX1280_IRQ_CAD_DONE                         = 0x1000,
    SX1280_IRQ_CAD_DETECTED                     = 0x2000,
    SX1280_IRQ_RX_TX_TIMEOUT                    = 0x4000,
    SX1280_IRQ_PREAMBLE_DETECTED                = 0x8000,
    SX1280_IRQ_ALL                              = 0xFFFF,
} SX1280_IrqMasks_t;

/** Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 *@{
 */
typedef enum {
    SX1280_STDBY_RC     = 0x00,     /* Used to reduce the energy consumption */
    SX1280_STDBY_XOSC   = 0x01,     /* Used for time critical applications */
} SX1280_StandbyModes_t;
/**@}*/

/** Represents the tick size available for Rx/Tx timeout operations
 *@{
 */
typedef enum {
    SX1280_TICK_SIZE_0015_US    = 0x00,
    SX1280_TICK_SIZE_0062_US    = 0x01,
    SX1280_TICK_SIZE_1000_US    = 0x02,
    SX1280_TICK_SIZE_4000_US    = 0x03,
} SX1280_TickSizes_t;
/**@}*/

/**
 * @brief The standards channels specified by Multitech
 */
typedef enum {
    SX1280_CHANNEL_0    = 2403000000UL,
    SX1280_CHANNEL_1    = 2479000000UL,
    SX1280_CHANNEL_2    = 2425000000UL,
} SX1280_LoRaWANChannels_t;

/** Represents an amount of time measurable by the radio clock
 *
 * @code
 * Time = Step * NbSteps
 * Example:
 * Step = SX1280_TICK_SIZE_4000_US( 4 ms )
 * NbSteps = 1000
 * Time = 4e-3 * 1000 = 4 seconds
 * @endcode
 *@{
 */
typedef struct {
    SX1280_TickSizes_t Step;  //!< The step of ticktime
    /*!
     * \brief The number of steps for ticktime
     * Special values are:
     *     - 0x0000 for single mode
     *     - 0xFFFF for continuous mode
     */
    uint16_t NbSteps;
} sx1280_tick_time_t;
/**@}*/

/**
 * @brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 *@{
 */
typedef enum {
    SX1280_USE_LDO  = 0x00,     //! Use LDO (default value)
    SX1280_USE_DCDC = 0x01,     //! Use DCDC
} SX1280_RegulatorModes_t;
/**@}*/

/**
 * @brief Represents the role of the radio during ranging operations
 *@{
 */
typedef enum {
    SX1280_RANGING_ROLE_SLAVE,
    SX1280_RANGING_ROLE_MASTER,
} SX1280_RangingRoles_t;
/**@}*/

/**
 * @brief Represents the ramping time for power amplifier
 *@{
 */
typedef enum {
    SX1280_RAMP_02_US   = 0x00,
    SX1280_RAMP_04_US   = 0x20,
    SX1280_RAMP_06_US   = 0x40,
    SX1280_RAMP_08_US   = 0x60,
    SX1280_RAMP_10_US   = 0x80,
    SX1280_RAMP_12_US   = 0xA0,
    SX1280_RAMP_16_US   = 0xC0,
    SX1280_RAMP_20_US   = 0xE0,
} SX1280_RampTimes_t;
/**@}*/

/**
 * @brief Represents the number of symbols to be used for channel activity detection operation
 *@{
 */
typedef enum {
    SX1280_LORA_CAD_01_SYMBOL   = 0x00,
    SX1280_LORA_CAD_02_SYMBOL   = 0x20,
    SX1280_LORA_CAD_04_SYMBOL   = 0x40,
    SX1280_LORA_CAD_08_SYMBOL   = 0x60,
    SX1280_LORA_CAD_16_SYMBOL   = 0x80,
} SX1280_LoRaCadSymbols_t;
/**@}*/

/*!
 * \brief Represents the possible combinations of bitrate and bandwidth for
 *        GFSK and BLE packet types
 *
 * The bitrate is expressed in Mb/s and the bandwidth in MHz
 */
typedef enum {
    SX1280_GFSK_BLE_BR_2_000_BW_2_4 = 0x04,
    SX1280_GFSK_BLE_BR_1_600_BW_2_4 = 0x28,
    SX1280_GFSK_BLE_BR_1_000_BW_2_4 = 0x4C,
    SX1280_GFSK_BLE_BR_1_000_BW_1_2 = 0x45,
    SX1280_GFSK_BLE_BR_0_800_BW_2_4 = 0x70,
    SX1280_GFSK_BLE_BR_0_800_BW_1_2 = 0x69,
    SX1280_GFSK_BLE_BR_0_500_BW_1_2 = 0x8D,
    SX1280_GFSK_BLE_BR_0_500_BW_0_6 = 0x86,
    SX1280_GFSK_BLE_BR_0_400_BW_1_2 = 0xB1,
    SX1280_GFSK_BLE_BR_0_400_BW_0_6 = 0xAA,
    SX1280_GFSK_BLE_BR_0_250_BW_0_6 = 0xCE,
    SX1280_GFSK_BLE_BR_0_250_BW_0_3 = 0xC7,
    SX1280_GFSK_BLE_BR_0_125_BW_0_3 = 0xEF,
} SX1280_GfskBleBitrates_t;

/*!
 * \brief Represents the modulation index used in GFSK and BLE packet
 *        types
 */
typedef enum {
    SX1280_GFSK_BLE_MOD_IND_0_35,
    SX1280_GFSK_BLE_MOD_IND_0_50,
    SX1280_GFSK_BLE_MOD_IND_0_75,
    SX1280_GFSK_BLE_MOD_IND_1_00,
    SX1280_GFSK_BLE_MOD_IND_1_25,
    SX1280_GFSK_BLE_MOD_IND_1_50,
    SX1280_GFSK_BLE_MOD_IND_1_75,
    SX1280_GFSK_BLE_MOD_IND_2_00,
    SX1280_GFSK_BLE_MOD_IND_2_25,
    SX1280_GFSK_BLE_MOD_IND_2_50,
    SX1280_GFSK_BLE_MOD_IND_2_75,
    SX1280_GFSK_BLE_MOD_IND_3_00,
    SX1280_GFSK_BLE_MOD_IND_3_25,
    SX1280_GFSK_BLE_MOD_IND_3_50,
    SX1280_GFSK_BLE_MOD_IND_3_75,
    SX1280_GFSK_BLE_MOD_IND_4_00,
} SX1280_GfskBleModIndexes_t;

/*!
 * \brief Represents the modulation shaping parameter for GFSK, FLRC and BLE
 *        packet types
 */
typedef enum {
    SX1280_MOD_SHAPING_BT_OFF   = 0x00, //! No filtering
    SX1280_MOD_SHAPING_BT_1_0   = 0x10,
    SX1280_MOD_SHAPING_BT_0_5   = 0x20,
} SX1280_ModShapings_t;

/*!
 * \brief Represents the bandwidth values for LORA packet type
 */
typedef enum {
    SX1280_LORA_BW_0200 = 0x34,
    SX1280_LORA_BW_0400 = 0x26,
    SX1280_LORA_BW_0800 = 0x18,
    SX1280_LORA_BW_1600 = 0x0A,
} SX1280_LoRaBandwidths_t;

/*!
 * \brief Represents the possible combination of bitrate and bandwidth for FLRC
 *        packet type
 *
 * The bitrate is in Mb/s and the bitrate in MHz
 */
typedef enum {
    SX1280_FLRC_BR_1_300_BW_1_2 = 0x45,
    SX1280_FLRC_BR_1_040_BW_1_2 = 0x69,
    SX1280_FLRC_BR_0_650_BW_0_6 = 0x86,
    SX1280_FLRC_BR_0_520_BW_0_6 = 0xAA,
    SX1280_FLRC_BR_0_325_BW_0_3 = 0xC7,
    SX1280_FLRC_BR_0_260_BW_0_3 = 0xEB,
} SX1280_FlrcBitrates_t;

/*!
 * \brief The length of sync words for GFSK packet type
 */
typedef enum {
    SX1280_GFSK_SYNCWORD_LENGTH_1_BYTE  = 0x00, //!< Sync word length: 1 byte
    SX1280_GFSK_SYNCWORD_LENGTH_2_BYTE  = 0x02, //!< Sync word length: 2 bytes
    SX1280_GFSK_SYNCWORD_LENGTH_3_BYTE  = 0x04, //!< Sync word length: 3 bytes
    SX1280_GFSK_SYNCWORD_LENGTH_4_BYTE  = 0x06, //!< Sync word length: 4 bytes
    SX1280_GFSK_SYNCWORD_LENGTH_5_BYTE  = 0x08, //!< Sync word length: 5 bytes
} SX1280_SyncWordLengths_t;

/*!
 * \brief Represents the possible combinations of SyncWord correlators
 *        activated for GFSK and FLRC packet types
 */
typedef enum {
    SX1280_RX_MATCH_SYNCWORD_OFF    =
        0x00,  //!< No correlator turned on, i.e. do not search for SyncWord
    SX1280_RX_MATCH_SYNCWORD_1      = 0x10,
    SX1280_RX_MATCH_SYNCWORD_2      = 0x20,
    SX1280_RX_MATCH_SYNCWORD_1_2    = 0x30,
    SX1280_RX_MATCH_SYNCWORD_3      = 0x40,
    SX1280_RX_MATCH_SYNCWORD_1_3    = 0x50,
    SX1280_RX_MATCH_SYNCWORD_2_3    = 0x60,
    SX1280_RX_MATCH_SYNCWORD_1_2_3  = 0x70,
} SX1280_SyncWordRxMatchs_t;

/*!
 * \brief Represents the preamble length values for GFSK and FLRC packet
 *        types
 */
typedef enum {
    SX1280_PREAMBLE_LENGTH_04_BITS  = 0x00, //!< Preamble length: 04 bits
    SX1280_PREAMBLE_LENGTH_08_BITS  = 0x10, //!< Preamble length: 08 bits
    SX1280_PREAMBLE_LENGTH_12_BITS  = 0x20, //!< Preamble length: 12 bits
    SX1280_PREAMBLE_LENGTH_16_BITS  = 0x30, //!< Preamble length: 16 bits
    SX1280_PREAMBLE_LENGTH_20_BITS  = 0x40, //!< Preamble length: 20 bits
    SX1280_PREAMBLE_LENGTH_24_BITS  = 0x50, //!< Preamble length: 24 bits
    SX1280_PREAMBLE_LENGTH_28_BITS  = 0x60, //!< Preamble length: 28 bits
    SX1280_PREAMBLE_LENGTH_32_BITS  = 0x70, //!< Preamble length: 32 bits
} SX1280_PreambleLengths_t;

/*!
 *  \brief Radio packet length mode for GFSK and FLRC packet types
 */
typedef enum {
    SX1280_PACKET_FIXED_LENGTH      =
        0x00,                               //!< The packet is known on both sides, no header included in the packet
    SX1280_PACKET_VARIABLE_LENGTH   = 0x20, //!< The packet is on variable size, header included
} SX1280_PacketLengthModes_t;

/*!
 * \brief Represents the CRC length for GFSK and FLRC packet types
 *
 * \warning Not all configurations are available for both GFSK and FLRC
 *          packet type. Refer to the datasheet for possible configuration.
 */
typedef enum {
    SX1280_CRC_OFF      = 0x00, //!< No CRC in use
    SX1280_CRC_1_BYTES  = 0x10,
    SX1280_CRC_2_BYTES  = 0x20,
    SX1280_CRC_3_BYTES  = 0x30,
} SX1280_CrcTypes_t;

/**
 * @brief Represents the result type to be used in ranging operation
 */
typedef enum {
    SX1280_RANGING_RESULT_RAW       = 0x00,
    SX1280_RANGING_RESULT_AVERAGED  = 0x01,
    SX1280_RANGING_RESULT_DEBIASED  = 0x02,
    SX1280_RANGING_RESULT_FILTERED  = 0x03,
} SX1280_RangingResultTypes_t;

/*!
 * \brief Radio whitening mode activated or deactivated for GFSK, FLRC and
 *        BLE packet types
 */
typedef enum {
    SX1280__WHITENING_ON    = 0x00,
    SX1280__WHITENING_OFF   = 0x08,
} SX1280_WhiteningModes_t;

/*!
 * \brief Holds the packet length mode of a LORA packet type
 */
typedef enum {
    SX1280_LORA_PACKET_VARIABLE_LENGTH  = 0x00, //!< The packet is on variable size, header included
    SX1280_LORA_PACKET_FIXED_LENGTH     =
        0x80,                                   //!< The packet is known on both sides, no header included in the packet
    SX1280_LORA_PACKET_EXPLICIT         = SX1280_LORA_PACKET_VARIABLE_LENGTH,
    SX1280_LORA_PACKET_IMPLICIT         = SX1280_LORA_PACKET_FIXED_LENGTH,
} SX1280_LoRaPacketLengthsModes_t;

/*!
 * \brief Represents the CRC mode for LORA packet type
 */
typedef enum {
    SX1280_LORA_CRC_ON  = 0x20,     //!< CRC activated
    SX1280_LORA_CRC_OFF = 0x00,     //!< CRC not used
} SX1280_LoRaCrcModes_t;

/*!
 * \brief Represents the IQ mode for LORA packet type
 */
typedef enum {
    SX1280_LORA_IQ_NORMAL   = 0x40,
    SX1280_LORA_IQ_INVERTED = 0x00,
} SX1280_LoRaIQModes_t;

/*!
 * \brief Represents the CRC field length for BLE packet type
 */
typedef enum {
    SX1280_BLE_CRC_OFF  = 0x00,
    SX1280_BLE_CRC_3B   = 0x10,
} SX1280_BleCrcFields_t;

/*!
 * \brief Represents the specific packets to use in BLE packet type
 */
typedef enum {
    SX1280_BLE_PRBS_9       = 0x00, //!< Pseudo Random Binary Sequence based on 9th degree polynomial
    SX1280_BLE_PRBS_15      = 0x0C, //!< Pseudo Random Binary Sequence based on 15th degree polynomial
    SX1280_BLE_EYELONG_1_0  = 0x04, //!< Repeated '11110000' sequence
    SX1280_BLE_EYELONG_0_1  = 0x18, //!< Repeated '00001111' sequence
    SX1280_BLE_EYESHORT_1_0 = 0x08, //!< Repeated '10101010' sequence
    SX1280_BLE_EYESHORT_0_1 = 0x1C, //!< Repeated '01010101' sequence
    SX1280_BLE_ALL_1        = 0x10, //!< Repeated '11111111' sequence
    SX1280_BLE_ALL_0        = 0x14, //!< Repeated '00000000' sequence
} SX1280_BlePacketTypes_t;

/*!
 * \brief Represents the connection state for BLE packet type
 */
typedef enum {
    SX1280_BLE_PAYLOAD_LENGTH_MAX_31_BYTES  = 0x00,
    SX1280_BLE_PAYLOAD_LENGTH_MAX_37_BYTES  = 0x20,
    SX1280_BLE_TX_TEST_MODE                 = 0x40,
    SX1280_BLE_PAYLOAD_LENGTH_MAX_255_BYTES = 0x80,
} SX1280_BleConnectionStates_t;

/*!
 * \brief Represents the SyncWord length for FLRC packet type
 */
typedef enum {
    SX1280_FLRC_NO_SYNCWORD             = 0x00,
    SX1280_FLRC_SYNCWORD_LENGTH_4_BYTE  = 0x04,
} SX1280_FlrcSyncWordLengths_t;

/*!
 * \brief Represents the possible packet type (i.e. modem) used
 */
typedef enum {
    SX1280_PACKET_TYPE_GFSK,
    SX1280_PACKET_TYPE_LORA,
    SX1280_PACKET_TYPE_RANGING,
    SX1280_PACKET_TYPE_FLRC,
    SX1280_PACKET_TYPE_BLE,
    SX1280_PACKET_TYPE_NONE = 0x0F,
} SX1280_PacketTypes_t;

/**
 * @brief   SX1280 hardware and global parameters.
 */
typedef struct {
    spi_t spi;          /**< SPI device */
    gpio_t nss_pin;     /**< SPI NSS pin */
    gpio_t reset_pin;   /**< Reset pin */
    gpio_t dio0_pin;    /**< Interrupt line DIO0 (Tx done) */
    gpio_t dio1_pin;    /**< Interrupt line DIO1 (Rx timeout) */
    gpio_t dio2_pin;    /**< Interrupt line DIO2 (FHSS channel change) */
    gpio_t dio3_pin;    /**< Interrupt line DIO3 (CAD done) */
    gpio_t dio4_pin;    /**< Interrupt line DIO4 (not used) */
    gpio_t dio5_pin;    /**< Interrupt line DIO5 (not used) */
    uint8_t paselect;   /**< Power amplifier mode (RFO or PABOOST) */
} sx1280_params_t;

/**
 * @brief The type describing the packet parameters for every packet types
 */
typedef struct {
    SX1280_PacketTypes_t PacketType;  //!< Packet to which the packet parameters are referring to
    struct {
        /**
         * @brief Holds the GFSK packet parameters
         */
        struct {
            SX1280_PreambleLengths_t PreambleLength;    //!< The preamble length for GFSK packet type
            SX1280_SyncWordLengths_t
                SyncWordLength;                         //!< The synchronization word length for GFSK packet type
            SX1280_SyncWordRxMatchs_t SyncWordMatch;    //!< The synchronization correlator to use to
            //!< check synchronization word
            SX1280_PacketLengthModes_t
                HeaderType;                     //!< If the header is explicit, it will be transmitted in the GFSK
            //!< packet. If the header is implicit, it will not be transmitted
            uint8_t PayloadLength;              //!< Size of the payload in the GFSK packet
            SX1280_CrcTypes_t CrcLength;        //!< Size of the CRC block in the GFSK packet
            SX1280_WhiteningModes_t Whitening;  //!< Usage of whitening on payload and CRC blocks
            //!< plus header block if header type is variable
        } Gfsk;
        /**
         * @brief Holds the LORA packet parameters
         */
        struct {
            SX1280_PreambleLengths_t
                PreambleLength;  //!< The preamble length is the number of LORA symbols in the
            //!< preamble. To set it, use the following formula
            //!< @code Number of symbols = PreambleLength[3:0] * (
            //!< 2^PreambleLength[7:4] ) @endcode
            SX1280_LoRaPacketLengthsModes_t
                HeaderType;                 //!< If the header is explicit, it will be transmitted in the LORA
            //!< packet. If the header is implicit, it will not be transmitted
            uint8_t PayloadLength;          //!< Size of the payload in the LORA packet
            SX1280_LoRaCrcModes_t CrcMode;  //!< Size of CRC block in LORA packet
            SX1280_LoRaIQModes_t InvertIQ;  //!< Allows to swap IQ for LORA packet
        } LoRa;
        /**
         * @brief Holds the FLRC packet parameters
         */
        struct {
            SX1280_PreambleLengths_t PreambleLength;    //!< The preamble length for FLRC packet type
            SX1280_FlrcSyncWordLengths_t
                SyncWordLength;                         //!< The synchronization word length for FLRC packet type
            SX1280_SyncWordRxMatchs_t SyncWordMatch;    //!< The synchronization correlator to use to
            //!< check synchronization word
            SX1280_PacketLengthModes_t
                HeaderType;                     //!< If the header is explicit, it will be transmitted in the FLRC
            //!< packet. If the header is implicit, it will not be transmitted.
            uint8_t PayloadLength;              //!< Size of the payload in the FLRC packet
            SX1280_CrcTypes_t CrcLength;        //!< Size of the CRC block in the FLRC packet
            SX1280_WhiteningModes_t Whitening;  //!< Usage of whitening on payload and CRC blocks
            //!< plus header block if header type is variable
        } Flrc;
        /**
         * @brief Holds the BLE packet parameters
         */
        struct {
            SX1280_BleConnectionStates_t ConnectionState;   //!< The BLE state
            SX1280_BleCrcFields_t CrcField;                 //!< Size of the CRC block in the BLE packet
            SX1280_BlePacketTypes_t BlePacketType;          //!< Special BLE packet types
            SX1280_WhiteningModes_t
                Whitening;                                  //!< Usage of whitening on PDU and CRC blocks of BLE packet
        } Ble;
    } Params;                                               //!< Holds the packet parameters structure
} sx1280_packet_params_t;
/**@}*/

/*!
 * \brief Structure describing the radio status
 */
typedef union {
    /*!
     * \brief Structure of the radio status
     */
    struct {
        uint8_t CpuBusy : 1;    //!< Flag for CPU radio busy
        uint8_t DmaBusy : 1;    //!< Flag for DMA busy
        uint8_t CmdStatus : 3;  //!< Command status
        uint8_t ChipMode : 3;   //!< Chip mode
    } Fields;

    /*!
     * \brief Serialized radio status
     */
    uint8_t Value;
} SX1280_Status_t;
/**@}*/

/**
 * @brief Represents the packet status for every packet type
 */
typedef struct {
    SX1280_PacketTypes_t packetType;  //!< Packet to which the packet status are referring to.
    union {
        struct {
            int8_t RssiAvg;                     //!< The averaged RSSI
            int8_t RssiSync;                    //!< The RSSI measured on last packet
            struct {
                bool SyncError : 1;             //!< SyncWord error on last packet
                bool LengthError : 1;           //!< Length error on last packet
                bool CrcError : 1;              //!< CRC error on last packet
                bool AbortError : 1;            //!< Abort error on last packet
                bool HeaderReceived : 1;        //!< Header received on last packet
                bool PacketReceived : 1;        //!< Packet received
                bool PacketControlerBusy : 1;   //!< Packet controller busy
            } ErrorStatus;                      //!< The error status Byte
            struct {
                bool RxNoAck : 1;               //!< No acknowledgment received for Rx with variable length
                //!< packets
                bool PacketSent : 1;            //!< Packet sent, only relevant in Tx mode
            } TxRxStatus;                       //!< The Tx/Rx status Byte
            uint8_t SyncAddrStatus : 3;         //!< The id of the correlator who found the packet
        } Gfsk;
        struct {
            int8_t RssiPkt;                     //!< The RSSI of the last packet
            int8_t SnrPkt;                      //!< The SNR of the last packet
            struct {
                bool SyncError : 1;             //!< SyncWord error on last packet
                bool LengthError : 1;           //!< Length error on last packet
                bool CrcError : 1;              //!< CRC error on last packet
                bool AbortError : 1;            //!< Abort error on last packet
                bool HeaderReceived : 1;        //!< Header received on last packet
                bool PacketReceived : 1;        //!< Packet received
                bool PacketControlerBusy : 1;   //!< Packet controller busy
            } ErrorStatus;                      //!< The error status Byte
            struct {
                bool RxNoAck : 1;               //!< No acknowledgment received for Rx with variable length
                //!< packets
                bool PacketSent : 1;            //!< Packet sent, only relevant in Tx mode
            } TxRxStatus;                       //!< The Tx/Rx status Byte
            uint8_t SyncAddrStatus : 3;         //!< The id of the correlator who found the packet
        } LoRa;
        struct {
            int8_t RssiAvg;                     //!< The averaged RSSI
            int8_t RssiSync;                    //!< The RSSI of the last packet
            struct {
                bool SyncError : 1;             //!< SyncWord error on last packet
                bool LengthError : 1;           //!< Length error on last packet
                bool CrcError : 1;              //!< CRC error on last packet
                bool AbortError : 1;            //!< Abort error on last packet
                bool HeaderReceived : 1;        //!< Header received on last packet
                bool PacketReceived : 1;        //!< Packet received
                bool PacketControlerBusy : 1;   //!< Packet controller busy
            } ErrorStatus;                      //!< The error status Byte
            struct {
                uint8_t RxPid : 2;              //!< PID of the Rx
                bool RxNoAck : 1;               //!< No acknowledgment received for Rx with variable length
                //!< packets
                bool RxPidErr : 1;              //!< Received PID error
                bool PacketSent : 1;            //!< Packet sent, only relevant in Tx mode
            } TxRxStatus;                       //!< The Tx/Rx status Byte
            uint8_t SyncAddrStatus : 3;         //!< The id of the correlator who found the packet
        } Flrc;
        struct {
            int8_t RssiAvg;                     //!< The averaged RSSI
            int8_t RssiSync;                    //!< The RSSI of the last packet
            struct {
                bool SyncError : 1;             //!< SyncWord error on last packet
                bool LengthError : 1;           //!< Length error on last packet
                bool CrcError : 1;              //!< CRC error on last packet
                bool AbortError : 1;            //!< Abort error on last packet
                bool HeaderReceived : 1;        //!< Header received on last packet
                bool PacketReceived : 1;        //!< Packet received
                bool PacketControlerBusy : 1;   //!< Packet controller busy
            } ErrorStatus;                      //!< The error status Byte
            struct {
                bool PacketSent : 1;            //!< Packet sent, only relevant in Tx mode
            } TxRxStatus;                       //!< The Tx/Rx status Byte
            uint8_t SyncAddrStatus : 3;         //!< The id of the correlator who found the packet
        } Ble;
    } Params;                                   /** Contains all possible params */
} sx1280_packet_status_t;
/**@}*/

/**
 * @brief   LoRa configuration structure
 *@{
 */
typedef struct {
    uint16_t preamble_len;      /**< Length of preamble header */
    int8_t power;               /**< Signal power */
    uint8_t bandwidth;          /**< Signal bandwidth */
    uint32_t frequency;         /**< Frequency of the radio (can be SX1280_LoRaWANChannels_t) */
    uint8_t datarate;           /**< Spreading factor rate, e.g datarate */
    uint8_t coderate;           /**< Error coding rate */
    uint8_t freq_hop_period;    /**< Frequency hop period */
    struct {
        bool single : 1;
        bool continuous : 1;
    } flags;                /**< Flags for RX mode */
    uint32_t rx_timeout;    /**< RX timeout in microseconds */
    uint32_t tx_timeout;    /**< TX timeout in microseconds */
} sx1280_lora_settings_t;
/**@}*/

/**
 * @brief   SX1280 settings
 *@{
 */
typedef struct {
    SX1280_PacketTypes_t packet;        /**< Packet type */
    SX1280_OperatingModes_t mode;       /**< Operating  Mode */
    SX1280_States_t state;              /**< State of the radio */
    SX1280_RegulatorModes_t regmode;    /**< Regulator Mode */
    bool PollingMode;                   /**< Polling state of the driver */
    sx1280_lora_settings_t lora;        /**< LoRa settings */

} sx1280_settings_t;

/**
 * @brief The type describing the modulation parameters for every packet types
 *@{
 */
typedef struct {
    SX1280_PacketTypes_t
        PacketType;     //!< Packet to which the modulation parameters are referring to.
                        //    union
    struct {
        /*!
         * \brief Holds the GFSK modulation parameters
         *
         * In GFSK modulation, the bit-rate and bandwidth are linked together. In this structure,
         * its values are set using the same token.
         */
        struct {
            SX1280_GfskBleBitrates_t BitrateBandwidth;  //!< The bandwidth and bit-rate values for
            //!< BLE and GFSK modulations
            SX1280_GfskBleModIndexes_t
                ModulationIndex;    //!< The coding rate for BLE and GFSK modulations
            SX1280_ModShapings_t
                ModulationShaping;  //!< The modulation shaping for BLE and GFSK modulations
        } Gfsk;
        /*!
         * \brief Holds the LORA modulation parameters
         *
         * LORA modulation is defined by Spreading Factor (SF), Bandwidth and Coding Rate
         */
        struct {
            SX1280_LoRaSpreadingFactors_t
                SpreadingFactor;                    //!< Spreading Factor for the LORA modulation
            SX1280_LoRaBandwidths_t Bandwidth;      //!< Bandwidth for the LORA modulation
            SX1280_LoRaCodingRates_t CodingRate;    //!< Coding rate for the LORA modulation
        } LoRa;
        /*!
         * \brief Holds the FLRC modulation parameters
         *
         * In FLRC modulation, the bit-rate and bandwidth are linked together. In this structure,
         * its values are set using the same token.
         */
        struct {
            SX1280_FlrcBitrates_t
                BitrateBandwidth;                   //!< The bandwidth and bit-rate values for FLRC modulation
            uint8_t CodingRate;                     //!< The coding rate for FLRC modulation
            SX1280_ModShapings_t ModulationShaping; //!< The modulation shaping for FLRC modulation
        } Flrc;
        /*!
         * \brief Holds the BLE modulation parameters
         *
         * In BLE modulation, the bit-rate and bandwidth are linked together. In this structure, its
         * values are set using the same token.
         */
        struct {
            SX1280_GfskBleBitrates_t BitrateBandwidth;  //!< The bandwidth and bit-rate values for
            //!< BLE and GFSK modulations
            SX1280_GfskBleModIndexes_t
                ModulationIndex;    //!< The coding rate for BLE and GFSK modulations
            SX1280_ModShapings_t
                ModulationShaping;  //!< The modulation shaping for BLE and GFSK modulations
        } Ble;
    } Params;                       //!< Holds the modulation parameters structure
} sx1280_modulation_params_t;
/**@}*/

/**
 * @brief   SX1280 internal data.
 */
typedef struct {
    /* Data that will be passed to events handler in application */
    xtimer_t tx_timeout_timer;  /**< TX operation timeout timer */
    xtimer_t rx_timeout_timer;  /**< RX operation timeout timer */
    uint32_t last_channel;      /**< Last channel in frequency hopping sequence */
    bool is_last_cad_success;   /**< Sign of success of last CAD operation (activity detected) */
} sx1280_internal_t;

/**
 * @brief   SX1280 device descriptor.
 * @extends netdev_t
 *@{
 */
typedef struct {
    netdev_t netdev;                        /**< Netdev parent struct */
    sx1280_settings_t settings;             /**< Radio settings */
    sx1280_params_t params;                 /**< Device driver parameters */
    uint8_t irq;                            /**< Device IRQ flags */
    sx1280_internal_t _internal;            /**< Internal sx1280 data used within the driver */
    sx1280_modulation_params_t modulation;  /**< Modulation Parameters */
    sx1280_packet_params_t packet;          /**< Packet Parameters */
    sx1280_packet_status_t pktStatus;       /**< Packet Status */
} sx1280_t;
/**@}*/

/**
 * @brief   Initializes the transceiver.
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return result of initialization
 */
int sx1280_init(sx1280_t *dev);

/**
 * @brief   Setup the SX1280
 *
 * @param[in] dev                      Device descriptor
 * @param[in] params                   Parameters for device initialization
 *
 * @return the netdev associated with the sx1280
 */
netdev_t *sx1280_setup(sx1280_t *dev, const sx1280_params_t *params);

/**
 * @brief   Resets the SX1280
 *
 * @param[in] dev                      The sx1280 device descriptor
 */
int sx1280_reset(const sx1280_t *dev);

/**
 * @brief   Initialize radio settings with default values
 *
 * @param[in] dev                      The sx1280 device pointer
 */
void sx1280_init_radio_settings(sx1280_t *dev);

/**
 * @brief   Wake up the device from sleep mode
 *
 * @param[in] dev                      The sx1280 device structure pointer
 */
void sx1280_wakeup(const sx1280_t *dev);

/**
 * @brief   Gets current state of transceiver.
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return radio state [SX1280_RF_IDLE, SX1280_RF_RX_RUNNING, SX1280_RF_TX_RUNNING]
 */
SX1280_States_t sx1280_get_state(const sx1280_t *dev);

/**
 * @brief   Sets current state of transceiver.
 *
 * @param[in] dev                      The sx1280 device descriptor
 * @param[in] state                    The new radio state
 *
 * @return radio state [SX1280_RF_IDLE, SX1280_RF_RX_RUNNING, SX1280_RF_TX_RUNNING]
 */
void sx1280_set_state(sx1280_t *dev, SX1280_States_t state);

/**
 * @brief Estimation of the TOA (time on air) of a packet
 *
 * @returns       the TOA [us]
 */
double sx1280_get_time_on_air(const sx1280_t *dev, uint8_t len);

/**
 * @brief   Generates 32 bits random value based on the RSSI readings
 *
 * @attention This function sets the radio in LoRa mode and disables all
 *            interrupts from it. After calling this function either
 *            sx1280_set_rx_config or sx1280_set_tx_config functions must
 *            be called.
 *
 * @param[in] dev                      The sx1280 device structure pointer
 *
 * @return random 32 bits value
 */
uint32_t sx1280_random(sx1280_t *dev);

/**
 * @brief Set the radio in single RX mode, return to standy after
 * receiving the first packet
 */
void sx1280_set_rx_single_mode(sx1280_t *dev, bool single);

/**
 * @brief   Print the status of the device to stdout
 *
 * @param[in] dev                      The sx1280 device structure pointer
 */
void sx1280_pretty_status(sx1280_t *dev);
/**
 * @brief   Sets the synchronization word given by index used in GFSK, FLRC and BLE protocols.
 *
 * @remark 5th byte isn't used in FLRC and BLE protocols
 *
 * @param[in]       dev                     The sx1280 device descriptor
 * @param[in]   syncWordIdx                 Index of SyncWord to be set [1..3]
 * @param[in]       syncword                The synchronization word
 *
 * @return          status                      [0: OK, 1: NOK]
 */
uint8_t sx1280_set_syncword(sx1280_t *dev, uint8_t syncWordIdx,
                            uint8_t *syncword);

/**
 * @brief Sets the radio in sleep mode
 *
 * @param[in]		dev                     The sx1280 device descriptor
 * @param[in]		sleepConfig                 The sleep configuration describing
 * data retention and RTC wake-up
 */
void sx1280_set_sleep(sx1280_t *dev, sx1280_sleep_params_t sleepConfig);

/**
 * @brief   Sets the radio in stand-by mode
 *
 * @param[in]  dev                      The sx1280 device descriptor
 * @param[in]  standbyConfig				 The standby mode to put the radio into
 */
void sx1280_set_standby(sx1280_t *dev, SX1280_StandbyModes_t standbyConfig);

/**
 * @brief   Sets the radio in reception mode.
 *
 * @param[in] dev                      The sx1280device descriptor
 * @param[in] timeout						Structure describing the reception
 * timeout value
 */
void sx1280_set_rx(sx1280_t *dev, sx1280_tick_time_t timeout);

/**
 * @brief Sets the radio in transmission mode
 *
 * @param[in]  dev                      The sx1280device descriptor
 * @param[in]  timeout                       Structure describing the transmission
 * timeout value
 */
void sx1280_set_tx(sx1280_t *dev, sx1280_tick_time_t timeout);

/**
 * @brief Clears the IRQs
 *
 * @param[in]  dev                      The sx1280device descriptor
 * @param[in]  irq           IRQ(s) to be cleared
 */
void sx1280_clear_irq_status(sx1280_t *dev, uint16_t irq);

/**
 * @brief Set the role of the radio during ranging operations
 *
 * @param[in] dev                      The sx1280device descriptor
 * @param[in] role                     Role of the radio
 */
void sx1280_set_ranging_role(sx1280_t *dev, SX1280_RangingRoles_t role);

/**
 * @brief   Gets the SX1280 bandwidth
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return the bandwidth
 */
SX1280_LoRaBandwidths_t sx1280_get_bandwidth(const sx1280_t *dev);

/**
 * @brief Gets the current Operation Mode of the Radio
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return      last operating mode
 */
SX1280_OperatingModes_t sx1280_get_operation_mode(sx1280_t *dev);

/**
 * @brief Sets the radio in FS mode
 *
 * @param[in] dev                      The sx1280 device descriptor
 */
void sx1280_set_fs(sx1280_t *dev);

/**
 * @brief Sets the Rx duty cycle management parameters
 *
 * @param[in] dev                      The sx1280 device descriptor
 * @param[in] step                          Structure describing reception
 * timeout value
 * @param[in] NbStepRx
 * @param[in] RxNbStepSleep
 */
void sx1280_set_rx_dutycycle(sx1280_t *dev, SX1280_TickSizes_t step,
                             uint16_t NbStepRx,
                             uint16_t RxNbStepSleep);

/**
 * @brief Returns the current device firmware version
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return             Firmware version
 */
uint16_t sx1280_get_firmware_version(sx1280_t *dev);

/**
 * @brief Sets the radio in CAD mode
 *
 * @param[in] dev                      The sx1280 device descriptor
 */
void sx1280_set_cad(sx1280_t *dev);

/**
 * @brief Sets the radio in continuous wave transmission mode
 *
 * @param[in] dev                      The sx1280 device descriptor
 */
void sx1280_set_tx_continuous_wave(sx1280_t *dev);

/**
 * @brief Sets the radio in continuous preamble transmission mode
 *
 * @param[in] dev                      The sx1280 device descriptor
 */
void sx1280_set_tx_continuous_preamble(sx1280_t *dev);

/**
 * @brief Sets the radio for the given protocol
 *
 * @param[in] dev                      The sx1280 device descriptor
 * @param[in] packetType	                The packet type [PACKET_TYPE_GFSK,
 * PACKET_TYPE_LORA, PACKET_TYPE_RANGING, PACKET_TYPE_FLRC,PACKET_TYPE_BLE]
 *
 * @remark This method has to be called before SetRfFrequency, SetModulationParams and
 * SetPacketParams
 */
void sx1280_set_packet_type(sx1280_t *dev, SX1280_PacketTypes_t packetType);

/**
 * @brief Get the packet type
 *
 * @param[in] dev                      The sx1280 device descriptor
 * @param[in] localCopy	               Retrieve data from cache or device
 *
 * @return The packet type
 */
SX1280_PacketTypes_t sx1280_get_packet_type(sx1280_t *dev, bool localCopy);

/**
 * @brief Sets the RF frequency
 *
 * @param[in]  dev                      The sx1280 device descriptor
 * @param[in]  frequency				    RF frequency [Hz]
 */
void sx1280_set_rf_frequency(sx1280_t *dev, uint32_t frequency);

/**
 * @brief Get the RF frequency
 *
 * @param[in]  dev                      The sx1280 device descriptor
 *
 * @return         RF frequency [Hz]
 */
uint32_t sx1280_get_rf_frequency(sx1280_t *dev);

/**
 * @brief Sets the transmission parameters
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  power                        RF output power [-18..13] dBm
 * @param[in]  rampTime				      Transmission ramp up time
 */
void sx1280_set_tx_params(sx1280_t *dev, int8_t power,
                          SX1280_RampTimes_t rampTime);

/**
 * @brief Sets the number of symbols to be used for Channel Activity Detection operation
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  cadSymbolNum                 The number of symbol to use for Channel
 * Activity Detection operations [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL, LORA_CAD_04_SYMBOL,
 * LORA_CAD_08_SYMBOL, LORA_CAD_16_SYMBOL]
 */
void sx1280_set_cad_params(sx1280_t *dev, SX1280_LoRaCadSymbols_t cadSymbolNum);

/**
 * @brief Sets the data buffer base address for transmission and reception
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  txBaseAddress                Transmission base address
 * @param[in]  rxBaseAddress                Reception base address
 */
void sx1280_set_buffer_base_address(sx1280_t *dev, uint8_t txBaseAddress,
                                    uint8_t rxBaseAddress);

/**
 * @brief Returns the instantaneous RSSI value for the last packet received
 *
 * @param[in]  dev                     The sx1280 device descriptor
 *
 * @return      rssiInst      Instantaneous RSSI
 */
int8_t sx1280_get_rssi_inst(sx1280_t *dev);

/**
 * @brief   Sets the IRQ mask and DIO masks
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  irqMask                      General IRQ mask
 * @param[in]  dio1Mask                     DIO1 mask
 * @param[in]  dio2Mask                     DIO2 mask
 * @param[in]  dio3Mask                     DIO3 mask
 */
void sx1280_set_dio_irq_params(sx1280_t *dev, uint16_t irqMask,
                               uint16_t dio1Mask,
                               uint16_t dio2Mask, uint16_t dio3Mask);

/**
 * @brief Returns the current IRQ status
 *
 * @param[in]  dev                     The sx1280 device descriptor
 *
 * @return      irqStatus     IRQ status
 */
uint16_t sx1280_get_irq_status(sx1280_t *dev);

/**
 * @brief Sets the power regulators operating mode
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  mode                         [0: LDO, 1:DC_DC]
 */
void sx1280_set_regulator_mode(sx1280_t *dev, SX1280_RegulatorModes_t mode);

/**
 * @brief Saves the current selected modem configuration into data RAM
 *
 * @param[in]  dev                     The sx1280 device descriptor
 */
void sx1280_set_save_context(sx1280_t *dev);

/**
 * @brief Sets the chip to automatically send a packet after the end of a packet reception
 *
 * @remark The offset is automatically compensated inside the function
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  time			            The delay in us after which a Tx is done
 */
void sx1280_set_auto_tx(sx1280_t *dev, uint16_t time);

/**
 * @brief Stop the chip from automatically sending a packet after the end of a packet reception
 * if previously activated with sx1280_set_AutoTx command
 *
 * \see sx1280_set_AutoTx
 * @param[in]  dev                     The sx1280 device descriptor
 */
void sx1280_stop_auto_tx(sx1280_t *dev);

/**
 * @brief Sets the chip to automatically receive a packet after the end of a packet transmission
 *
 * @remark The offset is automatically compensated inside the function
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  enable                       [0: Disable, 1: Enable]
 */
void sx1280_set_auto_fs(sx1280_t *dev, uint8_t enable);

/**
 * @brief Enables or disables long preamble detection mode
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  enable                       [0: Disable, 1: Enable]
 */
void sx1280_set_long_preamble(sx1280_t *dev, uint8_t enable);

/**
 * @brief Saves the payload to be send in the radio buffer
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  payload                      A pointer to the payload
 * @param[in]  size                        The size of the payload
 */
void sx1280_set_payload(sx1280_t *dev, uint8_t *payload, uint8_t size);

/**
 * @brief Sends a payload
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  payload                      A pointer to the payload to send
 * @param[in]  size                         The size of the payload to send
 * @param[in]  timeout                      The timeout for Tx operation
 */
void sx1280_send_payload(sx1280_t *dev, uint8_t *payload, uint8_t size,
                         sx1280_tick_time_t timeout);

/**
 * @brief Defines how many error bits are tolerated in sync word detection
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  errorBits                    Number of error bits supported to validate
 * the Sync word detection ( default is 4 bit, minimum is 1 bit )
 */
void sx1280_set_sync_word_error_tolerance(sx1280_t *dev, uint8_t errorBits);

/**
 * @brief Sets the Initial value for the LFSR used for the CRC calculation
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  seed                         Initial LFSR value ( 4 bytes )
 *
 */
uint8_t sx1280_set_crc_seed(sx1280_t *dev, uint8_t *seed);

/**
 * @brief Set the Access Address field of BLE packet
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  accessAddress                The access address to be used for next BLE
 * packet sent
 */
void sx1280_set_ble_access_address(sx1280_t *dev, uint32_t accessAddress);

/**
 * @brief Set the Access Address for Advertizer BLE packets
 *
 * All advertizer BLE packets must use a particular value for Access
 * Address field. This method sets it.
 *
 * @param[in]  dev                     The sx1280 device descriptor
 */
void sx1280_set_ble_advertizer_access_address(sx1280_t *dev);

/**
 * @brief Sets the seed used for the CRC calculation
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  seed                         The seed value
 *
 */
void sx1280_set_crc_polynomial(sx1280_t *dev, uint16_t seed);

/**
 * @brief Sets the Initial value of the LFSR used for the whitening in GFSK, FLRC and BLE protocols
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  seed                         Initial LFSR value
 */
void sx1280_set_whitening_seed(sx1280_t *dev, uint8_t seed);

/**
 * @brief Set the gain for LNA
 *
 * SX1280EnableManualGain must be call before using this function
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  gain                         The value of gain to set, refer to datasheet
 * for value meaning
 *
 * \see SX1280EnableManualGain, SX1280DisableManualGain
 */
void sx1280_set_manual_gain_value(sx1280_t *dev, uint8_t gain);

/**
 * @brief Sets ranging device id
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  address                      Device address
 */
void sx1280_set_device_ranging_address(sx1280_t *dev, uint32_t address);

/**
 * @brief Sets the device id to ping in a ranging request
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  address                      Address of the device to ping
 */
void sx1280_set_ranging_request_address(sx1280_t *dev, uint32_t address);

/**
 * @brief Return the last ranging result power indicator
 *
 * The value returned is not an absolute power measurement. It is
 * a relative power measurement.
 *
 * @param[in]  dev                     The sx1280 device descriptor
 *
 * @return      deltaThreshold  A relative power indicator
 */
uint8_t sx1280_get_ranging_power_delta_threshold_indicator(sx1280_t *dev);

/**
 * @brief Sets the standard processing delay between Master and Slave
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  cal                          RxTx delay offset for correcting ranging
 * bias.
 *
 * The calibration value reflects the group delay of the radio front end and
 * must be re-performed for each new SX1280 PCB design. The value is obtained
 * empirically by either conducted measurement in a known electrical length
 * coaxial RF cable (where the design is connectorised) or by radiated
 * measurement, at a known distance, where an antenna is present.
 * The result of the calibration process is that the SX1280 ranging result
 * accurately reflects the physical range, the calibration procedure therefore
 * removes the average timing error from the time-of-flight measurement for a
 * given design.
 *
 * The values are Spreading Factor dependents, and depend also of the board
 * design. Some typical values are provided in the next table.
 *
 * Spreading Factor | Calibration Value
 * ---------------- | -----------------
 *   SF5            |  12200
 *   SF6            |  12200
 *   SF7            |  12400
 *   SF8            |  12650
 *   SF9            |  12940
 *   SF10           |  13000
 *   SF11           |  13060
 *   SF12           |  13120
 */
void sx1280_set_ranging_calibration(sx1280_t *dev, uint16_t cal);

/**
 * @brief Gets the current radio status
 *
 * @param[in]  dev                     The sx1280 device descriptor
 *
 * @return      status        Radio status
 */
SX1280_Status_t sx1280_get_status(sx1280_t *dev);

/**
 * @brief Set the modulation parameters
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  modulationParams        The new modulation parameters
 */
void sx1280_set_modulation_params(sx1280_t *dev,
                                  sx1280_modulation_params_t *modulationParams);

/**
 * @brief Sets the packet parameters
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  modulationParams        The new packet parameters
 */
void sx1280_set_packet_params(sx1280_t *dev,
                              sx1280_packet_params_t *packetParams);

/**
 * @brief Gets the last received packet buffer status
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[out] payloadLength                Last received packet payload length
 * @param[out] rxStartBuffer                Last received packet buffer address pointer
 */
void sx1280_get_rx_buffer_status(sx1280_t *dev, uint8_t *payloadLength,
                                 uint8_t *rxStartBufferPointer);

/**
 * @brief Reads the payload received. If the received payload is longer
 * than maxSize, then the method returns 1 and do not set size and payload.
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[out] buffer                      A pointer to a buffer into which the payload
 * will be copied
 * @param[out] size                         A pointer to the size of the payload
 * received
 * @param[in]  maxSize                      The maximal size allowed to copy into the
 * buffer
 */
uint8_t sx1280_get_payload(sx1280_t *dev, uint8_t *buffer, uint8_t *size,
                           uint8_t maxSize);

/**
 * @brief Configure the LNA regime of operation
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  lnaSetting                   The LNA setting. Possible values are
 * LNA_LOW_POWER_MODE and LNA_HIGH_SENSITIVITY_MODE
 */
void sx1280_set_lna_gain_setting(sx1280_t *dev,
                                 const SX1280_LnaSettings_t lnaSetting);

/**
 * @brief Sets the number of bits used to check that ranging request match ranging ID
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @param[in]  length                       [0: 8 bits, 1: 16 bits, 2: 24 bits, 3: 32
 * bits]
 */
void sx1280_set_ranging_id_length(sx1280_t *dev,
                                  SX1280_RangingIdCheckLengths_t length);

// TO DO
/**
 * @brief Return the ranging result value
 *
 * @param[in]   dev                     The sx1280 device descriptor
 * @param [in]  resultType               Specifies the type of result.
 *                                       [0: RAW, 1: Averaged, 2: De-biased, 3:Filtered]
 *
 * @retval      ranging       The ranging measure filtered according to resultType [m]
 */
double sx1280_get_ranging_result(sx1280_t *dev,
                                 SX1280_RangingResultTypes_t resultType);

/**
 * @brief Return the Estimated Frequency Error in LORA and RANGING operations
 *
 * @param[in]  dev                     The sx1280 device descriptor
 * @return      efe                     The estimated frequency error [Hz]
 */
double sx1280_get_frequency_error(sx1280_t *dev);

/**
 * @brief Compute the two's complement for a register of size lower than
 *        32bits
 *
 * @param [in]  num            The register to be two's complemented
 * @param [in]  bitCnt         The position of the sign bit
 */
int32_t sx1280_complement2(const uint32_t num, const uint8_t bitCnt);

/**
 * @brief   Gets the SX1280 LoRa spreading factor
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return the spreading factor
 */
SX1280_LoRaSpreadingFactors_t sx1280_get_spreading_factor(const sx1280_t *dev);

/**
 * @brief   Gets the SX1280 LoRa coding rate
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return the current LoRa coding rate
 */
SX1280_LoRaCodingRates_t sx1280_get_coding_rate(const sx1280_t *dev);

/**
 * @brief Gets the last received packet payload length
 *
 * @param[in]  dev                      The sx1280 device descriptor
 * @param[out] packetStatus			       A structure of packet status
 */
void sx1280_get_packet_status(sx1280_t *dev,
                              sx1280_packet_status_t *packetStatus);

/**
 * @brief   Checks if the SX1280 CRC verification mode is enabled
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return true if crc is enable
 */
SX1280_LoRaCrcModes_t sx1280_get_crc(const sx1280_t *dev);

/**
 * @brief   Checks if the SX1280 LoRa inverted IQ mode is enabled/disabled
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return the LoRa IQ inverted mode
 */
SX1280_LoRaIQModes_t sx1280_get_iq_invert(const sx1280_t *dev);

/**
 * @brief   Checks if the SX1280 LoRa RX single mode is enabled/disabled
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return the LoRa single mode
 */
bool sx1280_get_rx_single(const sx1280_t *dev);

/**
 * @brief   Gets the SX1280 TX radio power
 *
 * @param[in] dev                      The sx1280 device descriptor
 *
 * @return the radio power
 */
uint8_t sx1280_get_tx_power(const sx1280_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* SX1280_H */
