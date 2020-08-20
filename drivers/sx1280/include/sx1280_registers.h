/*
 * Copyright (c) 2016 Unwired Devices <info@unwds.com>
 *               2017 Inria Chile
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @}
 */

/*
 * Copyright (C) 2020 Université Grenoble Alpes
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup    drivers_sx1280
 *
 * @file			sx1280_registers.h
 * @brief      Register definitions for SX1280
 *
 * @author     Julia CORREA REIS
 * @author     Pierre Millot
 */

#ifndef SX1280_REGISTERS_H
#define SX1280_REGISTERS_H

#ifdef __cplusplus
extern "C" {
#endif

/** The address of the register holding the firmware version MSB */
#define REG_LR_FIRMWARE_VERSION_MSB 0x0153

/** The address of the register holding the first byte defining the CRC seed
 *
 * @remark Only used for packet types GFSK and Flrc
 */
#define REG_LR_CRCSEEDBASEADDR 0x09C8

/**The address of the register holding the first byte defining the CRC polynomial
 *
 * @remark Only used for packet types GFSK and Flrc
 */
#define REG_LR_CRCPOLYBASEADDR 0x09C6

/** The address of the register holding the first byte defining the whitening seed
 *
 * @remark Only used for packet types GFSK, FLRC and BLE
 */
#define REG_LR_WHITSEEDBASEADDR 0x09C5

/** The address of the register holding the ranging id check length
 *
 * @remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGIDCHECKLENGTH 0x0931

/** The address of the register holding the device ranging id
 *
 * @remark Only used for packet type Ranging
 */
#define REG_LR_DEVICERANGINGADDR 0x0916

/** The address of the register holding the device ranging id
 *
 * @remark Only used for packet type Ranging
 */
#define REG_LR_REQUESTRANGINGADDR 0x0912

/** The address of the register holding ranging results configuration and the corresponding mask
 *
 * @remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTCONFIG 0x0924
#define MASK_RANGINGMUXSEL         0xCF

/** The address of the register holding the first byte of ranging results
 * Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTBASEADDR 0x0961

/** The address of the register allowing to read ranging results
 *
 * @remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTSFREEZE 0x097F

/** The address of the register holding the first byte of ranging calibration
 *
 * @remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRERXTXDELAYCAL 0x092C

/** The address of the register holding the ranging filter window size
 *
 * @remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGFILTERWINDOWSIZE 0x091E

/** The address of the register to reset for clearing ranging filter
 *
 * @remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTCLEARREG 0x0923
#define REG_RANGING_RSSI             0x0964

/** The default number of samples considered in built-in ranging filter */
#define DEFAULT_RANGING_FILTER_SIZE 127

/** The address of the register holding LORA packet parameters*/
#define REG_LR_PACKETPARAMS 0x903

/** The address of the register holding payload length
 *
 * @remark Do NOT try to read it directly. Use GetRxBuffer( ) instead.
 */
#define REG_LR_PAYLOADLENGTH 0x901

/** The address of the instruction RAM and its size */
#define IRAM_START_ADDRESS 0x8000
#define IRAM_SIZE          0x4000

/** The addresses of the registers holding SyncWords values
 *
 * @remark The addresses depends on the Packet Type in use, and not all SyncWords are available for
 * every Packet Type
 */
#define REG_LR_SYNCWORDBASEADDRESS1 0x09CE
#define REG_LR_SYNCWORDBASEADDRESS2 0x09D3
#define REG_LR_SYNCWORDBASEADDRESS3 0x09D8

/** The MSB address and mask used to read the estimated frequency error */
#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB  0x0954
#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK 0x0FFFFF

/* Defines how many bit errors are tolerated in sync word detection */
#define REG_LR_SYNCWORDTOLERANCE 0x09CD

/** Register for MSB Access Address (BLE) */
#define REG_LR_BLE_ACCESS_ADDRESS     0x09CF
#define BLE_ADVERTIZER_ACCESS_ADDRESS 0x8E89BED6

/** Register address and mask for LNA regime selection */
#define REG_LNA_REGIME  0x0891
#define MASK_LNA_REGIME 0xC0

/** Register and mask enabling manual gain control */
#define REG_ENABLE_MANUAL_GAIN_CONTROL 0x089F
#define MASK_MANUAL_GAIN_CONTROL       0x80

/** Register and mask controlling demodulation detection */
#define REG_DEMOD_DETECTION  0x0895
#define MASK_DEMOD_DETECTION 0xFE

/** Register and mask setting manual gain value */
#define REG_MANUAL_GAIN_VALUE  0x089E
#define MASK_MANUAL_GAIN_VALUE 0xF0

#ifdef __cplusplus
}
#endif

#endif /* SX1280_REGISTERS_H */
