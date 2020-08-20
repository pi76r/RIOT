/*
 * Copyright (C) 2020 Université Grenoble Alpes
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx1302
 * @{
 * @file        sx1250.h
 * @brief       Basic functionality of sx1250 driver
 *
 * @author      Pierre Millot
 * @}
 */

#ifndef SX1250_H
#define SX1250_H

#include "sx1302.h"

#ifdef _cplusplus
extern "C" {
#endif

typedef enum {
    SX1250_CALIBRATE                = 0x89,
    SX1250_CALIBRATE_IMAGE          = 0x98,
    SX1250_CLR_IRQ_STATUS           = 0x02,
    SX1250_STOP_TIMER_ON_PREAMBLE   = 0x9F,
    SX1250_SET_RFSWITCHMODE         = 0x9D,
    SX1250_GET_IRQ_STATUS           = 0x12,
    SX1250_GET_RX_BUFFER_STATUS     = 0x13,
    SX1250_GET_PACKET_STATUS        = 0x14,
    SX1250_READ_BUFFER              = 0x1E,
    SX1250_READ_REGISTER            = 0x1D,
    SX1250_SET_DIO_IRQ_PARAMS       = 0x08,
    SX1250_SET_MODULATION_PARAMS    = 0x8B,
    SX1250_SET_PA_CONFIG            = 0x95,
    SX1250_SET_PACKET_PARAMS        = 0x8C,
    SX1250_SET_PACKET_TYPE          = 0x8A,
    SX1250_SET_RF_FREQUENCY         = 0x86,
    SX1250_SET_BUFFER_BASE_ADDRESS  = 0x8F,
    SX1250_SET_SLEEP                = 0x84,
    SX1250_SET_STANDBY              = 0x80,
    SX1250_SET_RX                   = 0x82,
    SX1250_SET_TX                   = 0x83,
    SX1250_SET_TX_PARAMS            = 0x8E,
    SX1250_WRITE_BUFFER             = 0x0E,
    SX1250_WRITE_REGISTER           = 0x0D,
    SX1250_SET_TXCONTINUOUSWAVE     = 0xD1,
    SX1250_SET_TXCONTINUOUSPREAMBLE = 0xD2,
    SX1250_GET_STATUS               = 0xC0,
    SX1250_SET_REGULATORMODE        = 0x96,
    SX1250_SET_FS                   = 0xC1,
    SX1250_GET_DEVICE_ERRORS        = 0x17
} SX1250_op_code_t;

typedef enum {
    SX1250_STDBY_RC   = 0x00,
    SX1250_STDBY_XOSC = 0x01
} SX1250_standby_modes_t;

int sx1250_calibrate(sx1302_t *dev, uint8_t rf_chain, uint32_t freq_hz);

int sx1250_setup(sx1302_t *dev, uint8_t rf_chain, uint32_t freq_hz,
                 bool single_input_mode);

#ifdef _cplusplus
}
#endif

#endif
