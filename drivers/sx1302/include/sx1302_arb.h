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
 * @file
 * @brief       ARB for SX1302
 *
 * @author      Pierre Millot
 */

#ifndef SX1302_ARB_H
#define SX1302_ARB_H

#include "sx1302.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SX1302_AGC_RADIO_A_INIT_DONE   0x80
#define SX1302_AGC_RADIO_B_INIT_DONE   0x20

#define SX1302_AGC_RADIO_GAIN_AUTO  0xFF

/* size of the firmware IN BYTES (= twice the number of 14b       \
                words) */
#define SX1302_MCU_FW_SIZE 8192

#define SX1302_FW_VERSION_AGC 1 /* Expected version of AGC firmware */
#define SX1302_FW_VERSION_ARB 1 /* Expected version of arbiter firmware */

#define SX1302_MCU_AGC 0x01
#define SX1302_MCU_ARB 0x02

#define SX1302_AGC_MEM_ADDR 0x0000
#define SX1302_ARB_MEM_ADDR 0x2000

typedef struct {
    uint8_t ana_min;
    uint8_t ana_max;
    uint8_t ana_thresh_l;
    uint8_t ana_thresh_h;
    uint8_t dec_attn_min;
    uint8_t dec_attn_max;
    uint8_t dec_thresh_l;
    uint8_t dec_thresh_h1;
    uint8_t dec_thresh_h2;
    uint8_t chan_attn_min;
    uint8_t chan_attn_max;
    uint8_t chan_thresh_l;
    uint8_t chan_thresh_h;
    uint8_t deviceSel;      /* sx1250 only */
    uint8_t hpMax;          /* sx1250 only */
    uint8_t paDutyCycle;    /* sx1250 only */
} sx1302_agc_gain_params_t;

int sx1302_arb_load_firmware(sx1302_t *dev, const uint8_t *firmware);

int sx1302_arb_start(sx1302_t *dev, uint8_t version);

int sx1302_agc_load_firmware(sx1302_t *dev, const uint8_t *firmware);

int sx1302_agc_start(sx1302_t *dev, uint8_t version, uint8_t ana_gain,
                     uint8_t dec_gain, uint8_t fdd_mode);

#ifdef __cplusplus
}
#endif

#endif /* SX127X_ARB_H */
/** @} */
