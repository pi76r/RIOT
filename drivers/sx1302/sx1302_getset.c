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
 * @brief       Implementation of get and set functions for SX1302
 *
 * @author      Pierre Millot
 * @}
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "sx1302.h"
#include "sx1302_internal.h"
#include "sx1302_registers.h"

#define ENABLE_DEBUG SX1302_DEBUG
#include "debug.h"

uint64_t sx1302_get_eui(sx1302_t *dev) {
    int32_t val;

    uint64_t eui = 0;
    for (int i = 0; i < 8; i++) {
        sx1302_reg_write(dev, SX1302_REG_OTP_BYTE_ADDR_ADDR, i);
        xtimer_usleep(10000);
        val = sx1302_reg_read(dev, SX1302_REG_OTP_RD_DATA_RD_DATA);
        eui |= (uint64_t)((uint8_t)val) << (56 - (i * 8));
    }

    return eui;
}

int32_t sx1302_get_bandwidth_value(SX1302_Bandwidth_t bandwidth) {
    switch (bandwidth) {
        case SX1302_LORA_BW_500_KHZ:
            return 500000;
        case SX1302_LORA_BW_250_KHZ:
            return 250000;
        case SX1302_LORA_BW_125_KHZ:
            return 125000;
        default:
            return -1;
    }
}

int sx1302_tx_set_start_delay(sx1302_t *dev, uint8_t rf_chain,
                              uint8_t bandwidth, uint16_t *delay) {
    uint16_t tx_start_delay = SX1302_TX_START_DELAY_DEFAULT * 32;
    uint16_t radio_bw_delay = 0;
    uint16_t filter_delay   = 0;
    uint16_t modem_delay    = 0;
    int32_t bw_hz           = sx1302_get_bandwidth_value(bandwidth);
    uint8_t chirp_low_pass  = 0;

    if (bandwidth == SX1302_LORA_BW_125_KHZ) {
        radio_bw_delay = 19;
    } else if (bandwidth == SX1302_LORA_BW_250_KHZ) {
        radio_bw_delay = 24;
    } else if (bandwidth == SX1302_LORA_BW_500_KHZ) {
        radio_bw_delay = 21;
    } else {
        DEBUG_PUTS("ERROR: bandwidth not supported\n");
        return -1;
    }

    int32_t val =
        sx1302_reg_read(dev, SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_LOWPASS(0));
    chirp_low_pass = (uint8_t)val;
    filter_delay   = ((1 << chirp_low_pass) - 1) * 1e6 / bw_hz;
    modem_delay =
        8 * (32e6 / (32 * bw_hz)); /* if bw=125k then modem freq=4MHz */

    /* Compute total delay */
    tx_start_delay -= (radio_bw_delay + filter_delay + modem_delay);

    DEBUG(
        "INFO: tx_start_delay=%u (%u, radio_bw_delay=%u, "
        "filter_delay=%u, modem_delay=%u)\n",
        (uint16_t)tx_start_delay, SX1302_TX_START_DELAY_DEFAULT * 32,
        radio_bw_delay, filter_delay, modem_delay);

    /* Configure the SX1302 with the calculated delay */
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_START_DELAY_MSB_TX_START_DELAY(rf_chain),
        (uint8_t)(tx_start_delay >> 8));
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_START_DELAY_LSB_TX_START_DELAY(rf_chain),
        (uint8_t)(tx_start_delay >> 0));

    /* return tx_start_delay */
    *delay = tx_start_delay;

    return 0;
}

SX1302_TX_Status_t sx1302_tx_status(sx1302_t *dev, uint8_t rf_chain) {
    uint16_t read_value = sx1302_reg_read(
        dev, SX1302_REG_TX_TOP_TX_FSM_STATUS_TX_STATUS(rf_chain));
    // TODO SX1302_TX_OFF
    if (read_value == 0x80) {
        return SX1302_TX_FREE;
    } else if ((read_value == 0x30) || (read_value == 0x50) ||
               (read_value == 0x60) || (read_value == 0x70)) {
        return SX1302_TX_EMITTING;
    } else if ((read_value == 0x91) || (read_value == 0x92)) {
        return SX1302_TX_SCHEDULED;
    } else {
        DEBUG("ERROR: UNKNOWN TX STATUS 0x%02X\n", read_value);
        return SX1302_TX_STATUS_UNKNOWN;
    }
}

void sx1302_set_default(sx1302_t *dev) {
    static const int32_t channel_if[9] = {
        -400000, -200000, 0,       -400000, -200000,
        0,       -400000, -200000, -400000 /* lora service */
    };
    static const uint8_t channel_rfchain[9] = {1, 1, 1, 0, 0, 0, 0, 0, 1};
    sx1302_radio_settings_t settings;
    settings.channel           = SX1302_CHANNEL_DEFAULT;
    settings.state             = SX1302_RF_IDLE;
    settings.lora.preamble_len = SX1302_STD_LORA_PREAMBLE;
    settings.lora.power        = 14;
    settings.lora.bandwidth    = SX1302_LORA_BW_125_KHZ;
    settings.lora.datarate     = SX1302_DR_LORA_SF7;
    settings.lora.coderate     = SX1302_CR_LORA_4_5;
    settings.lora.rx_timeout   = 100000;
    settings.lora.tx_timeout   = 100000;

    dev->settings = settings;

    dev->rf_chain[0].enable            = true;
    dev->rf_chain[0].freq_hz           = SX1302_CHANNEL_DEFAULT;
    dev->rf_chain[0].rssi_tcomp        = (sx1302_rssi_tconf_t){0, 0, 0, 0, 0};
    dev->rf_chain[0].tx_enable         = false;
    dev->rf_chain[0].single_input_mode = false;

    dev->rf_chain[1].enable            = true;
    dev->rf_chain[1].freq_hz           = SX1302_CHANNEL_DEFAULT;
    dev->rf_chain[1].rssi_tcomp        = (sx1302_rssi_tconf_t){0, 0, 0, 0, 0};
    dev->rf_chain[1].tx_enable         = false;
    dev->rf_chain[1].single_input_mode = false;

    dev->board.lorawan_public = true;
    dev->board.clksrc         = 0;
    dev->board.full_duplex    = false;

    for (int i = 0; i < 8; i++) {
        dev->if_chain[i].enable = true;

        dev->if_chain[i].rf_chain = channel_rfchain[i];
        dev->if_chain[i].freq_hz  = channel_if[i];
        dev->if_chain[i].datarate = SX1302_DR_LORA_SF7;
    }

    dev->if_chain[8].enable    = true;
    dev->if_chain[8].rf_chain  = channel_rfchain[8];
    dev->if_chain[8].freq_hz   = channel_if[8];
    dev->if_chain[8].datarate  = SX1302_DR_LORA_SF7;
    dev->if_chain[8].bandwidth = SX1302_LORA_BW_125_KHZ;

    dev->lora_service.enable                  = true;
    dev->lora_service.rf_chain                = channel_rfchain[8];
    dev->lora_service.freq_hz                 = channel_if[8];
    dev->lora_service.datarate                = SX1302_DR_LORA_SF7;
    dev->lora_service.bandwidth               = SX1302_LORA_BW_125_KHZ;
    dev->lora_service.implicit_hdr            = 0;
    dev->lora_service.implicit_payload_length = 0;
    dev->lora_service.implicit_crc_en         = 0;
    dev->lora_service.implicit_coderate       = 0;
}

SX1302_States_t sx1302_get_state(sx1302_t *dev) {
    return dev->settings.state;
}

void sx1302_set_state(sx1302_t *dev, SX1302_States_t state) {
    dev->settings.state = state;
}
