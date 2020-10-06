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
 * @brief       Basic functionality of sx1302 driver
 *
 * @author      Pierre Millot
 * @}
 */
#include "sx1302.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "agc_fw_sx1250.var"
#include "arb_fw.var"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "sx1302_arb.h"
#include "sx1302_internal.h"
#include "sx1302_netdev.h"
#include "sx1302_registers.h"
#include "thread.h"
#include "xtimer.h"

#define ENABLE_DEBUG SX1302_DEBUG
#include "debug.h"

netdev_t *sx1302_setup(sx1302_t *dev, const sx1302_params_t *params) {
    netdev_t *netdev = (netdev_t *)dev;
    netdev->driver   = &sx1302_driver;
    dev->params      = *params;
    return netdev;
}

static int _init_spi(sx1302_t *dev) {
    int res;

    /* Setup SPI for SX1302 */

    // TODO should only be called once for the whole system
    spi_init(dev->params.spi);

    res = spi_init_cs(dev->params.spi, dev->params.nss_pin);

#ifdef MODULE_PERIPH_SPI_GPIO_MODE
    spi_gpio_mode_t gpio_modes = {
        .mosi = (GPIO_OUT | SX1302_DIO_PULL_MODE),
        .miso = (SX1302_DIO_PULL_MODE),
        .sclk = (GPIO_OUT | SX1302_DIO_PULL_MODE),
    };
    res += spi_init_with_gpio_mode(dev->params.spi, gpio_modes);
#endif

    if (res != SPI_OK) {
        DEBUG(
            "[sx1302] error: failed to initialize SPI_%i device "
            "(code %i)\n",
            dev->params.spi, res);
        return -1;
    }

    DEBUG("[sx1302] SPI_%i initialized with success\n", dev->params.spi);
    return 0;
}

int sx1302_init(sx1302_t *dev) {
    /* Do internal initialization routines */
    if (_init_spi(dev) < 0) {
        DEBUG("[sx1302] error: failed to initialize SPI\n");
        return -SX1302_ERR_SPI;
    }

    sx1302_reset(dev);

    return SX1302_INIT_OK;
}

void sx1302_reset(sx1302_t *dev) {
    gpio_init(dev->params.power_en_pin, GPIO_OUT);  // SX1302_POWER_EN_PIN
    gpio_init(dev->params.reset_pin, GPIO_OUT);     // SX1302_RESET_PIN
    xtimer_usleep(1000);

    // write output for SX1302 CoreCell power_enable
    gpio_set(dev->params.power_en_pin);
    xtimer_usleep(10000);
    // write output for SX1302 CoreCell reset
    gpio_set(dev->params.reset_pin);
    xtimer_usleep(10000);
    gpio_clear(dev->params.reset_pin);
    xtimer_usleep(10000);
}

void sx1302_radio_reset(sx1302_t *dev, uint8_t rf_chain) {
    uint16_t reg_radio_en;
    uint16_t reg_radio_rst;


    //sx1302_spi_acquire_set(false);
    //sx1302_spi_acquire(dev);


    /* Switch to SPI clock before reseting the radio */
    sx1302_reg_write(dev, SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL, 0x00);

    /* Enable the radio */
    reg_radio_en =
        SX1302_REG_SELECT(rf_chain, SX1302_REG_AGC_MCU_RF_EN_A_RADIO_EN,
                          SX1302_REG_AGC_MCU_RF_EN_B_RADIO_EN);
    sx1302_reg_write(dev, reg_radio_en, 0x01);

    /* Select the proper reset sequence depending on the radio type */
    reg_radio_rst =
        SX1302_REG_SELECT(rf_chain, SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST,
                          SX1302_REG_AGC_MCU_RF_EN_B_RADIO_RST);
    sx1302_reg_write(dev, reg_radio_rst, 0x01);
    xtimer_usleep(500000);
    sx1302_reg_write(dev, reg_radio_rst, 0x00);
    xtimer_usleep(10000);
    sx1302_reg_write(dev, reg_radio_rst, 0x01);
    xtimer_usleep(10000); /* wait for auto calibration to complete */
    DEBUG("INFO: reset sx1250 (RADIO_%s) done\n",
          SX1302_REG_SELECT(rf_chain, "A", "B"));


    //sx1302_spi_release(dev);
    //sx1302_spi_acquire_set(true);
}

void sx1302_radio_set_mode(sx1302_t *dev, uint8_t rf_chain) {
    uint16_t reg;

    /* Set the radio mode */
    reg =
        SX1302_REG_SELECT(rf_chain, SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A,
                          SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_B);

    DEBUG("Setting rf_chain_%u in sx1250 mode\n", rf_chain);
    sx1302_reg_write(dev, reg, 0x01);
}

int sx1302_radio_clock_select(sx1302_t *dev, uint8_t rf_chain) {
    /* Switch SX1302 clock from SPI clock to radio clock of the selected RF
     * chain */
    switch (rf_chain) {
        case 0:
            DEBUG_PUTS("Select Radio A clock");
            sx1302_reg_write(dev, SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL,
                             0x01);
            sx1302_reg_write(dev, SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL,
                             0x00);
            break;
        case 1:
            DEBUG_PUTS("Select Radio B clock");
            sx1302_reg_write(dev, SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL,
                             0x00);
            sx1302_reg_write(dev, SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL,
                             0x01);
            break;
        default:
            return -1;
    }

    /* Enable clock dividers */
    sx1302_reg_write(dev, SX1302_REG_CLK_CTRL_CLK_SEL_CLKDIV_EN, 0x01);

    /* Set the RIF clock to the 32MHz clock of the radio */
    sx1302_reg_write(dev, SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL, 0x01);

    return 0;
}

void sx1302_radio_calibrate(sx1302_t *dev, uint8_t clksrc) {
    /* -- Reset radios */
    for (int i = 0; i < 2; i++) {
        if (dev->rf_chain[i].enable) {
            sx1302_radio_reset(dev, i);
            sx1302_radio_set_mode(dev, i);
        }
    }

    /* -- Select the radio which provides the clock to the sx1302 */
    sx1302_radio_clock_select(dev, clksrc);

    /* -- Ensure PA/LNA are disabled */
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL, 1);
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_RF_EN_A_PA_EN, 0);
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_RF_EN_A_LNA_EN, 0);

    DEBUG_PUTS("Calibrating sx1250 radios");
    for (int i = 0; i < 2; i++) {
        if (dev->rf_chain[i].enable == true) {
            if (sx1250_calibrate(dev, i, dev->rf_chain[i].freq_hz)) {
                printf("ERROR: radio calibration failed\n");
                return;
            }
        }
    }
    /* -- Release control over FE */
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL, 0);
}

void sx1302_radio_host_ctrl(sx1302_t *dev, bool host_ctrl) {
    sx1302_reg_write(dev, SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, host_ctrl);
}

void sx1302_pa_lna_lut_configure(sx1302_t *dev) {
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_LUT_TABLE_A_PA_LUT,
                     0x04); /* Enable PA: RADIO_CTRL[2] is high when PA_EN=1
                               & LNA_EN=0 */
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_LUT_TABLE_B_PA_LUT,
                     0x04); /* Enable PA: RADIO_CTRL[8] is high when PA_EN=1
                               & LNA_EN=0 */
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_LUT_TABLE_A_LNA_LUT,
                     0x02); /* Enable LNA: RADIO_CTRL[1] is high when
                               PA_EN=0 & LNA_EN=1 */
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_LUT_TABLE_B_LNA_LUT,
                     0x02); /* Enable LNA: RADIO_CTRL[7] is high when
                               PA_EN=0 & LNA_EN=1 */
}

void sx1302_radio_fe_configure(sx1302_t *dev) {
    sx1302_reg_write(
        dev,
        SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_A_RSSI_BB_FILTER_ALPHA,
        0x03);
    sx1302_reg_write(
        dev,
        SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_A_RSSI_DEC_FILTER_ALPHA,
        0x07);
    sx1302_reg_write(
        dev,
        SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_B_RSSI_BB_FILTER_ALPHA,
        0x03);
    sx1302_reg_write(
        dev,
        SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_B_RSSI_DEC_FILTER_ALPHA,
        0x07);

    sx1302_reg_write(
        dev, SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_A_RSSI_DB_DEFAULT_VALUE, 23);
    sx1302_reg_write(
        dev, SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_A_RSSI_DEC_DEFAULT_VALUE,
        66);
    sx1302_reg_write(
        dev, SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_B_RSSI_DB_DEFAULT_VALUE, 23);
    sx1302_reg_write(
        dev, SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_B_RSSI_DEC_DEFAULT_VALUE,
        66);

    sx1302_reg_write(dev, SX1302_REG_RADIO_FE_CTRL0_RADIO_A_DC_NOTCH_EN, 1);
    sx1302_reg_write(dev, SX1302_REG_RADIO_FE_CTRL0_RADIO_A_HOST_FILTER_GAIN,
                     0x0b);
    sx1302_reg_write(dev, SX1302_REG_RADIO_FE_CTRL0_RADIO_B_DC_NOTCH_EN, 1);
    sx1302_reg_write(dev, SX1302_REG_RADIO_FE_CTRL0_RADIO_B_HOST_FILTER_GAIN,
                     0x0b);
}

void sx1302_channelizer_configure(sx1302_t *dev, sx1302_conf_rxif_t *if_cfg,
                                  bool fix_gain) {
    int32_t if_freq;
    uint8_t channels_mask = 0x00;

    /* Select which radio is connected to each multi-SF channel */
    for (int i = 0; i < 8; i++) {
        channels_mask |= (if_cfg[i].rf_chain << i);
    }
    DEBUG("LoRa multi-SF radio select: 0x%02X\n", channels_mask);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_RADIO_SELECT_RADIO_SELECT,
                     channels_mask);

    /* Select which radio is connected to the LoRa service channel */
    DEBUG("LoRa service radio select: 0x%02X\n", if_cfg[8].rf_chain);
    sx1302_reg_write(
        dev,
        SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_RADIO_SEL_RADIO_SELECT,
        if_cfg[8].rf_chain);

    /* Select which radio is connected to the FSK channel */
    DEBUG("FSK radio select %u\n", if_cfg[9].rf_chain);
    sx1302_reg_write(dev,
                     SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RADIO_SELECT,
                     if_cfg[9].rf_chain);

    /* Configure multi-SF channels IF frequencies */
    if_freq = SX1302_IF_HZ_TO_REG(if_cfg[0].freq_hz);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_0_MSB_IF_FREQ_0,
                     (if_freq >> 8) & 0x0000001F);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_0_LSB_IF_FREQ_0,
                     (if_freq >> 0) & 0x000000FF);

    if_freq = SX1302_IF_HZ_TO_REG(if_cfg[1].freq_hz);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_1_MSB_IF_FREQ_1,
                     (if_freq >> 8) & 0x0000001F);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_1_LSB_IF_FREQ_1,
                     (if_freq >> 0) & 0x000000FF);

    if_freq = SX1302_IF_HZ_TO_REG(if_cfg[2].freq_hz);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_2_MSB_IF_FREQ_2,
                     (if_freq >> 8) & 0x0000001F);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_2_LSB_IF_FREQ_2,
                     (if_freq >> 0) & 0x000000FF);

    if_freq = SX1302_IF_HZ_TO_REG(if_cfg[3].freq_hz);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_3_MSB_IF_FREQ_3,
                     (if_freq >> 8) & 0x0000001F);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_3_LSB_IF_FREQ_3,
                     (if_freq >> 0) & 0x000000FF);

    if_freq = SX1302_IF_HZ_TO_REG(if_cfg[4].freq_hz);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_4_MSB_IF_FREQ_4,
                     (if_freq >> 8) & 0x0000001F);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_4_LSB_IF_FREQ_4,
                     (if_freq >> 0) & 0x000000FF);

    if_freq = SX1302_IF_HZ_TO_REG(if_cfg[5].freq_hz);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_5_MSB_IF_FREQ_5,
                     (if_freq >> 8) & 0x0000001F);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_5_LSB_IF_FREQ_5,
                     (if_freq >> 0) & 0x000000FF);

    if_freq = SX1302_IF_HZ_TO_REG(if_cfg[6].freq_hz);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_6_MSB_IF_FREQ_6,
                     (if_freq >> 8) & 0x0000001F);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_6_LSB_IF_FREQ_6,
                     (if_freq >> 0) & 0x000000FF);

    if_freq = SX1302_IF_HZ_TO_REG(if_cfg[7].freq_hz);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_7_MSB_IF_FREQ_7,
                     (if_freq >> 8) & 0x0000001F);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FREQ_7_LSB_IF_FREQ_7,
                     (if_freq >> 0) & 0x000000FF);

    /* Configure LoRa service channel IF frequency */
    if_freq = SX1302_IF_HZ_TO_REG(if_cfg[8].freq_hz);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_MSB_IF_FREQ_0,
        (if_freq >> 8) & 0x0000001F);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_LSB_IF_FREQ_0,
        (if_freq >> 0) & 0x000000FF);

    /* Configure FSK channel IF frequency */
    if_freq = SX1302_IF_HZ_TO_REG(if_cfg[9].freq_hz);
    sx1302_reg_write(dev,
                     SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_MSB_IF_FREQ_0,
                     (if_freq >> 8) & 0x0000001F);
    sx1302_reg_write(dev,
                     SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_LSB_IF_FREQ_0,
                     (if_freq >> 0) & 0x000000FF);

    /* Set the low pass filtering corner frequency for RSSI indicator */
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_RSSI_CONTROL_RSSI_FILTER_ALPHA,
                     0x05);

    /* Set the channelizer RSSI reset value */
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_RSSI_DEF_VALUE_CHAN_RSSI_DEF_VALUE,
                     85);

    /* Force channelizer in fix gain, or let it be controlled by AGC */
    if (fix_gain) {
        sx1302_reg_write(dev, SX1302_REG_RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE,
                         0x00);
        sx1302_reg_write(dev, SX1302_REG_RX_TOP_GAIN_CONTROL_CHAN_GAIN, 5);
    } else {
        /* Allow the AGC to control gains */
        sx1302_reg_write(dev, SX1302_REG_RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE,
                         0x01);
        /* Disable the internal DAGC */
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_CHANN_DAGC_CFG1_CHAN_DAGC_THRESHOLD_HIGH,
            255);
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_CHANN_DAGC_CFG2_CHAN_DAGC_THRESHOLD_LOW, 0);
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_CHANN_DAGC_CFG3_CHAN_DAGC_MAX_ATTEN, 15);
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_CHANN_DAGC_CFG3_CHAN_DAGC_MIN_ATTEN, 0);
    }
}

void sx1302_lora_correlator_configure(sx1302_t *dev) {
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF5_CFG2_ACC_PNR, 52);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF5_CFG4_MSP_PNR, 24);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF5_CFG6_MSP_PEAK_NB, 7);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF5_CFG7_MSP2_PEAK_NB, 5);

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF6_CFG2_ACC_PNR, 52);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF6_CFG4_MSP_PNR, 24);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF6_CFG6_MSP_PEAK_NB, 7);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF6_CFG7_MSP2_PEAK_NB, 5);

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF7_CFG2_ACC_PNR, 52);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF7_CFG4_MSP_PNR, 24);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF7_CFG6_MSP_PEAK_NB, 7);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF7_CFG7_MSP2_PEAK_NB, 5);

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF8_CFG2_ACC_PNR, 52);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF8_CFG4_MSP_PNR, 24);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF8_CFG6_MSP_PEAK_NB, 7);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF8_CFG7_MSP2_PEAK_NB, 5);

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF9_CFG2_ACC_PNR, 52);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF9_CFG4_MSP_PNR, 24);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF9_CFG6_MSP_PEAK_NB, 7);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF9_CFG7_MSP2_PEAK_NB, 5);

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF10_CFG2_ACC_PNR, 52);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF10_CFG4_MSP_PNR, 24);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF10_CFG6_MSP_PEAK_NB, 7);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF10_CFG7_MSP2_PEAK_NB, 5);

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF11_CFG2_ACC_PNR, 52);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF11_CFG4_MSP_PNR, 24);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF11_CFG6_MSP_PEAK_NB, 7);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF11_CFG7_MSP2_PEAK_NB, 5);

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF12_CFG2_ACC_PNR, 52);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF12_CFG4_MSP_PNR, 24);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF12_CFG6_MSP_PEAK_NB, 7);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_SF12_CFG7_MSP2_PEAK_NB, 5);

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_CORR_CLOCK_ENABLE_CLK_EN, 0xFF);

    sx1302_reg_write(
        dev,
        SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ONLY_FIRST_DET_EDGE_ENABLE_ONLY_FIRST_DET_EDGE,
        0xFF);
    sx1302_reg_write(
        dev,
        SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ACC_CLEAR_ENABLE_CORR_ACC_CLEAR,
        0xFF);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_CORRELATOR_SF_EN_CORR_SF_EN,
                     0xFF); /* 12 11 10 9 8 7 6 5 */
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_CORRELATOR_EN_CORR_EN,
                     0xFF); /* 1 correlator per channel */
}

static int sx1302_calculate_freq_to_time_drift(uint32_t freq_hz, uint8_t bw,
                                               uint16_t *mant, uint8_t *exp) {
    uint64_t mantissa_u64;
    uint8_t exponent = 0;
    int32_t bw_hz;

    bw_hz = sx1302_get_bandwidth_value(bw);
    if (bw_hz < 0) {
        printf(
            "ERROR: Unsupported bandwidth for frequency to time "
            "drift calculation\n");
        return -1;
    }

    mantissa_u64 = (uint64_t)bw_hz * (2 << (20 - 1)) / freq_hz;
    while (mantissa_u64 < 2048) {
        exponent += 1;
        mantissa_u64 <<= 1;
    }

    *mant = (uint16_t)mantissa_u64;
    *exp  = exponent;

    return 0;
}

int sx1302_lora_modem_configure(sx1302_t *dev, uint32_t radio_freq_hz) {
    uint16_t mantissa = 0;
    uint8_t exponent  = 0;

    /* TODO: test if channel is enabled */

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_DC_NOTCH_CFG1_ENABLE, 0x00);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_RX_DFE_AGC1_FORCE_DEFAULT_FIR,
                     0x01);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_DAGC_CFG_GAIN_DROP_COMP, 0x01);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_DAGC_CFG_TARGET_LVL, 0x01);

    /* Enable full modems */
    DEBUG_PUTS("Configuring 8 full-SF modems");
    sx1302_reg_write(dev, SX1302_REG_OTP_MODEM_EN_0_MODEM_EN, 0xFF);

    /* Enable limited modems */
    DEBUG_PUTS("Configuring 8 limited-SF modems");
    sx1302_reg_write(dev, SX1302_REG_OTP_MODEM_EN_1_MODEM_EN, 0xFF);

    /* Configure coarse sync between correlators and modems */
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_MSB_MODEM_SYNC_DELTA, 0);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_LSB_MODEM_SYNC_DELTA, 126);

    /* Configure fine sync offset for each channel */
    sx1302_reg_write(
        dev, SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_0_OFFSET, 1);
    sx1302_reg_write(
        dev, SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_1_OFFSET, 5);
    sx1302_reg_write(
        dev, SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_2_OFFSET, 9);
    sx1302_reg_write(
        dev, SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_3_OFFSET, 13);
    sx1302_reg_write(
        dev, SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_4_OFFSET, 1);
    sx1302_reg_write(
        dev, SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_5_OFFSET, 5);
    sx1302_reg_write(
        dev, SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_6_OFFSET, 9);
    sx1302_reg_write(
        dev, SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_7_OFFSET, 13);

    /* Configure PPM offset */
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF5,
                     0x00);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF6,
                     0x00);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF7,
                     0x00);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF8,
                     0x00);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF9,
                     0x00);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF10,
                     0x00);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF11,
                     0x01);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF12,
                     0x01);

    /* Improve SF5 and SF6 performances */
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FINE_TIMING_A_1_GAIN_P_AUTO,
                     3);  // Default is 1
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FINE_TIMING_A_1_GAIN_P_PAYLOAD,
                     3);  // Default is 2

    /* Improve SF11/SF12 performances */
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF11, 1);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF12, 1);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF11, 1);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF12, 1);

    /* Freq2TimeDrift computation */
    if (sx1302_calculate_freq_to_time_drift(
            radio_freq_hz, SX1302_LORA_BW_125_KHZ, &mantissa, &exponent) != 0) {
        printf(
            "ERROR: failed to calculate frequency to time drift for "
            "LoRa modem\n");
        return -1;
    }
    DEBUG(
        "Freq2TimeDrift MultiSF: Mantissa = %d (0x%02X, 0x%02X), "
        "Exponent = %d (0x%02X)\n",
        mantissa, (mantissa >> 8) & 0x00FF, (mantissa)&0x00FF, exponent,
        exponent);
    sx1302_reg_write(dev,
                     SX1302_REG_RX_TOP_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT,
                     (mantissa >> 8) & 0x00FF);
    sx1302_reg_write(dev,
                     SX1302_REG_RX_TOP_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT,
                     (mantissa)&0x00FF);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP, exponent);

    /* Time drift compensation */
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB, 1);

    return 0;
}

int sx1302_lora_service_correlator_configure(sx1302_t *dev,
                                             sx1302_conf_rxif_t *cfg) {
    /* Common config for all SF */
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB, 7);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB, 5);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_USE_GAIN_SYMB, 1);

    switch (cfg->datarate) {
        case 5:
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN,
                1);
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR,
                52);
            break;
        case 6:
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN,
                1);
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR,
                52);
            break;
        case 7:
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN,
                0);
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR,
                52);
            break;
        case 8:
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN,
                0);
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR,
                52);
            break;
        case 9:
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN,
                0);
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR,
                52);
            break;
        case 10:
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN,
                0);
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR,
                52);
            break;
        case 11:
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN,
                0);
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR,
                52);
            break;
        case 12:
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN,
                0);
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR,
                52);
            break;
        default:
            printf(
                "ERROR: Failed to configure LoRa service modem "
                "correlators\n");
            return -1;
    }

    return 0;
}

int sx1302_lora_service_modem_configure(sx1302_t *dev, sx1302_conf_rxif_t *cfg,
                                        uint32_t radio_freq_hz) {
    uint16_t mantissa = 0;
    uint8_t exponent  = 0;

    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_ENABLE, 0x00);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_FORCE_DEFAULT_FIR,
        0x01);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_GAIN_DROP_COMP, 0x01);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_TARGET_LVL, 0x01);

    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_AUTO, 0x03);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_PAYLOAD,
        0x03);

    switch (cfg->datarate) {
        case SX1302_DR_LORA_SF5:
        case SX1302_DR_LORA_SF6:
            sx1302_reg_write(
                dev,
                SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB,
                0x04);  // Default value
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN,
                0x00);  // Default value
            break;
        case SX1302_DR_LORA_SF7:
        case SX1302_DR_LORA_SF8:
        case SX1302_DR_LORA_SF9:
        case SX1302_DR_LORA_SF10:
            sx1302_reg_write(
                dev,
                SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB,
                0x06);
            sx1302_reg_write(
                dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN,
                0x00);
            break;
        case SX1302_DR_LORA_SF11:
        case SX1302_DR_LORA_SF12:
            sx1302_reg_write(
                dev,
                SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB,
                0x07);
            switch (cfg->bandwidth) {
                case SX1302_LORA_BW_125_KHZ:
                    sx1302_reg_write(
                        dev,
                        SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN,
                        0x01);
                    break;
                case SX1302_LORA_BW_250_KHZ:
                    sx1302_reg_write(
                        dev,
                        SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN,
                        0x02);
                    break;
                case SX1302_LORA_BW_500_KHZ:
                    sx1302_reg_write(
                        dev,
                        SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN,
                        0x03);
                    break;
                default:
                    printf(
                        "ERROR: unsupported bandwidth %u for LoRa "
                        "Service modem\n",
                        cfg->bandwidth);
                    break;
            }
            break;
        default:
            printf(
                "ERROR: unsupported datarate %u for LoRa Service "
                "modem\n",
                cfg->datarate);
            break;
    }

    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_IMPLICIT_HEADER,
        cfg->implicit_hdr);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CRC_EN,
                     cfg->implicit_crc_en);
    sx1302_reg_write(dev,
                     SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_CODING_RATE,
                     cfg->implicit_coderate);
    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG3_PAYLOAD_LENGTH,
        cfg->implicit_payload_length);

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_SF,
                     cfg->datarate);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_BW,
                     cfg->bandwidth);
    sx1302_reg_write(dev,
                     SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_PPM_OFFSET,
                     SX1302_SET_PPM_ON(cfg->bandwidth, cfg->datarate));

    /* Freq2TimeDrift computation */
    if (sx1302_calculate_freq_to_time_drift(radio_freq_hz, cfg->bandwidth,
                                            &mantissa, &exponent) != 0) {
        printf(
            "ERROR: failed to calculate frequency to time drift for "
            "LoRa service modem\n");
        return -1;
    }
    sx1302_reg_write(
        dev,
        SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT,
        (mantissa >> 8) & 0x00FF);
    sx1302_reg_write(
        dev,
        SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT,
        (mantissa)&0x00FF);
    sx1302_reg_write(
        dev,
        SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP,
        exponent);
    DEBUG(
        "Freq2TimeDrift SingleSF: Mantissa = %d (0x%02X, 0x%02X), "
        "Exponent = %d (0x%02X)\n",
        mantissa, (mantissa >> 8) & 0x00FF, (mantissa)&0x00FF, exponent,
        exponent);

    /* Time drift compensation */
    sx1302_reg_write(
        dev,
        SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB,
        1);

    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_DAGC_IN_COMP, 1);

    sx1302_reg_write(dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_MODEM_EN,
                     1);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CADRXTX,
                     1);

    sx1302_reg_write(
        dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_MODEM_START, 1);

    return 0;
}

void sx1302_lora_syncword(sx1302_t *dev, bool public, uint8_t lora_service_sf) {
    /* Multi-SF modem configuration */
    DEBUG_PUTS(
        "INFO: configuring LoRa (Multi-SF) SF5->SF6 with syncword "
        "PRIVATE (0x12)");
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FRAME_SYNCH0_SF5_PEAK1_POS_SF5, 2);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FRAME_SYNCH1_SF5_PEAK2_POS_SF5, 4);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FRAME_SYNCH0_SF6_PEAK1_POS_SF6, 2);
    sx1302_reg_write(dev, SX1302_REG_RX_TOP_FRAME_SYNCH1_SF6_PEAK2_POS_SF6, 4);
    if (public == true) {
        DEBUG_PUTS(
            "INFO: configuring LoRa (Multi-SF) SF7->SF12 with "
            "syncword PUBLIC (0x34)");
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_FRAME_SYNCH0_SF7TO12_PEAK1_POS_SF7TO12, 6);
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_FRAME_SYNCH1_SF7TO12_PEAK2_POS_SF7TO12, 8);
    } else {
        DEBUG_PUTS(
            "INFO: configuring LoRa (Multi-SF) SF7->SF12 with "
            "syncword PRIVATE (0x12)");
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_FRAME_SYNCH0_SF7TO12_PEAK1_POS_SF7TO12, 2);
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_FRAME_SYNCH1_SF7TO12_PEAK2_POS_SF7TO12, 4);
    }

    /* LoRa Service modem configuration */
    if (!public || lora_service_sf == SX1302_DR_LORA_SF5 ||
        lora_service_sf == SX1302_DR_LORA_SF6) {
        DEBUG(
            "INFO: configuring LoRa (Service) SF%u with syncword "
            "PRIVATE (0x12)\n",
            lora_service_sf);
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS, 2);
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS, 4);
    } else {
        DEBUG(
            "INFO: configuring LoRa (Service) SF%u with syncword "
            "PUBLIC (0x34)\n",
            lora_service_sf);
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS, 6);
        sx1302_reg_write(
            dev, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS, 8);
    }
}

void sx1302_modem_enable(sx1302_t *dev) {
    /* Enable LoRa multi-SF modems */
    sx1302_reg_write(dev, SX1302_REG_COMMON_GEN_CONCENTRATOR_MODEM_ENABLE,
                     0x01);

    /* Enable LoRa service modem */
    sx1302_reg_write(dev, SX1302_REG_COMMON_GEN_MBWSSF_MODEM_ENABLE, 0x01);

    /* Enable FSK modem */
    sx1302_reg_write(dev, SX1302_REG_COMMON_GEN_FSK_MODEM_ENABLE, 0x01);

    /* Enable RX */
    sx1302_reg_write(dev, SX1302_REG_COMMON_GEN_GLOBAL_EN, 0x01);
}

void sx1302_tx_configure(sx1302_t *dev) {
    /* Let AGC control PLL DIV (sx1250 only) */
    sx1302_reg_write(dev, SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC,
                     1);
    sx1302_reg_write(dev, SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC,
                     1);

    /* SX126x Tx RFFE */
    sx1302_reg_write(dev, SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01);
    sx1302_reg_write(dev, SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST, 0x01);

    /* Configure the TX mode of operation */
    sx1302_reg_write(dev, SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE,
                     0x01); /* Modulation */
    sx1302_reg_write(dev, SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE,
                     0x01); /* Modulation */

    /* Configure the output data clock edge */
    sx1302_reg_write(dev, SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE,
                     0x00); /* Data on rising edge */
    sx1302_reg_write(dev, SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_CLK_EDGE,
                     0x00); /* Data on rising edge */
}

void sx1302_gps_enable(sx1302_t *dev, bool enable) {
    if (enable) {
        sx1302_reg_write(dev, SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_EN, 1);
        sx1302_reg_write(dev, SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_POL,
                         1); /* invert polarity for PPS */
    } else {
        sx1302_reg_write(dev, SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_EN, 0);
    }
}

void sx1302_init_radio(sx1302_t *dev) {
    sx1302_radio_calibrate(dev, dev->board.clksrc);

    /* Setup radios for RX */
    for (int i = 0; i < 2; i++) {
        if (dev->rf_chain[i].enable) {
            sx1302_radio_reset(dev, i);
            sx1250_setup(dev, i, dev->rf_chain[i].freq_hz,
                         dev->rf_chain[i].single_input_mode);
            sx1302_radio_set_mode(dev, i);
        }
    }

    /* Select the radio which provides the clock to the sx1302 */
    sx1302_radio_clock_select(dev, dev->board.clksrc);

    /* Release host control on radio (will be controlled by AGC) */
    sx1302_radio_host_ctrl(dev, false);

    /* Basic initialization of the sx1302 */
    // sx1302_init(&CONTEXT_TIMESTAMP);

    /* Configure PA/LNA LUTs */
    sx1302_pa_lna_lut_configure(dev);

    /* Configure Radio FE */
    sx1302_radio_fe_configure(dev);

    /* Configure the Channelizer */
    sx1302_channelizer_configure(dev, dev->if_chain, false);

    /* configure LoRa 'multi' demodulators */
    sx1302_lora_correlator_configure(dev);
    sx1302_lora_modem_configure(
        dev, dev->rf_chain[0].freq_hz); /* TODO: freq_hz used to confiogure
                                           freq to time drift, based on RF0
                                           center freq only */

    /* configure LoRa 'stand-alone' modem */
    if (dev->if_chain[8].enable) {
        sx1302_lora_service_correlator_configure(dev, &dev->lora_service);
        sx1302_lora_service_modem_configure(
            dev, &dev->lora_service,
            dev->rf_chain[0].freq_hz); /* TODO: freq_hz used to confiogure
                                          freq to time drift, based on RF0
                                          center freq only */
    }

    /* configure syncword */
    sx1302_lora_syncword(dev, dev->board.lorawan_public,
                         dev->lora_service.datarate);

    /* enable demodulators - to be done before starting AGC/ARB */
    sx1302_modem_enable(dev);

    /* Load firmware */

    DEBUG_PUTS("Loading AGC fw for sx1250");
    if (sx1302_agc_load_firmware(dev, agc_firmware_sx1250) < 0) {
        return;
    }

    if (sx1302_agc_start(dev, 1, SX1302_AGC_RADIO_GAIN_AUTO,
                         SX1302_AGC_RADIO_GAIN_AUTO,
                         dev->board.full_duplex) != 0) {
        return;
    }

    DEBUG_PUTS("Loading ARB fw");
    if (sx1302_arb_load_firmware(dev, arb_firmware) != 0) {
        return;
    }
    if (sx1302_arb_start(dev, SX1302_FW_VERSION_ARB) != 0) {
        return;
    }

    /* static TX configuration */
    sx1302_tx_configure(dev);

    /* enable GPS */
    sx1302_gps_enable(dev, true);
}

int sx1302_send(sx1302_t *dev, bool lwan_public, sx1302_packet_tx_t *pkt_data) {
    uint32_t freq_reg, fdev_reg;
    uint32_t freq_dev;
    uint16_t mem_addr;
    uint32_t count_us;
    uint8_t power;
    uint8_t mod_bw;
    uint8_t pa_en;
    uint16_t tx_start_delay;

    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE(pkt_data->rf_chain),
        0x00);
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC(pkt_data->rf_chain),
        0x01);

    sx1302_tx_gain_t tx_lut = {.rf_power = 14,
                               .dig_gain = 0,
                               .pa_gain  = 2,
                               .dac_gain = 3,
                               .mix_gain = 10,
                               .offset_i = 0,
                               .offset_q = 0,
                               .pwr_idx  = 0};

    /* loading calibrated Tx DC offsets */
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_RFFE_IF_I_OFFSET_I_OFFSET(pkt_data->rf_chain),
        tx_lut.offset_i);
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_RFFE_IF_Q_OFFSET_Q_OFFSET(pkt_data->rf_chain),
        tx_lut.offset_q);

    DEBUG("INFO: Applying IQ offset (i:%d, q:%d)\n", tx_lut.offset_i,
          tx_lut.offset_q);

    pa_en = (tx_lut.pa_gain > 0) ? 1 : 0; /* only 1 bit used to control the
                                             external PA */
    power = (pa_en << 6) | tx_lut.pwr_idx;

    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_AGC_TX_PWR_AGC_TX_PWR(pkt_data->rf_chain),
        power);

    /* Set digital gain */
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_RFFE_IF_IQ_GAIN_IQ_GAIN(pkt_data->rf_chain),
        tx_lut.dig_gain);

    /* Set Tx frequency */
    freq_reg = SX1302_FREQ_TO_REG(
        pkt_data->freq_hz); /* TODO: AGC fw to be updated for sx1255 */
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_H_FREQ_RF(pkt_data->rf_chain),
        (freq_reg >> 16) & 0xFF);
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_M_FREQ_RF(pkt_data->rf_chain),
        (freq_reg >> 8) & 0xFF);
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_L_FREQ_RF(pkt_data->rf_chain),
        (freq_reg >> 0) & 0xFF);

    mod_bw = pkt_data->bandwidth;

    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_AGC_TX_BW_AGC_TX_BW(pkt_data->rf_chain), mod_bw);

    /* Set bandwidth */
    freq_dev = sx1302_get_bandwidth_value(pkt_data->bandwidth) / 2;
    fdev_reg = SX1302_FREQ_TO_REG(freq_dev);
    sx1302_reg_write(
        dev,
        SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV(pkt_data->rf_chain),
        (fdev_reg >> 8) & 0xFF);
    sx1302_reg_write(
        dev,
        SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV(pkt_data->rf_chain),
        (fdev_reg >> 0) & 0xFF);
    sx1302_reg_write(dev,
                     SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_BW(pkt_data->rf_chain),
                     pkt_data->bandwidth);

    /* Preamble length */
    if (pkt_data->preamble ==
        0) { /* if not explicit, use recommended LoRa preamble size
              */
        pkt_data->preamble = SX1302_STD_LORA_PREAMBLE;
    } else if (pkt_data->preamble <
               SX1302_MIN_LORA_PREAMBLE) { /* enforce minimum preamble size
                                            */
        pkt_data->preamble = SX1302_MIN_LORA_PREAMBLE;
        DEBUG_PUTS(
            "Note: preamble length adjusted to respect "
            "minimum LoRa preamble size");
    }
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TXRX_CFG1_3_PREAMBLE_SYMB_NB(pkt_data->rf_chain),
        (pkt_data->preamble >> 8) & 0xFF); /* MSB */
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TXRX_CFG1_2_PREAMBLE_SYMB_NB(pkt_data->rf_chain),
        (pkt_data->preamble >> 0) & 0xFF); /* LSB */

    /* LoRa datarate */
    sx1302_reg_write(dev,
                     SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_SF(pkt_data->rf_chain),
                     pkt_data->datarate);
    if (pkt_data->datarate < 10) {
        sx1302_reg_write(
            dev, SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_LOWPASS(pkt_data->rf_chain),
            6); /* less filtering for low SF : TBC */
    } else {
        sx1302_reg_write(
            dev, SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_LOWPASS(pkt_data->rf_chain),
            7);
    }

    /* Coding Rate */
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TXRX_CFG0_1_CODING_RATE(pkt_data->rf_chain),
        pkt_data->coderate);

    /* Start LoRa modem */
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TXRX_CFG0_2_MODEM_EN(pkt_data->rf_chain), 1);
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TXRX_CFG0_2_CADRXTX(pkt_data->rf_chain), 2);
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TXRX_CFG1_1_MODEM_START(pkt_data->rf_chain), 1);
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_CFG0_0_CONTINUOUS(pkt_data->rf_chain), 0);

    /* Modulation options */
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_INVERT(pkt_data->rf_chain),
        (pkt_data->invert_pol) ? 1 : 0);
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TXRX_CFG0_2_IMPLICIT_HEADER(pkt_data->rf_chain),
        (pkt_data->no_header) ? 1 : 0);
    sx1302_reg_write(dev,
                     SX1302_REG_TX_TOP_TXRX_CFG0_2_CRC_EN(pkt_data->rf_chain),
                     (pkt_data->no_crc) ? 0 : 1);

    /* Syncword */
    if (!lwan_public || (pkt_data->datarate == SX1302_DR_LORA_SF5) ||
        (pkt_data->datarate == SX1302_DR_LORA_SF6)) {
        DEBUG_PUTS("Setting LoRa syncword 0x12");
        sx1302_reg_write(
            dev, SX1302_REG_TX_TOP_FRAME_SYNCH_0_PEAK1_POS(pkt_data->rf_chain),
            2);
        sx1302_reg_write(
            dev, SX1302_REG_TX_TOP_FRAME_SYNCH_1_PEAK2_POS(pkt_data->rf_chain),
            4);
    } else {
        DEBUG_PUTS("Setting LoRa syncword 0x34");
        sx1302_reg_write(
            dev, SX1302_REG_TX_TOP_FRAME_SYNCH_0_PEAK1_POS(pkt_data->rf_chain),
            6);
        sx1302_reg_write(
            dev, SX1302_REG_TX_TOP_FRAME_SYNCH_1_PEAK2_POS(pkt_data->rf_chain),
            8);
    }

    /* Set Fine Sync for SF5/SF6 */
    if ((pkt_data->datarate == SX1302_DR_LORA_SF5) ||
        (pkt_data->datarate == SX1302_DR_LORA_SF6)) {
        DEBUG_PUTS("Enable Fine Sync");
        sx1302_reg_write(
            dev,
            SX1302_REG_TX_TOP_TXRX_CFG0_2_FINE_SYNCH_EN(pkt_data->rf_chain), 1);
    } else {
        DEBUG_PUTS("Disable Fine Sync");
        sx1302_reg_write(
            dev,
            SX1302_REG_TX_TOP_TXRX_CFG0_2_FINE_SYNCH_EN(pkt_data->rf_chain), 0);
    }

    /* Set Payload length */
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TXRX_CFG0_3_PAYLOAD_LENGTH(pkt_data->rf_chain),
        pkt_data->size);

    /* Set PPM offset (low datarate optimization) */
    sx1302_reg_write(
        dev,
        SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL(pkt_data->rf_chain),
        0);
    if (SX1302_SET_PPM_ON(pkt_data->bandwidth, pkt_data->datarate)) {
        DEBUG_PUTS("Low datarate optimization ENABLED");
        sx1302_reg_write(
            dev, SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET(pkt_data->rf_chain),
            1);
    } else {
        DEBUG_PUTS("Low datarate optimization DISABLED");
        sx1302_reg_write(
            dev, SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET(pkt_data->rf_chain),
            0);
    }

    /* Set TX start delay */
    sx1302_tx_set_start_delay(dev, pkt_data->rf_chain, pkt_data->bandwidth,
                              &tx_start_delay);

    /* Write payload in transmit buffer */
    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_CTRL_WRITE_BUFFER(pkt_data->rf_chain), 0x01);
    mem_addr = SX1302_REG_SELECT(pkt_data->rf_chain, 0x5300, 0x5500);

    sx1302_mem_write(dev, mem_addr, &(pkt_data->payload[0]), pkt_data->size);

    sx1302_reg_write(
        dev, SX1302_REG_TX_TOP_TX_CTRL_WRITE_BUFFER(pkt_data->rf_chain), 0x00);

    /* Trigger transmit */
    DEBUG("Start Tx: Freq:%lu SF%u size:%u preamb:%u\n", pkt_data->freq_hz,
          pkt_data->datarate, pkt_data->size, pkt_data->preamble);
    switch (pkt_data->tx_mode) {
        case SX1302_IMMEDIATE:
            sx1302_reg_write(
                dev,
                SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE(pkt_data->rf_chain),
                0x00); /* reset state machine */
            sx1302_reg_write(
                dev,
                SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE(pkt_data->rf_chain),
                0x01);
            break;
        case SX1302_TIMESTAMPED:
            count_us = pkt_data->count_us * 32 - tx_start_delay;
            DEBUG("--> programming trig delay at %lu (%lu)\n",
                  pkt_data->count_us - (tx_start_delay / 32), count_us);

            sx1302_reg_write(
                dev,
                SX1302_REG_TX_TOP_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG(
                    pkt_data->rf_chain),
                (uint8_t)((count_us >> 0) & 0x000000FF));
            sx1302_reg_write(
                dev,
                SX1302_REG_TX_TOP_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG(
                    pkt_data->rf_chain),
                (uint8_t)((count_us >> 8) & 0x000000FF));
            sx1302_reg_write(
                dev,
                SX1302_REG_TX_TOP_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG(
                    pkt_data->rf_chain),
                (uint8_t)((count_us >> 16) & 0x000000FF));
            sx1302_reg_write(
                dev,
                SX1302_REG_TX_TOP_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG(
                    pkt_data->rf_chain),
                (uint8_t)((count_us >> 24) & 0x000000FF));

            sx1302_reg_write(
                dev,
                SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED(pkt_data->rf_chain),
                0x00); /* reset state machine */
            sx1302_reg_write(
                dev,
                SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED(pkt_data->rf_chain),
                0x01);
            break;
        case SX1302_ON_GPS:
            sx1302_reg_write(
                dev, SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS(pkt_data->rf_chain),
                0x00); /* reset state machine */
            sx1302_reg_write(
                dev, SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS(pkt_data->rf_chain),
                0x01);
            break;
        default:
            printf("ERROR: TX mode not supported\n");
            return -1;
    }

    return 0;
}
