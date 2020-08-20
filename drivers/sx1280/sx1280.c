/*
 * Copyright (C) 2016 Unwired Devices <info@unwds.com>
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
 * @ingroup    drivers_SX1280
 *
 * @file	   sx1280.c
 * @brief      Driver for SX1280 devices
 *
 * @author     Julia CORREA REIS
 * @author     Pierre Millot
 */

#include <stdlib.h>
#include <string.h>

#define ENABLE_DEBUG SX1280_DEBUG
#include "debug.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "sx1280.h"
#include "sx1280_internal.h"
#include "sx1280_netdev.h"
#include "sx1280_params.h"
#include "sx1280_registers.h"
#include "thread.h"
#include "xtimer.h"

/* The reset signal must be applied for at least 100 µs to trigger the manual
   reset of the device. To ensure this value is big enough even with an
   inaccurate clock source, an additional 10 % error margin is added. */
#define SX1280_MANUAL_RESET_SIGNAL_LEN_US (110U)

/* After triggering a manual reset the device needs at least 5 ms to become
   ready before interacting with it. To ensure this value is big enough even
   with an inaccurate clock source, an additional 10 % error margin is added. */
#define SX1280_MANUAL_RESET_WAIT_FOR_READY_US (5500U)

/* When the device is started by enabling its power supply for the first time
   i.e. on Power-on-Reset (POR), it needs at least 10 ms after the POR cycle is
   done to become ready. To ensure this value is big enough even with an
   inaccurate clock source, an additional 10 % error margin is added. */
#define SX1280_POR_WAIT_FOR_READY_US (11U * US_PER_MS)

void sx1280_stop_auto_tx(sx1280_t *dev)
{
    uint8_t buf[2] = { 0x00, 0x00 };

    sx1280_write_command_buffer(dev, SX1280_SET_AUTOTX, buf, 2);
}

void sx1280_clear_irq_status(sx1280_t *dev, uint16_t irq)
{
    uint8_t buf[2];

    buf[0] = (irq >> 8) & 0x00FF;
    buf[1] = irq & 0x00FF;
    sx1280_write_command_buffer(dev, SX1280_CLR_IRQSTATUS, buf, 2);
}

void sx1280_send_payload(sx1280_t *dev, uint8_t *payload, uint8_t size,
                         sx1280_tick_time_t timeout)
{
    sx1280_set_payload(dev, payload, size);
    sx1280_set_tx(dev, timeout);
}

/**
 * @brief Get the bandwidth in kHz
 */
static double get_bandwidth(SX1280_LoRaBandwidths_t band)
{
    switch (band) {
        case SX1280_LORA_BW_0200:
            return 200.0;
        case SX1280_LORA_BW_0400:
            return 400.0;
        case SX1280_LORA_BW_0800:
            return 800.0;
        case SX1280_LORA_BW_1600:
            return 1600.0;
    }
    return -1;
}

static double sx1280_n_symbol(uint8_t payload_size)
{
    /* complicated computation, page 48 in doc */
    return payload_size;
}

double sx1280_get_time_on_air(const sx1280_t *dev, uint8_t len)
{
    return pow(2.0, dev->modulation.Params.LoRa.SpreadingFactor >> 4) /
           get_bandwidth(dev->modulation.Params.LoRa.Bandwidth) *
           sx1280_n_symbol(len) * 1000;
}

int32_t sx1280_complement2(const uint32_t num, const uint8_t bitCnt)
{
    int32_t retVal = (int32_t)num;

    if (num >= (uint32_t)(2 << (bitCnt - 2))) {
        retVal -= 2 << (bitCnt - 1);
    }
    return retVal;
}

static int _init_spi(sx1280_t *dev)
{
    int res;

    /* Setup SPI for sx1280 */
    res = spi_init_cs(dev->params.spi, dev->params.nss_pin);

#ifdef MODULE_PERIPH_SPI_GPIO_MODE
    spi_gpio_mode_t gpio_modes = {
        .mosi = (GPIO_OUT | SX1280_DIO_PULL_MODE),
        .miso = (SX1280_DIO_PULL_MODE),
        .sclk = (GPIO_OUT | SX1280_DIO_PULL_MODE),
    };
    res += spi_init_with_gpio_mode(dev->params.spi, gpio_modes);
#endif

    if (res != SPI_OK) {
        DEBUG("[sx1280] error: failed to initialize SPI_%i device (code %i)\n",
              dev->params.spi,
              res);
        return -1;
    }

    DEBUG("[sx1280] SPI_%i initialized with success\n", dev->params.spi);
    return 0;
}

netdev_t *sx1280_setup(sx1280_t *dev, const sx1280_params_t *params)
{
    netdev_t *netdev = (netdev_t *)dev;

    netdev->driver = &sx1280_driver;
    dev->params = *params;

    return netdev;
}

int sx1280_reset(const sx1280_t *dev)
{
    if (dev->params.reset_pin != GPIO_UNDEF) {
        gpio_init(dev->params.reset_pin, GPIO_OUT);

        /* set reset pin to the state that triggers manual reset */
        gpio_write(dev->params.reset_pin, 0);

        xtimer_usleep(SX1280_MANUAL_RESET_SIGNAL_LEN_US);

        /* Put reset pin in High-Z */
        gpio_init(dev->params.reset_pin, GPIO_IN);

        xtimer_usleep(SX1280_MANUAL_RESET_WAIT_FOR_READY_US);
    }

    return 0;
}

uint32_t sx1280_random(sx1280_t *dev)
{
    uint32_t rnd = 0;

    /* Disable LoRa modem interrupts */
    sx1280_clear_irq_status(dev, SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_RX_DONE |
                            SX1280_IRQ_CRC_ERROR | SX1280_IRQ_HEADER_VALID |
                            SX1280_IRQ_TX_DONE | SX1280_IRQ_CAD_DONE);

    /* Set radio in continuous reception */
    sx1280_set_rx(dev, (sx1280_tick_time_t){SX1280_TICK_SIZE_0015_US, 0xFFFF });

    for (unsigned i = 0; i < 32; i++) {
        xtimer_usleep(1000); /* wait for the chaos */

        /* Non-filtered RSSI value reading. Only takes the LSB value */
        rnd |= ((uint32_t)sx1280_get_rssi_inst(dev)) << i;
    }

    sx1280_sleep_params_t sleepConfig = { 0, 0, 1, 1 };
    sx1280_set_sleep(dev, sleepConfig);

    return rnd;
}

/**
 * IRQ handlers
 */
static void sx1280_isr(netdev_t *dev)
{
    netdev_trigger_event_isr(dev);
}

static void sx1280_on_dio_isr(sx1280_t *dev, uint8_t flag)
{
    dev->irq |= flag;
    sx1280_isr((netdev_t *)dev);
}

static void sx1280_on_dio1_isr(void *arg)
{
    sx1280_on_dio_isr((sx1280_t *)arg, SX1280_IRQ_DIO1);
}

static void sx1280_on_dio2_isr(void *arg)
{
    sx1280_on_dio_isr((sx1280_t *)arg, SX1280_IRQ_DIO2);
}

static void sx1280_on_dio3_isr(void *arg)
{
    sx1280_on_dio_isr((sx1280_t *)arg, SX1280_IRQ_DIO3);
}

/* Internal event handlers */
static int _init_gpios(sx1280_t *dev)
{
    int res;

    /* Check if DIO1 pin is defined */
    if (dev->params.dio1_pin != GPIO_UNDEF) {
        res = gpio_init_int(dev->params.dio1_pin, SX1280_DIO_PULL_MODE,
                            GPIO_RISING,
                            sx1280_on_dio1_isr, dev);
        if (res < 0) {
            DEBUG("[sx1280] error: failed to initialize DIO1 pin\n");
            return res;
        }
    }

    /* check if DIO2 pin is defined */
    if (dev->params.dio2_pin != GPIO_UNDEF) {
        res = gpio_init_int(dev->params.dio2_pin, SX1280_DIO_PULL_MODE,
                            GPIO_RISING,
                            sx1280_on_dio2_isr, dev);
        if (res < 0) {
            DEBUG("[sx1280] error: failed to initialize DIO2 pin\n");
            return res;
        }
    }

    /* check if DIO3 pin is defined */
    if (dev->params.dio3_pin != GPIO_UNDEF) {
        res = gpio_init_int(dev->params.dio3_pin, SX1280_DIO_PULL_MODE,
                            GPIO_RISING,
                            sx1280_on_dio3_isr, dev);
        if (res < 0) {
            DEBUG("[sx1280] error: failed to initialize DIO3 pin\n");
            return res;
        }
    }

    return res;
}

void sx1280_init_radio_settings(sx1280_t *dev)
{
    DEBUG("[sx1280] initializing radio settings\n");

    sx1280_set_regulator_mode(dev, SX1280_USE_DCDC);
    sx1280_set_packet_type(dev, SX1280_PACKET_TYPE_LORA);

    sx1280_modulation_params_t modulationParams;
    modulationParams.PacketType = SX1280_PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = SX1280_LORA_SF8;
    modulationParams.Params.LoRa.Bandwidth = SX1280_LORA_BW_0800;
    modulationParams.Params.LoRa.CodingRate = SX1280_LORA_CR_LI_4_5;
    sx1280_set_modulation_params(dev, &modulationParams);

    sx1280_packet_params_t param;
    param.PacketType = SX1280_PACKET_TYPE_LORA;
    param.Params.LoRa.PreambleLength = 12;
    param.Params.LoRa.HeaderType = SX1280_LORA_PACKET_VARIABLE_LENGTH;
    param.Params.LoRa.PayloadLength = 0xFF;
    param.Params.LoRa.CrcMode = SX1280_LORA_CRC_ON;
    param.Params.LoRa.InvertIQ = SX1280_LORA_IQ_NORMAL;
    sx1280_set_packet_params(dev, &param);

    sx1280_set_rf_frequency(dev, SX1280_CHANNEL_0);
    sx1280_set_buffer_base_address(dev, 0x00, 0x00);

    sx1280_set_tx_params(dev, SX1280_RADIO_TX_POWER, SX1280_RAMP_02_US);
    dev->settings.lora.tx_timeout = 500;

    sx1280_set_state(dev, SX1280_RF_IDLE);
}

static void _on_tx_timeout(void *arg)
{
    sx1280_t *dev = (sx1280_t *)arg;

    xtimer_remove(&dev->_internal.tx_timeout_timer);
    sx1280_clear_irq_status(dev, SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT);
    sx1280_set_state(dev, SX1280_RF_IDLE);
    dev->netdev.event_callback(&dev->netdev, NETDEV_EVENT_TX_TIMEOUT);
}

static void _on_rx_timeout(void *arg)
{
    netdev_t *dev = (netdev_t *)arg;

    dev->event_callback(dev, NETDEV_EVENT_RX_TIMEOUT);
}

static void _init_timers(sx1280_t *dev)
{
    dev->_internal.tx_timeout_timer.arg = dev;
    dev->_internal.tx_timeout_timer.callback = _on_tx_timeout;

    dev->_internal.rx_timeout_timer.arg = dev;
    dev->_internal.rx_timeout_timer.callback = _on_rx_timeout;
}

void sx1280_pretty_status(sx1280_t *dev)
{
    static char *mode[] = { "Reserved", "Reserved", "STDBY_RC", "STDBY_XOSC",
                            "FS",       "RX",       "TX",       "Error" };
    static char *cmd[] = { "Reserved",         "Sucess", "Available", "Timeout",
                           "Processing error", "Failed", "TX done",   "Error" };
    SX1280_Status_t status = sx1280_get_status(dev);

    printf("status: busy: %d   dma: %d   cmd status: %s   circuit mode: %s\n",
           status.Fields.CpuBusy, status.Fields.DmaBusy,
           cmd[status.Fields.CmdStatus],
           mode[status.Fields.ChipMode]);
}

int sx1280_init(sx1280_t *dev)
{
    if (_init_spi(dev) < 0) {
        DEBUG("[sx1280] error: failed to initialize SPI\n");
        return -SX1280_ERR_SPI;
    }

    _init_timers(dev);

    if (dev->params.reset_pin != GPIO_UNDEF) {
        /* reset pin should be left floating during POR */
        gpio_init(dev->params.reset_pin, GPIO_IN);
    }

    /* wait for the device to become ready */
    xtimer_usleep(SX1280_POR_WAIT_FOR_READY_US);

    sx1280_reset(dev);

    if (_init_gpios(dev) < 0) {
        DEBUG("[sx1280] error: failed to initialize GPIOs\n");
        return -SX1280_ERR_GPIOS;
    }

    sx1280_wakeup(dev);

    return SX1280_INIT_OK;
}
