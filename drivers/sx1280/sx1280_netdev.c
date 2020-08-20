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
 * @ingroup    drivers_sx1280
 *
 * @file			sx1280_netdev.c
 * @brief      Netdev adaptation for the sx1280 driver
 *
 * @author     Pierre Millot
 */

#include "sx1280_netdev.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "net/netdev/lora.h"
#include "net/netopt.h"
#include "sx1280.h"
#include "sx1280_internal.h"
#include "sx1280_params.h"
#include "sx1280_registers.h"

#define ENABLE_DEBUG SX1280_DEBUG
#include "debug.h"

int _set_state(sx1280_t *dev, netopt_state_t state)
{
    sx1280_tick_time_t timeout;

    timeout.Step = 0x0000;
    timeout.NbSteps = 0x0000;

    if (sx1280_get_operation_mode(dev) == SX1280_MODE_SLEEP &&
        state != NETOPT_STATE_SLEEP) {
        sx1280_wakeup(dev);
        xtimer_usleep(SX1280_WAKEUP_TIME); /* wait for chip wake up */
    }

    switch ((uint8_t)state) {
        case NETOPT_STATE_SLEEP: {
            sx1280_sleep_params_t sleepConfig = { 0, 0, 1, 1 };
            sx1280_set_sleep(dev, sleepConfig);
            break;
        }
        case NETOPT_STATE_STANDBY_RC:
            sx1280_set_standby(dev, SX1280_STDBY_RC);
            break;

        case NETOPT_STATE_STANDBY_XOSC:
            sx1280_set_standby(dev, SX1280_STDBY_XOSC);
            break;

        case NETOPT_STATE_FS:
            sx1280_set_fs(dev);
            break;

        case NETOPT_STATE_CAD:
            sx1280_set_cad(dev);
            break;

        case NETOPT_STATE_RX:
            if (dev->settings.lora.flags.continuous) {
                timeout.NbSteps = 0xFFFF;
            }
            else {
                timeout.NbSteps = dev->settings.lora.rx_timeout;
            }
            sx1280_set_rx(dev, timeout);
            /* bug, have to set it twice */
            xtimer_usleep(1000);
            sx1280_set_rx(dev, timeout);
            break;

        case NETOPT_STATE_TX:
            timeout.NbSteps = dev->settings.lora.tx_timeout;
            sx1280_set_tx(dev, timeout);
            break;

        case NETOPT_STATE_RESET:
            sx1280_reset(dev);
            sx1280_init_radio_settings(dev);
            break;

        default:
            return -ENOTSUP;
    }
    return sizeof(netopt_state_t);
}

int _get_state(sx1280_t *dev, void *val)
{
    uint8_t op_mode = sx1280_get_operation_mode(dev);
    netopt_state_t state = NETOPT_STATE_OFF;

    switch (op_mode) {
        case SX1280_MODE_SLEEP:
            state = NETOPT_STATE_SLEEP;
            break;

        case SX1280_MODE_STDBY_RC:
            state = NETOPT_STATE_STANDBY_RC;
            break;

        case SX1280_MODE_STDBY_XOSC:
            state = NETOPT_STATE_STANDBY_XOSC;
            break;

        case SX1280_MODE_FS:
            state = NETOPT_STATE_FS;
            break;

        case SX1280_MODE_CAD:
            state = NETOPT_STATE_CAD;
            break;

        case SX1280_MODE_TX:
            state = NETOPT_STATE_TX;
            break;

        case SX1280_MODE_RX:
            state = NETOPT_STATE_RX;
            break;

        default:
            break;
    }
    memcpy(val, &state, sizeof(netopt_state_t));
    return sizeof(netopt_state_t);
}

void on_dio1_irq(sx1280_t *dev, uint16_t irq)
{
    netdev_t *netdev = &dev->netdev;

    switch (dev->settings.state) {
        case SX1280_RF_RX_RUNNING:
            switch (dev->settings.packet) {
                case SX1280_PACKET_TYPE_LORA:
                    xtimer_remove(&dev->_internal.rx_timeout_timer);
                    /*  Clear Irq */
                    sx1280_clear_irq_status(dev, SX1280_IRQ_RX_TX_TIMEOUT |
                                            SX1280_IRQ_HEADER_VALID |
                                            SX1280_IRQ_RX_DONE);

                    if (irq & SX1280_IRQ_RX_TX_TIMEOUT) {
                        netdev->event_callback(netdev, NETDEV_EVENT_RX_TIMEOUT);
                    }
                    else {
                        netdev->event_callback(netdev,
                                               NETDEV_EVENT_RX_COMPLETE);
                    }
                    break;
                default:
                    DEBUG("[SX1280] unsupported packet type %d",
                          dev->settings.packet);
                    break;
            }
            break;
        case SX1280_RF_TX_RUNNING:
            xtimer_remove(&dev->_internal.tx_timeout_timer);
            switch (dev->settings.packet) {
                case SX1280_PACKET_TYPE_LORA:
                    /* Clear IRQ */
                    sx1280_clear_irq_status(dev,
                                            SX1280_IRQ_TX_DONE |
                                            SX1280_IRQ_RX_TX_TIMEOUT);
                    sx1280_set_state(dev, SX1280_RF_IDLE);
                    if (irq & SX1280_IRQ_TX_DONE) {
                        netdev->event_callback(netdev,
                                               NETDEV_EVENT_TX_COMPLETE);
                    }
                    else {
                        netdev->event_callback(netdev, NETDEV_EVENT_TX_TIMEOUT);
                    }
                    break;
                default:
                    DEBUG("[SX1280] unsupported packet type %d",
                          dev->settings.packet);
                    break;
            }
            break;
        case SX1280_RF_IDLE:
            DEBUG("[sx1280] netdev: sx1280_on_dio0: IDLE state\n");
            break;
        case SX1280_RF_CAD:
            switch (dev->settings.packet) {
                case SX1280_PACKET_TYPE_LORA:
                    /* Clear IRQ */
                    sx1280_clear_irq_status(dev,
                                            SX1280_IRQ_CAD_DONE |
                                            SX1280_IRQ_CAD_DETECTED);
                    /* Send event message */
                    dev->_internal.is_last_cad_success =
                        ((sx1280_get_irq_status(dev) &
                          SX1280_IRQ_CAD_DETECTED) ==
                         SX1280_IRQ_CAD_DETECTED);
                    netdev->event_callback(netdev, NETDEV_EVENT_CAD_DONE);
                    break;
                default:
                    DEBUG("[sx1280] netdev: sx1280_on_dio0: unknown packet");
                    break;
            }
            break;
        default:
            DEBUG("[sx1280] netdev: sx1280_on_dio0: unknown state [%d]\n",
                  dev->settings.state);
            /* at least the related interrupts should be cleared in this case */
            sx1280_clear_irq_status(
                dev,
                SX1280_IRQ_HEADER_VALID | SX1280_IRQ_CAD_DETECTED |
                SX1280_IRQ_CAD_DONE);
            break;
    }
}

static int _send(netdev_t *netdev, const iolist_t *iolist)
{
    sx1280_t *dev = (sx1280_t *)netdev;

    if (sx1280_get_state(dev) == SX1280_RF_TX_RUNNING) {
        DEBUG(
            "[sx1280] Cannot send packet: radio already in transmitting state.\n");
        return -ENOTSUP;
    }

    uint8_t size = iolist_size(iolist);

    /* Ignore send if packet size is 0 */
    if (size == 0) {
        return 0;
    }

    if (sx1280_get_operation_mode(dev) == SX1280_MODE_SLEEP) {
        sx1280_wakeup(dev);
        /* Device running on XTAL 52MHz, set STDBY_XOSC mode */
        sx1280_set_standby(dev, SX1280_STDBY_XOSC);
        xtimer_usleep(SX1280_WAKEUP_TIME); /* wait for chip wake up */
    }

    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_LORA:
            sx1280_set_buffer_base_address(dev, 0x00, 0x00);

            sx1280_set_dio_irq_params(dev,
                                      SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT,
                                      SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT,
                                      SX1280_IRQ_NONE, SX1280_IRQ_NONE);

            /* Write payload buffer */
            for (const iolist_t *iol = iolist; iol; iol = iol->iol_next) {
                if (iol->iol_len > 0) {
                    /* set the payload size */
                    sx1280_packet_params_t *param = &dev->packet;
                    param->Params.LoRa.PayloadLength = iol->iol_len;
                    sx1280_set_packet_params(dev, param);

                    sx1280_send_payload(dev, iol->iol_base, iol->iol_len,
                                        (sx1280_tick_time_t){SX1280_TICK_SIZE_1000_US,
                                                             dev->settings.lora.tx_timeout });
                }
            }
            break;
        default:
            DEBUG("[sx1280] netdev: Unsupported modem (%d)\n",
                  dev->settings.packet);
            break;
    }

    return 0;
}

static int _recv(netdev_t *netdev, void *buf, size_t len, void *info)
{
    sx1280_t *dev = (sx1280_t *)netdev;
    volatile uint16_t irq_flags = 0;
    uint8_t size = 0;

    sx1280_packet_status_t packetStatus;

    sx1280_get_packet_status(dev, &packetStatus);
    switch (packetStatus.packetType) {
        case SX1280_PACKET_TYPE_LORA:
            /* Clear IRQ */
            sx1280_clear_irq_status(dev, SX1280_IRQ_RX_DONE);
            xtimer_remove(&dev->_internal.rx_timeout_timer);

            netdev_lora_rx_info_t *pktInfo = (netdev_lora_rx_info_t *)info;
            pktInfo->rssi = packetStatus.Params.LoRa.RssiPkt;
            pktInfo->snr = packetStatus.Params.LoRa.SnrPkt;

            irq_flags = sx1280_get_irq_status(dev);
            if ((irq_flags & SX1280_IRQ_CRC_ERROR) == SX1280_IRQ_CRC_ERROR) {
                /* Clear IRQ */
                sx1280_clear_irq_status(dev, SX1280_IRQ_CRC_ERROR);
                xtimer_remove(&dev->_internal.rx_timeout_timer);
                netdev->event_callback(netdev, NETDEV_EVENT_CRC_ERROR);
                return -EBADMSG;
            }

            if (buf == NULL) {
                uint8_t offset;
                sx1280_get_rx_buffer_status(dev, &size, &offset);
                if (len == 0) {
                    return size;
                }
                uint8_t tmp[size];
                sx1280_read_buffer(dev, offset, tmp, size);
                return size;
            }
            if (sx1280_get_payload(dev, buf, &size, len) != 0) {
                return -ENOBUFS;
            }
            else {
                if (dev->settings.lora.flags.continuous) {
                    _set_state(dev, NETOPT_STATE_RX);
                }
                return size;
            }
            break;
        default:
            DEBUG("[SX1280] Received unsupported packet type %d",
                  dev->settings.packet);
            break;
    }

    return size;
}

static int _init(netdev_t *netdev)
{
    sx1280_t *sx1280 = (sx1280_t *)netdev;

    sx1280->irq = 0;
    sx1280_settings_t settings;
    settings.packet = SX1280_PACKET_TYPE_LORA;
    settings.state = SX1280_RF_IDLE;

    sx1280->settings = settings;

    /* Launch initialization of driver and device */
    DEBUG("[sx1280] netdev: initializing driver...\n");
    if (sx1280_init(sx1280) != SX1280_INIT_OK) {
        DEBUG("[sx1280] netdev: initialization failed\n");
        return -1;
    }

    sx1280_set_standby(sx1280, SX1280_STDBY_RC);
    sx1280_init_radio_settings(sx1280);

    uint32_t start = xtimer_now_usec();
    SX1280_Status_t status = sx1280_get_status(sx1280);

    while (status.Fields.ChipMode == 7 &&
           xtimer_now_usec() - start < 1000000UL) {
        sx1280_reset(sx1280);
        sx1280_init_radio_settings(sx1280);

        xtimer_usleep(100000);
        status = sx1280_get_status(sx1280);

        DEBUG_PUTS("[sx1280] Initialization failed. Trying again...")
    }

    if(status.Fields.ChipMode == 7) {
        return -1;
    }

    sx1280_sleep_params_t sleepConfig = { 0, 0, 1, 1 };
    sx1280_set_sleep(sx1280, sleepConfig);

    DEBUG("[sx1280] netdev: initialization done\n");

    return 0;
}

static void _isr(netdev_t *netdev)
{
    sx1280_t *dev = (sx1280_t *)netdev;

    uint16_t interruptReg = sx1280_get_irq_status(dev);

    on_dio1_irq(dev, interruptReg);
}

static int _get(netdev_t *netdev, netopt_t opt, void *val, size_t max_len)
{
    (void)max_len; /* unused when compiled without debug, assert empty */
    sx1280_t *dev = (sx1280_t *)netdev;

    if (dev == NULL) {
        return -ENODEV;
    }

    switch (opt) {
        case NETOPT_STATE:
            assert(max_len >= sizeof(netopt_state_t));
            return _get_state(dev, val);

        case NETOPT_DEVICE_TYPE:
            assert(max_len >= sizeof(uint16_t));
            *((uint16_t *)val) = NETDEV_TYPE_LORA;
            return sizeof(uint16_t);

        case NETOPT_PREAMBLE_LENGTH:
            assert(max_len >= sizeof(uint8_t));
            *((uint8_t *)val) = dev->packet.Params.LoRa.PreambleLength;
            return sizeof(uint8_t);

        case NETOPT_BANDWIDTH:
            assert(max_len >= sizeof(uint8_t));
            *((uint8_t *)val) = sx1280_get_bandwidth(dev);
            return sizeof(uint8_t);

        case NETOPT_CHANNEL_FREQUENCY:
            assert(max_len >= sizeof(uint32_t));
            *((uint32_t *)val) = sx1280_get_rf_frequency(dev);
            return sizeof(uint32_t);

        case NETOPT_SPREADING_FACTOR:
            assert(max_len >= sizeof(uint8_t));
            *((uint8_t *)val) = sx1280_get_spreading_factor(dev);
            return sizeof(uint8_t);

        case NETOPT_CODING_RATE:
            assert(max_len >= sizeof(uint8_t));
            *((uint8_t *)val) = sx1280_get_coding_rate(dev);
            return sizeof(uint8_t);

        case NETOPT_RANDOM:
            assert(max_len >= sizeof(uint32_t));
            *((uint32_t *)val) = sx1280_random(dev);
            return sizeof(uint32_t);

        case NETOPT_INTEGRITY_CHECK:
            assert(max_len >= sizeof(netopt_enable_t));
            *((netopt_enable_t *)val) =
                sx1280_get_crc(dev) ? NETOPT_ENABLE : NETOPT_DISABLE;
            return sizeof(netopt_enable_t);

        case NETOPT_SINGLE_RECEIVE:
            assert(max_len >= sizeof(uint8_t));
            *((netopt_enable_t *)val) =
                sx1280_get_rx_single(dev) ? NETOPT_ENABLE : NETOPT_DISABLE;
            return sizeof(netopt_enable_t);

        case NETOPT_IQ_INVERT:
            assert(max_len >= sizeof(SX1280_LoRaIQModes_t));
            *((SX1280_LoRaIQModes_t *)val) = sx1280_get_iq_invert(dev);
            return sizeof(SX1280_LoRaIQModes_t);

        case NETOPT_TX_POWER:
            assert(max_len >= sizeof(int16_t));
            *((int16_t *)val) = (int16_t)sx1280_get_tx_power(dev);
            return sizeof(int16_t);
        default:
            break;
    }

    return -ENOTSUP;
}

static int _set(netdev_t *netdev, netopt_t opt, const void *val, size_t len)
{
    (void)len; /* unused when compiled without debug, assert empty */

    sx1280_t *dev = (sx1280_t *)netdev;
    int res = -ENOTSUP;

    if (dev == NULL) {
        return -ENODEV;
    }
    switch (opt) {
        case NETOPT_STATE:
            assert(len == sizeof(netopt_state_t));
            return _set_state(dev, *((const netopt_state_t *)val));

        case NETOPT_DEVICE_TYPE:
            assert(len <= sizeof(uint16_t));
            if (*(const uint16_t *)val == NETDEV_TYPE_LORA) {
                sx1280_set_packet_type(dev, SX1280_PACKET_TYPE_LORA);
                return sizeof(uint16_t);
            }
            else {
                return -EINVAL;
            }

        case NETOPT_CHANNEL_FREQUENCY: {
            uint32_t freq = *((const uint32_t *)val);
            sx1280_set_rf_frequency(dev, freq);
            return sizeof(uint32_t);
        }

        case NETOPT_PREAMBLE_LENGTH:
            assert(len <= sizeof(uint8_t));
            uint8_t preamble_len = *((const uint8_t *)val);
            sx1280_packet_params_t *packet = &dev->packet;
            packet->Params.LoRa.PreambleLength = preamble_len;
            sx1280_set_packet_params(dev, packet);
            return sizeof(uint8_t);

        case NETOPT_IQ_INVERT:
            assert(len <= sizeof(uint8_t));
            packet = &dev->packet;
            packet->Params.LoRa.InvertIQ = *((const uint8_t *)val);
            sx1280_set_packet_params(dev, packet);
            return sizeof(uint8_t);

        case NETOPT_BANDWIDTH:
            assert(len <= sizeof(uint8_t));
            uint8_t bw = *((const uint8_t *)val);
            if (bw > SX1280_LORA_BW_1600) {
                res = -EINVAL;
                break;
            }
            sx1280_modulation_params_t *modulationParams = &dev->modulation;
            modulationParams->Params.LoRa.Bandwidth = bw;
            sx1280_set_modulation_params(dev, modulationParams);
            return sizeof(uint8_t);

        case NETOPT_SPREADING_FACTOR:
            assert(len <= sizeof(uint8_t));
            uint8_t sf = *((const uint8_t *)val);
            if ((sf < SX1280_LORA_SF6) || (sf > SX1280_LORA_SF12)) {
                res = -EINVAL;
                break;
            }
            modulationParams = &dev->modulation;
            modulationParams->Params.LoRa.SpreadingFactor = sf;
            sx1280_set_modulation_params(dev, modulationParams);
            return sizeof(uint8_t);

        case NETOPT_CODING_RATE:
            assert(len <= sizeof(uint8_t));
            uint8_t cr = *((const uint8_t *)val);
            if ((cr < SX1280_LORA_CR_4_5) || (cr > SX1280_LORA_CR_4_8)) {
                res = -EINVAL;
                break;
            }
            modulationParams = &dev->modulation;
            modulationParams->Params.LoRa.CodingRate = cr;
            sx1280_set_modulation_params(dev, modulationParams);
            return sizeof(uint8_t);

        case NETOPT_INTEGRITY_CHECK:
            assert(len <= sizeof(netopt_enable_t));
            sx1280_packet_params_t *packetParams = &dev->packet;
            packetParams->Params.LoRa.CrcMode =
                *((const netopt_enable_t *)val) ? true : false;
            sx1280_set_packet_params(dev, packetParams);
            return sizeof(netopt_enable_t);

        case NETOPT_RX_TIMEOUT:
            assert(len <= sizeof(uint32_t));
            dev->settings.lora.rx_timeout = *(uint32_t *)val;
            return sizeof(uint32_t);

        case NETOPT_SINGLE_RECEIVE:
            assert(len <= sizeof(uint8_t));
            sx1280_set_rx_single_mode(dev, *(bool *)val);
            return sizeof(uint8_t);

        case NETOPT_TX_TIMEOUT:
            assert(len <= sizeof(uint32_t));
            sx1280_set_tx(dev, *((const sx1280_tick_time_t *)val));
            return sizeof(uint32_t);

        case NETOPT_TX_POWER:
            assert(len <= sizeof(int16_t));
            int16_t power = *((const int16_t *)val);
            if ((power < INT8_MIN) || (power > INT8_MAX)) {
                res = -EINVAL;
                break;
            }
            sx1280_set_tx_params(dev, (int8_t)power, SX1280_RAMP_02_US);
            return sizeof(int16_t);

        case NETOPT_SYNCWORD:
            assert(len <= sizeof(uint8_t));
            sx1280_set_syncword(dev, *(uint8_t *)len, (uint8_t *)val);
            return sizeof(uint8_t);

        default:
            break;
    }
    return res;
}

const netdev_driver_t sx1280_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr = _isr,
    .get = _get,
    .set = _set,
};
