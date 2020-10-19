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
 * @brief       Netdev adaptation for the sx1302 driver
 *
 * @author      Pierre Millot
 * @}
 */

#include "sx1302_netdev.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>

#include "net/netdev.h"
#include "net/netopt.h"
#include "sx1302.h"
#include "sx1302_internal.h"
#include "sx1302_registers.h"
#include "sx1302_rx.h"

#define ENABLE_DEBUG SX1302_DEBUG
#include "debug.h"

static int _send(netdev_t *netdev, const iolist_t *iolist) {
    uint8_t size = iolist_size(iolist);
    /* Ignore send if packet size is 0 */
    if (size == 0) {
        return 0;
    }

    sx1302_t *dev = (sx1302_t *)netdev;
    sx1302_packet_tx_t packet;
    packet.freq_hz     = dev->settings.channel;
    packet.tx_mode     = SX1302_IMMEDIATE;
    packet.count_us    = 0;
    packet.rf_chain    = 0;
    packet.rf_power    = dev->settings.lora.power;
    packet.freq_offset = 0;
    packet.bandwidth   = dev->settings.lora.bandwidth;
    packet.datarate    = dev->settings.lora.datarate;
    packet.coderate    = dev->settings.lora.coderate;
    packet.invert_pol  = false;
    packet.preamble    = dev->settings.lora.preamble_len;
    packet.no_crc      = true;
    packet.no_header   = false;

    SX1302_States_t old_state = sx1302_get_state(dev);

    sx1302_set_state(dev, SX1302_RF_TX_RUNNING);

    for (const iolist_t *iol = iolist; iol; iol = iol->iol_next) {
        if (iol->iol_len > 0) {
            packet.size = iol->iol_len;
            memcpy(packet.payload, iol->iol_base, iol->iol_len);

            sx1302_send(dev, dev->board.lorawan_public, &packet);

            // wait for packet to be sent
            SX1302_TX_Status_t tx_status;
            do {
                    xtimer_usleep(5000);
                    tx_status =
                            sx1302_tx_status(dev, packet.rf_chain);
                    printf("[sx1302] tx_status: %d\n",tx_status);

            } while ((tx_status != SX1302_TX_FREE));
        }
    }

    sx1302_set_state(dev, old_state);

    return 0;
}

static int _recv(netdev_t *netdev, void *buf, size_t len, void *info) {
    (void)netdev;
    (void)buf;
    (void)len;
    (void)info;

    DEBUG_PUTS("[sx1302] netdev recv TODO");

    return 0;
}

static int _init(netdev_t *netdev) {
    sx1302_t *sx1302 = (sx1302_t *)netdev;

    /* Launch initialization of driver and device */
    DEBUG_PUTS("[sx1302] netdev: initializing driver...");
    if (sx1302_init(sx1302) != SX1302_INIT_OK) {
        DEBUG_PUTS("[sx1302] netdev: initialization failed");
        return -1;
    }

    sx1302_set_default(sx1302);
    //sx1302_init_radio(sx1302);

    DEBUG_PUTS("[sx1302] netdev: initialization done");

    return 0;
}

static void _isr(netdev_t *netdev) {
    (void)netdev;

    DEBUG_PUTS("[sx1302] netdev send TODO");
}

static int _get(netdev_t *netdev, netopt_t opt, void *val, size_t max_len) {
    (void)max_len; /* unused when compiled without debug, assert empty */
    (void)val;

    sx1302_t *dev = (sx1302_t *)netdev;

    if (dev == NULL) {
        return -ENODEV;
    }

    switch (opt) {
        default:
            DEBUG("[sx1302] netdev get : %s not implemented yet\n",
                  netopt2str(opt));
    }

    return 0;
}

static char rx_stack[THREAD_STACKSIZE_DEFAULT * 2];

void *_rx_thread(void *arg) {
    sx1302_t *dev = (sx1302_t *)arg;

    sx1302_packet_rx_t rxpkt[16];
    int nb_pkt_crc_ok = 0;

    while (sx1302_get_state(dev) == SX1302_RF_RX_RUNNING) {
        int nb_pkt = sx1302_receive(dev, 16, rxpkt);

        if (nb_pkt == 0) {
            xtimer_usleep(100000);
        } else {
            for (int i = 0; i < nb_pkt; i++) {
                if (rxpkt[i].status == SX1302_STAT_CRC_OK) {
                    nb_pkt_crc_ok += 1;
                }
                printf("\n----- LoRa packet -----\n");
                printf("  count_us: %lu\n", rxpkt[i].count_us);
                printf("  size:     %u\n", rxpkt[i].size);
                printf("  chan:     %u\n", rxpkt[i].if_chain);
                printf("  status:   0x%02X\n", rxpkt[i].status);
                printf("  datr:     %u\n", rxpkt[i].datarate);
                printf("  codr:     %u\n", rxpkt[i].coderate);
                printf("  rf_chain  %u\n", rxpkt[i].rf_chain);
                printf("  freq_hz   %lu\n", rxpkt[i].freq_hz);
                printf("  snr_avg:  %.1f\n", rxpkt[i].snr);
                printf("  rssi_chan:%.1f\n", rxpkt[i].rssic);
                printf("  rssi_sig :%.1f\n", rxpkt[i].rssis);
                printf("  crc:      0x%04X\n", rxpkt[i].crc);
                for (int j = 0; j < rxpkt[i].size; j++) {
                    printf("%02X ", rxpkt[i].payload[j]);
                }
                printf("\n");
            }
            printf("Received %d packets (total:%u)\n", nb_pkt, nb_pkt_crc_ok);
        }
    }

    return NULL;
}

static int _set_state(sx1302_t *dev, netopt_state_t state) {
    switch (state) {
        case NETOPT_STATE_RX: {
            sx1302_set_state(dev, SX1302_RF_RX_RUNNING);
            thread_create(rx_stack, sizeof(rx_stack), THREAD_PRIORITY_MAIN - 1,
                          0, _rx_thread, dev, "rx_thread");
            break;
        }
        case NETOPT_STATE_RESET:
            sx1302_reset(dev);
            sx1302_set_default(dev);
            sx1302_init_radio(dev);
            break;

        case NETOPT_STATE_IDLE:
			sx1302_set_state(dev, SX1302_RF_IDLE);
			break;
        default:
            DEBUG("[sx1302] netdev set state : %d not implemented yet\n",
                  state);
            return -ENOTSUP;
    }
    return sizeof(netopt_state_t);
}

static int _set(netdev_t *netdev, netopt_t opt, const void *val, size_t len) {
    (void)len; /* unused when compiled without debug, assert empty */
    (void)val;

    sx1302_t *dev = (sx1302_t *)netdev;
    int res       = -ENOTSUP;

    if (dev == NULL) {
        return -ENODEV;
    }

    switch (opt) {
        case NETOPT_STATE:
            assert(len == sizeof(netopt_state_t));
            return _set_state(dev, *((const netopt_state_t *)val));
        case NETOPT_SINGLE_RECEIVE:
            assert(len <= sizeof(netopt_enable_t));
            dev->settings.single_receive = *(netopt_enable_t *)val;
            return sizeof(netopt_enable_t);
        case NETOPT_RX_TIMEOUT:
            assert(len <= sizeof(uint32_t));
            dev->settings.lora.rx_timeout = *(uint32_t *)val;
            return sizeof(uint32_t);
        default:
            DEBUG("[sx1302] netdev set : %s not implemented yet\n",
                  netopt2str(opt));
    }

    return res;
}

const netdev_driver_t sx1302_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr  = _isr,
    .get  = _get,
    .set  = _set,
};
