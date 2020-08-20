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
 * @brief       SX1302 receiving code
 *
 * @author      Pierre Millot
 * @}
 */

#include "sx1302_rx.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "sx1302_internal.h"
#include "sx1302_registers.h"
#include "thread.h"
#include "xtimer.h"

#define ENABLE_DEBUG SX1302_DEBUG
#include "debug.h"

int sx1302_update(sx1302_t *dev) {
    /* Check MCUs parity errors */
    int32_t val = sx1302_reg_read(dev, SX1302_REG_AGC_MCU_CTRL_PARITY_ERROR);
    if (val != 0) {
        printf("ERROR: Parity error check failed on AGC firmware\n");
        return -1;
    }
    val = sx1302_reg_read(dev, SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR);
    if (val != 0) {
        printf("ERROR: Parity error check failed on ARB firmware\n");
        return -1;
    }

    /* Update internal timestamp counter wrapping status */
    // timestamp_counter_get(&counter_us, false); /* maintain inst counter
    // timestamp_counter_get(&counter_us, true); /* maintain pps counter
    //

    return 0;
}

void sx1302_init_rx_buffer(sx1302_t *dev) {
    /* Initialize members */
    memset(dev->rx_buffer.buffer, 0, sizeof(dev->rx_buffer.buffer));
    dev->rx_buffer.buffer_size   = 0;
    dev->rx_buffer.buffer_index  = 0;
    dev->rx_buffer.buffer_pkt_nb = 0;
}

int sx1302_rx_buffer_fetch(sx1302_t *dev) {
    int i;
    uint8_t buff[2];
    int32_t msb;

    /* Check if there is data in the FIFO */
    sx1302_reg_read_batch(
        dev, SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES, buff,
        sizeof buff);
    /* Workaround concentrator chip issue:
        - read MSB again
        - if MSB changed, read the full size gain
     */
    msb = sx1302_reg_read(
        dev, SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES);
    if (buff[0] != (uint8_t)msb) {
        sx1302_reg_read_batch(
            dev, SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES,
            buff, sizeof buff);
    }

    dev->rx_buffer.buffer_size = (buff[0] << 8) | (buff[1] << 0);

    /* Fetch bytes from fifo if any */
    if (dev->rx_buffer.buffer_size > 0) {
        DEBUG_PUTS("-----------------");
        DEBUG("%s: nb_bytes to be fetched: %u (%u %u)\n", __FUNCTION__,
              dev->rx_buffer.buffer_size, buff[1], buff[0]);

        memset(dev->rx_buffer.buffer, 0, sizeof dev->rx_buffer.buffer);
        sx1302_mem_read(dev, 0x4000, dev->rx_buffer.buffer,
                        dev->rx_buffer.buffer_size, true);

        /* print debug info : TODO to be removed */
        DEBUG("RX_BUFFER: ");
        for (i = 0; i < dev->rx_buffer.buffer_size; i++) {
            DEBUG("%02X ", dev->rx_buffer.buffer[i]);
        }
        DEBUG_PUTS("");
    }

    /* Parse buffer to get number of packet fetched */
    uint8_t payload_len;
    uint16_t next_pkt_idx;
    int idx = 0;
    while (idx < dev->rx_buffer.buffer_size) {
        if ((dev->rx_buffer.buffer[idx] != SX1302_PKT_SYNCWORD_BYTE_0) ||
            (dev->rx_buffer.buffer[idx + 1] != SX1302_PKT_SYNCWORD_BYTE_1)) {
            printf("ERROR: syncword not found in rx_buffer\n");
            return -1;
        }
        /* One packet found in the buffer */
        dev->rx_buffer.buffer_pkt_nb += 1;

        /* Compute the number of bytes for thsi packet */
        payload_len  = SX1302_PKT_PAYLOAD_LENGTH(dev->rx_buffer.buffer, idx);
        next_pkt_idx = SX1302_PKT_HEAD_METADATA + payload_len +
                       SX1302_PKT_TAIL_METADATA +
                       (2 * SX1302_PKT_NUM_TS_METRICS(dev->rx_buffer.buffer,
                                                      idx + payload_len));

        /* Move to next packet */
        idx += (int)next_pkt_idx;
    }

    /* Initialize the current buffer index to iterate on */
    dev->rx_buffer.buffer_index = 0;

    return 0;
}

int sx1302_fetch(sx1302_t *dev, uint8_t *nb_pkt) {
    int err;

    /* Fetch packets from sx1302 if no more left in RX buffer */
    if (dev->rx_buffer.buffer_pkt_nb == 0) {
        /* Initialize RX buffer */
        sx1302_init_rx_buffer(dev);

        /* Fetch RX buffer if any data available */
        err = sx1302_rx_buffer_fetch(dev);
        if (err != 0) {
            printf("ERROR: Failed to fetch RX buffer\n");
            return -1;
        }
    } else {
        printf(
            "Note: remaining %u packets in RX buffer, do not fetch "
            "sx1302 yet...\n",
            dev->rx_buffer.buffer_pkt_nb);
    }

    /* Return the number of packet fetched */
    *nb_pkt = dev->rx_buffer.buffer_pkt_nb;

    return 0;
}

int sx1302_rx_buffer_pop(sx1302_t *dev, sx1302_rx_packet_t *pkt) {
    int i;
    uint8_t checksum_rcv, checksum_calc = 0;
    uint16_t checksum_idx;
    uint16_t pkt_num_bytes;

    /* Is there any data to be parsed ? */
    if (dev->rx_buffer.buffer_index >= dev->rx_buffer.buffer_size) {
        DEBUG_PUTS("INFO: No more data to be parsed");
        return -1;
    }

    /* Get pkt sync words */
    if ((dev->rx_buffer.buffer[dev->rx_buffer.buffer_index] !=
         SX1302_PKT_SYNCWORD_BYTE_0) ||
        (dev->rx_buffer.buffer[dev->rx_buffer.buffer_index + 1] !=
         SX1302_PKT_SYNCWORD_BYTE_1)) {
        printf("INFO: searching syncword...\n");
        dev->rx_buffer.buffer_index += 1;
        return -1;
        /* TODO: while loop until syncword found ?? */
    }
    DEBUG("INFO: pkt syncword found at index %u\n",
          dev->rx_buffer.buffer_index);

    /* Get payload length */
    pkt->rxbytenb_modem = SX1302_PKT_PAYLOAD_LENGTH(
        dev->rx_buffer.buffer, dev->rx_buffer.buffer_index);

    /* Get fine timestamp metrics */
    pkt->num_ts_metrics_stored = SX1302_PKT_NUM_TS_METRICS(
        dev->rx_buffer.buffer,
        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem);

    /* Calculate the total number of bytes in the packet */
    pkt_num_bytes = SX1302_PKT_HEAD_METADATA + pkt->rxbytenb_modem +
                    SX1302_PKT_TAIL_METADATA + (2 * pkt->num_ts_metrics_stored);

    /* Check if we have a complete packet in the rx buffer fetched */
    if ((dev->rx_buffer.buffer_index + pkt_num_bytes) >
        dev->rx_buffer.buffer_size) {
        printf("WARNING: aborting truncated message (size=%u)\n",
               dev->rx_buffer.buffer_size);
        return -1;
    }

    /* Get the checksum as received in the RX buffer */
    checksum_idx = pkt_num_bytes - 1;
    checksum_rcv =
        dev->rx_buffer.buffer[dev->rx_buffer.buffer_index + pkt_num_bytes - 1];

    /* Calculate the checksum from the actual payload bytes received */
    for (i = 0; i < (int)checksum_idx; i++) {
        checksum_calc += dev->rx_buffer.buffer[dev->rx_buffer.buffer_index + i];
    }

    /* Check if the checksum is correct */
    if (checksum_rcv != checksum_calc) {
        printf("WARNING: checksum failed (got:0x%02X calc:0x%02X)\n",
               checksum_rcv, checksum_calc);
        return -1;
    } else {
        DEBUG("Packet checksum OK (0x%02X)\n", checksum_rcv);
    }

    /* Parse packet metadata */
    pkt->modem_id =
        SX1302_PKT_MODEM_ID(dev->rx_buffer.buffer, dev->rx_buffer.buffer_index);
    pkt->rx_channel_in =
        SX1302_PKT_CHANNEL(dev->rx_buffer.buffer, dev->rx_buffer.buffer_index);
    pkt->crc_en =
        SX1302_PKT_CRC_EN(dev->rx_buffer.buffer, dev->rx_buffer.buffer_index);
    pkt->payload_crc_error =
        SX1302_PKT_CRC_ERROR(dev->rx_buffer.buffer,
                             dev->rx_buffer.buffer_index + pkt->rxbytenb_modem);
    pkt->sync_error = SX1302_PKT_SYNC_ERROR(
        dev->rx_buffer.buffer,
        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem);
    pkt->header_error = SX1302_PKT_HEADER_ERROR(
        dev->rx_buffer.buffer,
        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem);
    pkt->timing_set = SX1302_PKT_TIMING_SET(
        dev->rx_buffer.buffer,
        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem);
    pkt->coding_rate = SX1302_PKT_CODING_RATE(dev->rx_buffer.buffer,
                                              dev->rx_buffer.buffer_index);
    pkt->rx_rate_sf =
        SX1302_PKT_DATARATE(dev->rx_buffer.buffer, dev->rx_buffer.buffer_index);
    pkt->rssi_chan_avg =
        SX1302_PKT_RSSI_CHAN(dev->rx_buffer.buffer,
                             dev->rx_buffer.buffer_index + pkt->rxbytenb_modem);
    pkt->rssi_signal_avg =
        SX1302_PKT_RSSI_SIG(dev->rx_buffer.buffer,
                            dev->rx_buffer.buffer_index + pkt->rxbytenb_modem);
    pkt->rx_crc16_value =
        (uint16_t)((SX1302_PKT_CRC_PAYLOAD_7_0(
                        dev->rx_buffer.buffer,
                        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem)
                    << 0) &
                   0x00FF);
    pkt->rx_crc16_value |=
        (uint16_t)((SX1302_PKT_CRC_PAYLOAD_15_8(
                        dev->rx_buffer.buffer,
                        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem)
                    << 8) &
                   0xFF00);
    pkt->snr_average = (int8_t)SX1302_PKT_SNR_AVG(
        dev->rx_buffer.buffer,
        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem);

    pkt->frequency_offset_error =
        (int32_t)((SX1302_PKT_FREQ_OFFSET_19_16(dev->rx_buffer.buffer,
                                                dev->rx_buffer.buffer_index)
                   << 16) |
                  (SX1302_PKT_FREQ_OFFSET_15_8(dev->rx_buffer.buffer,
                                               dev->rx_buffer.buffer_index)
                   << 8) |
                  (SX1302_PKT_FREQ_OFFSET_7_0(dev->rx_buffer.buffer,
                                              dev->rx_buffer.buffer_index)
                   << 0));
    if (pkt->frequency_offset_error >=
        (1 << 19)) { /* Handle signed value on 20bits */
        pkt->frequency_offset_error = (pkt->frequency_offset_error - (1 << 20));
    }

    /* Packet timestamp (32MHz ) */
    pkt->timestamp_cnt =
        (uint32_t)((SX1302_PKT_TIMESTAMP_7_0(
                        dev->rx_buffer.buffer,
                        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem)
                    << 0) &
                   0x000000FF);
    pkt->timestamp_cnt |=
        (uint32_t)((SX1302_PKT_TIMESTAMP_15_8(
                        dev->rx_buffer.buffer,
                        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem)
                    << 8) &
                   0x0000FF00);
    pkt->timestamp_cnt |=
        (uint32_t)((SX1302_PKT_TIMESTAMP_23_16(
                        dev->rx_buffer.buffer,
                        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem)
                    << 16) &
                   0x00FF0000);
    pkt->timestamp_cnt |=
        (uint32_t)((SX1302_PKT_TIMESTAMP_31_24(
                        dev->rx_buffer.buffer,
                        dev->rx_buffer.buffer_index + pkt->rxbytenb_modem)
                    << 24) &
                   0xFF000000);

    DEBUG_PUTS("-----------------");
    DEBUG("  modem:      %u\n", pkt->modem_id);
    DEBUG("  chan:       %u\n", pkt->rx_channel_in);
    DEBUG("  size:       %u\n", pkt->rxbytenb_modem);
    DEBUG("  crc_en:     %u\n", pkt->crc_en);
    DEBUG("  crc_err:    %u\n", pkt->payload_crc_error);
    DEBUG("  sync_err:   %u\n", pkt->sync_error);
    DEBUG("  hdr_err:    %u\n", pkt->header_error);
    DEBUG("  timing_set: %u\n", pkt->timing_set);
    DEBUG("  codr:       %u\n", pkt->coding_rate);
    DEBUG("  datr:       %u\n", pkt->rx_rate_sf);
    DEBUG("  num_ts:     %u\n", pkt->num_ts_metrics_stored);
    DEBUG_PUTS("-----------------");

    /* Sanity checks: check the range of few metadata */
    if (pkt->modem_id > SX1302_FSK_MODEM_ID) {
        printf("ERROR: modem_id is out of range - %u\n", pkt->modem_id);
        return -1;
    } else {
        if (pkt->modem_id <= SX1302_LORA_STD_MODEM_ID) { /* LoRa modems */
            if (pkt->rx_channel_in > 9) {
                printf("ERROR: channel is out of range - %u\n",
                       pkt->rx_channel_in);
                return -1;
            }
            if ((pkt->rx_rate_sf < 5) || (pkt->rx_rate_sf > 12)) {
                printf("ERROR: SF is out of range - %u\n", pkt->rx_rate_sf);
                return -1;
            }
        }
    }

    /* Parse & copy payload in packet struct */
    memcpy((void *)pkt->payload,
           (void *)(&(dev->rx_buffer.buffer[dev->rx_buffer.buffer_index +
                                            SX1302_PKT_HEAD_METADATA])),
           pkt->rxbytenb_modem);

    /* Move buffer index toward next message */
    dev->rx_buffer.buffer_index +=
        (SX1302_PKT_HEAD_METADATA + pkt->rxbytenb_modem +
         SX1302_PKT_TAIL_METADATA + (2 * pkt->num_ts_metrics_stored));

    /* Update the number of packets currently stored in the rx_buffer */
    dev->rx_buffer.buffer_pkt_nb -= 1;

    return 0;
}

void sx1302_lora_crc16(const char data, int *crc) {
    int next = 0;
    next     = (((data >> 0) & 1) ^ ((*crc >> 12) & 1) ^ ((*crc >> 8) & 1));
    next += ((((data >> 1) & 1) ^ ((*crc >> 13) & 1) ^ ((*crc >> 9) & 1)) << 1);
    next +=
        ((((data >> 2) & 1) ^ ((*crc >> 14) & 1) ^ ((*crc >> 10) & 1)) << 2);
    next +=
        ((((data >> 3) & 1) ^ ((*crc >> 15) & 1) ^ ((*crc >> 11) & 1)) << 3);
    next += ((((data >> 4) & 1) ^ ((*crc >> 12) & 1)) << 4);
    next += ((((data >> 5) & 1) ^ ((*crc >> 13) & 1) ^ ((*crc >> 12) & 1) ^
              ((*crc >> 8) & 1))
             << 5);
    next += ((((data >> 6) & 1) ^ ((*crc >> 14) & 1) ^ ((*crc >> 13) & 1) ^
              ((*crc >> 9) & 1))
             << 6);
    next += ((((data >> 7) & 1) ^ ((*crc >> 15) & 1) ^ ((*crc >> 14) & 1) ^
              ((*crc >> 10) & 1))
             << 7);
    next +=
        ((((*crc >> 0) & 1) ^ ((*crc >> 15) & 1) ^ ((*crc >> 11) & 1)) << 8);
    next += ((((*crc >> 1) & 1) ^ ((*crc >> 12) & 1)) << 9);
    next += ((((*crc >> 2) & 1) ^ ((*crc >> 13) & 1)) << 10);
    next += ((((*crc >> 3) & 1) ^ ((*crc >> 14) & 1)) << 11);
    next += ((((*crc >> 4) & 1) ^ ((*crc >> 15) & 1) ^ ((*crc >> 12) & 1) ^
              ((*crc >> 8) & 1))
             << 12);
    next +=
        ((((*crc >> 5) & 1) ^ ((*crc >> 13) & 1) ^ ((*crc >> 9) & 1)) << 13);
    next +=
        ((((*crc >> 6) & 1) ^ ((*crc >> 14) & 1) ^ ((*crc >> 10) & 1)) << 14);
    next +=
        ((((*crc >> 7) & 1) ^ ((*crc >> 15) & 1) ^ ((*crc >> 11) & 1)) << 15);
    (*crc) = next;
}

uint16_t sx1302_lora_payload_crc(const uint8_t *data, uint8_t size) {
    int crc = 0;

    for (int i = 0; i < size; i++) {
        sx1302_lora_crc16(data[i], &crc);
    }

    // printf("CRC16: 0x%02X 0x%02X (%X)\n", (uint8_t)(crc >> 8), (uint8_t)crc,
    // crc);
    return (uint16_t)crc;
}

const uint8_t ifmod_config[] = {SX1302_IF_LORA_MULTI, SX1302_IF_LORA_MULTI,
                                SX1302_IF_LORA_MULTI, SX1302_IF_LORA_MULTI,
                                SX1302_IF_LORA_MULTI, SX1302_IF_LORA_MULTI,
                                SX1302_IF_LORA_MULTI, SX1302_IF_LORA_MULTI,
                                SX1302_IF_LORA_STD,   SX1302_IF_FSK_STD};

int sx1302_parse(sx1302_t *dev, sx1302_packet_rx_t *p) {
    int err;
    uint16_t payload_crc16_calc;
    uint8_t cr;
    uint32_t timestamp_correction; /* correction to account for processing
                                      delay */
    sx1302_rx_packet_t pkt;

    /* get packet from RX buffer */
    err = sx1302_rx_buffer_pop(dev, &pkt);
    if (err != 0) {
        return -1;
    }

    /* copy payload to result struct */
    memcpy((void *)p->payload, (void *)(&(pkt.payload)), pkt.rxbytenb_modem);
    p->size = pkt.rxbytenb_modem;

    /* process metadata */
    p->modem_id = pkt.modem_id;
    p->if_chain = pkt.rx_channel_in;
    if (p->if_chain >= 10) {
        DEBUG("WARNING: %u NOT A VALID IF_CHAIN NUMBER, ABORTING\n",
              p->if_chain);
        return -1;
    }
    /* type of if_chain/modem a packet was received by */
    int ifmod = ifmod_config[p->if_chain];
    DEBUG("[%d 0x%02X]\n", p->if_chain, ifmod);

    p->rf_chain = dev->if_chain[p->if_chain].rf_chain;

    /* Get the frequency for the channel configuration */
    p->freq_hz = (uint32_t)((int32_t)dev->rf_chain[p->rf_chain].freq_hz +
                            dev->if_chain[p->if_chain].freq_hz);

    /* Get signal strength : offset and temperature compensation will be
     * applied later */
    p->rssic = (float)(pkt.rssi_chan_avg);
    p->rssis = (float)(pkt.rssi_signal_avg);

    /* Get modulation metadata */
    if ((ifmod == SX1302_IF_LORA_MULTI) || (ifmod == SX1302_IF_LORA_STD)) {
        DEBUG("Note: LoRa packet (modem %u chan %u)\n", p->modem_id,
              p->if_chain);
        /* Get CRC status */
        if (pkt.crc_en || dev->lora_service.implicit_crc_en) {
            /* CRC enabled */
            if (pkt.payload_crc_error) {
                p->status = SX1302_STAT_CRC_BAD;
            } else {
                p->status = SX1302_STAT_CRC_OK;

                /* Sanity check of the payload CRC */
                if (p->size > 0) {
                    payload_crc16_calc =
                        sx1302_lora_payload_crc(p->payload, p->size);
                    if (payload_crc16_calc != pkt.rx_crc16_value) {
                        printf(
                            "ERROR: Payload CRC16 "
                            "check failed "
                            "(got:0x%04X "
                            "calc:0x%04X)\n",
                            pkt.rx_crc16_value, payload_crc16_calc);
                    }
                    return -1;
                } else {
                    DEBUG(
                        "Payload CRC check OK "
                        "(0x%04X)\n",
                        pkt.rx_crc16_value);
                }
            }
        } else {
            /* CRC disabled */
            p->status = SX1302_STAT_NO_CRC;
        }

        /* Get SNR - converted from 0.25dB step to dB */
        p->snr = (float)(pkt.snr_average) / 4;

        /* Get bandwidth */
        if (ifmod == SX1302_IF_LORA_MULTI) {
            p->bandwidth = SX1302_LORA_BW_125_KHZ; /* fixed in hardware */
        } else {
            /* get the parameter from the config variable */
            p->bandwidth = dev->lora_service.bandwidth;
        }

        /* Get datarate */
        p->datarate = pkt.rx_rate_sf;

        /* Get coding rate */
        if ((ifmod == SX1302_IF_LORA_MULTI) ||
            !dev->lora_service.implicit_hdr) {
            cr = pkt.coding_rate;
        } else {
            cr = dev->lora_service.implicit_coderate;
        }
        p->coderate = cr;

        /* Get frequency offset in Hz depending on bandwidth */
        switch (p->bandwidth) {
            case SX1302_LORA_BW_125_KHZ:
                p->freq_offset = (int32_t)((float)(pkt.frequency_offset_error) *
                                           SX1302_FREQ_OFFSET_LSB_125KHZ);
                break;
            case SX1302_LORA_BW_250_KHZ:
                p->freq_offset = (int32_t)((float)(pkt.frequency_offset_error) *
                                           SX1302_FREQ_OFFSET_LSB_250KHZ);
                break;
            case SX1302_LORA_BW_500_KHZ:
                p->freq_offset = (int32_t)((float)(pkt.frequency_offset_error) *
                                           SX1302_FREQ_OFFSET_LSB_500KHZ);
                break;
            default:
                p->freq_offset = 0;
                printf("Invalid frequency offset\n");
                break;
        }

        /* Get timestamp correction to be applied */
        // timestamp_correction = timestamp_counter_correction(ifmod,
        // p->bandwidth, p->datarate, p->coderate, pkt.crc_en,
        // pkt.rxbytenb_modem);
    } else {
        DEBUG_PUTS("ERROR: UNEXPECTED PACKET ORIGIN");
        p->status            = 0;
        p->rssic             = -128.0;
        p->rssis             = -128.0;
        p->snr               = -128.0;
        p->snr_min           = -128.0;
        p->snr_max           = -128.0;
        p->bandwidth         = 0;
        p->datarate          = 0;
        p->coderate          = 0;
        timestamp_correction = 0;
    }

    /* Update counter reference / wrap status before expanding */
    // timestamp_counter_get(&counter_us, false);

    /* Scale 32 MHz packet timestamp to 1 MHz (microseconds) */
    p->count_us = pkt.timestamp_cnt / 32;

    /* Expand 27-bits counter to 32-bits counter, based on current wrapping
     * status */
    // p->count_us = timestamp_pkt_expand(&counter_us, p->count_us);

    /* Packet timestamp corrected */
    p->count_us = p->count_us - timestamp_correction;

    /* Packet CRC status */
    p->crc = pkt.rx_crc16_value;

    return 0;
}

float sx1302_rssi_get_temperature_offset(sx1302_t *dev, uint8_t rf_chain,
                                         float temperature) {
    DEBUG_PUTS("INFO: RSSI temperature compensation:");
    DEBUG("       coeff_a: %.3f\n", dev->rf_chain[rf_chain].rssi_tcomp.coeff_a);
    DEBUG("       coeff_b: %.3f\n", dev->rf_chain[rf_chain].rssi_tcomp.coeff_b);
    DEBUG("       coeff_c: %.3f\n", dev->rf_chain[rf_chain].rssi_tcomp.coeff_c);
    DEBUG("       coeff_d: %.3f\n", dev->rf_chain[rf_chain].rssi_tcomp.coeff_d);
    DEBUG("       coeff_e: %.3f\n", dev->rf_chain[rf_chain].rssi_tcomp.coeff_e);

    /* Compute the offset to be applied to RSSI for given temperature */
    return ((dev->rf_chain[rf_chain].rssi_tcomp.coeff_a * pow(temperature, 4)) +
            (dev->rf_chain[rf_chain].rssi_tcomp.coeff_b * pow(temperature, 3)) +
            (dev->rf_chain[rf_chain].rssi_tcomp.coeff_c * pow(temperature, 2)) +
            (dev->rf_chain[rf_chain].rssi_tcomp.coeff_d * temperature) +
            dev->rf_chain[rf_chain].rssi_tcomp.coeff_e) /
           pow(2, 16);
}

int sx1302_receive(sx1302_t *dev, uint8_t max_pkt,
                   sx1302_packet_rx_t *pkt_data) {
    int res;
    uint8_t nb_pkt_fetched    = 0;
    uint16_t nb_pkt_found     = 0;
    uint16_t nb_pkt_left      = 0;
    float current_temperature = 25;
    float rssi_temperature_offset;

    /* Check that AGC/ARB firmwares are not corrupted, and update internal
     * counter */
    /* WARNING: this needs to be called regularly by the upper layer */
    res = sx1302_update(dev);
    if (res != 0) {
        return -1;
    }

    /* Get packets from SX1302, if any */
    res = sx1302_fetch(dev, &nb_pkt_fetched);
    if (res != 0) {
        printf("ERROR: failed to fetch packets from SX1302\n");
        return -1;
    }
    if (nb_pkt_fetched == 0) {
        return 0;
    }
    if (nb_pkt_fetched > max_pkt) {
        nb_pkt_left = nb_pkt_fetched - max_pkt;
        printf(
            "WARNING: not enough space allocated, fetched %d "
            "packet(s), %d will be left in RX buffer\n",
            nb_pkt_fetched, nb_pkt_left);
    }

    /* Apply RSSI temperature compensation
    res = stts751_get_temperature(ts_fd, ts_addr, &current_temperature);
    if (res != LGW_I2C_SUCCESS) {
            printf("ERROR: failed to get current temperature\n");
            return LGW_HAL_ERROR;
    }*/

    /* Iterate on the RX buffer to get parsed packets */
    for (nb_pkt_found = 0;
         nb_pkt_found <
         ((nb_pkt_fetched <= max_pkt) ? nb_pkt_fetched : max_pkt);
         nb_pkt_found++) {
        /* Get packet and move to next one */
        res = sx1302_parse(dev, &pkt_data[nb_pkt_found]);
        if (res != 0) {
            printf(
                "ERROR: failed to parse fetched packet %d, "
                "aborting...\n",
                nb_pkt_found);
            return -1;
        }

        /* Appli RSSI offset calibrated for the board */
        pkt_data[nb_pkt_found].rssic +=
            dev->rf_chain[pkt_data[nb_pkt_found].rf_chain].rssi_offset;
        pkt_data[nb_pkt_found].rssis +=
            dev->rf_chain[pkt_data[nb_pkt_found].rf_chain].rssi_offset;

        rssi_temperature_offset = sx1302_rssi_get_temperature_offset(
            dev, pkt_data[nb_pkt_found].rf_chain, current_temperature);
        pkt_data[nb_pkt_found].rssic += rssi_temperature_offset;
        pkt_data[nb_pkt_found].rssis += rssi_temperature_offset;
        DEBUG(
            "INFO: RSSI temperature offset applied: %.3f dB "
            "(current temperature %.1f C)\n",
            rssi_temperature_offset, current_temperature);
    }

    DEBUG("INFO: nb pkt found:%u left:%u\n", nb_pkt_found, nb_pkt_left);

    return nb_pkt_found;
}