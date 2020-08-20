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
 *
 * @file
 * @brief       SX1302 Receiving code
 *
 * @author      Pierre Millot
 *
 */

#ifndef SX1302_RX_H
#define SX1302_RX_H

#include "sx1302.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RX buffer packet structure */
#define SX1302_PKT_SYNCWORD_BYTE_0 0xA5
#define SX1302_PKT_SYNCWORD_BYTE_1 0xC0
#define SX1302_PKT_HEAD_METADATA   9
#define SX1302_PKT_TAIL_METADATA   14

/* modem IDs */
#define SX1302_LORA_MODEM_ID_MAX 15
#define SX1302_LORA_STD_MODEM_ID 16
#define SX1302_FSK_MODEM_ID      17

#define SX1302_IF_LORA_STD 0x10 /* if + standard single-SF LoRa modem */
/* if + LoRa receiver with multi-SF capability */
#define SX1302_IF_LORA_MULTI 0x11
#define SX1302_IF_FSK_STD    0x20 /* if + standard FSK modem */

#define SX1302_PKT_PAYLOAD_LENGTH(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 2], 0, 8)
#define SX1302_PKT_CHANNEL(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 3], 0, 8)
#define SX1302_PKT_CRC_EN(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 4], 0, 1)
#define SX1302_PKT_CODING_RATE(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 4], 1, 3)
#define SX1302_PKT_DATARATE(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 4], 4, 4)
#define SX1302_PKT_MODEM_ID(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 5], 0, 8)
#define SX1302_PKT_FREQ_OFFSET_7_0(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 6], 0, 8)
#define SX1302_PKT_FREQ_OFFSET_15_8(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 7], 0, 8)
#define SX1302_PKT_FREQ_OFFSET_19_16(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 8], 0, 4)
#define SX1302_PKT_CRC_ERROR(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 9], 0, 1)
#define SX1302_PKT_SYNC_ERROR(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 9], 2, 1)
#define SX1302_PKT_HEADER_ERROR(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 9], 3, 1)
#define SX1302_PKT_TIMING_SET(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 9], 4, 1)
#define SX1302_PKT_SNR_AVG(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 10], 0, 8)
#define SX1302_PKT_RSSI_CHAN(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 11], 0, 8)
#define SX1302_PKT_RSSI_SIG(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 12], 0, 8)
#define SX1302_PKT_RSSI_CHAN_MAX_NEG_DELTA(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 13], 0, 4)
#define SX1302_PKT_RSSI_CHAN_MAX_POS_DELTA(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 13], 4, 4)
#define SX1302_PKT_RSSI_SIG_MAX_NEG_DELTA(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 14], 0, 4)
#define SX1302_PKT_RSSI_SIG_MAX_POS_DELTA(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 14], 4, 4)
#define SX1302_PKT_TIMESTAMP_7_0(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 15], 0, 8)
#define SX1302_PKT_TIMESTAMP_15_8(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 16], 0, 8)
#define SX1302_PKT_TIMESTAMP_23_16(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 17], 0, 8)
#define SX1302_PKT_TIMESTAMP_31_24(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 18], 0, 8)
#define SX1302_PKT_CRC_PAYLOAD_7_0(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 19], 0, 8)
#define SX1302_PKT_CRC_PAYLOAD_15_8(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 20], 0, 8)
#define SX1302_PKT_NUM_TS_METRICS(buffer, start_index) \
    SX1302_TAKE_N_BITS_FROM(buffer[start_index + 21], 0, 8)

/**
@struct sx1302_rx_packet_t
@brief packet structure as contained in the sx1302 RX packet engine
*/
typedef struct {
    uint8_t rxbytenb_modem;
    uint8_t rx_channel_in;
    bool crc_en;
    uint8_t coding_rate; /* LoRa only */
    uint8_t rx_rate_sf;  /* LoRa only */
    uint8_t modem_id;
    int32_t frequency_offset_error; /* LoRa only */
    uint8_t payload[255];
    bool payload_crc_error;
    bool sync_error;    /* LoRa only */
    bool header_error;  /* LoRa only */
    bool timing_set;    /* LoRa only */
    int8_t snr_average; /* LoRa only */
    uint8_t rssi_chan_avg;
    uint8_t rssi_signal_avg; /* LoRa only */
    uint8_t rssi_chan_max_neg_delta;
    uint8_t rssi_chan_max_pos_delta;
    uint8_t rssi_sig_max_neg_delta; /* LoRa only */
    uint8_t rssi_sig_max_pos_delta; /* LoRa only */
    uint32_t timestamp_cnt;
    uint16_t rx_crc16_value;       /* LoRa only */
    uint8_t num_ts_metrics_stored; /* LoRa only */
    uint8_t timestamp_avg[255];    /* LoRa only */
    uint8_t timestamp_stddev[255]; /* LoRa only */
    uint8_t packet_checksum;
} sx1302_rx_packet_t;

int sx1302_receive(sx1302_t *dev, uint8_t max_pkt,
                   sx1302_packet_rx_t *pkt_data);

#ifdef __cplusplus
}
#endif

#endif /* SX1302_RX_H */
/** @} */
