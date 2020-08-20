/*
 * Copyright (C) 2020 Université Grenoble Alpes
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_sx1302
 * @ingroup     drivers_netdev
 * @brief       Driver for Semtech SX1302.
 *
 * @{
 *
 * @file
 * @brief       Public interface for SX1302 driver
 *
 * @author      Pierre Millot
 */

#ifndef SX1302_H
#define SX1302_H

#include "net/netdev.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "xtimer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SX1302_DEBUG 1

/**
 * @brief   GPIO mode of DIOx Pins.
 */
#ifndef SX1302_DIO_PULL_MODE
#define SX1302_DIO_PULL_MODE (GPIO_IN_PD)
#endif

#define SX1302_CHANNEL_DEFAULT \
    (868300000UL) /**< Default channel frequency, 868.3MHz (Europe) */

#define SX1302_MIN_LORA_PREAMBLE 6
#define SX1302_STD_LORA_PREAMBLE 8

#define SX1302_TX_START_DELAY_DEFAULT      \
    1500 /* Calibrated value for 500KHz BW \
          */

#define SX1302_REG_SELECT(rf_chain, a, b) ((rf_chain == 0) ? a : b)

#define SX1302_IF_HZ_TO_REG(f) ((f << 5) / 15625)
#define SX1302_FREQ_TO_REG(f)  (uint32_t)((uint64_t)f * (1 << 18) / 32000000U)

#define SX1302_SET_PPM_ON(bw, dr)                                      \
    (((bw == SX1302_LORA_BW_125_KHZ) &&                                \
      ((dr == SX1302_DR_LORA_SF11) || (dr == SX1302_DR_LORA_SF12))) || \
     ((bw == SX1302_LORA_BW_250_KHZ) && (dr == SX1302_DR_LORA_SF12)))

/**
@brief Get a particular bit value from a byte
@param b [in]   Any byte from which we want a bit value
@param p [in]   Position of the bit in the byte [0..7]
@param n [in]   Number of bits we want to get
@return The value corresponding the requested bits
*/
#define SX1302_TAKE_N_BITS_FROM(b, p, n) (((b) >> (p)) & ((1 << (n)) - 1))


/* values available for the 'status' parameter */
/* NOTE: values according to hardware specification */
#define SX1302_STAT_UNDEFINED 0x00
#define SX1302_STAT_NO_CRC    0x01
#define SX1302_STAT_CRC_BAD   0x11
#define SX1302_STAT_CRC_OK    0x10

/* 125000 * 32 / 2^6 / 2^19 */
#define SX1302_FREQ_OFFSET_LSB_125KHZ 0.11920929f
/* 250000 * 32 / 2^6 / 2^19 */
#define SX1302_FREQ_OFFSET_LSB_250KHZ 0.238418579f
/* 500000 * 32 / 2^6 / 2^19 */
#define SX1302_FREQ_OFFSET_LSB_500KHZ 0.476837158f

/**
 * @brief   SX1302 initialization result.
 */
enum {
    SX1302_INIT_OK = 0, /**< Initialization was successful */
    SX1302_ERR_SPI,     /**< Failed to initialize SPI bus or CS line */
    SX1302_ERR_GPIOS,   /**< Failed to initialize GPIOs */
    SX1302_ERR_NODEV    /**< No valid device version found */
};

typedef enum {
    SX1302_DR_LORA_SF5 = 5,
    SX1302_DR_LORA_SF6,
    SX1302_DR_LORA_SF7,
    SX1302_DR_LORA_SF8,
    SX1302_DR_LORA_SF9,
    SX1302_DR_LORA_SF10,
    SX1302_DR_LORA_SF11,
    SX1302_DR_LORA_SF12
} SX1302_DataRate_t;

typedef enum {
    SX1302_CR_LORA_4_5 = 1,
    SX1302_CR_LORA_4_6,
    SX1302_CR_LORA_4_7,
    SX1302_CR_LORA_4_8
} SX1302_CodeRate_t;

/**
 * @brief   Radio driver internal state machine states definition.
 */
typedef enum {
    SX1302_RF_IDLE = 0,   /**< Idle state */
    SX1302_RF_RX_RUNNING, /**< Sending state */
    SX1302_RF_TX_RUNNING  /**< Receiving state */
} SX1302_States_t;

typedef enum {
    SX1302_SPI_MUX_TARGET_SX1302,
    SX1302_SPI_MUX_TARGET_RADIOA,
    SX1302_SPI_MUX_TARGET_RADIOB
} SX1302_SpiMuxTarget_t;

typedef enum {
    SX1302_IMMEDIATE,
    SX1302_TIMESTAMPED,
    SX1302_ON_GPS
} SX1302_TX_Mode_t;

/**
 * @brief   LoRa modulation bandwidth.
 */
typedef enum {
    SX1302_LORA_BW_125_KHZ = 4, /**< 125 kHz bandwidth */
    SX1302_LORA_BW_250_KHZ,     /**< 250 kHz bandwidth */
    SX1302_LORA_BW_500_KHZ      /**< 500 kHz bandwidth */
} SX1302_Bandwidth_t;

/* status code for TX_STATUS */
/* NOTE: arbitrary values */
typedef enum {
    SX1302_TX_STATUS_UNKNOWN,
    SX1302_TX_OFF,       /* TX modem disabled, it will ignore commands */
    SX1302_TX_FREE,      /* TX modem is free, ready to receive a command */
    SX1302_TX_SCHEDULED, /* TX modem is loaded, ready to send the packet
                            after an*/
                         /* event and/or delay */
    SX1302_TX_EMITTING   /* TX modem is emitting */
} SX1302_TX_Status_t;

/**
@struct sx1302_tx_gain_t
@brief Structure containing all gains of Tx chain
*/
typedef struct {
    int8_t rf_power;  /*!> measured TX power at the board connector, in dBm
                       */
    uint8_t dig_gain; /*!> (sx125x) 2 bits: control of the digital gain of
                         SX1302 */
    uint8_t pa_gain;  /*!> (sx125x) 2 bits: control of the external PA
                         (SX1302 I/O)  (sx1250) 1 bits: enable/disable the
                         external PA (SX1302 I/O) */
    uint8_t dac_gain; /*!> (sx125x) 2 bits: control of the radio DAC */
    uint8_t mix_gain; /*!> (sx125x) 4 bits: control of the radio mixer */
    int8_t offset_i;  /*!> (sx125x) calibrated I offset */
    int8_t offset_q;  /*!> (sx125x) calibrated Q offset */
    uint8_t pwr_idx;  /*!> (sx1250) 6 bits: control the radio power index to
                         be used for configuration */
} sx1302_tx_gain_t;

/**
@struct sx1302_conf_rxif_t
@brief Configuration structure for an IF chain
*/
typedef struct {
    bool enable;      /*!> enable or disable that IF chain */
    uint8_t rf_chain; /*!> to which RF chain is that IF chain associated */
    int32_t freq_hz;  /*!> center frequ of the IF chain, relative to RF chain
                         frequency */
    SX1302_Bandwidth_t bandwidth; /*!> RX bandwidth, 0 for default */
    SX1302_DataRate_t datarate;   /*!> RX datarate, 0 for default */
    uint8_t sync_word_size;       /*!> size of FSK sync word (number of bytes, 0
                                     for default) */
    uint64_t sync_word; /*!> FSK sync word (ALIGN RIGHT, eg. 0xC194C1) */
    bool implicit_hdr;  /*!> LoRa Service implicit header */
    uint8_t implicit_payload_length; /*!> LoRa Service implicit header
                                        payload length (number of bytes, 0
                                        for default) */
    bool implicit_crc_en;      /*!> LoRa Service implicit header CRC enable */
    uint8_t implicit_coderate; /*!> LoRa Service implicit header coding rate
                                */
} sx1302_conf_rxif_t;

/**
@struct sx1302_packet_tx_t
@brief Structure containing the metadata of a packet that was received and a
pointer to the payload
*/
typedef struct {
    uint32_t freq_hz; /*!> central frequency of the IF chain */
    int32_t freq_offset;
    uint8_t if_chain;  /*!> by which IF chain was packet received */
    uint8_t status;    /*!> status of the received packet */
    uint32_t count_us; /*!> internal concentrator counter for timestamping,
                          1 microsecond resolution */
    uint8_t rf_chain;  /*!> through which RF chain the packet was received */
    uint8_t modem_id;
    SX1302_Bandwidth_t bandwidth; /*!> modulation bandwidth (LoRa only) */
    SX1302_DataRate_t datarate; /*!> RX datarate of the packet (SF for LoRa) */
    SX1302_CodeRate_t coderate; /*!> error-correcting code of the packet
                                   (LoRa only) */
    float rssic;                /*!> average RSSI of the channel in dB */
    float rssis;                /*!> average RSSI of the signal in dB */
    float snr;                  /*!> average packet SNR, in dB (LoRa only) */
    float snr_min;              /*!> minimum packet SNR, in dB (LoRa only) */
    float snr_max;              /*!> maximum packet SNR, in dB (LoRa only) */
    uint16_t crc;               /*!> CRC that was received in the payload */
    uint16_t size;              /*!> payload size in bytes */
    uint8_t payload[256];       /*!> buffer containing the payload */
} sx1302_packet_rx_t;

/**
@struct sx1302_packet_tx_t
@brief Structure containing the configuration of a packet to send and a pointer
to the payload
*/
typedef struct {
    uint32_t freq_hz; /*!> center frequency of TX */
    SX1302_TX_Mode_t
        tx_mode;        /*!> select on what event/time the TX is triggered */
    uint32_t count_us;  /*!> timestamp or delay in microseconds for TX
                           trigger */
    uint8_t rf_chain;   /*!> through which RF chain will the packet be sent */
    int8_t rf_power;    /*!> TX power, in dBm */
    int8_t freq_offset; /*!> frequency offset from Radio Tx frequency (CW
                           mode) */
    SX1302_Bandwidth_t bandwidth; /*!> modulation bandwidth (LoRa only) */
    SX1302_DataRate_t datarate;   /*!> TX datarate, SF */
    SX1302_CodeRate_t
        coderate;      /*!> error-correcting code of the packet (LoRa only)
                        */
    bool invert_pol;   /*!> invert signal polarity, for orthogonal downlinks
                          (LoRa only) */
    uint16_t preamble; /*!> set the preamble length, 0 for default */
    bool no_crc;       /*!> if true, do not send a CRC in the packet */
    bool no_header;    /*!> if true, enable implicit header mode (LoRa), fixed
                          length (FSK) */
    uint16_t size;     /*!> payload size in bytes */
    uint8_t payload[256]; /*!> buffer containing the payload */
} sx1302_packet_tx_t;

/**
@struct sx1302_rx_buffer_t
@brief buffer to hold the data fetched from the sx1302 RX buffer
*/
typedef struct {
    uint8_t buffer[1024]; /*!> byte array to hald the data fetched from the
                             RX buffer */
    uint16_t buffer_size; /*!> The number of bytes currently stored in the
                             buffer */
    int buffer_index;     /*!> Current parsing index in the buffer */
    uint8_t buffer_pkt_nb;
} sx1302_rx_buffer_t;

/**
 * @brief   LoRa configuration structure.
 */
typedef struct {
    uint16_t preamble_len;        /**< Length of preamble header */
    int8_t power;                 /**< Signal power */
    SX1302_Bandwidth_t bandwidth; /**< Signal bandwidth */
    SX1302_DataRate_t datarate;   /**< Spreading factor rate, e.g datarate */
    SX1302_CodeRate_t coderate;   /**< Error coding rate */
    uint32_t rx_timeout;          /**< RX timeout in microseconds */
    uint32_t tx_timeout;          /**< TX timeout in microseconds */
} sx1302_lora_settings_t;

/**
@struct sx1302_rssi_tconf_t
@brief Structure containing all coefficients necessary to compute the offset to
be applied on RSSI for current temperature
*/
typedef struct {
    float coeff_a;
    float coeff_b;
    float coeff_c;
    float coeff_d;
    float coeff_e;
} sx1302_rssi_tconf_t;

/**
@struct sx1302_radio_conf_t
@brief Configuration structure for a RF chain
*/
typedef struct {
    bool enable;       /*!> enable or disable that RF chain */
    uint32_t freq_hz;  /*!> center frequency of the radio in Hz */
    float rssi_offset; /*!> Board-specific RSSI correction factor */
    sx1302_rssi_tconf_t rssi_tcomp; /*!> Board-specific RSSI temperature
                                    compensation coefficients */
    bool tx_enable;         /*!> enable or disable TX on that RF chain */
    bool single_input_mode; /*!> Configure the radio in single or
                               differential input mode (SX1250 only) */
} sx1302_radio_conf_t;

/**
 * @brief   Radio settings.
 */
typedef struct {
    uint32_t channel;            /**< Radio channel */
    uint8_t state;               /**< Radio state */
    sx1302_lora_settings_t lora; /**< LoRa settings */
    bool single_receive;
} sx1302_radio_settings_t;

/**
 * @brief   SX1302 hardware and global parameters.
 */
typedef struct {
    spi_t spi;           /**< SPI device */
    gpio_t nss_pin;      /**< SPI NSS pin */
    gpio_t reset_pin;    /**< Reset pin */
    gpio_t power_en_pin; /**< Power enable pin */
} sx1302_params_t;

/**
@struct sx1302_board_t
@brief Configuration structure for board specificities
*/
typedef struct {
    bool lorawan_public; /*!> Enable ONLY for *public* networks using the
                            LoRa MAC protocol */
    uint8_t clksrc;      /*!> Index of RF chain which provides clock to
                            concentrator */
    bool full_duplex;    /*!> Indicates if the gateway operates in full duplex
                            mode or not */
} sx1302_board_t;

/**
 * @brief   SX1302 device descriptor.
 * @extends netdev_t
 */
typedef struct {
    netdev_t netdev;                  /**< Netdev parent struct */
    sx1302_radio_settings_t settings; /**< Radio settings */
    sx1302_params_t params;           /**< Device driver parameters */
    sx1302_radio_conf_t rf_chain[2];
    sx1302_conf_rxif_t if_chain[10];
    sx1302_board_t board;
    sx1302_conf_rxif_t lora_service;
    sx1302_rx_buffer_t rx_buffer;
} sx1302_t;

/**
 * @brief   Setup the SX1302
 *
 * @param[in] dev                      Device descriptor
 * @param[in] params                   Parameters for device initialization
 *
 * @return the netdev associated with the device
 */
netdev_t *sx1302_setup(sx1302_t *dev, const sx1302_params_t *params);

/**
 * @brief   Initializes the transceiver.
 *
 * @param[in] dev                      The sx1302 device descriptor
 *
 * @return result of initialization
 */
int sx1302_init(sx1302_t *dev);

void sx1302_set_default(sx1302_t *dev);

void sx1302_reset(sx1302_t *dev);

uint64_t sx1302_get_eui(sx1302_t *dev);

int32_t sx1302_get_bandwidth_value(SX1302_Bandwidth_t bandwidth);

int sx1302_tx_set_start_delay(sx1302_t *dev, uint8_t rf_chain,
                              uint8_t bandwidth, uint16_t *delay);

void sx1302_init_radio(sx1302_t *dev);

SX1302_States_t sx1302_get_state(sx1302_t *dev);

void sx1302_set_state(sx1302_t *dev, SX1302_States_t state);



int sx1302_send(sx1302_t *dev, bool lwan_public, sx1302_packet_tx_t *pkt_data);

SX1302_TX_Status_t sx1302_tx_status(sx1302_t *dev, uint8_t rf_chain);

#ifdef __cplusplus
}
#endif

#endif /* SX1302_H */
/** @} */
