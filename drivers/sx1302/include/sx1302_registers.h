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
 * @brief       SX1302 registers
 *
 * @author      Pierre Millot
 *
 */

#ifndef SX1302_REGISTERS_H
#define SX1302_REGISTERS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name   SX1302 LoRa modem internal registers addresses
 * @{
 */

typedef struct {
    int8_t page;   /*!< page containing the register (-1 for all pages) */
    uint16_t addr; /*!< base address of the register (15 bit) */
    uint8_t offs;  /*!< position of the register LSB (between 0 to 7) */
    bool sign;     /*!< 1 indicates the register is signed (2 complem.) */
    uint8_t leng;  /*!< number of bits in the register */
    bool rdon;     /*!< 1 indicates a read-only register */
    bool chck;     /*!< register can be checked or not: (pulse, w0clr, w1clr) */
    int32_t dflt;  /*!< register default value */
} sx1302_regs_t;

#define SX1302_REG_EXT_MEM_PAGED_BASE_ADDR           0x0
#define SX1302_REG_RX_BUFFER_BASE_ADDR               0x4000
#define SX1302_REG_TX_TOP_A_BASE_ADDR                0x5200
#define SX1302_REG_TX_TOP_B_BASE_ADDR                0x5400
#define SX1302_REG_COMMON_BASE_ADDR                  0x5600
#define SX1302_REG_GPIO_BASE_ADDR                    0x5640
#define SX1302_REG_MBIST_BASE_ADDR                   0x56c0
#define SX1302_REG_RADIO_FE_BASE_ADDR                0x5700
#define SX1302_REG_AGC_MCU_BASE_ADDR                 0x5780
#define SX1302_REG_CLK_CTRL_BASE_ADDR                0x57c0
#define SX1302_REG_RX_TOP_BASE_ADDR                  0x5800
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR 0x5b00
#define SX1302_REG_CAPTURE_RAM_BASE_ADDR             0x6000
#define SX1302_REG_ARB_MCU_BASE_ADDR                 0x6080
#define SX1302_REG_TIMESTAMP_BASE_ADDR               0x6100
#define SX1302_REG_OTP_BASE_ADDR                     0x6180

#define SX1302_READ_REGISTER  0x00
#define SX1302_WRITE_REGISTER 0x80

#define SX1302_REG_COMMON_PAGE_PAGE                                        0
#define SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL                             1
#define SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL                            2
#define SX1302_REG_COMMON_CTRL0_RADIO_MISC_EN                              3
#define SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_B                        4
#define SX1302_REG_COMMON_CTRL0_SX1261_MODE_RADIO_A                        5
#define SX1302_REG_COMMON_CTRL1_SWAP_IQ_RADIO_B                            6
#define SX1302_REG_COMMON_CTRL1_SAMPLING_EDGE_RADIO_B                      7
#define SX1302_REG_COMMON_CTRL1_SWAP_IQ_RADIO_A                            8
#define SX1302_REG_COMMON_CTRL1_SAMPLING_EDGE_RADIO_A                      9
#define SX1302_REG_COMMON_SPI_DIV_RATIO_SPI_HALF_PERIOD                    10
#define SX1302_REG_COMMON_RADIO_SELECT_RADIO_SELECT                        11
#define SX1302_REG_COMMON_GEN_GLOBAL_EN                                    12
#define SX1302_REG_COMMON_GEN_FSK_MODEM_ENABLE                             13
#define SX1302_REG_COMMON_GEN_CONCENTRATOR_MODEM_ENABLE                    14
#define SX1302_REG_COMMON_GEN_MBWSSF_MODEM_ENABLE                          15
#define SX1302_REG_COMMON_VERSION_VERSION                                  16
#define SX1302_REG_COMMON_DUMMY_DUMMY                                      17
#define SX1302_REG_AGC_MCU_CTRL_CLK_EN                                     18
#define SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL                         19
#define SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR                                  20
#define SX1302_REG_AGC_MCU_CTRL_HOST_PROG                                  21
#define SX1302_REG_AGC_MCU_CTRL_PARITY_ERROR                               22
#define SX1302_REG_AGC_MCU_MCU_AGC_STATUS_MCU_AGC_STATUS                   23
#define SX1302_REG_AGC_MCU_PA_GAIN_PA_B_GAIN                               24
#define SX1302_REG_AGC_MCU_PA_GAIN_PA_A_GAIN                               25
#define SX1302_REG_AGC_MCU_RF_EN_A_RADIO_RST                               26
#define SX1302_REG_AGC_MCU_RF_EN_A_RADIO_EN                                27
#define SX1302_REG_AGC_MCU_RF_EN_A_PA_EN                                   28
#define SX1302_REG_AGC_MCU_RF_EN_A_LNA_EN                                  29
#define SX1302_REG_AGC_MCU_RF_EN_B_RADIO_RST                               30
#define SX1302_REG_AGC_MCU_RF_EN_B_RADIO_EN                                31
#define SX1302_REG_AGC_MCU_RF_EN_B_PA_EN                                   32
#define SX1302_REG_AGC_MCU_RF_EN_B_LNA_EN                                  33
#define SX1302_REG_AGC_MCU_LUT_TABLE_A_PA_LUT                              34
#define SX1302_REG_AGC_MCU_LUT_TABLE_A_LNA_LUT                             35
#define SX1302_REG_AGC_MCU_LUT_TABLE_B_PA_LUT                              36
#define SX1302_REG_AGC_MCU_LUT_TABLE_B_LNA_LUT                             37
#define SX1302_REG_AGC_MCU_UART_CFG_MSBF                                   38
#define SX1302_REG_AGC_MCU_UART_CFG_PAR_EN                                 39
#define SX1302_REG_AGC_MCU_UART_CFG_PAR_MODE                               40
#define SX1302_REG_AGC_MCU_UART_CFG_START_LEN                              41
#define SX1302_REG_AGC_MCU_UART_CFG_STOP_LEN                               42
#define SX1302_REG_AGC_MCU_UART_CFG_WORD_LEN                               43
#define SX1302_REG_AGC_MCU_UART_CFG2_BIT_RATE                              44
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE3_MCU_MAIL_BOX_WR_DATA 45
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE2_MCU_MAIL_BOX_WR_DATA 46
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE1_MCU_MAIL_BOX_WR_DATA 47
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA 48
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE3_MCU_MAIL_BOX_RD_DATA 49
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE2_MCU_MAIL_BOX_RD_DATA 50
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE1_MCU_MAIL_BOX_RD_DATA 51
#define SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE0_MCU_MAIL_BOX_RD_DATA 52
#define SX1302_REG_AGC_MCU_DUMMY_DUMMY3                                    53
#define SX1302_REG_CLK_CTRL_CLK_SEL_CLKDIV_EN                              54
#define SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL                        55
#define SX1302_REG_CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL                        56
#define SX1302_REG_CLK_CTRL_DUMMY_DUMMY                                    57
#define SX1302_REG_TX_TOP_A_TX_TRIG_TX_FSM_CLR                             58
#define SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_GPS                            59
#define SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_DELAYED                        60
#define SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE                      61
#define SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG            62
#define SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG            63
#define SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG            64
#define SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG            65
#define SX1302_REG_TX_TOP_A_TX_START_DELAY_MSB_TX_START_DELAY              66
#define SX1302_REG_TX_TOP_A_TX_START_DELAY_LSB_TX_START_DELAY              67
#define SX1302_REG_TX_TOP_A_TX_CTRL_WRITE_BUFFER                           68
#define SX1302_REG_TX_TOP_A_TX_RAMP_DURATION_TX_RAMP_DURATION              69
#define SX1302_REG_TX_TOP_A_GEN_CFG_0_MODULATION_TYPE                      70
#define SX1302_REG_TX_TOP_A_TEST_0_TX_ACTIVE_CTRL                          71
#define SX1302_REG_TX_TOP_A_TEST_0_TX_ACTIVE_SEL                           72
#define SX1302_REG_TX_TOP_A_TX_FLAG_TX_TIMEOUT                             73
#define SX1302_REG_TX_TOP_A_TX_FLAG_PKT_DONE                               74
#define SX1302_REG_TX_TOP_A_AGC_TX_BW_AGC_TX_BW                            75
#define SX1302_REG_TX_TOP_A_AGC_TX_PWR_AGC_TX_PWR                          76
#define SX1302_REG_TX_TOP_A_TIMEOUT_CNT_BYTE_2_TIMEOUT_CNT                 77
#define SX1302_REG_TX_TOP_A_TIMEOUT_CNT_BYTE_1_TIMEOUT_CNT                 78
#define SX1302_REG_TX_TOP_A_TIMEOUT_CNT_BYTE_0_TIMEOUT_CNT                 79
#define SX1302_REG_TX_TOP_A_TX_FSM_STATUS_TX_STATUS                        80
#define SX1302_REG_TX_TOP_A_DUMMY_CONTROL_DUMMY                            81
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_PLL_DIV_CTRL                   82
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE                    83
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE                        84
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST                      85
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC                      86
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL2_SX125X_IQ_INVERT              87
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC              88
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_IQ_GAIN_IQ_GAIN                     89
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET                   90
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET                   91
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF                   92
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF                   93
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF                   94
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV                 95
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV                 96
#define SX1302_REG_TX_TOP_A_TX_RFFE_IF_TEST_MOD_FREQ                       97
#define SX1302_REG_TX_TOP_A_DUMMY_MODULATOR_DUMMY                          98
#define SX1302_REG_TX_TOP_A_FSK_PKT_LEN_PKT_LENGTH                         99
#define SX1302_REG_TX_TOP_A_FSK_CFG_0_TX_CONT                              100
#define SX1302_REG_TX_TOP_A_FSK_CFG_0_CRC_IBM                              101
#define SX1302_REG_TX_TOP_A_FSK_CFG_0_DCFREE_ENC                           102
#define SX1302_REG_TX_TOP_A_FSK_CFG_0_CRC_EN                               103
#define SX1302_REG_TX_TOP_A_FSK_CFG_0_PKT_MODE                             104
#define SX1302_REG_TX_TOP_A_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE            105
#define SX1302_REG_TX_TOP_A_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE            106
#define SX1302_REG_TX_TOP_A_FSK_BIT_RATE_MSB_BIT_RATE                      107
#define SX1302_REG_TX_TOP_A_FSK_BIT_RATE_LSB_BIT_RATE                      108
#define SX1302_REG_TX_TOP_A_FSK_MOD_FSK_REF_PATTERN_SIZE                   109
#define SX1302_REG_TX_TOP_A_FSK_MOD_FSK_PREAMBLE_SEQ                       110
#define SX1302_REG_TX_TOP_A_FSK_MOD_FSK_REF_PATTERN_EN                     111
#define SX1302_REG_TX_TOP_A_FSK_MOD_FSK_GAUSSIAN_SELECT_BT                 112
#define SX1302_REG_TX_TOP_A_FSK_MOD_FSK_GAUSSIAN_EN                        113
#define SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN          114
#define SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN          115
#define SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN          116
#define SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN          117
#define SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN          118
#define SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN          119
#define SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN          120
#define SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN          121
#define SX1302_REG_TX_TOP_A_DUMMY_GSFK_DUMMY                               122
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW                           123
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_SF                           124
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL                125
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_1_PPM_OFFSET                         126
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG             127
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_1_CODING_RATE                        128
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_2_FINE_SYNCH_EN                      129
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_2_MODEM_EN                           130
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CADRXTX                            131
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_2_IMPLICIT_HEADER                    132
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CRC_EN                             133
#define SX1302_REG_TX_TOP_A_TXRX_CFG0_3_PAYLOAD_LENGTH                     134
#define SX1302_REG_TX_TOP_A_TXRX_CFG1_0_INT_STEP_ORIDE_EN                  135
#define SX1302_REG_TX_TOP_A_TXRX_CFG1_0_INT_STEP_ORIDE                     136
#define SX1302_REG_TX_TOP_A_TXRX_CFG1_1_MODEM_START                        137
#define SX1302_REG_TX_TOP_A_TXRX_CFG1_1_HEADER_DIFF_MODE                   138
#define SX1302_REG_TX_TOP_A_TXRX_CFG1_1_ZERO_PAD                           139
#define SX1302_REG_TX_TOP_A_TXRX_CFG1_2_PREAMBLE_SYMB_NB                   140
#define SX1302_REG_TX_TOP_A_TXRX_CFG1_3_PREAMBLE_SYMB_NB                   141
#define SX1302_REG_TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_INT_DELAY                 142
#define SX1302_REG_TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_RX                        143
#define SX1302_REG_TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_TX                        144
#define SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_LOWPASS                        145
#define SX1302_REG_TX_TOP_A_TX_CFG0_0_PPM_OFFSET_SIG                       146
#define SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTCHIRP                            147
#define SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_INVERT                         148
#define SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTINUOUS                           149
#define SX1302_REG_TX_TOP_A_TX_CFG0_1_POWER_RANGING                        150
#define SX1302_REG_TX_TOP_A_TX_CFG1_0_FRAME_NB                             151
#define SX1302_REG_TX_TOP_A_TX_CFG1_1_HOP_CTRL                             152
#define SX1302_REG_TX_TOP_A_TX_CFG1_1_IFS                                  153
#define SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_AUTO_SCALE                       154
#define SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_DROP_ON_SYNCH                    155
#define SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_GAIN                             156
#define SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_PEAK1_POS                        157
#define SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_FINETIME_ON_LAST                 158
#define SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_TIMEOUT_OPT                      159
#define SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_PEAK2_POS                        160
#define SX1302_REG_TX_TOP_A_LORA_TX_STATE_STATUS                           161
#define SX1302_REG_TX_TOP_A_LORA_TX_FLAG_FRAME_DONE                        162
#define SX1302_REG_TX_TOP_A_LORA_TX_FLAG_CONT_DONE                         163
#define SX1302_REG_TX_TOP_A_LORA_TX_FLAG_PLD_DONE                          164
#define SX1302_REG_TX_TOP_A_DUMMY_LORA_DUMMY                               165
#define SX1302_REG_TX_TOP_B_TX_TRIG_TX_FSM_CLR                             166
#define SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_GPS                            167
#define SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_DELAYED                        168
#define SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_IMMEDIATE                      169
#define SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG            170
#define SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG            171
#define SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG            172
#define SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG            173
#define SX1302_REG_TX_TOP_B_TX_START_DELAY_MSB_TX_START_DELAY              174
#define SX1302_REG_TX_TOP_B_TX_START_DELAY_LSB_TX_START_DELAY              175
#define SX1302_REG_TX_TOP_B_TX_CTRL_WRITE_BUFFER                           176
#define SX1302_REG_TX_TOP_B_TX_RAMP_DURATION_TX_RAMP_DURATION              177
#define SX1302_REG_TX_TOP_B_GEN_CFG_0_MODULATION_TYPE                      178
#define SX1302_REG_TX_TOP_B_TEST_0_TX_ACTIVE_CTRL                          179
#define SX1302_REG_TX_TOP_B_TEST_0_TX_ACTIVE_SEL                           180
#define SX1302_REG_TX_TOP_B_TX_FLAG_TX_TIMEOUT                             181
#define SX1302_REG_TX_TOP_B_TX_FLAG_PKT_DONE                               182
#define SX1302_REG_TX_TOP_B_AGC_TX_BW_AGC_TX_BW                            183
#define SX1302_REG_TX_TOP_B_AGC_TX_PWR_AGC_TX_PWR                          184
#define SX1302_REG_TX_TOP_B_TIMEOUT_CNT_BYTE_2_TIMEOUT_CNT                 185
#define SX1302_REG_TX_TOP_B_TIMEOUT_CNT_BYTE_1_TIMEOUT_CNT                 186
#define SX1302_REG_TX_TOP_B_TIMEOUT_CNT_BYTE_0_TIMEOUT_CNT                 187
#define SX1302_REG_TX_TOP_B_TX_FSM_STATUS_TX_STATUS                        188
#define SX1302_REG_TX_TOP_B_DUMMY_CONTROL_DUMMY                            189
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_PLL_DIV_CTRL                   190
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_CLK_EDGE                    191
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE                        192
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST                      193
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_SRC                      194
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL2_SX125X_IQ_INVERT              195
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC              196
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_IQ_GAIN_IQ_GAIN                     197
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET                   198
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET                   199
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_H_FREQ_RF                   200
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_M_FREQ_RF                   201
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_L_FREQ_RF                   202
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV                 203
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV                 204
#define SX1302_REG_TX_TOP_B_TX_RFFE_IF_TEST_MOD_FREQ                       205
#define SX1302_REG_TX_TOP_B_DUMMY_MODULATOR_DUMMY                          206
#define SX1302_REG_TX_TOP_B_FSK_PKT_LEN_PKT_LENGTH                         207
#define SX1302_REG_TX_TOP_B_FSK_CFG_0_TX_CONT                              208
#define SX1302_REG_TX_TOP_B_FSK_CFG_0_CRC_IBM                              209
#define SX1302_REG_TX_TOP_B_FSK_CFG_0_DCFREE_ENC                           210
#define SX1302_REG_TX_TOP_B_FSK_CFG_0_CRC_EN                               211
#define SX1302_REG_TX_TOP_B_FSK_CFG_0_PKT_MODE                             212
#define SX1302_REG_TX_TOP_B_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE            213
#define SX1302_REG_TX_TOP_B_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE            214
#define SX1302_REG_TX_TOP_B_FSK_BIT_RATE_MSB_BIT_RATE                      215
#define SX1302_REG_TX_TOP_B_FSK_BIT_RATE_LSB_BIT_RATE                      216
#define SX1302_REG_TX_TOP_B_FSK_MOD_FSK_REF_PATTERN_SIZE                   217
#define SX1302_REG_TX_TOP_B_FSK_MOD_FSK_PREAMBLE_SEQ                       218
#define SX1302_REG_TX_TOP_B_FSK_MOD_FSK_REF_PATTERN_EN                     219
#define SX1302_REG_TX_TOP_B_FSK_MOD_FSK_GAUSSIAN_SELECT_BT                 220
#define SX1302_REG_TX_TOP_B_FSK_MOD_FSK_GAUSSIAN_EN                        221
#define SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN          222
#define SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN          223
#define SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN          224
#define SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN          225
#define SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN          226
#define SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN          227
#define SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN          228
#define SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN          229
#define SX1302_REG_TX_TOP_B_DUMMY_GSFK_DUMMY                               230
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_BW                           231
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_SF                           232
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL                233
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_1_PPM_OFFSET                         234
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG             235
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_1_CODING_RATE                        236
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_2_FINE_SYNCH_EN                      237
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_2_MODEM_EN                           238
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_2_CADRXTX                            239
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_2_IMPLICIT_HEADER                    240
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_2_CRC_EN                             241
#define SX1302_REG_TX_TOP_B_TXRX_CFG0_3_PAYLOAD_LENGTH                     242
#define SX1302_REG_TX_TOP_B_TXRX_CFG1_0_INT_STEP_ORIDE_EN                  243
#define SX1302_REG_TX_TOP_B_TXRX_CFG1_0_INT_STEP_ORIDE                     244
#define SX1302_REG_TX_TOP_B_TXRX_CFG1_1_MODEM_START                        245
#define SX1302_REG_TX_TOP_B_TXRX_CFG1_1_HEADER_DIFF_MODE                   246
#define SX1302_REG_TX_TOP_B_TXRX_CFG1_1_ZERO_PAD                           247
#define SX1302_REG_TX_TOP_B_TXRX_CFG1_2_PREAMBLE_SYMB_NB                   248
#define SX1302_REG_TX_TOP_B_TXRX_CFG1_3_PREAMBLE_SYMB_NB                   249
#define SX1302_REG_TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_INT_DELAY                 250
#define SX1302_REG_TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_RX                        251
#define SX1302_REG_TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_TX                        252
#define SX1302_REG_TX_TOP_B_TX_CFG0_0_CHIRP_LOWPASS                        253
#define SX1302_REG_TX_TOP_B_TX_CFG0_0_PPM_OFFSET_SIG                       254
#define SX1302_REG_TX_TOP_B_TX_CFG0_0_CONTCHIRP                            255
#define SX1302_REG_TX_TOP_B_TX_CFG0_0_CHIRP_INVERT                         256
#define SX1302_REG_TX_TOP_B_TX_CFG0_0_CONTINUOUS                           257
#define SX1302_REG_TX_TOP_B_TX_CFG0_1_POWER_RANGING                        258
#define SX1302_REG_TX_TOP_B_TX_CFG1_0_FRAME_NB                             259
#define SX1302_REG_TX_TOP_B_TX_CFG1_1_HOP_CTRL                             260
#define SX1302_REG_TX_TOP_B_TX_CFG1_1_IFS                                  261
#define SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_AUTO_SCALE                       262
#define SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_DROP_ON_SYNCH                    263
#define SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_GAIN                             264
#define SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_PEAK1_POS                        265
#define SX1302_REG_TX_TOP_B_FRAME_SYNCH_1_FINETIME_ON_LAST                 266
#define SX1302_REG_TX_TOP_B_FRAME_SYNCH_1_TIMEOUT_OPT                      267
#define SX1302_REG_TX_TOP_B_FRAME_SYNCH_1_PEAK2_POS                        268
#define SX1302_REG_TX_TOP_B_LORA_TX_STATE_STATUS                           269
#define SX1302_REG_TX_TOP_B_LORA_TX_FLAG_FRAME_DONE                        270
#define SX1302_REG_TX_TOP_B_LORA_TX_FLAG_CONT_DONE                         271
#define SX1302_REG_TX_TOP_B_LORA_TX_FLAG_PLD_DONE                          272
#define SX1302_REG_TX_TOP_B_DUMMY_LORA_DUMMY                               273
#define SX1302_REG_GPIO_GPIO_DIR_H_DIRECTION                               274
#define SX1302_REG_GPIO_GPIO_DIR_L_DIRECTION                               275
#define SX1302_REG_GPIO_GPIO_OUT_H_OUT_VALUE                               276
#define SX1302_REG_GPIO_GPIO_OUT_L_OUT_VALUE                               277
#define SX1302_REG_GPIO_GPIO_IN_H_IN_VALUE                                 278
#define SX1302_REG_GPIO_GPIO_IN_L_IN_VALUE                                 279
#define SX1302_REG_GPIO_GPIO_PD_H_PD_VALUE                                 280
#define SX1302_REG_GPIO_GPIO_PD_L_PD_VALUE                                 281
#define SX1302_REG_GPIO_GPIO_SEL_0_SELECTION                               282
#define SX1302_REG_GPIO_GPIO_SEL_1_SELECTION                               283
#define SX1302_REG_GPIO_GPIO_SEL_2_SELECTION                               284
#define SX1302_REG_GPIO_GPIO_SEL_3_SELECTION                               285
#define SX1302_REG_GPIO_GPIO_SEL_4_SELECTION                               286
#define SX1302_REG_GPIO_GPIO_SEL_5_SELECTION                               287
#define SX1302_REG_GPIO_GPIO_SEL_6_SELECTION                               288
#define SX1302_REG_GPIO_GPIO_SEL_7_SELECTION                               289
#define SX1302_REG_GPIO_GPIO_SEL_8_11_GPIO_11_9_SEL                        290
#define SX1302_REG_GPIO_GPIO_SEL_8_11_GPIO_8_SEL                           291
#define SX1302_REG_GPIO_HOST_IRQ_TX_TIMEOUT_B                              292
#define SX1302_REG_GPIO_HOST_IRQ_TX_TIMEOUT_A                              293
#define SX1302_REG_GPIO_HOST_IRQ_TX_DONE_B                                 294
#define SX1302_REG_GPIO_HOST_IRQ_TX_DONE_A                                 295
#define SX1302_REG_GPIO_HOST_IRQ_TIMESTAMP                                 296
#define SX1302_REG_GPIO_HOST_IRQ_RX_BUFFER_WATERMARK                       297
#define SX1302_REG_GPIO_HOST_IRQ_EN_TX_TIMEOUT_B                           298
#define SX1302_REG_GPIO_HOST_IRQ_EN_TX_TIMEOUT_A                           299
#define SX1302_REG_GPIO_HOST_IRQ_EN_TX_DONE_B                              300
#define SX1302_REG_GPIO_HOST_IRQ_EN_TX_DONE_A                              301
#define SX1302_REG_GPIO_HOST_IRQ_EN_TIMESTAMP                              302
#define SX1302_REG_GPIO_HOST_IRQ_EN_RX_BUFFER_WATERMARK                    303
#define SX1302_REG_GPIO_DUMMY_DUMMY                                        304
#define SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_POL                              305
#define SX1302_REG_TIMESTAMP_GPS_CTRL_GPS_EN                               306
#define SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS              307
#define SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_MSB1_TIMESTAMP_PPS              308
#define SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_LSB2_TIMESTAMP_PPS              309
#define SX1302_REG_TIMESTAMP_TIMESTAMP_PPS_LSB1_TIMESTAMP_PPS              310
#define SX1302_REG_TIMESTAMP_TIMESTAMP_MSB2_TIMESTAMP                      311
#define SX1302_REG_TIMESTAMP_TIMESTAMP_MSB1_TIMESTAMP                      312
#define SX1302_REG_TIMESTAMP_TIMESTAMP_LSB2_TIMESTAMP                      313
#define SX1302_REG_TIMESTAMP_TIMESTAMP_LSB1_TIMESTAMP                      314
#define SX1302_REG_TIMESTAMP_TIMESTAMP_SET3_TIMESTAMP                      315
#define SX1302_REG_TIMESTAMP_TIMESTAMP_SET2_TIMESTAMP                      316
#define SX1302_REG_TIMESTAMP_TIMESTAMP_SET1_TIMESTAMP                      317
#define SX1302_REG_TIMESTAMP_TIMESTAMP_SET0_TIMESTAMP                      318
#define SX1302_REG_TIMESTAMP_TIMESTAMP_IRQ_3_TIMESTAMP                     319
#define SX1302_REG_TIMESTAMP_TIMESTAMP_IRQ_2_TIMESTAMP                     320
#define SX1302_REG_TIMESTAMP_TIMESTAMP_IRQ_1_TIMESTAMP                     321
#define SX1302_REG_TIMESTAMP_TIMESTAMP_IRQ_0_TIMESTAMP                     322
#define SX1302_REG_TIMESTAMP_DUMMY_DUMMY                                   323
#define SX1302_REG_RX_TOP_FREQ_0_MSB_IF_FREQ_0                             324
#define SX1302_REG_RX_TOP_FREQ_0_LSB_IF_FREQ_0                             325
#define SX1302_REG_RX_TOP_FREQ_1_MSB_IF_FREQ_1                             326
#define SX1302_REG_RX_TOP_FREQ_1_LSB_IF_FREQ_1                             327
#define SX1302_REG_RX_TOP_FREQ_2_MSB_IF_FREQ_2                             328
#define SX1302_REG_RX_TOP_FREQ_2_LSB_IF_FREQ_2                             329
#define SX1302_REG_RX_TOP_FREQ_3_MSB_IF_FREQ_3                             330
#define SX1302_REG_RX_TOP_FREQ_3_LSB_IF_FREQ_3                             331
#define SX1302_REG_RX_TOP_FREQ_4_MSB_IF_FREQ_4                             332
#define SX1302_REG_RX_TOP_FREQ_4_LSB_IF_FREQ_4                             333
#define SX1302_REG_RX_TOP_FREQ_5_MSB_IF_FREQ_5                             334
#define SX1302_REG_RX_TOP_FREQ_5_LSB_IF_FREQ_5                             335
#define SX1302_REG_RX_TOP_FREQ_6_MSB_IF_FREQ_6                             336
#define SX1302_REG_RX_TOP_FREQ_6_LSB_IF_FREQ_6                             337
#define SX1302_REG_RX_TOP_FREQ_7_MSB_IF_FREQ_7                             338
#define SX1302_REG_RX_TOP_FREQ_7_LSB_IF_FREQ_7                             339
#define SX1302_REG_RX_TOP_RADIO_SELECT_RADIO_SELECT                        340
#define SX1302_REG_RX_TOP_RSSI_CONTROL_RSSI_FILTER_ALPHA                   341
#define SX1302_REG_RX_TOP_RSSI_CONTROL_SELECT_RSSI                         342
#define SX1302_REG_RX_TOP_RSSI_DEF_VALUE_CHAN_RSSI_DEF_VALUE               343
#define SX1302_REG_RX_TOP_CHANN_DAGC_CFG1_CHAN_DAGC_THRESHOLD_HIGH         344
#define SX1302_REG_RX_TOP_CHANN_DAGC_CFG2_CHAN_DAGC_THRESHOLD_LOW          345
#define SX1302_REG_RX_TOP_CHANN_DAGC_CFG3_CHAN_DAGC_MAX_ATTEN              346
#define SX1302_REG_RX_TOP_CHANN_DAGC_CFG3_CHAN_DAGC_MIN_ATTEN              347
#define SX1302_REG_RX_TOP_CHANN_DAGC_CFG4_CHAN_DAGC_STEP                   348
#define SX1302_REG_RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE                   349
#define SX1302_REG_RX_TOP_RSSI_VALUE_CHAN_RSSI                             350
#define SX1302_REG_RX_TOP_GAIN_CONTROL_CHAN_GAIN_VALID                     351
#define SX1302_REG_RX_TOP_GAIN_CONTROL_CHAN_GAIN                           352
#define SX1302_REG_RX_TOP_CLK_CONTROL_CHAN_CLK_EN                          353
#define SX1302_REG_RX_TOP_DUMMY0_DUMMY0                                    354
#define SX1302_REG_RX_TOP_CORR_CLOCK_ENABLE_CLK_EN                         355
#define SX1302_REG_RX_TOP_CORRELATOR_EN_CORR_EN                            356
#define SX1302_REG_RX_TOP_CORRELATOR_SF_EN_CORR_SF_EN                      357
#define SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ONLY_FIRST_DET_EDGE_ENABLE_ONLY_FIRST_DET_EDGE \
    358
#define SX1302_REG_RX_TOP_CORRELATOR_ENABLE_ACC_CLEAR_ENABLE_CORR_ACC_CLEAR 359
#define SX1302_REG_RX_TOP_SF5_CFG1_ACC_WIN_LEN                              360
#define SX1302_REG_RX_TOP_SF5_CFG1_ACC_PEAK_SUM_EN                          361
#define SX1302_REG_RX_TOP_SF5_CFG1_ACC_PEAK_POS_SEL                         362
#define SX1302_REG_RX_TOP_SF5_CFG1_ACC_COEFF                                363
#define SX1302_REG_RX_TOP_SF5_CFG1_ACC_AUTO_RESCALE                         364
#define SX1302_REG_RX_TOP_SF5_CFG1_ACC_2_SAME_PEAKS                         365
#define SX1302_REG_RX_TOP_SF5_CFG2_ACC_MIN2                                 366
#define SX1302_REG_RX_TOP_SF5_CFG2_ACC_PNR                                  367
#define SX1302_REG_RX_TOP_SF5_CFG3_MIN_SINGLE_PEAK                          368
#define SX1302_REG_RX_TOP_SF5_CFG4_MSP_PNR                                  369
#define SX1302_REG_RX_TOP_SF5_CFG5_MSP2_PNR                                 370
#define SX1302_REG_RX_TOP_SF5_CFG6_MSP_PEAK_NB                              371
#define SX1302_REG_RX_TOP_SF5_CFG6_MSP_CNT_MODE                             372
#define SX1302_REG_RX_TOP_SF5_CFG6_MSP_POS_SEL                              373
#define SX1302_REG_RX_TOP_SF5_CFG7_MSP2_PEAK_NB                             374
#define SX1302_REG_RX_TOP_SF5_CFG7_NOISE_COEFF                              375
#define SX1302_REG_RX_TOP_SF6_CFG1_ACC_WIN_LEN                              376
#define SX1302_REG_RX_TOP_SF6_CFG1_ACC_PEAK_SUM_EN                          377
#define SX1302_REG_RX_TOP_SF6_CFG1_ACC_PEAK_POS_SEL                         378
#define SX1302_REG_RX_TOP_SF6_CFG1_ACC_COEFF                                379
#define SX1302_REG_RX_TOP_SF6_CFG1_ACC_AUTO_RESCALE                         380
#define SX1302_REG_RX_TOP_SF6_CFG1_ACC_2_SAME_PEAKS                         381
#define SX1302_REG_RX_TOP_SF6_CFG2_ACC_MIN2                                 382
#define SX1302_REG_RX_TOP_SF6_CFG2_ACC_PNR                                  383
#define SX1302_REG_RX_TOP_SF6_CFG3_MIN_SINGLE_PEAK                          384
#define SX1302_REG_RX_TOP_SF6_CFG4_MSP_PNR                                  385
#define SX1302_REG_RX_TOP_SF6_CFG5_MSP2_PNR                                 386
#define SX1302_REG_RX_TOP_SF6_CFG6_MSP_PEAK_NB                              387
#define SX1302_REG_RX_TOP_SF6_CFG6_MSP_CNT_MODE                             388
#define SX1302_REG_RX_TOP_SF6_CFG6_MSP_POS_SEL                              389
#define SX1302_REG_RX_TOP_SF6_CFG7_MSP2_PEAK_NB                             390
#define SX1302_REG_RX_TOP_SF6_CFG7_NOISE_COEFF                              391
#define SX1302_REG_RX_TOP_SF7_CFG1_ACC_WIN_LEN                              392
#define SX1302_REG_RX_TOP_SF7_CFG1_ACC_PEAK_SUM_EN                          393
#define SX1302_REG_RX_TOP_SF7_CFG1_ACC_PEAK_POS_SEL                         394
#define SX1302_REG_RX_TOP_SF7_CFG1_ACC_COEFF                                395
#define SX1302_REG_RX_TOP_SF7_CFG1_ACC_AUTO_RESCALE                         396
#define SX1302_REG_RX_TOP_SF7_CFG1_ACC_2_SAME_PEAKS                         397
#define SX1302_REG_RX_TOP_SF7_CFG2_ACC_MIN2                                 398
#define SX1302_REG_RX_TOP_SF7_CFG2_ACC_PNR                                  399
#define SX1302_REG_RX_TOP_SF7_CFG3_MIN_SINGLE_PEAK                          400
#define SX1302_REG_RX_TOP_SF7_CFG4_MSP_PNR                                  401
#define SX1302_REG_RX_TOP_SF7_CFG5_MSP2_PNR                                 402
#define SX1302_REG_RX_TOP_SF7_CFG6_MSP_PEAK_NB                              403
#define SX1302_REG_RX_TOP_SF7_CFG6_MSP_CNT_MODE                             404
#define SX1302_REG_RX_TOP_SF7_CFG6_MSP_POS_SEL                              405
#define SX1302_REG_RX_TOP_SF7_CFG7_MSP2_PEAK_NB                             406
#define SX1302_REG_RX_TOP_SF7_CFG7_NOISE_COEFF                              407
#define SX1302_REG_RX_TOP_SF8_CFG1_ACC_WIN_LEN                              408
#define SX1302_REG_RX_TOP_SF8_CFG1_ACC_PEAK_SUM_EN                          409
#define SX1302_REG_RX_TOP_SF8_CFG1_ACC_PEAK_POS_SEL                         410
#define SX1302_REG_RX_TOP_SF8_CFG1_ACC_COEFF                                411
#define SX1302_REG_RX_TOP_SF8_CFG1_ACC_AUTO_RESCALE                         412
#define SX1302_REG_RX_TOP_SF8_CFG1_ACC_2_SAME_PEAKS                         413
#define SX1302_REG_RX_TOP_SF8_CFG2_ACC_MIN2                                 414
#define SX1302_REG_RX_TOP_SF8_CFG2_ACC_PNR                                  415
#define SX1302_REG_RX_TOP_SF8_CFG3_MIN_SINGLE_PEAK                          416
#define SX1302_REG_RX_TOP_SF8_CFG4_MSP_PNR                                  417
#define SX1302_REG_RX_TOP_SF8_CFG5_MSP2_PNR                                 418
#define SX1302_REG_RX_TOP_SF8_CFG6_MSP_PEAK_NB                              419
#define SX1302_REG_RX_TOP_SF8_CFG6_MSP_CNT_MODE                             420
#define SX1302_REG_RX_TOP_SF8_CFG6_MSP_POS_SEL                              421
#define SX1302_REG_RX_TOP_SF8_CFG7_MSP2_PEAK_NB                             422
#define SX1302_REG_RX_TOP_SF8_CFG7_NOISE_COEFF                              423
#define SX1302_REG_RX_TOP_SF9_CFG1_ACC_WIN_LEN                              424
#define SX1302_REG_RX_TOP_SF9_CFG1_ACC_PEAK_SUM_EN                          425
#define SX1302_REG_RX_TOP_SF9_CFG1_ACC_PEAK_POS_SEL                         426
#define SX1302_REG_RX_TOP_SF9_CFG1_ACC_COEFF                                427
#define SX1302_REG_RX_TOP_SF9_CFG1_ACC_AUTO_RESCALE                         428
#define SX1302_REG_RX_TOP_SF9_CFG1_ACC_2_SAME_PEAKS                         429
#define SX1302_REG_RX_TOP_SF9_CFG2_ACC_MIN2                                 430
#define SX1302_REG_RX_TOP_SF9_CFG2_ACC_PNR                                  431
#define SX1302_REG_RX_TOP_SF9_CFG3_MIN_SINGLE_PEAK                          432
#define SX1302_REG_RX_TOP_SF9_CFG4_MSP_PNR                                  433
#define SX1302_REG_RX_TOP_SF9_CFG5_MSP2_PNR                                 434
#define SX1302_REG_RX_TOP_SF9_CFG6_MSP_PEAK_NB                              435
#define SX1302_REG_RX_TOP_SF9_CFG6_MSP_CNT_MODE                             436
#define SX1302_REG_RX_TOP_SF9_CFG6_MSP_POS_SEL                              437
#define SX1302_REG_RX_TOP_SF9_CFG7_MSP2_PEAK_NB                             438
#define SX1302_REG_RX_TOP_SF9_CFG7_NOISE_COEFF                              439
#define SX1302_REG_RX_TOP_SF10_CFG1_ACC_WIN_LEN                             440
#define SX1302_REG_RX_TOP_SF10_CFG1_ACC_PEAK_SUM_EN                         441
#define SX1302_REG_RX_TOP_SF10_CFG1_ACC_PEAK_POS_SEL                        442
#define SX1302_REG_RX_TOP_SF10_CFG1_ACC_COEFF                               443
#define SX1302_REG_RX_TOP_SF10_CFG1_ACC_AUTO_RESCALE                        444
#define SX1302_REG_RX_TOP_SF10_CFG1_ACC_2_SAME_PEAKS                        445
#define SX1302_REG_RX_TOP_SF10_CFG2_ACC_MIN2                                446
#define SX1302_REG_RX_TOP_SF10_CFG2_ACC_PNR                                 447
#define SX1302_REG_RX_TOP_SF10_CFG3_MIN_SINGLE_PEAK                         448
#define SX1302_REG_RX_TOP_SF10_CFG4_MSP_PNR                                 449
#define SX1302_REG_RX_TOP_SF10_CFG5_MSP2_PNR                                450
#define SX1302_REG_RX_TOP_SF10_CFG6_MSP_PEAK_NB                             451
#define SX1302_REG_RX_TOP_SF10_CFG6_MSP_CNT_MODE                            452
#define SX1302_REG_RX_TOP_SF10_CFG6_MSP_POS_SEL                             453
#define SX1302_REG_RX_TOP_SF10_CFG7_MSP2_PEAK_NB                            454
#define SX1302_REG_RX_TOP_SF10_CFG7_NOISE_COEFF                             455
#define SX1302_REG_RX_TOP_SF11_CFG1_ACC_WIN_LEN                             456
#define SX1302_REG_RX_TOP_SF11_CFG1_ACC_PEAK_SUM_EN                         457
#define SX1302_REG_RX_TOP_SF11_CFG1_ACC_PEAK_POS_SEL                        458
#define SX1302_REG_RX_TOP_SF11_CFG1_ACC_COEFF                               459
#define SX1302_REG_RX_TOP_SF11_CFG1_ACC_AUTO_RESCALE                        460
#define SX1302_REG_RX_TOP_SF11_CFG1_ACC_2_SAME_PEAKS                        461
#define SX1302_REG_RX_TOP_SF11_CFG2_ACC_MIN2                                462
#define SX1302_REG_RX_TOP_SF11_CFG2_ACC_PNR                                 463
#define SX1302_REG_RX_TOP_SF11_CFG3_MIN_SINGLE_PEAK                         464
#define SX1302_REG_RX_TOP_SF11_CFG4_MSP_PNR                                 465
#define SX1302_REG_RX_TOP_SF11_CFG5_MSP2_PNR                                466
#define SX1302_REG_RX_TOP_SF11_CFG6_MSP_PEAK_NB                             467
#define SX1302_REG_RX_TOP_SF11_CFG6_MSP_CNT_MODE                            468
#define SX1302_REG_RX_TOP_SF11_CFG6_MSP_POS_SEL                             469
#define SX1302_REG_RX_TOP_SF11_CFG7_MSP2_PEAK_NB                            470
#define SX1302_REG_RX_TOP_SF11_CFG7_NOISE_COEFF                             471
#define SX1302_REG_RX_TOP_SF12_CFG1_ACC_WIN_LEN                             472
#define SX1302_REG_RX_TOP_SF12_CFG1_ACC_PEAK_SUM_EN                         473
#define SX1302_REG_RX_TOP_SF12_CFG1_ACC_PEAK_POS_SEL                        474
#define SX1302_REG_RX_TOP_SF12_CFG1_ACC_COEFF                               475
#define SX1302_REG_RX_TOP_SF12_CFG1_ACC_AUTO_RESCALE                        476
#define SX1302_REG_RX_TOP_SF12_CFG1_ACC_2_SAME_PEAKS                        477
#define SX1302_REG_RX_TOP_SF12_CFG2_ACC_MIN2                                478
#define SX1302_REG_RX_TOP_SF12_CFG2_ACC_PNR                                 479
#define SX1302_REG_RX_TOP_SF12_CFG3_MIN_SINGLE_PEAK                         480
#define SX1302_REG_RX_TOP_SF12_CFG4_MSP_PNR                                 481
#define SX1302_REG_RX_TOP_SF12_CFG5_MSP2_PNR                                482
#define SX1302_REG_RX_TOP_SF12_CFG6_MSP_PEAK_NB                             483
#define SX1302_REG_RX_TOP_SF12_CFG6_MSP_CNT_MODE                            484
#define SX1302_REG_RX_TOP_SF12_CFG6_MSP_POS_SEL                             485
#define SX1302_REG_RX_TOP_SF12_CFG7_MSP2_PEAK_NB                            486
#define SX1302_REG_RX_TOP_SF12_CFG7_NOISE_COEFF                             487
#define SX1302_REG_RX_TOP_DUMMY1_DUMMY1                                     488
#define SX1302_REG_RX_TOP_DC_NOTCH_CFG1_BW_START                            489
#define SX1302_REG_RX_TOP_DC_NOTCH_CFG1_AUTO_BW_RED                         490
#define SX1302_REG_RX_TOP_DC_NOTCH_CFG1_NO_FAST_START                       491
#define SX1302_REG_RX_TOP_DC_NOTCH_CFG1_BYPASS                              492
#define SX1302_REG_RX_TOP_DC_NOTCH_CFG1_ENABLE                              493
#define SX1302_REG_RX_TOP_DC_NOTCH_CFG2_BW_LOCKED                           494
#define SX1302_REG_RX_TOP_DC_NOTCH_CFG2_BW                                  495
#define SX1302_REG_RX_TOP_DC_NOTCH_CFG3_BW_RED                              496
#define SX1302_REG_RX_TOP_DC_NOTCH_CFG4_IIR_DCC_TIME                        497
#define SX1302_REG_RX_TOP_RX_DFE_FIR1_0_FIR1_COEFF_0                        498
#define SX1302_REG_RX_TOP_RX_DFE_FIR1_1_FIR1_COEFF_1                        499
#define SX1302_REG_RX_TOP_RX_DFE_FIR1_2_FIR1_COEFF_2                        500
#define SX1302_REG_RX_TOP_RX_DFE_FIR1_3_FIR1_COEFF_3                        501
#define SX1302_REG_RX_TOP_RX_DFE_FIR1_4_FIR1_COEFF_4                        502
#define SX1302_REG_RX_TOP_RX_DFE_FIR1_5_FIR1_COEFF_5                        503
#define SX1302_REG_RX_TOP_RX_DFE_FIR1_6_FIR1_COEFF_6                        504
#define SX1302_REG_RX_TOP_RX_DFE_FIR1_7_FIR1_COEFF_7                        505
#define SX1302_REG_RX_TOP_RX_DFE_FIR2_0_FIR2_COEFF_0                        506
#define SX1302_REG_RX_TOP_RX_DFE_FIR2_1_FIR2_COEFF_1                        507
#define SX1302_REG_RX_TOP_RX_DFE_FIR2_2_FIR2_COEFF_2                        508
#define SX1302_REG_RX_TOP_RX_DFE_FIR2_3_FIR2_COEFF_3                        509
#define SX1302_REG_RX_TOP_RX_DFE_FIR2_4_FIR2_COEFF_4                        510
#define SX1302_REG_RX_TOP_RX_DFE_FIR2_5_FIR2_COEFF_5                        511
#define SX1302_REG_RX_TOP_RX_DFE_FIR2_6_FIR2_COEFF_6                        512
#define SX1302_REG_RX_TOP_RX_DFE_FIR2_7_FIR2_COEFF_7                        513
#define SX1302_REG_RX_TOP_RX_DFE_AGC0_RADIO_GAIN_RED_SEL                    514
#define SX1302_REG_RX_TOP_RX_DFE_AGC0_RADIO_GAIN_RED_DB                     515
#define SX1302_REG_RX_TOP_RX_DFE_AGC1_DC_COMP_EN                            516
#define SX1302_REG_RX_TOP_RX_DFE_AGC1_FORCE_DEFAULT_FIR                     517
#define SX1302_REG_RX_TOP_RX_DFE_AGC1_RSSI_EARLY_LATCH                      518
#define SX1302_REG_RX_TOP_RX_DFE_AGC1_FREEZE_ON_SYNC                        519
#define SX1302_REG_RX_TOP_RX_DFE_AGC2_DAGC_IN_COMP                          520
#define SX1302_REG_RX_TOP_RX_DFE_AGC2_DAGC_FIR_HYST                         521
#define SX1302_REG_RX_TOP_RX_DFE_AGC2_RSSI_MAX_SAMPLE                       522
#define SX1302_REG_RX_TOP_RX_DFE_AGC2_RSSI_MIN_SAMPLE                       523
#define SX1302_REG_RX_TOP_RX_DFE_GAIN0_DAGC_FIR_FAST                        524
#define SX1302_REG_RX_TOP_RX_DFE_GAIN0_FORCE_GAIN_FIR                       525
#define SX1302_REG_RX_TOP_RX_DFE_GAIN0_GAIN_FIR1                            526
#define SX1302_REG_RX_TOP_RX_DFE_GAIN0_GAIN_FIR2                            527
#define SX1302_REG_RX_TOP_DAGC_CFG_TARGET_LVL                               528
#define SX1302_REG_RX_TOP_DAGC_CFG_GAIN_INCR_STEP                           529
#define SX1302_REG_RX_TOP_DAGC_CFG_GAIN_DROP_COMP                           530
#define SX1302_REG_RX_TOP_DAGC_CFG_COMB_FILTER_EN                           531
#define SX1302_REG_RX_TOP_DAGC_CFG_NO_FREEZE_START                          532
#define SX1302_REG_RX_TOP_DAGC_CFG_FREEZE_ON_SYNC                           533
#define SX1302_REG_RX_TOP_DAGC_CNT0_SAMPLE                                  534
#define SX1302_REG_RX_TOP_DAGC_CNT1_THR_M6                                  535
#define SX1302_REG_RX_TOP_DAGC_CNT2_THR_M12                                 536
#define SX1302_REG_RX_TOP_DAGC_CNT3_THR_M18                                 537
#define SX1302_REG_RX_TOP_DAGC_CNT4_GAIN                                    538
#define SX1302_REG_RX_TOP_DAGC_CNT4_FORCE_GAIN                              539
#define SX1302_REG_RX_TOP_TXRX_CFG1_PPM_OFFSET_HDR_CTRL                     540
#define SX1302_REG_RX_TOP_TXRX_CFG1_PPM_OFFSET                              541
#define SX1302_REG_RX_TOP_TXRX_CFG1_MODEM_EN                                542
#define SX1302_REG_RX_TOP_TXRX_CFG1_CODING_RATE                             543
#define SX1302_REG_RX_TOP_TXRX_CFG2_MODEM_START                             544
#define SX1302_REG_RX_TOP_TXRX_CFG2_CADRXTX                                 545
#define SX1302_REG_RX_TOP_TXRX_CFG2_IMPLICIT_HEADER                         546
#define SX1302_REG_RX_TOP_TXRX_CFG2_CRC_EN                                  547
#define SX1302_REG_RX_TOP_TXRX_CFG3_PAYLOAD_LENGTH                          548
#define SX1302_REG_RX_TOP_TXRX_CFG4_INT_STEP_ORIDE_EN                       549
#define SX1302_REG_RX_TOP_TXRX_CFG4_INT_STEP_ORIDE                          550
#define SX1302_REG_RX_TOP_TXRX_CFG5_HEADER_DIFF_MODE                        551
#define SX1302_REG_RX_TOP_TXRX_CFG5_ZERO_PAD                                552
#define SX1302_REG_RX_TOP_TXRX_CFG6_PREAMBLE_SYMB_NB                        553
#define SX1302_REG_RX_TOP_TXRX_CFG7_PREAMBLE_SYMB_NB                        554
#define SX1302_REG_RX_TOP_TXRX_CFG8_AUTO_ACK_INT_DELAY                      555
#define SX1302_REG_RX_TOP_TXRX_CFG8_AUTO_ACK_RX                             556
#define SX1302_REG_RX_TOP_TXRX_CFG8_AUTO_ACK_TX                             557
#define SX1302_REG_RX_TOP_TXRX_CFG8_POST_PREAMBLE_GAP_LONG                  558
#define SX1302_REG_RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF12                      559
#define SX1302_REG_RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF11                      560
#define SX1302_REG_RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF10                      561
#define SX1302_REG_RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF9                       562
#define SX1302_REG_RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF8                       563
#define SX1302_REG_RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF7                       564
#define SX1302_REG_RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF6                       565
#define SX1302_REG_RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF5                       566
#define SX1302_REG_RX_TOP_RX_CFG0_DFT_PEAK_EN                               567
#define SX1302_REG_RX_TOP_RX_CFG0_CHIRP_INVERT                              568
#define SX1302_REG_RX_TOP_RX_CFG0_SWAP_IQ                                   569
#define SX1302_REG_RX_TOP_RX_CFG0_CONTINUOUS                                570
#define SX1302_REG_RX_TOP_RX_CFG1_DETECT_TIMEOUT                            571
#define SX1302_REG_RX_TOP_RX_CFG2_CLK_EN_RESYNC_DIN                         572
#define SX1302_REG_RX_TOP_RX_CFG2_LLR_SCALE                                 573
#define SX1302_REG_RX_TOP_FRAME_SYNCH0_SF5_PEAK1_POS_SF5                    574
#define SX1302_REG_RX_TOP_FRAME_SYNCH1_SF5_PEAK2_POS_SF5                    575
#define SX1302_REG_RX_TOP_FRAME_SYNCH0_SF6_PEAK1_POS_SF6                    576
#define SX1302_REG_RX_TOP_FRAME_SYNCH1_SF6_PEAK2_POS_SF6                    577
#define SX1302_REG_RX_TOP_FRAME_SYNCH0_SF7TO12_PEAK1_POS_SF7TO12            578
#define SX1302_REG_RX_TOP_FRAME_SYNCH1_SF7TO12_PEAK2_POS_SF7TO12            579
#define SX1302_REG_RX_TOP_FRAME_SYNCH2_FINETIME_ON_LAST                     580
#define SX1302_REG_RX_TOP_FRAME_SYNCH2_AUTO_SCALE                           581
#define SX1302_REG_RX_TOP_FRAME_SYNCH2_DROP_ON_SYNCH                        582
#define SX1302_REG_RX_TOP_FRAME_SYNCH2_GAIN                                 583
#define SX1302_REG_RX_TOP_FRAME_SYNCH2_TIMEOUT_OPT                          584
#define SX1302_REG_RX_TOP_FINE_TIMING_A_0_GAIN_P_HDR_RED                    585
#define SX1302_REG_RX_TOP_FINE_TIMING_A_0_ROUNDING                          586
#define SX1302_REG_RX_TOP_FINE_TIMING_A_0_POS_LIMIT                         587
#define SX1302_REG_RX_TOP_FINE_TIMING_A_0_SUM_SIZE                          588
#define SX1302_REG_RX_TOP_FINE_TIMING_A_0_MODE                              589
#define SX1302_REG_RX_TOP_FINE_TIMING_A_1_GAIN_P_AUTO                       590
#define SX1302_REG_RX_TOP_FINE_TIMING_A_1_GAIN_P_PAYLOAD                    591
#define SX1302_REG_RX_TOP_FINE_TIMING_A_1_GAIN_P_PREAMB                     592
#define SX1302_REG_RX_TOP_FINE_TIMING_A_2_GAIN_I_AUTO                       593
#define SX1302_REG_RX_TOP_FINE_TIMING_A_2_GAIN_I_PAYLOAD                    594
#define SX1302_REG_RX_TOP_FINE_TIMING_A_2_GAIN_I_PREAMB                     595
#define SX1302_REG_RX_TOP_FINE_TIMING_A_3_FINESYNCH_SUM                     596
#define SX1302_REG_RX_TOP_FINE_TIMING_A_3_FINESYNCH_GAIN                    597
#define SX1302_REG_RX_TOP_FINE_TIMING_A_4_GAIN_I_EN_SF8                     598
#define SX1302_REG_RX_TOP_FINE_TIMING_A_4_GAIN_I_EN_SF7                     599
#define SX1302_REG_RX_TOP_FINE_TIMING_A_4_GAIN_I_EN_SF6                     600
#define SX1302_REG_RX_TOP_FINE_TIMING_A_4_GAIN_I_EN_SF5                     601
#define SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF12                    602
#define SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF11                    603
#define SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF10                    604
#define SX1302_REG_RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF9                     605
#define SX1302_REG_RX_TOP_FINE_TIMING_A_6_GAIN_P_PREAMB_SF12                606
#define SX1302_REG_RX_TOP_FINE_TIMING_A_6_GAIN_P_PREAMB_SF5_6               607
#define SX1302_REG_RX_TOP_FINE_TIMING_7_GAIN_I_AUTO_MAX                     608
#define SX1302_REG_RX_TOP_FINE_TIMING_7_GAIN_P_AUTO_MAX                     609
#define SX1302_REG_RX_TOP_FINE_TIMING_B_0_GAIN_P_HDR_RED                    610
#define SX1302_REG_RX_TOP_FINE_TIMING_B_0_ROUNDING                          611
#define SX1302_REG_RX_TOP_FINE_TIMING_B_0_POS_LIMIT                         612
#define SX1302_REG_RX_TOP_FINE_TIMING_B_0_SUM_SIZE                          613
#define SX1302_REG_RX_TOP_FINE_TIMING_B_0_MODE                              614
#define SX1302_REG_RX_TOP_FINE_TIMING_B_1_GAIN_P_AUTO                       615
#define SX1302_REG_RX_TOP_FINE_TIMING_B_1_GAIN_P_PAYLOAD                    616
#define SX1302_REG_RX_TOP_FINE_TIMING_B_1_GAIN_P_PREAMB                     617
#define SX1302_REG_RX_TOP_FINE_TIMING_B_2_GAIN_I_AUTO                       618
#define SX1302_REG_RX_TOP_FINE_TIMING_B_2_GAIN_I_PAYLOAD                    619
#define SX1302_REG_RX_TOP_FINE_TIMING_B_2_GAIN_I_PREAMB                     620
#define SX1302_REG_RX_TOP_FINE_TIMING_B_3_FINESYNCH_SUM                     621
#define SX1302_REG_RX_TOP_FINE_TIMING_B_3_FINESYNCH_GAIN                    622
#define SX1302_REG_RX_TOP_FINE_TIMING_B_4_GAIN_I_EN_SF8                     623
#define SX1302_REG_RX_TOP_FINE_TIMING_B_4_GAIN_I_EN_SF7                     624
#define SX1302_REG_RX_TOP_FINE_TIMING_B_4_GAIN_I_EN_SF6                     625
#define SX1302_REG_RX_TOP_FINE_TIMING_B_4_GAIN_I_EN_SF5                     626
#define SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF12                    627
#define SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF11                    628
#define SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF10                    629
#define SX1302_REG_RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF9                     630
#define SX1302_REG_RX_TOP_FINE_TIMING_B_6_GAIN_P_PREAMB_SF12                631
#define SX1302_REG_RX_TOP_FINE_TIMING_B_6_GAIN_P_PREAMB_SF5_6               632
#define SX1302_REG_RX_TOP_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT             633
#define SX1302_REG_RX_TOP_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT             634
#define SX1302_REG_RX_TOP_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP              635
#define SX1302_REG_RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FREQ_DELTA      636
#define SX1302_REG_RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FINE_DELTA      637
#define SX1302_REG_RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FREQ_ERROR      638
#define SX1302_REG_RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB       639
#define SX1302_REG_RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_OFFSET     640
#define SX1302_REG_RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_DETECT          641
#define SX1302_REG_RX_TOP_FREQ_TO_TIME4_FREQ_TO_TIME_INVERT_RNG             642
#define SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF8                  643
#define SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF7                  644
#define SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF6                  645
#define SX1302_REG_RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF5                  646
#define SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF12                 647
#define SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF11                 648
#define SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF10                 649
#define SX1302_REG_RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF9                  650
#define SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF8                  651
#define SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF7                  652
#define SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF6                  653
#define SX1302_REG_RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF5                  654
#define SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF12                 655
#define SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF11                 656
#define SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF10                 657
#define SX1302_REG_RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF9                  658
#define SX1302_REG_RX_TOP_FREQ_TRACK2_FREQ_TRACK_FINE                       659
#define SX1302_REG_RX_TOP_FREQ_TRACK2_FREQ_TRACK_HDR_SKIP                   660
#define SX1302_REG_RX_TOP_FREQ_TRACK3_FREQ_SYNCH_GAIN                       661
#define SX1302_REG_RX_TOP_FREQ_TRACK3_FREQ_TRACK_AUTO_THR                   662
#define SX1302_REG_RX_TOP_FREQ_TRACK4_SNR_MIN_WINDOW                        663
#define SX1302_REG_RX_TOP_FREQ_TRACK4_GAIN_AUTO_SNR_MIN                     664
#define SX1302_REG_RX_TOP_FREQ_TRACK4_FREQ_SYNCH_THR                        665
#define SX1302_REG_RX_TOP_DETECT_MSP0_MSP_PNR                               666
#define SX1302_REG_RX_TOP_DETECT_MSP1_MSP2_PNR                              667
#define SX1302_REG_RX_TOP_DETECT_MSP2_MSP2_PEAK_NB                          668
#define SX1302_REG_RX_TOP_DETECT_MSP2_MSP_PEAK_NB                           669
#define SX1302_REG_RX_TOP_DETECT_MSP3_ACC_MIN2                              670
#define SX1302_REG_RX_TOP_DETECT_MSP3_ACC_WIN_LEN                           671
#define SX1302_REG_RX_TOP_DETECT_MSP3_MSP_POS_SEL                           672
#define SX1302_REG_RX_TOP_DETECT_MSP3_MSP_CNT_MODE                          673
#define SX1302_REG_RX_TOP_DETECT_ACC1_USE_GAIN_SYMB                         674
#define SX1302_REG_RX_TOP_DETECT_ACC1_ACC_PNR                               675
#define SX1302_REG_RX_TOP_DETECT_ACC2_NOISE_COEFF                           676
#define SX1302_REG_RX_TOP_DETECT_ACC2_ACC_COEFF                             677
#define SX1302_REG_RX_TOP_DETECT_ACC2_ACC_2_SAME_PEAKS                      678
#define SX1302_REG_RX_TOP_DETECT_ACC2_ACC_AUTO_RESCALE                      679
#define SX1302_REG_RX_TOP_DETECT_ACC2_ACC_PEAK_POS_SEL                      680
#define SX1302_REG_RX_TOP_DETECT_ACC2_ACC_PEAK_SUM_EN                       681
#define SX1302_REG_RX_TOP_DETECT_ACC3_MIN_SINGLE_PEAK                       682
#define SX1302_REG_RX_TOP_TIMESTAMP_SEL_SNR_MIN                             683
#define SX1302_REG_RX_TOP_TIMESTAMP_ENABLE                                  684
#define SX1302_REG_RX_TOP_TIMESTAMP_NB_SYMB                                 685
#define SX1302_REG_RX_TOP_MODEM_BUSY_MSB_RX_MODEM_BUSY                      686
#define SX1302_REG_RX_TOP_MODEM_BUSY_LSB_RX_MODEM_BUSY                      687
#define SX1302_REG_RX_TOP_MODEM_STATE_RX_MODEM_STS_SPARE                    688
#define SX1302_REG_RX_TOP_MODEM_STATE_RX_MODEM_STATE                        689
#define SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_MSB_PEAK_POS_FINE_GAIN_H         690
#define SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_MSB_PEAK_POS_FINE_GAIN_L         691
#define SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_MSB_PEAK_POS_FINE_SIGN           692
#define SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_MSB_MODEM_SYNC_DELTA             693
#define SX1302_REG_RX_TOP_MODEM_SYNC_DELTA_LSB_MODEM_SYNC_DELTA             694
#define SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF8                  695
#define SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF7                  696
#define SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF6                  697
#define SX1302_REG_RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF5                  698
#define SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF12                 699
#define SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF11                 700
#define SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF10                 701
#define SX1302_REG_RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF9                  702
#define SX1302_REG_RX_TOP_MODEM_CLOCK_GATE_OVERRIDE_3_CLK_OVERRIDE          703
#define SX1302_REG_RX_TOP_MODEM_CLOCK_GATE_OVERRIDE_2_CLK_OVERRIDE          704
#define SX1302_REG_RX_TOP_MODEM_CLOCK_GATE_OVERRIDE_1_CLK_OVERRIDE          705
#define SX1302_REG_RX_TOP_MODEM_CLOCK_GATE_OVERRIDE_0_CLK_OVERRIDE          706
#define SX1302_REG_RX_TOP_DUMMY2_DUMMY2                                     707
#define SX1302_REG_RX_TOP_RX_BUFFER_DEBUG_MODE                              708
#define SX1302_REG_RX_TOP_RX_BUFFER_DIRECT_RAM_IF                           709
#define SX1302_REG_RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP                        710
#define SX1302_REG_RX_TOP_RX_BUFFER_STORE_HEADER_ERR_META                   711
#define SX1302_REG_RX_TOP_RX_BUFFER_STORE_SYNC_FAIL_META                    712
#define SX1302_REG_RX_TOP_RX_BUFFER_TIMESTAMP_CFG_MAX_TS_METRICS            713
#define SX1302_REG_RX_TOP_RX_BUFFER_IRQ_CTRL_MSB_RX_BUFFER_IRQ_THRESHOLD    714
#define SX1302_REG_RX_TOP_RX_BUFFER_IRQ_CTRL_LSB_RX_BUFFER_IRQ_THRESHOLD    715
#define SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_READ_MSB_LAST_ADDR_READ       716
#define SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_READ_LSB_LAST_ADDR_READ       717
#define SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_WRITE_MSB_LAST_ADDR_WRITE     718
#define SX1302_REG_RX_TOP_RX_BUFFER_LAST_ADDR_WRITE_LSB_LAST_ADDR_WRITE     719
#define SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES         720
#define SX1302_REG_RX_TOP_RX_BUFFER_NB_BYTES_LSB_RX_BUFFER_NB_BYTES         721
#define SX1302_REG_RX_TOP_MULTI_SF_SYNC_ERR_PKT_CNT_MULTI_SF_SYNC_ERR_PKTS  722
#define SX1302_REG_RX_TOP_MULTI_SF_PLD_ERR_PKT_CNT_MULTI_SF_PLD_ERR_PKTS    723
#define SX1302_REG_RX_TOP_MULTI_SF_GOOD_PKT_CNT_MULTI_SF_GOOD_PKTS          724
#define SX1302_REG_RX_TOP_SERV_MODEM_SYNC_ERR_PKT_CNT_SERV_MODEM_SYNC_ERR_PKTS \
    725
#define SX1302_REG_RX_TOP_SERV_MODEM_PLD_ERR_PKT_CNT_SERV_MODEM_PLD_ERR_PKTS 726
#define SX1302_REG_RX_TOP_SERV_MODEM_GOOD_PKT_CNT_SERV_MODEM_GOOD_PKTS       727
#define SX1302_REG_RX_TOP_GFSK_MODEM_SYNC_ERR_PKT_CNT_GFSK_MODEM_SYNC_ERR_PKTS \
    728
#define SX1302_REG_RX_TOP_GFSK_MODEM_PLD_ERR_PKT_CNT_GFSK_MODEM_PLD_ERR_PKTS 729
#define SX1302_REG_RX_TOP_GFSK_MODEM_GOOD_PKT_CNT_GFSK_MODEM_GOOD_PKTS       730
#define SX1302_REG_RX_TOP_BAD_MODEM_ID_WRITE_0_BAD_MODEM_ID_WRITE            731
#define SX1302_REG_RX_TOP_BAD_MODEM_ID_WRITE_1_BAD_MODEM_ID_WRITE            732
#define SX1302_REG_RX_TOP_BAD_MODEM_ID_WRITE_2_BAD_MODEM_ID_WRITE            733
#define SX1302_REG_RX_TOP_BAD_MODEM_ID_READ_0_BAD_MODEM_ID_READ              734
#define SX1302_REG_RX_TOP_BAD_MODEM_ID_READ_1_BAD_MODEM_ID_READ              735
#define SX1302_REG_RX_TOP_BAD_MODEM_ID_READ_2_BAD_MODEM_ID_READ              736
#define SX1302_REG_RX_TOP_CLOCK_GATE_OVERRIDE_0_CLK_OVERRIDE                 737
#define SX1302_REG_RX_TOP_SAMPLE_4_MSPS_LATCHED_125K_SAMPLE_4_MSPS_LATCHED_125K \
    738
#define SX1302_REG_RX_TOP_DUMMY3_DUMMY3                                 739
#define SX1302_REG_ARB_MCU_CTRL_CLK_EN                                  740
#define SX1302_REG_ARB_MCU_CTRL_RADIO_RST                               741
#define SX1302_REG_ARB_MCU_CTRL_FORCE_HOST_FE_CTRL                      742
#define SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR                               743
#define SX1302_REG_ARB_MCU_CTRL_HOST_PROG                               744
#define SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR                            745
#define SX1302_REG_ARB_MCU_MCU_ARB_STATUS_MCU_ARB_STATUS                746
#define SX1302_REG_ARB_MCU_UART_CFG_MSBF                                747
#define SX1302_REG_ARB_MCU_UART_CFG_PAR_EN                              748
#define SX1302_REG_ARB_MCU_UART_CFG_PAR_MODE                            749
#define SX1302_REG_ARB_MCU_UART_CFG_START_LEN                           750
#define SX1302_REG_ARB_MCU_UART_CFG_STOP_LEN                            751
#define SX1302_REG_ARB_MCU_UART_CFG_WORD_LEN                            752
#define SX1302_REG_ARB_MCU_UART_CFG2_BIT_RATE                           753
#define SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_0_ARB_DEBUG_CFG_0              754
#define SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_1_ARB_DEBUG_CFG_1              755
#define SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_2_ARB_DEBUG_CFG_2              756
#define SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_3_ARB_DEBUG_CFG_3              757
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_0_ARB_DEBUG_STS_0              758
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_1_ARB_DEBUG_STS_1              759
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_2_ARB_DEBUG_STS_2              760
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_3_ARB_DEBUG_STS_3              761
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_4_ARB_DEBUG_STS_4              762
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_5_ARB_DEBUG_STS_5              763
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_6_ARB_DEBUG_STS_6              764
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_7_ARB_DEBUG_STS_7              765
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_8_ARB_DEBUG_STS_8              766
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_9_ARB_DEBUG_STS_9              767
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_10_ARB_DEBUG_STS_10            768
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_11_ARB_DEBUG_STS_11            769
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_12_ARB_DEBUG_STS_12            770
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_13_ARB_DEBUG_STS_13            771
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_14_ARB_DEBUG_STS_14            772
#define SX1302_REG_ARB_MCU_ARB_DEBUG_STS_15_ARB_DEBUG_STS_15            773
#define SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_1_OFFSET      774
#define SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_0_OFFSET      775
#define SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_3_OFFSET      776
#define SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_2_OFFSET      777
#define SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_5_OFFSET      778
#define SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_4_OFFSET      779
#define SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_7_OFFSET      780
#define SX1302_REG_ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_6_OFFSET      781
#define SX1302_REG_ARB_MCU_DUMMY_DUMMY3                                 782
#define SX1302_REG_RADIO_FE_GLBL_CTRL_DECIM_B_CLR                       783
#define SX1302_REG_RADIO_FE_GLBL_CTRL_DECIM_A_CLR                       784
#define SX1302_REG_RADIO_FE_CTRL0_RADIO_A_DC_NOTCH_EN                   785
#define SX1302_REG_RADIO_FE_CTRL0_RADIO_A_FORCE_HOST_FILTER_GAIN        786
#define SX1302_REG_RADIO_FE_CTRL0_RADIO_A_HOST_FILTER_GAIN              787
#define SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_A_RSSI_DB_DEFAULT_VALUE   788
#define SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_A_RSSI_DEC_DEFAULT_VALUE 789
#define SX1302_REG_RADIO_FE_RSSI_DEC_RD_RADIO_A_RSSI_DEC_OUT            790
#define SX1302_REG_RADIO_FE_RSSI_BB_RD_RADIO_A_RSSI_BB_OUT              791
#define SX1302_REG_RADIO_FE_DEC_FILTER_RD_RADIO_A_DEC_FILTER_GAIN       792
#define SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_A_RSSI_BB_FILTER_ALPHA \
    793
#define SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_A_RSSI_DEC_FILTER_ALPHA \
    794
#define SX1302_REG_RADIO_FE_IQ_COMP_AMP_COEFF_RADIO_A_AMP_COEFF         795
#define SX1302_REG_RADIO_FE_IQ_COMP_PHI_COEFF_RADIO_A_PHI_COEFF         796
#define SX1302_REG_RADIO_FE_RADIO_DIO_TEST_MODE_RADIO_A_DIO_TEST_MODE   797
#define SX1302_REG_RADIO_FE_RADIO_DIO_TEST_DIR_RADIO_A_DIO_TEST_DIR     798
#define SX1302_REG_RADIO_FE_RADIO_DIO_DIR_RADIO_A_DIO_DIR               799
#define SX1302_REG_RADIO_FE_CTRL0_RADIO_B_DC_NOTCH_EN                   800
#define SX1302_REG_RADIO_FE_CTRL0_RADIO_B_FORCE_HOST_FILTER_GAIN        801
#define SX1302_REG_RADIO_FE_CTRL0_RADIO_B_HOST_FILTER_GAIN              802
#define SX1302_REG_RADIO_FE_RSSI_DB_DEF_RADIO_B_RSSI_DB_DEFAULT_VALUE   803
#define SX1302_REG_RADIO_FE_RSSI_DEC_DEF_RADIO_B_RSSI_DEC_DEFAULT_VALUE 804
#define SX1302_REG_RADIO_FE_RSSI_DEC_RD_RADIO_B_RSSI_DEC_OUT            805
#define SX1302_REG_RADIO_FE_RSSI_BB_RD_RADIO_B_RSSI_BB_OUT              806
#define SX1302_REG_RADIO_FE_DEC_FILTER_RD_RADIO_B_DEC_FILTER_GAIN       807
#define SX1302_REG_RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_B_RSSI_BB_FILTER_ALPHA \
    808
#define SX1302_REG_RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_B_RSSI_DEC_FILTER_ALPHA \
    809
#define SX1302_REG_RADIO_FE_IQ_COMP_AMP_COEFF_RADIO_B_AMP_COEFF            810
#define SX1302_REG_RADIO_FE_IQ_COMP_PHI_COEFF_RADIO_B_PHI_COEFF            811
#define SX1302_REG_RADIO_FE_RADIO_DIO_TEST_MODE_RADIO_B_DIO_TEST_MODE      812
#define SX1302_REG_RADIO_FE_RADIO_DIO_TEST_DIR_RADIO_B_DIO_TEST_DIR        813
#define SX1302_REG_RADIO_FE_RADIO_DIO_DIR_RADIO_B_DIO_DIR                  814
#define SX1302_REG_RADIO_FE_SIG_ANA_CFG_VALID                              815
#define SX1302_REG_RADIO_FE_SIG_ANA_CFG_BUSY                               816
#define SX1302_REG_RADIO_FE_SIG_ANA_CFG_DURATION                           817
#define SX1302_REG_RADIO_FE_SIG_ANA_CFG_FORCE_HAL_CTRL                     818
#define SX1302_REG_RADIO_FE_SIG_ANA_CFG_START                              819
#define SX1302_REG_RADIO_FE_SIG_ANA_CFG_RADIO_SEL                          820
#define SX1302_REG_RADIO_FE_SIG_ANA_CFG_EN                                 821
#define SX1302_REG_RADIO_FE_SIG_ANA_FREQ_FREQ                              822
#define SX1302_REG_RADIO_FE_SIG_ANA_ABS_MSB_CORR_ABS_OUT                   823
#define SX1302_REG_RADIO_FE_SIG_ANA_ABS_LSB_CORR_ABS_OUT                   824
#define SX1302_REG_RADIO_FE_DUMMY_DUMMY                                    825
#define SX1302_REG_OTP_BYTE_ADDR_ADDR                                      826
#define SX1302_REG_OTP_RD_DATA_RD_DATA                                     827
#define SX1302_REG_OTP_STATUS_CHECKSUM_STATUS                              828
#define SX1302_REG_OTP_STATUS_FSM_READY                                    829
#define SX1302_REG_OTP_CFG_ACCESS_MODE                                     830
#define SX1302_REG_OTP_BIT_POS_POS                                         831
#define SX1302_REG_OTP_PIN_CTRL_0_TM                                       832
#define SX1302_REG_OTP_PIN_CTRL_0_STROBE                                   833
#define SX1302_REG_OTP_PIN_CTRL_0_PGENB                                    834
#define SX1302_REG_OTP_PIN_CTRL_0_LOAD                                     835
#define SX1302_REG_OTP_PIN_CTRL_0_CSB                                      836
#define SX1302_REG_OTP_PIN_CTRL_1_FSCK                                     837
#define SX1302_REG_OTP_PIN_CTRL_1_FSI                                      838
#define SX1302_REG_OTP_PIN_CTRL_1_FRST                                     839
#define SX1302_REG_OTP_PIN_STATUS_FSO                                      840
#define SX1302_REG_OTP_MODEM_EN_0_MODEM_EN                                 841
#define SX1302_REG_OTP_MODEM_EN_1_MODEM_EN                                 842
#define SX1302_REG_OTP_MODEM_SF_EN_SF_EN                                   843
#define SX1302_REG_OTP_TIMESTAMP_EN_TIMESTAMP_EN                           844
#define SX1302_REG_OTP_DUMMY_DUMMY                                         845
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_MSB_IF_FREQ_0 846
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_LSB_IF_FREQ_0 847
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_RADIO_SEL_RADIO_SELECT \
    848
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_BW_START           849
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_AUTO_BW_RED        850
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_NO_FAST_START      851
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_BYPASS             852
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_ENABLE             853
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG2_BW_LOCKED          854
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG2_BW                 855
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG3_BW_RED             856
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG4_IIR_DCC_TIME       857
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_0_FIR1_COEFF_0       858
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_1_FIR1_COEFF_1       859
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_2_FIR1_COEFF_2       860
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_3_FIR1_COEFF_3       861
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_4_FIR1_COEFF_4       862
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_5_FIR1_COEFF_5       863
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_6_FIR1_COEFF_6       864
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_7_FIR1_COEFF_7       865
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_0_FIR2_COEFF_0       866
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_1_FIR2_COEFF_1       867
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_2_FIR2_COEFF_2       868
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_3_FIR2_COEFF_3       869
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_4_FIR2_COEFF_4       870
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_5_FIR2_COEFF_5       871
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_6_FIR2_COEFF_6       872
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_7_FIR2_COEFF_7       873
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC0_RADIO_GAIN_RED_SEL   874
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC0_RADIO_GAIN_RED_DB    875
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_DC_COMP_EN           876
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_FORCE_DEFAULT_FIR    877
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_RSSI_EARLY_LATCH     878
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_FREEZE_ON_SYNC       879
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_DAGC_IN_COMP         880
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_DAGC_FIR_HYST        881
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_RSSI_MAX_SAMPLE      882
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_RSSI_MIN_SAMPLE      883
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_GAIN0_DAGC_FIR_FAST       884
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_GAIN0_FORCE_GAIN_FIR      885
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_GAIN0_GAIN_FIR1           886
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_DFE_GAIN0_GAIN_FIR2           887
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_TARGET_LVL              888
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_GAIN_INCR_STEP          889
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_GAIN_DROP_COMP          890
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_COMB_FILTER_EN          891
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_NO_FREEZE_START         892
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_FREEZE_ON_SYNC          893
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CNT0_SAMPLE                 894
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CNT1_THR_M6                 895
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CNT2_THR_M12                896
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CNT3_THR_M18                897
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CNT4_GAIN                   898
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DAGC_CNT4_FORCE_GAIN             899
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_BW               900
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_SF               901
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_PPM_OFFSET_HDR_CTRL    902
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_PPM_OFFSET             903
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_MODEM_EN               904
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_CODING_RATE            905
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN          906
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_MODEM_START            907
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CADRXTX                908
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_IMPLICIT_HEADER        909
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CRC_EN                 910
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG3_PAYLOAD_LENGTH         911
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG4_INT_STEP_ORIDE_EN      912
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG4_INT_STEP_ORIDE         913
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG5_HEADER_DIFF_MODE       914
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG5_ZERO_PAD               915
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG6_PREAMBLE_SYMB_NB       916
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG7_PREAMBLE_SYMB_NB       917
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG8_AUTO_ACK_INT_DELAY     918
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG8_AUTO_ACK_RX            919
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG8_AUTO_ACK_TX            920
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TXRX_CFG8_POST_PREAMBLE_GAP_LONG 921
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG0_DFT_PEAK_EN              922
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG0_CHIRP_INVERT             923
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG0_SWAP_IQ                  924
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG0_CONTINUOUS               925
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG1_DETECT_TIMEOUT           926
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG2_AUTO_ACK_RANGE           927
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG2_AUTO_ACK_DELAY           928
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG3_RESTART_ON_HDR_ERR       929
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG3_CLK_EN_RESYNC_DIN        930
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_RX_CFG3_LLR_SCALE                931
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS           932
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS           933
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH2_FINETIME_ON_LAST    934
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH2_AUTO_SCALE          935
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH2_DROP_ON_SYNCH       936
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH2_GAIN                937
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH2_TIMEOUT_OPT         938
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING0_GAIN_P_HDR_RED      939
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING0_ROUNDING            940
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING0_POS_LIMIT           941
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING0_SUM_SIZE            942
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING0_MODE                943
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_AUTO         944
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PAYLOAD      945
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB       946
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN           947
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_PAYLOAD      948
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_PREAMB       949
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING3_FINESYNCH_SUM       950
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING3_FINESYNCH_GAIN      951
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING3_GAIN_I_AUTO         952
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING4_GAIN_I_AUTO_MAX     953
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FINE_TIMING4_GAIN_P_AUTO_MAX     954
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT \
    955
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT \
    956
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP \
    957
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FREQ_DELTA \
    958
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FINE_DELTA \
    959
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FREQ_ERROR \
    960
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB \
    961
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_OFFSET \
    962
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_DETECT \
    963
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME4_FREQ_TO_TIME_INVERT_RNG \
    964
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK0_FREQ_TRACK_FINE     965
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK0_FREQ_TRACK_HDR_SKIP 966
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK0_FREQ_TRACK_EN       967
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK1_FREQ_SYNCH_GAIN     968
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK1_FREQ_TRACK_AUTO_THR 969
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK2_SNR_MIN_WINDOW      970
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK2_GAIN_AUTO_SNR_MIN   971
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK2_FREQ_SYNCH_THR      972
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR             973
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR            974
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB        975
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB         976
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_ACC_MIN2            977
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_ACC_WIN_LEN         978
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_POS_SEL         979
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_CNT_MODE        980
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_USE_GAIN_SYMB       981
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR             982
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_NOISE_COEFF         983
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_ACC_COEFF           984
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_ACC_2_SAME_PEAKS    985
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_ACC_AUTO_RESCALE    986
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_ACC_PEAK_POS_SEL    987
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_ACC_PEAK_SUM_EN     988
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DETECT_ACC3_MIN_SINGLE_PEAK     989
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TIMESTAMP_SEL_SNR_MIN           990
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TIMESTAMP_ENABLE                991
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_TIMESTAMP_NB_SYMB               992
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_CLOCK_GATE_OVERRIDE_FSK_TRANSPOSE_CLK_OVERRIDE \
    993
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_CLOCK_GATE_OVERRIDE_FSK_MODEM_CLK_OVERRIDE \
    994
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_CLOCK_GATE_OVERRIDE_TRANSPOSE_CLK_OVERRIDE \
    995
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_CLOCK_GATE_OVERRIDE_MODEM_CLK_OVERRIDE \
    996
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DUMMY0_DUMMY0             997
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_MSB_IF_FREQ_0    998
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_LSB_IF_FREQ_0    999
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_CRC_IBM         1000
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_DCFREE_ENC      1001
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_CRC_EN          1002
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_PKT_MODE        1003
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_ADRS_COMP       1004
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_PSIZE           1005
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_CH_BW_EXPO      1006
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_MODEM_INVERT_IQ 1007
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_AUTO_AFC        1008
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RADIO_SELECT    1009
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RX_INVERT       1010
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_4_RSSI_LENGTH     1011
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_CFG_4_ERROR_OSR_TOL   1012
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_NODE_ADRS_NODE_ADRS   1013
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_BROADCAST_BROADCAST   1014
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_PKT_LENGTH_PKT_LENGTH 1015
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_TIMEOUT_MSB_TIMEOUT   1016
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_TIMEOUT_LSB_TIMEOUT   1017
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BIT_RATE_MSB_BIT_RATE     1018
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BIT_RATE_LSB_BIT_RATE     1019
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN \
    1020
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN \
    1021
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN \
    1022
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN \
    1023
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN \
    1024
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN \
    1025
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN \
    1026
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN \
    1027
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_FSK_RSSI_FILTER_ALPHA_FSK_RSSI_FILTER_ALPHA \
    1028
#define SX1302_REG_RX_TOP_LORA_SERVICE_FSK_DUMMY1_DUMMY1        1029
#define SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_ENABLE               1030
#define SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTUREWRAP          1031
#define SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTUREFORCETRIGGER  1032
#define SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTURESTART         1033
#define SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_RAMCONFIG            1034
#define SX1302_REG_CAPTURE_RAM_CAPTURE_SOURCE_A_SOURCEMUX       1035
#define SX1302_REG_CAPTURE_RAM_CAPTURE_SOURCE_B_SOURCEMUX       1036
#define SX1302_REG_CAPTURE_RAM_CAPTURE_PERIOD_0_CAPTUREPERIOD   1037
#define SX1302_REG_CAPTURE_RAM_CAPTURE_PERIOD_1_CAPTUREPERIOD   1038
#define SX1302_REG_CAPTURE_RAM_STATUS_CAPCOMPLETE               1039
#define SX1302_REG_CAPTURE_RAM_LAST_RAM_ADDR_0_LASTRAMADDR      1040
#define SX1302_REG_CAPTURE_RAM_LAST_RAM_ADDR_1_LASTRAMADDR      1041
#define SX1302_REG_CAPTURE_RAM_CLOCK_GATE_OVERRIDE_CLK_OVERRIDE 1042
#define SX1302_REG_CAPTURE_RAM_DUMMY0_DUMMY0                    1043

#define SX1302_TOTALREGS 1044

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define SX1302_REG_TX_TOP_TX_TRIG_TX_FSM_CLR(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_TRIG_TX_FSM_CLR \
                     : SX1302_REG_TX_TOP_B_TX_TRIG_TX_FSM_CLR)
#define SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_GPS(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_GPS \
                     : SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_GPS)
#define SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_DELAYED(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_DELAYED \
                     : SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_DELAYED)
#define SX1302_REG_TX_TOP_TX_TRIG_TX_TRIG_IMMEDIATE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE \
                     : SX1302_REG_TX_TOP_B_TX_TRIG_TX_TRIG_IMMEDIATE)
#define SX1302_REG_TX_TOP_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG(rf_chain) \
    ((rf_chain == 0)                                                    \
         ? SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG      \
         : SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG)
#define SX1302_REG_TX_TOP_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG(rf_chain) \
    ((rf_chain == 0)                                                    \
         ? SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG      \
         : SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG)
#define SX1302_REG_TX_TOP_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG(rf_chain) \
    ((rf_chain == 0)                                                    \
         ? SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG      \
         : SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG)
#define SX1302_REG_TX_TOP_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG(rf_chain) \
    ((rf_chain == 0)                                                    \
         ? SX1302_REG_TX_TOP_A_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG      \
         : SX1302_REG_TX_TOP_B_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG)
#define SX1302_REG_TX_TOP_TX_START_DELAY_MSB_TX_START_DELAY(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_START_DELAY_MSB_TX_START_DELAY \
                     : SX1302_REG_TX_TOP_B_TX_START_DELAY_MSB_TX_START_DELAY)
#define SX1302_REG_TX_TOP_TX_START_DELAY_LSB_TX_START_DELAY(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_START_DELAY_LSB_TX_START_DELAY \
                     : SX1302_REG_TX_TOP_B_TX_START_DELAY_LSB_TX_START_DELAY)
#define SX1302_REG_TX_TOP_TX_CTRL_WRITE_BUFFER(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_CTRL_WRITE_BUFFER \
                     : SX1302_REG_TX_TOP_B_TX_CTRL_WRITE_BUFFER)
#define SX1302_REG_TX_TOP_TX_RAMP_DURATION_TX_RAMP_DURATION(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RAMP_DURATION_TX_RAMP_DURATION \
                     : SX1302_REG_TX_TOP_B_TX_RAMP_DURATION_TX_RAMP_DURATION)
#define SX1302_REG_TX_TOP_GEN_CFG_0_MODULATION_TYPE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_GEN_CFG_0_MODULATION_TYPE \
                     : SX1302_REG_TX_TOP_B_GEN_CFG_0_MODULATION_TYPE)
#define SX1302_REG_TX_TOP_TEST_0_TX_ACTIVE_CTRL(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TEST_0_TX_ACTIVE_CTRL \
                     : SX1302_REG_TX_TOP_B_TEST_0_TX_ACTIVE_CTRL)
#define SX1302_REG_TX_TOP_TEST_0_TX_ACTIVE_SEL(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TEST_0_TX_ACTIVE_SEL \
                     : SX1302_REG_TX_TOP_B_TEST_0_TX_ACTIVE_SEL)
#define SX1302_REG_TX_TOP_TX_FLAG_TX_TIMEOUT(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_FLAG_TX_TIMEOUT \
                     : SX1302_REG_TX_TOP_B_TX_FLAG_TX_TIMEOUT)
#define SX1302_REG_TX_TOP_TX_FLAG_PKT_DONE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_FLAG_PKT_DONE \
                     : SX1302_REG_TX_TOP_B_TX_FLAG_PKT_DONE)
#define SX1302_REG_TX_TOP_AGC_TX_BW_AGC_TX_BW(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_AGC_TX_BW_AGC_TX_BW \
                     : SX1302_REG_TX_TOP_B_AGC_TX_BW_AGC_TX_BW)
#define SX1302_REG_TX_TOP_AGC_TX_PWR_AGC_TX_PWR(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_AGC_TX_PWR_AGC_TX_PWR \
                     : SX1302_REG_TX_TOP_B_AGC_TX_PWR_AGC_TX_PWR)
#define SX1302_REG_TX_TOP_TIMEOUT_CNT_BYTE_2_TIMEOUT_CNT(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TIMEOUT_CNT_BYTE_2_TIMEOUT_CNT \
                     : SX1302_REG_TX_TOP_B_TIMEOUT_CNT_BYTE_2_TIMEOUT_CNT)
#define SX1302_REG_TX_TOP_TIMEOUT_CNT_BYTE_1_TIMEOUT_CNT(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TIMEOUT_CNT_BYTE_1_TIMEOUT_CNT \
                     : SX1302_REG_TX_TOP_B_TIMEOUT_CNT_BYTE_1_TIMEOUT_CNT)
#define SX1302_REG_TX_TOP_TIMEOUT_CNT_BYTE_0_TIMEOUT_CNT(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TIMEOUT_CNT_BYTE_0_TIMEOUT_CNT \
                     : SX1302_REG_TX_TOP_B_TIMEOUT_CNT_BYTE_0_TIMEOUT_CNT)
#define SX1302_REG_TX_TOP_TX_FSM_STATUS_TX_STATUS(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_FSM_STATUS_TX_STATUS \
                     : SX1302_REG_TX_TOP_B_TX_FSM_STATUS_TX_STATUS)
#define SX1302_REG_TX_TOP_DUMMY_CONTROL_DUMMY(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_DUMMY_CONTROL_DUMMY \
                     : SX1302_REG_TX_TOP_B_DUMMY_CONTROL_DUMMY)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_PLL_DIV_CTRL(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_PLL_DIV_CTRL \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_PLL_DIV_CTRL)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_CLK_EDGE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_CLK_EDGE)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_MODE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_DST(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL_TX_IF_SRC(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_SRC)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL2_SX125X_IQ_INVERT(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL2_SX125X_IQ_INVERT \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL2_SX125X_IQ_INVERT)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_IQ_GAIN_IQ_GAIN(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_IQ_GAIN_IQ_GAIN \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_IQ_GAIN_IQ_GAIN)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_I_OFFSET_I_OFFSET(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_Q_OFFSET_Q_OFFSET(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_H_FREQ_RF(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_H_FREQ_RF)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_M_FREQ_RF(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_M_FREQ_RF)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_RF_L_FREQ_RF(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_RF_L_FREQ_RF)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV)
#define SX1302_REG_TX_TOP_TX_RFFE_IF_TEST_MOD_FREQ(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_RFFE_IF_TEST_MOD_FREQ \
                     : SX1302_REG_TX_TOP_B_TX_RFFE_IF_TEST_MOD_FREQ)
#define SX1302_REG_TX_TOP_DUMMY_MODULATOR_DUMMY(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_DUMMY_MODULATOR_DUMMY \
                     : SX1302_REG_TX_TOP_B_DUMMY_MODULATOR_DUMMY)
#define SX1302_REG_TX_TOP_FSK_PKT_LEN_PKT_LENGTH(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_PKT_LEN_PKT_LENGTH \
                     : SX1302_REG_TX_TOP_B_FSK_PKT_LEN_PKT_LENGTH)
#define SX1302_REG_TX_TOP_FSK_CFG_0_TX_CONT(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_CFG_0_TX_CONT \
                     : SX1302_REG_TX_TOP_B_FSK_CFG_0_TX_CONT)
#define SX1302_REG_TX_TOP_FSK_CFG_0_CRC_IBM(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_CFG_0_CRC_IBM \
                     : SX1302_REG_TX_TOP_B_FSK_CFG_0_CRC_IBM)
#define SX1302_REG_TX_TOP_FSK_CFG_0_DCFREE_ENC(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_CFG_0_DCFREE_ENC \
                     : SX1302_REG_TX_TOP_B_FSK_CFG_0_DCFREE_ENC)
#define SX1302_REG_TX_TOP_FSK_CFG_0_CRC_EN(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_CFG_0_CRC_EN \
                     : SX1302_REG_TX_TOP_B_FSK_CFG_0_CRC_EN)
#define SX1302_REG_TX_TOP_FSK_CFG_0_PKT_MODE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_CFG_0_PKT_MODE \
                     : SX1302_REG_TX_TOP_B_FSK_CFG_0_PKT_MODE)
#define SX1302_REG_TX_TOP_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE(rf_chain) \
    ((rf_chain == 0)                                                    \
         ? SX1302_REG_TX_TOP_A_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE      \
         : SX1302_REG_TX_TOP_B_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE)
#define SX1302_REG_TX_TOP_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE(rf_chain) \
    ((rf_chain == 0)                                                    \
         ? SX1302_REG_TX_TOP_A_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE      \
         : SX1302_REG_TX_TOP_B_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE)
#define SX1302_REG_TX_TOP_FSK_BIT_RATE_MSB_BIT_RATE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_BIT_RATE_MSB_BIT_RATE \
                     : SX1302_REG_TX_TOP_B_FSK_BIT_RATE_MSB_BIT_RATE)
#define SX1302_REG_TX_TOP_FSK_BIT_RATE_LSB_BIT_RATE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_BIT_RATE_LSB_BIT_RATE \
                     : SX1302_REG_TX_TOP_B_FSK_BIT_RATE_LSB_BIT_RATE)
#define SX1302_REG_TX_TOP_FSK_MOD_FSK_REF_PATTERN_SIZE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_MOD_FSK_REF_PATTERN_SIZE \
                     : SX1302_REG_TX_TOP_B_FSK_MOD_FSK_REF_PATTERN_SIZE)
#define SX1302_REG_TX_TOP_FSK_MOD_FSK_PREAMBLE_SEQ(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_MOD_FSK_PREAMBLE_SEQ \
                     : SX1302_REG_TX_TOP_B_FSK_MOD_FSK_PREAMBLE_SEQ)
#define SX1302_REG_TX_TOP_FSK_MOD_FSK_REF_PATTERN_EN(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_MOD_FSK_REF_PATTERN_EN \
                     : SX1302_REG_TX_TOP_B_FSK_MOD_FSK_REF_PATTERN_EN)
#define SX1302_REG_TX_TOP_FSK_MOD_FSK_GAUSSIAN_SELECT_BT(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_MOD_FSK_GAUSSIAN_SELECT_BT \
                     : SX1302_REG_TX_TOP_B_FSK_MOD_FSK_GAUSSIAN_SELECT_BT)
#define SX1302_REG_TX_TOP_FSK_MOD_FSK_GAUSSIAN_EN(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FSK_MOD_FSK_GAUSSIAN_EN \
                     : SX1302_REG_TX_TOP_B_FSK_MOD_FSK_GAUSSIAN_EN)
#define SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN(rf_chain) \
    ((rf_chain == 0)                                                      \
         ? SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN      \
         : SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN)
#define SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN(rf_chain) \
    ((rf_chain == 0)                                                      \
         ? SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN      \
         : SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN)
#define SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN(rf_chain) \
    ((rf_chain == 0)                                                      \
         ? SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN      \
         : SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN)
#define SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN(rf_chain) \
    ((rf_chain == 0)                                                      \
         ? SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN      \
         : SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN)
#define SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN(rf_chain) \
    ((rf_chain == 0)                                                      \
         ? SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN      \
         : SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN)
#define SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN(rf_chain) \
    ((rf_chain == 0)                                                      \
         ? SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN      \
         : SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN)
#define SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN(rf_chain) \
    ((rf_chain == 0)                                                      \
         ? SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN      \
         : SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN)
#define SX1302_REG_TX_TOP_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN(rf_chain) \
    ((rf_chain == 0)                                                      \
         ? SX1302_REG_TX_TOP_A_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN      \
         : SX1302_REG_TX_TOP_B_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN)
#define SX1302_REG_TX_TOP_DUMMY_GSFK_DUMMY(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_DUMMY_GSFK_DUMMY \
                     : SX1302_REG_TX_TOP_B_DUMMY_GSFK_DUMMY)
#define SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_BW(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_BW \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_BW)
#define SX1302_REG_TX_TOP_TXRX_CFG0_0_MODEM_SF(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_0_MODEM_SF \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_0_MODEM_SF)
#define SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL)
#define SX1302_REG_TX_TOP_TXRX_CFG0_1_PPM_OFFSET(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_1_PPM_OFFSET \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_1_PPM_OFFSET)
#define SX1302_REG_TX_TOP_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG)
#define SX1302_REG_TX_TOP_TXRX_CFG0_1_CODING_RATE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_1_CODING_RATE \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_1_CODING_RATE)
#define SX1302_REG_TX_TOP_TXRX_CFG0_2_FINE_SYNCH_EN(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_2_FINE_SYNCH_EN \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_2_FINE_SYNCH_EN)
#define SX1302_REG_TX_TOP_TXRX_CFG0_2_MODEM_EN(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_2_MODEM_EN \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_2_MODEM_EN)
#define SX1302_REG_TX_TOP_TXRX_CFG0_2_CADRXTX(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CADRXTX \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_2_CADRXTX)
#define SX1302_REG_TX_TOP_TXRX_CFG0_2_IMPLICIT_HEADER(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_2_IMPLICIT_HEADER \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_2_IMPLICIT_HEADER)
#define SX1302_REG_TX_TOP_TXRX_CFG0_2_CRC_EN(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_2_CRC_EN \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_2_CRC_EN)
#define SX1302_REG_TX_TOP_TXRX_CFG0_3_PAYLOAD_LENGTH(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG0_3_PAYLOAD_LENGTH \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG0_3_PAYLOAD_LENGTH)
#define SX1302_REG_TX_TOP_TXRX_CFG1_0_INT_STEP_ORIDE_EN(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG1_0_INT_STEP_ORIDE_EN \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG1_0_INT_STEP_ORIDE_EN)
#define SX1302_REG_TX_TOP_TXRX_CFG1_0_INT_STEP_ORIDE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG1_0_INT_STEP_ORIDE \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG1_0_INT_STEP_ORIDE)
#define SX1302_REG_TX_TOP_TXRX_CFG1_1_MODEM_START(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG1_1_MODEM_START \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG1_1_MODEM_START)
#define SX1302_REG_TX_TOP_TXRX_CFG1_1_HEADER_DIFF_MODE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG1_1_HEADER_DIFF_MODE \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG1_1_HEADER_DIFF_MODE)
#define SX1302_REG_TX_TOP_TXRX_CFG1_1_ZERO_PAD(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG1_1_ZERO_PAD \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG1_1_ZERO_PAD)
#define SX1302_REG_TX_TOP_TXRX_CFG1_2_PREAMBLE_SYMB_NB(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG1_2_PREAMBLE_SYMB_NB \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG1_2_PREAMBLE_SYMB_NB)
#define SX1302_REG_TX_TOP_TXRX_CFG1_3_PREAMBLE_SYMB_NB(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG1_3_PREAMBLE_SYMB_NB \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG1_3_PREAMBLE_SYMB_NB)
#define SX1302_REG_TX_TOP_TXRX_CFG1_4_AUTO_ACK_INT_DELAY(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_INT_DELAY \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_INT_DELAY)
#define SX1302_REG_TX_TOP_TXRX_CFG1_4_AUTO_ACK_RX(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_RX \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_RX)
#define SX1302_REG_TX_TOP_TXRX_CFG1_4_AUTO_ACK_TX(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_TX \
                     : SX1302_REG_TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_TX)
#define SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_LOWPASS(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_LOWPASS \
                     : SX1302_REG_TX_TOP_B_TX_CFG0_0_CHIRP_LOWPASS)
#define SX1302_REG_TX_TOP_TX_CFG0_0_PPM_OFFSET_SIG(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_CFG0_0_PPM_OFFSET_SIG \
                     : SX1302_REG_TX_TOP_B_TX_CFG0_0_PPM_OFFSET_SIG)
#define SX1302_REG_TX_TOP_TX_CFG0_0_CONTCHIRP(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTCHIRP \
                     : SX1302_REG_TX_TOP_B_TX_CFG0_0_CONTCHIRP)
#define SX1302_REG_TX_TOP_TX_CFG0_0_CHIRP_INVERT(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_CFG0_0_CHIRP_INVERT \
                     : SX1302_REG_TX_TOP_B_TX_CFG0_0_CHIRP_INVERT)
#define SX1302_REG_TX_TOP_TX_CFG0_0_CONTINUOUS(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_CFG0_0_CONTINUOUS \
                     : SX1302_REG_TX_TOP_B_TX_CFG0_0_CONTINUOUS)
#define SX1302_REG_TX_TOP_TX_CFG0_1_POWER_RANGING(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_CFG0_1_POWER_RANGING \
                     : SX1302_REG_TX_TOP_B_TX_CFG0_1_POWER_RANGING)
#define SX1302_REG_TX_TOP_TX_CFG1_0_FRAME_NB(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_CFG1_0_FRAME_NB \
                     : SX1302_REG_TX_TOP_B_TX_CFG1_0_FRAME_NB)
#define SX1302_REG_TX_TOP_TX_CFG1_1_HOP_CTRL(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_CFG1_1_HOP_CTRL \
                     : SX1302_REG_TX_TOP_B_TX_CFG1_1_HOP_CTRL)
#define SX1302_REG_TX_TOP_TX_CFG1_1_IFS(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_TX_CFG1_1_IFS \
                     : SX1302_REG_TX_TOP_B_TX_CFG1_1_IFS)
#define SX1302_REG_TX_TOP_FRAME_SYNCH_0_AUTO_SCALE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_AUTO_SCALE \
                     : SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_AUTO_SCALE)
#define SX1302_REG_TX_TOP_FRAME_SYNCH_0_DROP_ON_SYNCH(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_DROP_ON_SYNCH \
                     : SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_DROP_ON_SYNCH)
#define SX1302_REG_TX_TOP_FRAME_SYNCH_0_GAIN(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_GAIN \
                     : SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_GAIN)
#define SX1302_REG_TX_TOP_FRAME_SYNCH_0_PEAK1_POS(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FRAME_SYNCH_0_PEAK1_POS \
                     : SX1302_REG_TX_TOP_B_FRAME_SYNCH_0_PEAK1_POS)
#define SX1302_REG_TX_TOP_FRAME_SYNCH_1_FINETIME_ON_LAST(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_FINETIME_ON_LAST \
                     : SX1302_REG_TX_TOP_B_FRAME_SYNCH_1_FINETIME_ON_LAST)
#define SX1302_REG_TX_TOP_FRAME_SYNCH_1_TIMEOUT_OPT(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_TIMEOUT_OPT \
                     : SX1302_REG_TX_TOP_B_FRAME_SYNCH_1_TIMEOUT_OPT)
#define SX1302_REG_TX_TOP_FRAME_SYNCH_1_PEAK2_POS(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_FRAME_SYNCH_1_PEAK2_POS \
                     : SX1302_REG_TX_TOP_B_FRAME_SYNCH_1_PEAK2_POS)
#define SX1302_REG_TX_TOP_LORA_TX_STATE_STATUS(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_LORA_TX_STATE_STATUS \
                     : SX1302_REG_TX_TOP_B_LORA_TX_STATE_STATUS)
#define SX1302_REG_TX_TOP_LORA_TX_FLAG_FRAME_DONE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_LORA_TX_FLAG_FRAME_DONE \
                     : SX1302_REG_TX_TOP_B_LORA_TX_FLAG_FRAME_DONE)
#define SX1302_REG_TX_TOP_LORA_TX_FLAG_CONT_DONE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_LORA_TX_FLAG_CONT_DONE \
                     : SX1302_REG_TX_TOP_B_LORA_TX_FLAG_CONT_DONE)
#define SX1302_REG_TX_TOP_LORA_TX_FLAG_PLD_DONE(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_LORA_TX_FLAG_PLD_DONE \
                     : SX1302_REG_TX_TOP_B_LORA_TX_FLAG_PLD_DONE)
#define SX1302_REG_TX_TOP_DUMMY_LORA_DUMMY(rf_chain)        \
    ((rf_chain == 0) ? SX1302_REG_TX_TOP_A_DUMMY_LORA_DUMMY \
                     : SX1302_REG_TX_TOP_B_DUMMY_LORA_DUMMY)

extern const sx1302_regs_t loregs[SX1302_TOTALREGS + 1];

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SX127X_REGISTERS_H */
/** @} */
