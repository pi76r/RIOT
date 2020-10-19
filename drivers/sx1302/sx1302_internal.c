/*
 * Copyright (c) 2020 Université Grenoble Alpes
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx1302
 * @{
 * @file
 * @brief       implementation of internal functions for sx1302
 *
 * @author      Pierre Millot
 * @}
 */
#include "sx1302_internal.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "net/lora.h"
#include "sx1250.h"
#include "sx1302.h"
#include "sx1302_params.h"
#include "sx1302_registers.h"

#define ENABLE_DEBUG SX1302_DEBUG
#include "debug.h"

//#define CHUNK_SIZE_MAX 1024
#define CHUNK_SIZE_MAX 512

static bool SX1302_SPI_ACQUIRE = true;

#ifndef SX1302_SPI_MODE
#define SX1302_SPI_MODE  (SPI_MODE_0)
#endif

#ifndef SX1302_SPI_SPEED
//#define SX1302_SPI_SPEED  (SPI_CLK_400KHZ)
#define SX1302_SPI_SPEED  (SPI_CLK_5MHZ)
#endif

#define SPI_PAUSE_DELAY 1000

#if SPI_PAUSE_DELAY == 0
#define SPI_PAUSE
#else
#include "xtimer.h"
#define SPI_PAUSE      xtimer_usleep(SPI_PAUSE_DELAY)
#endif

#define DEBUG_SPI 1
#if DEBUG_SPI == 1


static void printfhex(const uint8_t *data, const uint16_t size)
{
        for (int i=0; i < size; ++i) {
                printf("%02x", data[i]);
        }
        printf("\n");
}

    #define DEBUG_MSG(str)                printf(str)
	#define DEBUG_PRINTF(fmt, args...)    printf("%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define DEBUG_RW(fmt, args...)    	  printf(fmt, args)
    #define DEBUG_HEX(data, size)    	  printfhex(data, size)
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define DEBUG_RW(fmt, args...)
    #define DEBUG_HEX(data, size)
#endif



void sx1302_spi_acquire_set(const bool acquire) {
	SX1302_SPI_ACQUIRE = acquire;
}

void sx1302_spi_acquire(const sx1302_t *dev) {
    if(!SX1302_SPI_ACQUIRE) spi_acquire(dev->params.spi, dev->params.nss_pin, SX1302_SPI_MODE,
                SX1302_SPI_SPEED);
}

void sx1302_spi_release(const sx1302_t *dev) {
    if(!SX1302_SPI_ACQUIRE) spi_release(dev->params.spi);
}

static void _sx1302_spi_acquire(const sx1302_t *dev) {
    if(SX1302_SPI_ACQUIRE) spi_acquire(dev->params.spi, dev->params.nss_pin, SX1302_SPI_MODE,
                SX1302_SPI_SPEED);
}

static void _sx1302_spi_release(const sx1302_t *dev) {
    if(SX1302_SPI_ACQUIRE) spi_release(dev->params.spi);
}

static void sx1302_write_reg_buffer(const sx1302_t *dev,
                                    SX1302_SpiMuxTarget_t target, uint16_t addr,
                                    const uint8_t *buffer, uint16_t size) {


    DEBUG_RW("[lgw_spi_wbu] %4x %d ", addr, size); // TODO add data
    DEBUG_HEX(buffer,size);
    DEBUG_MSG("\n");

    // wait_on_busy(dev);
	_sx1302_spi_acquire(dev);

    uint8_t cmd[size + 3];
    cmd[0] = target;
    cmd[1] = SX1302_WRITE_REGISTER | ((addr >> 8) & 0x7F);
    cmd[2] = addr & 0xFF;
    memcpy(cmd + 3, buffer, size);
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, false, cmd, NULL,
                       size + 3);
    SPI_PAUSE;
    _sx1302_spi_release(dev);
    // wait_on_busy(dev);
    SPI_PAUSE;
}

static void sx1302_read_reg_buffer(const sx1302_t *dev,
                                   SX1302_SpiMuxTarget_t target, uint16_t addr,
                                   uint8_t *buffer, uint16_t size) {

    // wait_on_busy(dev);
	_sx1302_spi_acquire(dev);

    uint8_t cmd[4];
    cmd[0] = target;
    cmd[1] = SX1302_READ_REGISTER | ((addr >> 8) & 0x7F);
    cmd[2] = addr & 0xFF;
    cmd[3] = 0;
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, true, cmd, NULL,
                       4);
    //SPI_PAUSE;
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, false, NULL,
                       buffer, size);
    SPI_PAUSE;
    _sx1302_spi_release(dev);
    // wait_on_busy(dev);

    DEBUG_RW("[lgw_spi_rbu] %4x %d ", addr, size);
    DEBUG_HEX(buffer,size);
    DEBUG_MSG("\n");

}

void sx1302_reg_read_batch(const sx1302_t *dev, uint16_t register_id,
                           uint8_t *data, uint16_t size) {

    if (register_id >= SX1302_TOTALREGS) {
        DEBUG_PUTS("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return;
    }
    sx1302_regs_t r = loregs[register_id];

	_sx1302_spi_acquire(dev);

    uint8_t cmd[4];
    cmd[0] = SX1302_SPI_MUX_TARGET_SX1302;
    cmd[1] = SX1302_READ_REGISTER | ((r.addr >> 8) & 0x7F);
    cmd[2] = r.addr & 0xFF;
    cmd[3] = 0;
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, true, cmd, NULL,
                       4);
    SPI_PAUSE;
    int chunk_size;
    int offset = 0;
    /* write memory by chunks */
    while (size > 0) {
        /* full or partial chunk ? */
        chunk_size = (size > CHUNK_SIZE_MAX) ? CHUNK_SIZE_MAX : size;

        /* do the burst write */
        spi_transfer_bytes(dev->params.spi, dev->params.nss_pin,
                           size > CHUNK_SIZE_MAX, NULL, &data[offset],
                           chunk_size);
        SPI_PAUSE;
        /* prepare for next write */
        size -= chunk_size;
        offset += CHUNK_SIZE_MAX;
    }

    _sx1302_spi_release(dev);

    DEBUG_RW("[lgw_spi_rba] %4x %d ", register_id, size);
    DEBUG_HEX(data,size);
    DEBUG_MSG("\n");

}

static void sx1302_write_reg_align32(const sx1302_t *dev,
                                     SX1302_SpiMuxTarget_t spi_mux_target,
                                     sx1302_regs_t r, int32_t reg_value) {
    int i, size_byte;
    uint8_t buf[4] = {0,0,0,0};

    if ((r.leng == 8) && (r.offs == 0)) {
        /* direct write */
        uint8_t val = (uint8_t)reg_value;
        sx1302_write_reg_buffer(dev, spi_mux_target, r.addr, &val, 1);

    } else if ((r.offs + r.leng) <= 8) {
        /* single-byte read-modify-write, offs:[0-7], leng:[1-7] */
        sx1302_read_reg_buffer(dev, spi_mux_target, r.addr, &buf[0], 1);
        buf[1] = ((1 << r.leng) - 1) << r.offs;  /* bit mask */
        buf[2] = ((uint8_t)reg_value) << r.offs; /* new data offsetted */
        buf[3] =
            (~buf[1] & buf[0]) | (buf[1] & buf[2]); /* mixing old & new data */
        sx1302_write_reg_buffer(dev, spi_mux_target, r.addr, &buf[3], 1);

    } else if ((r.offs == 0) && (r.leng > 0) && (r.leng <= 32)) {
        /* multi-byte direct write routine */
        size_byte = (r.leng + 7) /
                    8; /* add a byte if it's not an exact multiple of 8 */
        for (i = 0; i < size_byte; ++i) {
            /* big endian register file for a file on N bytes
            Least significant byte is stored in buf[0], most one in buf[N-1] */
            buf[i]    = (uint8_t)(0x000000FF & reg_value);
            reg_value = (reg_value >> 8);
        }
        sx1302_write_reg_buffer(
            dev, spi_mux_target, r.addr, buf,
            size_byte); /* write the register in one burst */

    } else {
        /* register spanning multiple memory bytes but with an offset */
        DEBUG_PUTS("ERROR: REGISTER SIZE AND OFFSET ARE NOT SUPPORTED\n");
    }
}

static int32_t sx1302_read_reg_align32(const sx1302_t *dev,
                                       uint8_t spi_mux_target,
                                       sx1302_regs_t r) {
    uint8_t bufu[4] = {0,0,0,0};
    int8_t *bufs    = (int8_t *)bufu;
    int i, size_byte;
    uint32_t u = 0;

    if ((r.offs + r.leng) <= 8) {
        /* read one byte, then shift and mask bits to get reg value with sign
         * extension if needed */
        sx1302_read_reg_buffer(dev, spi_mux_target, r.addr, &bufu[0], 1);

        bufu[1] = bufu[0] << (8 - r.leng - r.offs); /* left-align the data */
        if (r.sign) {
            bufs[2] = bufs[1] >>
                      (8 - r.leng);  /* right align the data with sign extension
                                        (ARITHMETIC right shift) */
            return (int32_t)bufs[2]; /* signed pointer -> 32b sign extension */
        } else {
            bufu[2] =
                bufu[1] >>
                (8 - r.leng); /* right align the data, no sign extension */
            return (int32_t)bufu[2]; /* unsigned pointer -> no sign extension */
        }
    } else if ((r.offs == 0) && (r.leng > 0) && (r.leng <= 32)) {
        size_byte = (r.leng + 7) /
                    8; /* add a byte if it's not an exact multiple of 8 */
        sx1302_read_reg_buffer(dev, spi_mux_target, r.addr, bufu, size_byte);
        u = 0;
        for (i = (size_byte - 1); i >= 0; --i) {
            u = (uint32_t)bufu[i] +
                (u << 8); /* transform a 4-byte array into a 32 bit word */
        }
        if (r.sign) {
            u = u << (32 - r.leng); /* left-align the data */
            return (int32_t)u >>
                   (32 - r.leng); /* right-align the data with sign extension
                                     (ARITHMETIC right shift) */
        } else {
            return (int32_t)u; /* unsigned value -> return 'as is' */
        }
    } else {
        /* register spanning multiple memory bytes but with an offset */
        DEBUG_PUTS("ERROR: REGISTER SIZE AND OFFSET ARE NOT SUPPORTED\n");
    }

    return -1;
}

void sx1302_reg_write(const sx1302_t *dev, uint16_t register_id,
                      int32_t reg_value) {
    if (register_id >= SX1302_TOTALREGS) {
        DEBUG_PUTS("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return;
    }
    sx1302_regs_t r = loregs[register_id];

    /* reject write to read-only registers */
    if (r.rdon) {
        DEBUG_PUTS("ERROR: TRYING TO WRITE A READ-ONLY REGISTER\n");
        return;
    }

    sx1302_write_reg_align32(dev, SX1302_SPI_MUX_TARGET_SX1302, r, reg_value);

}

int32_t sx1302_reg_read(const sx1302_t *dev, uint16_t register_id) {
    if (register_id >= SX1302_TOTALREGS) {
        DEBUG_PUTS("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
        return -1;
    }
    sx1302_regs_t r = loregs[register_id];

    return sx1302_read_reg_align32(dev, SX1302_SPI_MUX_TARGET_SX1302, r);
}

int sx1302_mem_write(const sx1302_t *dev, uint16_t addr, const uint8_t *data,
                     uint16_t size) {


    int chunk_cnt = 0;
    uint16_t chunk_size;

    /* check input parameters */
    if (size == 0) {
        DEBUG_PUTS("ERROR: BURST OF NULL LENGTH");
        return -1;
    }

    /* write memory by chunks */
    while (size > 0) {
        /* full or partial chunk ? */
        chunk_size = (size > CHUNK_SIZE_MAX) ? CHUNK_SIZE_MAX : size;

        /* do the burst write */
        sx1302_write_reg_buffer(dev, SX1302_SPI_MUX_TARGET_SX1302, addr,
                                &data[chunk_cnt * CHUNK_SIZE_MAX], chunk_size);

        /* prepare for next write */
        addr += chunk_size;
        size -= chunk_size;
        chunk_cnt += 1;
    }
    return 0;
}

void sx1302_mem_read(sx1302_t *dev, uint16_t addr, uint8_t *data, uint16_t size,
                     bool fifo_mode) {
	int chunk_cnt = 0;
    uint16_t chunk_size;

    if (size == 0) {
        DEBUG_PUTS("ERROR: BURST OF NULL LENGTH");
        return;
    }

    /* read memory by chunks */
    while (size > 0) {
        /* full or partial chunk ? */
        chunk_size = (size > CHUNK_SIZE_MAX) ? CHUNK_SIZE_MAX : size;
         DEBUG("[READ] of size %d at 0x%04X\n", chunk_size, addr);
        /* do the burst read */
        sx1302_read_reg_buffer(dev, SX1302_SPI_MUX_TARGET_SX1302, addr,
                               &data[chunk_cnt * CHUNK_SIZE_MAX], chunk_size);

        /* do not increment the address when the target memory is in FIFO mode
         * (auto-increment) */
        if (!fifo_mode) {
            addr += chunk_size;
        }

        /* prepare for next read */
        size -= chunk_size;
        chunk_cnt += 1;
    }
}

void sx1250_write_command(const sx1302_t *dev, uint8_t rf_chain,
                          SX1250_op_code_t op_code, uint8_t *data,
                          uint16_t size) {

    DEBUG_RW("[sx1250_write_command] %d %2x %d ", rf_chain, op_code, size);
    DEBUG_HEX(data,size);
    DEBUG_MSG("\n");

	// wait_on_busy(dev);
	_sx1302_spi_acquire(dev);

    uint8_t cmd[size + 2];
    cmd[0] = (rf_chain == 0) ? SX1302_SPI_MUX_TARGET_RADIOA
                             : SX1302_SPI_MUX_TARGET_RADIOB;
    cmd[1] = op_code;
    memcpy(cmd + 2, data, size);
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, false, cmd, NULL,
                       size + 2);

    SPI_PAUSE;
    _sx1302_spi_release(dev);
    // wait_on_busy(dev);
}

void sx1250_read_command(const sx1302_t *dev, uint8_t rf_chain,
                         SX1250_op_code_t op_code, uint8_t *data,
                         uint16_t size) {
    // wait_on_busy(dev);
	_sx1302_spi_acquire(dev);

    uint8_t cmd[size + 2];
    cmd[0] = (rf_chain == 0) ? SX1302_SPI_MUX_TARGET_RADIOA
                             : SX1302_SPI_MUX_TARGET_RADIOB;
    cmd[1] = op_code;

    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, true, cmd, NULL,
                       2);
    //SPI_PAUSE;
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, false, data, data,
                       size);
    SPI_PAUSE;

    _sx1302_spi_release(dev);
    // wait_on_busy(dev);

	DEBUG_RW("[sx1250_read_command] %d %2x %d ", rf_chain, op_code, size);
    DEBUG_HEX(data,size);
    DEBUG_MSG("\n");
}

const sx1302_regs_t loregs[SX1302_TOTALREGS+1] = {
    {0,SX1302_REG_COMMON_BASE_ADDR+0,0,0,2,0,1,0}, // COMMON_PAGE_PAGE
    {0,SX1302_REG_COMMON_BASE_ADDR+1,4,0,1,0,1,0}, // COMMON_CTRL0_CLK32_RIF_CTRL
    {0,SX1302_REG_COMMON_BASE_ADDR+1,3,0,1,0,1,1}, // COMMON_CTRL0_HOST_RADIO_CTRL
    {0,SX1302_REG_COMMON_BASE_ADDR+1,2,0,1,0,1,0}, // COMMON_CTRL0_RADIO_MISC_EN
    {0,SX1302_REG_COMMON_BASE_ADDR+1,1,0,1,0,1,1}, // COMMON_CTRL0_SX1261_MODE_RADIO_B
    {0,SX1302_REG_COMMON_BASE_ADDR+1,0,0,1,0,1,1}, // COMMON_CTRL0_SX1261_MODE_RADIO_A
    {0,SX1302_REG_COMMON_BASE_ADDR+2,3,0,1,0,1,0}, // COMMON_CTRL1_SWAP_IQ_RADIO_B
    {0,SX1302_REG_COMMON_BASE_ADDR+2,2,0,1,0,1,1}, // COMMON_CTRL1_SAMPLING_EDGE_RADIO_B
    {0,SX1302_REG_COMMON_BASE_ADDR+2,1,0,1,0,1,0}, // COMMON_CTRL1_SWAP_IQ_RADIO_A
    {0,SX1302_REG_COMMON_BASE_ADDR+2,0,0,1,0,1,1}, // COMMON_CTRL1_SAMPLING_EDGE_RADIO_A
    {0,SX1302_REG_COMMON_BASE_ADDR+3,0,0,8,0,1,2}, // COMMON_SPI_DIV_RATIO_SPI_HALF_PERIOD
    {0,SX1302_REG_COMMON_BASE_ADDR+4,0,0,8,0,1,128}, // COMMON_RADIO_SELECT_RADIO_SELECT
    {0,SX1302_REG_COMMON_BASE_ADDR+5,3,0,1,0,1,0}, // COMMON_GEN_GLOBAL_EN
    {0,SX1302_REG_COMMON_BASE_ADDR+5,2,0,1,0,1,0}, // COMMON_GEN_FSK_MODEM_ENABLE
    {0,SX1302_REG_COMMON_BASE_ADDR+5,1,0,1,0,1,0}, // COMMON_GEN_CONCENTRATOR_MODEM_ENABLE
    {0,SX1302_REG_COMMON_BASE_ADDR+5,0,0,1,0,1,0}, // COMMON_GEN_MBWSSF_MODEM_ENABLE
    {0,SX1302_REG_COMMON_BASE_ADDR+6,0,0,8,1,1,16}, // COMMON_VERSION_VERSION
    {0,SX1302_REG_COMMON_BASE_ADDR+7,0,0,1,1,1,0}, // COMMON_DUMMY_DUMMY
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+0,4,0,1,0,1,1}, // AGC_MCU_CTRL_CLK_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+0,3,0,1,0,1,0}, // AGC_MCU_CTRL_FORCE_HOST_FE_CTRL
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+0,2,0,1,0,1,1}, // AGC_MCU_CTRL_MCU_CLEAR
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+0,1,0,1,0,1,0}, // AGC_MCU_CTRL_HOST_PROG
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+0,0,0,1,1,1,0}, // AGC_MCU_CTRL_PARITY_ERROR
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+1,0,0,8,1,1,0}, // AGC_MCU_MCU_AGC_STATUS_MCU_AGC_STATUS
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+2,2,0,2,0,1,0}, // AGC_MCU_PA_GAIN_PA_B_GAIN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+2,0,0,2,0,1,0}, // AGC_MCU_PA_GAIN_PA_A_GAIN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+3,3,0,1,0,1,0}, // AGC_MCU_RF_EN_A_RADIO_RST
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+3,2,0,1,0,1,0}, // AGC_MCU_RF_EN_A_RADIO_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+3,1,0,1,0,1,0}, // AGC_MCU_RF_EN_A_PA_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+3,0,0,1,0,1,0}, // AGC_MCU_RF_EN_A_LNA_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+4,3,0,1,0,1,0}, // AGC_MCU_RF_EN_B_RADIO_RST
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+4,2,0,1,0,1,0}, // AGC_MCU_RF_EN_B_RADIO_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+4,1,0,1,0,1,0}, // AGC_MCU_RF_EN_B_PA_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+4,0,0,1,0,1,0}, // AGC_MCU_RF_EN_B_LNA_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+5,4,0,4,0,1,0}, // AGC_MCU_LUT_TABLE_A_PA_LUT
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+5,0,0,4,0,1,0}, // AGC_MCU_LUT_TABLE_A_LNA_LUT
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+6,4,0,4,0,1,0}, // AGC_MCU_LUT_TABLE_B_PA_LUT
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+6,0,0,4,0,1,0}, // AGC_MCU_LUT_TABLE_B_LNA_LUT
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,5,0,1,0,1,0}, // AGC_MCU_UART_CFG_MSBF
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,4,0,1,0,1,0}, // AGC_MCU_UART_CFG_PAR_EN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,3,0,1,0,1,0}, // AGC_MCU_UART_CFG_PAR_MODE
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,2,0,1,0,1,0}, // AGC_MCU_UART_CFG_START_LEN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,1,0,1,0,1,0}, // AGC_MCU_UART_CFG_STOP_LEN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+7,0,0,1,0,1,1}, // AGC_MCU_UART_CFG_WORD_LEN
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+8,0,0,8,0,1,0}, // AGC_MCU_UART_CFG2_BIT_RATE
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+9,0,0,8,0,1,0}, // AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE3_MCU_MAIL_BOX_WR_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+10,0,0,8,0,1,0}, // AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE2_MCU_MAIL_BOX_WR_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+11,0,0,8,0,1,0}, // AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE1_MCU_MAIL_BOX_WR_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+12,0,0,8,0,1,0}, // AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+13,0,0,8,1,1,0}, // AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE3_MCU_MAIL_BOX_RD_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+14,0,0,8,1,1,0}, // AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE2_MCU_MAIL_BOX_RD_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+15,0,0,8,1,1,0}, // AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE1_MCU_MAIL_BOX_RD_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+16,0,0,8,1,1,0}, // AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE0_MCU_MAIL_BOX_RD_DATA
    {0,SX1302_REG_AGC_MCU_BASE_ADDR+17,0,0,1,1,1,0}, // AGC_MCU_DUMMY_DUMMY3
    {0,SX1302_REG_CLK_CTRL_BASE_ADDR+0,2,0,1,0,1,0}, // CLK_CTRL_CLK_SEL_CLKDIV_EN
    {0,SX1302_REG_CLK_CTRL_BASE_ADDR+0,1,0,1,0,1,0}, // CLK_CTRL_CLK_SEL_CLK_RADIO_B_SEL
    {0,SX1302_REG_CLK_CTRL_BASE_ADDR+0,0,0,1,0,1,0}, // CLK_CTRL_CLK_SEL_CLK_RADIO_A_SEL
    {0,SX1302_REG_CLK_CTRL_BASE_ADDR+1,3,0,1,1,1,0}, // CLK_CTRL_DUMMY_DUMMY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+0,3,0,1,0,0,0}, // TX_TOP_A_TX_TRIG_TX_FSM_CLR
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+0,2,0,1,0,1,0}, // TX_TOP_A_TX_TRIG_TX_TRIG_GPS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+0,1,0,1,0,1,0}, // TX_TOP_A_TX_TRIG_TX_TRIG_DELAYED
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+0,0,0,1,0,1,0}, // TX_TOP_A_TX_TRIG_TX_TRIG_IMMEDIATE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+1,0,0,8,0,1,0}, // TX_TOP_A_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+2,0,0,8,0,1,0}, // TX_TOP_A_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+3,0,0,8,0,1,0}, // TX_TOP_A_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+4,0,0,8,0,1,0}, // TX_TOP_A_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+5,0,0,8,0,1,187}, // TX_TOP_A_TX_START_DELAY_MSB_TX_START_DELAY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+6,0,0,8,0,1,128}, // TX_TOP_A_TX_START_DELAY_LSB_TX_START_DELAY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+7,0,0,1,0,1,0}, // TX_TOP_A_TX_CTRL_WRITE_BUFFER
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+8,0,0,3,0,1,1}, // TX_TOP_A_TX_RAMP_DURATION_TX_RAMP_DURATION
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+9,0,0,1,0,1,0}, // TX_TOP_A_GEN_CFG_0_MODULATION_TYPE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+10,1,0,1,0,1,0}, // TX_TOP_A_TEST_0_TX_ACTIVE_CTRL
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+10,0,0,1,0,1,0}, // TX_TOP_A_TEST_0_TX_ACTIVE_SEL
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+11,1,0,1,0,0,0}, // TX_TOP_A_TX_FLAG_TX_TIMEOUT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+11,0,0,1,0,0,0}, // TX_TOP_A_TX_FLAG_PKT_DONE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+12,0,0,8,0,1,0}, // TX_TOP_A_AGC_TX_BW_AGC_TX_BW
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+13,0,0,8,0,1,0}, // TX_TOP_A_AGC_TX_PWR_AGC_TX_PWR
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+14,0,0,8,0,1,0}, // TX_TOP_A_TIMEOUT_CNT_BYTE_2_TIMEOUT_CNT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+15,0,0,8,0,1,0}, // TX_TOP_A_TIMEOUT_CNT_BYTE_1_TIMEOUT_CNT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+16,0,0,8,0,1,0}, // TX_TOP_A_TIMEOUT_CNT_BYTE_0_TIMEOUT_CNT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+17,0,0,8,1,1,0}, // TX_TOP_A_TX_FSM_STATUS_TX_STATUS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+18,3,0,1,1,1,0}, // TX_TOP_A_DUMMY_CONTROL_DUMMY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+32,5,0,3,0,1,0}, // TX_TOP_A_TX_RFFE_IF_CTRL_PLL_DIV_CTRL
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+32,4,0,1,0,1,1}, // TX_TOP_A_TX_RFFE_IF_CTRL_TX_CLK_EDGE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+32,3,0,1,0,1,1}, // TX_TOP_A_TX_RFFE_IF_CTRL_TX_MODE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+32,2,0,1,0,1,0}, // TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_DST
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+32,0,0,2,0,1,0}, // TX_TOP_A_TX_RFFE_IF_CTRL_TX_IF_SRC
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+33,1,0,1,0,1,0}, // TX_TOP_A_TX_RFFE_IF_CTRL2_SX125X_IQ_INVERT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+33,0,0,1,0,1,0}, // TX_TOP_A_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+34,0,0,2,0,1,0}, // TX_TOP_A_TX_RFFE_IF_IQ_GAIN_IQ_GAIN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+35,0,0,8,0,1,0}, // TX_TOP_A_TX_RFFE_IF_I_OFFSET_I_OFFSET
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+36,0,0,8,0,1,0}, // TX_TOP_A_TX_RFFE_IF_Q_OFFSET_Q_OFFSET
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+37,0,0,8,0,1,108}, // TX_TOP_A_TX_RFFE_IF_FREQ_RF_H_FREQ_RF
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+38,0,0,8,0,1,144}, // TX_TOP_A_TX_RFFE_IF_FREQ_RF_M_FREQ_RF
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+39,0,0,8,0,1,0}, // TX_TOP_A_TX_RFFE_IF_FREQ_RF_L_FREQ_RF
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+40,0,0,4,0,1,0}, // TX_TOP_A_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+41,0,0,8,0,1,0}, // TX_TOP_A_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+42,0,0,8,0,1,64}, // TX_TOP_A_TX_RFFE_IF_TEST_MOD_FREQ
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+43,3,0,1,1,1,0}, // TX_TOP_A_DUMMY_MODULATOR_DUMMY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+64,0,0,8,0,1,15}, // TX_TOP_A_FSK_PKT_LEN_PKT_LENGTH
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+65,5,0,1,0,1,0}, // TX_TOP_A_FSK_CFG_0_TX_CONT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+65,4,0,1,0,1,0}, // TX_TOP_A_FSK_CFG_0_CRC_IBM
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+65,2,0,2,0,1,0}, // TX_TOP_A_FSK_CFG_0_DCFREE_ENC
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+65,1,0,1,0,1,1}, // TX_TOP_A_FSK_CFG_0_CRC_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+65,0,0,1,0,1,0}, // TX_TOP_A_FSK_CFG_0_PKT_MODE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+66,0,0,8,0,1,0}, // TX_TOP_A_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+67,0,0,8,0,1,20}, // TX_TOP_A_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+68,0,0,8,0,1,26}, // TX_TOP_A_FSK_BIT_RATE_MSB_BIT_RATE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+69,0,0,8,0,1,11}, // TX_TOP_A_FSK_BIT_RATE_LSB_BIT_RATE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+70,5,0,3,0,1,3}, // TX_TOP_A_FSK_MOD_FSK_REF_PATTERN_SIZE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+70,4,0,1,0,1,0}, // TX_TOP_A_FSK_MOD_FSK_PREAMBLE_SEQ
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+70,3,0,1,0,1,1}, // TX_TOP_A_FSK_MOD_FSK_REF_PATTERN_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+70,1,0,2,0,1,0}, // TX_TOP_A_FSK_MOD_FSK_GAUSSIAN_SELECT_BT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+70,0,0,1,0,1,0}, // TX_TOP_A_FSK_MOD_FSK_GAUSSIAN_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+71,0,0,8,0,1,151}, // TX_TOP_A_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+72,0,0,8,0,1,35}, // TX_TOP_A_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+73,0,0,8,0,1,82}, // TX_TOP_A_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+74,0,0,8,0,1,37}, // TX_TOP_A_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+75,0,0,8,0,1,86}, // TX_TOP_A_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+76,0,0,8,0,1,83}, // TX_TOP_A_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+77,0,0,8,0,1,101}, // TX_TOP_A_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+78,0,0,8,0,1,100}, // TX_TOP_A_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+79,3,0,1,1,1,0}, // TX_TOP_A_DUMMY_GSFK_DUMMY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+96,4,0,4,0,1,5}, // TX_TOP_A_TXRX_CFG0_0_MODEM_BW
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+96,0,0,4,0,1,7}, // TX_TOP_A_TXRX_CFG0_0_MODEM_SF
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+97,6,0,2,0,1,2}, // TX_TOP_A_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+97,4,0,2,0,1,0}, // TX_TOP_A_TXRX_CFG0_1_PPM_OFFSET
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+97,3,0,1,0,1,0}, // TX_TOP_A_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+97,0,0,3,0,1,2}, // TX_TOP_A_TXRX_CFG0_1_CODING_RATE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+98,7,0,1,0,1,0}, // TX_TOP_A_TXRX_CFG0_2_FINE_SYNCH_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+98,6,0,1,0,1,0}, // TX_TOP_A_TXRX_CFG0_2_MODEM_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+98,4,0,2,0,1,2}, // TX_TOP_A_TXRX_CFG0_2_CADRXTX
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+98,1,0,1,0,1,0}, // TX_TOP_A_TXRX_CFG0_2_IMPLICIT_HEADER
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+98,0,0,1,0,1,1}, // TX_TOP_A_TXRX_CFG0_2_CRC_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+99,0,0,8,0,1,12}, // TX_TOP_A_TXRX_CFG0_3_PAYLOAD_LENGTH
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+100,7,0,1,0,1,0}, // TX_TOP_A_TXRX_CFG1_0_INT_STEP_ORIDE_EN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+100,0,0,6,0,1,0}, // TX_TOP_A_TXRX_CFG1_0_INT_STEP_ORIDE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+101,7,0,1,0,1,0}, // TX_TOP_A_TXRX_CFG1_1_MODEM_START
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+101,6,0,1,0,1,0}, // TX_TOP_A_TXRX_CFG1_1_HEADER_DIFF_MODE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+101,0,0,6,0,1,0}, // TX_TOP_A_TXRX_CFG1_1_ZERO_PAD
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+102,0,0,8,0,1,8}, // TX_TOP_A_TXRX_CFG1_2_PREAMBLE_SYMB_NB
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+103,0,0,8,0,1,0}, // TX_TOP_A_TXRX_CFG1_3_PREAMBLE_SYMB_NB
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+104,6,0,1,0,1,1}, // TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_INT_DELAY
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+104,5,0,1,0,1,0}, // TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_RX
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+104,4,0,1,0,1,0}, // TX_TOP_A_TXRX_CFG1_4_AUTO_ACK_TX
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+105,4,0,3,0,1,0}, // TX_TOP_A_TX_CFG0_0_CHIRP_LOWPASS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+105,3,0,1,0,1,0}, // TX_TOP_A_TX_CFG0_0_PPM_OFFSET_SIG
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+105,2,0,1,0,1,1}, // TX_TOP_A_TX_CFG0_0_CONTCHIRP
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+105,1,0,1,0,1,1}, // TX_TOP_A_TX_CFG0_0_CHIRP_INVERT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+105,0,0,1,0,1,0}, // TX_TOP_A_TX_CFG0_0_CONTINUOUS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+106,0,0,6,0,1,20}, // TX_TOP_A_TX_CFG0_1_POWER_RANGING
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+107,0,0,8,0,1,0}, // TX_TOP_A_TX_CFG1_0_FRAME_NB
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+108,5,0,2,0,1,0}, // TX_TOP_A_TX_CFG1_1_HOP_CTRL
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+108,0,0,5,0,1,10}, // TX_TOP_A_TX_CFG1_1_IFS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+109,7,0,1,0,1,1}, // TX_TOP_A_FRAME_SYNCH_0_AUTO_SCALE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+109,6,0,1,0,1,0}, // TX_TOP_A_FRAME_SYNCH_0_DROP_ON_SYNCH
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+109,5,0,1,0,1,1}, // TX_TOP_A_FRAME_SYNCH_0_GAIN
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+109,0,1,5,0,1,2}, // TX_TOP_A_FRAME_SYNCH_0_PEAK1_POS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+110,7,0,1,0,1,0}, // TX_TOP_A_FRAME_SYNCH_1_FINETIME_ON_LAST
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+110,5,0,2,0,1,3}, // TX_TOP_A_FRAME_SYNCH_1_TIMEOUT_OPT
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+110,0,1,5,0,1,4}, // TX_TOP_A_FRAME_SYNCH_1_PEAK2_POS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+111,0,0,4,1,1,0}, // TX_TOP_A_LORA_TX_STATE_STATUS
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+112,2,0,1,0,0,0}, // TX_TOP_A_LORA_TX_FLAG_FRAME_DONE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+112,1,0,1,0,0,0}, // TX_TOP_A_LORA_TX_FLAG_CONT_DONE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+112,0,0,1,0,0,0}, // TX_TOP_A_LORA_TX_FLAG_PLD_DONE
    {0,SX1302_REG_TX_TOP_A_BASE_ADDR+113,3,0,1,1,1,0}, // TX_TOP_A_DUMMY_LORA_DUMMY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+0,3,0,1,0,0,0}, // TX_TOP_B_TX_TRIG_TX_FSM_CLR
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+0,2,0,1,0,1,0}, // TX_TOP_B_TX_TRIG_TX_TRIG_GPS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+0,1,0,1,0,1,0}, // TX_TOP_B_TX_TRIG_TX_TRIG_DELAYED
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+0,0,0,1,0,1,0}, // TX_TOP_B_TX_TRIG_TX_TRIG_IMMEDIATE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+1,0,0,8,0,1,0}, // TX_TOP_B_TIMER_TRIG_BYTE3_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+2,0,0,8,0,1,0}, // TX_TOP_B_TIMER_TRIG_BYTE2_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+3,0,0,8,0,1,0}, // TX_TOP_B_TIMER_TRIG_BYTE1_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+4,0,0,8,0,1,0}, // TX_TOP_B_TIMER_TRIG_BYTE0_TIMER_DELAYED_TRIG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+5,0,0,8,0,1,187}, // TX_TOP_B_TX_START_DELAY_MSB_TX_START_DELAY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+6,0,0,8,0,1,128}, // TX_TOP_B_TX_START_DELAY_LSB_TX_START_DELAY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+7,0,0,1,0,1,0}, // TX_TOP_B_TX_CTRL_WRITE_BUFFER
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+8,0,0,3,0,1,1}, // TX_TOP_B_TX_RAMP_DURATION_TX_RAMP_DURATION
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+9,0,0,1,0,1,0}, // TX_TOP_B_GEN_CFG_0_MODULATION_TYPE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+10,1,0,1,0,1,0}, // TX_TOP_B_TEST_0_TX_ACTIVE_CTRL
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+10,0,0,1,0,1,0}, // TX_TOP_B_TEST_0_TX_ACTIVE_SEL
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+11,1,0,1,0,0,0}, // TX_TOP_B_TX_FLAG_TX_TIMEOUT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+11,0,0,1,0,0,0}, // TX_TOP_B_TX_FLAG_PKT_DONE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+12,0,0,8,0,1,0}, // TX_TOP_B_AGC_TX_BW_AGC_TX_BW
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+13,0,0,8,0,1,0}, // TX_TOP_B_AGC_TX_PWR_AGC_TX_PWR
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+14,0,0,8,0,1,0}, // TX_TOP_B_TIMEOUT_CNT_BYTE_2_TIMEOUT_CNT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+15,0,0,8,0,1,0}, // TX_TOP_B_TIMEOUT_CNT_BYTE_1_TIMEOUT_CNT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+16,0,0,8,0,1,0}, // TX_TOP_B_TIMEOUT_CNT_BYTE_0_TIMEOUT_CNT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+17,0,0,8,1,1,0}, // TX_TOP_B_TX_FSM_STATUS_TX_STATUS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+18,3,0,1,1,1,0}, // TX_TOP_B_DUMMY_CONTROL_DUMMY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+32,5,0,3,0,1,0}, // TX_TOP_B_TX_RFFE_IF_CTRL_PLL_DIV_CTRL
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+32,4,0,1,0,1,1}, // TX_TOP_B_TX_RFFE_IF_CTRL_TX_CLK_EDGE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+32,3,0,1,0,1,1}, // TX_TOP_B_TX_RFFE_IF_CTRL_TX_MODE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+32,2,0,1,0,1,0}, // TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_DST
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+32,0,0,2,0,1,0}, // TX_TOP_B_TX_RFFE_IF_CTRL_TX_IF_SRC
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+33,1,0,1,0,1,0}, // TX_TOP_B_TX_RFFE_IF_CTRL2_SX125X_IQ_INVERT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+33,0,0,1,0,1,0}, // TX_TOP_B_TX_RFFE_IF_CTRL2_PLL_DIV_CTRL_AGC
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+34,0,0,2,0,1,0}, // TX_TOP_B_TX_RFFE_IF_IQ_GAIN_IQ_GAIN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+35,0,0,8,0,1,0}, // TX_TOP_B_TX_RFFE_IF_I_OFFSET_I_OFFSET
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+36,0,0,8,0,1,0}, // TX_TOP_B_TX_RFFE_IF_Q_OFFSET_Q_OFFSET
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+37,0,0,8,0,1,108}, // TX_TOP_B_TX_RFFE_IF_FREQ_RF_H_FREQ_RF
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+38,0,0,8,0,1,144}, // TX_TOP_B_TX_RFFE_IF_FREQ_RF_M_FREQ_RF
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+39,0,0,8,0,1,0}, // TX_TOP_B_TX_RFFE_IF_FREQ_RF_L_FREQ_RF
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+40,0,0,4,0,1,0}, // TX_TOP_B_TX_RFFE_IF_FREQ_DEV_H_FREQ_DEV
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+41,0,0,8,0,1,0}, // TX_TOP_B_TX_RFFE_IF_FREQ_DEV_L_FREQ_DEV
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+42,0,0,8,0,1,64}, // TX_TOP_B_TX_RFFE_IF_TEST_MOD_FREQ
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+43,3,0,1,1,1,0}, // TX_TOP_B_DUMMY_MODULATOR_DUMMY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+64,0,0,8,0,1,15}, // TX_TOP_B_FSK_PKT_LEN_PKT_LENGTH
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+65,5,0,1,0,1,0}, // TX_TOP_B_FSK_CFG_0_TX_CONT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+65,4,0,1,0,1,0}, // TX_TOP_B_FSK_CFG_0_CRC_IBM
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+65,2,0,2,0,1,0}, // TX_TOP_B_FSK_CFG_0_DCFREE_ENC
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+65,1,0,1,0,1,1}, // TX_TOP_B_FSK_CFG_0_CRC_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+65,0,0,1,0,1,0}, // TX_TOP_B_FSK_CFG_0_PKT_MODE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+66,0,0,8,0,1,0}, // TX_TOP_B_FSK_PREAMBLE_SIZE_MSB_PREAMBLE_SIZE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+67,0,0,8,0,1,20}, // TX_TOP_B_FSK_PREAMBLE_SIZE_LSB_PREAMBLE_SIZE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+68,0,0,8,0,1,26}, // TX_TOP_B_FSK_BIT_RATE_MSB_BIT_RATE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+69,0,0,8,0,1,11}, // TX_TOP_B_FSK_BIT_RATE_LSB_BIT_RATE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+70,5,0,3,0,1,3}, // TX_TOP_B_FSK_MOD_FSK_REF_PATTERN_SIZE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+70,4,0,1,0,1,0}, // TX_TOP_B_FSK_MOD_FSK_PREAMBLE_SEQ
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+70,3,0,1,0,1,1}, // TX_TOP_B_FSK_MOD_FSK_REF_PATTERN_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+70,1,0,2,0,1,0}, // TX_TOP_B_FSK_MOD_FSK_GAUSSIAN_SELECT_BT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+70,0,0,1,0,1,0}, // TX_TOP_B_FSK_MOD_FSK_GAUSSIAN_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+71,0,0,8,0,1,151}, // TX_TOP_B_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+72,0,0,8,0,1,35}, // TX_TOP_B_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+73,0,0,8,0,1,82}, // TX_TOP_B_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+74,0,0,8,0,1,37}, // TX_TOP_B_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+75,0,0,8,0,1,86}, // TX_TOP_B_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+76,0,0,8,0,1,83}, // TX_TOP_B_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+77,0,0,8,0,1,101}, // TX_TOP_B_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+78,0,0,8,0,1,100}, // TX_TOP_B_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+79,3,0,1,1,1,0}, // TX_TOP_B_DUMMY_GSFK_DUMMY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+96,4,0,4,0,1,5}, // TX_TOP_B_TXRX_CFG0_0_MODEM_BW
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+96,0,0,4,0,1,7}, // TX_TOP_B_TXRX_CFG0_0_MODEM_SF
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+97,6,0,2,0,1,2}, // TX_TOP_B_TXRX_CFG0_1_PPM_OFFSET_HDR_CTRL
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+97,4,0,2,0,1,0}, // TX_TOP_B_TXRX_CFG0_1_PPM_OFFSET
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+97,3,0,1,0,1,0}, // TX_TOP_B_TXRX_CFG0_1_POST_PREAMBLE_GAP_LONG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+97,0,0,3,0,1,2}, // TX_TOP_B_TXRX_CFG0_1_CODING_RATE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+98,7,0,1,0,1,0}, // TX_TOP_B_TXRX_CFG0_2_FINE_SYNCH_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+98,6,0,1,0,1,0}, // TX_TOP_B_TXRX_CFG0_2_MODEM_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+98,4,0,2,0,1,2}, // TX_TOP_B_TXRX_CFG0_2_CADRXTX
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+98,1,0,1,0,1,0}, // TX_TOP_B_TXRX_CFG0_2_IMPLICIT_HEADER
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+98,0,0,1,0,1,1}, // TX_TOP_B_TXRX_CFG0_2_CRC_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+99,0,0,8,0,1,12}, // TX_TOP_B_TXRX_CFG0_3_PAYLOAD_LENGTH
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+100,7,0,1,0,1,0}, // TX_TOP_B_TXRX_CFG1_0_INT_STEP_ORIDE_EN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+100,0,0,6,0,1,0}, // TX_TOP_B_TXRX_CFG1_0_INT_STEP_ORIDE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+101,7,0,1,0,1,0}, // TX_TOP_B_TXRX_CFG1_1_MODEM_START
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+101,6,0,1,0,1,0}, // TX_TOP_B_TXRX_CFG1_1_HEADER_DIFF_MODE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+101,0,0,6,0,1,0}, // TX_TOP_B_TXRX_CFG1_1_ZERO_PAD
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+102,0,0,8,0,1,8}, // TX_TOP_B_TXRX_CFG1_2_PREAMBLE_SYMB_NB
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+103,0,0,8,0,1,0}, // TX_TOP_B_TXRX_CFG1_3_PREAMBLE_SYMB_NB
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+104,6,0,1,0,1,1}, // TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_INT_DELAY
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+104,5,0,1,0,1,0}, // TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_RX
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+104,4,0,1,0,1,0}, // TX_TOP_B_TXRX_CFG1_4_AUTO_ACK_TX
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+105,4,0,3,0,1,0}, // TX_TOP_B_TX_CFG0_0_CHIRP_LOWPASS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+105,3,0,1,0,1,0}, // TX_TOP_B_TX_CFG0_0_PPM_OFFSET_SIG
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+105,2,0,1,0,1,1}, // TX_TOP_B_TX_CFG0_0_CONTCHIRP
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+105,1,0,1,0,1,1}, // TX_TOP_B_TX_CFG0_0_CHIRP_INVERT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+105,0,0,1,0,1,0}, // TX_TOP_B_TX_CFG0_0_CONTINUOUS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+106,0,0,6,0,1,20}, // TX_TOP_B_TX_CFG0_1_POWER_RANGING
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+107,0,0,8,0,1,0}, // TX_TOP_B_TX_CFG1_0_FRAME_NB
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+108,5,0,2,0,1,0}, // TX_TOP_B_TX_CFG1_1_HOP_CTRL
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+108,0,0,5,0,1,10}, // TX_TOP_B_TX_CFG1_1_IFS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+109,7,0,1,0,1,1}, // TX_TOP_B_FRAME_SYNCH_0_AUTO_SCALE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+109,6,0,1,0,1,0}, // TX_TOP_B_FRAME_SYNCH_0_DROP_ON_SYNCH
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+109,5,0,1,0,1,1}, // TX_TOP_B_FRAME_SYNCH_0_GAIN
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+109,0,1,5,0,1,2}, // TX_TOP_B_FRAME_SYNCH_0_PEAK1_POS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+110,7,0,1,0,1,0}, // TX_TOP_B_FRAME_SYNCH_1_FINETIME_ON_LAST
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+110,5,0,2,0,1,3}, // TX_TOP_B_FRAME_SYNCH_1_TIMEOUT_OPT
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+110,0,1,5,0,1,4}, // TX_TOP_B_FRAME_SYNCH_1_PEAK2_POS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+111,0,0,4,1,1,0}, // TX_TOP_B_LORA_TX_STATE_STATUS
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+112,2,0,1,0,0,0}, // TX_TOP_B_LORA_TX_FLAG_FRAME_DONE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+112,1,0,1,0,0,0}, // TX_TOP_B_LORA_TX_FLAG_CONT_DONE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+112,0,0,1,0,0,0}, // TX_TOP_B_LORA_TX_FLAG_PLD_DONE
    {0,SX1302_REG_TX_TOP_B_BASE_ADDR+113,3,0,1,1,1,0}, // TX_TOP_B_DUMMY_LORA_DUMMY
    {0,SX1302_REG_GPIO_BASE_ADDR+0,0,0,4,0,1,0}, // GPIO_GPIO_DIR_H_DIRECTION
    {0,SX1302_REG_GPIO_BASE_ADDR+1,0,0,8,0,1,0}, // GPIO_GPIO_DIR_L_DIRECTION
    {0,SX1302_REG_GPIO_BASE_ADDR+2,0,0,4,0,1,0}, // GPIO_GPIO_OUT_H_OUT_VALUE
    {0,SX1302_REG_GPIO_BASE_ADDR+3,0,0,8,0,1,0}, // GPIO_GPIO_OUT_L_OUT_VALUE
    {0,SX1302_REG_GPIO_BASE_ADDR+4,0,0,4,1,1,0}, // GPIO_GPIO_IN_H_IN_VALUE
    {0,SX1302_REG_GPIO_BASE_ADDR+5,0,0,8,1,1,0}, // GPIO_GPIO_IN_L_IN_VALUE
    {0,SX1302_REG_GPIO_BASE_ADDR+6,0,0,4,0,1,0}, // GPIO_GPIO_PD_H_PD_VALUE
    {0,SX1302_REG_GPIO_BASE_ADDR+7,0,0,8,0,1,0}, // GPIO_GPIO_PD_L_PD_VALUE
    {0,SX1302_REG_GPIO_BASE_ADDR+8,0,0,4,0,1,0}, // GPIO_GPIO_SEL_0_SELECTION
    {0,SX1302_REG_GPIO_BASE_ADDR+9,0,0,4,0,1,0}, // GPIO_GPIO_SEL_1_SELECTION
    {0,SX1302_REG_GPIO_BASE_ADDR+10,0,0,4,0,1,0}, // GPIO_GPIO_SEL_2_SELECTION
    {0,SX1302_REG_GPIO_BASE_ADDR+11,0,0,4,0,1,0}, // GPIO_GPIO_SEL_3_SELECTION
    {0,SX1302_REG_GPIO_BASE_ADDR+12,0,0,4,0,1,0}, // GPIO_GPIO_SEL_4_SELECTION
    {0,SX1302_REG_GPIO_BASE_ADDR+13,0,0,4,0,1,0}, // GPIO_GPIO_SEL_5_SELECTION
    {0,SX1302_REG_GPIO_BASE_ADDR+14,0,0,4,0,1,0}, // GPIO_GPIO_SEL_6_SELECTION
    {0,SX1302_REG_GPIO_BASE_ADDR+15,0,0,4,0,1,0}, // GPIO_GPIO_SEL_7_SELECTION
    {0,SX1302_REG_GPIO_BASE_ADDR+16,1,0,4,0,1,0}, // GPIO_GPIO_SEL_8_11_GPIO_11_9_SEL
    {0,SX1302_REG_GPIO_BASE_ADDR+16,0,0,1,0,1,0}, // GPIO_GPIO_SEL_8_11_GPIO_8_SEL
    {0,SX1302_REG_GPIO_BASE_ADDR+17,5,0,1,0,0,0}, // GPIO_HOST_IRQ_TX_TIMEOUT_B
    {0,SX1302_REG_GPIO_BASE_ADDR+17,4,0,1,0,0,0}, // GPIO_HOST_IRQ_TX_TIMEOUT_A
    {0,SX1302_REG_GPIO_BASE_ADDR+17,3,0,1,0,0,0}, // GPIO_HOST_IRQ_TX_DONE_B
    {0,SX1302_REG_GPIO_BASE_ADDR+17,2,0,1,0,0,0}, // GPIO_HOST_IRQ_TX_DONE_A
    {0,SX1302_REG_GPIO_BASE_ADDR+17,1,0,1,0,0,0}, // GPIO_HOST_IRQ_TIMESTAMP
    {0,SX1302_REG_GPIO_BASE_ADDR+17,0,0,1,0,0,0}, // GPIO_HOST_IRQ_RX_BUFFER_WATERMARK
    {0,SX1302_REG_GPIO_BASE_ADDR+18,5,0,1,0,1,0}, // GPIO_HOST_IRQ_EN_TX_TIMEOUT_B
    {0,SX1302_REG_GPIO_BASE_ADDR+18,4,0,1,0,1,0}, // GPIO_HOST_IRQ_EN_TX_TIMEOUT_A
    {0,SX1302_REG_GPIO_BASE_ADDR+18,3,0,1,0,1,0}, // GPIO_HOST_IRQ_EN_TX_DONE_B
    {0,SX1302_REG_GPIO_BASE_ADDR+18,2,0,1,0,1,0}, // GPIO_HOST_IRQ_EN_TX_DONE_A
    {0,SX1302_REG_GPIO_BASE_ADDR+18,1,0,1,0,1,0}, // GPIO_HOST_IRQ_EN_TIMESTAMP
    {0,SX1302_REG_GPIO_BASE_ADDR+18,0,0,1,0,1,0}, // GPIO_HOST_IRQ_EN_RX_BUFFER_WATERMARK
    {0,SX1302_REG_GPIO_BASE_ADDR+19,0,0,1,1,1,0}, // GPIO_DUMMY_DUMMY
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+0,1,0,1,0,1,0}, // TIMESTAMP_GPS_CTRL_GPS_POL
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+0,0,0,1,0,1,0}, // TIMESTAMP_GPS_CTRL_GPS_EN
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+1,0,0,8,1,1,0}, // TIMESTAMP_TIMESTAMP_PPS_MSB2_TIMESTAMP_PPS
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+2,0,0,8,1,1,0}, // TIMESTAMP_TIMESTAMP_PPS_MSB1_TIMESTAMP_PPS
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+3,0,0,8,1,1,0}, // TIMESTAMP_TIMESTAMP_PPS_LSB2_TIMESTAMP_PPS
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+4,0,0,8,1,1,0}, // TIMESTAMP_TIMESTAMP_PPS_LSB1_TIMESTAMP_PPS
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+5,0,0,8,1,1,0}, // TIMESTAMP_TIMESTAMP_MSB2_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+6,0,0,8,1,1,0}, // TIMESTAMP_TIMESTAMP_MSB1_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+7,0,0,8,1,1,0}, // TIMESTAMP_TIMESTAMP_LSB2_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+8,0,0,8,1,1,0}, // TIMESTAMP_TIMESTAMP_LSB1_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+9,0,0,8,0,1,0}, // TIMESTAMP_TIMESTAMP_SET3_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+10,0,0,8,0,1,0}, // TIMESTAMP_TIMESTAMP_SET2_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+11,0,0,8,0,1,0}, // TIMESTAMP_TIMESTAMP_SET1_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+12,0,0,8,0,1,0}, // TIMESTAMP_TIMESTAMP_SET0_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+13,0,0,8,0,1,0}, // TIMESTAMP_TIMESTAMP_IRQ_3_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+14,0,0,8,0,1,0}, // TIMESTAMP_TIMESTAMP_IRQ_2_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+15,0,0,8,0,1,0}, // TIMESTAMP_TIMESTAMP_IRQ_1_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+16,0,0,8,0,1,0}, // TIMESTAMP_TIMESTAMP_IRQ_0_TIMESTAMP
    {0,SX1302_REG_TIMESTAMP_BASE_ADDR+17,0,0,1,1,1,0}, // TIMESTAMP_DUMMY_DUMMY
    {0,SX1302_REG_RX_TOP_BASE_ADDR+0,0,0,5,0,1,0}, // RX_TOP_FREQ_0_MSB_IF_FREQ_0
    {0,SX1302_REG_RX_TOP_BASE_ADDR+1,0,0,8,0,1,128}, // RX_TOP_FREQ_0_LSB_IF_FREQ_0
    {0,SX1302_REG_RX_TOP_BASE_ADDR+2,0,0,5,0,1,1}, // RX_TOP_FREQ_1_MSB_IF_FREQ_1
    {0,SX1302_REG_RX_TOP_BASE_ADDR+3,0,0,8,0,1,128}, // RX_TOP_FREQ_1_LSB_IF_FREQ_1
    {0,SX1302_REG_RX_TOP_BASE_ADDR+4,0,0,5,0,1,30}, // RX_TOP_FREQ_2_MSB_IF_FREQ_2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+5,0,0,8,0,1,128}, // RX_TOP_FREQ_2_LSB_IF_FREQ_2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+6,0,0,5,0,1,0}, // RX_TOP_FREQ_3_MSB_IF_FREQ_3
    {0,SX1302_REG_RX_TOP_BASE_ADDR+7,0,0,8,0,1,128}, // RX_TOP_FREQ_3_LSB_IF_FREQ_3
    {0,SX1302_REG_RX_TOP_BASE_ADDR+8,0,0,5,0,1,0}, // RX_TOP_FREQ_4_MSB_IF_FREQ_4
    {0,SX1302_REG_RX_TOP_BASE_ADDR+9,0,0,8,0,1,50}, // RX_TOP_FREQ_4_LSB_IF_FREQ_4
    {0,SX1302_REG_RX_TOP_BASE_ADDR+10,0,0,5,0,1,0}, // RX_TOP_FREQ_5_MSB_IF_FREQ_5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+11,0,0,8,0,1,60}, // RX_TOP_FREQ_5_LSB_IF_FREQ_5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+12,0,0,5,0,1,0}, // RX_TOP_FREQ_6_MSB_IF_FREQ_6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+13,0,0,8,0,1,70}, // RX_TOP_FREQ_6_LSB_IF_FREQ_6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+14,0,0,5,0,1,0}, // RX_TOP_FREQ_7_MSB_IF_FREQ_7
    {0,SX1302_REG_RX_TOP_BASE_ADDR+15,0,0,8,0,1,80}, // RX_TOP_FREQ_7_LSB_IF_FREQ_7
    {0,SX1302_REG_RX_TOP_BASE_ADDR+16,0,0,8,0,1,0}, // RX_TOP_RADIO_SELECT_RADIO_SELECT
    {0,SX1302_REG_RX_TOP_BASE_ADDR+17,3,0,5,0,1,7}, // RX_TOP_RSSI_CONTROL_RSSI_FILTER_ALPHA
    {0,SX1302_REG_RX_TOP_BASE_ADDR+17,0,0,3,0,1,0}, // RX_TOP_RSSI_CONTROL_SELECT_RSSI
    {0,SX1302_REG_RX_TOP_BASE_ADDR+18,0,0,8,0,1,0}, // RX_TOP_RSSI_DEF_VALUE_CHAN_RSSI_DEF_VALUE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+19,0,0,8,0,1,0}, // RX_TOP_CHANN_DAGC_CFG1_CHAN_DAGC_THRESHOLD_HIGH
    {0,SX1302_REG_RX_TOP_BASE_ADDR+20,0,0,8,0,1,0}, // RX_TOP_CHANN_DAGC_CFG2_CHAN_DAGC_THRESHOLD_LOW
    {0,SX1302_REG_RX_TOP_BASE_ADDR+21,4,0,4,0,1,15}, // RX_TOP_CHANN_DAGC_CFG3_CHAN_DAGC_MAX_ATTEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+21,0,0,4,0,1,0}, // RX_TOP_CHANN_DAGC_CFG3_CHAN_DAGC_MIN_ATTEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+22,0,0,4,0,1,0}, // RX_TOP_CHANN_DAGC_CFG4_CHAN_DAGC_STEP
    {0,SX1302_REG_RX_TOP_BASE_ADDR+23,0,0,2,0,1,0}, // RX_TOP_CHANN_DAGC_CFG5_CHAN_DAGC_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+24,0,0,8,1,1,0}, // RX_TOP_RSSI_VALUE_CHAN_RSSI
    {0,SX1302_REG_RX_TOP_BASE_ADDR+25,4,0,1,0,1,0}, // RX_TOP_GAIN_CONTROL_CHAN_GAIN_VALID
    {0,SX1302_REG_RX_TOP_BASE_ADDR+25,0,0,4,0,1,0}, // RX_TOP_GAIN_CONTROL_CHAN_GAIN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+26,0,0,1,0,1,1}, // RX_TOP_CLK_CONTROL_CHAN_CLK_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+27,0,0,1,1,1,0}, // RX_TOP_DUMMY0_DUMMY0
    {0,SX1302_REG_RX_TOP_BASE_ADDR+32,0,0,8,0,1,255}, // RX_TOP_CORR_CLOCK_ENABLE_CLK_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+33,0,0,8,0,1,0}, // RX_TOP_CORRELATOR_EN_CORR_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+34,0,0,8,0,1,255}, // RX_TOP_CORRELATOR_SF_EN_CORR_SF_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+35,0,0,8,0,1,255}, // RX_TOP_CORRELATOR_ENABLE_ONLY_FIRST_DET_EDGE_ENABLE_ONLY_FIRST_DET_EDGE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+36,0,0,8,0,1,255}, // RX_TOP_CORRELATOR_ENABLE_ACC_CLEAR_ENABLE_CORR_ACC_CLEAR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+37,6,0,2,0,1,2}, // RX_TOP_SF5_CFG1_ACC_WIN_LEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+37,5,0,1,0,1,1}, // RX_TOP_SF5_CFG1_ACC_PEAK_SUM_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+37,4,0,1,0,1,1}, // RX_TOP_SF5_CFG1_ACC_PEAK_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+37,2,0,2,0,1,2}, // RX_TOP_SF5_CFG1_ACC_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+37,1,0,1,0,1,1}, // RX_TOP_SF5_CFG1_ACC_AUTO_RESCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+37,0,0,1,0,1,1}, // RX_TOP_SF5_CFG1_ACC_2_SAME_PEAKS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+38,7,0,1,0,1,0}, // RX_TOP_SF5_CFG2_ACC_MIN2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+38,0,0,7,0,1,55}, // RX_TOP_SF5_CFG2_ACC_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+39,0,0,8,0,1,11}, // RX_TOP_SF5_CFG3_MIN_SINGLE_PEAK
    {0,SX1302_REG_RX_TOP_BASE_ADDR+40,0,0,7,0,1,32}, // RX_TOP_SF5_CFG4_MSP_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+41,0,0,7,0,1,48}, // RX_TOP_SF5_CFG5_MSP2_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+42,3,0,3,0,1,5}, // RX_TOP_SF5_CFG6_MSP_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+42,1,0,2,0,1,1}, // RX_TOP_SF5_CFG6_MSP_CNT_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+42,0,0,1,0,1,1}, // RX_TOP_SF5_CFG6_MSP_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+43,2,0,3,0,1,5}, // RX_TOP_SF5_CFG7_MSP2_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+43,0,0,2,0,1,2}, // RX_TOP_SF5_CFG7_NOISE_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+44,6,0,2,0,1,2}, // RX_TOP_SF6_CFG1_ACC_WIN_LEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+44,5,0,1,0,1,1}, // RX_TOP_SF6_CFG1_ACC_PEAK_SUM_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+44,4,0,1,0,1,1}, // RX_TOP_SF6_CFG1_ACC_PEAK_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+44,2,0,2,0,1,2}, // RX_TOP_SF6_CFG1_ACC_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+44,1,0,1,0,1,1}, // RX_TOP_SF6_CFG1_ACC_AUTO_RESCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+44,0,0,1,0,1,1}, // RX_TOP_SF6_CFG1_ACC_2_SAME_PEAKS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+45,7,0,1,0,1,0}, // RX_TOP_SF6_CFG2_ACC_MIN2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+45,0,0,7,0,1,55}, // RX_TOP_SF6_CFG2_ACC_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+46,0,0,8,0,1,11}, // RX_TOP_SF6_CFG3_MIN_SINGLE_PEAK
    {0,SX1302_REG_RX_TOP_BASE_ADDR+47,0,0,7,0,1,32}, // RX_TOP_SF6_CFG4_MSP_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+48,0,0,7,0,1,48}, // RX_TOP_SF6_CFG5_MSP2_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+49,3,0,3,0,1,4}, // RX_TOP_SF6_CFG6_MSP_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+49,1,0,2,0,1,1}, // RX_TOP_SF6_CFG6_MSP_CNT_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+49,0,0,1,0,1,1}, // RX_TOP_SF6_CFG6_MSP_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+50,2,0,3,0,1,5}, // RX_TOP_SF6_CFG7_MSP2_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+50,0,0,2,0,1,2}, // RX_TOP_SF6_CFG7_NOISE_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+51,6,0,2,0,1,2}, // RX_TOP_SF7_CFG1_ACC_WIN_LEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+51,5,0,1,0,1,1}, // RX_TOP_SF7_CFG1_ACC_PEAK_SUM_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+51,4,0,1,0,1,1}, // RX_TOP_SF7_CFG1_ACC_PEAK_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+51,2,0,2,0,1,2}, // RX_TOP_SF7_CFG1_ACC_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+51,1,0,1,0,1,1}, // RX_TOP_SF7_CFG1_ACC_AUTO_RESCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+51,0,0,1,0,1,1}, // RX_TOP_SF7_CFG1_ACC_2_SAME_PEAKS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+52,7,0,1,0,1,0}, // RX_TOP_SF7_CFG2_ACC_MIN2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+52,0,0,7,0,1,55}, // RX_TOP_SF7_CFG2_ACC_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+53,0,0,8,0,1,11}, // RX_TOP_SF7_CFG3_MIN_SINGLE_PEAK
    {0,SX1302_REG_RX_TOP_BASE_ADDR+54,0,0,7,0,1,32}, // RX_TOP_SF7_CFG4_MSP_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+55,0,0,7,0,1,48}, // RX_TOP_SF7_CFG5_MSP2_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+56,3,0,3,0,1,3}, // RX_TOP_SF7_CFG6_MSP_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+56,1,0,2,0,1,1}, // RX_TOP_SF7_CFG6_MSP_CNT_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+56,0,0,1,0,1,1}, // RX_TOP_SF7_CFG6_MSP_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+57,2,0,3,0,1,5}, // RX_TOP_SF7_CFG7_MSP2_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+57,0,0,2,0,1,2}, // RX_TOP_SF7_CFG7_NOISE_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+58,6,0,2,0,1,2}, // RX_TOP_SF8_CFG1_ACC_WIN_LEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+58,5,0,1,0,1,1}, // RX_TOP_SF8_CFG1_ACC_PEAK_SUM_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+58,4,0,1,0,1,1}, // RX_TOP_SF8_CFG1_ACC_PEAK_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+58,2,0,2,0,1,2}, // RX_TOP_SF8_CFG1_ACC_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+58,1,0,1,0,1,1}, // RX_TOP_SF8_CFG1_ACC_AUTO_RESCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+58,0,0,1,0,1,1}, // RX_TOP_SF8_CFG1_ACC_2_SAME_PEAKS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+59,7,0,1,0,1,0}, // RX_TOP_SF8_CFG2_ACC_MIN2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+59,0,0,7,0,1,56}, // RX_TOP_SF8_CFG2_ACC_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+60,0,0,8,0,1,11}, // RX_TOP_SF8_CFG3_MIN_SINGLE_PEAK
    {0,SX1302_REG_RX_TOP_BASE_ADDR+61,0,0,7,0,1,32}, // RX_TOP_SF8_CFG4_MSP_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+62,0,0,7,0,1,48}, // RX_TOP_SF8_CFG5_MSP2_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+63,3,0,3,0,1,3}, // RX_TOP_SF8_CFG6_MSP_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+63,1,0,2,0,1,1}, // RX_TOP_SF8_CFG6_MSP_CNT_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+63,0,0,1,0,1,1}, // RX_TOP_SF8_CFG6_MSP_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+64,2,0,3,0,1,5}, // RX_TOP_SF8_CFG7_MSP2_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+64,0,0,2,0,1,2}, // RX_TOP_SF8_CFG7_NOISE_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+65,6,0,2,0,1,2}, // RX_TOP_SF9_CFG1_ACC_WIN_LEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+65,5,0,1,0,1,1}, // RX_TOP_SF9_CFG1_ACC_PEAK_SUM_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+65,4,0,1,0,1,1}, // RX_TOP_SF9_CFG1_ACC_PEAK_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+65,2,0,2,0,1,2}, // RX_TOP_SF9_CFG1_ACC_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+65,1,0,1,0,1,1}, // RX_TOP_SF9_CFG1_ACC_AUTO_RESCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+65,0,0,1,0,1,1}, // RX_TOP_SF9_CFG1_ACC_2_SAME_PEAKS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+66,7,0,1,0,1,0}, // RX_TOP_SF9_CFG2_ACC_MIN2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+66,0,0,7,0,1,58}, // RX_TOP_SF9_CFG2_ACC_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+67,0,0,8,0,1,11}, // RX_TOP_SF9_CFG3_MIN_SINGLE_PEAK
    {0,SX1302_REG_RX_TOP_BASE_ADDR+68,0,0,7,0,1,32}, // RX_TOP_SF9_CFG4_MSP_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+69,0,0,7,0,1,48}, // RX_TOP_SF9_CFG5_MSP2_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+70,3,0,3,0,1,3}, // RX_TOP_SF9_CFG6_MSP_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+70,1,0,2,0,1,1}, // RX_TOP_SF9_CFG6_MSP_CNT_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+70,0,0,1,0,1,1}, // RX_TOP_SF9_CFG6_MSP_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+71,2,0,3,0,1,5}, // RX_TOP_SF9_CFG7_MSP2_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+71,0,0,2,0,1,2}, // RX_TOP_SF9_CFG7_NOISE_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+72,6,0,2,0,1,2}, // RX_TOP_SF10_CFG1_ACC_WIN_LEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+72,5,0,1,0,1,1}, // RX_TOP_SF10_CFG1_ACC_PEAK_SUM_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+72,4,0,1,0,1,1}, // RX_TOP_SF10_CFG1_ACC_PEAK_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+72,2,0,2,0,1,2}, // RX_TOP_SF10_CFG1_ACC_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+72,1,0,1,0,1,1}, // RX_TOP_SF10_CFG1_ACC_AUTO_RESCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+72,0,0,1,0,1,1}, // RX_TOP_SF10_CFG1_ACC_2_SAME_PEAKS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+73,7,0,1,0,1,0}, // RX_TOP_SF10_CFG2_ACC_MIN2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+73,0,0,7,0,1,60}, // RX_TOP_SF10_CFG2_ACC_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+74,0,0,8,0,1,11}, // RX_TOP_SF10_CFG3_MIN_SINGLE_PEAK
    {0,SX1302_REG_RX_TOP_BASE_ADDR+75,0,0,7,0,1,32}, // RX_TOP_SF10_CFG4_MSP_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+76,0,0,7,0,1,48}, // RX_TOP_SF10_CFG5_MSP2_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+77,3,0,3,0,1,3}, // RX_TOP_SF10_CFG6_MSP_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+77,1,0,2,0,1,1}, // RX_TOP_SF10_CFG6_MSP_CNT_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+77,0,0,1,0,1,1}, // RX_TOP_SF10_CFG6_MSP_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+78,2,0,3,0,1,5}, // RX_TOP_SF10_CFG7_MSP2_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+78,0,0,2,0,1,2}, // RX_TOP_SF10_CFG7_NOISE_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+79,6,0,2,0,1,2}, // RX_TOP_SF11_CFG1_ACC_WIN_LEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+79,5,0,1,0,1,1}, // RX_TOP_SF11_CFG1_ACC_PEAK_SUM_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+79,4,0,1,0,1,1}, // RX_TOP_SF11_CFG1_ACC_PEAK_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+79,2,0,2,0,1,2}, // RX_TOP_SF11_CFG1_ACC_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+79,1,0,1,0,1,1}, // RX_TOP_SF11_CFG1_ACC_AUTO_RESCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+79,0,0,1,0,1,1}, // RX_TOP_SF11_CFG1_ACC_2_SAME_PEAKS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+80,7,0,1,0,1,0}, // RX_TOP_SF11_CFG2_ACC_MIN2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+80,0,0,7,0,1,60}, // RX_TOP_SF11_CFG2_ACC_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+81,0,0,8,0,1,11}, // RX_TOP_SF11_CFG3_MIN_SINGLE_PEAK
    {0,SX1302_REG_RX_TOP_BASE_ADDR+82,0,0,7,0,1,32}, // RX_TOP_SF11_CFG4_MSP_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+83,0,0,7,0,1,48}, // RX_TOP_SF11_CFG5_MSP2_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+84,3,0,3,0,1,3}, // RX_TOP_SF11_CFG6_MSP_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+84,1,0,2,0,1,1}, // RX_TOP_SF11_CFG6_MSP_CNT_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+84,0,0,1,0,1,1}, // RX_TOP_SF11_CFG6_MSP_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+85,2,0,3,0,1,5}, // RX_TOP_SF11_CFG7_MSP2_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+85,0,0,2,0,1,2}, // RX_TOP_SF11_CFG7_NOISE_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+86,6,0,2,0,1,2}, // RX_TOP_SF12_CFG1_ACC_WIN_LEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+86,5,0,1,0,1,1}, // RX_TOP_SF12_CFG1_ACC_PEAK_SUM_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+86,4,0,1,0,1,1}, // RX_TOP_SF12_CFG1_ACC_PEAK_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+86,2,0,2,0,1,2}, // RX_TOP_SF12_CFG1_ACC_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+86,1,0,1,0,1,1}, // RX_TOP_SF12_CFG1_ACC_AUTO_RESCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+86,0,0,1,0,1,1}, // RX_TOP_SF12_CFG1_ACC_2_SAME_PEAKS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+87,7,0,1,0,1,0}, // RX_TOP_SF12_CFG2_ACC_MIN2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+87,0,0,7,0,1,60}, // RX_TOP_SF12_CFG2_ACC_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+88,0,0,8,0,1,11}, // RX_TOP_SF12_CFG3_MIN_SINGLE_PEAK
    {0,SX1302_REG_RX_TOP_BASE_ADDR+89,0,0,7,0,1,32}, // RX_TOP_SF12_CFG4_MSP_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+90,0,0,7,0,1,48}, // RX_TOP_SF12_CFG5_MSP2_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+91,3,0,3,0,1,3}, // RX_TOP_SF12_CFG6_MSP_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+91,1,0,2,0,1,1}, // RX_TOP_SF12_CFG6_MSP_CNT_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+91,0,0,1,0,1,1}, // RX_TOP_SF12_CFG6_MSP_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+92,2,0,3,0,1,5}, // RX_TOP_SF12_CFG7_MSP2_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+92,0,0,2,0,1,2}, // RX_TOP_SF12_CFG7_NOISE_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+93,0,0,1,1,1,0}, // RX_TOP_DUMMY1_DUMMY1
    {0,SX1302_REG_RX_TOP_BASE_ADDR+96,4,0,3,0,1,0}, // RX_TOP_DC_NOTCH_CFG1_BW_START
    {0,SX1302_REG_RX_TOP_BASE_ADDR+96,3,0,1,0,1,1}, // RX_TOP_DC_NOTCH_CFG1_AUTO_BW_RED
    {0,SX1302_REG_RX_TOP_BASE_ADDR+96,2,0,1,0,1,0}, // RX_TOP_DC_NOTCH_CFG1_NO_FAST_START
    {0,SX1302_REG_RX_TOP_BASE_ADDR+96,1,0,1,0,1,0}, // RX_TOP_DC_NOTCH_CFG1_BYPASS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+96,0,0,1,0,1,0}, // RX_TOP_DC_NOTCH_CFG1_ENABLE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+97,3,0,3,0,1,1}, // RX_TOP_DC_NOTCH_CFG2_BW_LOCKED
    {0,SX1302_REG_RX_TOP_BASE_ADDR+97,0,0,3,0,1,5}, // RX_TOP_DC_NOTCH_CFG2_BW
    {0,SX1302_REG_RX_TOP_BASE_ADDR+98,0,0,3,0,1,0}, // RX_TOP_DC_NOTCH_CFG3_BW_RED
    {0,SX1302_REG_RX_TOP_BASE_ADDR+99,0,0,8,0,1,0}, // RX_TOP_DC_NOTCH_CFG4_IIR_DCC_TIME
    {0,SX1302_REG_RX_TOP_BASE_ADDR+100,0,1,8,0,1,2}, // RX_TOP_RX_DFE_FIR1_0_FIR1_COEFF_0
    {0,SX1302_REG_RX_TOP_BASE_ADDR+101,0,1,8,0,1,3}, // RX_TOP_RX_DFE_FIR1_1_FIR1_COEFF_1
    {0,SX1302_REG_RX_TOP_BASE_ADDR+102,0,1,8,0,1,2}, // RX_TOP_RX_DFE_FIR1_2_FIR1_COEFF_2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+103,0,1,8,0,1,3}, // RX_TOP_RX_DFE_FIR1_3_FIR1_COEFF_3
    {0,SX1302_REG_RX_TOP_BASE_ADDR+104,0,1,8,0,1,5}, // RX_TOP_RX_DFE_FIR1_4_FIR1_COEFF_4
    {0,SX1302_REG_RX_TOP_BASE_ADDR+105,0,1,8,0,1,8}, // RX_TOP_RX_DFE_FIR1_5_FIR1_COEFF_5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+106,0,1,8,0,1,6}, // RX_TOP_RX_DFE_FIR1_6_FIR1_COEFF_6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+107,0,1,8,0,1,4}, // RX_TOP_RX_DFE_FIR1_7_FIR1_COEFF_7
    {0,SX1302_REG_RX_TOP_BASE_ADDR+108,0,1,8,0,1,2}, // RX_TOP_RX_DFE_FIR2_0_FIR2_COEFF_0
    {0,SX1302_REG_RX_TOP_BASE_ADDR+109,0,1,8,0,1,-2}, // RX_TOP_RX_DFE_FIR2_1_FIR2_COEFF_1
    {0,SX1302_REG_RX_TOP_BASE_ADDR+110,0,1,8,0,1,-4}, // RX_TOP_RX_DFE_FIR2_2_FIR2_COEFF_2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+111,0,1,8,0,1,-3}, // RX_TOP_RX_DFE_FIR2_3_FIR2_COEFF_3
    {0,SX1302_REG_RX_TOP_BASE_ADDR+112,0,1,8,0,1,3}, // RX_TOP_RX_DFE_FIR2_4_FIR2_COEFF_4
    {0,SX1302_REG_RX_TOP_BASE_ADDR+113,0,1,8,0,1,11}, // RX_TOP_RX_DFE_FIR2_5_FIR2_COEFF_5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+114,0,1,8,0,1,19}, // RX_TOP_RX_DFE_FIR2_6_FIR2_COEFF_6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+115,0,1,8,0,1,10}, // RX_TOP_RX_DFE_FIR2_7_FIR2_COEFF_7
    {0,SX1302_REG_RX_TOP_BASE_ADDR+116,7,0,1,0,1,0}, // RX_TOP_RX_DFE_AGC0_RADIO_GAIN_RED_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+116,0,0,7,0,1,0}, // RX_TOP_RX_DFE_AGC0_RADIO_GAIN_RED_DB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+117,4,0,1,0,1,0}, // RX_TOP_RX_DFE_AGC1_DC_COMP_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+117,3,0,1,0,1,0}, // RX_TOP_RX_DFE_AGC1_FORCE_DEFAULT_FIR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+117,2,0,1,0,1,1}, // RX_TOP_RX_DFE_AGC1_RSSI_EARLY_LATCH
    {0,SX1302_REG_RX_TOP_BASE_ADDR+117,0,0,2,0,1,3}, // RX_TOP_RX_DFE_AGC1_FREEZE_ON_SYNC
    {0,SX1302_REG_RX_TOP_BASE_ADDR+118,6,0,1,0,1,0}, // RX_TOP_RX_DFE_AGC2_DAGC_IN_COMP
    {0,SX1302_REG_RX_TOP_BASE_ADDR+118,5,0,1,0,1,1}, // RX_TOP_RX_DFE_AGC2_DAGC_FIR_HYST
    {0,SX1302_REG_RX_TOP_BASE_ADDR+118,3,0,2,0,1,1}, // RX_TOP_RX_DFE_AGC2_RSSI_MAX_SAMPLE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+118,0,0,3,0,1,2}, // RX_TOP_RX_DFE_AGC2_RSSI_MIN_SAMPLE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+119,7,0,1,0,1,0}, // RX_TOP_RX_DFE_GAIN0_DAGC_FIR_FAST
    {0,SX1302_REG_RX_TOP_BASE_ADDR+119,6,0,1,0,1,0}, // RX_TOP_RX_DFE_GAIN0_FORCE_GAIN_FIR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+119,4,0,2,0,1,3}, // RX_TOP_RX_DFE_GAIN0_GAIN_FIR1
    {0,SX1302_REG_RX_TOP_BASE_ADDR+119,0,0,3,0,1,1}, // RX_TOP_RX_DFE_GAIN0_GAIN_FIR2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+120,6,0,2,0,1,0}, // RX_TOP_DAGC_CFG_TARGET_LVL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+120,5,0,1,0,1,0}, // RX_TOP_DAGC_CFG_GAIN_INCR_STEP
    {0,SX1302_REG_RX_TOP_BASE_ADDR+120,4,0,1,0,1,0}, // RX_TOP_DAGC_CFG_GAIN_DROP_COMP
    {0,SX1302_REG_RX_TOP_BASE_ADDR+120,3,0,1,0,1,1}, // RX_TOP_DAGC_CFG_COMB_FILTER_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+120,2,0,1,0,1,0}, // RX_TOP_DAGC_CFG_NO_FREEZE_START
    {0,SX1302_REG_RX_TOP_BASE_ADDR+120,0,0,2,0,1,3}, // RX_TOP_DAGC_CFG_FREEZE_ON_SYNC
    {0,SX1302_REG_RX_TOP_BASE_ADDR+121,0,0,8,0,1,60}, // RX_TOP_DAGC_CNT0_SAMPLE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+122,0,0,8,0,1,6}, // RX_TOP_DAGC_CNT1_THR_M6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+123,0,0,8,0,1,25}, // RX_TOP_DAGC_CNT2_THR_M12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+124,0,0,8,0,1,42}, // RX_TOP_DAGC_CNT3_THR_M18
    {0,SX1302_REG_RX_TOP_BASE_ADDR+125,4,0,4,0,1,8}, // RX_TOP_DAGC_CNT4_GAIN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+125,0,0,1,0,1,0}, // RX_TOP_DAGC_CNT4_FORCE_GAIN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+126,6,0,2,0,1,2}, // RX_TOP_TXRX_CFG1_PPM_OFFSET_HDR_CTRL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+126,4,0,2,0,1,0}, // RX_TOP_TXRX_CFG1_PPM_OFFSET
    {0,SX1302_REG_RX_TOP_BASE_ADDR+126,3,0,1,0,1,0}, // RX_TOP_TXRX_CFG1_MODEM_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+126,0,0,3,0,1,2}, // RX_TOP_TXRX_CFG1_CODING_RATE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+127,4,0,1,0,0,0}, // RX_TOP_TXRX_CFG2_MODEM_START
    {0,SX1302_REG_RX_TOP_BASE_ADDR+127,2,0,2,0,1,1}, // RX_TOP_TXRX_CFG2_CADRXTX
    {0,SX1302_REG_RX_TOP_BASE_ADDR+127,1,0,1,0,1,0}, // RX_TOP_TXRX_CFG2_IMPLICIT_HEADER
    {0,SX1302_REG_RX_TOP_BASE_ADDR+127,0,0,1,0,1,1}, // RX_TOP_TXRX_CFG2_CRC_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+128,0,0,8,0,1,12}, // RX_TOP_TXRX_CFG3_PAYLOAD_LENGTH
    {0,SX1302_REG_RX_TOP_BASE_ADDR+129,7,0,1,0,1,0}, // RX_TOP_TXRX_CFG4_INT_STEP_ORIDE_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+129,0,0,6,0,1,0}, // RX_TOP_TXRX_CFG4_INT_STEP_ORIDE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+130,6,0,1,0,1,0}, // RX_TOP_TXRX_CFG5_HEADER_DIFF_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+130,0,0,6,0,1,0}, // RX_TOP_TXRX_CFG5_ZERO_PAD
    {0,SX1302_REG_RX_TOP_BASE_ADDR+131,0,0,8,0,1,8}, // RX_TOP_TXRX_CFG6_PREAMBLE_SYMB_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+132,0,0,8,0,1,0}, // RX_TOP_TXRX_CFG7_PREAMBLE_SYMB_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+133,3,0,1,0,1,1}, // RX_TOP_TXRX_CFG8_AUTO_ACK_INT_DELAY
    {0,SX1302_REG_RX_TOP_BASE_ADDR+133,2,0,1,0,1,0}, // RX_TOP_TXRX_CFG8_AUTO_ACK_RX
    {0,SX1302_REG_RX_TOP_BASE_ADDR+133,1,0,1,0,1,0}, // RX_TOP_TXRX_CFG8_AUTO_ACK_TX
    {0,SX1302_REG_RX_TOP_BASE_ADDR+133,0,0,1,0,1,0}, // RX_TOP_TXRX_CFG8_POST_PREAMBLE_GAP_LONG
    {0,SX1302_REG_RX_TOP_BASE_ADDR+134,7,0,1,0,1,0}, // RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+134,6,0,1,0,1,0}, // RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF11
    {0,SX1302_REG_RX_TOP_BASE_ADDR+134,5,0,1,0,1,0}, // RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF10
    {0,SX1302_REG_RX_TOP_BASE_ADDR+134,4,0,1,0,1,0}, // RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF9
    {0,SX1302_REG_RX_TOP_BASE_ADDR+134,3,0,1,0,1,0}, // RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF8
    {0,SX1302_REG_RX_TOP_BASE_ADDR+134,2,0,1,0,1,0}, // RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF7
    {0,SX1302_REG_RX_TOP_BASE_ADDR+134,1,0,1,0,1,1}, // RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+134,0,0,1,0,1,1}, // RX_TOP_TXRX_CFG9_FINE_SYNCH_EN_SF5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+135,4,0,2,0,1,3}, // RX_TOP_RX_CFG0_DFT_PEAK_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+135,2,0,1,0,1,1}, // RX_TOP_RX_CFG0_CHIRP_INVERT
    {0,SX1302_REG_RX_TOP_BASE_ADDR+135,1,0,1,0,1,0}, // RX_TOP_RX_CFG0_SWAP_IQ
    {0,SX1302_REG_RX_TOP_BASE_ADDR+135,0,0,1,0,1,0}, // RX_TOP_RX_CFG0_CONTINUOUS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+136,0,0,8,0,1,0}, // RX_TOP_RX_CFG1_DETECT_TIMEOUT
    {0,SX1302_REG_RX_TOP_BASE_ADDR+137,4,0,1,0,1,1}, // RX_TOP_RX_CFG2_CLK_EN_RESYNC_DIN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+137,0,0,4,0,1,11}, // RX_TOP_RX_CFG2_LLR_SCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+138,0,1,5,0,1,2}, // RX_TOP_FRAME_SYNCH0_SF5_PEAK1_POS_SF5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+139,0,1,5,0,1,4}, // RX_TOP_FRAME_SYNCH1_SF5_PEAK2_POS_SF5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+140,0,1,5,0,1,2}, // RX_TOP_FRAME_SYNCH0_SF6_PEAK1_POS_SF6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+141,0,1,5,0,1,4}, // RX_TOP_FRAME_SYNCH1_SF6_PEAK2_POS_SF6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+142,0,1,5,0,1,2}, // RX_TOP_FRAME_SYNCH0_SF7TO12_PEAK1_POS_SF7TO12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+143,0,1,5,0,1,4}, // RX_TOP_FRAME_SYNCH1_SF7TO12_PEAK2_POS_SF7TO12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+144,5,0,1,0,1,0}, // RX_TOP_FRAME_SYNCH2_FINETIME_ON_LAST
    {0,SX1302_REG_RX_TOP_BASE_ADDR+144,4,0,1,0,1,1}, // RX_TOP_FRAME_SYNCH2_AUTO_SCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+144,3,0,1,0,1,0}, // RX_TOP_FRAME_SYNCH2_DROP_ON_SYNCH
    {0,SX1302_REG_RX_TOP_BASE_ADDR+144,2,0,1,0,1,1}, // RX_TOP_FRAME_SYNCH2_GAIN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+144,0,0,2,0,1,3}, // RX_TOP_FRAME_SYNCH2_TIMEOUT_OPT
    {0,SX1302_REG_RX_TOP_BASE_ADDR+145,7,0,1,0,1,1}, // RX_TOP_FINE_TIMING_A_0_GAIN_P_HDR_RED
    {0,SX1302_REG_RX_TOP_BASE_ADDR+145,6,0,1,0,1,0}, // RX_TOP_FINE_TIMING_A_0_ROUNDING
    {0,SX1302_REG_RX_TOP_BASE_ADDR+145,4,0,2,0,1,0}, // RX_TOP_FINE_TIMING_A_0_POS_LIMIT
    {0,SX1302_REG_RX_TOP_BASE_ADDR+145,2,0,2,0,1,0}, // RX_TOP_FINE_TIMING_A_0_SUM_SIZE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+145,0,0,2,0,1,3}, // RX_TOP_FINE_TIMING_A_0_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+146,6,0,2,0,1,1}, // RX_TOP_FINE_TIMING_A_1_GAIN_P_AUTO
    {0,SX1302_REG_RX_TOP_BASE_ADDR+146,3,0,3,0,1,2}, // RX_TOP_FINE_TIMING_A_1_GAIN_P_PAYLOAD
    {0,SX1302_REG_RX_TOP_BASE_ADDR+146,0,0,3,0,1,6}, // RX_TOP_FINE_TIMING_A_1_GAIN_P_PREAMB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+147,6,0,2,0,1,3}, // RX_TOP_FINE_TIMING_A_2_GAIN_I_AUTO
    {0,SX1302_REG_RX_TOP_BASE_ADDR+147,3,0,3,0,1,1}, // RX_TOP_FINE_TIMING_A_2_GAIN_I_PAYLOAD
    {0,SX1302_REG_RX_TOP_BASE_ADDR+147,0,0,3,0,1,4}, // RX_TOP_FINE_TIMING_A_2_GAIN_I_PREAMB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+148,7,0,1,0,1,0}, // RX_TOP_FINE_TIMING_A_3_FINESYNCH_SUM
    {0,SX1302_REG_RX_TOP_BASE_ADDR+148,4,0,3,0,1,5}, // RX_TOP_FINE_TIMING_A_3_FINESYNCH_GAIN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+149,6,0,2,0,1,0}, // RX_TOP_FINE_TIMING_A_4_GAIN_I_EN_SF8
    {0,SX1302_REG_RX_TOP_BASE_ADDR+149,4,0,2,0,1,0}, // RX_TOP_FINE_TIMING_A_4_GAIN_I_EN_SF7
    {0,SX1302_REG_RX_TOP_BASE_ADDR+149,2,0,2,0,1,0}, // RX_TOP_FINE_TIMING_A_4_GAIN_I_EN_SF6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+149,0,0,2,0,1,0}, // RX_TOP_FINE_TIMING_A_4_GAIN_I_EN_SF5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+150,6,0,2,0,1,0}, // RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+150,4,0,2,0,1,0}, // RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF11
    {0,SX1302_REG_RX_TOP_BASE_ADDR+150,2,0,2,0,1,0}, // RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF10
    {0,SX1302_REG_RX_TOP_BASE_ADDR+150,0,0,2,0,1,0}, // RX_TOP_FINE_TIMING_A_5_GAIN_I_EN_SF9
    {0,SX1302_REG_RX_TOP_BASE_ADDR+151,4,0,3,0,1,7}, // RX_TOP_FINE_TIMING_A_6_GAIN_P_PREAMB_SF12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+151,0,0,3,0,1,4}, // RX_TOP_FINE_TIMING_A_6_GAIN_P_PREAMB_SF5_6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+152,4,0,3,0,1,0}, // RX_TOP_FINE_TIMING_7_GAIN_I_AUTO_MAX
    {0,SX1302_REG_RX_TOP_BASE_ADDR+152,0,0,3,0,1,0}, // RX_TOP_FINE_TIMING_7_GAIN_P_AUTO_MAX
    {0,SX1302_REG_RX_TOP_BASE_ADDR+153,7,0,1,0,1,1}, // RX_TOP_FINE_TIMING_B_0_GAIN_P_HDR_RED
    {0,SX1302_REG_RX_TOP_BASE_ADDR+153,6,0,1,0,1,0}, // RX_TOP_FINE_TIMING_B_0_ROUNDING
    {0,SX1302_REG_RX_TOP_BASE_ADDR+153,4,0,2,0,1,0}, // RX_TOP_FINE_TIMING_B_0_POS_LIMIT
    {0,SX1302_REG_RX_TOP_BASE_ADDR+153,2,0,2,0,1,0}, // RX_TOP_FINE_TIMING_B_0_SUM_SIZE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+153,0,0,2,0,1,3}, // RX_TOP_FINE_TIMING_B_0_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+154,6,0,2,0,1,1}, // RX_TOP_FINE_TIMING_B_1_GAIN_P_AUTO
    {0,SX1302_REG_RX_TOP_BASE_ADDR+154,3,0,3,0,1,2}, // RX_TOP_FINE_TIMING_B_1_GAIN_P_PAYLOAD
    {0,SX1302_REG_RX_TOP_BASE_ADDR+154,0,0,3,0,1,6}, // RX_TOP_FINE_TIMING_B_1_GAIN_P_PREAMB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+155,6,0,2,0,1,3}, // RX_TOP_FINE_TIMING_B_2_GAIN_I_AUTO
    {0,SX1302_REG_RX_TOP_BASE_ADDR+155,3,0,3,0,1,1}, // RX_TOP_FINE_TIMING_B_2_GAIN_I_PAYLOAD
    {0,SX1302_REG_RX_TOP_BASE_ADDR+155,0,0,3,0,1,4}, // RX_TOP_FINE_TIMING_B_2_GAIN_I_PREAMB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+156,7,0,1,0,1,0}, // RX_TOP_FINE_TIMING_B_3_FINESYNCH_SUM
    {0,SX1302_REG_RX_TOP_BASE_ADDR+156,4,0,3,0,1,5}, // RX_TOP_FINE_TIMING_B_3_FINESYNCH_GAIN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+157,6,0,2,0,1,0}, // RX_TOP_FINE_TIMING_B_4_GAIN_I_EN_SF8
    {0,SX1302_REG_RX_TOP_BASE_ADDR+157,4,0,2,0,1,0}, // RX_TOP_FINE_TIMING_B_4_GAIN_I_EN_SF7
    {0,SX1302_REG_RX_TOP_BASE_ADDR+157,2,0,2,0,1,0}, // RX_TOP_FINE_TIMING_B_4_GAIN_I_EN_SF6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+157,0,0,2,0,1,0}, // RX_TOP_FINE_TIMING_B_4_GAIN_I_EN_SF5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+158,6,0,2,0,1,0}, // RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+158,4,0,2,0,1,0}, // RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF11
    {0,SX1302_REG_RX_TOP_BASE_ADDR+158,2,0,2,0,1,0}, // RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF10
    {0,SX1302_REG_RX_TOP_BASE_ADDR+158,0,0,2,0,1,0}, // RX_TOP_FINE_TIMING_B_5_GAIN_I_EN_SF9
    {0,SX1302_REG_RX_TOP_BASE_ADDR+159,4,0,3,0,1,7}, // RX_TOP_FINE_TIMING_B_6_GAIN_P_PREAMB_SF12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+159,0,0,3,0,1,4}, // RX_TOP_FINE_TIMING_B_6_GAIN_P_PREAMB_SF5_6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+160,0,0,4,0,1,9}, // RX_TOP_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT
    {0,SX1302_REG_RX_TOP_BASE_ADDR+161,0,0,8,0,1,112}, // RX_TOP_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT
    {0,SX1302_REG_RX_TOP_BASE_ADDR+162,0,0,3,0,1,3}, // RX_TOP_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP
    {0,SX1302_REG_RX_TOP_BASE_ADDR+163,5,0,1,0,1,0}, // RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FREQ_DELTA
    {0,SX1302_REG_RX_TOP_BASE_ADDR+163,4,0,1,0,1,0}, // RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FINE_DELTA
    {0,SX1302_REG_RX_TOP_BASE_ADDR+163,3,0,1,0,1,1}, // RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FREQ_ERROR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+163,2,0,1,0,1,0}, // RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+163,1,0,1,0,1,0}, // RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_OFFSET
    {0,SX1302_REG_RX_TOP_BASE_ADDR+163,0,0,1,0,1,1}, // RX_TOP_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_DETECT
    {0,SX1302_REG_RX_TOP_BASE_ADDR+164,0,0,8,0,1,33}, // RX_TOP_FREQ_TO_TIME4_FREQ_TO_TIME_INVERT_RNG
    {0,SX1302_REG_RX_TOP_BASE_ADDR+165,6,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF8
    {0,SX1302_REG_RX_TOP_BASE_ADDR+165,4,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF7
    {0,SX1302_REG_RX_TOP_BASE_ADDR+165,2,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+165,0,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_A_0_FREQ_TRACK_EN_SF5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+166,6,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+166,4,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF11
    {0,SX1302_REG_RX_TOP_BASE_ADDR+166,2,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF10
    {0,SX1302_REG_RX_TOP_BASE_ADDR+166,0,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_A_1_FREQ_TRACK_EN_SF9
    {0,SX1302_REG_RX_TOP_BASE_ADDR+167,6,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF8
    {0,SX1302_REG_RX_TOP_BASE_ADDR+167,4,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF7
    {0,SX1302_REG_RX_TOP_BASE_ADDR+167,2,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+167,0,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_B_0_FREQ_TRACK_EN_SF5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+168,6,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+168,4,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF11
    {0,SX1302_REG_RX_TOP_BASE_ADDR+168,2,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF10
    {0,SX1302_REG_RX_TOP_BASE_ADDR+168,0,0,2,0,1,3}, // RX_TOP_FREQ_TRACK_B_1_FREQ_TRACK_EN_SF9
    {0,SX1302_REG_RX_TOP_BASE_ADDR+169,7,0,1,0,1,0}, // RX_TOP_FREQ_TRACK2_FREQ_TRACK_FINE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+169,4,0,3,0,1,4}, // RX_TOP_FREQ_TRACK2_FREQ_TRACK_HDR_SKIP
    {0,SX1302_REG_RX_TOP_BASE_ADDR+170,4,0,3,0,1,5}, // RX_TOP_FREQ_TRACK3_FREQ_SYNCH_GAIN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+170,0,0,4,0,1,3}, // RX_TOP_FREQ_TRACK3_FREQ_TRACK_AUTO_THR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+171,5,0,2,0,1,1}, // RX_TOP_FREQ_TRACK4_SNR_MIN_WINDOW
    {0,SX1302_REG_RX_TOP_BASE_ADDR+171,4,0,1,0,1,0}, // RX_TOP_FREQ_TRACK4_GAIN_AUTO_SNR_MIN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+171,0,0,4,0,1,8}, // RX_TOP_FREQ_TRACK4_FREQ_SYNCH_THR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+172,0,0,7,0,1,32}, // RX_TOP_DETECT_MSP0_MSP_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+173,0,0,7,0,1,48}, // RX_TOP_DETECT_MSP1_MSP2_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+174,4,0,3,0,1,5}, // RX_TOP_DETECT_MSP2_MSP2_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+174,0,0,3,0,1,3}, // RX_TOP_DETECT_MSP2_MSP_PEAK_NB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+175,6,0,1,0,1,0}, // RX_TOP_DETECT_MSP3_ACC_MIN2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+175,4,0,2,0,1,2}, // RX_TOP_DETECT_MSP3_ACC_WIN_LEN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+175,2,0,1,0,1,1}, // RX_TOP_DETECT_MSP3_MSP_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+175,0,0,2,0,1,1}, // RX_TOP_DETECT_MSP3_MSP_CNT_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+176,7,0,1,0,1,0}, // RX_TOP_DETECT_ACC1_USE_GAIN_SYMB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+176,0,0,7,0,1,64}, // RX_TOP_DETECT_ACC1_ACC_PNR
    {0,SX1302_REG_RX_TOP_BASE_ADDR+177,6,0,2,0,1,2}, // RX_TOP_DETECT_ACC2_NOISE_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+177,4,0,2,0,1,2}, // RX_TOP_DETECT_ACC2_ACC_COEFF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+177,3,0,1,0,1,1}, // RX_TOP_DETECT_ACC2_ACC_2_SAME_PEAKS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+177,2,0,1,0,1,1}, // RX_TOP_DETECT_ACC2_ACC_AUTO_RESCALE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+177,1,0,1,0,1,1}, // RX_TOP_DETECT_ACC2_ACC_PEAK_POS_SEL
    {0,SX1302_REG_RX_TOP_BASE_ADDR+177,0,0,1,0,1,1}, // RX_TOP_DETECT_ACC2_ACC_PEAK_SUM_EN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+178,0,0,8,0,1,11}, // RX_TOP_DETECT_ACC3_MIN_SINGLE_PEAK
    {0,SX1302_REG_RX_TOP_BASE_ADDR+179,4,0,1,0,1,0}, // RX_TOP_TIMESTAMP_SEL_SNR_MIN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+179,3,0,1,0,1,0}, // RX_TOP_TIMESTAMP_ENABLE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+179,0,0,3,0,1,7}, // RX_TOP_TIMESTAMP_NB_SYMB
    {0,SX1302_REG_RX_TOP_BASE_ADDR+180,0,0,8,1,1,0}, // RX_TOP_MODEM_BUSY_MSB_RX_MODEM_BUSY
    {0,SX1302_REG_RX_TOP_BASE_ADDR+181,0,0,8,1,1,0}, // RX_TOP_MODEM_BUSY_LSB_RX_MODEM_BUSY
    {0,SX1302_REG_RX_TOP_BASE_ADDR+182,4,0,4,1,1,0}, // RX_TOP_MODEM_STATE_RX_MODEM_STS_SPARE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+182,0,0,4,1,1,0}, // RX_TOP_MODEM_STATE_RX_MODEM_STATE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+183,6,0,2,0,1,2}, // RX_TOP_MODEM_SYNC_DELTA_MSB_PEAK_POS_FINE_GAIN_H
    {0,SX1302_REG_RX_TOP_BASE_ADDR+183,4,0,2,0,1,3}, // RX_TOP_MODEM_SYNC_DELTA_MSB_PEAK_POS_FINE_GAIN_L
    {0,SX1302_REG_RX_TOP_BASE_ADDR+183,3,0,1,0,1,1}, // RX_TOP_MODEM_SYNC_DELTA_MSB_PEAK_POS_FINE_SIGN
    {0,SX1302_REG_RX_TOP_BASE_ADDR+183,0,0,3,0,1,0}, // RX_TOP_MODEM_SYNC_DELTA_MSB_MODEM_SYNC_DELTA
    {0,SX1302_REG_RX_TOP_BASE_ADDR+184,0,0,8,0,1,127}, // RX_TOP_MODEM_SYNC_DELTA_LSB_MODEM_SYNC_DELTA
    {0,SX1302_REG_RX_TOP_BASE_ADDR+185,6,0,2,0,1,0}, // RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF8
    {0,SX1302_REG_RX_TOP_BASE_ADDR+185,4,0,2,0,1,0}, // RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF7
    {0,SX1302_REG_RX_TOP_BASE_ADDR+185,2,0,2,0,1,0}, // RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF6
    {0,SX1302_REG_RX_TOP_BASE_ADDR+185,0,0,2,0,1,0}, // RX_TOP_MODEM_PPM_OFFSET1_PPM_OFFSET_SF5
    {0,SX1302_REG_RX_TOP_BASE_ADDR+186,6,0,2,0,1,1}, // RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF12
    {0,SX1302_REG_RX_TOP_BASE_ADDR+186,4,0,2,0,1,1}, // RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF11
    {0,SX1302_REG_RX_TOP_BASE_ADDR+186,2,0,2,0,1,0}, // RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF10
    {0,SX1302_REG_RX_TOP_BASE_ADDR+186,0,0,2,0,1,0}, // RX_TOP_MODEM_PPM_OFFSET2_PPM_OFFSET_SF9
    {0,SX1302_REG_RX_TOP_BASE_ADDR+187,0,0,8,0,1,85}, // RX_TOP_MODEM_CLOCK_GATE_OVERRIDE_3_CLK_OVERRIDE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+188,0,0,8,0,1,85}, // RX_TOP_MODEM_CLOCK_GATE_OVERRIDE_2_CLK_OVERRIDE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+189,0,0,8,0,1,85}, // RX_TOP_MODEM_CLOCK_GATE_OVERRIDE_1_CLK_OVERRIDE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+190,0,0,8,0,1,85}, // RX_TOP_MODEM_CLOCK_GATE_OVERRIDE_0_CLK_OVERRIDE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+191,0,0,1,1,1,0}, // RX_TOP_DUMMY2_DUMMY2
    {0,SX1302_REG_RX_TOP_BASE_ADDR+192,4,0,1,0,1,0}, // RX_TOP_RX_BUFFER_DEBUG_MODE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+192,3,0,1,0,1,0}, // RX_TOP_RX_BUFFER_DIRECT_RAM_IF
    {0,SX1302_REG_RX_TOP_BASE_ADDR+192,2,0,1,0,1,0}, // RX_TOP_RX_BUFFER_LEGACY_TIMESTAMP
    {0,SX1302_REG_RX_TOP_BASE_ADDR+192,1,0,1,0,1,0}, // RX_TOP_RX_BUFFER_STORE_HEADER_ERR_META
    {0,SX1302_REG_RX_TOP_BASE_ADDR+192,0,0,1,0,1,0}, // RX_TOP_RX_BUFFER_STORE_SYNC_FAIL_META
    {0,SX1302_REG_RX_TOP_BASE_ADDR+193,0,0,8,0,1,255}, // RX_TOP_RX_BUFFER_TIMESTAMP_CFG_MAX_TS_METRICS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+194,0,0,5,0,1,0}, // RX_TOP_RX_BUFFER_IRQ_CTRL_MSB_RX_BUFFER_IRQ_THRESHOLD
    {0,SX1302_REG_RX_TOP_BASE_ADDR+195,0,0,8,0,1,21}, // RX_TOP_RX_BUFFER_IRQ_CTRL_LSB_RX_BUFFER_IRQ_THRESHOLD
    {0,SX1302_REG_RX_TOP_BASE_ADDR+196,0,0,4,1,1,0}, // RX_TOP_RX_BUFFER_LAST_ADDR_READ_MSB_LAST_ADDR_READ
    {0,SX1302_REG_RX_TOP_BASE_ADDR+197,0,0,8,1,1,0}, // RX_TOP_RX_BUFFER_LAST_ADDR_READ_LSB_LAST_ADDR_READ
    {0,SX1302_REG_RX_TOP_BASE_ADDR+198,0,0,4,1,1,0}, // RX_TOP_RX_BUFFER_LAST_ADDR_WRITE_MSB_LAST_ADDR_WRITE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+199,0,0,8,1,1,0}, // RX_TOP_RX_BUFFER_LAST_ADDR_WRITE_LSB_LAST_ADDR_WRITE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+200,0,0,5,1,1,0}, // RX_TOP_RX_BUFFER_NB_BYTES_MSB_RX_BUFFER_NB_BYTES
    {0,SX1302_REG_RX_TOP_BASE_ADDR+201,0,0,8,1,1,0}, // RX_TOP_RX_BUFFER_NB_BYTES_LSB_RX_BUFFER_NB_BYTES
    {0,SX1302_REG_RX_TOP_BASE_ADDR+202,0,0,8,1,1,0}, // RX_TOP_MULTI_SF_SYNC_ERR_PKT_CNT_MULTI_SF_SYNC_ERR_PKTS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+203,0,0,8,1,1,0}, // RX_TOP_MULTI_SF_PLD_ERR_PKT_CNT_MULTI_SF_PLD_ERR_PKTS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+204,0,0,8,1,1,0}, // RX_TOP_MULTI_SF_GOOD_PKT_CNT_MULTI_SF_GOOD_PKTS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+205,0,0,8,1,1,0}, // RX_TOP_SERV_MODEM_SYNC_ERR_PKT_CNT_SERV_MODEM_SYNC_ERR_PKTS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+206,0,0,8,1,1,0}, // RX_TOP_SERV_MODEM_PLD_ERR_PKT_CNT_SERV_MODEM_PLD_ERR_PKTS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+207,0,0,8,1,1,0}, // RX_TOP_SERV_MODEM_GOOD_PKT_CNT_SERV_MODEM_GOOD_PKTS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+208,0,0,8,1,1,0}, // RX_TOP_GFSK_MODEM_SYNC_ERR_PKT_CNT_GFSK_MODEM_SYNC_ERR_PKTS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+209,0,0,8,1,1,0}, // RX_TOP_GFSK_MODEM_PLD_ERR_PKT_CNT_GFSK_MODEM_PLD_ERR_PKTS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+210,0,0,8,1,1,0}, // RX_TOP_GFSK_MODEM_GOOD_PKT_CNT_GFSK_MODEM_GOOD_PKTS
    {0,SX1302_REG_RX_TOP_BASE_ADDR+211,0,0,2,1,1,0}, // RX_TOP_BAD_MODEM_ID_WRITE_0_BAD_MODEM_ID_WRITE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+212,0,0,8,1,1,0}, // RX_TOP_BAD_MODEM_ID_WRITE_1_BAD_MODEM_ID_WRITE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+213,0,0,8,1,1,0}, // RX_TOP_BAD_MODEM_ID_WRITE_2_BAD_MODEM_ID_WRITE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+214,0,0,2,1,1,0}, // RX_TOP_BAD_MODEM_ID_READ_0_BAD_MODEM_ID_READ
    {0,SX1302_REG_RX_TOP_BASE_ADDR+215,0,0,8,1,1,0}, // RX_TOP_BAD_MODEM_ID_READ_1_BAD_MODEM_ID_READ
    {0,SX1302_REG_RX_TOP_BASE_ADDR+216,0,0,8,1,1,0}, // RX_TOP_BAD_MODEM_ID_READ_2_BAD_MODEM_ID_READ
    {0,SX1302_REG_RX_TOP_BASE_ADDR+217,0,0,2,0,1,1}, // RX_TOP_CLOCK_GATE_OVERRIDE_0_CLK_OVERRIDE
    {0,SX1302_REG_RX_TOP_BASE_ADDR+218,0,0,8,1,1,0}, // RX_TOP_SAMPLE_4_MSPS_LATCHED_125K_SAMPLE_4_MSPS_LATCHED_125K
    {0,SX1302_REG_RX_TOP_BASE_ADDR+219,0,0,1,1,1,0}, // RX_TOP_DUMMY3_DUMMY3
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+0,5,0,1,0,1,1}, // ARB_MCU_CTRL_CLK_EN
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+0,4,0,1,0,1,0}, // ARB_MCU_CTRL_RADIO_RST
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+0,3,0,1,0,1,0}, // ARB_MCU_CTRL_FORCE_HOST_FE_CTRL
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+0,2,0,1,0,1,1}, // ARB_MCU_CTRL_MCU_CLEAR
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+0,1,0,1,0,1,0}, // ARB_MCU_CTRL_HOST_PROG
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+0,0,0,1,1,1,0}, // ARB_MCU_CTRL_PARITY_ERROR
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+1,0,0,8,1,1,0}, // ARB_MCU_MCU_ARB_STATUS_MCU_ARB_STATUS
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+7,5,0,1,0,1,0}, // ARB_MCU_UART_CFG_MSBF
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+7,4,0,1,0,1,0}, // ARB_MCU_UART_CFG_PAR_EN
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+7,3,0,1,0,1,0}, // ARB_MCU_UART_CFG_PAR_MODE
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+7,2,0,1,0,1,0}, // ARB_MCU_UART_CFG_START_LEN
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+7,1,0,1,0,1,0}, // ARB_MCU_UART_CFG_STOP_LEN
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+7,0,0,1,0,1,1}, // ARB_MCU_UART_CFG_WORD_LEN
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+8,0,0,8,0,1,0}, // ARB_MCU_UART_CFG2_BIT_RATE
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+9,0,0,8,0,1,0}, // ARB_MCU_ARB_DEBUG_CFG_0_ARB_DEBUG_CFG_0
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+10,0,0,8,0,1,0}, // ARB_MCU_ARB_DEBUG_CFG_1_ARB_DEBUG_CFG_1
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+11,0,0,8,0,1,0}, // ARB_MCU_ARB_DEBUG_CFG_2_ARB_DEBUG_CFG_2
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+12,0,0,8,0,1,0}, // ARB_MCU_ARB_DEBUG_CFG_3_ARB_DEBUG_CFG_3
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+13,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_0_ARB_DEBUG_STS_0
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+14,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_1_ARB_DEBUG_STS_1
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+15,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_2_ARB_DEBUG_STS_2
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+16,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_3_ARB_DEBUG_STS_3
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+17,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_4_ARB_DEBUG_STS_4
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+18,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_5_ARB_DEBUG_STS_5
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+19,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_6_ARB_DEBUG_STS_6
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+20,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_7_ARB_DEBUG_STS_7
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+21,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_8_ARB_DEBUG_STS_8
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+22,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_9_ARB_DEBUG_STS_9
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+23,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_10_ARB_DEBUG_STS_10
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+24,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_11_ARB_DEBUG_STS_11
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+25,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_12_ARB_DEBUG_STS_12
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+26,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_13_ARB_DEBUG_STS_13
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+27,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_14_ARB_DEBUG_STS_14
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+28,0,0,8,1,1,0}, // ARB_MCU_ARB_DEBUG_STS_15_ARB_DEBUG_STS_15
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+29,4,0,4,0,1,0}, // ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_1_OFFSET
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+29,0,0,4,0,1,0}, // ARB_MCU_CHANNEL_SYNC_OFFSET_01_CHANNEL_0_OFFSET
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+30,4,0,4,0,1,0}, // ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_3_OFFSET
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+30,0,0,4,0,1,0}, // ARB_MCU_CHANNEL_SYNC_OFFSET_23_CHANNEL_2_OFFSET
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+31,4,0,4,0,1,0}, // ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_5_OFFSET
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+31,0,0,4,0,1,0}, // ARB_MCU_CHANNEL_SYNC_OFFSET_45_CHANNEL_4_OFFSET
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+32,4,0,4,0,1,0}, // ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_7_OFFSET
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+32,0,0,4,0,1,0}, // ARB_MCU_CHANNEL_SYNC_OFFSET_67_CHANNEL_6_OFFSET
    {0,SX1302_REG_ARB_MCU_BASE_ADDR+33,0,0,1,1,1,0}, // ARB_MCU_DUMMY_DUMMY3
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+0,1,0,1,0,1,0}, // RADIO_FE_GLBL_CTRL_DECIM_B_CLR
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+0,0,0,1,0,1,0}, // RADIO_FE_GLBL_CTRL_DECIM_A_CLR
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+1,5,0,1,0,1,0}, // RADIO_FE_CTRL0_RADIO_A_DC_NOTCH_EN
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+1,4,0,1,0,1,0}, // RADIO_FE_CTRL0_RADIO_A_FORCE_HOST_FILTER_GAIN
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+1,0,0,4,0,1,0}, // RADIO_FE_CTRL0_RADIO_A_HOST_FILTER_GAIN
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+2,0,0,8,0,1,0}, // RADIO_FE_RSSI_DB_DEF_RADIO_A_RSSI_DB_DEFAULT_VALUE
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+3,0,0,8,0,1,0}, // RADIO_FE_RSSI_DEC_DEF_RADIO_A_RSSI_DEC_DEFAULT_VALUE
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+4,0,0,8,1,1,0}, // RADIO_FE_RSSI_DEC_RD_RADIO_A_RSSI_DEC_OUT
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+5,0,0,8,1,1,0}, // RADIO_FE_RSSI_BB_RD_RADIO_A_RSSI_BB_OUT
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+6,0,0,4,1,1,0}, // RADIO_FE_DEC_FILTER_RD_RADIO_A_DEC_FILTER_GAIN
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+7,0,0,5,0,1,0}, // RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_A_RSSI_BB_FILTER_ALPHA
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+8,0,0,5,0,1,0}, // RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_A_RSSI_DEC_FILTER_ALPHA
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+9,0,0,6,0,1,0}, // RADIO_FE_IQ_COMP_AMP_COEFF_RADIO_A_AMP_COEFF
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+10,0,0,6,0,1,0}, // RADIO_FE_IQ_COMP_PHI_COEFF_RADIO_A_PHI_COEFF
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+11,0,0,6,0,1,0}, // RADIO_FE_RADIO_DIO_TEST_MODE_RADIO_A_DIO_TEST_MODE
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+12,0,0,6,0,1,0}, // RADIO_FE_RADIO_DIO_TEST_DIR_RADIO_A_DIO_TEST_DIR
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+13,0,0,6,1,1,0}, // RADIO_FE_RADIO_DIO_DIR_RADIO_A_DIO_DIR
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+14,5,0,1,0,1,0}, // RADIO_FE_CTRL0_RADIO_B_DC_NOTCH_EN
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+14,4,0,1,0,1,0}, // RADIO_FE_CTRL0_RADIO_B_FORCE_HOST_FILTER_GAIN
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+14,0,0,4,0,1,0}, // RADIO_FE_CTRL0_RADIO_B_HOST_FILTER_GAIN
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+15,0,0,8,0,1,0}, // RADIO_FE_RSSI_DB_DEF_RADIO_B_RSSI_DB_DEFAULT_VALUE
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+16,0,0,8,0,1,0}, // RADIO_FE_RSSI_DEC_DEF_RADIO_B_RSSI_DEC_DEFAULT_VALUE
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+17,0,0,8,1,1,0}, // RADIO_FE_RSSI_DEC_RD_RADIO_B_RSSI_DEC_OUT
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+18,0,0,8,1,1,0}, // RADIO_FE_RSSI_BB_RD_RADIO_B_RSSI_BB_OUT
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+19,0,0,4,1,1,0}, // RADIO_FE_DEC_FILTER_RD_RADIO_B_DEC_FILTER_GAIN
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+20,0,0,5,0,1,0}, // RADIO_FE_RSSI_BB_FILTER_ALPHA_RADIO_B_RSSI_BB_FILTER_ALPHA
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+21,0,0,5,0,1,0}, // RADIO_FE_RSSI_DEC_FILTER_ALPHA_RADIO_B_RSSI_DEC_FILTER_ALPHA
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+22,0,0,6,0,1,0}, // RADIO_FE_IQ_COMP_AMP_COEFF_RADIO_B_AMP_COEFF
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+23,0,0,6,0,1,0}, // RADIO_FE_IQ_COMP_PHI_COEFF_RADIO_B_PHI_COEFF
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+24,0,0,6,0,1,0}, // RADIO_FE_RADIO_DIO_TEST_MODE_RADIO_B_DIO_TEST_MODE
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+25,0,0,6,0,1,0}, // RADIO_FE_RADIO_DIO_TEST_DIR_RADIO_B_DIO_TEST_DIR
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+26,0,0,6,1,1,0}, // RADIO_FE_RADIO_DIO_DIR_RADIO_B_DIO_DIR
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+27,7,0,1,1,1,0}, // RADIO_FE_SIG_ANA_CFG_VALID
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+27,6,0,1,1,1,0}, // RADIO_FE_SIG_ANA_CFG_BUSY
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+27,4,0,2,0,1,0}, // RADIO_FE_SIG_ANA_CFG_DURATION
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+27,3,0,1,0,1,0}, // RADIO_FE_SIG_ANA_CFG_FORCE_HAL_CTRL
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+27,2,0,1,0,1,0}, // RADIO_FE_SIG_ANA_CFG_START
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+27,1,0,1,0,1,0}, // RADIO_FE_SIG_ANA_CFG_RADIO_SEL
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+27,0,0,1,0,1,0}, // RADIO_FE_SIG_ANA_CFG_EN
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+28,0,0,8,0,1,0}, // RADIO_FE_SIG_ANA_FREQ_FREQ
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+29,0,0,8,1,1,0}, // RADIO_FE_SIG_ANA_ABS_MSB_CORR_ABS_OUT
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+30,0,0,8,1,1,0}, // RADIO_FE_SIG_ANA_ABS_LSB_CORR_ABS_OUT
    {0,SX1302_REG_RADIO_FE_BASE_ADDR+31,0,0,1,1,1,0}, // RADIO_FE_DUMMY_DUMMY
    {0,SX1302_REG_OTP_BASE_ADDR+0,0,0,8,0,1,0}, // OTP_BYTE_ADDR_ADDR
    {0,SX1302_REG_OTP_BASE_ADDR+1,0,0,8,1,1,0}, // OTP_RD_DATA_RD_DATA
    {0,SX1302_REG_OTP_BASE_ADDR+2,4,0,4,1,1,0}, // OTP_STATUS_CHECKSUM_STATUS
    {0,SX1302_REG_OTP_BASE_ADDR+2,0,0,1,1,1,0}, // OTP_STATUS_FSM_READY
    {0,SX1302_REG_OTP_BASE_ADDR+3,0,0,2,0,1,0}, // OTP_CFG_ACCESS_MODE
    {0,SX1302_REG_OTP_BASE_ADDR+4,0,0,3,0,1,0}, // OTP_BIT_POS_POS
    {0,SX1302_REG_OTP_BASE_ADDR+5,4,0,4,0,1,0}, // OTP_PIN_CTRL_0_TM
    {0,SX1302_REG_OTP_BASE_ADDR+5,3,0,1,0,1,0}, // OTP_PIN_CTRL_0_STROBE
    {0,SX1302_REG_OTP_BASE_ADDR+5,2,0,1,0,1,0}, // OTP_PIN_CTRL_0_PGENB
    {0,SX1302_REG_OTP_BASE_ADDR+5,1,0,1,0,1,0}, // OTP_PIN_CTRL_0_LOAD
    {0,SX1302_REG_OTP_BASE_ADDR+5,0,0,1,0,1,0}, // OTP_PIN_CTRL_0_CSB
    {0,SX1302_REG_OTP_BASE_ADDR+6,2,0,1,0,1,0}, // OTP_PIN_CTRL_1_FSCK
    {0,SX1302_REG_OTP_BASE_ADDR+6,1,0,1,0,1,0}, // OTP_PIN_CTRL_1_FSI
    {0,SX1302_REG_OTP_BASE_ADDR+6,0,0,1,0,1,0}, // OTP_PIN_CTRL_1_FRST
    {0,SX1302_REG_OTP_BASE_ADDR+7,0,0,1,1,1,0}, // OTP_PIN_STATUS_FSO
    {0,SX1302_REG_OTP_BASE_ADDR+8,0,0,8,0,1,255}, // OTP_MODEM_EN_0_MODEM_EN
    {0,SX1302_REG_OTP_BASE_ADDR+9,0,0,8,0,1,255}, // OTP_MODEM_EN_1_MODEM_EN
    {0,SX1302_REG_OTP_BASE_ADDR+10,0,0,8,0,1,255}, // OTP_MODEM_SF_EN_SF_EN
    {0,SX1302_REG_OTP_BASE_ADDR+11,0,0,1,0,1,1}, // OTP_TIMESTAMP_EN_TIMESTAMP_EN
    {0,SX1302_REG_OTP_BASE_ADDR+12,0,0,1,1,1,0}, // OTP_DUMMY_DUMMY
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+0,0,0,5,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_MSB_IF_FREQ_0
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+1,0,0,8,0,1,128}, // RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_FREQ_LSB_IF_FREQ_0
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+2,0,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_LORA_SERVICE_RADIO_SEL_RADIO_SELECT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+3,4,0,3,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_BW_START
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+3,3,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_AUTO_BW_RED
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+3,2,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_NO_FAST_START
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+3,1,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_BYPASS
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+3,0,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG1_ENABLE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+4,3,0,3,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG2_BW_LOCKED
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+4,0,0,3,0,1,5}, // RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG2_BW
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+5,0,0,3,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG3_BW_RED
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+6,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DC_NOTCH_CFG4_IIR_DCC_TIME
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+7,0,1,8,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_0_FIR1_COEFF_0
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+8,0,1,8,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_1_FIR1_COEFF_1
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+9,0,1,8,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_2_FIR1_COEFF_2
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+10,0,1,8,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_3_FIR1_COEFF_3
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+11,0,1,8,0,1,5}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_4_FIR1_COEFF_4
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+12,0,1,8,0,1,8}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_5_FIR1_COEFF_5
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+13,0,1,8,0,1,6}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_6_FIR1_COEFF_6
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+14,0,1,8,0,1,4}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR1_7_FIR1_COEFF_7
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+15,0,1,8,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_0_FIR2_COEFF_0
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+16,0,1,8,0,1,-2}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_1_FIR2_COEFF_1
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+17,0,1,8,0,1,-4}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_2_FIR2_COEFF_2
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+18,0,1,8,0,1,-3}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_3_FIR2_COEFF_3
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+19,0,1,8,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_4_FIR2_COEFF_4
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+20,0,1,8,0,1,11}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_5_FIR2_COEFF_5
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+21,0,1,8,0,1,19}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_6_FIR2_COEFF_6
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+22,0,1,8,0,1,10}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_FIR2_7_FIR2_COEFF_7
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+23,7,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC0_RADIO_GAIN_RED_SEL
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+23,0,0,7,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC0_RADIO_GAIN_RED_DB
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+24,4,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_DC_COMP_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+24,3,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_FORCE_DEFAULT_FIR
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+24,2,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_RSSI_EARLY_LATCH
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+24,0,0,2,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC1_FREEZE_ON_SYNC
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+25,6,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_DAGC_IN_COMP
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+25,5,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_DAGC_FIR_HYST
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+25,3,0,2,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_RSSI_MAX_SAMPLE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+25,0,0,3,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_AGC2_RSSI_MIN_SAMPLE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+26,7,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_GAIN0_DAGC_FIR_FAST
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+26,6,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_GAIN0_FORCE_GAIN_FIR
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+26,4,0,2,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_GAIN0_GAIN_FIR1
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+26,0,0,3,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_RX_DFE_GAIN0_GAIN_FIR2
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+27,6,0,2,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_TARGET_LVL
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+27,5,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_GAIN_INCR_STEP
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+27,4,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_GAIN_DROP_COMP
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+27,3,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_COMB_FILTER_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+27,2,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_NO_FREEZE_START
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+27,0,0,2,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CFG_FREEZE_ON_SYNC
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+28,0,0,8,0,1,60}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CNT0_SAMPLE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+29,0,0,8,0,1,6}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CNT1_THR_M6
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+30,0,0,8,0,1,25}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CNT2_THR_M12
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+31,0,0,8,0,1,42}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CNT3_THR_M18
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+32,4,0,4,0,1,8}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CNT4_GAIN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+32,0,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DAGC_CNT4_FORCE_GAIN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+33,4,0,4,0,1,4}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_BW
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+33,0,0,4,0,1,7}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG0_MODEM_SF
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+34,6,0,2,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_PPM_OFFSET_HDR_CTRL
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+34,4,0,2,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_PPM_OFFSET
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+34,3,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_MODEM_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+34,0,0,3,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG1_CODING_RATE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+35,5,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_FINE_SYNCH_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+35,4,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_MODEM_START
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+35,2,0,2,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CADRXTX
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+35,1,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_IMPLICIT_HEADER
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+35,0,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG2_CRC_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+36,0,0,8,0,1,12}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG3_PAYLOAD_LENGTH
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+37,7,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG4_INT_STEP_ORIDE_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+37,0,0,6,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG4_INT_STEP_ORIDE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+38,6,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG5_HEADER_DIFF_MODE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+38,0,0,6,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG5_ZERO_PAD
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+39,0,0,8,0,1,8}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG6_PREAMBLE_SYMB_NB
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+40,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG7_PREAMBLE_SYMB_NB
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+41,3,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG8_AUTO_ACK_INT_DELAY
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+41,2,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG8_AUTO_ACK_RX
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+41,1,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG8_AUTO_ACK_TX
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+41,0,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TXRX_CFG8_POST_PREAMBLE_GAP_LONG
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+42,4,0,2,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_RX_CFG0_DFT_PEAK_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+42,2,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_RX_CFG0_CHIRP_INVERT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+42,1,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_RX_CFG0_SWAP_IQ
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+42,0,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_RX_CFG0_CONTINUOUS
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+43,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_RX_CFG1_DETECT_TIMEOUT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+44,5,0,2,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_RX_CFG2_AUTO_ACK_RANGE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+44,0,0,5,0,1,22}, // RX_TOP_LORA_SERVICE_FSK_RX_CFG2_AUTO_ACK_DELAY
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+45,5,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_RX_CFG3_RESTART_ON_HDR_ERR
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+45,4,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_RX_CFG3_CLK_EN_RESYNC_DIN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+45,0,0,4,0,1,11}, // RX_TOP_LORA_SERVICE_FSK_RX_CFG3_LLR_SCALE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+46,0,1,5,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH0_PEAK1_POS
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+47,0,1,5,0,1,4}, // RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH1_PEAK2_POS
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+48,5,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH2_FINETIME_ON_LAST
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+48,4,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH2_AUTO_SCALE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+48,3,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH2_DROP_ON_SYNCH
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+48,2,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH2_GAIN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+48,0,0,2,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_FRAME_SYNCH2_TIMEOUT_OPT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+49,7,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING0_GAIN_P_HDR_RED
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+49,6,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING0_ROUNDING
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+49,4,0,2,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING0_POS_LIMIT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+49,2,0,2,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING0_SUM_SIZE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+49,0,0,2,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING0_MODE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+50,6,0,2,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_AUTO
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+50,3,0,3,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PAYLOAD
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+50,0,0,3,0,1,4}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING1_GAIN_P_PREAMB
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+51,6,0,2,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+51,3,0,3,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_PAYLOAD
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+51,0,0,3,0,1,4}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING2_GAIN_I_PREAMB
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+52,7,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING3_FINESYNCH_SUM
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+52,4,0,3,0,1,5}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING3_FINESYNCH_GAIN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+52,0,0,2,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING3_GAIN_I_AUTO
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+53,4,0,3,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING4_GAIN_I_AUTO_MAX
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+53,0,0,3,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FINE_TIMING4_GAIN_P_AUTO_MAX
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+54,0,0,4,0,1,9}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME0_FREQ_TO_TIME_DRIFT_MANT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+55,0,0,8,0,1,112}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME1_FREQ_TO_TIME_DRIFT_MANT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+56,0,0,3,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME2_FREQ_TO_TIME_DRIFT_EXP
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+57,5,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FREQ_DELTA
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+57,4,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FINE_DELTA
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+57,3,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_FREQ_ERROR
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+57,2,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_SYMB
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+57,1,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_TIME_OFFSET
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+57,0,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME3_FREQ_TO_TIME_INVERT_DETECT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+58,0,0,8,0,1,33}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TO_TIME4_FREQ_TO_TIME_INVERT_RNG
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+59,7,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK0_FREQ_TRACK_FINE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+59,4,0,3,0,1,4}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK0_FREQ_TRACK_HDR_SKIP
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+59,0,0,2,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK0_FREQ_TRACK_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+60,4,0,3,0,1,5}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK1_FREQ_SYNCH_GAIN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+60,0,0,4,0,1,3}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK1_FREQ_TRACK_AUTO_THR
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+61,5,0,2,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK2_SNR_MIN_WINDOW
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+61,4,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK2_GAIN_AUTO_SNR_MIN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+61,0,0,4,0,1,8}, // RX_TOP_LORA_SERVICE_FSK_FREQ_TRACK2_FREQ_SYNCH_THR
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+62,0,0,7,0,1,24}, // RX_TOP_LORA_SERVICE_FSK_DETECT_MSP0_MSP_PNR
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+63,0,0,7,0,1,48}, // RX_TOP_LORA_SERVICE_FSK_DETECT_MSP1_MSP2_PNR
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+64,4,0,3,0,1,7}, // RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP2_PEAK_NB
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+64,0,0,3,0,1,7}, // RX_TOP_LORA_SERVICE_FSK_DETECT_MSP2_MSP_PEAK_NB
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+65,7,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_ACC_MIN2
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+65,4,0,2,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_ACC_WIN_LEN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+65,2,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_POS_SEL
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+65,0,0,2,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_DETECT_MSP3_MSP_CNT_MODE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+66,7,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_USE_GAIN_SYMB
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+66,0,0,7,0,1,55}, // RX_TOP_LORA_SERVICE_FSK_DETECT_ACC1_ACC_PNR
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+67,6,0,2,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_NOISE_COEFF
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+67,4,0,2,0,1,2}, // RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_ACC_COEFF
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+67,3,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_ACC_2_SAME_PEAKS
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+67,2,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_ACC_AUTO_RESCALE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+67,1,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_ACC_PEAK_POS_SEL
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+67,0,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_DETECT_ACC2_ACC_PEAK_SUM_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+68,0,0,8,0,1,11}, // RX_TOP_LORA_SERVICE_FSK_DETECT_ACC3_MIN_SINGLE_PEAK
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+69,4,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TIMESTAMP_SEL_SNR_MIN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+69,3,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_TIMESTAMP_ENABLE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+69,0,0,3,0,1,7}, // RX_TOP_LORA_SERVICE_FSK_TIMESTAMP_NB_SYMB
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+70,6,0,2,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_CLOCK_GATE_OVERRIDE_FSK_TRANSPOSE_CLK_OVERRIDE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+70,4,0,2,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_CLOCK_GATE_OVERRIDE_FSK_MODEM_CLK_OVERRIDE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+70,2,0,2,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_CLOCK_GATE_OVERRIDE_TRANSPOSE_CLK_OVERRIDE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+70,0,0,2,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_CLOCK_GATE_OVERRIDE_MODEM_CLK_OVERRIDE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+71,0,0,1,1,1,0}, // RX_TOP_LORA_SERVICE_FSK_DUMMY0_DUMMY0
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+80,0,0,5,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_MSB_IF_FREQ_0
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+81,0,0,8,0,1,128}, // RX_TOP_LORA_SERVICE_FSK_FSK_FREQ_LSB_IF_FREQ_0
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+82,4,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_CRC_IBM
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+82,2,0,2,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_DCFREE_ENC
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+82,1,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_CRC_EN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+82,0,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_0_PKT_MODE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+83,6,0,2,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_ADRS_COMP
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+83,3,0,3,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_PSIZE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+83,0,0,3,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_1_CH_BW_EXPO
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+84,3,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_MODEM_INVERT_IQ
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+84,2,0,1,0,1,1}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_AUTO_AFC
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+84,1,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RADIO_SELECT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+84,0,0,1,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_3_RX_INVERT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+85,5,0,3,0,1,4}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_4_RSSI_LENGTH
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+85,0,0,5,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_CFG_4_ERROR_OSR_TOL
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+86,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_NODE_ADRS_NODE_ADRS
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+87,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_BROADCAST_BROADCAST
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+88,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_PKT_LENGTH_PKT_LENGTH
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+89,0,0,2,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_TIMEOUT_MSB_TIMEOUT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+90,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_TIMEOUT_LSB_TIMEOUT
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+91,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_BIT_RATE_MSB_BIT_RATE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+92,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_BIT_RATE_LSB_BIT_RATE
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+93,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE7_FSK_REF_PATTERN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+94,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE6_FSK_REF_PATTERN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+95,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE5_FSK_REF_PATTERN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+96,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE4_FSK_REF_PATTERN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+97,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE3_FSK_REF_PATTERN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+98,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE2_FSK_REF_PATTERN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+99,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE1_FSK_REF_PATTERN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+100,0,0,8,0,1,0}, // RX_TOP_LORA_SERVICE_FSK_FSK_REF_PATTERN_BYTE0_FSK_REF_PATTERN
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+101,0,0,4,0,1,4}, // RX_TOP_LORA_SERVICE_FSK_FSK_RSSI_FILTER_ALPHA_FSK_RSSI_FILTER_ALPHA
    {0,SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR+102,0,0,1,1,1,0}, // RX_TOP_LORA_SERVICE_FSK_DUMMY1_DUMMY1
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+0,4,0,1,0,1,0}, // CAPTURE_RAM_CAPTURE_CFG_ENABLE
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+0,3,0,1,0,1,0}, // CAPTURE_RAM_CAPTURE_CFG_CAPTUREWRAP
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+0,2,0,1,0,1,0}, // CAPTURE_RAM_CAPTURE_CFG_CAPTUREFORCETRIGGER
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+0,1,0,1,0,1,0}, // CAPTURE_RAM_CAPTURE_CFG_CAPTURESTART
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+0,0,0,1,0,1,0}, // CAPTURE_RAM_CAPTURE_CFG_RAMCONFIG
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+1,0,0,5,0,1,0}, // CAPTURE_RAM_CAPTURE_SOURCE_A_SOURCEMUX
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+2,0,0,5,0,1,0}, // CAPTURE_RAM_CAPTURE_SOURCE_B_SOURCEMUX
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+3,0,0,8,0,1,0}, // CAPTURE_RAM_CAPTURE_PERIOD_0_CAPTUREPERIOD
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+4,0,0,8,0,1,0}, // CAPTURE_RAM_CAPTURE_PERIOD_1_CAPTUREPERIOD
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+5,0,0,1,1,1,0}, // CAPTURE_RAM_STATUS_CAPCOMPLETE
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+6,0,0,8,1,1,0}, // CAPTURE_RAM_LAST_RAM_ADDR_0_LASTRAMADDR
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+7,0,0,4,1,1,0}, // CAPTURE_RAM_LAST_RAM_ADDR_1_LASTRAMADDR
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+8,0,0,2,0,1,2}, // CAPTURE_RAM_CLOCK_GATE_OVERRIDE_CLK_OVERRIDE
    {0,SX1302_REG_CAPTURE_RAM_BASE_ADDR+9,0,0,1,1,1,0}, // CAPTURE_RAM_DUMMY0_DUMMY0
    {0,0,0,0,0,0,0,0}
};
