/*
 * Copyright (c) 2016 Unwired Devices <info@unwds.com>
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
 * @file	   sx1280_internal.c
 * @brief      Semtech SX1280 internal functions
 *
 * @author     Julia CORREA REIS
 * @author     Pierre Millot
 */

#include "sx1280_internal.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "sx1280.h"

#define ENABLE_DEBUG SX1280_DEBUG
#include "debug.h"

#define SX1280_SPI_SPEED (SPI_CLK_1MHZ)
#define SX1280_SPI_MODE  (SPI_MODE_0)

void wait_on_busy(const sx1280_t *dev)
{
    while (gpio_read(dev->params.dio0_pin)) {
        xtimer_usleep(100);
    }
}

void sx1280_wakeup(const sx1280_t *dev)
{
    DEBUG("[sx1280] Wakeup !\n");
    spi_acquire(dev->params.spi, dev->params.nss_pin, SX1280_SPI_MODE,
                SX1280_SPI_SPEED);
    uint8_t cmd[] = { SX1280_GET_STATUS, 0 };
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, false, cmd, NULL,
                       2);
    spi_release(dev->params.spi);
    wait_on_busy(dev);

    xtimer_usleep(SX1280_WAKEUP_TIME);
}

void sx1280_reg_write(const sx1280_t *dev, uint16_t addr, uint8_t data)
{
    sx1280_write_reg_buffer(dev, addr, &data, 1);
}

void sx1280_write_reg_buffer(const sx1280_t *dev, uint16_t addr,
                             uint8_t *buffer, uint8_t size)
{
    wait_on_busy(dev);
    spi_acquire(dev->params.spi, dev->params.nss_pin, SX1280_SPI_MODE,
                SX1280_SPI_SPEED);

    uint8_t cmd[size + 3];
    cmd[0] = SX1280_WRITE_REGISTER;
    cmd[1] = (addr & 0xFF00) >> 8;
    cmd[2] = addr & 0x00FF;
    memcpy(cmd + 3, buffer, size);
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, false, cmd, NULL,
                       size + 3);

    spi_release(dev->params.spi);
    wait_on_busy(dev);
}

uint8_t sx1280_reg_read(const sx1280_t *dev, uint16_t addr)
{
    uint8_t data;

    sx1280_read_reg_buffer(dev, addr, &data, 1);
    return data;
}

void sx1280_read_reg_buffer(const sx1280_t *dev, uint16_t addr, uint8_t *buffer,
                            uint8_t size)
{
    wait_on_busy(dev);
    spi_acquire(dev->params.spi, dev->params.nss_pin, SX1280_SPI_MODE,
                SX1280_SPI_SPEED);

    uint8_t cmd[4];
    cmd[0] = SX1280_READ_REGISTER;
    cmd[1] = (addr & 0xFF00) >> 8;
    cmd[2] = addr & 0x00FF;
    cmd[3] = 0;
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, true, cmd, NULL,
                       4);
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, false, NULL,
                       buffer, size);

    spi_release(dev->params.spi);
    wait_on_busy(dev);
}

void sx1280_write_command(const sx1280_t *dev, SX1280_Commands_t command,
                          uint8_t data)
{
    sx1280_write_command_buffer(dev, command, &data, 1);
}

void sx1280_write_command_buffer(const sx1280_t *dev, SX1280_Commands_t command,
                                 uint8_t *buffer,
                                 uint16_t size)
{
    wait_on_busy(dev);
    spi_acquire(dev->params.spi, dev->params.nss_pin, SX1280_SPI_MODE,
                SX1280_SPI_SPEED);

    spi_transfer_regs(dev->params.spi, dev->params.nss_pin, command, buffer,
                      NULL, size);

    spi_release(dev->params.spi);

    if (command != SX1280_SET_SLEEP) {
        wait_on_busy(dev);
    }
}

uint8_t sx1280_read_command(const sx1280_t *dev, SX1280_Commands_t command)
{
    uint8_t data;

    sx1280_read_command_buffer(dev, command, &data, 1);
    return data;
}

void sx1280_read_command_buffer(const sx1280_t *dev, SX1280_Commands_t command,
                                uint8_t *buffer,
                                uint16_t size)
{
    wait_on_busy(dev);
    spi_acquire(dev->params.spi, dev->params.nss_pin, SX1280_SPI_MODE,
                SX1280_SPI_SPEED);

    if (command == SX1280_GET_STATUS) {
        buffer[0] = spi_transfer_byte(dev->params.spi, dev->params.nss_pin,
                                      true, command);
        spi_transfer_byte(dev->params.spi, dev->params.nss_pin, true, 0);
        spi_transfer_byte(dev->params.spi, dev->params.nss_pin, false, 0);
    }
    else {
        uint8_t cmd[2];
        cmd[0] = command;
        cmd[1] = 0;
        spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, true, cmd,
                           NULL, 2);
        spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, false, NULL,
                           buffer, size);
    }

    spi_release(dev->params.spi);
    wait_on_busy(dev);
}

void sx1280_write_buffer(const sx1280_t *dev, uint8_t offset, uint8_t *buffer,
                         uint8_t size)
{
    wait_on_busy(dev);
    spi_acquire(dev->params.spi, dev->params.nss_pin, SX1280_SPI_MODE,
                SX1280_SPI_SPEED);

    uint8_t cmd[2];
    cmd[0] = SX1280_WRITE_BUFFER;
    cmd[1] = offset;
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, true, cmd, NULL,
                       2);
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, false, buffer,
                       NULL, size);

    spi_release(dev->params.spi);
    wait_on_busy(dev);
}

void sx1280_read_buffer(const sx1280_t *dev, uint8_t offset, uint8_t *buffer,
                        uint8_t size)
{
    wait_on_busy(dev);
    spi_acquire(dev->params.spi, dev->params.nss_pin, SX1280_SPI_MODE,
                SX1280_SPI_SPEED);

    uint8_t cmd[3];
    cmd[0] = SX1280_READ_BUFFER;
    cmd[1] = offset;
    cmd[2] = 0;
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, true, cmd, NULL,
                       3);
    spi_transfer_bytes(dev->params.spi, dev->params.nss_pin, false, NULL,
                       buffer, size);

    spi_release(dev->params.spi);
    wait_on_busy(dev);
}
