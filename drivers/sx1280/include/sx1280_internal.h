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
 * @ingroup    drivers_sx1280
 *
 * @file			sx1280_internal.h
 * @brief      Semtech SX1280 internal functions
 *
 * @author     Julia CORREA REIS
 * @author     Pierre Millot
 */

#ifndef SX1280_INTERNAL_H
#define SX1280_INTERNAL_H

#include <inttypes.h>

#include "sx1280.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SX1280_POR_ACTIVE_LOGIC_LEVEL (0)   /* the reset logic level */

/**
 * @brief Writes single radio register at addr
 *
 * @param[in] dev                      The sx1280 device structure pointer
 * @param[in] addr                     Radio register address
 * @param[in] value                    The value to write
 */
void sx1280_reg_write(const sx1280_t *dev, uint16_t addr, uint8_t value);

/**
 * @brief   Writes multiple radio registers starting at address (burst-mode).
 *
 * @param[in] dev                      The sx1280 device structure pointer
 * @param[in] addr                     First radio register address
 * @param[in] buffer                   Buffer containing the new register's values
 * @param[in] size                     Number of registers to be written
 */
void sx1280_write_reg_buffer(const sx1280_t *dev, uint16_t addr,
                             uint8_t *buffer, uint8_t size);

/**
 * @brief  Read a single byte from register
 *
 * @param[in] dev                      The sx1280 device structure pointer
 * @param[in] addr                     Radio register address
 *
 * @return    the value at addr
 */
uint8_t sx1280_reg_read(const sx1280_t *dev, uint16_t addr);

/**
 * @brief   Reads multiple radio registers starting at address.
 *
 * @param[in]  dev                     The sx1280 device structure pointer
 * @param[in]  addr                    First radio register address
 * @param[in]  size                    Number of registers to be read
 * @param[out] buffer                  Buffer where to copy registers data
 */
void sx1280_read_reg_buffer(const sx1280_t *dev, uint16_t addr, uint8_t *buffer,
                            uint8_t size);

/**
 * @brief Send a command that write data to the radio
 *
 * @param[in]  dev                   The sx1280 device structure pointer
 * @param[in]  command               Opcode of the command
 * @param[in]  data                  Single byte of data to send
 */
void sx1280_write_command(const sx1280_t *dev, SX1280_Commands_t command,
                          uint8_t data);
/**
 * @brief Send a command that write data to the radio
 *
 * @param[in]	dev                   The sx1280 device structure pointer
 * @param[in]  command               Opcode of the command
 * @param[in]  buffer                    Buffer to be send to the radio
 * @param[in]  size                      Size of the buffer to send
 */
void sx1280_write_command_buffer(const sx1280_t *dev, SX1280_Commands_t command,
                                 uint8_t *buffer,
                                 uint16_t size);


/**
 * @brief Send a command that read data from the radio
 *
 * @param[in]  dev                   The sx1280 device structure pointer
 * @param[in]  command                Opcode of the command
 *
 * @return     the result of the command
 */
uint8_t sx1280_read_command(const sx1280_t *dev, SX1280_Commands_t command);

/**
 * @brief Read a command from the radio
 *
 * @param[in]	dev                   The sx1280 device structure pointer
 * @param[in]  command                   Opcode of the command
 * @param[out] buffer                    Buffer holding data from the radio
 * @param[in]  size                      Size of the buffer
 */
void sx1280_read_command_buffer(const sx1280_t *dev, SX1280_Commands_t command,
                                uint8_t *buffer,
                                uint16_t size);


/**
 * @brief Write data to the radio buffer
 *
 * @param[in]  dev                   The sx1280 device structure pointer
 * @param[in]  offset                Offset inside the buffer
 * @param[in]  buffer                Buffer to write to the radio buffer
 * @param[in]  size                  Size of the buffer
 */
void sx1280_write_buffer(const sx1280_t *dev, uint8_t offset, uint8_t *buffer,
                         uint8_t size);

/**
 * @brief Read data from the radio buffer
 *
 * @param[in]  dev                   The sx1280 device structure pointer
 * @param[in]  offset                Offset inside the buffer
 * @param[out] buffer                Buffer holding data from the radio
 * @param[in]  size                  Size of the buffer
 */
void sx1280_read_buffer(const sx1280_t *dev, uint8_t offset, uint8_t *buffer,
                        uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* SX1280_INTERNAL_H */
