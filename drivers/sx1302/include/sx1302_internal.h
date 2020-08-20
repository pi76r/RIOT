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
 * @brief       Semtech SX1302 internal functions
 *
 * @author      Pierre Millot
 */

#ifndef SX1302_INTERNAL_H
#define SX1302_INTERNAL_H

#include <inttypes.h>

#include "sx1250.h"
#include "sx1302.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Writes the radio register at specified address.
 *
 * @param[in] dev                      The sx127x device structure pointer
 * @param[in] addr                     Register address
 * @param[in] data                     New register value
 */
void sx1302_reg_write(const sx1302_t *dev, uint16_t register_id,
                      int32_t reg_value);

/**
 * @brief   Reads the radio register at specified address.
 *
 * @param[in] dev                      The sx127x device structure pointer
 * @param[in] addr                     Register address
 *
 * @return	Register value
 */
int32_t sx1302_reg_read(const sx1302_t *dev, uint16_t register_id);

void sx1302_reg_read_batch(const sx1302_t *dev, uint16_t register_id,
                           uint8_t *data, uint16_t size);

int sx1302_mem_write(const sx1302_t *dev, uint16_t addr, const uint8_t *data,
                     uint16_t size);

void sx1302_mem_read(sx1302_t *dev, uint16_t addr, uint8_t *data, uint16_t size,
                     bool fifo_mode);

void sx1250_write_command(const sx1302_t *dev, uint8_t rf_chain,
                          SX1250_op_code_t op_code, uint8_t *data,
                          uint16_t size);

void sx1250_read_command(const sx1302_t *dev, uint8_t rf_chain,
                         SX1250_op_code_t op_code, uint8_t *data,
                         uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* SX127X_INTERNAL_H */
/** @} */
