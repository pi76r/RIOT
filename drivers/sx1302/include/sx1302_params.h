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
 * @brief       Default configuration for SX1302 driver
 *
 * @author      Pierre Millot
 */

#ifndef SX1302_PARAMS_H
#define SX1302_PARAMS_H

#include "board.h"
#include "sx1302.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the SX1302 driver
 *          Pins are adapted to STM32 Nucleo-64 boards.
 * @{
 */
#ifndef SX1302_PARAM_SPI
#define SX1302_PARAM_SPI                    (SPI_DEV(0))
#endif

#ifndef SX1302_PARAM_SPI_NSS
#define SX1302_PARAM_SPI_NSS                ARDUINO_PIN_10
#endif

#ifndef SX1302_PARAM_RESET
#define SX1302_PARAM_RESET                  ARDUINO_PIN_9
#endif

#ifndef SX1302_PARAM_POWER_EN
#define SX1302_PARAM_POWER_EN               ARDUINO_PIN_8
#endif

#ifndef SX1302_PARAMS
#define SX1302_PARAMS             { .spi          = SX1302_PARAM_SPI,          \
                                    .nss_pin      = SX1302_PARAM_SPI_NSS,      \
                                    .reset_pin    = SX1302_PARAM_RESET,        \
                                    .power_en_pin = SX1302_PARAM_POWER_EN      \
                                    }
#endif
/**@}*/

/**
 * @brief   SX1302 configuration
 */
static const sx1302_params_t sx1302_params[] =
{
    SX1302_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* SX1302_PARAMS_H */
/** @} */
