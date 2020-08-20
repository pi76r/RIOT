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
 * @file			sx1280_params.h
 * @brief      Default configuration for SX1280 driver
 *
 * @author     Julia CORREA REIS
 * @author     Pierre Millot
 */

#ifndef SX1280_PARAMS_H
#define SX1280_PARAMS_H

#include "board.h"
#include "sx1280.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SX1280_PARAM_SPI
#define SX1280_PARAM_SPI (SPI_DEV(0))
#endif

#ifndef SX1280_PARAM_SPI_NSS
#define SX1280_PARAM_SPI_NSS ARDUINO_PIN_10
#endif

#ifndef SX1280_PARAM_RESET
#define SX1280_PARAM_RESET ARDUINO_PIN_7
#endif

#ifndef SX1280_PARAM_DIO0
#define SX1280_PARAM_DIO0 ARDUINO_PIN_8
#endif

#ifndef SX1280_PARAM_DIO1
#define SX1280_PARAM_DIO1 ARDUINO_PIN_9
#endif

#ifndef SX1280_PARAM_DIO2
#define SX1280_PARAM_DIO2 GPIO_UNDEF
#endif

#ifndef SX1280_PARAM_DIO3
#define SX1280_PARAM_DIO3 GPIO_UNDEF
#endif

#ifndef SX1280_PARAM_PASELECT
#define SX1280_PARAM_PASELECT (0)
#endif

#ifndef SX1280_PARAM_TX_SWITCH
#define SX1280_PARAM_TX_SWITCH GPIO_UNDEF
#endif

#ifndef SX1280_PARAM_RX_SWITCH
#define SX1280_PARAM_RX_SWITCH GPIO_UNDEF
#endif

#ifndef SX1280_PARAMS
#define SX1280_PARAMS             { .spi = SX1280_PARAM_SPI,          \
                                    .nss_pin = SX1280_PARAM_SPI_NSS,      \
                                    .reset_pin = SX1280_PARAM_RESET,        \
                                    .dio0_pin = SX1280_PARAM_DIO0,         \
                                    .dio1_pin = SX1280_PARAM_DIO1,         \
                                    .dio2_pin = SX1280_PARAM_DIO2,         \
                                    .dio3_pin = SX1280_PARAM_DIO3,         \
                                    .paselect = SX1280_PARAM_PASELECT }
#endif

static const sx1280_params_t sx1280_params[] = { SX1280_PARAMS };

#ifdef __cplusplus
}
#endif

#endif /* SX1280_PARAMS_H */
