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
 * @file			sx1280_netdev.h
 * @brief      Netdev adaptation for the sx1280 driver
 *
 * @author     Julia CORREA REIS
 * @author     Pierre Millot
 */

#ifndef SX1280_NETDEV_H
#define SX1280_NETDEV_H

#include "net/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const netdev_driver_t sx1280_driver;    /* netdev driver of the sx1280 */

#ifdef __cplusplus
}
#endif

#endif /* SX1280_NETDEV_H */
