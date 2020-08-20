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
 * @brief       Netdev driver definitions for SX1302 driver
 *
 * @author      Pierre Millot
 */

#ifndef SX1302_NETDEV_H
#define SX1302_NETDEV_H

#include "net/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Reference to the netdev device driver struct
 */
extern const netdev_driver_t sx1302_driver;

#ifdef __cplusplus
}
#endif

#endif /* SX127X_NETDEV_H */
/** @} */
