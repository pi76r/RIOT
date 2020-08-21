/*
 * Copyright (C) 2020 Université Grenoble Alpes
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_stts751 STTS751 I2C Temperature Sensor
 * @ingroup     drivers_sensors
 * @brief       Device driver for the STTS751 Temperature Sensor
 *
 * @{
 *
 * @author      Pierre Millot
 * @file
 */

#ifndef STTS751_H
#define STTS751_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "periph/i2c.h"

#include "stts751_regs.h"

#define STTS751_OK 0

#define STTS751_DEFAULT_MANUFACTURER_ID 53

/** 
 * @brief   STTS751 conversion rate table, in conversion per second
 */
typedef enum {
    STTS751_CONV_RATE_00625,  /** 0.0625 conversion/second */
    STTS751_CONV_RATE_0125,   /** 0.125  conversion/second */
    STTS751_CONV_RATE_025,    /** 0.25   conversion/second */
    STTS751_CONV_RATE_05,     /** 0.5    conversion/second */
    STTS751_CONV_RATE_1,      /** 1      conversion/second */
    STTS751_CONV_RATE_2,      /** 2      conversion/second */
    STTS751_CONV_RATE_4,      /** 4      conversion/second */
    STTS751_CONV_RATE_8,      /** 8      conversion/second */
    STTS751_CONV_RATE_16,     /** 16     conversion/second */
    STTS751_CONV_RATE_32      /** 32     conversion/second */
} STTS751_conversion_rate_e;

/** 
 * @brief STTS751 conversion resolution table, in bits
 */
typedef enum {
    STTS751_CONV_RES_10, /** 10 bits (0.25°)   */
    STTS751_CONV_RES_11, /** 11 bits (0.125°)  */
    STTS751_CONV_RES_12, /** 12 bits (0.0625°) */
    STTS751_CONV_RES_9,  /**  9 bits (0.5°)    */
} STTS751_conversion_resolution_e;

/**
 * @brief   STTS751 device initialization parameters
 */
typedef struct {
    i2c_t i2c_dev;              /**< I2C device */
    uint16_t i2c_addr;          /**< I2C address of device */
} stts751_params_t;

/**
 * @brief   STTS751 PWM device data structure type
 */
typedef struct {
    stts751_params_t params;     /**< Device initialization parameters */
    STTS751_conversion_rate_e conversion_rate;
    STTS751_conversion_resolution_e conversion_resolution;
} stts751_t;

/**
 * @brief Initialization.
 *
 * @param[in] dev       Device descriptor of the STTS751
 * @param[in] params    Parameters for device initialization
 *
 * @return  STTS751_OK on success
 * @return  -EIO When slave device doesn't ACK the byte
 * @return  -ENXIO When no devices respond on the address sent on the bus
 * @return  -ETIMEDOUT When timeout occurs before device's response
 * @return  -EINVAL When an invalid argument is given
 * @return  -EOPNOTSUPP When MCU driver doesn't support the flag operation
 * @return  -EAGAIN When a lost bus arbitration occurs
 */
int stts751_init(stts751_t *dev, const stts751_params_t *params);

int stts751_get_manufacturer_id(const stts751_t *dev, uint8_t *id);


/**
 * @brief Get the temperature
 *
 * @param[in] dev             Device descriptor of the STTS751
 * @param[out] temperature    The temperature (Celsius) if no error
 *
 * @return STTS751_OK on success
 */
int stts751_get_temperature(const stts751_t *dev, uint16_t *temperature);

#ifdef __cplusplus
}
#endif

#endif /* STTS751_H */
/** @} */
