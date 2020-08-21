/*
 * Copyright (C) 2020 Université Grenoble Alpes
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_stts751
 * @{
 * @brief       Device driver for the STTS751 I2C Temperature Sensor
 * @author      Pierre Millot
 * @file
 * @}
 */

#include "stts751.h"
#include "stts751_regs.h"

/**
 * @brief Write data to a register.
 *
 * @param[in] dev       Device descriptor of the STTS751
 * @param[in] reg       Register address to write to
 * @param[in] data      Data to write
 *
 * @return  0 on success
 * @return  -EIO When slave device doesn't ACK the byte
 * @return  -ENXIO When no devices respond on the address sent on the bus
 * @return  -ETIMEDOUT When timeout occurs before device's response
 * @return  -EINVAL When an invalid argument is given
 * @return  -EOPNOTSUPP When MCU driver doesn't support the flag operation
 * @return  -EAGAIN When a lost bus arbitration occurs
 */
int _write_reg(const stts751_t *dev, uint8_t reg, uint8_t data)
{
    i2c_t i2c_dev = dev->params.i2c_dev;

    if (i2c_acquire(i2c_dev) != 0) {
        return -1;
    }
    int rc = i2c_write_reg(i2c_dev, dev->params.i2c_addr, reg, data, 0);
    i2c_release(i2c_dev);

    return rc;
}

/**
 * @brief Read data from a register.
 *
 * @param[in] dev       Device descriptor of the STTS751
 * @param[in] reg       Register address to read from
 *
 * @return  0 on success
 * @return  -EIO When slave device doesn't ACK the byte
 * @return  -ENXIO When no devices respond on the address sent on the bus
 * @return  -ETIMEDOUT When timeout occurs before device's response
 * @return  -EINVAL When an invalid argument is given
 * @return  -EOPNOTSUPP When MCU driver doesn't support the flag operation
 * @return  -EAGAIN When a lost bus arbitration occurs
 */
static int _read_reg(const stts751_t *dev, uint8_t reg, uint8_t *data)
{
    i2c_t i2c_dev = dev->params.i2c_dev;

    if (i2c_acquire(i2c_dev) != 0) {
        return -1;
    }
    int rc = i2c_read_reg(i2c_dev, dev->params.i2c_addr, reg, data, 0);
    i2c_release(i2c_dev);

    return rc;
}

int stts751_init(stts751_t *dev, const stts751_params_t *params)
{
    assert(dev);
    assert(params);

    dev->params = *params;
    dev->conversion_rate = STTS751_CONV_RATE_1;
    dev->conversion_resolution = STTS751_CONV_RES_10;

    /* check id */
    uint8_t id;
    int rc = stts751_get_manufacturer_id(dev, &id);
    if(rc != 0 || id != STTS751_DEFAULT_MANUFACTURER_ID) 
        return -1;

    return STTS751_OK;
}

int stts751_get_manufacturer_id(const stts751_t *dev, uint8_t *id) {
    return _read_reg(dev, STTS751_REG_PROD_ID, id);
}

STTS751_conversion_rate_e stts751_get_conversion_rate(const stts751_t *dev) {
    return dev->conversion_rate;
}

int stts751_set_conversion_rate(const stts751_t *dev, STTS751_conversion_rate_e rate) {
    if(stts751_get_conversion_rate(dev) == rate) {
        return STTS751_OK;
    }

    dev->conversion_rate = rate;
    return _write_reg(dev, STTS751_REG_RATE, rate);
}

STTS751_conversion_resolution_e stts751_get_conversion_resolution(const stts751_t *dev) {
    return dev->conversion_resolution;
}

int stts751_set_conversion_resolution(const stts751_t *dev, STTS751_conversion_resolution_e res) {
    if(stts751_get_conversion_resolution(dev) == res) {
        return STTS751_OK;
    }

    dev->conversion_resolution = res;
    uint8_t old_conf;
    int rc = _read_reg(dev, STTS751_REG_CONF, old_conf);
    if(rc != 0)
        return rc;

    old_conf &= 0xF3; /* set res to 0 */
    old_conf |= res << 2;
    return _write_reg(dev, STTS751_REG_CONF, old_conf);
}

int stts751_get_temperature(const stts751_t *dev, uint16_t *temperature)
{
    uint8_t high;
    _read_reg(dev, STTS751_REG_TEMP_H, &high);
    uint8_t low = _read_reg(dev, STTS751_REG_TEMP_L);



    return ((high << 8) | low) / 256.0;
}
