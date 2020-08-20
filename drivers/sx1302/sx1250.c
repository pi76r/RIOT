#include "sx1250.h"

#include <stdlib.h>
#include <string.h>

#include "debug.h"
#include "sx1302.h"
#include "sx1302_internal.h"

int sx1250_calibrate(sx1302_t *dev, uint8_t rf_chain, uint32_t freq_hz) {
    uint8_t buff[16];

    buff[0] = 0x00;
    sx1250_read_command(dev, rf_chain, SX1250_GET_STATUS, buff, 1);

    /* Run calibration */
    if ((freq_hz > 430E6) && (freq_hz < 440E6)) {
        buff[0] = 0x6B;
        buff[1] = 0x6F;
    } else if ((freq_hz > 470E6) && (freq_hz < 510E6)) {
        buff[0] = 0x75;
        buff[1] = 0x81;
    } else if ((freq_hz > 779E6) && (freq_hz < 787E6)) {
        buff[0] = 0xC1;
        buff[1] = 0xC5;
    } else if ((freq_hz > 863E6) && (freq_hz < 870E6)) {
        buff[0] = 0xD7;
        buff[1] = 0xDB;
    } else if ((freq_hz > 902E6) && (freq_hz < 928E6)) {
        buff[0] = 0xE1;
        buff[1] = 0xE9;
    } else {
        DEBUG(
            "ERROR: failed to calibrate sx1250 radio, frequency "
            "range not supported (%lu)\n",
            freq_hz);
        return -1;
    }
    sx1250_write_command(dev, rf_chain, SX1250_CALIBRATE_IMAGE, buff, 2);

    /* Wait for calibration to complete */
    xtimer_usleep(10000);

    buff[0] = 0x00;
    buff[1] = 0x00;
    buff[2] = 0x00;
    sx1250_read_command(dev, rf_chain, SX1250_GET_DEVICE_ERRORS, buff, 3);
    if (SX1302_TAKE_N_BITS_FROM(buff[2], 4, 1) != 0) {
        DEBUG_PUTS("ERROR: sx1250 Image Calibration Error");
        return -1;
    }

    return 0;
}

int sx1250_setup(sx1302_t *dev, uint8_t rf_chain, uint32_t freq_hz,
                 bool single_input_mode) {
    int32_t freq_reg;
    uint8_t buff[16];

    /* Set Radio in Standby for calibrations */
    buff[0] = (uint8_t)SX1250_STDBY_RC;
    sx1250_write_command(dev, rf_chain, SX1250_SET_STANDBY, buff, 1);
    xtimer_usleep(10000);

    /* Get status to check Standby mode has been properly set */
    buff[0] = 0x00;
    sx1250_read_command(dev, rf_chain, SX1250_GET_STATUS, buff, 1);
    if ((uint8_t)(SX1302_TAKE_N_BITS_FROM(buff[0], 4, 3)) != 0x02) {
        printf("ERROR: Failed to set SX1250_%u in STANDBY_RC mode\n", rf_chain);
        return -1;
    }

    /* Run all calibrations (TCXO) */
    buff[0] = 0x7F;
    sx1250_write_command(dev, rf_chain, SX1250_CALIBRATE, buff, 1);
    xtimer_usleep(10000);

    /* Set Radio in Standby with XOSC ON */
    buff[0] = (uint8_t)SX1250_STDBY_XOSC;
    sx1250_write_command(dev, rf_chain, SX1250_SET_STANDBY, buff, 1);
    xtimer_usleep(10000);

    /* Get status to check Standby mode has been properly set */
    buff[0] = 0x00;
    sx1250_read_command(dev, rf_chain, SX1250_GET_STATUS, buff, 1);
    if ((uint8_t)(SX1302_TAKE_N_BITS_FROM(buff[0], 4, 3)) != 0x03) {
        printf("ERROR: Failed to set SX1250_%u in STANDBY_XOSC mode\n",
               rf_chain);
        return -1;
    }

    /* Set Bitrate to maximum (to lower TX to FS switch time) */
    buff[0] = 0x06;
    buff[1] = 0xA1;
    buff[2] = 0x01;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff, 3);
    buff[0] = 0x06;
    buff[1] = 0xA2;
    buff[2] = 0x00;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff, 3);
    buff[0] = 0x06;
    buff[1] = 0xA3;
    buff[2] = 0x00;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff, 3);

    /* Configure DIO for Rx */
    buff[0] = 0x05;
    buff[1] = 0x82;
    buff[2] = 0x00;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff,
                         3); /* Drive strength to min */
    buff[0] = 0x05;
    buff[1] = 0x83;
    buff[2] = 0x00;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff,
                         3); /* Input enable, all disabled */
    buff[0] = 0x05;
    buff[1] = 0x84;
    buff[2] = 0x00;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff,
                         3); /* No pull up */
    buff[0] = 0x05;
    buff[1] = 0x85;
    buff[2] = 0x00;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff,
                         3); /* No pull down */
    buff[0] = 0x05;
    buff[1] = 0x80;
    buff[2] = 0x00;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff,
                         3); /* Output enable, all enabled */

    /* Set fix gain (??) */
    buff[0] = 0x08;
    buff[1] = 0xB6;
    buff[2] = 0x2A;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff, 3);

    /* Set frequency */
    freq_reg = SX1302_FREQ_TO_REG(freq_hz);
    buff[0]  = (uint8_t)(freq_reg >> 24);
    buff[1]  = (uint8_t)(freq_reg >> 16);
    buff[2]  = (uint8_t)(freq_reg >> 8);
    buff[3]  = (uint8_t)(freq_reg >> 0);
    sx1250_write_command(dev, rf_chain, SX1250_SET_RF_FREQUENCY, buff, 4);

    /* Set frequency offset to 0 */
    buff[0] = 0x08;
    buff[1] = 0x8F;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff, 5);

    /* Set Radio in Rx mode, necessary to give a clock to SX1302 */
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = 0xFF;
    sx1250_write_command(dev, rf_chain, SX1250_SET_RX, buff,
                         3); /* Rx Continuous */

    /* Select single input or differential input mode */
    if (single_input_mode == true) {
        DEBUG("INFO: Configuring SX1250_%u in single input mode\n", rf_chain);
        buff[0] = 0x08;
        buff[1] = 0xE2;
        buff[2] = 0x0D;
        sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff, 3);
    }

    buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x0B;
    sx1250_write_command(dev, rf_chain, SX1250_WRITE_REGISTER, buff,
                         3); /* FPGA_MODE_RX */

    return 0;
}