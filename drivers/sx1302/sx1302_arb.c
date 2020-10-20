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
 * @brief       ARB for SX1302
 *
 * @author      Pierre Millot
 * @}
 */

#include "sx1302_arb.h"

#include <string.h>
#include <stdlib.h>

#include "sx1302_internal.h"
#include "sx1302_registers.h"
#include "xtimer.h"

#define ENABLE_DEBUG SX1302_DEBUG
#include "debug.h"

int32_t sx1302_arb_status(sx1302_t *dev) {
    return sx1302_reg_read(dev,
                           SX1302_REG_ARB_MCU_MCU_ARB_STATUS_MCU_ARB_STATUS);
}

int sx1302_arb_wait_status(sx1302_t *dev, uint8_t status) {
    while (sx1302_arb_status(dev) != status) {
    }

    return 0;
}

void sx1302_arb_set_debug_stats(sx1302_t *dev, bool enable, uint8_t sf) {
    if (enable) {
        DEBUG("ARB: Debug stats enabled for SF%u\n", sf);
        sx1302_reg_write(
            dev, SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_0_ARB_DEBUG_CFG_0, sf);
    } else {
        DEBUG_PUTS("ARB: Debug stats disabled");
    }
}

uint8_t sx1302_arb_debug_read(sx1302_t *dev, uint8_t reg_id) {
    uint16_t reg = SX1302_REG_ARB_MCU_ARB_DEBUG_STS_0_ARB_DEBUG_STS_0 + reg_id;
    return sx1302_reg_read(dev, reg);
}

int sx1302_arb_debug_write(sx1302_t *dev, uint8_t reg_id, uint8_t value) {
    uint16_t reg;

    /* Check parameters */
    if (reg_id > 3) {
        printf("ERROR: invalid ARB debug register ID\n");
        return -1;
    }

    reg = SX1302_REG_ARB_MCU_ARB_DEBUG_CFG_0_ARB_DEBUG_CFG_0 + reg_id;
    sx1302_reg_write(dev, reg, (int32_t)value);

    return 0;
}

void hexDump(char *desc, const void *addr, int len) {
    int i;
    unsigned char buff[17];
    unsigned char *pc = (unsigned char *)addr;

    // Output description if given.
    if (desc != NULL)
        printf("%s:\n", desc);

    // Process every byte in the data.
    for (i = 0; i < len; i++) {
        // Multiple of 16 means new line (with line offset).

        if ((i % 16) == 0) {
            // Just don't print ASCII for the zeroth line.
            if (i != 0)
                printf("  %s\n", buff);

            // Output the offset.
            printf("  %04x ", i);
        }

        // Now the hex code for the specific character.
        printf(" %02x", pc[i]);

        // And store a printable ASCII character for later.
        if ((pc[i] < 0x20) || (pc[i] > 0x7e)) {
            buff[i % 16] = '.';
        } else {
            buff[i % 16] = pc[i];
        }

        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0) {
        printf("   ");
        i++;
    }

    // And print the final ASCII bit.
    printf("  %s\n", buff);
}

int sx1302_arb_load_firmware(sx1302_t *dev, const uint8_t *firmware) {
    // uint8_t fw_check[SX1302_MCU_FW_SIZE];
    int32_t gpio_sel = SX1302_MCU_ARB;
    int32_t val;

    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_0_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_1_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_2_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_3_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_4_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_5_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_6_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_7_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_DIR_L_DIRECTION,
                     0xFF); /* GPIO output direction */

    /* Take control over ARB MCU */
    sx1302_reg_write(dev, SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x01);
    sx1302_reg_write(dev, SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x01);
    sx1302_reg_write(dev, SX1302_REG_COMMON_PAGE_PAGE, 0x00);

    /* Write ARB fw in ARB MEM */
    sx1302_mem_write(dev, SX1302_ARB_MEM_ADDR, &firmware[0],
                     SX1302_MCU_FW_SIZE);

    /* Read back and check */
    if (ENABLE_DEBUG) {
        uint8_t* fw_check = malloc((SX1302_MCU_FW_SIZE/2) * sizeof(uint8_t));
        sx1302_mem_read(dev, SX1302_ARB_MEM_ADDR, fw_check, SX1302_MCU_FW_SIZE/2, false);
        if (memcmp(firmware, fw_check, sizeof fw_check) != 0) {
			printf ("ERROR: Failed to load fw\n");
			return -1;
		} else {
			printf ("INFO: fw correctly loaded\n");
		}
		free(fw_check);
	}

    /* Release control over ARB MCU */
    sx1302_reg_write(dev, SX1302_REG_ARB_MCU_CTRL_HOST_PROG, 0x00);
    sx1302_reg_write(dev, SX1302_REG_ARB_MCU_CTRL_MCU_CLEAR, 0x00);

    val = sx1302_reg_read(dev, SX1302_REG_ARB_MCU_CTRL_PARITY_ERROR);
    DEBUG("ARB fw loaded (parity error:0x%02X)\n", (uint8_t)val);

    return 0;
}

int sx1302_arb_start(sx1302_t *dev, uint8_t version) {
    uint8_t val;

    /* Wait for ARB fw to be started, and VERSION available in debug registers
     */
    sx1302_arb_wait_status(dev, 0x01);

    /* Get firmware VERSION */
    val = sx1302_arb_debug_read(dev, 0);
    if (val != version) {
        printf("ERROR: wrong ARB fw version (%d)\n", val);
        return -1;
    }
    DEBUG("ARB FW VERSION: %d\n", val);

    /* Enable/disable ARB detect/modem alloc stats for the specified SF */
    sx1302_arb_set_debug_stats(dev, true, SX1302_DR_LORA_SF7);

    /* 0:Disable 1:Enable double demod for different timing set (best_timestamp
     * / best_demodulation) - Only available for SF9 -> SF12 */
    sx1302_arb_debug_write(dev, 3, 0);

    /* Set double detect packet filtering threshold [0..3] */
    sx1302_arb_debug_write(dev, 2, 3);

    /* Notify ARB that it can resume */
    sx1302_arb_debug_write(dev, 1, 1);

    /* Wait for ARB to acknoledge */
    sx1302_arb_wait_status(dev, 0x00);

    DEBUG_PUTS("ARB: started");

    return 0;
}

int sx1302_agc_load_firmware(sx1302_t *dev, const uint8_t *firmware) {
    int32_t val;
    // uint8_t fw_check[MCU_FW_SIZE];
    int32_t gpio_sel = SX1302_MCU_AGC;

    /* Configure GPIO to let AGC MCU access board LEDs */
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_0_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_1_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_2_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_3_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_4_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_5_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_6_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_SEL_7_SELECTION, gpio_sel);
    sx1302_reg_write(dev, SX1302_REG_GPIO_GPIO_DIR_L_DIRECTION,
                     0xFF); /* GPIO output direction */

    /* Take control over AGC MCU */
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x01);
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x01);
    sx1302_reg_write(dev, SX1302_REG_COMMON_PAGE_PAGE, 0x00);

    /* Write AGC fw in AGC MEM */
    sx1302_mem_write(dev, SX1302_AGC_MEM_ADDR, firmware, SX1302_MCU_FW_SIZE);


    /* Read back and check */
    if (ENABLE_DEBUG) {
        uint8_t* fw_check = malloc(SX1302_MCU_FW_SIZE * sizeof(uint8_t));
    	sx1302_mem_read(dev, SX1302_AGC_MEM_ADDR, fw_check, SX1302_MCU_FW_SIZE, false);
		if (memcmp(firmware, fw_check, sizeof fw_check) != 0) {
			printf ("ERROR: Failed to load fw\n");
			return -1;
		} else {
			printf ("INFO: fw correctly loaded\n");
		}
		free(fw_check);
	}

    /* Release control over AGC MCU */
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_CTRL_HOST_PROG, 0x00);
    sx1302_reg_write(dev, SX1302_REG_AGC_MCU_CTRL_MCU_CLEAR, 0x00);

    val = sx1302_reg_read(dev, SX1302_REG_AGC_MCU_CTRL_PARITY_ERROR);
    DEBUG("AGC fw loaded (parity error:0x%02X)\n", (uint8_t)val);

    return 0;
}

int32_t sx1302_agc_status(sx1302_t *dev) {
    return sx1302_reg_read(dev,
                           SX1302_REG_AGC_MCU_MCU_AGC_STATUS_MCU_AGC_STATUS);
}

int sx1302_agc_wait_status(sx1302_t *dev, uint8_t status) {
    while (sx1302_agc_status(dev) != status) {
    }

    return 0;
}

int sx1302_agc_mailbox_read(sx1302_t *dev, uint8_t mailbox, uint8_t *value) {
    uint16_t reg;

    /* Check parameters */
    if (mailbox > 3) {
        printf("ERROR: invalid AGC mailbox ID\n");
        return -1;
    }

    reg = SX1302_REG_AGC_MCU_MCU_MAIL_BOX_RD_DATA_BYTE0_MCU_MAIL_BOX_RD_DATA -
          mailbox;
    *value = (uint8_t)sx1302_reg_read(dev, reg);

    return 0;
}

int sx1302_agc_mailbox_write(sx1302_t *dev, uint8_t mailbox, uint8_t value) {
    uint16_t reg;

    /* Check parameters */
    if (mailbox > 3) {
        printf("ERROR: invalid AGC mailbox ID\n");
        return -1;
    }

    reg = SX1302_REG_AGC_MCU_MCU_MAIL_BOX_WR_DATA_BYTE0_MCU_MAIL_BOX_WR_DATA -
          mailbox;
    sx1302_reg_write(dev, reg, (int32_t)value);

    return 0;
}

const sx1302_agc_gain_params_t agc_params = {.ana_min       = 1,
                                             .ana_max       = 13,
                                             .ana_thresh_l  = 3,
                                             .ana_thresh_h  = 12,
                                             .dec_attn_min  = 4,
                                             .dec_attn_max  = 15,
                                             .dec_thresh_l  = 40,
                                             .dec_thresh_h1 = 80,
                                             .dec_thresh_h2 = 90,
                                             .chan_attn_min = 4,
                                             .chan_attn_max = 14,
                                             .chan_thresh_l = 52,
                                             .chan_thresh_h = 132,
                                             .deviceSel     = 0,
                                             .hpMax         = 7,
                                             .paDutyCycle   = 4};

int sx1302_agc_start(sx1302_t *dev, uint8_t version, uint8_t ana_gain,
                     uint8_t dec_gain, uint8_t fdd_mode) {
    uint8_t val;

    /* Wait for AGC fw to be started, and VERSION available in mailbox */
    sx1302_agc_wait_status(
        dev, 0x01); /* fw has started, VERSION is ready in mailbox */

    sx1302_agc_mailbox_read(dev, 0, &val);
    if (val != version) {
        printf("ERROR: wrong AGC fw version (%d)\n", val);
        return -1;
    }
    DEBUG("AGC FW VERSION: %d\n", val);

    /* Configure Radio A gains */
    sx1302_agc_mailbox_write(dev, 0, ana_gain); /* 0:auto agc*/
    sx1302_agc_mailbox_write(dev, 1, dec_gain);

    /* notify AGC that gains has been set to mailbox for Radio A */
    sx1302_agc_mailbox_write(dev, 3, SX1302_AGC_RADIO_A_INIT_DONE);

    /* Wait for AGC to acknoledge it has received gain settings for Radio A */
    sx1302_agc_wait_status(dev, 0x02);

    /* Check ana_gain setting */
    sx1302_agc_mailbox_read(dev, 0, &val);
    if (val != ana_gain) {
        printf("ERROR: Analog gain of Radio A has not been set properly\n");
        return -1;
    }

    /* Check dec_gain setting */
    sx1302_agc_mailbox_read(dev, 1, &val);
    if (val != dec_gain) {
        printf("ERROR: Decimator gain of Radio A has not been set properly\n");
        return -1;
    }

    /* Check FDD mode setting */
    sx1302_agc_mailbox_read(dev, 2, &val);
    if (val != fdd_mode) {
        printf("ERROR: FDD mode of Radio A has not been set properly\n");
        return -1;
    }

    DEBUG_PUTS("AGC: Radio A config done");

    /* Configure Radio B gains */
    sx1302_agc_mailbox_write(dev, 0, ana_gain); /* 0:auto agc*/
    sx1302_agc_mailbox_write(dev, 1, dec_gain);

    /* notify AGC that gains has been set to mailbox for Radio B */
    sx1302_agc_mailbox_write(dev, 3, SX1302_AGC_RADIO_B_INIT_DONE);

    /* Wait for AGC to acknoledge it has received gain settings for Radio B */
    sx1302_agc_wait_status(dev, 0x03);

    /* Check ana_gain setting */
    sx1302_agc_mailbox_read(dev, 0, &val);
    if (val != ana_gain) {
        printf("ERROR: Analog gain of Radio B has not been set properly\n");
        return -1;
    }

    /* Check dec_gain setting */
    sx1302_agc_mailbox_read(dev, 1, &val);
    if (val != dec_gain) {
        printf("ERROR: Decimator gain of Radio B has not been set properly\n");
        return -1;
    }

    /* Check FDD mode setting */
    sx1302_agc_mailbox_read(dev, 2, &val);
    if (val != fdd_mode) {
        printf("ERROR: FDD mode of Radio B has not been set properly\n");
        return -1;
    }

    DEBUG_PUTS("AGC: Radio B config done");

    /* Configure analog gain min/max */
    sx1302_agc_mailbox_write(dev, 0, agc_params.ana_min);
    sx1302_agc_mailbox_write(dev, 1, agc_params.ana_max);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(dev, 3, 0x03);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(dev, 0x04);

    /* Check params */
    sx1302_agc_mailbox_read(dev, 0, &val);
    if (val != agc_params.ana_min) {
        printf("ERROR: wrong ana_min (w:%u r:%u)\n", agc_params.ana_min, val);
        return -1;
    }
    sx1302_agc_mailbox_read(dev, 1, &val);
    if (val != agc_params.ana_max) {
        printf("ERROR: ana_max (w:%u r:%u)\n", agc_params.ana_max, val);
        return -1;
    }

    DEBUG_PUTS("AGC: config of analog gain min/max done");

    /* Configure analog thresholds */
    sx1302_agc_mailbox_write(dev, 0, agc_params.ana_thresh_l);
    sx1302_agc_mailbox_write(dev, 1, agc_params.ana_thresh_h);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(dev, 3, 0x04);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(dev, 0x05);

    /* Check params */
    sx1302_agc_mailbox_read(dev, 0, &val);
    if (val != agc_params.ana_thresh_l) {
        printf("ERROR: wrong ana_thresh_l (w:%u r:%u)\n",
               agc_params.ana_thresh_l, val);
        return -1;
    }
    sx1302_agc_mailbox_read(dev, 1, &val);
    if (val != agc_params.ana_thresh_h) {
        printf("ERROR: wrong ana_thresh_h (w:%u r:%u)\n",
               agc_params.ana_thresh_h, val);
        return -1;
    }

    DEBUG_PUTS("AGC: config of analog threshold done");

    /* Configure decimator attenuation min/max */
    sx1302_agc_mailbox_write(dev, 0, agc_params.dec_attn_min);
    sx1302_agc_mailbox_write(dev, 1, agc_params.dec_attn_max);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(dev, 3, 0x05);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(dev, 0x06);

    /* Check params */
    sx1302_agc_mailbox_read(dev, 0, &val);
    if (val != agc_params.dec_attn_min) {
        printf("ERROR: wrong dec_attn_min (w:%u r:%u)\n",
               agc_params.dec_attn_min, val);
        return -1;
    }
    sx1302_agc_mailbox_read(dev, 1, &val);
    if (val != agc_params.dec_attn_max) {
        printf("ERROR: wrong dec_attn_max (w:%u r:%u)\n",
               agc_params.dec_attn_max, val);
        return -1;
    }

    DEBUG_PUTS("AGC: config of decimator atten min/max done");

    /* Configure decimator attenuation thresholds */
    sx1302_agc_mailbox_write(dev, 0, agc_params.dec_thresh_l);
    sx1302_agc_mailbox_write(dev, 1, agc_params.dec_thresh_h1);
    sx1302_agc_mailbox_write(dev, 2, agc_params.dec_thresh_h2);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(dev, 3, 0x06);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(dev, 0x07);

    /* Check params */
    sx1302_agc_mailbox_read(dev, 0, &val);
    if (val != agc_params.dec_thresh_l) {
        printf("ERROR: wrong dec_thresh_l (w:%u r:%u)\n",
               agc_params.dec_thresh_l, val);
        return -1;
    }
    sx1302_agc_mailbox_read(dev, 1, &val);
    if (val != agc_params.dec_thresh_h1) {
        printf("ERROR: wrong dec_thresh_h1 (w:%u r:%u)\n",
               agc_params.dec_thresh_h1, val);
        return -1;
    }
    sx1302_agc_mailbox_read(dev, 2, &val);
    if (val != agc_params.dec_thresh_h2) {
        printf("ERROR: wrong dec_thresh_h2 (w:%u r:%u)\n",
               agc_params.dec_thresh_h2, val);
        return -1;
    }

    DEBUG_PUTS("AGC: config of decimator threshold done");

    /* Configure channel attenuation min/max */
    sx1302_agc_mailbox_write(dev, 0, agc_params.chan_attn_min);
    sx1302_agc_mailbox_write(dev, 1, agc_params.chan_attn_max);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(dev, 3, 0x07);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(dev, 0x08);

    /* Check params */
    sx1302_agc_mailbox_read(dev, 0, &val);
    if (val != agc_params.chan_attn_min) {
        printf("ERROR: wrong chan_attn_min (w:%u r:%u)\n",
               agc_params.chan_attn_min, val);
        return -1;
    }
    sx1302_agc_mailbox_read(dev, 1, &val);
    if (val != agc_params.chan_attn_max) {
        printf("ERROR: wrong chan_attn_max (w:%u r:%u)\n",
               agc_params.chan_attn_max, val);
        return -1;
    }

    DEBUG_PUTS("AGC: config of channel atten min/max done");

    /* Configure channel attenuation threshold */
    sx1302_agc_mailbox_write(dev, 0, agc_params.chan_thresh_l);
    sx1302_agc_mailbox_write(dev, 1, agc_params.chan_thresh_h);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(dev, 3, 0x08);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(dev, 0x09);

    /* Check params */
    sx1302_agc_mailbox_read(dev, 0, &val);
    if (val != agc_params.chan_thresh_l) {
        printf("ERROR: wrong chan_thresh_l (w:%u r:%u)\n",
               agc_params.chan_thresh_l, val);
        return -1;
    }
    sx1302_agc_mailbox_read(dev, 1, &val);
    if (val != agc_params.chan_thresh_h) {
        printf("ERROR: wrong chan_thresh_h (w:%u r:%u)\n",
               agc_params.chan_thresh_h, val);
        return -1;
    }

    DEBUG_PUTS("AGC: config of channel atten threshold done");

    /* Configure sx1250 SetPAConfig */
    sx1302_agc_mailbox_write(dev, 0, agc_params.deviceSel);
    sx1302_agc_mailbox_write(dev, 1, agc_params.hpMax);
    sx1302_agc_mailbox_write(dev, 2, agc_params.paDutyCycle);

    /* notify AGC that params have been set to mailbox */
    sx1302_agc_mailbox_write(dev, 3, 0x09);

    /* Wait for AGC to acknoledge it has received params */
    sx1302_agc_wait_status(dev, 0x0A);

    /* Check params */
    sx1302_agc_mailbox_read(dev, 0, &val);
    if (val != agc_params.deviceSel) {
        printf("ERROR: wrong deviceSel (w:%u r:%u)\n", agc_params.deviceSel,
               val);
        return -1;
    }
    sx1302_agc_mailbox_read(dev, 1, &val);
    if (val != agc_params.hpMax) {
        printf("ERROR: wrong hpMax (w:%u r:%u)\n", agc_params.hpMax, val);
        return -1;
    }
    sx1302_agc_mailbox_read(dev, 2, &val);
    if (val != agc_params.paDutyCycle) {
        printf("ERROR: wrong paDutyCycle (w:%u r:%u)\n", agc_params.paDutyCycle,
               val);
        return -1;
    }

    DEBUG_PUTS("AGC: config of sx1250 PA optimal settings done");

    /* notify AGC that it can resume */
    sx1302_agc_mailbox_write(dev, 3, 0x0A);

    DEBUG_PUTS("AGC: started");

    return 0;
}
