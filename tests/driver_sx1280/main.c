/*
 * Copyright (C) 2016 Unwired Devices <info@unwds.com>
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
 * @file	   main.c
 * @brief      Implementation of get and set functions for SX1280
 *
 * @author     Julia CORREA REIS
 * @author     Pierre Millot
 */

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "fmt.h"
#include "net/netdev.h"
#include "net/netdev/lora.h"
#include "shell.h"
#include "shell_commands.h"
#include "sx1280_internal.h"
#include "sx1280_params.h"
#include "thread.h"
#include "xtimer.h"
#include "random.h"

#define SX1280_LORA_MSG_QUEUE (16U)
#define SX1280_STACKSIZE      (THREAD_STACKSIZE_DEFAULT)

#define MSG_TYPE_ISR (0x3456)

static char stack[SX1280_STACKSIZE];
static kernel_pid_t _recv_pid;

static char message[32];
static sx1280_t sx1280;

int lora_setup_cmd(int argc, char **argv) {
    if (argc < 4) {
        puts(
            "usage: setup "
            "<bandwidth (200, 400, 800, 1600)> "
            "<spreading factor (7..12)> "
            "<code rate (5..8)> "
            "[li]");
        return -1;
    }

    /* Check bandwidth value */
    int bw = atoi(argv[1]);
    uint8_t lora_bw;
    switch (bw) {
        case 200:
            puts("setup: setting 200KHz bandwidth");
            lora_bw = SX1280_LORA_BW_0200;
            break;

        case 400:
            puts("setup: setting 400KHz bandwidth");
            lora_bw = SX1280_LORA_BW_0400;
            break;

        case 800:
            puts("setup: setting 800KHz bandwidth");
            lora_bw = SX1280_LORA_BW_0800;
            break;

        case 1600:
            puts("setup: setting 1600KHz bandwidth");
            lora_bw = SX1280_LORA_BW_1600;
            break;

        default:
            puts(
                "[Error] setup: invalid bandwidth value given, "
                "only 200, 400, 800 or 1600 allowed.");
            return -1;
    }

    /* Check spreading factor value */
    uint8_t lora_sf = atoi(argv[2]);
    if (lora_sf < 7 || lora_sf > 12) {
        puts("[Error] setup: invalid spreading factor value given");
        return -1;
    }

    /* Check coding rate value */
    int cr  = atoi(argv[3]);
    bool li = argc > 4 && strcmp(argv[4], "li") == 0;
    if ((!li && (cr < 5 || cr > 8)) || (li && (cr < 5 || cr > 7))) {
        puts("[Error ]setup: invalid coding rate value given");
        return -1;
    }
    uint8_t lora_cr = (uint8_t)(cr - 4);
    if (li)
        lora_cr = cr;

    /* Configure radio device */
    netdev_t *netdev = (netdev_t *)&sx1280;
    netdev->driver->set(netdev, NETOPT_BANDWIDTH, &lora_bw, sizeof(lora_bw));
    netdev->driver->set(netdev, NETOPT_SPREADING_FACTOR, &lora_sf, sizeof(lora_sf));
    netdev->driver->set(netdev, NETOPT_CODING_RATE, &lora_cr, sizeof(lora_cr));

    puts("[Info] setup: configuration set with success");

    return 0;
}

int random_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;

    netdev_t *netdev = (netdev_t *)&sx1280;
    uint32_t rand;
    netdev->driver->get(netdev, NETOPT_RANDOM, &rand, sizeof(rand));
    printf("random: number from sx1280: %lu\n", rand);

    /* reinit the transceiver to default values */
    sx1280_init_radio_settings((sx1280_t *)netdev);

    return 0;
}

int register_cmd(int argc, char **argv) {
    if (argc < 2) {
        puts("usage: register <get | set>");
        return -1;
    }

    if (strstr(argv[1], "get") != NULL) {
        if (argc < 3) {
            puts("usage: register get <all | allinline | regnum>");
            return -1;
        }

        if (strcmp(argv[2], "all") == 0) {
            puts("- listing all registers -");
            uint8_t reg = 0, data = 0;
            /* Listing registers map */
            puts("Reg   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
            for (unsigned i = 0; i <= 7; i++) {
                printf("0x%02X ", i << 4);

                for (unsigned j = 0; j <= 15; j++, reg++) {
                    data = sx1280_reg_read(&sx1280, reg);
                    printf("%02X ", data);
                }
                puts("");
            }
            puts("-done-");
            return 0;
        } else if (strcmp(argv[2], "allinline") == 0) {
            puts("- listing all registers in one line -");
            /* Listing registers map */
            for (uint16_t reg = 0; reg < 256; reg++) {
                printf("%02X ", sx1280_reg_read(&sx1280, (uint8_t)reg));
            }
            puts("- done -");
            return 0;
        } else {
            long int num = 0;
            /* Register number in hex */
            if (strstr(argv[2], "0x") != NULL) {
                num = strtol(argv[2], NULL, 16);
            } else {
                num = atoi(argv[2]);
            }

            if (num >= 0 && num <= 255) {
                printf("[regs] 0x%02X = 0x%02X\n", (uint8_t)num, sx1280_reg_read(&sx1280, (uint8_t)num));
            } else {
                puts("regs: invalid register number specified");
                return -1;
            }
        }
    } else if (strstr(argv[1], "set") != NULL) {
        if (argc < 4) {
            puts("usage: register set <regnum> <value>");
            return -1;
        }

        long num, val;

        /* Register number in hex */
        if (strstr(argv[2], "0x") != NULL) {
            num = strtol(argv[2], NULL, 16);
        } else {
            num = atoi(argv[2]);
        }

        /* Register value in hex */
        if (strstr(argv[3], "0x") != NULL) {
            val = strtol(argv[3], NULL, 16);
        } else {
            val = atoi(argv[3]);
        }

        sx1280_reg_write(&sx1280, (uint8_t)num, (uint8_t)val);
    } else {
        puts("usage: register get <all | allinline | regnum>");
        return -1;
    }

    return 0;
}

int send_cmd(int argc, char **argv) {
    if (argc < 2) {
        puts("usage: send <number of packet> <payload>");
        puts("Options: if payload is empty, will send a random payload of size 16");
        return -1;
    }

    char *payload = argv[1];
    int count     = atoi(argv[1]);

    if (argc == 2) { //random mode
        payload = NULL;
    } else {
        payload = argv[2];
    }
    char random_payload[17] = { 0 };

    for (int i = 1; i <= count; i++) {
        char *to_send = payload;
        if (payload == NULL) {
            for(int j = 0; j < 16; j++)
                random_payload[j] = (random_uint32() % 26) + 'A';
            to_send = random_payload;
        }

        printf("sending \"%s\" payload (%u bytes)   (%d/%d)\n", to_send, (unsigned)strlen(to_send) + 1, i, count);

        iolist_t iolist = {.iol_base = to_send, .iol_len = (strlen(to_send) + 1)};

        netdev_t *netdev = (netdev_t *)&sx1280;
        if (netdev->driver->send(netdev, &iolist) == -ENOTSUP) {
            puts("Cannot send: radio is still transmitting");
        }

        xtimer_usleep(100000);
    }

    return 0;
}

int listen_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;

    netdev_t *netdev = (netdev_t *)&sx1280;
    /* Switch to continuous listen mode */
    const netopt_enable_t single = false;
    netdev->driver->set(netdev, NETOPT_SINGLE_RECEIVE, &single, sizeof(single));
    const uint32_t timeout = 0;
    netdev->driver->set(netdev, NETOPT_RX_TIMEOUT, &timeout, sizeof(timeout));

    /* Switch to RX state */
    netopt_state_t state = NETOPT_STATE_RX;
    netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(state));

    printf("Listen mode set\n");

    return 0;
}

int channel_cmd(int argc, char **argv) {
    if (argc < 2) {
        puts("usage: channel <get|set>");
        return -1;
    }

    netdev_t *netdev = (netdev_t *)&sx1280;
    uint32_t chan;
    if (strstr(argv[1], "get") != NULL) {
        netdev->driver->get(netdev, NETOPT_CHANNEL_FREQUENCY, &chan, sizeof(chan));
        printf("Channel: %lu\n", chan);
        return 0;
    }

    if (strstr(argv[1], "set") != NULL) {
        if (argc < 3) {
            puts("usage: channel set <channel>");
            return -1;
        }
        chan = strtoull(argv[2], NULL, 10);
        netdev->driver->set(netdev, NETOPT_CHANNEL_FREQUENCY, &chan, sizeof(chan));
        printf("New channel set to %lu Hz\n", chan);
    } else {
        puts("usage: channel <get|set>");
        return -1;
    }

    return 0;
}

int preamble_cmd(int argc, char **argv) {
    if (argc < 2) {
        puts("usage: preamble <get|set>");
        return -1;
    }

    netdev_t *netdev = (netdev_t *)&sx1280;
    uint8_t preamble;
    if (strstr(argv[1], "get") != NULL) {
        netdev->driver->get(netdev, NETOPT_PREAMBLE_LENGTH, &preamble, sizeof(preamble));
        printf("Preamble length: %d\n", preamble);
        return 0;
    }

    if (strstr(argv[1], "set") != NULL) {
        if (argc < 3) {
            puts("usage: preamble set <length>");
            return -1;
        }
        preamble = atoi(argv[2]);
        netdev->driver->set(netdev, NETOPT_PREAMBLE_LENGTH, &preamble, sizeof(preamble));
        printf("New preamble length set to %d\n", preamble);
    } else {
        puts("usage: preamble <get|set>");
        return -1;
    }

    return 0;
}

int invertiq_cmd(int argc, char **argv) {
    if (argc < 2) {
        puts("usage: invert_iq <get|set>");
        return -1;
    }

    netdev_t *netdev = (netdev_t *)&sx1280;
    SX1280_LoRaIQModes_t inv;
    if (strstr(argv[1], "get") != NULL) {
        netdev->driver->get(netdev, NETOPT_IQ_INVERT, &inv, sizeof(inv));
        printf("InvertIQ: %s\n", inv == SX1280_LORA_IQ_NORMAL ? "normal" : "inverted");
        return 0;
    }

    if (strstr(argv[1], "set") != NULL) {
        if (argc < 3) {
            puts("usage: invert_iq set <normal/inverted>");
            return -1;
        }
        if (strcmp(argv[2], "normal") == 0)
            inv = SX1280_LORA_IQ_NORMAL;
        else if (strcmp(argv[2], "inverted") == 0)
            inv = SX1280_LORA_IQ_INVERTED;
        else {
            puts("usage: invert_iq set <normal/inverted>");
            return -1;
        }
        netdev->driver->set(netdev, NETOPT_IQ_INVERT, &inv, sizeof(inv));
        printf("New InvertIQ: %s\n", inv == SX1280_LORA_IQ_NORMAL ? "normal" : "inverted");
    } else {
        puts("usage: invert_iq <get|set>");
        return -1;
    }

    return 0;
}

int rx_timeout_cmd(int argc, char **argv) {
    if (argc < 2) {
        puts("usage: rx_timeout <get|set>");
        return -1;
    }

    netdev_t *netdev = (netdev_t *)&sx1280;
    uint16_t rx_timeout;
    if (strstr(argv[1], "set") != NULL) {
        if (argc < 3) {
            puts("usage: rx_timeout set <rx_timeout>");
            return -1;
        }
        rx_timeout = atoi(argv[2]);
        netdev->driver->set(netdev, NETOPT_RX_SYMBOL_TIMEOUT, &rx_timeout, sizeof(rx_timeout));
        printf("rx_timeout set to %i\n", rx_timeout);
    } else {
        puts("usage: rx_timeout set");
        return -1;
    }

    return 0;
}

int reset_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;
    netdev_t *netdev = (netdev_t *)&sx1280;
    puts("resetting sx1280...");
    netopt_state_t state = NETOPT_STATE_RESET;
    netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    return 0;
}

int status_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;
    sx1280_pretty_status(&sx1280);
    return 0;
}

static const shell_command_t shell_commands[] = {{"setup", "Initialize LoRa modulation settings", lora_setup_cmd},
                                                 {"random", "Get random number from sx1280", random_cmd},
                                                 {"rx_timeout", "Set the RX timeout", rx_timeout_cmd},
                                                 {"channel", "Get/Set channel frequency (in Hz)", channel_cmd},
                                                 {"register", "Get/Set value(s) of registers of sx1280", register_cmd},
                                                 {"send", "Send raw payload string", send_cmd},
                                                 {"listen", "Start raw payload listener", listen_cmd},
                                                 {"reset", "Reset the sx1280 device", reset_cmd},
                                                 {"status", "Get the sx1280 status", status_cmd},
                                                 {"preamble", "Get/Set the preamble length", preamble_cmd},
                                                 {"invert_iq", "Get/Set IQ swapping", invertiq_cmd},
                                                 {NULL, NULL, NULL}};

static void _event_cb(netdev_t *dev, netdev_event_t event) {
    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;

        msg.type        = MSG_TYPE_ISR;
        msg.content.ptr = dev;

        if (msg_send(&msg, _recv_pid) <= 0) {
            puts("gnrc_netdev: possibly lost interrupt.");
        }
    } else {
        size_t len;
        netdev_lora_rx_info_t packet_info;
        sx1280_sleep_params_t sleepConfig = {0, 0, 1, 1};
        switch (event) {
            case NETDEV_EVENT_RX_STARTED:
                puts("Data reception started");
                break;

            case NETDEV_EVENT_RX_COMPLETE:
                len = dev->driver->recv(dev, NULL, 0, 0);
                dev->driver->recv(dev, message, len, &packet_info);
                printf(
                    "{Payload: \"%s\" (%d bytes), RSSI: %i, SNR: %i, TOA: "
                    "%" PRIu32 " us}\n",
                    message, (int)len, packet_info.rssi, (int)packet_info.snr, (unsigned long)sx1280_get_time_on_air((const sx1280_t *)dev, len));
                break;

            case NETDEV_EVENT_TX_COMPLETE:
                sx1280_set_sleep(&sx1280, sleepConfig);
                puts("Transmission completed");
                break;

            case NETDEV_EVENT_CAD_DONE:
                break;

            case NETDEV_EVENT_TX_TIMEOUT:
                // sx1280_set_sleep(&sx1280, sleepConfig);
                puts("TX timeout");
                break;

            case NETDEV_EVENT_CRC_ERROR:
                puts("CRC error on received packet");
                break;

            default:
                printf("Unexpected netdev event received: %d\n", event);
                break;
        }
    }
}

void *_recv_thread(void *arg) {
    (void)arg;

    static msg_t _msg_q[SX1280_LORA_MSG_QUEUE];
    msg_init_queue(_msg_q, SX1280_LORA_MSG_QUEUE);

    while (1) {
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == MSG_TYPE_ISR) {
            netdev_t *dev = msg.content.ptr;
            dev->driver->isr(dev);
        } else {
            puts("Unexpected msg type");
        }
    }
}

int main(void) {
    netdev_t *netdev = sx1280_setup(&sx1280, &sx1280_params[0]);

    if (netdev->driver->init(netdev) < 0) {
        puts("Failed to initialize SX127x device, exiting");
        return 1;
    }

    netdev->event_callback = _event_cb;

    _recv_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, _recv_thread, NULL, "recv_thread");

    if (_recv_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return 1;
    }

    /* start the shell */
    puts("Initialization successful - starting the shell now");
    sx1280_pretty_status(&sx1280);
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
