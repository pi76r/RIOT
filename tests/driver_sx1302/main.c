/*
 * Copyright (C) 2016 Unwired Devices <info@unwds.com>
 *               2017 Inria Chile
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 * @file
 * @brief       Test application for SX1302 modem driver
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author      Pierre Millot
 * @}
 */

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "fmt.h"
#include "net/lora.h"
#include "net/netdev.h"
#include "net/netdev/lora.h"
#include "shell.h"
#include "shell_commands.h"
#include "sx1302_internal.h"
#include "sx1302_netdev.h"
#include "sx1302_params.h"
#include "sx1302_registers.h"
#include "thread.h"
#include "xtimer.h"
#include "random.h"

#define SX1302_LORA_MSG_QUEUE (16U)
#define SX1302_STACKSIZE      (THREAD_STACKSIZE_DEFAULT)

#define MSG_TYPE_ISR (0x3456)

static char stack[SX1302_STACKSIZE];
static kernel_pid_t _recv_pid;

static char message[32];
static sx1302_t sx1302;

int lora_setup_cmd(int argc, char **argv) {
    if (argc < 4) {
        puts(
            "usage: setup "
            "<bandwidth (125, 250, 500)> "
            "<spreading factor (7..12)> "
            "<code rate (5..8)>");
        return -1;
    }

    /* Check bandwidth value */
    int bw = atoi(argv[1]);
    uint8_t lora_bw;
    switch (bw) {
        case 125:
            puts("setup: setting 125KHz bandwidth");
            lora_bw = LORA_BW_125_KHZ;
            break;

        case 250:
            puts("setup: setting 250KHz bandwidth");
            lora_bw = LORA_BW_250_KHZ;
            break;

        case 500:
            puts("setup: setting 500KHz bandwidth");
            lora_bw = LORA_BW_500_KHZ;
            break;

        default:
            puts(
                "[Error] setup: invalid bandwidth value given, "
                "only 125, 250 or 500 allowed.");
            return -1;
    }

    /* Check spreading factor value */
    uint8_t lora_sf = atoi(argv[2]);
    if (lora_sf < 5 || lora_sf > 12) {
        puts("[Error] setup: invalid spreading factor value given");
        return -1;
    }

    /* Check coding rate value */
    int cr = atoi(argv[3]);
    if (cr < 5 || cr > 8) {
        puts("[Error ]setup: invalid coding rate value given");
        return -1;
    }
    uint8_t lora_cr = (uint8_t)(cr - 4);

    /* Configure radio device */
    netdev_t *netdev = (netdev_t *)&sx1302;
    netdev->driver->set(netdev, NETOPT_BANDWIDTH, &lora_bw, sizeof(lora_bw));
    netdev->driver->set(netdev, NETOPT_SPREADING_FACTOR, &lora_sf,
                        sizeof(lora_sf));
    netdev->driver->set(netdev, NETOPT_CODING_RATE, &lora_cr, sizeof(lora_cr));

    puts("[Info] setup: configuration set with success");

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

        netdev_t *netdev = (netdev_t *)&sx1302;
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

    netdev_t *netdev = (netdev_t *)&sx1302;
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

int syncword_cmd(int argc, char **argv) {
    if (argc < 2) {
        puts("usage: syncword <get|set>");
        return -1;
    }

    netdev_t *netdev = (netdev_t *)&sx1302;
    uint8_t syncword;
    if (strstr(argv[1], "get") != NULL) {
        netdev->driver->get(netdev, NETOPT_SYNCWORD, &syncword,
                            sizeof(syncword));
        printf("Syncword: 0x%02x\n", syncword);
        return 0;
    }

    if (strstr(argv[1], "set") != NULL) {
        if (argc < 3) {
            puts("usage: syncword set <syncword>");
            return -1;
        }
        syncword = fmt_hex_byte(argv[2]);
        netdev->driver->set(netdev, NETOPT_SYNCWORD, &syncword,
                            sizeof(syncword));
        printf("Syncword set to %02x\n", syncword);
    } else {
        puts("usage: syncword <get|set>");
        return -1;
    }

    return 0;
}
int channel_cmd(int argc, char **argv) {
    if (argc < 2) {
        puts("usage: channel <get|set>");
        return -1;
    }

    netdev_t *netdev = (netdev_t *)&sx1302;
    uint32_t chan;
    if (strstr(argv[1], "get") != NULL) {
        netdev->driver->get(netdev, NETOPT_CHANNEL_FREQUENCY, &chan,
                            sizeof(chan));
        printf("Channel: %i\n", (int)chan);
        return 0;
    }

    if (strstr(argv[1], "set") != NULL) {
        if (argc < 3) {
            puts("usage: channel set <channel>");
            return -1;
        }
        chan = atoi(argv[2]);
        netdev->driver->set(netdev, NETOPT_CHANNEL_FREQUENCY, &chan,
                            sizeof(chan));
        printf("New channel set\n");
    } else {
        puts("usage: channel <get|set>");
        return -1;
    }

    return 0;
}

int rx_timeout_cmd(int argc, char **argv) {
    if (argc < 2) {
        puts("usage: rx_timeout <get|set>");
        return -1;
    }

    netdev_t *netdev = (netdev_t *)&sx1302;
    uint16_t rx_timeout;
    if (strstr(argv[1], "set") != NULL) {
        if (argc < 3) {
            puts("usage: rx_timeout set <rx_timeout>");
            return -1;
        }
        rx_timeout = atoi(argv[2]);
        netdev->driver->set(netdev, NETOPT_RX_TIMEOUT, &rx_timeout, sizeof(rx_timeout));
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
    netdev_t *netdev = (netdev_t *)&sx1302;
    puts("resetting sx1302...");
    netopt_state_t state = NETOPT_STATE_RESET;
    netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    return 0;
}

int eui_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;
    char eui[20] = {0};
    fmt_u64_hex(eui, sx1302_get_eui(&sx1302));
    printf("SX1302 EUI: 0x%s\n", eui);
    return 0;
}

int status_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;
    static char *status[] = {"unkonwn", "off", "free", "scheduled", "emitting"};
    printf("SX1302 TX status: %s\n", status[sx1302_tx_status(&sx1302, 0)]);
    return 0;
}

bool reg_ignored[SX1302_TOTALREGS]    = {0};
uint8_t rand_values[SX1302_TOTALREGS] = {0};

int reg_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;

    static bool ran = false;

    /* all test fails if we set this one to 1 */
    reg_ignored[SX1302_REG_COMMON_CTRL0_CLK32_RIF_CTRL] = true;
    bool error_found                                    = false;

    if (ran) {
        printf("TEST#1 can only be done once after reboot\n\n");
    } else {
        /* Test 1: read all registers and check default value for
         * non-read-only registers */
        printf(
            "## TEST#1: read all registers and check default value "
            "for non-read-only registers\n");
        for (int i = 0; i < SX1302_TOTALREGS; i++) {
            if (loregs[i].rdon == 0) {
                int32_t val = sx1302_reg_read(&sx1302, i);
                if (val != loregs[i].dflt) {
                    printf(
                        "ERROR: default value for "
                        "register at index %d is %ld, "
                        "should be %ld\n",
                        i, val, loregs[i].dflt);
                    error_found = true;
                }
            }
        }
        printf("------------------\n");
        printf(" TEST#1 %s\n", (error_found == false) ? "PASSED" : "FAILED");
        printf("------------------\n\n");

        ran = true;
    }

    /* Test 2: read/write test on all non-read-only, non-pulse, non-w0clr,
     * non-w1clr registers */
    printf(
        "## TEST#2: read/write test on all non-read-only, non-pulse, "
        "non-w0clr, non-w1clr registers\n");
    /* Write all registers with a random value */
    error_found = false;
    int32_t reg_val;
    for (int i = 0; i < SX1302_TOTALREGS; i++) {
        if ((loregs[i].rdon == 0) && (reg_ignored[i] == false)) {
            /* Peek a random value different form the default reg
             * value */
            int reg_max = pow(2, loregs[i].leng) - 1;
            if (loregs[i].leng == 1) {
                reg_val = !loregs[i].dflt;
            } else {
                /* ensure random value is not the default one */
                do {
                    if (loregs[i].sign == 1) {
                        reg_val = rand() % (reg_max / 2);
                    } else {
                        reg_val = rand() % reg_max;
                    }
                } while (reg_val == loregs[i].dflt);
            }
            /* Write selected value */
            sx1302_reg_write(&sx1302, i, reg_val);
            /* store value for later check */
            rand_values[i] = reg_val;
        }
    }
    /* Read all registers and check if we got proper random value back */
    for (int i = 0; i < SX1302_TOTALREGS; i++) {
        if ((loregs[i].rdon == 0) && (loregs[i].chck == 1) &&
            (reg_ignored[i] == false)) {
            int32_t val = sx1302_reg_read(&sx1302, i);
            /* check value */
            if (val != rand_values[i]) {
                printf(
                    "ERROR: value read from register at "
                    "index %d differs from the written "
                    "value (w:%u r:%ld)\n",
                    i, rand_values[i], val);
                error_found = true;
            } else {
                // printf("INFO: MATCH reg %d (%u, %u)\n", i,
                // rand_values[i], (uint8_t)val);
            }
        }
    }
    printf("------------------\n");
    printf(" TEST#2 %s\n", (error_found == false) ? "PASSED" : "FAILED");
    printf("------------------\n\n");

    return 0;
}

int idle_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;
    netdev_t *netdev    = (netdev_t *)&sx1302;
    netopt_state_t idle = NETOPT_STATE_IDLE;
    netdev->driver->set(netdev, NETOPT_STATE, &idle, sizeof(netopt_state_t));

	puts("SX1302 set to IDLE mode");
    return 0;
}

static const shell_command_t shell_commands[] = {
    {"setup", "Initialize LoRa modulation settings", lora_setup_cmd},
    {"send", "Send raw payload string", send_cmd},
    {"listen", "Start raw payload listener", listen_cmd},
    {"reset", "Reset the sx1302 device", reset_cmd},
    {"eui", "Get the unique identifier", eui_cmd},
    {"status", "Get the TX status", status_cmd},
    {"test_reg", "Test writing and reading from registers", reg_cmd},
    {"syncword", "Get/Set the syncword", syncword_cmd},
    {"channel", "Get/Set channel frequency (in Hz)", channel_cmd},
    {"rx_timeout", "Set the RX timeout", rx_timeout_cmd},
    {"idle", "Stop the RX mode", idle_cmd},
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
        switch (event) {
            case NETDEV_EVENT_RX_STARTED:
                puts("Data reception started");
                break;

            case NETDEV_EVENT_RX_COMPLETE:
                len = dev->driver->recv(dev, NULL, 0, 0);
                dev->driver->recv(dev, message, len, &packet_info);
                printf(
                    "{Payload: \"%s\" (%d bytes), RSSI: %i, SNR: "
                    "%i}\n",
                    message, (int)len, packet_info.rssi, (int)packet_info.snr);
                break;

            case NETDEV_EVENT_TX_COMPLETE:
                puts("Transmission completed");
                break;

            case NETDEV_EVENT_CAD_DONE:
                break;

            case NETDEV_EVENT_TX_TIMEOUT:
                break;

            default:
                printf("Unexpected netdev event received: %d\n", event);
                break;
        }
    }
}

void *_recv_thread(void *arg) {
    (void)arg;

    static msg_t _msg_q[SX1302_LORA_MSG_QUEUE];
    msg_init_queue(_msg_q, SX1302_LORA_MSG_QUEUE);

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
    netdev_t *netdev = sx1302_setup(&sx1302, &sx1302_params[0]);

    if (netdev->driver->init(netdev) < 0) {
        puts("Failed to initialize SX1302 device, exiting");
        return 1;
    }

    netdev->event_callback = _event_cb;

    _recv_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, _recv_thread, NULL,
                              "recv_thread");

    if (_recv_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return 1;
    }

    /* start the shell */
    puts("Initialization successful - starting the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
