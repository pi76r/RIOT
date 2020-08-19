
/*
Cubesat Space Protocol - A small network-layer protocol designed for Cubesats
Copyright (C) 2012 GomSpace ApS (http://www.gomspace.com)
Copyright (C) 2012 AAUSAT3 Project (http://aausat3.space.aau.dk)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include "shell.h"


#define CSP_USE_ASSERT 1  // always enable CSP assert

#include <csp/csp.h>
#include <csp/csp_debug.h>
#include <csp/arch/csp_thread.h>
#include <csp/arch/csp_clock.h>
#include <csp/arch/csp_time.h>
#include <csp/arch/csp_malloc.h>
#include <csp/arch/csp_queue.h>
#include <csp/arch/csp_semaphore.h>
#include <csp/drivers/can_socketcan.h>

#include "can/raw.h"


/* Server port, the port the server listens on for incoming connections from the client. */
#define MY_SERVER_PORT		10

#define SERVER_ADDRESS      1

/* Server task - handles requests from clients */
CSP_DEFINE_TASK(task_server) {
    (void)arg;
	csp_log_info("Server task started");

	/* Create socket with no specific socket options, e.g. accepts CRC32, HMAC, XTEA, etc. if enabled during compilation */
	csp_socket_t *sock = csp_socket(CSP_SO_NONE);

	/* Bind socket to all ports, e.g. all incoming connections will be handled here */
	csp_bind(sock, CSP_ANY);

	/* Create a backlog of 10 connections, i.e. up to 10 new connections can be queued */
	csp_listen(sock, 10);

	/* Wait for connections and then process packets on the connection */
	while (1) {

		/* Wait for a new connection, 10000 mS timeout */
		csp_conn_t *conn;
		if ((conn = csp_accept(sock, 10000)) == NULL) {
			/* timeout */
			continue;
		}

		/* Read packets on connection, timout is 100 mS */
		csp_packet_t *packet;
		while ((packet = csp_read(conn, 50)) != NULL) {
			switch (csp_conn_dport(conn)) {
			case MY_SERVER_PORT:
				/* Process packet here */
				csp_log_info("Packet received on MY_SERVER_PORT: %s", (char *) packet->data);
				csp_buffer_free(packet);
				break;

			default:
				/* Call the default CSP service handler, handle pings, buffer use, etc. */
				csp_service_handler(conn, packet);
				break;
			}
		}

		/* Close current connection */
		csp_close(conn);

	}

	return CSP_TASK_RETURN;

}
/* End of server task */

/* Client task sending requests to server task */
CSP_DEFINE_TASK(task_client) {
    (void)arg;

	csp_log_info("Client task started");

	unsigned int count = 0;

	while (1) {
		csp_sleep_ms(1000);
		/* Send ping to server, timeout 1000 mS, ping size 100 bytes */
		int result = csp_ping(SERVER_ADDRESS, 1000, 100, CSP_O_NONE);
		csp_log_info("Ping address: %u, result %d [mS]", SERVER_ADDRESS, result);

		/* Send reboot request to server, the server has no actual implementation of csp_sys_reboot() and fails to reboot */
		csp_reboot(SERVER_ADDRESS);
		csp_log_info("reboot system request sent to address: %u", SERVER_ADDRESS);

		/* Send data packet (string) to server */

		/* 1. Connect to host on 'SERVER_ADDRESS', port MY_SERVER_PORT with regular UDP-like protocol and 1000 ms timeout */
		csp_conn_t * conn = csp_connect(CSP_PRIO_NORM, SERVER_ADDRESS, MY_SERVER_PORT, 1000, CSP_O_NONE);
		if (conn == NULL) {
			/* Connect failed */
			csp_log_error("Connection failed");
			return CSP_TASK_RETURN;
		}

		/* 2. Get packet buffer for message/data */
		csp_packet_t * packet = csp_buffer_get(100);
		if (packet == NULL) {
			/* Could not get buffer element */
			csp_log_error("Failed to get CSP buffer");
			return CSP_TASK_RETURN;
		}

		/* 3. Copy data to packet */
		snprintf((char *) packet->data, csp_buffer_data_size(), "Hello World (%u)", ++count);

		/* 4. Set packet length */
		packet->length = (strlen((char *) packet->data) + 1); /* include the 0 termination */

		/* 5. Send packet */
		if (!csp_send(conn, packet, 1000)) {
			/* Send failed */
			csp_log_error("Send failed");
			csp_buffer_free(packet);
		}

		/* 6. Close connection */
		csp_close(conn);
	}

	return CSP_TASK_RETURN;
}
/* End of client task */

/* initialization of CSP and start of server/client tasks */
void setup_csp(uint8_t address, const char *device) {
    csp_debug_level_t debug_level = CSP_INFO;
   
    /* enable/disable debug levels */
    for (csp_debug_level_t i = 0; i <= CSP_LOCK; ++i) {
        csp_debug_set_level(i, (i <= debug_level) ? true : false);
    }

    csp_log_info("Initialising CSP (address=%d)", address);

    /* Init CSP with address and default settings */
    csp_conf_t csp_conf;
    csp_conf_get_defaults(&csp_conf);
    csp_conf.address = address;
    int error = csp_init(&csp_conf);
    if (error != CSP_ERR_NONE) {
        csp_log_error("csp_init() failed, error: %d", error);
        exit(1);
    }

    csp_route_start_task(THREAD_STACKSIZE_MAIN, THREAD_PRIORITY_MAIN - 1);

    /* Add interface(s) */
    csp_iface_t * default_iface = NULL;

    error = csp_can_socketcan_open_and_add_interface(device, CSP_IF_CAN_DEFAULT_NAME, 0, false, &default_iface);
    if (error != CSP_ERR_NONE) {
        csp_log_error("failed to add CAN interface [%s], error: %d", device, error);
        exit(1);
    }

    if (default_iface) {
        csp_rtable_set(CSP_DEFAULT_ROUTE, 0, default_iface, CSP_NO_VIA_ADDRESS);
    }

    printf("Connection table\r\n");
    csp_conn_print_table();

    printf("Interfaces\r\n");
    csp_iflist_print();

    printf("Route table\r\n");
    csp_rtable_print();

    if (address == SERVER_ADDRESS) {
        csp_thread_create(task_server, "SERVER", 1000, NULL, THREAD_PRIORITY_MAIN - 1, NULL);
    } else {
        csp_thread_create(task_client, "CLIENT", 1000, NULL, THREAD_PRIORITY_MAIN - 1, NULL);
    }
}

int server(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    setup_csp(SERVER_ADDRESS, "can0");

    return 0;
}

int client(int argc, char *argv[]) {
    uint8_t address = 2;
    char *device = "can0";
    if(argc > 1) {
        address = atoi(argv[1]);
    }
    if(argc > 2) {
        device = argv[2];
    }
    setup_csp(address, (const char *)device);

    return 0;
}

static bool thread_executed = false;

void csp_assert_fail_action(const char *assertion, const char *file, int line) {
    printf("assertion: [%s], file: %s:%d\r\n", assertion, file, line);
    exit(1);
}

CSP_DEFINE_TASK(thread_func) {
    (void)arg;
    csp_log_info("Thread started");
    thread_executed = true;
    return CSP_TASK_RETURN;
}

int test_arch(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    // debug/log - enable all levels
    for (int i = 0; i <= CSP_LOCK; ++i) {
        csp_debug_set_level(i, true);
    }
    csp_log_error("csp_log_error(...), level: %d", CSP_ERROR);
    csp_log_warn("csp_log_warn(...), level: %d", CSP_WARN);
    csp_log_info("csp_log_info((...), level: %d", CSP_INFO);
    csp_log_buffer("csp_log_buffer(...), level: %d", CSP_BUFFER);
    csp_log_packet("csp_log_packet(...), level: %d", CSP_PACKET);
    csp_log_protocol("csp_log_protocol(...), level: %d", CSP_PROTOCOL);
    csp_log_lock("csp_log_lock(...), level: %d", CSP_LOCK);

    // create a thread - csp_thread doesn't support join
    csp_thread_handle_t thread = 0;
    int res = csp_thread_create(thread_func, "thread", 0, NULL, 0, &thread);
    csp_assert(res == CSP_ERR_NONE);
    csp_assert(thread != 0);

    // clock
    csp_timestamp_t csp_clock = {};
    csp_clock_get_time(&csp_clock);
    csp_assert(csp_clock.tv_sec != 0);
    csp_log_info("csp_clock_get_time(..) -> sec:nsec = %"PRIu32":%"PRIu32, csp_clock.tv_sec, csp_clock.tv_nsec);

    // relative time
    const uint32_t msec1 = csp_get_ms();
    const uint32_t msec2 = csp_get_ms_isr();
    const uint32_t sec1 = csp_get_s();
    const uint32_t sec2 = csp_get_s_isr();
    csp_sleep_ms(2000);
    csp_assert(csp_get_ms() >= (msec1 + 500));
    csp_assert(csp_get_ms_isr() >= (msec2 + 500));
    csp_assert(csp_get_s() >= (sec1 + 1));
    csp_assert(csp_get_s_isr() >= (sec2 + 1));
    csp_log_info("time working correctly");

    // malloc
    uint32_t * ptr = csp_malloc(sizeof(*ptr));
    csp_assert(ptr != NULL);
    ptr[0] = 10;
    csp_free(ptr);
    ptr = csp_calloc(1, sizeof(*ptr));
    csp_assert(ptr != NULL);
    csp_assert(*ptr == 0);
    ptr[0] = 20;
    csp_free(ptr);
    csp_log_info("malloc working correctly");
    
    // check thread actually executed
    csp_assert(thread_executed != false);
    csp_log_info("thread working correctly");

    // queue handling
    uint32_t value;
    csp_queue_handle_t q = csp_queue_create(3, sizeof(value));
    // if test_arch crashes, you need to add a puts("something") right here
    // so there is now way to debug it
    csp_assert(csp_queue_size(q) == 0);
    csp_assert(csp_queue_size_isr(q) == 0);
    csp_assert(csp_queue_dequeue(q, &value, 0) == CSP_QUEUE_ERROR);
    csp_assert(csp_queue_dequeue(q, &value, 1000) == CSP_QUEUE_ERROR);
    csp_assert(csp_queue_dequeue_isr(q, &value, NULL) == CSP_QUEUE_ERROR);
    value = 1;
    csp_assert(csp_queue_enqueue(q, &value, 0) == CSP_QUEUE_OK);
    value = 2;
    csp_assert(csp_queue_enqueue(q, &value, 200) == CSP_QUEUE_OK);
    value = 3;
    csp_assert(csp_queue_enqueue_isr(q, &value, NULL) == CSP_QUEUE_OK);
    csp_assert(csp_queue_size(q) == 3);
    csp_assert(csp_queue_size_isr(q) == 3);
    value = 10;
    csp_assert(csp_queue_enqueue(q, &value, 0) == CSP_QUEUE_ERROR);
    value = 20;
    csp_assert(csp_queue_enqueue(q, &value, 200) == CSP_QUEUE_ERROR);
    value = 30;
    csp_assert(csp_queue_enqueue_isr(q, &value, NULL) == CSP_QUEUE_ERROR);
    value = 100;
    csp_assert(csp_queue_dequeue(q, &value, 0) == CSP_QUEUE_OK);
    csp_assert(value == 1);
    csp_assert(csp_queue_dequeue(q, &value, 200) == CSP_QUEUE_OK);
    csp_assert(value == 2);
    csp_assert(csp_queue_dequeue_isr(q, &value, NULL) == CSP_QUEUE_OK);
    csp_assert(value == 3);
    csp_queue_remove(q);
    csp_log_info("queue working correctly");

    // mutex - the actual mutex lock can't be tested from a single thread
    csp_mutex_t m;
    csp_assert(csp_mutex_create(&m) == CSP_MUTEX_OK);
    csp_assert(csp_mutex_lock(&m, 0) == CSP_MUTEX_OK);
    csp_assert(csp_mutex_lock(&m, 200) == CSP_MUTEX_ERROR);
    csp_assert(csp_mutex_unlock(&m) == CSP_MUTEX_OK);
    csp_assert(csp_mutex_lock(&m, 200) == CSP_MUTEX_OK);
    csp_assert(csp_mutex_unlock(&m) == CSP_MUTEX_OK);
    csp_assert(csp_mutex_remove(&m) == CSP_MUTEX_OK);
    csp_log_info("mutex working correctly");

    // semaphore
    csp_bin_sem_handle_t s;
    csp_assert(csp_bin_sem_create(&s) == CSP_SEMAPHORE_OK);
    csp_assert(csp_bin_sem_wait(&s, 0) == CSP_SEMAPHORE_OK);
    csp_assert(csp_bin_sem_post(&s) == CSP_SEMAPHORE_OK);
    csp_assert(csp_bin_sem_post_isr(&s, NULL) == CSP_SEMAPHORE_OK);
    csp_assert(csp_bin_sem_wait(&s, 200) == CSP_SEMAPHORE_OK);
    csp_assert(csp_bin_sem_wait(&s, 200) == CSP_SEMAPHORE_ERROR);
    csp_assert(csp_bin_sem_remove(&s) == CSP_SEMAPHORE_OK);
    csp_log_info("semaphore working correctly");

    return 0;
}

static int get_bitrate(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: %s ifnum\n", argv[0]);
        return 1;
    }

    int ifnum = strtol(argv[1], NULL, 0);
    struct can_bittiming bittiming;
    can_opt_t opt;
    opt.data = &bittiming;
    opt.data_len = sizeof(bittiming);
    opt.opt = CANOPT_BITTIMING;

    int ret = raw_can_get_can_opt(ifnum, &opt);
    if (ret < 0) {
        printf("Error when getting bitrate: res=%d\n", ret);
        return 1;
    }

    printf("Bitrate read: bitrate=%" PRIu32 ", sample_point=%" PRIu32
           "\nbrp=%" PRIu32 "phase-seg1=%" PRIu32
           ", phase-seg2=%" PRIu32 ", sjw=%" PRIu32 "\n", bittiming.bitrate,
           bittiming.sample_point, bittiming.brp, bittiming.phase_seg1,
           bittiming.phase_seg2, bittiming.sjw);
    return 0;
}

static const shell_command_t _commands[] = {
    {"test_arch", "Test RIOT architecture implementation for libcsp", test_arch},
    {"client", "Setup client", client},
    {"server", "Setup server", server},
    {"bitrate", "Get bitrate", get_bitrate},
    { NULL, NULL, NULL},
};

int main(void)
{
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
