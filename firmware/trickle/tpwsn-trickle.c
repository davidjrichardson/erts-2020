/*
 * Copyright (c) 2012, George Oikonomou - <oikonomou@users.sourceforge.net>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Example trickle-based protocol demonstrating the functionality of the
 * trickle (trickle_timer) library (RFC 6206) */
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include "dev/serial-line.h"
#include "dev/leds.h"

#include "lib/trickle-timer.h"
#include "lib/random.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define DEBUG DEBUG_PRINT

#include "sys/log.h"

#define LOG_MODULE "TPWSN-TRICKLE"
#define LOG_LEVEL LOG_LEVEL_INFO

#include "net/ipv6/uip-debug.h"

/* Trickle variables and constants */
static struct trickle_timer tt;
static long imin = 16;
static long imax = 10;
static long redundancy_const = 2;
static long msg_limit = 1;

/* Networking */
#define TRICKLE_PROTO_PORT 30001
static struct uip_udp_conn *trickle_conn;
static uip_ipaddr_t ipaddr;     /* destination: link-local all-nodes multicast */
static bool suppress_trickle = false;
static bool is_source = false;
static bool is_sink = false;
static bool reset_scheduled = false;

/*
 * For this 'protocol', nodes exchange a token (1 byte) at a frequency
 * governed by trickle. A node detects an inconsistency when it receives a
 * token different than the one it knows.
 * In this case, either:
 * - 'they' have a 'newer' token and we also update our own value, or
 * - 'we' have a 'newer' token, in which case we trigger an inconsistency
 *   without updating our value.
 * In this context, 'newer' is defined in serial number arithmetic terms.
 *
 * Every NEW_TOKEN_INTERVAL clock ticks each node will generate a new token
 * with probability 1/NEW_TOKEN_PROB. This is controlled by etimer et.
 */
#define NEW_TOKEN_INTERVAL  5 * CLOCK_SECOND
#define NEW_TOKEN_PROB      2
static uint8_t token;
static struct etimer et; /* Used to periodically generate inconsistencies */
static struct etimer rt; /* Used to 'restart' the node  */
/*---------------------------------------------------------------------------*/
PROCESS(trickle_protocol_process, "Trickle Protocol process");
AUTOSTART_PROCESSES(&trickle_protocol_process);

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void) {
    if (uip_newdata()) {
        if (is_sink) {
            // Print out that the sink received a token at time
            LOG_INFO("Sink recv'd at %lu (I=%lu, c=%u): ",
                     (unsigned long) clock_time(), (unsigned long) tt.i_cur, tt.c);
            LOG_INFO("Our token=0x%02x, theirs=0x%02x\n", token,
                     ((uint8_t *) uip_appdata)[0]);
        } else {
            // Print out that the sink received a token at time
            LOG_INFO("At %lu (I=%lu, c=%u): ",
                     (unsigned long) clock_time(), (unsigned long) tt.i_cur, tt.c);
            LOG_INFO("Our token=0x%02x, theirs=0x%02x\n", token,
                     ((uint8_t *) uip_appdata)[0]);
        }
        if (token == ((uint8_t *) uip_appdata)[0]) {
            LOG_INFO("Consistent RX\n");
            trickle_timer_consistency(&tt);
        } else {
            if ((signed char) (token - ((uint8_t *) uip_appdata)[0]) < 0) {
                LOG_INFO("Theirs is newer. Update\n");
                token = ((uint8_t *) uip_appdata)[0];
            } else {
                LOG_INFO("They are behind\n");
            }
            trickle_timer_inconsistency(&tt);

            /*
             * Here tt.ct.etimer.timer.{start + interval} points to time t in the
             * current interval. However, between t and I it points to the interval's
             * end so if you're going to use this, do so with caution.
             */
            LOG_INFO("At %lu: Trickle inconsistency. Scheduled TX for %lu\n",
                     (unsigned long) clock_time(),
                     (unsigned long) (tt.ct.etimer.timer.start +
                                      tt.ct.etimer.timer.interval));
        }
    }
    return;
}

/*---------------------------------------------------------------------------*/
static void
trickle_tx(void *ptr, uint8_t suppress) {
    /* *ptr is a pointer to the trickle_timer that triggered this callback. In
     * his example we know that ptr points to tt. However, we pretend that we did
     * not know (which would be the case if we e.g. had multiple trickle timers)
     * and cast it to a local struct trickle_timer* */
    struct trickle_timer *loc_tt = (struct trickle_timer *) ptr;

    if (suppress == TRICKLE_TIMER_TX_SUPPRESS || suppress_trickle) {
        return;
    }

    LOG_INFO("At %lu (I=%lu, c=%u): ",
             (unsigned long) clock_time(), (unsigned long) loc_tt->i_cur,
             loc_tt->c);
    LOG_INFO_("Trickle TX token 0x%02x\n", token);

    /* Instead of changing ->ripaddr around by ourselves, we could have used
     * uip_udp_packet_sendto which would have done it for us. However it puts an
     * extra ~20 bytes on stack and the cc2x3x micros hate it, so we stick with
     * send() */

    /* Destination IP: link-local all-nodes multicast */
    uip_ipaddr_copy(&trickle_conn->ripaddr, &ipaddr);
    uip_udp_packet_send(trickle_conn, &token, sizeof(token));

    /* Restore to 'accept incoming from any IP' */
    uip_create_unspecified(&trickle_conn->ripaddr);
}

/*---------------------------------------------------------------------------*/
static void
trickle_init() {
    token = 0;
    suppress_trickle = false;

    trickle_timer_config(&tt, imin, imax, redundancy_const);
    trickle_timer_set(&tt, trickle_tx, &tt);
    /*
     * At this point trickle is started and is running the first interval. All
     * nodes 'agree' that token == 0. This will change when one of them randomly
     * decides to generate a new one
     */
    etimer_set(&et, NEW_TOKEN_INTERVAL);
}

/*---------------------------------------------------------------------------*/
static void
serial_handler(char *data) {
    char *ptr = strtok(data, " ");
    char *endptr;
    long delay = 0;
    bool seen_sleep = false;
    bool seen_set = false;
    bool seen_init = false;
    bool seen_imin = false;
    bool seen_imax = false;
    bool seen_cost = false;
    bool seen_limit = false;
    bool seen_print = false;

    // Iterate over the tokenised string
    while (ptr != NULL) {
        // Parse serial input to initialise trickle
        if (strcmp(ptr, "init") == 0) {
            seen_init = true;
        }
        if (seen_init) {
            if (seen_imax && seen_imin && seen_cost) {
                break;
            } else if (seen_imin && seen_imax) {
                redundancy_const = strtol(ptr, &endptr, 10);
                seen_cost = true;
            } else if (!seen_imax && !seen_imin) {
                imax = strtol(ptr, &endptr, 10);
                seen_imax = true;
            } else if (seen_imax && !seen_imin) {
                imin = strtol(ptr, &endptr, 10);
                seen_imin = true;
            }
        }

        // Parse serial input for setting a source message limit
        if (strcmp(ptr, "limit") == 0) {
            LOG_INFO("Seen limit\n");
            seen_limit = true;
        }
        if (seen_limit) {
            msg_limit = strtol(ptr, &endptr, 10);
            LOG_INFO("Setting limit to %ld\n", msg_limit);
        }

        // Parse serial input to output the current token of the node
        if (strcmp(ptr, "print") == 0) {
            LOG_INFO("Seen print\n");
            seen_print = true;
        }
        if (seen_print) {
            LOG_INFO("Current token: %d\n", token);
            NETSTACK_RADIO.off();
            suppress_trickle = true;
        }

        // Parse serial input for restarting a node
        if (strcmp(ptr, "sleep") == 0) {
            seen_sleep = true;
        }
        if (seen_sleep) {
            delay = strtol(ptr, &endptr, 10);
        }

        // Parse serial input to set a node as a sink or source
        if (strcmp(ptr, "set")) {
            seen_set = true;
        }
        if (seen_set) {
            if (strcmp(ptr, "sink") == 0) {
                LOG_INFO("Setting node status to SINK\n");
                is_sink = true;
                trickle_init();
            } else if (strcmp(ptr, "source") == 0) {
                LOG_INFO("Setting node status to SOURCE\n");
                is_source = true;
                trickle_init();
            }
        }

        ptr = strtok(NULL, " ");
    }

    if (seen_sleep && delay > 0) {
        LOG_INFO("Restarting with delay of %ld seconds\n", delay);

        NETSTACK_RADIO.off();
        etimer_set(&rt, (delay * CLOCK_SECOND));
        suppress_trickle = true;
        reset_scheduled = true;
        leds_on(LEDS_ALL);
    }
}

/*-------------------------------------œ--------------------------------------*/
static void
restart_node(void) {
    // Reset the internal trickle state to emulate power loss
    trickle_init();
    etimer_stop(&rt);
    reset_scheduled = false;
    NETSTACK_RADIO.on();
    leds_off(LEDS_ALL);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(trickle_protocol_process, ev, data) {
    PROCESS_BEGIN();

                LOG_INFO("Trickle protocol started\n");

                uip_create_linklocal_allnodes_mcast(&ipaddr); /* Store for later */

                trickle_conn = udp_new(NULL, UIP_HTONS(TRICKLE_PROTO_PORT), NULL);
                udp_bind(trickle_conn, UIP_HTONS(TRICKLE_PROTO_PORT));

                LOG_INFO("Connection: local/remote port %u/%u\n",
                         UIP_HTONS(trickle_conn->lport), UIP_HTONS(trickle_conn->rport));

                trickle_init();

                while (1) {
                    PROCESS_YIELD();
                    if (ev == tcpip_event) {
                        tcpip_handler();
                    } else if (ev == serial_line_event_message && data != NULL) {
                        serial_handler(data);
                    } else if (etimer_expired(&et) && is_source) {
                        /* Periodically (and randomly) generate a new token. This will trigger
                         * a trickle inconsistency */
                        // Will only trigger a new token if the node is marked as a source node
                        if ((random_rand() % NEW_TOKEN_PROB) == 0 && token < msg_limit) {
                            token++;
                            LOG_INFO("At %lu: Generating a new token 0x%02x\n",
                                     (unsigned long) clock_time(), token);
                            trickle_timer_reset_event(&tt);
                        }
                        etimer_set(&et, NEW_TOKEN_INTERVAL);
                    } else if (etimer_expired(&rt) && reset_scheduled) {
                        LOG_INFO("Restarting node at time %lu\n", (unsigned long) clock_time());
                        restart_node();
                    }
                }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
