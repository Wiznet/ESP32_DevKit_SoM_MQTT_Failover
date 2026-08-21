/*
 * SPDX-License-Identifier: CC0-1.0
 *
 * Helpers from toe_socket_shim.c. Only exist under
 * CONFIG_WSM_DRIVER_SOCKET_WRAP (TOE) - callers need the same guard.
 */
#ifndef TOE_SOCKET_SHIM_H
#define TOE_SOCKET_SHIM_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Live PHY link state (one SPI read, gated on chip init). NOT
 * wiznet_net_is_up(), which is a never-cleared init flag. */
bool toe_shim_eth_link_up(void);

/* Short name of the path the most recent socket() was routed to: "eth-toe",
 * "wifi-lwip", or "none". For log and test-payload use only. */
const char *toe_shim_active_route(void);

#ifdef __cplusplus
}
#endif

#endif /* TOE_SOCKET_SHIM_H */
