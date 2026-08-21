/*
 * SPDX-License-Identifier: CC0-1.0
 *
 * TOE-backend socket shim for ESP-MQTT, in two layers:
 *   1. fcntl()/select(), which esp-tls and tcp_transport need and the
 *      component's 13-symbol --wrap list lacks.
 *   2. Routing one level BELOW the component's wraps: its __wrap_lwip_* are
 *      thin translators over wiztoe_* symbols, so wrapping THOSE
 *      (--wrap=wiztoe_*, main/CMakeLists.txt) steers each new socket() to the
 *      chip's hardware TCP/IP (Ethernet) or, via __real_lwip_*, to the real
 *      LwIP the Wi-Fi STA lives on. ESP-MQTT reconnects with a fresh socket,
 *      so failover is just this decision re-running.
 *
 * fd convention: the component passes `slot = fd - LWIP_SOCKET_OFFSET` down.
 * A TOE slot is the hardware socket number; the LwIP route returns
 * `real_fd - OFFSET` so the component's `+ OFFSET` reconstructs the true fd.
 * One shared slot namespace — safe ONLY because ESP-MQTT is the single socket
 * consumer here (one broker socket at a time; Wi-Fi DHCP/DNS use LwIP's raw
 * API). The ownership table guards that assumption with a loud log.
 *
 * Scope: plain mqtt:// with an IP-literal broker. mqtts:// needs read()/write()
 * (internal calls --wrap cannot see); hostnames resolve on the Wi-Fi route
 * only; bind()/getsockname() are not re-routed (unused by the client path).
 *
 * NOTE: every wiztoe_* symbol, constant and enum below is PRIVATE component API
 * copied from port/ioLibrary_Driver/inc/wiznet_toe.h — re-check on component
 * updates; enum value drift is the one change the linker cannot catch.
 */
#include "sdkconfig.h"

#if defined(CONFIG_WSM_DRIVER_SOCKET_WRAP) && CONFIG_WSM_DRIVER_SOCKET_WRAP

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

/* Order matters: wizchip_conf.h defines SOCK_STREAM/SOCK_DGRAM, net_backend.h
 * #undefs them so lwip/sockets.h wins — same rule as app_main.c. It also
 * provides the register accessors (getSn_*, wizphy_getphylink) used below. */
#include "wizchip_conf.h"
#include "net_backend.h"

#include "lwip/sockets.h"

#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "wifi_backend.h"       /* wifi_net_is_up() — tracks got-IP/disconnect */
#endif

#include "toe_socket_shim.h"

static const char *TAG = "toe_shim";

/* ---- real LwIP entry points (same __real_* escape as net_wifi_ops.c) ---- */
extern int     __real_lwip_socket(int domain, int type, int protocol);
extern int     __real_lwip_connect(int s, const struct sockaddr *name, socklen_t namelen);
extern ssize_t __real_lwip_send(int s, const void *data, size_t size, int flags);
extern ssize_t __real_lwip_recv(int s, void *mem, size_t len, int flags);
extern int     __real_lwip_close(int s);
extern int     __real_lwip_setsockopt(int s, int level, int optname, const void *optval, socklen_t optlen);
extern int     __real_lwip_getsockopt(int s, int level, int optname, void *optval, socklen_t *optlen);
extern int     __real_lwip_fcntl(int s, int cmd, int val);
extern int     __real_lwip_select(int maxfdp1, fd_set *readset, fd_set *writeset,
                                  fd_set *exceptset, struct timeval *timeout);

/* ---- real wiztoe entry points, re-declared (PRIV header; see file NOTE) ---- */
extern int __real_wiztoe_socket(int domain, int type, int protocol);
extern int __real_wiztoe_connect(int fd, const uint8_t ip[4], uint16_t port);
extern int __real_wiztoe_send(int fd, const void *buf, size_t len);
extern int __real_wiztoe_recv(int fd, void *buf, size_t len);
extern int __real_wiztoe_close(int fd);
extern int __real_wiztoe_setsockopt(int fd, int opt, const void *val, size_t len);
extern int __real_wiztoe_getsockopt(int fd, int opt, void *val, size_t *len);

/* From wiznet_toe.h: the SO_RCVTIMEO return; must match or a timeout turns
 * into a hard EIO upstairs. */
#define WIZTOE_ERR_TIMEOUT (-2)

/* From wiznet_toe.h: wiztoe_opt_t in declaration order (implicit 0..10). */
enum {
    TOE_OPT_KEEPALIVE = 0,
    TOE_OPT_KEEPIDLE,
    TOE_OPT_NODELAY,
    TOE_OPT_TTL,
    TOE_OPT_TOS,
    TOE_OPT_RCVTIMEO_MS,
    TOE_OPT_SNDTIMEO_MS,
    TOE_OPT_RCVBUF,
    TOE_OPT_SNDBUF,
    TOE_OPT_ERROR,
    TOE_OPT_TYPE,
};

/* ---- ownership: one byte per slot, sized for the larger fd space. No lock —
 * every lifecycle call runs on the MQTT task (see header). */
typedef enum { OWNER_NONE = 0, OWNER_TOE, OWNER_REAL } slot_owner_t;

#if CONFIG_LWIP_MAX_SOCKETS > _WIZCHIP_SOCK_NUM_
#define TOE_SLOT_COUNT CONFIG_LWIP_MAX_SOCKETS
#else
#define TOE_SLOT_COUNT _WIZCHIP_SOCK_NUM_
#endif

static uint8_t s_owner[TOE_SLOT_COUNT];

static bool slot_valid(int slot)
{
    return slot >= 0 && slot < TOE_SLOT_COUNT;
}

static bool slot_is_real(int slot)
{
    return slot_valid(slot) && s_owner[slot] == OWNER_REAL;
}

/* ---- link-state policy: preference first, live link second ------------------
 * Ethernet state is the PHY register, NOT wiznet_net_is_up() (an init flag that
 * never clears — used only as the SPI-safe gate). One SPI read, mutex-
 * serialized; fine at connect/1 Hz frequency, NOT per select() iteration.
 */
bool toe_shim_eth_link_up(void)
{
#if CONFIG_EXAMPLE_CONNECT_ETHERNET
    /* W5500 answers 0/1, W6300 PHYSR bit0; > 0 covers both, -1 counts as down. */
    return wiznet_net_is_up() && wizphy_getphylink() > 0;
#else
    return false;
#endif
}

static bool shim_wifi_up(void)
{
#if CONFIG_EXAMPLE_CONNECT_WIFI
    return wifi_net_is_up();
#else
    return false;
#endif
}

typedef enum { ROUTE_TOE, ROUTE_REAL, ROUTE_NONE } route_t;

/* One-shot failure memory: a path can be dead BEYOND a healthy PHY, where
 * preferred-first would re-pick the same dead route forever. A failed connect
 * marks its route; the next decision steps around it once; success clears. */
static route_t s_last_failed_route = ROUTE_NONE;

static const char *s_active_route = "none";

const char *toe_shim_active_route(void)
{
    return s_active_route;
}

static route_t pick_route(void)
{
    bool eth_up  = toe_shim_eth_link_up();
    bool wifi_up = shim_wifi_up();

#if CONFIG_EXAMPLE_PRIMARY_WIFI
    const bool prefer_eth = false;
#else
    /* Also the single-interface answer: with one side off its *_up is false. */
    const bool prefer_eth = true;
#endif

    route_t route = ROUTE_NONE;
    if (prefer_eth) {
        if (eth_up)       route = ROUTE_TOE;
        else if (wifi_up) route = ROUTE_REAL;
    } else {
        if (wifi_up)      route = ROUTE_REAL;
        else if (eth_up)  route = ROUTE_TOE;
    }

    /* Consume the one-shot failure mark: if the choice above just failed to
     * connect and the other interface is usable, step around it this once. */
    route_t failed = s_last_failed_route;
    s_last_failed_route = ROUTE_NONE;
    if (route != ROUTE_NONE && route == failed) {
        route_t other = (route == ROUTE_TOE) ? (wifi_up ? ROUTE_REAL : route)
                                             : (eth_up  ? ROUTE_TOE  : route);
        if (other != route) {
            ESP_LOGW(TAG, "last connect on that route failed — trying the other interface first");
            route = other;
        }
    }

    if (route == ROUTE_TOE) {
        s_active_route = "eth-toe";
        ESP_LOGI(TAG, "mqtt socket -> TOE ethernet (eth link=%d wifi=%d)", eth_up, wifi_up);
    } else if (route == ROUTE_REAL) {
        s_active_route = "wifi-lwip";
        ESP_LOGI(TAG, "mqtt socket -> lwIP wifi (eth link=%d wifi=%d)", eth_up, wifi_up);
    } else {
        s_active_route = "none";
        ESP_LOGW(TAG, "no interface is up — failing socket(), ESP-MQTT will retry");
    }
    return route;
}

/* ---- readiness: register reads only, chip-owned slots only ---------------- */
#define TOE_POLL_RD 0x01
#define TOE_POLL_WR 0x02

static int toe_poll_one(uint8_t sn)
{
    uint8_t sr = getSn_SR(sn);
    int ev = 0;

    /* Not-ESTABLISHED counts as readable ON PURPOSE: tcp_transport detects a
     * clean close only from "poll said readable, recv returned 0". */
    if (getSn_RX_RSR(sn) > 0 || sr != SOCK_ESTABLISHED) {
        ev |= TOE_POLL_RD;
    }
    if (sr == SOCK_ESTABLISHED && getSn_TX_FSR(sn) > 0) {
        ev |= TOE_POLL_WR;
    }
    return ev;
}

/* ---- socket: the routing decision point. Failure -> ENFILE upstairs, and
 * ESP-MQTT retries after its reconnect delay — wanted while nothing is up. */
int __wrap_wiztoe_socket(int domain, int type, int protocol)
{
    route_t route = pick_route();

    if (route == ROUTE_TOE) {
        int slot = __real_wiztoe_socket(domain, type, protocol);
        if (slot < 0) {
            return -1;
        }
        if (!slot_valid(slot) || s_owner[slot] != OWNER_NONE) {
            /* Single-socket assumption broken: refuse, do not cross-route. */
            ESP_LOGE(TAG, "slot %d already owned (%u) — single-socket assumption broken",
                     slot, (unsigned)(slot_valid(slot) ? s_owner[slot] : 0));
            __real_wiztoe_close(slot);
            return -1;
        }
        s_owner[slot] = OWNER_TOE;
        return slot;
    }

    if (route == ROUTE_REAL) {
        int real_fd = __real_lwip_socket(domain, type, protocol);
        if (real_fd < 0) {
            return -1;
        }
        int slot = real_fd - LWIP_SOCKET_OFFSET;
        if (!slot_valid(slot) || s_owner[slot] != OWNER_NONE) {
            ESP_LOGE(TAG, "slot %d already owned (%u) — single-socket assumption broken",
                     slot, (unsigned)(slot_valid(slot) ? s_owner[slot] : 0));
            __real_lwip_close(real_fd);
            return -1;
        }
        s_owner[slot] = OWNER_REAL;
        return slot;
    }

    return -1;
}

/* ---- connect ---------------------------------------------------------------
 * LwIP route: rebuild the sockaddr (inverse of the component's conversion) and
 * absorb EINPROGRESS HERE — the fd really is non-blocking, and the component
 * collapses any failure into ECONNREFUSED, which esp-tls treats as fatal.
 * Waiting select-for-writable + SO_ERROR keeps connect() synchronous on both
 * routes. Budget mirrors ESP_TLS_DEFAULT_CONN_TIMEOUT (10 s).
 */
#define TOE_CONNECT_BUDGET_S 10

static int real_connect_blocking(int real_fd, const uint8_t ip[4], uint16_t port)
{
    struct sockaddr_in sin;
    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_port = lwip_htons(port);
    sin.sin_addr.s_addr = lwip_htonl(((uint32_t)ip[0] << 24) | ((uint32_t)ip[1] << 16) |
                                     ((uint32_t)ip[2] << 8) | ip[3]);

    if (__real_lwip_connect(real_fd, (struct sockaddr *)&sin, sizeof(sin)) == 0) {
        return 0;
    }
    if (errno != EINPROGRESS) {
        return -1;      /* the component maps this to errno = ECONNREFUSED */
    }

    struct timeval tv = { .tv_sec = TOE_CONNECT_BUDGET_S, .tv_usec = 0 };
    fd_set wset;
    FD_ZERO(&wset);
    FD_SET(real_fd, &wset);

    if (__real_lwip_select(real_fd + 1, NULL, &wset, NULL, &tv) <= 0) {
        return -1;      /* timeout, or select itself failed */
    }

    int sockerr = 0;
    socklen_t len = sizeof(sockerr);
    if (__real_lwip_getsockopt(real_fd, SOL_SOCKET, SO_ERROR, &sockerr, &len) < 0 || sockerr != 0) {
        return -1;
    }
    return 0;
}

int __wrap_wiztoe_connect(int fd, const uint8_t ip[4], uint16_t port)
{
    int rc;

    /* Both branches feed the one-shot failure memory in pick_route(). */
    if (!slot_is_real(fd)) {
        rc = __real_wiztoe_connect(fd, ip, port);
        s_last_failed_route = (rc == 0) ? ROUTE_NONE : ROUTE_TOE;
        return rc;
    }

    rc = real_connect_blocking(fd + LWIP_SOCKET_OFFSET, ip, port);
    s_last_failed_route = (rc == 0) ? ROUTE_NONE : ROUTE_REAL;
    return rc;
}

/* ---- recv: map the real stack's answers into wiztoe conventions (0 = EOF,
 * EWOULDBLOCK -> WIZTOE_ERR_TIMEOUT, else -1; precise errno flattens to EIO
 * upstairs, which tcp_transport treats as dead either way). */
int __wrap_wiztoe_recv(int fd, void *buf, size_t len)
{
    if (!slot_is_real(fd)) {
        return __real_wiztoe_recv(fd, buf, len);
    }

    ssize_t n = __real_lwip_recv(fd + LWIP_SOCKET_OFFSET, buf, len, 0);
    if (n >= 0) {
        return (int)n;
    }
    if (errno == EWOULDBLOCK || errno == EAGAIN) {
        return WIZTOE_ERR_TIMEOUT;
    }
    return -1;
}

/* ---- send: route, and absorb SOCK_BUSY on the TOE side ---------------------
 * ioLibrary reports "chip still flushing" as SOCK_BUSY (== 0), which escapes
 * the component as a successful zero-byte write — silent no-progress or data
 * loss. Retry here until progress or a deadline. Pre-existing defect, not a
 * routing artifact.
 */
#define TOE_SEND_BUDGET_US (10 * 1000 * 1000)

int __wrap_wiztoe_send(int fd, const void *buf, size_t len)
{
    if (slot_is_real(fd)) {
        ssize_t n = __real_lwip_send(fd + LWIP_SOCKET_OFFSET, buf, len, 0);
        return (n < 0) ? -1 : (int)n;
    }

    int64_t deadline_us = esp_timer_get_time() + TOE_SEND_BUDGET_US;

    for (;;) {
        int n = __real_wiztoe_send(fd, buf, len);

        /* A genuine zero-length request may legitimately return 0. */
        if (n != 0 || len == 0) {
            return n;
        }
        if (esp_timer_get_time() >= deadline_us) {
            return -1;      /* the component maps this to errno = EIO */
        }
        vTaskDelay(1);
    }
}

/* ---- close: clear ownership even if the underlying close fails — a stale
 * entry would misroute the slot's next owner. */
int __wrap_wiztoe_close(int fd)
{
    if (slot_is_real(fd)) {
        s_owner[fd] = OWNER_NONE;
        return __real_lwip_close(fd + LWIP_SOCKET_OFFSET);
    }
    if (slot_valid(fd)) {
        s_owner[fd] = OWNER_NONE;
    }
    return __real_wiztoe_close(fd);
}

/* ---- setsockopt / getsockopt: undo the component's POSIX -> wiztoe-code
 * conversion for LwIP-owned fds. RCVTIMEO/SNDTIMEO travel as uint32 ms and
 * need their timeval back; unknown codes fail visibly (-1 -> EINVAL).
 * TCP_KEEPINTVL/KEEPCNT never arrive — the component rejects them upstairs,
 * which is why app_main.c disables tcp_keep_alive_cfg under this wrap.
 */
int __wrap_wiztoe_setsockopt(int fd, int opt, const void *val, size_t len)
{
    if (!slot_is_real(fd)) {
        return __real_wiztoe_setsockopt(fd, opt, val, len);
    }

    int real_fd = fd + LWIP_SOCKET_OFFSET;

    switch (opt) {
    case TOE_OPT_KEEPALIVE:
        return __real_lwip_setsockopt(real_fd, SOL_SOCKET, SO_KEEPALIVE, val, (socklen_t)len);
    case TOE_OPT_KEEPIDLE:
        return __real_lwip_setsockopt(real_fd, IPPROTO_TCP, TCP_KEEPIDLE, val, (socklen_t)len);
    case TOE_OPT_NODELAY:
        return __real_lwip_setsockopt(real_fd, IPPROTO_TCP, TCP_NODELAY, val, (socklen_t)len);
    case TOE_OPT_TTL:
        return __real_lwip_setsockopt(real_fd, IPPROTO_IP, IP_TTL, val, (socklen_t)len);
    case TOE_OPT_TOS:
        return __real_lwip_setsockopt(real_fd, IPPROTO_IP, IP_TOS, val, (socklen_t)len);
    case TOE_OPT_RCVTIMEO_MS:
    case TOE_OPT_SNDTIMEO_MS: {
        if (val == NULL || len < sizeof(uint32_t)) {
            return -1;
        }
        uint32_t ms = *(const uint32_t *)val;
        struct timeval tv = {
            .tv_sec  = (long)(ms / 1000),
            .tv_usec = (long)((ms % 1000) * 1000),
        };
        int optname = (opt == TOE_OPT_RCVTIMEO_MS) ? SO_RCVTIMEO : SO_SNDTIMEO;
        return __real_lwip_setsockopt(real_fd, SOL_SOCKET, optname, &tv, sizeof(tv));
    }
    default:
        return -1;
    }
}

int __wrap_wiztoe_getsockopt(int fd, int opt, void *val, size_t *len)
{
    if (!slot_is_real(fd)) {
        return __real_wiztoe_getsockopt(fd, opt, val, len);
    }

    int real_fd = fd + LWIP_SOCKET_OFFSET;
    int optname;

    switch (opt) {
    case TOE_OPT_ERROR:  optname = SO_ERROR;  break;
    case TOE_OPT_TYPE:   optname = SO_TYPE;   break;
    case TOE_OPT_RCVBUF: optname = SO_RCVBUF; break;
    case TOE_OPT_SNDBUF: optname = SO_SNDBUF; break;
    case TOE_OPT_RCVTIMEO_MS:
    case TOE_OPT_SNDTIMEO_MS: {
        struct timeval tv = { 0 };
        socklen_t sl = sizeof(tv);
        optname = (opt == TOE_OPT_RCVTIMEO_MS) ? SO_RCVTIMEO : SO_SNDTIMEO;
        if (val == NULL || len == NULL || *len < sizeof(uint32_t)) {
            return -1;
        }
        if (__real_lwip_getsockopt(real_fd, SOL_SOCKET, optname, &tv, &sl) < 0) {
            return -1;
        }
        *(uint32_t *)val = (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
        *len = sizeof(uint32_t);
        return 0;
    }
    default:
        return -1;
    }

    if (val == NULL || len == NULL) {
        return -1;
    }
    socklen_t sl = (socklen_t)*len;
    if (__real_lwip_getsockopt(real_fd, SOL_SOCKET, optname, val, &sl) < 0) {
        return -1;
    }
    *len = (size_t)sl;
    return 0;
}

/* ---- fcntl -----------------------------------------------------------------
 * LwIP-owned fds take the real call (O_NONBLOCK genuinely applies — hence the
 * EINPROGRESS absorption in connect above). For chip-owned fds esp-tls toggles
 * O_NONBLOCK around connect() and treats EITHER call failing as fatal, so both
 * must succeed; the bit is remembered for F_GETFL but not applied — the TOE
 * connect is synchronous and recv() is bounded by SO_RCVTIMEO instead.
 */
static uint16_t s_nonblock;     /* one bit per hardware socket */

int __wrap_lwip_fcntl(int s, int cmd, int val)
{
    int sn = s - LWIP_SOCKET_OFFSET;

    if (slot_is_real(sn)) {
        return __real_lwip_fcntl(s, cmd, val);
    }

    if (sn < 0 || sn >= _WIZCHIP_SOCK_NUM_) {
        errno = EBADF;
        return -1;
    }

    switch (cmd) {
    case F_GETFL:
        errno = 0;
        return O_RDWR | ((s_nonblock & (1u << sn)) ? O_NONBLOCK : 0);

    case F_SETFL:
        if (val & O_NONBLOCK) s_nonblock |=  (uint16_t)(1u << sn);
        else                  s_nonblock &= (uint16_t)~(1u << sn);
        errno = 0;
        return 0;

    default:
        /* F_GETFD and friends: meaningless here, and failing is fatal upstairs. */
        errno = 0;
        return 0;
    }
}

/* ---- select ----------------------------------------------------------------
 * Ownership decides which stack answers. ESP-MQTT's two reachable call sites
 * (transport poll_read/poll_write) are single-fd, so the sets are uniform;
 * mixed sets mean a broken ownership table and fail hard with EBADF.
 * LwIP-owned sets are forwarded wholesale.
 *
 * Chip-owned polling NEVER populates exceptfds: a non-empty except set makes
 * tcp_transport read SO_ERROR and fail the poll regardless — an unbreakable
 * silent reconnect loop ("Poll read error: 0"). Trouble is reported as
 * readable instead, so the following recv() turns it into a clean EOF.
 */
int __wrap_lwip_select(int maxfdp1, fd_set *readset, fd_set *writeset,
                       fd_set *exceptset, struct timeval *timeout)
{
    int n_real = 0;
    int n_toe = 0;

    for (int s = LWIP_SOCKET_OFFSET; s < maxfdp1; s++) {
        int want = (readset   != NULL && FD_ISSET(s, readset))  ||
                   (writeset  != NULL && FD_ISSET(s, writeset)) ||
                   (exceptset != NULL && FD_ISSET(s, exceptset));
        int slot = s - LWIP_SOCKET_OFFSET;

        if (!want) {
            continue;
        }
        if (slot_is_real(slot)) {
            n_real++;
        } else if (slot_valid(slot) && s_owner[slot] == OWNER_TOE && slot < _WIZCHIP_SOCK_NUM_) {
            n_toe++;
        } else {
            /* Neither stack handed this out; answer EBADF like real select(). */
            errno = EBADF;
            return -1;
        }
    }

    if (n_real > 0) {
        if (n_toe > 0) {
            errno = EBADF;      /* mixed sets: see the header comment above */
            return -1;
        }
        return __real_lwip_select(maxfdp1, readset, writeset, exceptset, timeout);
    }

    /* All chip-owned (or empty, which degenerates to a plain timed wait). */

    /* A NULL timeout means block until something is ready. */
    int64_t deadline_us = INT64_MAX;
    if (timeout != NULL) {
        deadline_us = esp_timer_get_time()
                    + (int64_t)timeout->tv_sec * 1000000 + (int64_t)timeout->tv_usec;
    }

    for (;;) {
        fd_set r, w;
        int ready = 0;

        FD_ZERO(&r);
        FD_ZERO(&w);

        for (int s = LWIP_SOCKET_OFFSET; s < maxfdp1; s++) {
            int want_r = (readset  != NULL) && FD_ISSET(s, readset);
            int want_w = (writeset != NULL) && FD_ISSET(s, writeset);
            int sn     = s - LWIP_SOCKET_OFFSET;

            if (!want_r && !want_w) {
                continue;
            }

            int ev = toe_poll_one((uint8_t)sn);
            if (want_r && (ev & TOE_POLL_RD)) { FD_SET(s, &r); ready++; }
            if (want_w && (ev & TOE_POLL_WR)) { FD_SET(s, &w); ready++; }
        }

        if (ready > 0 || esp_timer_get_time() >= deadline_us) {
            if (readset   != NULL) *readset = r;
            if (writeset  != NULL) *writeset = w;
            if (exceptset != NULL) FD_ZERO(exceptset);
            errno = 0;
            return ready;
        }

        /* One tick is the floor (no ISR on the INT pin). Needs
         * CONFIG_FREERTOS_HZ=1000: at 100 Hz this is 10 ms and the component's
         * yield-counted timeouts run 10x long. Pinned in sdkconfig.defaults;
         * an EXISTING sdkconfig must be updated by hand in menuconfig. */
        vTaskDelay(1);
    }
}

#endif /* CONFIG_WSM_DRIVER_SOCKET_WRAP */
