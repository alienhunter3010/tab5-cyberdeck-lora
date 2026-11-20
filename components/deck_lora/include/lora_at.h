/*
 * deck_lora: host side of the RUI3 AT interface (RAK3172 / STM32WLE5) that
 * the M5Stack Module LoRaWAN-EU868 ships with.
 *
 * Pure C, no ESP-IDF and no LVGL: bytes go in and out through `lora_io_t`,
 * so the same code runs on the Tab5 (deck_hal UART), in the simulator (a fake
 * module) and in host unit tests. Nothing blocks: commands are queued and
 * lora_link_tick() is called from a timer, writes one command at a time,
 * parses whatever the module sent and reports finished commands and
 * asynchronous "+EVT:" events through callbacks.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Transport ------------------------------------------------------------ */

typedef struct {
    int (*read)(uint8_t *buf, size_t n, void *ctx);          /* non-blocking, bytes read (0 = none) */
    int (*write)(const uint8_t *buf, size_t n, void *ctx);   /* bytes written, < 0 on error */
    void *ctx;
} lora_io_t;

/* ---- Events (the module's "+EVT:" lines) ----------------------------------- */

typedef enum {
    LORA_EVT_NONE = 0,
    LORA_EVT_JOINED,
    LORA_EVT_JOIN_FAILED,
    LORA_EVT_SEND_OK,        /* +EVT:SEND_CONFIRMED_OK */
    LORA_EVT_SEND_FAILED,    /* +EVT:SEND_CONFIRMED_FAILED */
    LORA_EVT_TX_DONE,        /* +EVT:TXP2P DONE */
    LORA_EVT_RX_P2P,         /* +EVT:RXP2P:<rssi>:<snr>:<hex> */
    LORA_EVT_RX_WAN,         /* +EVT:RX_1:<rssi>:<snr>:UNICAST:<port>:<hex> */
    LORA_EVT_RX_TIMEOUT,     /* +EVT:RXP2P RECEIVE TIMEOUT */
    LORA_EVT_BOOT,           /* the module's reset banner */
    LORA_EVT_OTHER,
} lora_evt_type_t;

#define LORA_PAYLOAD_MAX 255

typedef struct {
    lora_evt_type_t type;
    int rssi;                         /* dBm (RX events) */
    int snr;                          /* dB  (RX events) */
    int port;                         /* LoRaWAN FPort (RX_WAN), else 0 */
    size_t len;                       /* payload bytes */
    uint8_t data[LORA_PAYLOAD_MAX];   /* payload (RX events) */
    char text[96];                    /* the raw line, trimmed (OTHER / JOIN_FAILED reason) */
} lora_evt_t;

/* Parse one line from the module. Returns false if it is not an event. */
bool lora_parse_evt(const char *line, lora_evt_t *out);

/* ---- Commands ------------------------------------------------------------- */

typedef enum {
    LORA_OK = 0,
    LORA_ERR_AT,        /* the module answered AT_ERROR, AT_PARAM_ERROR, ... (text in `reply`) */
    LORA_ERR_TIMEOUT,   /* no answer */
    LORA_ERR_DROPPED,   /* the link was flushed or freed before the command ran */
    LORA_ERR_IO,        /* the transport failed */
} lora_rc_t;

/* Command flags. */
#define LORA_F_REBOOT 0x01  /* the module restarts: done when the banner has gone by (or on timeout) */

/* `reply` holds the lines before OK / the error, without the echo, '\n' separated. */
typedef void (*lora_done_cb)(lora_rc_t rc, const char *reply, int tag, void *user);
typedef void (*lora_evt_cb)(const lora_evt_t *evt, void *user);

typedef struct lora_link lora_link_t;

lora_link_t *lora_link_new(const lora_io_t *io, lora_evt_cb on_evt, void *user);
void         lora_link_free(lora_link_t *l);

/* Queue a command (without the trailing CR/LF), e.g. "AT+NWM=?". `done` may be
 * NULL. Returns false if the queue is full. */
bool lora_link_send(lora_link_t *l, const char *cmd, uint32_t timeout_ms, uint8_t flags,
                    lora_done_cb done, int tag, void *user);

/* Drive the link. `now_ms` is any monotonic millisecond clock. */
void lora_link_tick(lora_link_t *l, uint32_t now_ms);

bool lora_link_busy(const lora_link_t *l);          /* a command is in flight or queued */
void lora_link_flush(lora_link_t *l);               /* drop everything queued (the in-flight one too) */
uint32_t lora_link_rx_bytes(const lora_link_t *l);  /* total bytes heard: "the module is talking" */

/* ---- Helpers --------------------------------------------------------------- */

/* "AT+VER=RUI_4.0.4_RAK3172-E" -> "RUI_4.0.4_RAK3172-E". Pass the command name
 * without "AT+" ("VER"). If the first line has no prefix it is returned as is. */
bool lora_reply_value(const char *reply, const char *name, char *out, size_t n);

size_t lora_hex_encode(const uint8_t *in, size_t n, char *out, size_t out_n);   /* upper case, NUL terminated */
int    lora_hex_decode(const char *hex, uint8_t *out, size_t out_n);            /* bytes, -1 if malformed */

/* ---- LoRa time on air and the EU868 duty cycle ----------------------------- */

/* Milliseconds a packet of `payload_len` bytes occupies the air (explicit
 * header, CRC on). bw_khz 125/250/500, cr 1..4 meaning 4/5..4/8. */
uint32_t lora_airtime_ms(unsigned payload_len, unsigned sf, unsigned bw_khz, unsigned cr, unsigned preamble);

/* Rolling one-hour budget. 1 % of an hour is 36 s. */
typedef struct {
    uint32_t used_ms[60];   /* airtime per minute, ring */
    uint32_t minute;        /* minute number of the newest slot */
    bool     started;
    uint32_t limit_ms;      /* e.g. 36000 */
} lora_duty_t;

void     lora_duty_init(lora_duty_t *d, uint32_t limit_ms);
uint32_t lora_duty_used(lora_duty_t *d, uint32_t now_ms);                 /* airtime in the last hour */
bool     lora_duty_allow(lora_duty_t *d, uint32_t now_ms, uint32_t airtime_ms);
void     lora_duty_add(lora_duty_t *d, uint32_t now_ms, uint32_t airtime_ms);

#ifdef __cplusplus
}
#endif
