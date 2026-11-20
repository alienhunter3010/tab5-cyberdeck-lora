/*
 * Simulator side of deck_hal's M5-Bus serial: a pretend RAK3172 running RUI3
 * AT firmware, wired on UART1 without swap (any other combination stays
 * silent, so the probe's cycling can be seen). Enough of the AT set to drive
 * the LORA module: P2P chat, OTAA join, uplink and a canned downlink.
 */
#include "deck_hal.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define OUTQ 48

static bool s_open, s_wired;
static bool s_echo;
static int s_mode;                       /* 0 P2P, 1 LoRaWAN */
static char s_p2p[64] = "868100000:7:125:0:8:14";
static int s_precv;                      /* 0 off, 65534/65535/ms */
static char s_deveui[17] = "AC1F09FFFE0A1B2C", s_appeui[17] = "0000000000000000", s_appkey[33] = "00000000000000000000000000000000";
static int s_njm = 1, s_band = 4, s_cfm = 1;
static char s_class[2] = "A";
static bool s_joined;
static uint32_t s_next_inbound;

static struct { char text[160]; uint32_t due; bool used; } s_out[OUTQ];
static char s_cur[160];                  /* bytes of the string being drained */
static size_t s_cur_off, s_cur_len;
static char s_cmd[200];
static size_t s_cmd_len;

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static void emit(const char *s, uint32_t delay)
{
    for (int i = 0; i < OUTQ; i++) {
        if (!s_out[i].used) {
            snprintf(s_out[i].text, sizeof(s_out[i].text), "%s\r\n", s);
            s_out[i].due  = now_ms() + delay;
            s_out[i].used = true;
            return;
        }
    }
}

static void emitf(uint32_t delay, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
#include <stdarg.h>
static void emitf(uint32_t delay, const char *fmt, ...)
{
    char b[150];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    emit(b, delay);
}

static void banner(uint32_t delay)
{
    emit("RAKwireless RAK3172", delay);
    emit("------------------------------------------------------", delay);
    emit("Version: RUI_4.0.4_RAK3172-E", delay);
    emitf(delay, "Current Work Mode: %s.", s_mode ? "LoRaWAN" : "LoRa P2P");
}

static bool ishex(const char *s, size_t n)
{
    if (strlen(s) != n) return false;
    for (size_t i = 0; i < n; i++) if (!isxdigit((unsigned char)s[i])) return false;
    return true;
}

static void hex_of(const char *txt, char *out)
{
    for (; *txt; txt++) { sprintf(out, "%02X", (unsigned char)*txt); out += 2; }
}

static void query_or_set(const char *name, char *store, size_t hexlen, const char *arg)
{
    if (strcmp(arg, "?") == 0) { emitf(0, "AT+%s=%s", name, store); emit("OK", 0); return; }
    if (!ishex(arg, hexlen)) { emit("AT_PARAM_ERROR", 0); return; }
    snprintf(store, hexlen + 1, "%s", arg);
    for (char *c = store; *c; c++) *c = (char)toupper((unsigned char)*c);
    emit("OK", 0);
}

static void int_cfg(const char *name, int *v, const char *arg, int lo, int hi)
{
    if (strcmp(arg, "?") == 0) { emitf(0, "AT+%s=%d", name, *v); emit("OK", 0); return; }
    int n = atoi(arg);
    if (n < lo || n > hi) { emit("AT_PARAM_ERROR", 0); return; }
    *v = n;
    emit("OK", 0);
}

static void run_cmd(char *c)
{
    if (s_echo) emit(c, 0);
    if (strcmp(c, "AT") == 0) { emit("OK", 0); return; }
    if (strcmp(c, "ATE") == 0) { s_echo = !s_echo; emit("OK", 0); return; }
    if (strcmp(c, "ATZ") == 0) { banner(300); return; }
    if (strncmp(c, "AT+", 3) != 0) { emit("AT_ERROR", 0); return; }
    char *eq = strchr(c, '=');
    if (!eq) { emit("AT_ERROR", 0); return; }
    *eq = 0;
    const char *name = c + 3, *arg = eq + 1;

    if (strcmp(name, "VER") == 0) { emit("AT+VER=RUI_4.0.4_RAK3172-E", 0); emit("OK", 0); }
    else if (strcmp(name, "NWM") == 0) {
        if (strcmp(arg, "?") == 0) { emitf(0, "AT+NWM=%d", s_mode); emit("OK", 0); }
        else if (!strcmp(arg, "0") || !strcmp(arg, "1")) { s_mode = atoi(arg); s_joined = false; s_precv = 0; emit("OK", 0); banner(500); }
        else emit("AT_PARAM_ERROR", 0);
    }
    else if (strcmp(name, "P2P") == 0) {
        if (strcmp(arg, "?") == 0) { emitf(0, "AT+P2P=%s", s_p2p); emit("OK", 0); return; }
        if (s_mode != 0) { emit("AT_ERROR", 0); return; }
        if (s_precv) { emit("AT_BUSY_ERROR", 0); return; }
        int colons = 0; for (const char *p = arg; *p; p++) colons += *p == ':';
        if (colons != 5) { emit("AT_PARAM_ERROR", 0); return; }
        snprintf(s_p2p, sizeof(s_p2p), "%s", arg);
        emit("OK", 0);
    }
    else if (strcmp(name, "PRECV") == 0) {
        if (s_mode != 0) { emit("AT_ERROR", 0); return; }
        int v = atoi(arg);
        if (v != 0 && s_precv != 0) { emit("AT_BUSY_ERROR", 0); return; }
        s_precv = v;
        if (v) s_next_inbound = now_ms() + 9000;
        emit("OK", 0);
    }
    else if (strcmp(name, "PSEND") == 0) {
        if (s_mode != 0) { emit("AT_ERROR", 0); return; }
        if (s_precv) { emit("AT_BUSY_ERROR", 0); return; }
        size_t n = strlen(arg);
        if (n < 2 || n % 2 || !ishex(arg, n)) { emit("AT_PARAM_ERROR", 0); return; }
        emit("OK", 0);
        emit("+EVT:TXP2P DONE", 250);
    }
    else if (strcmp(name, "DEVEUI") == 0) query_or_set("DEVEUI", s_deveui, 16, arg);
    else if (strcmp(name, "APPEUI") == 0) query_or_set("APPEUI", s_appeui, 16, arg);
    else if (strcmp(name, "APPKEY") == 0) query_or_set("APPKEY", s_appkey, 32, arg);
    else if (strcmp(name, "NJM") == 0) int_cfg("NJM", &s_njm, arg, 0, 1);
    else if (strcmp(name, "BAND") == 0) int_cfg("BAND", &s_band, arg, 0, 11);
    else if (strcmp(name, "CFM") == 0) int_cfg("CFM", &s_cfm, arg, 0, 1);
    else if (strcmp(name, "CLASS") == 0) {
        if (strcmp(arg, "?") == 0) { emitf(0, "AT+CLASS=%s", s_class); emit("OK", 0); }
        else if (strlen(arg) == 1 && strchr("ABC", arg[0])) { s_class[0] = arg[0]; emit("OK", 0); }
        else emit("AT_PARAM_ERROR", 0);
    }
    else if (strcmp(name, "NJS") == 0) { emitf(0, "AT+NJS=%d", s_joined); emit("OK", 0); }
    else if (strcmp(name, "JOIN") == 0) {
        if (s_mode != 1) { emit("AT_ERROR", 0); return; }
        emit("OK", 0);
        if (strspn(s_appkey, "0") == strlen(s_appkey)) { emit("+EVT:JOIN_FAILED_RX_TIMEOUT", 3000); }
        else { s_joined = true; emit("+EVT:JOINED", 1800); }
    }
    else if (strcmp(name, "SEND") == 0) {
        if (s_mode != 1) { emit("AT_ERROR", 0); return; }
        if (!s_joined) { emit("AT_NO_NETWORK_JOINED", 0); return; }
        const char *col = strchr(arg, ':');
        if (!col || !ishex(col + 1, strlen(col + 1)) || strlen(col + 1) < 2) { emit("AT_PARAM_ERROR", 0); return; }
        emit("OK", 0);
        if (s_cfm) emit("+EVT:SEND_CONFIRMED_OK", 1500);
        char hx[16]; hex_of("ACK", hx);
        emitf(2200, "+EVT:RX_1:-92:6:UNICAST:%d:%s", atoi(arg), hx);
    }
    else emit("AT_ERROR", 0);
}

/* ---- hal_mbus_* ------------------------------------------------------------- */

bool hal_mbus_power(bool on) { printf("[sim] M5-Bus 5V %s\n", on ? "on" : "off"); return true; }

bool hal_mbus_uart_open(hal_mbus_uart_t which, bool swap, uint32_t baud)
{
    s_open  = true;
    s_wired = which == HAL_MBUS_UART1 && !swap;
    memset(s_out, 0, sizeof(s_out));
    s_cur_len = s_cur_off = s_cmd_len = 0;
    printf("[sim] M5-Bus UART open: %s @ %u (the fake module %s)\n", hal_mbus_uart_describe(which, swap), (unsigned)baud, s_wired ? "answers here" : "is not on this pair");
    return true;
}
void hal_mbus_uart_close(void) { s_open = false; }
bool hal_mbus_uart_is_open(void) { return s_open; }
void hal_mbus_uart_flush(void) { memset(s_out, 0, sizeof(s_out)); s_cur_len = s_cur_off = 0; }
void hal_mbus_reset_pulse(void) { if (s_wired) { s_precv = 0; s_joined = false; banner(200); } }

const char *hal_mbus_uart_describe(hal_mbus_uart_t which, bool swap)
{
    static char b[48];
    int rx = which == HAL_MBUS_UART0 ? 38 : 7, tx = which == HAL_MBUS_UART0 ? 37 : 6;
    if (swap) { int t = rx; rx = tx; tx = t; }
    snprintf(b, sizeof(b), "RX G%d / TX G%d  (M5-BUS %s)", rx, tx, which == HAL_MBUS_UART0 ? "13/14" : "15/16");
    return b;
}

int hal_mbus_uart_write(const uint8_t *buf, size_t n)
{
    if (!s_open) return -1;
    if (!s_wired) return (int)n;
    for (size_t i = 0; i < n; i++) {
        char c = (char)buf[i];
        if (c == '\n' || c == '\r') {
            if (s_cmd_len) { s_cmd[s_cmd_len] = 0; s_cmd_len = 0; run_cmd(s_cmd); }
        } else if (s_cmd_len < sizeof(s_cmd) - 1) {
            s_cmd[s_cmd_len++] = c;
        }
    }
    return (int)n;
}

int hal_mbus_uart_read(uint8_t *buf, size_t n)
{
    if (!s_open || !s_wired) return 0;
    uint32_t t = now_ms();
    /* A canned inbound P2P message while the module listens. */
    if (s_mode == 0 && s_precv && (int32_t)(t - s_next_inbound) >= 0) {
        char hx[100];
        hex_of("deck-2: ciao dal sim, mi senti?", hx);
        emitf(0, "+EVT:RXP2P:-71:8:%s", hx);
        s_next_inbound = t + 20000;
    }
    size_t got = 0;
    while (got < n) {
        if (s_cur_off >= s_cur_len) {
            /* next due string, oldest first */
            int best = -1;
            for (int i = 0; i < OUTQ; i++)
                if (s_out[i].used && (int32_t)(t - s_out[i].due) >= 0 && (best < 0 || (int32_t)(s_out[i].due - s_out[best].due) < 0)) best = i;
            if (best < 0) break;
            s_cur_len = strlen(s_out[best].text);
            memcpy(s_cur, s_out[best].text, s_cur_len);
            s_cur_off = 0;
            s_out[best].used = false;
        }
        buf[got++] = (uint8_t)s_cur[s_cur_off++];
    }
    return (int)got;
}
