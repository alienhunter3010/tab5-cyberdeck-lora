/*
 * RUI3 AT link, event parser and EU868 helpers. See lora_at.h.
 *
 * RUI3 facts this relies on (RAKwireless RUI3 AT Command Manual):
 *  - 115200 8N1, commands end in CR LF, echo is OFF by default (ATE toggles
 *    it) but an echoed command line is skipped anyway;
 *  - every command ends in "OK" or in an "AT_..." error line;
 *  - a query answers "AT+NAME=value" before the OK;
 *  - asynchronous events arrive as "+EVT:..." lines at any time.
 */
#include "lora_at.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define QUEUE_N   16
#define CMD_MAX   200
#define LINE_MAX_ 600
#define REPLY_MAX 600

typedef struct {
    char cmd[CMD_MAX];
    uint32_t timeout_ms;
    uint8_t flags;
    lora_done_cb done;
    int tag;
    void *user;
} job_t;

struct lora_link {
    lora_io_t io;
    lora_evt_cb on_evt;
    void *user;

    job_t queue[QUEUE_N];
    int q_head, q_count;

    bool active;
    job_t cur;
    uint32_t sent_at;
    char reply[REPLY_MAX];
    size_t reply_len;
    bool saw_output;

    char line[LINE_MAX_];
    size_t line_len;
    uint32_t rx_total;
};

/* ---- small helpers --------------------------------------------------------- */

static char *trim(char *s)
{
    while (*s && isspace((unsigned char)*s)) s++;
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) s[--n] = 0;
    return s;
}

static bool starts(const char *s, const char *p) { return strncmp(s, p, strlen(p)) == 0; }

size_t lora_hex_encode(const uint8_t *in, size_t n, char *out, size_t out_n)
{
    static const char H[] = "0123456789ABCDEF";
    if (out_n == 0) return 0;
    size_t o = 0;
    for (size_t i = 0; i < n && o + 2 < out_n; i++) {
        out[o++] = H[in[i] >> 4];
        out[o++] = H[in[i] & 15];
    }
    out[o] = 0;
    return o;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int lora_hex_decode(const char *hex, uint8_t *out, size_t out_n)
{
    size_t n = strlen(hex);
    if (n % 2) return -1;
    if (n / 2 > out_n) return -1;
    for (size_t i = 0; i < n; i += 2) {
        int a = hexval(hex[i]), b = hexval(hex[i + 1]);
        if (a < 0 || b < 0) return -1;
        out[i / 2] = (uint8_t)(a << 4 | b);
    }
    return (int)(n / 2);
}

/* Next ':' separated field in *p (NUL-terminates it, advances *p). */
static char *field(char **p)
{
    char *s = *p;
    if (!s) return NULL;
    char *c = strchr(s, ':');
    if (c) {
        *c = 0;
        *p = c + 1;
    } else {
        *p = NULL;
    }
    return s;
}

static bool parse_int(const char *s, int *out)
{
    if (!s || !*s) return false;
    char *end;
    long v = strtol(s, &end, 10);
    if (*end) return false;
    *out = (int)v;
    return true;
}

/* ---- events ----------------------------------------------------------------- */

bool lora_parse_evt(const char *line, lora_evt_t *e)
{
    memset(e, 0, sizeof(*e));
    if (!starts(line, "+EVT:")) return false;
    const char *r = line + 5;
    snprintf(e->text, sizeof(e->text), "%s", r);

    if (strcmp(r, "JOINED") == 0) {
        e->type = LORA_EVT_JOINED;
    } else if (starts(r, "JOIN_FAILED") || starts(r, "JOIN FAILED")) {
        e->type = LORA_EVT_JOIN_FAILED;
    } else if (strcmp(r, "SEND_CONFIRMED_OK") == 0 || strcmp(r, "SEND CONFIRMED OK") == 0) {
        e->type = LORA_EVT_SEND_OK;
    } else if (strcmp(r, "SEND_CONFIRMED_FAILED") == 0 || strcmp(r, "SEND CONFIRMED FAILED") == 0) {
        e->type = LORA_EVT_SEND_FAILED;
    } else if (strcmp(r, "TXP2P DONE") == 0 || strcmp(r, "TX_DONE") == 0) {
        e->type = LORA_EVT_TX_DONE;
    } else if (starts(r, "RXP2P RECEIVE TIMEOUT")) {
        e->type = LORA_EVT_RX_TIMEOUT;
    } else if (starts(r, "RXP2P:")) {
        char tmp[LINE_MAX_];
        snprintf(tmp, sizeof(tmp), "%s", r + 6);
        char *p = tmp;
        int rssi, snr;
        char *a = field(&p), *b = field(&p);
        if (!parse_int(a, &rssi) || !parse_int(b, &snr) || !p) {
            e->type = LORA_EVT_OTHER;
            return true;
        }
        int n = lora_hex_decode(p, e->data, sizeof(e->data));
        if (n < 0) {
            e->type = LORA_EVT_OTHER;
            return true;
        }
        e->type = LORA_EVT_RX_P2P;
        e->rssi = rssi;
        e->snr  = snr;
        e->len  = (size_t)n;
    } else if (starts(r, "RX_1:") || starts(r, "RX_2:") || starts(r, "RX_C:") || starts(r, "RX_B:")) {
        char tmp[LINE_MAX_];
        snprintf(tmp, sizeof(tmp), "%s", r + 5);
        char *p = tmp;
        int rssi, snr, port;
        char *a = field(&p), *b = field(&p), *cast = field(&p), *pt = field(&p);
        (void)cast;
        if (!parse_int(a, &rssi) || !parse_int(b, &snr) || !parse_int(pt, &port)) {
            e->type = LORA_EVT_OTHER;
            return true;
        }
        int n = p ? lora_hex_decode(p, e->data, sizeof(e->data)) : 0;
        if (n < 0) {
            e->type = LORA_EVT_OTHER;
            return true;
        }
        e->type = LORA_EVT_RX_WAN;
        e->rssi = rssi;
        e->snr  = snr;
        e->port = port;
        e->len  = (size_t)n;
    } else {
        e->type = LORA_EVT_OTHER;
    }
    return true;
}

/* ---- link --------------------------------------------------------------------- */

lora_link_t *lora_link_new(const lora_io_t *io, lora_evt_cb on_evt, void *user)
{
    lora_link_t *l = calloc(1, sizeof(*l));
    if (!l) return NULL;
    l->io     = *io;
    l->on_evt = on_evt;
    l->user   = user;
    return l;
}

static void complete(lora_link_t *l, lora_rc_t rc)
{
    if (!l->active) return;
    job_t j     = l->cur;
    l->active   = false;
    l->reply[l->reply_len] = 0;
    /* drop the trailing newline of the reply */
    if (l->reply_len && l->reply[l->reply_len - 1] == '\n') l->reply[--l->reply_len] = 0;
    if (j.done) j.done(rc, l->reply, j.tag, j.user);
}

void lora_link_flush(lora_link_t *l)
{
    if (l->active) complete(l, LORA_ERR_DROPPED);
    while (l->q_count > 0) {
        job_t j = l->queue[l->q_head];
        l->q_head = (l->q_head + 1) % QUEUE_N;
        l->q_count--;
        if (j.done) j.done(LORA_ERR_DROPPED, "", j.tag, j.user);
    }
}

void lora_link_free(lora_link_t *l)
{
    if (!l) return;
    /* no callbacks from here: the owner is going away */
    free(l);
}

bool lora_link_send(lora_link_t *l, const char *cmd, uint32_t timeout_ms, uint8_t flags, lora_done_cb done, int tag,
                    void *user)
{
    if (l->q_count >= QUEUE_N || strlen(cmd) >= CMD_MAX) return false;
    job_t *j = &l->queue[(l->q_head + l->q_count) % QUEUE_N];
    snprintf(j->cmd, sizeof(j->cmd), "%s", cmd);
    j->timeout_ms = timeout_ms;
    j->flags      = flags;
    j->done       = done;
    j->tag        = tag;
    j->user       = user;
    l->q_count++;
    return true;
}

bool lora_link_busy(const lora_link_t *l) { return l->active || l->q_count > 0; }
uint32_t lora_link_rx_bytes(const lora_link_t *l) { return l->rx_total; }

static bool is_banner(const char *s)
{
    return starts(s, "RAKwireless") || starts(s, "Version:") || starts(s, "-----") || starts(s, "Current Work Mode");
}

static void handle_line(lora_link_t *l, char *raw)
{
    char *s = trim(raw);
    if (!*s) return;

    lora_evt_t e;
    if (lora_parse_evt(s, &e)) {
        if (l->on_evt) l->on_evt(&e, l->user);
        return;
    }

    if (is_banner(s)) {
        l->saw_output = true;
        if (starts(s, "Current Work Mode")) {
            if (l->active && (l->cur.flags & LORA_F_REBOOT)) {
                complete(l, LORA_OK);
            }
            lora_evt_t b;
            memset(&b, 0, sizeof(b));
            b.type = LORA_EVT_BOOT;
            snprintf(b.text, sizeof(b.text), "%s", s);
            if (l->on_evt) l->on_evt(&b, l->user);
        }
        return;
    }

    if (!l->active) return; /* stray text */
    l->saw_output = true;

    if (strcmp(s, l->cur.cmd) == 0) return; /* echo */

    if (strcmp(s, "OK") == 0) {
        if (l->cur.flags & LORA_F_REBOOT) return; /* wait for the banner */
        complete(l, LORA_OK);
    } else if (starts(s, "AT_")) {
        size_t n = strlen(s);
        if (l->reply_len + n + 2 < REPLY_MAX) {
            memcpy(l->reply + l->reply_len, s, n);
            l->reply_len += n;
            l->reply[l->reply_len] = 0;
        }
        complete(l, LORA_ERR_AT);
    } else {
        size_t n = strlen(s);
        if (l->reply_len + n + 2 < REPLY_MAX) {
            memcpy(l->reply + l->reply_len, s, n);
            l->reply_len += n;
            l->reply[l->reply_len++] = '\n';
            l->reply[l->reply_len]   = 0;
        }
    }
}

void lora_link_tick(lora_link_t *l, uint32_t now_ms)
{
    /* Start the next command. */
    if (!l->active && l->q_count > 0) {
        l->cur     = l->queue[l->q_head];
        l->q_head  = (l->q_head + 1) % QUEUE_N;
        l->q_count--;
        l->active      = true;
        l->reply_len   = 0;
        l->reply[0]    = 0;
        l->saw_output  = false;
        l->sent_at     = now_ms;
        char out[CMD_MAX + 4];
        int n = snprintf(out, sizeof(out), "%s\r\n", l->cur.cmd);
        if (l->io.write((const uint8_t *)out, (size_t)n, l->io.ctx) < 0) {
            complete(l, LORA_ERR_IO);
        }
    }

    /* Read and parse. */
    uint8_t buf[128];
    for (int guard = 0; guard < 16; guard++) {
        int n = l->io.read(buf, sizeof(buf), l->io.ctx);
        if (n <= 0) break;
        l->rx_total += (uint32_t)n;
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\n' || c == '\r') {
                if (l->line_len) {
                    l->line[l->line_len] = 0;
                    l->line_len          = 0;
                    handle_line(l, l->line);
                }
            } else if (l->line_len < LINE_MAX_ - 1) {
                l->line[l->line_len++] = c;
            } else {
                l->line_len = 0; /* overlong garbage: drop it */
            }
        }
    }

    /* Timeout. */
    if (l->active && (uint32_t)(now_ms - l->sent_at) > l->cur.timeout_ms) {
        if ((l->cur.flags & LORA_F_REBOOT) && l->saw_output) {
            complete(l, LORA_OK);
        } else {
            complete(l, LORA_ERR_TIMEOUT);
        }
    }
}

bool lora_reply_value(const char *reply, const char *name, char *out, size_t n)
{
    char key[40];
    snprintf(key, sizeof(key), "AT+%s=", name);
    const char *p = reply;
    bool first = true;
    while (*p) {
        const char *e = strchr(p, '\n');
        size_t len    = e ? (size_t)(e - p) : strlen(p);
        if (strncmp(p, key, strlen(key)) == 0) {
            size_t vl = len - strlen(key);
            if (vl >= n) vl = n - 1;
            memcpy(out, p + strlen(key), vl);
            out[vl] = 0;
            return true;
        }
        if (first && !starts(p, "AT+")) {
            size_t vl = len < n - 1 ? len : n - 1;
            memcpy(out, p, vl);
            out[vl] = 0;
            return true;
        }
        first = false;
        if (!e) break;
        p = e + 1;
    }
    return false;
}

/* ---- airtime and duty cycle ------------------------------------------------- */

uint32_t lora_airtime_ms(unsigned payload_len, unsigned sf, unsigned bw_khz, unsigned cr, unsigned preamble)
{
    if (sf < 5 || sf > 12 || bw_khz == 0 || cr < 1 || cr > 4) return 0;
    double bw   = bw_khz * 1000.0;
    double tsym = (double)(1u << sf) / bw; /* seconds */
    int de      = tsym > 0.016 ? 1 : 0;    /* low data rate optimisation */
    int h       = 0;                       /* explicit header */
    double num  = 8.0 * payload_len - 4.0 * sf + 28.0 + 16.0 - 20.0 * h;
    double den  = 4.0 * (sf - 2 * de);
    double pl   = ceil(num / den) * (cr + 4);
    if (pl < 0) pl = 0;
    double total = (preamble + 4.25) * tsym + (8.0 + pl) * tsym;
    return (uint32_t)ceil(total * 1000.0);
}

void lora_duty_init(lora_duty_t *d, uint32_t limit_ms)
{
    memset(d, 0, sizeof(*d));
    d->limit_ms = limit_ms;
}

static void duty_advance(lora_duty_t *d, uint32_t now_ms)
{
    uint32_t m = now_ms / 60000u;
    if (!d->started) {
        d->started = true;
        d->minute  = m;
        return;
    }
    if (m == d->minute) return;
    uint32_t gap = m - d->minute;
    if (gap >= 60) {
        memset(d->used_ms, 0, sizeof(d->used_ms));
    } else {
        for (uint32_t i = 1; i <= gap; i++) d->used_ms[(d->minute + i) % 60] = 0;
    }
    d->minute = m;
}

uint32_t lora_duty_used(lora_duty_t *d, uint32_t now_ms)
{
    duty_advance(d, now_ms);
    uint32_t sum = 0;
    for (int i = 0; i < 60; i++) sum += d->used_ms[i];
    return sum;
}

bool lora_duty_allow(lora_duty_t *d, uint32_t now_ms, uint32_t airtime_ms)
{
    return lora_duty_used(d, now_ms) + airtime_ms <= d->limit_ms;
}

void lora_duty_add(lora_duty_t *d, uint32_t now_ms, uint32_t airtime_ms)
{
    duty_advance(d, now_ms);
    d->used_ms[d->minute % 60] += airtime_ms;
}
