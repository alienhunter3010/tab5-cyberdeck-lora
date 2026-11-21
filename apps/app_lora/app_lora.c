/*
 * LORA: the M5Stack Module LoRaWAN-EU868 on the Tab5's M5-Bus.
 *
 * The module is a RAK3172 running RAKwireless' RUI3 AT firmware, driven over
 * a UART with the non-blocking link in components/deck_lora. Three tabs:
 *   LINK  find the module (it cycles through the four UART/swap choices until
 *         one answers), firmware, work mode, reset.
 *   P2P   raw LoRa between two modules: a chat log and a text field.
 *   WAN   LoRaWAN OTAA: keys, join, uplink with an optional confirm, downlinks.
 * P2P transmissions are held to a one hour airtime budget (EU868 duty cycle).
 *
 * Meshtastic needs different firmware on the module and comes later.
 */
#include "app_lora.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "deck_attr.h"
#include "deck_hal.h"
#include "deck_icons.h"
#include "deck_shell.h"
#include "deck_theme.h"
#include "deck_widgets.h"
#include "lora_at.h"

#define TABS     3
#define TAB_LINK 0
#define TAB_P2P  1
#define TAB_WAN  2

#define BAUD   115200
#define LOG_N  40
#define TXT_MAX 120

#define HID_P 0x13

/* ---- Radio presets (EU868) --------------------------------------------------- */

static const struct {
    uint32_t hz;
    uint32_t duty_ms; /* airtime allowed per hour */
    const char *label;
} FREQS[] = {
    {868100000, 36000, "868.100 MHz  1%"},
    {868300000, 36000, "868.300 MHz  1%"},
    {868500000, 36000, "868.500 MHz  1%"},
    {869525000, 360000, "869.525 MHz  10%"},
};
#define N_FREQ ((int)(sizeof(FREQS) / sizeof(FREQS[0])))
static const int BWS[] = {125, 250, 500};

enum { P_FREQ, P_SF, P_BW, P_CR, P_PWR, P_PORT };

/* RUI3 answers to remember which question a reply belongs to. */
enum { T_PROBE = 1, T_VER, T_NWM, T_NWM_SET, T_IGN, T_P2P, T_RX, T_PSEND, T_CFG, T_JOIN, T_NJS, T_DEVEUI, T_SEND };

typedef enum { PH_PROBING, PH_ONLINE, PH_FAIL } phase_t;

/* ---- State kept while the app is closed (chat history, duty budget, settings) --- */

typedef struct {
    int kind; /* 0 RX, 1 TX, 2 info, 3 error */
    int rssi, snr;
    char text[TXT_MAX + 72];
} logrow_t;

typedef struct {
    logrow_t row[LOG_N];
    int n;
} log_t;

static log_t s_p2p_log DECK_PSRAM_BSS, s_wan_log DECK_PSRAM_BSS;
static lora_duty_t s_duty;
static bool s_duty_ready;

static struct {
    int tx, rx; /* the Tab5's TX and RX GPIOs that reached the module last time */
    int freq, sf, bw, cr, pwr, port, cfm;
    char deveui[17], appeui[17], appkey[33];
} s_cfg;

static void log_add(log_t *l, int kind, int rssi, int snr, const char *text)
{
    if (l->n == LOG_N) {
        memmove(&l->row[0], &l->row[1], sizeof(l->row[0]) * (LOG_N - 1));
        l->n--;
    }
    logrow_t *r = &l->row[l->n++];
    r->kind     = kind;
    r->rssi     = rssi;
    r->snr      = snr;
    snprintf(r->text, sizeof(r->text), "%s", text);
}

static void cfg_load(void)
{
    s_cfg.tx   = hal_cfg_get_i32("lr_tx", 6);
    s_cfg.rx   = hal_cfg_get_i32("lr_rx", 7);
    s_cfg.freq = hal_cfg_get_i32("lr_freq", 0);
    s_cfg.sf   = hal_cfg_get_i32("lr_sf", 7);
    s_cfg.bw   = hal_cfg_get_i32("lr_bw", 0);
    s_cfg.cr   = hal_cfg_get_i32("lr_cr", 0);
    s_cfg.pwr  = hal_cfg_get_i32("lr_pwr", 14);
    s_cfg.port = hal_cfg_get_i32("lr_port", 2);
    s_cfg.cfm  = hal_cfg_get_i32("lr_cfm", 1) ? 1 : 0;
    if (s_cfg.freq < 0 || s_cfg.freq >= N_FREQ) s_cfg.freq = 0;
    if (s_cfg.sf < 7 || s_cfg.sf > 12) s_cfg.sf = 7;
    if (s_cfg.bw < 0 || s_cfg.bw > 2) s_cfg.bw = 0;
    if (s_cfg.cr < 0 || s_cfg.cr > 3) s_cfg.cr = 0;
    if (s_cfg.pwr < 2 || s_cfg.pwr > 14) s_cfg.pwr = 14; /* 14 dBm: the limit of the 868.0-868.6 sub-band */
    if (s_cfg.port < 1 || s_cfg.port > 223) s_cfg.port = 2;
    s_cfg.deveui[0] = s_cfg.appeui[0] = s_cfg.appkey[0] = 0;
    hal_cfg_get_str("lr_deveui", s_cfg.deveui, sizeof(s_cfg.deveui));
    hal_cfg_get_str("lr_appeui", s_cfg.appeui, sizeof(s_cfg.appeui));
    hal_cfg_get_str("lr_appkey", s_cfg.appkey, sizeof(s_cfg.appkey));
}

static void cfg_save_radio(void)
{
    hal_cfg_set_i32("lr_freq", s_cfg.freq);
    hal_cfg_set_i32("lr_sf", s_cfg.sf);
    hal_cfg_set_i32("lr_bw", s_cfg.bw);
    hal_cfg_set_i32("lr_cr", s_cfg.cr);
    hal_cfg_set_i32("lr_pwr", s_cfg.pwr);
    hal_cfg_set_i32("lr_port", s_cfg.port);
    hal_cfg_set_i32("lr_cfm", s_cfg.cfm);
}

/* ---- Which M5-Bus pins carry the module's UART --------------------------------
 * The module's DIP switches put its RX on bus pin 2, 13, 15 or 26 and its TX on
 * pin 14, 16, 21, 23 or 24; the doc does not say whether those labels are the
 * module's or the host's side, so both readings are tried. Pairs are Tab5 GPIOs:
 * pin 2 = G16, 13 = G38, 15 = G7, 26 = G51; 14 = G37, 16 = G6, 21 = G2, 23 = G47, 24 = G35.
 * The first four are the usual suspects and make up the quick scan. */
#define QUICK_N 4
#define N_PAIRS 40

typedef struct {
    int8_t tx, rx;
} pair_t;

static pair_t s_pairs[N_PAIRS];

static bool pair_has(int n, int tx, int rx)
{
    for (int i = 0; i < n; i++)
        if (s_pairs[i].tx == tx && s_pairs[i].rx == rx) return true;
    return false;
}

static void pairs_init(void)
{
    static bool done;
    if (done) return;
    done = true;
    static const int8_t SET_A[] = {7, 38, 16, 51};        /* bus pins 15, 13, 2, 26 */
    static const int8_t SET_B[] = {6, 37, 2, 47, 35};     /* bus pins 16, 14, 21, 23, 24 */
    int n = 0;
    s_pairs[n++] = (pair_t){6, 7};
    s_pairs[n++] = (pair_t){7, 6};
    s_pairs[n++] = (pair_t){37, 38};
    s_pairs[n++] = (pair_t){38, 37};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 5; j++)
            if (!pair_has(n, SET_A[i], SET_B[j])) s_pairs[n++] = (pair_t){SET_A[i], SET_B[j]}; /* module RX on A, TX on B */
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 5; j++)
            if (!pair_has(n, SET_B[j], SET_A[i])) s_pairs[n++] = (pair_t){SET_B[j], SET_A[i]}; /* the other reading */
}

static int pair_index(int tx, int rx)
{
    for (int i = 0; i < N_PAIRS; i++)
        if (s_pairs[i].tx == tx && s_pairs[i].rx == rx) return i;
    return 0;
}

/* ---- UI state -------------------------------------------------------------------- */

static struct {
    bool alive;
    int tab;
    lv_obj_t *tab_btn[TABS], *page[TABS], *status;
    lv_obj_t *osk;
    lv_timer_t *timer;

    /* link and bring-up */
    lora_link_t *link;
    phase_t phase;
    int pair;       /* index into s_pairs, currently open */
    int scan_n;     /* pairs in this scan: QUICK_N or N_PAIRS */
    int pairs_left;
    int attempts;   /* tries per pair */
    int attempt;
    char ver[40];
    int mode;      /* 0 P2P, 1 LoRaWAN, -1 unknown */
    bool joined;
    bool expect_reboot;
    char status_text[80];

    /* P2P */
    bool await_tx;
    bool tx_done;
    uint32_t await_tx_at;
    uint32_t last_tx_airtime;

    /* widgets */
    lv_obj_t *v_state, *v_fw, *v_mode, *v_pins, *v_bytes;
    lv_obj_t *p2p_box, *p2p_ta, *p2p_info;
    lv_obj_t *p_val[6];
    lv_obj_t *wan_box, *wan_ta, *ta_deveui, *ta_appeui, *ta_appkey, *wan_state, *cfm_btn_lbl;
    int cfg_pending, cfg_failed;
} s_ui;

static void render_link(void);
static void render_p2p(void);
static void render_wan(void);

static void set_status(const char *s)
{
    snprintf(s_ui.status_text, sizeof(s_ui.status_text), "%s", s);
    if (s_ui.status) lv_label_set_text(s_ui.status, s_ui.status_text);
}

/* ---- Sending commands ------------------------------------------------------------ */

static void on_done(lora_rc_t rc, const char *reply, int tag, void *user);

static bool cmd(const char *c, uint32_t timeout_ms, uint8_t flags, int tag)
{
    if (!s_ui.link) return false;
    if (!lora_link_send(s_ui.link, c, timeout_ms, flags, on_done, tag, NULL)) {
        set_status("COMMAND QUEUE FULL");
        return false;
    }
    return true;
}

static void sanitize(const uint8_t *in, size_t n, char *out, size_t out_n)
{
    size_t o = 0;
    for (size_t i = 0; i < n && o + 1 < out_n; i++) out[o++] = (in[i] < 0x20 || in[i] == 0x7F) ? '.' : (char)in[i];
    out[o] = 0;
}

static void open_pair(void)
{
    hal_mbus_uart_open(s_pairs[s_ui.pair].tx, s_pairs[s_ui.pair].rx, BAUD);
    hal_mbus_uart_flush();
}

static void probe_send(void)
{
    s_ui.phase = PH_PROBING;
    cmd("AT", 700, 0, T_PROBE);
    render_link();
}

/* Ask "AT" on the saved pins first, then walk the other pairs: the quick four, or with `deep` all of them. */
static void probe_begin(bool deep)
{
    if (!s_ui.link) return;
    lora_link_flush(s_ui.link);
    s_ui.scan_n     = deep ? N_PAIRS : QUICK_N;
    s_ui.pairs_left = s_ui.scan_n;
    s_ui.attempts   = deep ? 1 : 2;
    s_ui.attempt    = 0;
    s_ui.mode       = -1;
    s_ui.ver[0]     = 0;
    s_ui.joined     = false;
    s_ui.pair       = pair_index(s_cfg.tx, s_cfg.rx);
    if (s_ui.pair >= s_ui.scan_n) s_ui.pair = 0;
    open_pair();
    set_status(deep ? "DEEP SCAN: TRYING EVERY PIN PAIR..." : "LOOKING FOR THE MODULE...");
    probe_send();
}

static void p2p_apply(void)
{
    char c[96];
    s_duty.limit_ms = FREQS[s_cfg.freq].duty_ms;
    cmd("AT+PRECV=0", 800, 0, T_IGN);
    snprintf(c, sizeof(c), "AT+P2P=%u:%d:%d:%d:8:%d", (unsigned)FREQS[s_cfg.freq].hz, s_cfg.sf, BWS[s_cfg.bw], s_cfg.cr,
             s_cfg.pwr);
    cmd(c, 1000, 0, T_P2P);
    cmd("AT+PRECV=65534", 1000, 0, T_RX);
}

static void after_mode_known(void)
{
    if (s_ui.mode == 0) {
        p2p_apply();
    } else if (s_ui.mode == 1) {
        cmd("AT+NJS=?", 800, 0, T_NJS);
        cmd("AT+DEVEUI=?", 800, 0, T_DEVEUI);
    }
}

static void on_done(lora_rc_t rc, const char *reply, int tag, void *user)
{
    (void)user;
    if (!s_ui.alive || rc == LORA_ERR_DROPPED) return; /* flushed on purpose */
    char val[96];

    switch (tag) {
    case T_PROBE:
        if (rc == LORA_OK) {
            s_ui.phase = PH_ONLINE;
            s_cfg.tx = s_pairs[s_ui.pair].tx;
            s_cfg.rx = s_pairs[s_ui.pair].rx;
            hal_cfg_set_i32("lr_tx", s_cfg.tx);
            hal_cfg_set_i32("lr_rx", s_cfg.rx);
            set_status("MODULE ONLINE");
            cmd("AT+VER=?", 800, 0, T_VER);
            cmd("AT+NWM=?", 800, 0, T_NWM);
        } else if (++s_ui.attempt < s_ui.attempts) {
            probe_send();
        } else if (--s_ui.pairs_left > 0) {
            s_ui.attempt = 0;
            s_ui.pair    = (s_ui.pair + 1) % s_ui.scan_n;
            open_pair();
            if (s_ui.scan_n == N_PAIRS) {
                char m[48];
                snprintf(m, sizeof(m), "DEEP SCAN %d/%d", N_PAIRS - s_ui.pairs_left + 1, N_PAIRS);
                set_status(m);
            }
            probe_send();
        } else {
            s_ui.phase = PH_FAIL;
            set_status(s_ui.scan_n == N_PAIRS ? "NO ANSWER ON ANY PAIR: CHECK THE DIP SWITCHES AND THE MODULE POWER"
                                              : "NO ANSWER: TRY DEEP SCAN, CHECK THE DIP SWITCHES");
        }
        break;
    case T_VER:
        if (rc == LORA_OK && lora_reply_value(reply, "VER", val, sizeof(val))) snprintf(s_ui.ver, sizeof(s_ui.ver), "%s", val);
        break;
    case T_NWM:
        s_ui.expect_reboot = false;
        if (rc == LORA_OK && lora_reply_value(reply, "NWM", val, sizeof(val))) {
            s_ui.mode = atoi(val);
            after_mode_known();
        }
        break;
    case T_NWM_SET:
        if (rc == LORA_OK) {
            cmd("AT+NWM=?", 800, 0, T_NWM); /* clears expect_reboot when it answers */
        } else {
            s_ui.expect_reboot = false;
            set_status(reply[0] ? reply : "MODE CHANGE FAILED");
        }
        break;
    case T_P2P:
        if (rc != LORA_OK) {
            log_add(&s_p2p_log, 3, 0, 0, reply[0] ? reply : "AT+P2P: NO ANSWER");
            set_status("P2P SETUP REFUSED");
        } else {
            set_status("P2P LISTENING");
        }
        break;
    case T_RX:
        if (rc != LORA_OK) log_add(&s_p2p_log, 3, 0, 0, reply[0] ? reply : "AT+PRECV: NO ANSWER");
        break;
    case T_PSEND:
        if (rc == LORA_OK) {
            lora_duty_add(&s_duty, lv_tick_get(), s_ui.last_tx_airtime);
            s_ui.await_tx    = true;
            s_ui.tx_done     = false;
            s_ui.await_tx_at = lv_tick_get();
        } else {
            log_add(&s_p2p_log, 3, 0, 0, reply[0] ? reply : "SEND FAILED: NO ANSWER");
            cmd("AT+PRECV=65534", 1000, 0, T_RX);
        }
        break;
    case T_NJS:
        if (rc == LORA_OK && lora_reply_value(reply, "NJS", val, sizeof(val))) s_ui.joined = atoi(val) == 1;
        break;
    case T_DEVEUI:
        if (rc == LORA_OK && lora_reply_value(reply, "DEVEUI", val, sizeof(val)) && strlen(val) == 16 && !s_cfg.deveui[0]) {
            snprintf(s_cfg.deveui, sizeof(s_cfg.deveui), "%s", val);
            hal_cfg_set_str("lr_deveui", s_cfg.deveui);
        }
        break;
    case T_CFG:
        if (rc != LORA_OK) {
            s_ui.cfg_failed++;
            log_add(&s_wan_log, 3, 0, 0, reply[0] ? reply : "CONFIG: NO ANSWER");
        }
        if (--s_ui.cfg_pending == 0) set_status(s_ui.cfg_failed ? "CONFIG: SOME SETTINGS WERE REFUSED" : "KEYS AND BAND SET");
        break;
    case T_JOIN:
        if (rc == LORA_OK) {
            set_status("JOINING...");
        } else {
            log_add(&s_wan_log, 3, 0, 0, reply[0] ? reply : "JOIN: NO ANSWER");
            set_status("JOIN REFUSED");
        }
        break;
    case T_SEND:
        if (rc != LORA_OK) {
            log_add(&s_wan_log, 3, 0, 0, reply[0] ? reply : "SEND: NO ANSWER");
            set_status("UPLINK REFUSED");
        } else {
            set_status(s_cfg.cfm ? "UPLINK SENT, WAITING FOR THE CONFIRM" : "UPLINK SENT");
        }
        break;
    default:
        break;
    }
    render_link();
    render_p2p();
    render_wan();
}

static void on_evt(const lora_evt_t *e, void *user)
{
    (void)user;
    if (!s_ui.alive) return;
    char txt[TXT_MAX + 24];
    switch (e->type) {
    case LORA_EVT_RX_P2P:
        sanitize(e->data, e->len, txt, sizeof(txt));
        log_add(&s_p2p_log, 0, e->rssi, e->snr, txt);
        break;
    case LORA_EVT_TX_DONE:
        s_ui.tx_done = true;
        break;
    case LORA_EVT_JOINED:
        s_ui.joined = true;
        log_add(&s_wan_log, 2, 0, 0, "JOINED");
        set_status("JOINED");
        break;
    case LORA_EVT_JOIN_FAILED:
        s_ui.joined = false;
        snprintf(txt, sizeof(txt), "JOIN FAILED  %s", e->text);
        log_add(&s_wan_log, 3, 0, 0, txt);
        set_status("JOIN FAILED");
        break;
    case LORA_EVT_SEND_OK:
        log_add(&s_wan_log, 2, 0, 0, "UPLINK CONFIRMED BY THE NETWORK");
        set_status("UPLINK CONFIRMED");
        break;
    case LORA_EVT_SEND_FAILED:
        log_add(&s_wan_log, 3, 0, 0, "UPLINK NOT CONFIRMED");
        set_status("UPLINK NOT CONFIRMED");
        break;
    case LORA_EVT_RX_WAN:
        sanitize(e->data, e->len, txt, sizeof(txt) - 16);
        {
            char line[TXT_MAX + 40];
            snprintf(line, sizeof(line), "PORT %d  %s", e->port, txt);
            log_add(&s_wan_log, 0, e->rssi, e->snr, line);
        }
        break;
    case LORA_EVT_BOOT:
        if (!s_ui.expect_reboot) {
            /* the module restarted on its own: it forgot the radio setup */
            log_add(&s_p2p_log, 2, 0, 0, "MODULE RESTARTED");
            cmd("AT+NWM=?", 800, 0, T_NWM);
        }
        break;
    default:
        break;
    }
    render_p2p();
    render_wan();
    render_link();
}

/* ---- Rendering ---------------------------------------------------------------------- */

static void render_log(lv_obj_t *box, const log_t *l)
{
    if (!box) return;
    lv_obj_clean(box);
    if (l->n == 0) {
        lv_obj_t *e = deck_label(box, g_font.mono_m, g_pal.dim, "Nothing yet.");
        lv_obj_set_width(e, LV_PCT(100));
    }
    for (int i = 0; i < l->n; i++) {
        const logrow_t *r = &l->row[i];
        char line[TXT_MAX + 120];
        lv_color_t col = g_pal.text;
        if (r->kind == 0) {
            snprintf(line, sizeof(line), "RX  %d dBm %+d dB   %s", r->rssi, r->snr, r->text);
            col = g_pal.accent;
        } else if (r->kind == 1) {
            snprintf(line, sizeof(line), "TX  %s", r->text);
            col = g_pal.text;
        } else if (r->kind == 2) {
            snprintf(line, sizeof(line), "--  %s", r->text);
            col = g_pal.accent2;
        } else {
            snprintf(line, sizeof(line), "!!  %s", r->text);
            col = g_pal.danger;
        }
        lv_obj_t *lab = deck_label(box, g_font.mono_m, col, line);
        lv_obj_set_width(lab, LV_PCT(100));
        lv_label_set_long_mode(lab, LV_LABEL_LONG_MODE_WRAP);
    }
    lv_obj_update_layout(box);
    lv_obj_scroll_to_y(box, LV_COORD_MAX, LV_ANIM_OFF);
}

static void render_link(void)
{
    if (!s_ui.v_state) return;
    const char *st = s_ui.phase == PH_ONLINE ? "ONLINE" : s_ui.phase == PH_PROBING ? "LOOKING..." : "NO ANSWER";
    lv_color_t sc  = s_ui.phase == PH_ONLINE ? g_pal.ok : s_ui.phase == PH_PROBING ? g_pal.warn : g_pal.danger;
    lv_label_set_text(s_ui.v_state, st);
    lv_obj_set_style_text_color(s_ui.v_state, sc, 0);
    lv_label_set_text(s_ui.v_fw, s_ui.ver[0] ? s_ui.ver : "-");
    lv_label_set_text(s_ui.v_mode, s_ui.mode == 0 ? "LORA P2P" : s_ui.mode == 1 ? (s_ui.joined ? "LORAWAN  JOINED" : "LORAWAN  NOT JOINED") : "-");
    lv_label_set_text(s_ui.v_pins, hal_mbus_pins_describe(s_pairs[s_ui.pair].tx, s_pairs[s_ui.pair].rx));
    if (s_ui.link) lv_label_set_text_fmt(s_ui.v_bytes, "%u BYTES HEARD", (unsigned)lora_link_rx_bytes(s_ui.link));
}

static void render_p2p_info(void)
{
    if (!s_ui.p2p_box) return;
    lv_label_set_text(s_ui.p_val[P_FREQ], FREQS[s_cfg.freq].label);
    lv_label_set_text_fmt(s_ui.p_val[P_SF], "SF%d", s_cfg.sf);
    lv_label_set_text_fmt(s_ui.p_val[P_BW], "%d kHz", BWS[s_cfg.bw]);
    lv_label_set_text_fmt(s_ui.p_val[P_CR], "4/%d", s_cfg.cr + 5);
    lv_label_set_text_fmt(s_ui.p_val[P_PWR], "%d dBm", s_cfg.pwr);

    uint32_t toa  = lora_airtime_ms(24, (unsigned)s_cfg.sf, (unsigned)BWS[s_cfg.bw], (unsigned)s_cfg.cr + 1, 8);
    uint32_t used = lora_duty_used(&s_duty, lv_tick_get());
    lv_label_set_text_fmt(s_ui.p2p_info,
                          "24 BYTES = %u MS ON AIR\nBUDGET  %u.%u S OF %u S THIS HOUR\n(%s)",
                          (unsigned)toa, (unsigned)(used / 1000), (unsigned)(used % 1000 / 100),
                          (unsigned)(s_duty.limit_ms / 1000), FREQS[s_cfg.freq].duty_ms == 36000 ? "1% SUB-BAND" : "10% SUB-BAND");
}

static void render_p2p(void)
{
    if (!s_ui.p2p_box) return;
    render_log(s_ui.p2p_box, &s_p2p_log);
    render_p2p_info();
}

static void render_wan(void)
{
    if (!s_ui.wan_box) return;
    render_log(s_ui.wan_box, &s_wan_log);
    lv_label_set_text(s_ui.wan_state, s_ui.mode != 1 ? "MODULE IS NOT IN LORAWAN MODE (LINK TAB)" : s_ui.joined ? "JOINED" : "NOT JOINED");
    lv_obj_set_style_text_color(s_ui.wan_state, s_ui.joined ? g_pal.ok : g_pal.warn, 0);
    lv_label_set_text_fmt(s_ui.p_val[P_PORT], "PORT %d", s_cfg.port);
    lv_label_set_text(s_ui.cfm_btn_lbl, s_cfg.cfm ? "CONFIRMED" : "UNCONFIRMED");
}

/* ---- Actions --------------------------------------------------------------------------- */

static void do_p2p_send(void)
{
    const char *t = lv_textarea_get_text(s_ui.p2p_ta);
    if (!t[0]) return;
    if (s_ui.phase != PH_ONLINE) {
        set_status("NO MODULE");
        return;
    }
    if (s_ui.mode != 0) {
        set_status("SWITCH THE MODULE TO P2P ON THE LINK TAB");
        return;
    }
    if (s_ui.await_tx) {
        set_status("STILL TRANSMITTING");
        return;
    }
    size_t n = strlen(t);
    uint32_t toa = lora_airtime_ms((unsigned)n, (unsigned)s_cfg.sf, (unsigned)BWS[s_cfg.bw], (unsigned)s_cfg.cr + 1, 8);
    if (!lora_duty_allow(&s_duty, lv_tick_get(), toa)) {
        log_add(&s_p2p_log, 3, 0, 0, "DUTY CYCLE BUDGET USED UP: WAIT BEFORE SENDING AGAIN");
        set_status("DUTY CYCLE");
        render_p2p();
        return;
    }
    char hex[2 * TXT_MAX + 4], c[2 * TXT_MAX + 24];
    lora_hex_encode((const uint8_t *)t, n, hex, sizeof(hex));
    s_ui.last_tx_airtime = toa;
    cmd("AT+PRECV=0", 800, 0, T_IGN);
    snprintf(c, sizeof(c), "AT+PSEND=%s", hex);
    if (cmd(c, 2000, 0, T_PSEND)) {
        log_add(&s_p2p_log, 1, 0, 0, t);
        lv_textarea_set_text(s_ui.p2p_ta, "");
        set_status("SENDING...");
    }
    render_p2p();
}

static void do_wan_send(void)
{
    const char *t = lv_textarea_get_text(s_ui.wan_ta);
    if (!t[0]) return;
    if (s_ui.mode != 1) {
        set_status("SWITCH THE MODULE TO LORAWAN ON THE LINK TAB");
        return;
    }
    if (!s_ui.joined) {
        set_status("JOIN FIRST");
        return;
    }
    char hex[2 * TXT_MAX + 4], c[2 * TXT_MAX + 24];
    lora_hex_encode((const uint8_t *)t, strlen(t), hex, sizeof(hex));
    snprintf(c, sizeof(c), "AT+CFM=%d", s_cfg.cfm);
    cmd(c, 800, 0, T_IGN);
    snprintf(c, sizeof(c), "AT+SEND=%d:%s", s_cfg.port, hex);
    if (cmd(c, 3000, 0, T_SEND)) {
        char line[TXT_MAX + 24];
        snprintf(line, sizeof(line), "PORT %d  %s", s_cfg.port, t);
        log_add(&s_wan_log, 1, 0, 0, line);
        lv_textarea_set_text(s_ui.wan_ta, "");
    }
    render_wan();
}

static bool is_hex_len(const char *s, size_t n)
{
    if (strlen(s) != n) return false;
    for (size_t i = 0; i < n; i++)
        if (!isxdigit((unsigned char)s[i])) return false;
    return true;
}

static void do_wan_config(void)
{
    if (s_ui.mode != 1) {
        set_status("SWITCH THE MODULE TO LORAWAN ON THE LINK TAB");
        return;
    }
    const char *d = lv_textarea_get_text(s_ui.ta_deveui), *a = lv_textarea_get_text(s_ui.ta_appeui),
               *k = lv_textarea_get_text(s_ui.ta_appkey);
    if (!is_hex_len(d, 16) || !is_hex_len(a, 16) || !is_hex_len(k, 32)) {
        set_status("DEVEUI AND APPEUI ARE 16 HEX DIGITS, APPKEY IS 32");
        return;
    }
    snprintf(s_cfg.deveui, sizeof(s_cfg.deveui), "%s", d);
    snprintf(s_cfg.appeui, sizeof(s_cfg.appeui), "%s", a);
    snprintf(s_cfg.appkey, sizeof(s_cfg.appkey), "%s", k);
    hal_cfg_set_str("lr_deveui", s_cfg.deveui);
    hal_cfg_set_str("lr_appeui", s_cfg.appeui);
    hal_cfg_set_str("lr_appkey", s_cfg.appkey);

    char c[64];
    s_ui.cfg_pending = 6;
    s_ui.cfg_failed  = 0;
    cmd("AT+NJM=1", 800, 0, T_CFG);   /* OTAA */
    cmd("AT+BAND=4", 800, 0, T_CFG);  /* EU868 */
    cmd("AT+CLASS=A", 800, 0, T_CFG);
    snprintf(c, sizeof(c), "AT+DEVEUI=%s", s_cfg.deveui);
    cmd(c, 800, 0, T_CFG);
    snprintf(c, sizeof(c), "AT+APPEUI=%s", s_cfg.appeui);
    cmd(c, 800, 0, T_CFG);
    snprintf(c, sizeof(c), "AT+APPKEY=%s", s_cfg.appkey);
    cmd(c, 800, 0, T_CFG);
    set_status("SETTING KEYS...");
}

/* ---- Widget callbacks --------------------------------------------------------------------- */

static void btn_probe(lv_event_t *e) { (void)e; probe_begin(false); }
static void btn_deep(lv_event_t *e) { (void)e; probe_begin(true); }

static void btn_reset(lv_event_t *e)
{
    (void)e;
    hal_mbus_reset_pulse();
    set_status("RESET SENT (NEEDS THE DIP SWITCH ON RST PIN 8)");
    probe_begin(false);
}

static void btn_mode(lv_event_t *e)
{
    intptr_t m = (intptr_t)lv_event_get_user_data(e);
    if (s_ui.phase != PH_ONLINE) {
        set_status("NO MODULE");
        return;
    }
    if (s_ui.mode == (int)m) return;
    s_ui.expect_reboot = true;
    set_status("SWITCHING MODE: THE MODULE RESTARTS...");
    cmd("AT+PRECV=0", 800, 0, T_IGN);
    cmd(m ? "AT+NWM=1" : "AT+NWM=0", 6000, LORA_F_REBOOT, T_NWM_SET);
}

static void btn_p2p_send(lv_event_t *e) { (void)e; do_p2p_send(); }
static void btn_p2p_apply(lv_event_t *e) { (void)e; if (s_ui.phase == PH_ONLINE && s_ui.mode == 0) p2p_apply(); }
static void btn_wan_send(lv_event_t *e) { (void)e; do_wan_send(); }
static void btn_wan_config(lv_event_t *e) { (void)e; do_wan_config(); }

static void btn_join(lv_event_t *e)
{
    (void)e;
    if (s_ui.mode != 1) {
        set_status("SWITCH THE MODULE TO LORAWAN ON THE LINK TAB");
        return;
    }
    cmd("AT+JOIN=1:0:10:8", 1500, 0, T_JOIN);
}

static void btn_cfm(lv_event_t *e)
{
    (void)e;
    s_cfg.cfm ^= 1;
    cfg_save_radio();
    render_wan();
}

static void param_step(lv_event_t *e)
{
    intptr_t v = (intptr_t)lv_event_get_user_data(e);
    int id = (int)(v >> 8), dir = (int)(int8_t)(v & 0xFF);
    switch (id) {
    case P_FREQ: s_cfg.freq = (s_cfg.freq + dir + N_FREQ) % N_FREQ; break;
    case P_SF:   s_cfg.sf = s_cfg.sf + dir < 7 ? 7 : s_cfg.sf + dir > 12 ? 12 : s_cfg.sf + dir; break;
    case P_BW:   s_cfg.bw = (s_cfg.bw + dir + 3) % 3; break;
    case P_CR:   s_cfg.cr = (s_cfg.cr + dir + 4) % 4; break;
    case P_PWR:  s_cfg.pwr = s_cfg.pwr + dir < 2 ? 2 : s_cfg.pwr + dir > 14 ? 14 : s_cfg.pwr + dir; break;
    case P_PORT: s_cfg.port = s_cfg.port + dir < 1 ? 1 : s_cfg.port + dir > 223 ? 223 : s_cfg.port + dir; break;
    }
    cfg_save_radio();
    s_duty.limit_ms = FREQS[s_cfg.freq].duty_ms;
    render_p2p();
    render_wan();
    set_status(id == P_PORT ? "" : "CHANGED: APPLY TO PUT IT ON THE MODULE (BOTH SIDES MUST MATCH)");
}

static void ta_ready_p2p(lv_event_t *e) { (void)e; do_p2p_send(); }
static void ta_ready_wan(lv_event_t *e) { (void)e; do_wan_send(); }

static void ta_focus(lv_event_t *e)
{
    lv_obj_t *ta = lv_event_get_target(e);
    if (!s_ui.osk) return;
    if (lv_event_get_code(e) == LV_EVENT_FOCUSED) {
        lv_keyboard_set_textarea(s_ui.osk, ta);
        lv_obj_remove_flag(s_ui.osk, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_ui.osk, LV_OBJ_FLAG_HIDDEN);
    }
}

/* ---- Building the screens ------------------------------------------------------------------ */

static lv_obj_t *small_button(lv_obj_t *parent, const char *text, lv_event_cb_t cb, intptr_t ud)
{
    lv_obj_t *b = deck_button(parent, text, NULL);
    lv_obj_set_style_min_height(b, 46, 0);
    lv_obj_set_style_pad_ver(b, 8, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, (void *)ud);
    return b;
}

static lv_obj_t *panel(lv_obj_t *parent, int w)
{
    lv_obj_t *p = deck_panel(parent, DECK_CUT_TL | DECK_CUT_BR, 18);
    if (w > 0) lv_obj_set_size(p, w, LV_PCT(100));
    else {
        lv_obj_set_height(p, LV_PCT(100));
        lv_obj_set_flex_grow(p, 1);
    }
    lv_obj_set_flex_flow(p, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(p, 16, 0);
    lv_obj_set_style_pad_row(p, 8, 0);
    return p;
}

static lv_obj_t *kv(lv_obj_t *parent, const char *key)
{
    lv_obj_t *row = deck_box(parent);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    deck_label(row, g_font.mono_m, g_pal.dim, key);
    return deck_label(row, g_font.mono_m, g_pal.text, "-");
}

static lv_obj_t *text_field(lv_obj_t *parent, const char *placeholder, int max, int width)
{
    lv_obj_t *ta = lv_textarea_create(parent);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_placeholder_text(ta, placeholder);
    lv_textarea_set_max_length(ta, (uint32_t)max);
    if (width > 0) lv_obj_set_width(ta, width);
    lv_obj_set_style_text_font(ta, g_font.mono_m, 0);
    lv_obj_add_event_cb(ta, ta_focus, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(ta, ta_focus, LV_EVENT_DEFOCUSED, NULL);
    return ta;
}

static lv_obj_t *hex_field(lv_obj_t *parent, const char *label, const char *init, int digits)
{
    deck_label(parent, g_font.mono_s, g_pal.dim, label);
    lv_obj_t *ta = text_field(parent, "hex", digits, LV_PCT(100));
    lv_textarea_set_accepted_chars(ta, "0123456789abcdefABCDEF");
    lv_textarea_set_text(ta, init);
    return ta;
}

static void param_row(lv_obj_t *parent, int id, const char *name)
{
    lv_obj_t *row = deck_box(parent);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, 8, 0);
    lv_obj_t *n = deck_label(row, g_font.mono_s, g_pal.dim, name);
    lv_obj_set_width(n, 56);
    small_button(row, "<", param_step, (id << 8) | 0xFF);
    s_ui.p_val[id] = deck_label(row, g_font.mono_m, g_pal.text, "-");
    lv_obj_set_width(s_ui.p_val[id], id == P_FREQ ? 190 : 100);
    small_button(row, ">", param_step, (id << 8) | 1);
}

static void build_link(lv_obj_t *page)
{
    lv_obj_set_flex_flow(page, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(page, 20, 0);

    lv_obj_t *left = panel(page, 620);
    deck_section(left, "MODULE");
    s_ui.v_state = kv(left, "STATE");
    s_ui.v_fw    = kv(left, "FIRMWARE");
    s_ui.v_mode  = kv(left, "MODE");
    s_ui.v_pins  = kv(left, "SERIAL");
    s_ui.v_bytes = kv(left, "TRAFFIC");

    lv_obj_t *row = deck_box(left);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_column(row, 10, 0);
    lv_obj_set_style_pad_row(row, 10, 0);
    small_button(row, "FIND MODULE", btn_probe, 0);
    small_button(row, "DEEP SCAN", btn_deep, 0);
    small_button(row, "RESET", btn_reset, 0);
    deck_label(left, g_font.mono_s, g_pal.dim, "WORK MODE (THE MODULE RESTARTS)");
    lv_obj_t *mrow = deck_box(left);
    lv_obj_set_flex_flow(mrow, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(mrow, 10, 0);
    small_button(mrow, "LORA P2P", btn_mode, 0);
    small_button(mrow, "LORAWAN", btn_mode, 1);

    lv_obj_t *right = panel(page, 0);
    deck_section(right, "WIRING");
    lv_obj_t *t = deck_label(right, g_font.mono_m, g_pal.text,
                             "M5-Bus 5 V comes on with this module.\n"
                             "The DIP switches pick the bus pins:\n"
                             "  module RX: pin 2, 13, 15 or 26\n"
                             "  module TX: pin 14, 16, 21, 23, 24\n"
                             "  RST: pin 8 (22 is shared with audio)\n"
                             "FIND MODULE tries the usual pairs;\n"
                             "DEEP SCAN tries all 40 (about 30 s)\n"
                             "and the answer is remembered.\n"
                             "\n"
                             "EU868 is shared: P2P sends are held to\n"
                             "the duty cycle of the sub-band, 14 dBm.\n"
                             "Fit the antenna before any transmit.");
    lv_obj_set_width(t, LV_PCT(100));
    lv_obj_set_style_text_line_space(t, 4, 0);
}

static void build_p2p(lv_obj_t *page)
{
    lv_obj_set_flex_flow(page, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(page, 20, 0);

    lv_obj_t *left = panel(page, 0);
    deck_section(left, "LORA P2P");
    s_ui.p2p_box = deck_box(left);
    lv_obj_set_width(s_ui.p2p_box, LV_PCT(100));
    lv_obj_set_flex_grow(s_ui.p2p_box, 1);
    lv_obj_set_flex_flow(s_ui.p2p_box, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_ui.p2p_box, 6, 0);
    lv_obj_add_flag(s_ui.p2p_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(s_ui.p2p_box, LV_DIR_VER);

    lv_obj_t *row = deck_box(left);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 10, 0);
    s_ui.p2p_ta = text_field(row, "message", TXT_MAX, 580);
    lv_obj_add_event_cb(s_ui.p2p_ta, ta_ready_p2p, LV_EVENT_READY, NULL);
    small_button(row, "SEND", btn_p2p_send, 0);

    lv_obj_t *right = panel(page, 430);
    deck_section(right, "RADIO");
    param_row(right, P_FREQ, "FREQ");
    param_row(right, P_SF, "SF");
    param_row(right, P_BW, "BW");
    param_row(right, P_CR, "CR");
    param_row(right, P_PWR, "POWER");
    small_button(right, "APPLY", btn_p2p_apply, 0);
    s_ui.p2p_info = deck_label(right, g_font.mono_s, g_pal.dim, "");
    lv_obj_set_width(s_ui.p2p_info, LV_PCT(100));
    lv_obj_set_style_text_line_space(s_ui.p2p_info, 4, 0);
}

static void build_wan(lv_obj_t *page)
{
    lv_obj_set_flex_flow(page, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(page, 20, 0);

    lv_obj_t *left = panel(page, 520);
    deck_section(left, "OTAA KEYS");
    s_ui.ta_deveui = hex_field(left, "DEVEUI (module default is filled in)", s_cfg.deveui, 16);
    s_ui.ta_appeui = hex_field(left, "APPEUI / JOINEUI", s_cfg.appeui, 16);
    s_ui.ta_appkey = hex_field(left, "APPKEY", s_cfg.appkey, 32);
    lv_obj_t *row = deck_box(left);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 10, 0);
    small_button(row, "SET KEYS", btn_wan_config, 0);
    small_button(row, "JOIN", btn_join, 0);
    s_ui.wan_state = deck_label(left, g_font.mono_m, g_pal.warn, "");

    lv_obj_t *right = panel(page, 0);
    deck_section(right, "UPLINK / DOWNLINK");
    s_ui.wan_box = deck_box(right);
    lv_obj_set_width(s_ui.wan_box, LV_PCT(100));
    lv_obj_set_flex_grow(s_ui.wan_box, 1);
    lv_obj_set_flex_flow(s_ui.wan_box, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_ui.wan_box, 6, 0);
    lv_obj_add_flag(s_ui.wan_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(s_ui.wan_box, LV_DIR_VER);

    lv_obj_t *opt = deck_box(right);
    lv_obj_set_width(opt, LV_PCT(100));
    lv_obj_set_flex_flow(opt, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(opt, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(opt, 8, 0);
    small_button(opt, "<", param_step, (P_PORT << 8) | 0xFF);
    s_ui.p_val[P_PORT] = deck_label(opt, g_font.mono_m, g_pal.text, "PORT 2");
    lv_obj_set_width(s_ui.p_val[P_PORT], 120);
    small_button(opt, ">", param_step, (P_PORT << 8) | 1);
    lv_obj_t *cb = small_button(opt, "CONFIRMED", btn_cfm, 0);
    s_ui.cfm_btn_lbl = lv_obj_get_child(cb, 0);

    lv_obj_t *row2 = deck_box(right);
    lv_obj_set_width(row2, LV_PCT(100));
    lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row2, 10, 0);
    s_ui.wan_ta = text_field(row2, "payload (text)", TXT_MAX, 400);
    lv_obj_add_event_cb(s_ui.wan_ta, ta_ready_wan, LV_EVENT_READY, NULL);
    small_button(row2, "SEND", btn_wan_send, 0);
}

/* ---- Tabs and keys ------------------------------------------------------------------------------ */

static void set_tab(int tab)
{
    s_ui.tab = tab;
    for (int i = 0; i < TABS; i++) {
        if (i == tab) lv_obj_remove_flag(s_ui.page[i], LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_ui.page[i], LV_OBJ_FLAG_HIDDEN);
        deck_panel_set_outline(s_ui.tab_btn[i], i == tab ? g_pal.accent : g_pal.line);
    }
    lv_group_remove_all_objs(deck_input_group());
    lv_obj_t *page = s_ui.page[tab];
    /* every button and field on the page, in tree order */
    lv_obj_t *stack[64];
    int sp = 0;
    stack[sp++] = page;
    lv_obj_t *order[64];
    int on = 0;
    while (sp > 0 && on < 64) {
        lv_obj_t *o = stack[--sp];
        if (lv_obj_has_flag(o, LV_OBJ_FLAG_CLICK_FOCUSABLE) && o != page && lv_obj_has_flag(o, LV_OBJ_FLAG_CLICKABLE)) order[on++] = o;
        else if (lv_obj_check_type(o, &lv_textarea_class)) order[on++] = o;
        uint32_t n = lv_obj_get_child_count(o);
        for (int i = (int)n - 1; i >= 0 && sp < 63; i--) stack[sp++] = lv_obj_get_child(o, (uint32_t)i);
    }
    for (int i = 0; i < on; i++) lv_group_add_obj(deck_input_group(), order[i]);
    if (on) lv_group_focus_obj(order[0]);
    if (tab == TAB_P2P) render_p2p();
    if (tab == TAB_WAN) render_wan();
    if (tab == TAB_LINK) render_link();
    lv_label_set_text(s_ui.status, s_ui.status_text);
}

static void tab_clicked(lv_event_t *e) { set_tab((int)(intptr_t)lv_event_get_user_data(e)); }

static void tick(lv_timer_t *t)
{
    (void)t;
    if (!s_ui.alive) return;
    uint32_t now = lv_tick_get();
    if (s_ui.link) lora_link_tick(s_ui.link, now);

    /* After a P2P send, go back to listening once the radio reports done (or 3 s). */
    if (s_ui.await_tx && (s_ui.tx_done || lv_tick_elaps(s_ui.await_tx_at) > 3000)) {
        s_ui.await_tx = false;
        s_ui.tx_done  = false;
        cmd("AT+PRECV=65534", 1000, 0, T_RX);
        set_status("P2P LISTENING");
    }
    static uint32_t last;
    if (now - last > 500) {
        last = now;
        if (s_ui.tab == TAB_LINK) render_link();
        if (s_ui.tab == TAB_P2P) render_p2p_info();
    }
}

static int lio_read(uint8_t *b, size_t n, void *c) { (void)c; return hal_mbus_uart_read(b, n); }
static int lio_write(const uint8_t *b, size_t n, void *c) { (void)c; return hal_mbus_uart_write(b, n); }

static bool start(deck_app_t *self, lv_obj_t *parent)
{
    (void)self;
    memset(&s_ui, 0, sizeof(s_ui));
    s_ui.mode = -1;
    pairs_init();
    cfg_load();
    if (!s_duty_ready) {
        lora_duty_init(&s_duty, FREQS[s_cfg.freq].duty_ms);
        s_duty_ready = true;
    }
    s_duty.limit_ms = FREQS[s_cfg.freq].duty_ms;

    lv_obj_t *root = deck_box(parent);
    lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(root, 20, 0);
    lv_obj_set_style_pad_row(root, 14, 0);

    lv_obj_t *bar = deck_box(root);
    lv_obj_set_width(bar, LV_PCT(100));
    lv_obj_t *tabs = deck_box(bar);
    lv_obj_set_flex_flow(tabs, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(tabs, 10, 0);
    static const char *const names[TABS] = {"LINK", "P2P", "LORAWAN"};
    for (int i = 0; i < TABS; i++) {
        s_ui.tab_btn[i] = small_button(tabs, names[i], tab_clicked, i);
        lv_group_remove_obj(s_ui.tab_btn[i]); /* 1..3 switch tabs */
    }
    s_ui.status = deck_label(bar, g_font.mono_s, g_pal.dim, "");
    lv_obj_align(s_ui.status, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *content = deck_box(root);
    lv_obj_set_width(content, LV_PCT(100));
    lv_obj_set_flex_grow(content, 1);
    for (int i = 0; i < TABS; i++) {
        s_ui.page[i] = deck_box(content);
        lv_obj_set_size(s_ui.page[i], LV_PCT(100), LV_PCT(100));
        lv_obj_add_flag(s_ui.page[i], LV_OBJ_FLAG_HIDDEN);
    }
    build_link(s_ui.page[TAB_LINK]);
    build_p2p(s_ui.page[TAB_P2P]);
    build_wan(s_ui.page[TAB_WAN]);

    deck_label(root, g_font.mono_s, g_pal.dim, "[1..3] TAB    [P] FIND MODULE    [TAB] NEXT FIELD    [ENTER] SEND    [ESC] BACK");

    if (!hal_kbd_present()) {
        s_ui.osk = lv_keyboard_create(root);
        lv_obj_add_flag(s_ui.osk, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_ui.osk, LV_PCT(100), LV_PCT(40));
        lv_obj_align(s_ui.osk, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_group_remove_obj(s_ui.osk);
    }

    s_ui.alive = true;
    hal_mbus_power(true);
    static const lora_io_t io = {lio_read, lio_write, NULL};
    s_ui.link  = lora_link_new(&io, on_evt, NULL);
    s_ui.timer = lv_timer_create(tick, 20, NULL);
    set_tab(TAB_LINK);
    probe_begin(false);
    return true;
}

static void stop(deck_app_t *self)
{
    (void)self;
    s_ui.alive = false; /* no callback touches widgets from here on */
    if (s_ui.timer) lv_timer_delete(s_ui.timer);
    lora_link_free(s_ui.link);
    hal_mbus_uart_close();
    memset(&s_ui, 0, sizeof(s_ui));
}

static bool on_key(deck_app_t *self, const deck_key_t *k)
{
    (void)self;
    if (k->mods & (DECK_MOD_CTRL | DECK_MOD_ALT)) return false;
    lv_obj_t *f = lv_group_get_focused(deck_input_group());
    if (f && lv_obj_check_type(f, &lv_textarea_class)) return false; /* typing */
    if (k->code >= HID_1 && k->code < HID_1 + TABS) {
        set_tab(k->code - HID_1);
        return true;
    }
    if (k->code == HID_P) {
        probe_begin(false);
        return true;
    }
    return false;
}

static deck_app_t s_app = {
    .name     = "LORA",
    .tagline  = "EU868 MODULE",
    .icon     = ICON_ANTENNA,
    .eta      = NULL,
    .hotkey   = HID_LETTER('L'),
    .on_start = start,
    .on_stop  = stop,
    .on_key   = on_key,
};

deck_app_t *app_lora(void) { return &s_app; }
