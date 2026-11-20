/* Host test: gcc -I include test_lora.c lora_at.c -lm -o /tmp/lora_test && /tmp/lora_test */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "lora_at.h"

static char rx[2048]; static size_t rx_r, rx_w;
static char tx[2048]; static size_t tx_w;
static void feed(const char *s) { size_t n = strlen(s); memcpy(rx + rx_w, s, n); rx_w += n; }
static int io_read(uint8_t *b, size_t n, void *c) { (void)c; size_t k = rx_w - rx_r; if (k > n) k = n; memcpy(b, rx + rx_r, k); rx_r += k; return (int)k; }
static int io_write(const uint8_t *b, size_t n, void *c) { (void)c; memcpy(tx + tx_w, b, n); tx_w += n; tx[tx_w] = 0; return (int)n; }

static lora_rc_t g_rc = (lora_rc_t)-1; static char g_reply[600]; static int g_done;
static void done(lora_rc_t rc, const char *r, int tag, void *u) { (void)tag; (void)u; g_rc = rc; snprintf(g_reply, sizeof g_reply, "%s", r); g_done++; }
static lora_evt_t g_evt[8]; static int g_evn;
static void on_evt(const lora_evt_t *e, void *u) { (void)u; if (g_evn < 8) g_evt[g_evn++] = *e; }

int main(void)
{
    lora_io_t io = { io_read, io_write, NULL };
    lora_link_t *l = lora_link_new(&io, on_evt, NULL);

    /* query with echo and prefixed reply */
    lora_link_send(l, "AT+VER=?", 1000, 0, done, 1, NULL);
    lora_link_tick(l, 0);
    assert(strcmp(tx, "AT+VER=?\r\n") == 0);
    feed("AT+VER=?\r\nAT+VER=RUI_4.0.4_RAK3172-E\r\nOK\r\n");
    lora_link_tick(l, 10);
    assert(g_rc == LORA_OK);
    char v[64]; assert(lora_reply_value(g_reply, "VER", v, sizeof v)); assert(strcmp(v, "RUI_4.0.4_RAK3172-E") == 0);

    /* error */
    g_rc = (lora_rc_t)-1; lora_link_send(l, "AT+FOO=1", 1000, 0, done, 2, NULL);
    lora_link_tick(l, 20); feed("AT_PARAM_ERROR\r\n"); lora_link_tick(l, 30);
    assert(g_rc == LORA_ERR_AT); assert(strcmp(g_reply, "AT_PARAM_ERROR") == 0);

    /* timeout */
    g_rc = (lora_rc_t)-1; lora_link_send(l, "AT", 500, 0, done, 3, NULL);
    lora_link_tick(l, 100); lora_link_tick(l, 700);
    assert(g_rc == LORA_ERR_TIMEOUT);

    /* events interleaved with a command */
    g_rc = (lora_rc_t)-1; g_evn = 0;
    lora_link_send(l, "AT+PSEND=4869", 1000, 0, done, 4, NULL);
    lora_link_tick(l, 800);
    feed("+EVT:RXP2P:-60:11:48656C6C6F\r\nOK\r\n+EVT:TXP2P DONE\r\n");
    lora_link_tick(l, 810);
    assert(g_rc == LORA_OK && g_evn == 2);
    assert(g_evt[0].type == LORA_EVT_RX_P2P && g_evt[0].rssi == -60 && g_evt[0].snr == 11 && g_evt[0].len == 5 && memcmp(g_evt[0].data, "Hello", 5) == 0);
    assert(g_evt[1].type == LORA_EVT_TX_DONE);

    /* LoRaWAN downlink and join events (both spellings) */
    g_evn = 0;
    feed("+EVT:RX_1:-95:7:UNICAST:2:CAFE\r\n+EVT:JOINED\r\n+EVT:JOIN_FAILED_RX_TIMEOUT\r\n+EVT:SEND CONFIRMED OK\r\n+EVT:SEND_CONFIRMED_FAILED\r\n");
    lora_link_tick(l, 900);
    assert(g_evn == 5);
    assert(g_evt[0].type == LORA_EVT_RX_WAN && g_evt[0].port == 2 && g_evt[0].len == 2 && g_evt[0].data[0] == 0xCA);
    assert(g_evt[1].type == LORA_EVT_JOINED && g_evt[2].type == LORA_EVT_JOIN_FAILED);
    assert(g_evt[3].type == LORA_EVT_SEND_OK && g_evt[4].type == LORA_EVT_SEND_FAILED);

    /* reboot command completes on the banner */
    g_rc = (lora_rc_t)-1; g_evn = 0;
    lora_link_send(l, "AT+NWM=0", 5000, LORA_F_REBOOT, done, 5, NULL);
    lora_link_tick(l, 1000);
    feed("OK\r\nRAKwireless RAK3172\r\n------\r\nVersion: RUI_4.0.4_RAK3172-E\r\nCurrent Work Mode: LoRa P2P.\r\n");
    lora_link_tick(l, 1100);
    assert(g_rc == LORA_OK && g_evn == 1 && g_evt[0].type == LORA_EVT_BOOT);

    /* garbage and split lines */
    g_rc = (lora_rc_t)-1; lora_link_send(l, "AT", 500, 0, done, 6, NULL);
    lora_link_tick(l, 2000); feed("O"); lora_link_tick(l, 2001); feed("K\r\n"); lora_link_tick(l, 2002);
    assert(g_rc == LORA_OK);

    /* flush */
    g_done = 0; lora_link_send(l, "AT", 500, 0, done, 7, NULL); lora_link_send(l, "AT", 500, 0, done, 8, NULL);
    lora_link_flush(l); assert(g_done == 2 && !lora_link_busy(l));

    /* hex */
    uint8_t b[4]; assert(lora_hex_decode("0aFf", b, 4) == 2 && b[0] == 10 && b[1] == 255);
    assert(lora_hex_decode("abc", b, 4) < 0); assert(lora_hex_decode("zz", b, 4) < 0);
    char h[16]; lora_hex_encode((const uint8_t *)"Hi", 2, h, sizeof h); assert(strcmp(h, "4869") == 0);

    /* airtime: SF7/125k, 10 byte payload, 8 preamble, CR4/5 = 41.2 ms (Semtech calculator); SF12/125k 10 B ~ 991 ms */
    uint32_t t7 = lora_airtime_ms(10, 7, 125, 1, 8); printf("SF7 10B = %u ms\n", t7); assert(t7 >= 40 && t7 <= 43);
    uint32_t t12 = lora_airtime_ms(10, 12, 125, 1, 8); printf("SF12 10B = %u ms\n", t12); assert(t12 >= 990 && t12 <= 1000);

    /* duty cycle */
    lora_duty_t d; lora_duty_init(&d, 36000);
    assert(lora_duty_allow(&d, 0, 30000)); lora_duty_add(&d, 0, 30000);
    assert(!lora_duty_allow(&d, 1000, 7000)); assert(lora_duty_allow(&d, 1000, 6000));
    assert(lora_duty_used(&d, 61 * 60000u) == 0); /* an hour later the budget is back */

    lora_link_free(l);
    puts("lora_at: all tests passed");
    return 0;
}
