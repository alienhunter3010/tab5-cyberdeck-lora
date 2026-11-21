/* End-to-end: the real AT link against the simulator's fake RAK3172.
 * gcc -Icomponents/deck_lora/include -Icomponents/deck_hal/include sim/lora_flow_test.c sim/lora_sim.c components/deck_lora/lora_at.c -lm */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include "deck_hal.h"
#include "lora_at.h"

static int rd(uint8_t *b, size_t n, void *c) { (void)c; return hal_mbus_uart_read(b, n); }
static int wr(const uint8_t *b, size_t n, void *c) { (void)c; return hal_mbus_uart_write(b, n); }

static uint32_t now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint32_t)(t.tv_sec * 1000u + t.tv_nsec / 1000000u); }
static lora_rc_t rcs[32]; static char rep[32][120]; static int nrc;
static void done(lora_rc_t rc, const char *r, int tag, void *u) { (void)u; rcs[tag] = rc; snprintf(rep[tag], sizeof rep[tag], "%s", r); nrc++; }
static lora_evt_t evs[16]; static int nev;
static void evt(const lora_evt_t *e, void *u) { (void)u; if (nev < 16) evs[nev++] = *e; }
static void run(lora_link_t *l, uint32_t ms) { uint32_t t0 = now(); while (now() - t0 < ms) { lora_link_tick(l, now()); usleep(2000); } }
static bool saw(lora_evt_type_t t) { for (int i = 0; i < nev; i++) if (evs[i].type == t) return true; return false; }

int main(void)
{
    lora_io_t io = {rd, wr, NULL};
    lora_link_t *l = lora_link_new(&io, evt, NULL);

    /* the fake module answers on TX G6 / RX G7 only */
    static const int pairs[][2] = {{7, 6}, {37, 38}, {38, 37}, {6, 7}};
    for (int c = 0; c < 4; c++) {
        hal_mbus_uart_open(pairs[c][0], pairs[c][1], 115200);
        nrc = 0; lora_link_send(l, "AT", 300, 0, done, 1, NULL); run(l, 400);
        printf("TX G%d RX G%d: %s\n", pairs[c][0], pairs[c][1], rcs[1] == LORA_OK ? "ANSWERS" : "silent");
        assert((c == 3) == (rcs[1] == LORA_OK));
    }

    hal_mbus_uart_open(6, 7, 115200);
    char v[64];
    lora_link_send(l, "AT+VER=?", 500, 0, done, 2, NULL); lora_link_send(l, "AT+NWM=?", 500, 0, done, 3, NULL); run(l, 100);
    assert(lora_reply_value(rep[2], "VER", v, sizeof v) && strstr(v, "RUI_"));
    assert(lora_reply_value(rep[3], "NWM", v, sizeof v) && atoi(v) == 0);

    /* P2P: configure, listen, TX needs listening off, loopback inbound */
    lora_link_send(l, "AT+P2P=868100000:7:125:0:8:14", 500, 0, done, 4, NULL);
    lora_link_send(l, "AT+PRECV=65534", 500, 0, done, 5, NULL);
    lora_link_send(l, "AT+PSEND=4869", 500, 0, done, 6, NULL); run(l, 150);
    assert(rcs[4] == LORA_OK && rcs[5] == LORA_OK && rcs[6] == LORA_ERR_AT && strstr(rep[6], "BUSY")); /* the reason for PRECV=0 first */
    lora_link_send(l, "AT+PRECV=0", 500, 0, done, 7, NULL); lora_link_send(l, "AT+PSEND=4869", 500, 0, done, 8, NULL); run(l, 500);
    assert(rcs[8] == LORA_OK && saw(LORA_EVT_TX_DONE));
    lora_link_send(l, "AT+PRECV=65534", 500, 0, done, 9, NULL); run(l, 9500);
    assert(saw(LORA_EVT_RX_P2P)); for (int i = 0; i < nev; i++) if (evs[i].type == LORA_EVT_RX_P2P) printf("RX %d dBm %d dB: %.*s\n", evs[i].rssi, evs[i].snr, (int)evs[i].len, evs[i].data);

    /* switch to LoRaWAN (reboot flag), join with a key, send, downlink */
    nev = 0; lora_link_send(l, "AT+PRECV=0", 500, 0, done, 10, NULL);
    lora_link_send(l, "AT+NWM=1", 6000, LORA_F_REBOOT, done, 11, NULL); run(l, 800);
    assert(rcs[11] == LORA_OK && saw(LORA_EVT_BOOT));
    lora_link_send(l, "AT+APPKEY=00112233445566778899AABBCCDDEEFF", 500, 0, done, 12, NULL);
    lora_link_send(l, "AT+APPKEY=ZZ", 500, 0, done, 13, NULL);
    lora_link_send(l, "AT+JOIN=1:0:10:8", 500, 0, done, 14, NULL); run(l, 2500);
    assert(rcs[12] == LORA_OK && rcs[13] == LORA_ERR_AT && rcs[14] == LORA_OK && saw(LORA_EVT_JOINED));
    nev = 0; lora_link_send(l, "AT+SEND=2:48656C6C6F", 500, 0, done, 15, NULL); run(l, 3000);
    assert(rcs[15] == LORA_OK && saw(LORA_EVT_SEND_OK) && saw(LORA_EVT_RX_WAN));
    for (int i = 0; i < nev; i++) if (evs[i].type == LORA_EVT_RX_WAN) assert(evs[i].port == 2 && memcmp(evs[i].data, "ACK", 3) == 0);

    puts("lora flow: all checks passed");
    lora_link_free(l);
    return 0;
}
