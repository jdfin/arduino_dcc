#include <Arduino.h>

#include "sys_led.h"
#include "xassert.h"

#include "edges.h"

#include "dcc_config.h"
#include "dcc_bit.h"
#include "dcc_pkt.h"

static const int verbosity = 0;

// PIO timestamp resolution
// Much below assumes this is 1 MHz
static const uint pio_tick_hz = 1'000'000;

static DccBit dcc(verbosity);

static void pkt_recv(const uint8_t *pkt, int pkt_len, int preamble_len,
                     uint64_t start_us, int bad_cnt);


void setup()
{
    Serial.begin(115200);

    SysLed::begin();
    SysLed::pattern(50, 950); // indicates "waiting for serial"

    while (!Serial)
        SysLed::loop();

    SysLed::off();

    Serial.printf("\n");
    Serial.printf("DCC Spy on GPIO %d\n", dcc_signal_gpio);
    Serial.printf("\n");

    Edges::setup(dcc_signal_gpio, pio_tick_hz);

    dcc.on_pkt_recv(&pkt_recv);

    dcc.begin();
}


void loop()
{
    SysLed::loop();

    uint32_t edge32_us;
    if (!Edges::get_tick(edge32_us))
        return;

    // The timestamp from the PIO is 32 bits (in usec). After 71 minutes,
    // it will wrap around to zero again. Since we expect edges 10s of usec
    // apart, we convert to a 64-bit timestamp by just looking for that
    // overflow and incrementing the upper half.

    static uint32_t edge32_last_us = edge32_us;

    static uint32_t edge_top = 0;

    if (edge32_us < edge32_last_us)
        edge_top++; // overflow

    uint64_t edge_us = (uint64_t(edge_top) << 32) | edge32_us;

    dcc.edge(edge_us);
}


static void pkt_recv(const uint8_t *pkt, int pkt_len, int preamble_len,
                     uint64_t start_us, int bad_cnt)
{
    static uint64_t last_pkt_us = 0;

    uint8_t check = 0;
    for (int i = 0; i < pkt_len; i++)
        check ^= pkt[i];

    Serial.printf("%8llu %8llu p: %d pkt:", start_us, start_us - last_pkt_us, preamble_len);

    for (int i = 0; i < pkt_len; i++)
        Serial.printf(" %02x", pkt[i]);

    for (int i = pkt_len; i < 4; i++)
        Serial.printf("   ");

    Serial.printf(" (%s)", (check == 0) ? "ok" : "error");

    DccPkt msg;
    msg.make_raw(pkt, pkt_len);

    char buf[80];
    Serial.printf(" %s", msg.show(buf, sizeof(buf)));

    if (bad_cnt != 0)
        Serial.printf(" bad_cnt=%d", bad_cnt);

    Serial.printf("\n");

    last_pkt_us = start_us;

} // static void pkt_recv(...)
