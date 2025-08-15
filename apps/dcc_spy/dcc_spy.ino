#include <Arduino.h>
#include "sys_led.h"
#include "xassert.h"
#include "edges.h"
#include "dcc_config.h"
#include "dcc_bit.h"
#include "dcc_pkt.h"

static const int verbosity = 0;

// Measurement resolution, ticks/microsecond (1, 2, 5, 10, 25, 50).
static const int tpu = 10;

// PIO timestamp resolution
static const uint pio_tick_hz = 1'000'000 * tpu;

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
    Serial.printf("DCC Spy on GPIO %d\n", dcc_sig_gpio);
    Serial.printf("\n");

    Edges::setup(dcc_sig_gpio, pio_tick_hz);

    dcc.on_pkt_recv(&pkt_recv);

    dcc.begin();
}


void loop()
{
    SysLed::loop();

    int rise;
    uint32_t edge32_tk;
    if (!Edges::get_tick(rise, edge32_tk))
        return;

    // Rising edges are adjusted for the slow rise time
    // subtracting adj_ns[ns] * (1[us]/1000[ns]) * (tpu[tk]/1[us])
    //           = (adj_ns * tpu / 1000)[tk], and do it with rounding
    const uint32_t adj_ns = 440;
    if (rise == 1)
        edge32_tk -= ((adj_ns * tpu + 500) / 1000);

    // The timestamp from the PIO is 32 bits. Worst case is 0.02 usec/tick,
    // which wraps to zero after 85 seconds. Since we expect edges 10s of usec
    // apart, we convert to a 64-bit timestamp by just looking for that
    // overflow and incrementing the upper half.

    static uint32_t edge32_last_tk = edge32_tk;

    static uint32_t edge32_tk_top = 0;

    if (edge32_tk < edge32_last_tk)
        edge32_tk_top++; // overflow

    // 64 bit ticks at 50 MHz wraps after > 11,000 years.
    uint64_t edge_tk = (uint64_t(edge32_tk_top) << 32) | edge32_tk;

    // convert from ticks to microseconds (with rounding)
    uint64_t edge_us = (edge_tk + tpu/2) / tpu;

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

    for (int i = pkt_len; i < 6; i++)
        Serial.printf("   ");

    Serial.printf(" (%s)", (check == 0) ? "ok" : "error");

    DccPkt msg(pkt, pkt_len);

    char buf[80];
    Serial.printf(" %s", msg.show(buf, sizeof(buf)));

    if (bad_cnt != 0)
        Serial.printf(" bad_cnt=%d", bad_cnt);

    Serial.printf("\n");

    last_pkt_us = start_us;

} // static void pkt_recv(...)
