#include <Arduino.h>
#include "sys_led.h"
#include "edges.h"
#include "dcc_config.h"

// Record histogram of DCC intervals

// Units
//  _tk ticks (received from pio)
//  _ns nanoseconds
//  _us microseconds
//  _ms milliseconds
//  _hz Hertz
//  _ct counts (unitless)

// Measurement resolution.
// Should divide evenly into 50. Valid settings:
//   1 tick/usec    1.00 usec/tick
//   2 tick/usec    0.50 usec/tick
//   5 tick/usec    0.20 usec/tick
//  10 tick/usec    0.10 usec/tick
//  25 tick/usec    0.04 usec/tick
//  50 tick/usec    0.02 usec/tick
static const int tpu = 50;

// Histogram bin index is ticks (_tk)
static const int hist_max_us = 200;                 // last bin, usec
static const int hist_max_tk = hist_max_us * tpu;   // last bin, tick
static uint16_t hist_hi_ct[hist_max_tk];
static uint16_t hist_lo_ct[hist_max_tk];
static uint32_t hist_ct = 0;

// How many intervals to histogram
// The bitstream runs roughly 10,000 bits/sec.
// Set to UINT32_MAX to run until any bin fills up.
// 100,000 half-bits takes about 8 seconds.
// Until-full takes about 2 minutes.
static const uint32_t hist_end_ct = 200000; //UINT32_MAX;

// PIO timestamp resolution
static const uint pio_tick_hz = 1'000'000 * tpu;

// DCC spec - these are half-bits, not full bit times
static const uint32_t min_1_us = 52;
static const uint32_t nom_1_us = 58;
static const uint32_t max_1_us = 64;
static const uint32_t min_0_us = 90;
static const uint32_t nom_0_us = 100;
static const uint32_t max_0_us = 10000;

// Dcc spec numbers converted to ticks
static const uint32_t min_1_tk = min_1_us * tpu;
static const uint32_t nom_1_tk = nom_1_us * tpu;
static const uint32_t max_1_tk = max_1_us * tpu;
static const uint32_t min_0_tk = min_0_us * tpu;
static const uint32_t nom_0_tk = nom_0_us * tpu;
static const uint32_t max_0_tk = max_0_us * tpu;


static inline float tick_to_usec(uint32_t tk)
{
    return float(tk) / float(tpu);
}


static inline uint32_t divide_round(uint32_t top, uint32_t bot)
{
    return (top + bot / 2) / bot;
}


void setup()
{
    Serial.begin(115200);

    SysLed::begin();
    SysLed::pattern(50, 950); // indicates "waiting for serial"

    while (!Serial)
        SysLed::loop();

    SysLed::off();
    
    Serial.printf("\n");
    Serial.printf("DCC Bit Histogram\n");
    Serial.printf("\n");

    Edges::setup(dcc_sig_gpio, pio_tick_hz);
}


void loop()
{
    static uint32_t start_ms = millis();

    SysLed::loop();

    if (hist_ct >= hist_end_ct)
        return;

    int rise;
    uint32_t edge_tk;
    if (!Edges::get_tick(rise, edge_tk))
        return;

    // When decoding DCC, we don't care which edge it is, but when looking
    // for asymmetry between rising and falling edges, it's nice to know.
    // Asymmetry happens when one edge (falling) is nice & sharp, and the
    // other (rising) is slow. If you look at a trace and see where the
    // transition start, the fast edge gets its timestamp right away but
    // the slow edge gets its timestamp a tiny bit later. The histogram
    // printout shows this clearly.

    // Rising edges are adjusted for the slow rise time
    const uint32_t adj_ns = 440;
    if (rise == 1)
        edge_tk -= ((tpu * adj_ns + 500) / 1000);

    static uint32_t edge_prv_tk = UINT32_MAX;

    if (edge_prv_tk != UINT32_MAX) {

        uint32_t edge_int_tk = edge_tk - edge_prv_tk;

        // hist_lo_ct is low intervals and hist_hi_ct is high intervals
        uint32_t bin_ct;
        if (rise == 1)
            bin_ct = ++hist_lo_ct[edge_int_tk]; // rising edge ends a low interval
        else
            bin_ct = ++hist_hi_ct[edge_int_tk]; // falling edge ends a high interval

        // Continue until hist_ct >= hist_end_ct, or until a bin is full,
        // whichever comes first
        if (bin_ct == UINT16_MAX)
            hist_ct = hist_end_ct;
        else
            ++hist_ct;

        if (hist_ct >= hist_end_ct) {

            // histogram full; print it

            Serial.printf("%d ticks/usec\n\n", tpu);

            Serial.printf("ticks   usec        hi    lo\n");
            //             ddddd fff.ff  c  uuuuu uuuuu

            // calculate averages of high and low intervals for each histogram

            uint32_t hi_1_sum_tk = 0;
            uint32_t hi_1_ct = 0;

            uint32_t hi_0_sum_tk = 0;
            uint32_t hi_0_ct = 0;

            uint32_t lo_1_sum_tk = 0;
            uint32_t lo_1_ct = 0;

            uint32_t lo_0_sum_tk = 0;
            uint32_t lo_0_ct = 0;

            for (uint32_t tk = 0; tk < hist_max_tk; ++tk) {

                if (tk == min_1_tk || tk == max_1_tk || tk == min_0_tk || tk == max_0_tk)
                    Serial.printf("----------------------------\n");

                if (hist_hi_ct[tk] == 0 && hist_lo_ct[tk] == 0)
                    continue;

                char bit_type = ' ';
                if (min_0_tk <= tk && tk <= max_0_tk) {
                    bit_type = '0';
                    lo_0_sum_tk += (tk * hist_lo_ct[tk]);
                    lo_0_ct += hist_lo_ct[tk];
                    hi_0_sum_tk += (tk * hist_hi_ct[tk]);
                    hi_0_ct += hist_hi_ct[tk];
                } else if (min_1_tk <= tk && tk <= max_1_tk) {
                    bit_type = '1';
                    lo_1_sum_tk += (tk * hist_lo_ct[tk]);
                    lo_1_ct += hist_lo_ct[tk];
                    hi_1_sum_tk += (tk * hist_hi_ct[tk]);
                    hi_1_ct += hist_hi_ct[tk];
                }

                Serial.printf("%5d %6.2f  %c  %5u %5u\n", tk, tick_to_usec(tk),
                              bit_type, hist_hi_ct[tk], hist_lo_ct[tk]);

            } // for (int tk...)

            Serial.printf("----------------------------\n");
            Serial.printf("\n");

            Serial.printf("averages:\n");

            uint32_t avg_lo_1_tk = divide_round(lo_1_sum_tk, lo_1_ct);
            uint32_t avg_hi_1_tk = divide_round(hi_1_sum_tk, hi_1_ct);
            uint32_t avg_lo_0_tk = divide_round(lo_0_sum_tk, lo_0_ct);
            uint32_t avg_hi_0_tk = divide_round(hi_0_sum_tk, hi_0_ct);

            float avg_lo_1_us = tick_to_usec(avg_lo_1_tk);
            float avg_hi_1_us = tick_to_usec(avg_hi_1_tk);
            float avg_lo_0_us = tick_to_usec(avg_lo_0_tk);
            float avg_hi_0_us = tick_to_usec(avg_hi_0_tk);

            int32_t adj_1_ns = ((avg_lo_1_us - avg_hi_1_us) * 1000) / 2;
            int32_t adj_0_ns = ((avg_lo_0_us - avg_hi_0_us) * 1000) / 2;

            Serial.printf("  half-one:  lo %lu_tk = %6.2f_us, hi %lu_tk = %6.2f_us"
                          " (adj_ns += %ld)\n",
                          avg_lo_1_tk, avg_lo_1_us, avg_hi_1_tk, avg_hi_1_us,
                          adj_1_ns);

            Serial.printf("  half-zero: lo %lu_tk = %6.2f_us; hi %lu_tk = %6.2f_us"
                          " (adj_ns += %ld)\n",
                          avg_lo_0_tk, avg_lo_0_us, avg_hi_0_tk, avg_hi_0_us,
                          adj_0_ns);

            Serial.printf("\n");

            Serial.printf("adj_ns: %ld_ns -> %ld_ns\n",
                          adj_ns, adj_ns + (adj_1_ns + adj_0_ns) / 2);

            Serial.printf("\n");

            // print reminder of what valid bits are

            Serial.printf("dcc spec:\n");

            Serial.printf("  half-one:  %lu_us - %lu_us - %lu_us\n",
                          min_1_us, nom_1_us, max_1_us);
            Serial.printf("  half-zero: %lu_us - %lu_us - %lu_us\n",
                          min_0_us, nom_0_us, max_0_us);

            Serial.printf("\n");

            Serial.printf("done in %0.1f sec\n", (millis() - start_ms) / 1000.0);

            SysLed::on();

        } // if (hist_ct >= hist_end_ct)

    } // if (edge_prv_tk != UINT32_MAX)

    edge_prv_tk = edge_tk;

} // void loop()
