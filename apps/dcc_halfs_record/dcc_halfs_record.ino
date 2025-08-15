#include <Arduino.h>
#include "sys_led.h"
#include "xassert.h"
#include "edges.h"
#include "dcc_config.h"
#include "dcc_bit.h"

// Record half-bits from a gpio

// Measurement resolution, ticks/microsecond.
static const int tpu = 50;

static const int halfs_end_ct = 8192; // total halfs to record
static const int line_ct = 32; // print this many per line
static uint8_t halfs[halfs_end_ct];
static int halfs_ct = 0;

// PIO timestamp resolution
static const uint pio_tick_hz = 1'000'000 * tpu;


void setup()
{
    Serial.begin(115200);

    SysLed::begin();
    SysLed::pattern(50, 950); // indicates "waiting for serial"

    while (!Serial)
        SysLed::loop();

    SysLed::off();

    Serial.printf("\n");
    Serial.printf("Recording Half-Bits on GPIO %d\n", dcc_sig_gpio);
    Serial.printf("\n");
    Serial.printf("Tick rate %u MHz\n", tpu);
    Serial.printf("\n");

    Edges::setup(dcc_sig_gpio, pio_tick_hz);
}


static void snipline(const char *filename)
{
    const char *dash = "--------";
    char snip_str[79]; // 78 + '\0'
    snprintf(snip_str, sizeof(snip_str), "8<%s %s %s%s%s%s%s%s%s%s",
             dash, filename, dash, dash, dash, dash, dash, dash, dash, dash);
    xassert(strlen(snip_str) == (sizeof(snip_str) - 1));
    Serial.printf("%s\n", snip_str);
}


void loop()
{
    SysLed::loop();

    if (halfs_ct >= halfs_end_ct)
        return;

    int rise;
    uint32_t edge_tk;
    if (!Edges::get_tick(rise, edge_tk))
        return;

    // Rising edges are adjusted for the slow rise time
    const uint32_t adj_ns = 440;
    if (rise == 1)
        edge_tk -= ((tpu * adj_ns + 500) / 1000);

    static uint32_t edge_prv_tk = UINT32_MAX;

    if (edge_prv_tk != UINT32_MAX) {

        // This is wraparound-safe as long as intervals are less than
        // UINT32_MAX/2. At 50 MHz, UINT32_MAX/2 ticks is about 43 sec.
        uint32_t edge_int_tk = edge_tk - edge_prv_tk;
        xassert(edge_int_tk <= (UINT32_MAX / 2));

        int usec = roundf(float(edge_int_tk) / float(tpu));

        halfs[halfs_ct] = DccBit::to_half(usec);

        halfs_ct++;

        if (halfs_ct >= halfs_end_ct) {
            // recording full

            Serial.printf("\n");

            snipline("dcc_halfs.h");

            Serial.printf("#pragma once\n");
            Serial.printf("\n");
            Serial.printf("#include <Arduino.h>\n");
            Serial.printf("\n");
            Serial.printf("const int dcc_halfs_max = %d;\n", halfs_end_ct);
            Serial.printf("\n");
            Serial.printf("extern const uint8_t dcc_halfs[];\n");

            snipline("dcc_halfs.h");

            Serial.printf("\n");
            Serial.printf("\n");

            snipline("dcc_halfs.cpp");

            Serial.printf("#include <Arduino.h>\n");
            Serial.printf("#include \"dcc_halfs.h\"\n");
            Serial.printf("\n");
            Serial.printf("uint8_t const dcc_halfs[dcc_halfs_max] = {\n");

            int count_0 = 0;
            int count_1 = 0;
            int count_x = 0;

            halfs_ct = 0;
            while (halfs_ct < halfs_end_ct) {
                Serial.printf("    ");
                for (int i = 0; i < line_ct; i++) {
                    int half = halfs[halfs_ct++];
                    if (half == 0)
                        count_0++;
                    else if (half == 1)
                        count_1++;
                    else
                        count_x++;
                    Serial.printf("%d,", half);
                }
                Serial.printf("\n");

            } // while

            Serial.printf("}; // dcc_halfs[]\n");

            Serial.printf("\n");

            Serial.printf("// 0: %d\n", count_0);
            Serial.printf("// 1: %d\n", count_1);
            Serial.printf("// X: %d\n", count_x);

            snipline("dcc_halfs.cpp");

            Serial.printf("\n");
            Serial.printf("\n");

            Serial.printf("done\n");

            SysLed::on();
        }

    } // if (edge_prv_tk != UINT32_MAX)

    edge_prv_tk = edge_tk;

} // void loop()
