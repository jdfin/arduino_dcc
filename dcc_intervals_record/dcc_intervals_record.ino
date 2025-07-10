#include <Arduino.h>

#include "sys_led.h"
#include "xassert.h"
#include "edges.h"

// Record intervals between edges on a gpio

// Units
//  _tk ticks (received from pio)
//  _ns nanoseconds
//  _us microseconds
//  _ms milliseconds
//  _hz Hertz
//  _ct counts (unitless)

// Measurement resolution, ticks/microsecond.
static const int tpu = 50;

static const int dcc_gpio = 7;

static const int interval_end_ct = 1024; // total intervals to record
static const int line_ct = 8; // print this many per line
static int32_t interval_tk[interval_end_ct];
static int interval_ct = 0;

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
    Serial.printf("Recording Edges on GPIO %d\n", dcc_gpio);
    Serial.printf("\n");
    Serial.printf("Tick rate %u MHz\n", tpu);
    Serial.printf("\n");

    Edges::setup(dcc_gpio, pio_tick_hz);
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

    if (interval_ct >= interval_end_ct)
        return;

    uint32_t tk;
    if (!Edges::get_tick(tk))
        return;

    int32_t edge_tk = int32_t(tk);

    bool is_hi = gpio_get(dcc_gpio);

    // Rising edges are adjusted for the slow rise time
    const uint32_t adj_ns = 440;
    if (is_hi)
        edge_tk -= ((tpu * adj_ns + 500) / 1000);

    static int32_t edge_prv_tk = INT32_MAX;

    if (edge_prv_tk != INT32_MAX) {

        int32_t edge_int_tk = edge_tk - edge_prv_tk;

        if (is_hi)
            interval_tk[interval_ct] = edge_int_tk; // positive is interval to rising edge
        else
            interval_tk[interval_ct] = -edge_int_tk; // negative is interval to falling edge

        interval_ct++;

        if (interval_ct >= interval_end_ct) {
            // recording full

            Serial.printf("\n");

            snipline("dcc_interval.h");

            Serial.printf("#pragma once\n");
            Serial.printf("\n");
            Serial.printf("#include <Arduino.h>\n");
            Serial.printf("\n");
            Serial.printf("const int dcc_interval_max = %d;\n", interval_end_ct);
            Serial.printf("\n");
            Serial.printf("extern float dcc_interval_us[];\n");

            snipline("dcc_interval.h");

            Serial.printf("\n");
            Serial.printf("\n");

            snipline("dcc_interval.cpp");

            Serial.printf("#include <Arduino.h>\n");
            Serial.printf("#include \"dcc_interval.h\"\n");
            Serial.printf("\n");
            Serial.printf("// positive is interval to rising edge\n");
            Serial.printf("// negative is interval to falling edge\n");
            Serial.printf("float dcc_interval_us[dcc_interval_max] = {\n");

            interval_ct = 0;
            while (interval_ct < interval_end_ct) {
                Serial.printf("   ");
                for (int ct = 0; ct < line_ct; ct++)
                    Serial.printf(" %7.2f,", float(interval_tk[interval_ct++]) / float(tpu));
                Serial.printf("\n");
            }

            Serial.printf("}; // dcc_interval_us[]\n");

            snipline("dcc_interval.cpp");

            Serial.printf("\n");
            Serial.printf("\n");

            Serial.printf("done\n");

            SysLed::on();
        }

    } // if (edge_prv_tk != UINT32_MAX)

    edge_prv_tk = edge_tk;

} // void loop()
