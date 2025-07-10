
#include <Arduino.h>
#include "sys_led.h"
#include "dcc_bit.h"
#include "dcc_halfs.h"

static const int dcc_verbosity = 9;

static DccBit dcc(dcc_verbosity);


void setup()
{
    Serial.begin(115200);

    SysLed::begin();
    SysLed::pattern(50, 950); // indicates "waiting for serial"

    while (!Serial)
        SysLed::loop();

    SysLed::off();

    dcc.begin();

    for (int i = 0; i < dcc_halfs_max; i++) {
        Serial.printf("%u:", dcc_halfs[i]);
        dcc.half_bit(dcc_halfs[i]);
        Serial.printf("\n");
    }
}


void loop()
{
}
