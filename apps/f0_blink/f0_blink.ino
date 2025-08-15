#include <Arduino.h>
#include "xassert.h"
#include "sys_led.h"
#include "dcc_config.h"
#include "dcc_throttle.h"
#include "dcc_command.h"
#include "dcc_function.h"

static DccAdc adc(-1);
static DccCommand command(dcc_sig_gpio, dcc_pwr_gpio, adc);
static DccThrottle *throttle = nullptr;


void setup()
{
    Serial.begin(115200);

    SysLed::begin();
#if 0
    SysLed::pattern(50, 950); // indicates "waiting for serial"

    while (!Serial)
        SysLed::loop();

    SysLed::off();
#endif

    Serial.printf("\n");
    Serial.printf("f0_blink\n");
    Serial.printf("\n");

    throttle = command.create_throttle();
    throttle->address(1734);

    command.mode_ops();
}


void loop()
{
    command.loop();

    uint32_t now_ms = millis();

    const uint32_t blink_hz = 2;
    const uint32_t half_ms = 1000 / blink_hz / 2;

    static uint32_t next_ms = now_ms + half_ms;
    static bool f0 = false;

    if (now_ms < next_ms)
        return;

    next_ms += half_ms;

    f0 = !f0;

    throttle->function(DccFunction::headlight, f0);
}
