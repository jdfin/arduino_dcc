#include <Arduino.h>
#include "xassert.h"
#include "sys_led.h"
#include "dcc_config.h"
#include "dcc_adc.h"
#include "dcc_throttle.h"
#include "dcc_command.h"

static DccAdc adc(dcc_adc_gpio);
static DccCommand command(dcc_sig_gpio, dcc_pwr_gpio, adc);

static const int cv_num = 8;


void setup()
{
    Serial.begin(115200);

    SysLed::begin();
    SysLed::pattern(50, 950); // indicates "waiting for serial"

    while (!Serial)
        SysLed::loop();

    SysLed::off();

    Serial.printf("\n");
    Serial.printf("svc_read: reading cv%d\n", cv_num);
    Serial.printf("\n");

    adc.log_reset();

    command.mode_svc_read_cv(cv_num);
}


void loop()
{
    command.loop();

    static uint32_t start_ms = millis();
    static uint32_t time_ms;

    static bool done = false;
    static bool result = false;
    static bool printed = false;
    static uint8_t value;

    if (!done) {
        done = command.svc_done(result, value);
        time_ms = millis() - start_ms;
    }

    if (millis() >= (start_ms + 2000) && !printed) {
        if (result)
            Serial.printf("succeeded in %lu msec: cv%d = %u (0x%02x)\n",
                          time_ms, cv_num, uint(value), uint(value));
        else
            Serial.printf("failed in %lu msec\n", time_ms);
        command.show_ack_ma();
        adc.log_show();
        printed = true;
    }
}
