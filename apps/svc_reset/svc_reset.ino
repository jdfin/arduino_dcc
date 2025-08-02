#include <Arduino.h>

#include "xassert.h"
#include "sys_led.h"

#include "dcc_adc.h"
#include "dcc_throttle.h"
#include "dcc_command.h"


static const int sig_gpio = 17;
static const int pwr_gpio = 16;
static const int adc_gpio = 26; // ADC0

static DccAdc adc(adc_gpio);

static DccCommand command(sig_gpio, pwr_gpio, adc);


void setup()
{
    Serial.begin(115200);

    SysLed::begin();
    SysLed::pattern(50, 950); // indicates "waiting for serial"

    while (!Serial)
        SysLed::loop();

    SysLed::off();

    Serial.printf("\n");
    Serial.printf("svc_reset\n");
    Serial.printf("\n");

    adc.log_reset();

    command.mode_svc_write_cv(8, 0x08);
}


void loop()
{
    static uint32_t start_ms = millis();
    static uint32_t time_ms;

    command.loop();

    static bool done = false;
    static bool result = false;
    static bool printed = false;

    if (!done) {
        done = command.svc_done(result);
        time_ms = millis() - start_ms;
    }

    if (millis() >= (start_ms + 1000) && !printed) {
        Serial.printf("reset %s in %lu msec\n",
                      result ? "succeeded" : "failed", time_ms);
        command.show_ack_ma();
        adc.log_show();
        printed = true;
    }
}
