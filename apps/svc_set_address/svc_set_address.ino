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

static uint address = 1734;


void setup()
{
    Serial.begin(115200);

    SysLed::begin();
    SysLed::pattern(50, 950); // indicates "waiting for serial"

    while (!Serial)
        SysLed::loop();

    SysLed::off();

    Serial.printf("\n");
    Serial.printf("svc_set_address\n");
    Serial.printf("\n");

    adc.log_reset();

    Serial.printf("setting address to %u\n", address);
}


void loop()
{
    command.loop();

    static uint step = 0;

    bool result;

    if (step == 0) {
        command.mode_svc_write_cv(18, address & 0xff);
        step = 1;
    } else if (step == 1) {
        // waiting for write to CV18 to finish
        if (command.svc_done(result)) {
            if (!result) {
                Serial.printf("ERROR writing CV18\n");
                step = 99;
            } else {
                command.mode_svc_write_cv(17, (address >> 8) | 0xc0);
                step = 2;
            }
        }
    } else if (step == 2) {
        // waiting for write to CV17 to finish
        if (command.svc_done(result)) {
            if (!result) {
                Serial.printf("ERROR writing CV17\n");
                step = 99;
            } else {
                command.mode_svc_read_cv(29);
                step = 3;
            }
        }
    } else if (step == 3) {
        // waiting for read of CV29 to finish
        uint8_t cv29;
        if (command.svc_done(result, cv29)) {
            if (!result) {
                Serial.printf("ERROR reading CV29\n");
                step = 99;
            } else {
                cv29 |= 0x20;
                command.mode_svc_write_cv(29, cv29);
                step = 4;
            }
        }
    } else if (step == 4) {
        // waiting for write to CV29 to finish
        if (command.svc_done(result)) {
            if (!result) {
                Serial.printf("ERROR writing CV29\n");
                step = 99;
            } else {
                Serial.printf("Done!\n");
                step = 5;
            }
        }
    }
}
