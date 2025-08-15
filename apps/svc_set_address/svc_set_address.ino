#include <Arduino.h>
#include "xassert.h"
#include "sys_led.h"
#include "dcc_config.h"
#include "dcc_adc.h"
#include "dcc_pkt.h"
#include "dcc_throttle.h"
#include "dcc_command.h"

static DccAdc adc(dcc_adc_gpio);
static DccCommand command(dcc_sig_gpio, dcc_pwr_gpio, adc);

static int address = 2265;

static int verbosity = 1;

// step:
//  0  read CV1
//  1  read CV17
//  2  read CV18
//  3  read CV29
// 10  write CV1
// 11  write CV17
// 12  write CV18
// 13  write bit CV29
// 20  read CV1
// 21  read CV17
// 22  read CV18
// 23  read CV29
static int step = 0;

static void loop_short();
static void loop_long();


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

    Serial.printf("setting address to %d\n", address);
}


void loop()
{
    command.loop();

    bool result;
    uint8_t cv_val;

    if (step == 0) {
        if (verbosity == 0) {
            step = 10;
        } else {
            command.mode_svc_read_cv(DccCv::address);
            step = 1;
        }
    } else if (step == 1) {
        if (command.svc_done(result, cv_val)) {
            if (!result) {
                Serial.printf("ERROR reading CV%d\n", DccCv::address);
                step = 99;
            } else {
                Serial.printf("CV1%d 0x%02x\n", DccCv::address, uint(cv_val));
                command.mode_svc_read_cv(DccCv::address_hi);
                step = 2;
            }
        }
    } else if (step == 2) {
        if (command.svc_done(result, cv_val)) {
            if (!result) {
                Serial.printf("ERROR reading CV%d\n", DccCv::address_hi);
                step = 99;
            } else {
                Serial.printf("CV%d: 0x%02x\n", DccCv::address_hi, uint(cv_val));
                command.mode_svc_read_cv(DccCv::address_lo);
                step = 3;
            }
        }
    } else if (step == 3) {
        if (command.svc_done(result, cv_val)) {
            if (!result) {
                Serial.printf("ERROR reading CV%d\n", DccCv::address_lo);
                step = 99;
            } else {
                Serial.printf("CV%d: 0x%02x\n", DccCv::address_lo, uint(cv_val));
                command.mode_svc_read_cv(DccCv::config);
                step = 4;
            }
        }
    } else if (step == 4) {
        if (command.svc_done(result, cv_val)) {
            if (!result) {
                Serial.printf("ERROR reading CV%d\n", DccCv::config);
                step = 99;
            } else {
                Serial.printf("CV%d: 0x%02x\n", DccCv::config, uint(cv_val));
                step = 10;
            }
        }
    } else if (10 <= step && step <= 19) {
        if (address <= DccPkt::address_short_max) {
            loop_short();
        } else {
            loop_long();
        }
    } else if (step == 20) {
        if (verbosity == 0) {
            step = 99;
        } else {
            command.mode_svc_read_cv(DccCv::address);
            step = 21;
        }
    } else if (step == 21) {
        if (command.svc_done(result, cv_val)) {
            if (!result) {
                Serial.printf("ERROR reading CV%d\n", DccCv::address);
                step = 99;
            } else {
                Serial.printf("CV1%d 0x%02x\n", DccCv::address, uint(cv_val));
                command.mode_svc_read_cv(DccCv::address_hi);
                step = 22;
            }
        }
    } else if (step == 22) {
        if (command.svc_done(result, cv_val)) {
            if (!result) {
                Serial.printf("ERROR reading CV%d\n", DccCv::address_hi);
                step = 99;
            } else {
                Serial.printf("CV%d: 0x%02x\n", DccCv::address_hi, uint(cv_val));
                command.mode_svc_read_cv(DccCv::address_lo);
                step = 23;
            }
        }
    } else if (step == 23) {
        if (command.svc_done(result, cv_val)) {
            if (!result) {
                Serial.printf("ERROR reading CV%d\n", DccCv::address_lo);
                step = 99;
            } else {
                Serial.printf("CV%d: 0x%02x\n", DccCv::address_lo, uint(cv_val));
                command.mode_svc_read_cv(DccCv::config);
                step = 24;
            }
        }
    } else if (step == 24) {
        if (command.svc_done(result, cv_val)) {
            if (!result) {
                Serial.printf("ERROR reading CV%d\n", DccCv::config);
                step = 99;
            } else {
                Serial.printf("CV%d: 0x%02x\n", DccCv::config, uint(cv_val));
                step = 99;
            }
        }
    } else if (step == 99) {
        Serial.printf("Done.\n");
        step = 100;
        adc.log_show();
    }
}


static void loop_short()
{

    bool result;

    if (step == 10) {
        command.mode_svc_write_cv(DccCv::address, address);
        step = 11;
    } else if (step == 11) {
        // waiting for write to CV1 to finish
        if (command.svc_done(result)) {
            if (!result) {
                Serial.printf("ERROR writing CV%d\n", DccCv::address);
                step = 99;
            } else {
                command.mode_svc_write_bit(DccCv::config, 5, 0);
                step = 12;
            }
        }
    } else if (step == 12) {
        // waiting for write of CV29 bit 5 to finish
        if (command.svc_done(result)) {
            if (!result) {
                Serial.printf("ERROR writing CV%d bit 5\n", DccCv::config);
                step = 99;
            } else {
                step = 20;
            }
        }
    }
}


static void loop_long()
{
    bool result;

    if (step == 10) {
        Serial.printf("setting address...\n");
        command.mode_svc_write_cv(DccCv::address_lo, address & 0xff);
        step = 11;
    } else if (step == 11) {
        // waiting for write to CV18 to finish
        if (command.svc_done(result)) {
            if (!result) {
                Serial.printf("ERROR writing CV%d\n", DccCv::address_lo);
                step = 99;
            } else {
                command.mode_svc_write_cv(DccCv::address_hi, (address >> 8) | 0xc0);
                step = 12;
            }
        }
    } else if (step == 12) {
        // waiting for write to CV17 to finish
        if (command.svc_done(result)) {
            if (!result) {
                Serial.printf("ERROR writing CV%d\n", DccCv::address_hi);
                step = 99;
            } else {
                command.mode_svc_write_bit(DccCv::config, 5, 1);
                step = 13;
            }
        }
    } else if (step == 13) {
        // waiting for write of CV29 bit 5 to finish
        if (command.svc_done(result)) {
            if (!result) {
                Serial.printf("ERROR writing CV%d bit 5\n", DccCv::config);
                step = 99;
            } else {
                step = 20;
            }
        }
    }
}
