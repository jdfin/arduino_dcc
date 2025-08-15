#pragma once

#if (defined ARDUINO_RASPBERRY_PI_PICO)

// drives DCC

#if 0
// breadboard
static const int dcc_sig_gpio = 17; // PH
static const int dcc_pwr_gpio = 16; // EN
static const int dcc_slp_gpio = -1; // SLP
static const int dcc_adc_gpio = 26; // CS (ADC0)
#else
// engine house
static const int dcc_sig_gpio = 27; // PH
static const int dcc_pwr_gpio = 28; // EN
static const int dcc_slp_gpio = 22; // SLP
static const int dcc_adc_gpio = 26; // CS (ADC0)
#endif

#elif (defined ARDUINO_PIMORONI_TINY2040)

// reads/decodes DCC

static const int dcc_sig_gpio = 7;

#else

#error Unknown board!

#endif
