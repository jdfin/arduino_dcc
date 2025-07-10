#pragma once

#if (defined ARDUINO_RASPBERRY_PI_PICO)

// drives DCC

#if 0
// breadboard
static const int dcc_signal_gpio = 17;  // PH
static const int dcc_power_gpio = 16;   // EN
static const int dcc_sleep_gpio = -1;   // SLP
static const int dcc_current_gpio = 26; // CS
#else
// engine house
static const int dcc_signal_gpio = 27;  // PH
static const int dcc_power_gpio = 28;   // EN
static const int dcc_sleep_gpio = 22;   // SLP
static const int dcc_current_gpio = 26; // CS
#endif

#elif (defined ARDUINO_PIMORONI_TINY2040)

// reads/decodes DCC

static const int dcc_signal_gpio = 7;

#else

#error Unknown board!

#endif
