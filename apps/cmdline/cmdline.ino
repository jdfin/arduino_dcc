#include <Arduino.h>
#include "xassert.h"
#include "sys_led.h"
#include "str_ops.h"
#include "dcc_adc.h"
#include "dcc_pkt.h"
#include "dcc_throttle.h"
#include "dcc_command.h"
#include "tokens.h"

// Stream used for both input (commands/parameters) and output messages.

static Stream& stream = Serial;

// Some commands, mainly service-mode reads and writes, take a while (a few
// hundred msec) to complete. When one of these is started, a function pointer
// is set to poll for progress/completion. Each time through the main loop()
// function, the "active" function is called; it is most often the no-op
// function, but is set to some other progress function when needed.

typedef bool (loop_func)();

static bool loop_nop();
static bool loop_svc_cv_read();
static bool loop_svc_cv_write();

static loop_func *active = &loop_nop;

// Console input is a stream of characters, which is broken up into a stream
// of tokens separated by whitespace. A token is not looked at until the
// following whitespace (typically space or cr/lf) is seen.
//
// The "Tokens" object takes a character at a time and builds an array of
// tokens, which can then be examined for commands and parameters.

static Tokens tokens(stream);

// These functions look at tokens and see if there are any complete commands
// to process.

static void cmd_try();
static void loco_try();
static void speed_try();
static void function_try();
static void track_try();
static void cv_try();
static void read_try();
static void write_try();

static void cmd_help(bool verbose=false);
static void loco_help(bool verbose=false);
static void speed_help(bool verbose=false);
static void function_help(bool verbose=false);
static void track_help(bool verbose=false);
static void cv_help(bool verbose=false);
static void read_help(bool verbose=false);
static void write_help(bool verbose=false);
static void print_help(bool verbose, const char *help_short,
                       int p_min, int p_max, const char *help_long);
static void print_help(bool verbose, const char *help_short, const char *help_long);
static void tab_over(int num_tokens, int to_column=12);

// DCC interface.

static const int dcc_sig_gpio = 17;
static const int dcc_pwr_gpio = 16;
static const int dcc_adc_gpio = 26; // GPIO 26 is ADC 0

static DccAdc adc(dcc_adc_gpio);
static DccCommand command(dcc_sig_gpio, dcc_pwr_gpio, adc);
static DccThrottle *throttle = nullptr;

// When reading/writing CVs, the cv_num is set in one command and the read or
// write command is in the next. Global statics are used to save them.

static int cv_num = DccPkt::cv_num_inv;
static int cv_val = DccPkt::cv_val_inv;

// The start time of a long operation (read or write in service mode) is saved
// so the overall time can be printed.

static uint32_t start_ms = 0;

// This prints a friendly message with the current command station mode, used
// when an operation is done that works but might not have any immediate
// effect (like changing the loco speed when the track is not powered on).

static const char *mode_info(DccCommand& command);

//////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(115200);
    SysLed::begin();
    SysLed::pattern(50, 950); // indicates "waiting for serial"
    while (!Serial)
        SysLed::loop();
    SysLed::off();

    adc.log_reset(); // logging must be enabled at compile time in dcc_adc.h

    tokens.reset();

    throttle = command.create_throttle(); // default address 3

    stream.printf("\n");
    cmd_help(true);
    stream.printf("\n");
}

//////////////////////////////////////////////////////////////////////////////

void loop()
{
    command.loop();

    // If any command is ongoing, let it try to progress.
    if (!(*active)())
        active = &loop_nop; // loop_* returning false means it's done

    // Consume input if available. This might result in a new token.
    if (stream.available() > 0) {
        char c = stream.read();
        //stream.printf("got 0x%02x\n", uint(c));
        tokens.add_char(c);
    }

    // Don't call command_try to start a new command if one is ongoing.
    if (active != &loop_nop)
        return;

    cmd_try();
}

//////////////////////////////////////////////////////////////////////////////

static void cmd_try()
{
    if (tokens.count() == 0)
        return;

    if (strcmp(tokens[0], "L") == 0) {
        loco_try();
    } else if (strcmp(tokens[0], "S") == 0) {
        speed_try();
    } else if (strcmp(tokens[0], "F") == 0) {
        function_try();
    } else if (strcmp(tokens[0], "T") == 0) {
        track_try();
    } else if (strcmp(tokens[0], "C") == 0) {
        cv_try();
    } else if (strcmp(tokens[0], "R") == 0) {
        read_try();
    } else if (strcmp(tokens[0], "W") == 0) {
        write_try();
    } else {
        // tokens[0] unrecognized
        tab_over(1);
        stream.printf("\"%s\" unrecognized as command\n", tokens[0]);
        tokens.eat(1);
    }
}

static void cmd_help(bool verbose)
{
    loco_help(verbose);
    speed_help(verbose);
    function_help(verbose);
    track_help(verbose);
    cv_help(verbose);
    read_help(verbose);
    write_help(verbose);
}

//////////////////////////////////////////////////////////////////////////////

// All paths with expected output:
//
// L X         ERROR: "X" not an integer
//             L <n>, 1 <= n <= 10239
// L 99999     ERROR: "99999" out of range
//             L <n>, 1 <= n <= 10239
// L 9999      OK: loco 9999

static void loco_try()
{
    if (tokens.count() < 2)
        return; // wait for another token

    int loco;
    if (!str_to_int(tokens[1], loco)) {
        tab_over(2);
        stream.printf("ERROR: \"%s\" not an integer\n", tokens[1]);
        tab_over(0);
        loco_help();
        // eat the "L" and unrecognized parameter
        tokens.eat(2);
        return;
    }

    if (DccPkt::address_min <= loco && loco <= DccPkt::address_max) {
        throttle->address(loco);
        tab_over(2);
        stream.printf("OK: loco %d\n", loco);
    } else {
        tab_over(2);
        stream.printf("ERROR: \"%s\" out of range\n", tokens[1]);
        tab_over(0);
        loco_help();
    }

    tokens.eat(2);
}

static void loco_help(bool verbose)
{
    print_help(verbose, "L <n>",
               DccPkt::address_min, DccPkt::address_max,
               "set loco for subsequent operations");
}

//////////////////////////////////////////////////////////////////////////////

// All paths with expected output:
//
// S X         ERROR: "X" not an integer
//             S <n>, -127 <= n <= 127
// S 200       ERROR: "200" out of range
//             S <n>, -127 <= n <= 127
// S 10        OK: speed 10
//             NOTE: command station is powered off
// T ON        OK: track on
// S 10        OK: speed 10

static void speed_try()
{
    if (tokens.count() < 2)
        return; // wait for another token

    int speed;
    if (!str_to_int(tokens[1], speed)) {
        tab_over(2);
        stream.printf("ERROR: \"%s\" not an integer\n", tokens[1]);
        tab_over(0);
        speed_help();
        // eat the "S" and unrecognized parameter
        tokens.eat(2);
        return;
    }

    if (DccPkt::speed_min <= speed && speed <= DccPkt::speed_max) {
        throttle->speed(speed);
        tab_over(2);
        stream.printf("OK: speed %d\n", speed);
        if (command.mode() != DccCommand::MODE_OPS) {
            tab_over(0);
            stream.printf("NOTE: %s\n", mode_info(command));
        }
    } else {
        tab_over(2);
        stream.printf("ERROR: \"%s\" out of range\n", tokens[1]);
        tab_over(0);
        speed_help();
    }

    tokens.eat(2);
}

static void speed_help(bool verbose)
{
    print_help(verbose, "S <n>",
               DccPkt::speed_min, DccPkt::speed_max,
               "set speed for current loco");
}

//////////////////////////////////////////////////////////////////////////////

// All paths with expected output:
//
// F X         ERROR: "X" not an integer
//             F <n> ON|OFF, 0 <= n <= 28
// F 33        ERROR: "33" out of range
//             F <n> ON|OFF, 0 <= n <= 28
// F 20 X      ERROR: "X" unrecognized
//             F <n> ON|OFF, 0 <= n <= 28
// F 20 ON     OK: f20 on
//             NOTE: command station is powered off
// F 20 OFF    OK: f20 off
//             NOTE: command station is powered off
// T ON        OK: track on
// F 20 ON     OK: f20 on
// F 20 OFF    OK: f20 off

static void function_try()
{
    if (tokens.count() < 2)
        return; // wait for another token

    // have at least the function number
    int func;
    if (!str_to_int(tokens[1], func)) {
        tab_over(2);
        stream.printf("ERROR: \"%s\" not an integer\n", tokens[1]);
        tab_over(0);
        function_help();
        // eat the "F" and unrecognized parameter
        tokens.eat(2);
        return;
    }

    if (func < DccPkt::function_min || func > DccPkt::function_max) {
        tab_over(2);
        stream.printf("ERROR: \"%s\" out of range\n", tokens[1]);
        tab_over(0);
        function_help();
        // eat the "F" and invalid function number
        tokens.eat(2);
        return;
    }

    if (tokens.count() < 3)
        return; // wait for another token

    if (strcmp(tokens[2], "ON") == 0) {
        throttle->function(func, true);
        tab_over(3);
        stream.printf("OK: f%d on\n", func);
        if (command.mode() != DccCommand::MODE_OPS) {
            tab_over(0);
            stream.printf("NOTE: %s\n", mode_info(command));
        }
    } else if (strcmp(tokens[2], "OFF") == 0) {
        throttle->function(func, false);
        tab_over(3);
        stream.printf("OK: f%d off\n", func);
        if (command.mode() != DccCommand::MODE_OPS) {
            tab_over(0);
            stream.printf("NOTE: %s\n", mode_info(command));
        }
    } else {
        tab_over(3);
        stream.printf("ERROR: \"%s\" unrecognized\n", tokens[2]);
        tab_over(0);
        function_help();
    }

    tokens.eat(3);
}

static void function_help(bool verbose)
{
    print_help(verbose, "F <n> ON|OFF",
               DccPkt::function_min, DccPkt::function_max,
               "set a function for current loco on/off");
}

//////////////////////////////////////////////////////////////////////////////

// All paths with expected output:
//
// T X         ERROR: "X" unrecognized
//             T ON|OFF
// T OFF       ERROR: can't turn track off
//             command station is powered off
// T ON        OK: track on
// T ON        ERROR: can't turn track on
//             command station is powered on
// T OFF       OK: track off

static void track_try()
{
    if (tokens.count() < 2)
        return; // wait for another token

    if (strcmp(tokens[1], "ON") == 0) {
        if (command.mode() == DccCommand::MODE_OFF) {
            command.mode_ops();
            tab_over(2);
            stream.printf("OK: track on\n");
        } else {
            tab_over(2);
            stream.printf("ERROR: can't turn track on\n");
            tab_over(0);
            stream.printf("%s\n", mode_info(command));
        }
    } else if (strcmp(tokens[1], "OFF") == 0) {
        if (command.mode() == DccCommand::MODE_OPS) {
            command.mode_off();
            tab_over(2);
            stream.printf("OK: track off\n");
        } else {
            tab_over(2);
            stream.printf("ERROR: can't turn track off\n");
            tab_over(0);
            stream.printf("%s\n", mode_info(command));
        }
    } else {
        tab_over(2);
        stream.printf("ERROR: \"%s\" unrecognized\n", tokens[1]);
        tab_over(0);
        track_help();
    }

    tokens.eat(2);
}

static void track_help(bool verbose)
{
    print_help(verbose, "T ON|OFF",
               "turn track power on/off");
}

//////////////////////////////////////////////////////////////////////////////

// All paths with expected output:
//
// C X         ERROR: "X" not an integer
//             C <n>, 1 <= n <= 1024
// C 2000      ERROR: out of range
//             C <n>, 1 <= n <= 1024
// C 8         OK: cv8

static void cv_try()
{
    // cv_num is global and is changed only if this command is successful

    if (tokens.count() < 2)
        return; // wait for another token

    int cv_num_new = DccPkt::cv_num_inv;
    if (!str_to_int(tokens[1], cv_num_new)) {
        tab_over(2);
        stream.printf("ERROR: \"%s\" not an integer\n", tokens[1]);
        tab_over(0);
        cv_help();
        // cv_num not changed
        // eat the "C" and unrecognized parameter
        tokens.eat(2);
        return;
    }

    if (DccPkt::cv_num_min <= cv_num_new && cv_num_new <= DccPkt::cv_num_max) {
        cv_num = cv_num_new;
        tab_over(2);
        stream.printf("OK: cv%d\n", cv_num);
    } else {
        tab_over(2);
        stream.printf("ERROR: out of range\n");
        tab_over(0);
        cv_help();
        // cv_num not changed
    }

    tokens.eat(2);
}

static void cv_help(bool verbose)
{
    print_help(verbose, "C <n>",
               DccPkt::cv_num_min, DccPkt::cv_num_max,
               "set cv number for subsequent operations");
}

//////////////////////////////////////////////////////////////////////////////

// All paths with expected output:
// R           ERROR: set cv number before 'R' command
//                    "C <n> R", 1 <= n <= 1024
// C 8         OK: cv8
// R           OK: reading cv8
//             read cv8 = 101 (0x65) in 816 ms
// T ON        OK: track on
// R           ERROR: track must be off to read a cv in service mode

static void read_try()
{
    // cv_num is global

    if (cv_num == DccPkt::cv_num_inv) {
        tab_over(1);
        stream.printf("ERROR: set cv number before 'R' command\n");
        tab_over(0);
        stream.printf("       \"C <n> R\", %d <= n <= %d\n",
                      DccPkt::cv_num_min, DccPkt::cv_num_max);
        tokens.eat(1);
        return;
    }

    if (command.mode() != DccCommand::MODE_OFF) {
        tab_over(1);
        stream.printf("ERROR: track must be off to read a cv in service mode\n");
        tokens.eat(1);
        return;
    }

    tab_over(1);
    stream.printf("OK: reading cv%d\n", cv_num);

    command.mode_svc_read_cv(cv_num);
    active = &loop_svc_cv_read;
    start_ms = millis();
    tokens.eat(1);
}

static void read_help(bool verbose)
{
    print_help(verbose, "R",
               "read current cv (previously set with C)");
}

//////////////////////////////////////////////////////////////////////////////

// All paths with expected output:
//
// W           ERROR: set cv number before 'W' command
//                    "C <n> W <v>", 1 <= n <= 1024, -127 <= v <= 255
// C 8         OK: cv8
// W X         ERROR: "X" not an integer
//             W <n>, -127 <= n <= 255
// W 500       ERROR: out of range
//             W <n>, -127 <= n <= 255
// W 8         OK: write cv8 = 8 (0x08) in svc mode
//             cv8 written with 8 (0x08) in 184 ms
// T ON        OK: track on
// W 8         OK: write cv8 = 8 (0x08) in ops mode

static void write_try()
{
    // cv_num and cv_val are global

    if (cv_num == DccPkt::cv_num_inv) {
        tab_over(1);
        stream.printf("ERROR: set cv number before 'W' command\n");
        tab_over(0);
        stream.printf("       \"C <n> W <v>\", %d <= n <= %d, %d <= v <= %d\n",
                      DccPkt::cv_num_min, DccPkt::cv_num_max,
                      DccPkt::cv_val_min, DccPkt::cv_val_max);
        tokens.eat(1);
        return;
    }

    if (tokens.count() < 2)
        return; // wait for another token

    if (!str_to_int(tokens[1], cv_val)) {
        tab_over(2);
        stream.printf("ERROR: \"%s\" not an integer\n", tokens[1]);
        tab_over(0);
        write_help();
        // eat the "W" and unrecognized parameter
        tokens.eat(2);
        return;
    }

    if (DccPkt::cv_val_min <= cv_val && cv_val <= DccPkt::cv_val_max) {
        tab_over(2);
        stream.printf("OK: write cv%d = %u (0x%02x)",
                      cv_num, uint(cv_val), uint(cv_val));
        // use svc mode if not already in ops mode
        if (command.mode() == DccCommand::MODE_OPS) {
            stream.printf(" in ops mode\n");
            throttle->write_cv(cv_num, cv_val);
        } else {
            xassert(command.mode() == DccCommand::MODE_OFF);
            stream.printf(" in svc mode\n");
            command.mode_svc_write_cv(cv_num, cv_val);
            active = &loop_svc_cv_write;
            start_ms = millis();
        }
    } else {
        tab_over(2);
        stream.printf("ERROR: out of range\n");
        tab_over(0);
        write_help();
    }

    tokens.eat(2);
}

static void write_help(bool verbose)
{
    print_help(verbose, "W <n>",
               DccPkt::cv_val_min, DccPkt::cv_val_max,
               "write current cv (previously set with C)");
}

//////////////////////////////////////////////////////////////////////////////

static void print_help(bool verbose, const char *help_short,
                       int p_min, int p_max, const char *help_long)
{
    int n;
    stream.printf("%s%n", help_short, &n);

    if (verbose)
        stream.printf("%*s", 16 - n, "");
    else if (p_min < p_max)
        stream.printf(", ");

    if (p_min < p_max)
        stream.printf("%d <= n <= %d%n", p_min, p_max, &n);
    else
        n = 0;

    if (verbose)
        stream.printf("%*s%s", 20 - n, "", help_long);

    stream.printf("\n");
}

static void print_help(bool verbose, const char *help_short, const char *help_long)
{
    print_help(verbose, help_short, 0, 0, help_long);
}

static void tab_over(int num_tokens, int to_column)
{
    xassert(num_tokens >= 0);
    xassert(to_column >= 0);

    // subtract characters already printed from to_column

    // spaces between tokens
    to_column -= num_tokens;

    // tokens
    for (int i = 0; i < num_tokens; i++)
        to_column -= strlen(tokens[i]);

    // go the rest of the way
    if (to_column > 0)
        stream.printf("%*s", to_column, "");
}

//////////////////////////////////////////////////////////////////////////////

static const char *mode_info(DccCommand& command)
{
    if (command.mode() == DccCommand::MODE_OFF)
        return "command station is powered off";
    else if (command.mode() == DccCommand::MODE_OPS)
        return "command station is powered on";
    else if (command.mode() == DccCommand::MODE_SVC_WRITE_CV)
        return "command station is writing a cv in service mode";
    else if (command.mode() == DccCommand::MODE_SVC_READ_CV)
        return "command station is reading a cv in service mode";
    else
        return "command station is in an unknown mode";
}

//////////////////////////////////////////////////////////////////////////////

// the "active" function when nothing needs doing
static bool loop_nop()
{
    return false;
}

//////////////////////////////////////////////////////////////////////////////

static bool loop_svc_cv_read()
{
    bool result;
    uint8_t value;
    if (command.svc_done(result, value)) {
        tab_over(0);
        if (result)
            stream.printf("read cv%d = %u (0x%02x) in %u ms\n", cv_num,
                          uint(value), uint(value), millis() - start_ms);
        else
            stream.printf("ERROR reading cv%d in %u ms\n", cv_num, millis() - start_ms);
        return false; // done!
    } else {
        return true;
    }
}

//////////////////////////////////////////////////////////////////////////////

static bool loop_svc_cv_write()
{
    bool result;
    if (command.svc_done(result)) {
        tab_over(0);
        if (result)
            stream.printf("cv%d written with %u (0x%02x) in %u ms\n", cv_num,
                          uint(cv_val), uint(cv_val), millis() - start_ms);
        else
            stream.printf("ERROR writing cv%d with %u (0x%02x) in %u ms\n", cv_num,
                          uint(cv_val), uint(cv_val), millis() - start_ms);
        cv_val = DccPkt::cv_val_inv;
        return false; // done!
    } else {
        return true;
    }
}
