#include <Arduino.h>
#include "xassert.h"
#include "sys_led.h"
#include "str_ops.h"
#include "dcc_config.h"
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
static bool loop_svc_address_read();
static bool loop_svc_address_write();

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
static void address_try();

static void cmd_help(bool verbose=false);
static void loco_help(bool verbose=false);
static void speed_help(bool verbose=false);
static void function_help(bool verbose=false);
static void track_help(bool verbose=false);
static void cv_help(bool verbose=false);
static void read_help(bool verbose=false);
static void write_help(bool verbose=false);
static void address_help(bool verbose=false);
static void print_help(bool verbose, const char *help_short,
                       int p_min, int p_max, const char *help_long);
static void print_help(bool verbose, const char *help_short, const char *help_long);
static void tab_over(int num_tokens, int to_column=12);

// DCC interface

static DccAdc adc(dcc_adc_gpio);
static DccCommand command(dcc_sig_gpio, dcc_pwr_gpio, adc);
static DccThrottle *throttle = nullptr;

// When reading/writing CVs, the cv_num_g is set in one command and the read or
// write command is in the next. Global statics are used to save them.

static int cv_num_g = DccPkt::cv_num_inv;
static int cv_val_g = DccPkt::cv_val_inv;

static int address_g = DccPkt::address_inv;

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

    if (dcc_slp_gpio >= 0) {
        gpio_init(dcc_slp_gpio);
        gpio_put(dcc_slp_gpio, 1);
        gpio_set_dir(dcc_slp_gpio, GPIO_OUT);
    }

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
    } else if (strcmp(tokens[0], "A") == 0) {
        address_try();
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
    // cv_num_g is global and is changed only if this command is successful

    if (tokens.count() < 2)
        return; // wait for another token

    int cv_num_new = DccPkt::cv_num_inv;
    if (!str_to_int(tokens[1], cv_num_new)) {
        tab_over(2);
        stream.printf("ERROR: \"%s\" not an integer\n", tokens[1]);
        tab_over(0);
        cv_help();
        // cv_num_g not changed
        // eat the "C" and unrecognized parameter
        tokens.eat(2);
        return;
    }

    if (DccPkt::cv_num_min <= cv_num_new && cv_num_new <= DccPkt::cv_num_max) {
        cv_num_g = cv_num_new;
        tab_over(2);
        stream.printf("OK: cv%d\n", cv_num_g);
    } else {
        tab_over(2);
        stream.printf("ERROR: out of range\n");
        tab_over(0);
        cv_help();
        // cv_num_g not changed
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
    // cv_num_g is global

    if (cv_num_g == DccPkt::cv_num_inv) {
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
    stream.printf("reading cv%d ...\n", cv_num_g);

    command.mode_svc_read_cv(cv_num_g);
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
    // cv_num_g and cv_val_g are global

    if (cv_num_g == DccPkt::cv_num_inv) {
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

    if (!str_to_int(tokens[1], cv_val_g)) {
        tab_over(2);
        stream.printf("ERROR: \"%s\" not an integer\n", tokens[1]);
        tab_over(0);
        write_help();
        // eat the "W" and unrecognized parameter
        tokens.eat(2);
        return;
    }

    if (DccPkt::cv_val_min <= cv_val_g && cv_val_g <= DccPkt::cv_val_max) {
        tab_over(2);
        stream.printf("write cv%d = %u (0x%02x)",
                      cv_num_g, uint(cv_val_g), uint(cv_val_g));
        // use svc mode if not already in ops mode
        if (command.mode() == DccCommand::MODE_OPS) {
            stream.printf(" in ops mode ...\n");
            throttle->write_cv(cv_num_g, cv_val_g);
        } else {
            xassert(command.mode() == DccCommand::MODE_OFF);
            stream.printf(" in svc mode ...\n");
            command.mode_svc_write_cv(cv_num_g, cv_val_g);
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

// All paths with expected output:
//
// C 8         OK: cv8
// W 8         write cv8 = 8 (0x08) in svc mode ...
//             OK: cv8 written with 8 (0x08) in 180 ms
// A R         read address ...
//             read cv29[5] ...
//             read cv29[5] = 0 in 179 ms
//             read cv1 = 3 (0x03) in 828 ms
//             OK: short address = 3
// A 66        set address to 66 ...
//             write cv1 = 66 (0x42) ...
//             cv1 written with 66 (0x42) in 180 ms
//             write cv29[5] = 0 ...
//             cv29 written with 0 (0x00) in 179 ms
//             OK: short address set to 66
// A R         read address ...
//             read cv29[5] ...
//             read cv29[5] = 0 in 180 ms
//             read cv1 = 66 (0x42) in 827 ms
//             OK: short address = 66
// A 9876      set address to 9876 ...
//             write cv18 = 148 (0x94) ...
//             cv18 written with 148 (0x94) in 179 ms
//             write cv17 = 230 (0xe6) ...
//             cv17 written with 230 (0xe6) in 180 ms
//             write cv29[5] = 1 ...
//             cv29 written with 1 (0x01) in 179 ms
//             OK: long address set to 9876
// A R         read address ...
//             read cv29[5] ...
//             read cv29[5] = 1 in 260 ms
//             read cv18 = 148 (0x94) in 822 ms
//             read cv17 = 230 (0xe6) in 826 ms
//             OK: long address = 9876
// A X         ERROR: "X" not "R" or an integer
//             A R
//             A <n>, 1 <= n <= 10239
// A 11000     ERROR: 11000 out of range
//             A R
//             A <n>, 1 <= n <= 10239
//
// Always two tokens; first is "A" and second is "R" to read or an integer to write
//
// Address is short if it is <= 127
// Address is long if it is > 127
//
// To write:
//  Here:
//      set address_g with address to write (long or short)
//      if short:
//          start write of address here:        cv_num_g = 1,  cv_val_g = address_g
//      if long:
//          start write of address_lo here:     cv_num_g = 18, cv_val_g = address_g & 0xff
//  In loop_svc_address_write:
//      if cv_num_g is 1 (address):
//          start clearing of cv29[5]:          cv_num_g = 29, cv_val_g = 0
//      else if cv_num_g is 18 (address_lo):
//          start write of address_hi:          cv_num_g = 17, cv_val_g = (address >> 8) | 0xc0
//      else if cv_num_g is 17 (address_hi):
//          start setting of cv29[5]:           cv_num_g = 29, cv_val_g = 1
//      else if cv_num_g is 29 (config):
//          done (success)
// To read:
//  Here:
//      start read of cv29[5]:                  cv_num_g = 29
//  In loop_svc_addres_read:
//      if cv_num_g is 29 (config):
//          if cv29[5]=0, start read of cv1:    cv_num_g = 1
//          if cv29[5]=1, start read of cv18:   cv_num_g = 18
//      else if cv_num_g is 1 (address):
//          done (success), address_g = value
//      else if cv_num_g is 18 (address_lo):
//          address_g = value
//          start read of cv17:                 cv_num_g = 17
//      else if cv_num_g is 17 (address_hi):
//          done (success), address_g = address_g | (value & 0x3f) << 8

static void address_try()
{
    // address_g is global

    if (tokens.count() < 2)
        return; // wait for another token

    bool read_address = (strcmp(tokens[1], "R") == 0);

    if (!read_address) {

        if (!str_to_int(tokens[1], address_g)) {
            tab_over(2);
            stream.printf("ERROR: \"%s\" not \"R\" or an integer\n", tokens[1]);
            address_help();
            // eat the "A" and unrecognized parameter
            tokens.eat(2);
            return;
        }

        if (address_g < DccPkt::address_min || address_g > DccPkt::address_max) {
            tab_over(2);
            stream.printf("ERROR: %d out of range\n", address_g);
            address_help();
            tokens.eat(2);
            return;
        }

    } // if (!read_address)

    if (command.mode() == DccCommand::MODE_OPS) {
        tab_over(2);
        stream.printf("ERROR: can't read or write address in ops mode (turn track off)\n");
        address_help();
        tokens.eat(2);
        return;
    }

    xassert(command.mode() == DccCommand::MODE_OFF);

    if (read_address) {
        tab_over(2);
        stream.printf("read address ...\n");
        // Read CV29, and bit 5 tells us if it's a short or long address
        // Bit 5 = 0: short address; read CV1
        // Bit 5 = 1: long address; read CV17 and CV18
        active = &loop_svc_address_read;
        cv_num_g = DccCv::config;
        tab_over(0);
        stream.printf("read cv%d[%d] ...\n", cv_num_g, 5);
        command.mode_svc_read_bit(cv_num_g, 5);
    } else {
        tab_over(2);
        stream.printf("set address to %d ...\n", address_g);
        // Short address is <= 127
        // Short address: write CV1, then clear CV29 bit 5
        // Long address: write CV18 and CV17, then set CV29 bit 5
        // Start the first write (CV1 or CV17) and let loop_svc_address_write
        // figure out what to do next (based on cv_num_g).
        if (address_g <= 127) {
            cv_num_g = DccCv::address;
            cv_val_g = address_g;
        } else {
            cv_num_g = DccCv::address_lo;
            cv_val_g = address_g & 0xff;
        }
        active = &loop_svc_address_write;
        tab_over(0);
        stream.printf("write cv%d = %d (0x%02x) ...\n", cv_num_g, uint(cv_val_g), uint(cv_val_g));
        command.mode_svc_write_cv(cv_num_g, cv_val_g);
    }

    start_ms = millis();

    tokens.eat(2);
}

static void address_help(bool verbose)
{
    tab_over(0);
    print_help(verbose, "A R", "read address (long or short)");
    tab_over(0);
    print_help(verbose, "A <n>", DccPkt::address_min, DccPkt::address_max,
               "write address (long or short)");
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
            stream.printf("OK: read cv%d = %u (0x%02x) in %u ms\n", cv_num_g,
                          uint(value), uint(value), millis() - start_ms);
        else
            stream.printf("ERROR reading cv%d in %u ms\n", cv_num_g, millis() - start_ms);
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
            stream.printf("OK: cv%d written with %u (0x%02x) in %u ms\n", cv_num_g,
                          uint(cv_val_g), uint(cv_val_g), millis() - start_ms);
        else
            stream.printf("ERROR writing cv%d with %u (0x%02x) in %u ms\n", cv_num_g,
                          uint(cv_val_g), uint(cv_val_g), millis() - start_ms);
        cv_val_g = DccPkt::cv_val_inv;
        return false; // done!
    } else {
        return true;
    }
}

//////////////////////////////////////////////////////////////////////////////

static bool loop_svc_address_read()
{
    bool result;
    uint8_t value;
    if (!command.svc_done(result, value))
        return true; // keep waiting

    if (!result) {
        tab_over(0);
        if (cv_num_g == DccCv::config)
            stream.printf("ERROR reading cv%d[5] in %u ms\n", cv_num_g, millis() - start_ms);
        else
            stream.printf("ERROR reading cv%d in %u ms\n", cv_num_g, millis() - start_ms);
        return false; // done!
    }

    if (cv_num_g == DccCv::config) {
        // done reading the config bit
        xassert(value == 0 || value == 1);
        tab_over(0);
        stream.printf("read cv%d[5] = %u in %u ms\n", cv_num_g,
                      uint(value), millis() - start_ms);
        if (value == 0)
            cv_num_g = DccCv::address; // short address, read it
        else
            cv_num_g = DccCv::address_lo; // long address, read lower half
        command.mode_svc_read_cv(cv_num_g);
        start_ms = millis();
        return true; // keep going
    } else {
        tab_over(0);
        stream.printf("read cv%d = %u (0x%02x) in %u ms\n", cv_num_g,
                      uint(value), uint(value), millis() - start_ms);
        if (cv_num_g == DccCv::address) {
            // short address, done
            address_g = uint(value);
            tab_over(0);
            stream.printf("OK: short address = %d\n", address_g);
            return false;
        } else if (cv_num_g == DccCv::address_lo) {
            // long address: save address_lo and read address_hi
            address_g = uint(value);
            cv_num_g = DccCv::address_hi;
            command.mode_svc_read_cv(cv_num_g);
            start_ms = millis();
            return true; // keep going
        } else {
            xassert(cv_num_g == DccCv::address_hi);
            // long address, done
            address_g |= (int(value & ~0xc0) << 8);
            tab_over(0);
            stream.printf("OK: long address = %d\n", address_g);
            return false;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////

static bool loop_svc_address_write()
{
    bool result;
    if (!command.svc_done(result))
        return true; // keep waiting

    // one of the writes has finished

    xassert(DccPkt::address_min <= address_g && address_g <= DccPkt::address_max);
    xassert(cv_num_g == DccCv::address ||
            cv_num_g == DccCv::address_lo || cv_num_g == DccCv::address_hi ||
            cv_num_g == DccCv::config);

    if (!result) {
        tab_over(0);
        if (cv_num_g == DccCv::config)
            stream.printf("ERROR writing cv%d[5] with %u in %u ms\n",
                          cv_num_g, uint(cv_val_g), millis() - start_ms);
        else
            stream.printf("ERROR writing cv%d with %u (0x%02x) in %u ms\n",
                          cv_num_g, uint(cv_val_g), uint(cv_val_g),
                          millis() - start_ms);
        return false; // done!
    }

    tab_over(0);
    stream.printf("cv%d written with %u (0x%02x) in %u ms\n", cv_num_g,
                  uint(cv_val_g), uint(cv_val_g), millis() - start_ms);

    tab_over(0);
    if (cv_num_g == DccCv::address) {
        // short address, and just wrote CV1 (address)
        // clear CV29 bit 5
        cv_num_g = DccCv::config;
        cv_val_g = 0; // only used for an error message if needed
        stream.printf("write cv%d[%d] = %d ...\n", cv_num_g, 5, 0);
        command.mode_svc_write_bit(cv_num_g, 5, 0);
        start_ms = millis();
        return true; // keep going
    } else if (cv_num_g == DccCv::address_lo) {
        cv_num_g = DccCv::address_hi;
        cv_val_g = (address_g >> 8) | 0xc0;
        stream.printf("write cv%d = %d (0x%02x) ...\n", cv_num_g,
                      uint(cv_val_g), uint(cv_val_g));
        command.mode_svc_write_cv(cv_num_g, cv_val_g);
        start_ms = millis();
        return true; // keep going
    } else if (cv_num_g == DccCv::address_hi) {
        // set CV29 bit 5
        cv_num_g = DccCv::config;
        cv_val_g = 1; // only used for an error message if needed
        stream.printf("write cv%d[%d] = %d ...\n", cv_num_g, 5, 1);
        command.mode_svc_write_bit(cv_num_g, 5, 1);
        start_ms = millis();
        return true; // keep going
    } else {
        xassert(cv_num_g == DccCv::config);
    }

    cv_num_g = DccPkt::cv_num_inv;
    cv_val_g = DccPkt::cv_val_inv;

    if (address_g <= 127)
        stream.printf("OK: short address set to %d\n", address_g);
    else
        stream.printf("OK: long address set to %d\n", address_g);

    return false; // done!
}
