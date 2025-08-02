#include <Arduino.h>
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "dbg_gpio.h"
#include "pwm_irq_mux.h"

#include "dcc_pkt.h"
#include "dcc_bitstream.h"

// PWM usage:
//
// Example: sending 0, 1, 1
//
//   |<--------0-------->|<----1---->|<----1---->|
//
//   +---------+         +-----+     +-----+     +--
//   |         |         |     |     |     |     |
// --+         +---------+     +-----+     +-----+
//   ^                   ^           ^           ^
//   A                   B           C           D
//
// At edge A, the PWM's CC and TOP registers are already programmed for the
// zero bit (done at the start of the bit ending at A). The interrupt handler
// called because of the wrap at edge A programs CC and TOP for the one bit
// that will start at edge B. Because of the double-buffering in CC and TOP,
// those values are not used until edge B.
//
// At edge B, the PWM's CC and TOP registers start using the values set at
// edge A. The handler called because of the wrap at edge B programs CC and
// TOP for the one bit starting at edge C.


DccBitstream::DccBitstream(int sig_gpio, int pwr_gpio) :
    _pwr_gpio(pwr_gpio),
    _pkt_idle(),
    _pkt_reset(),
    _pkt_a(),
    _pkt_b(),
    _current(&_pkt_idle),
    _next(&_pkt_idle),
    _preamble_bits(DccPkt::ops_preamble_bits),
    _slice(pwm_gpio_to_slice_num(sig_gpio)),
    _channel(pwm_gpio_to_channel(sig_gpio)),
    _byte(INT_MAX), // set in start_*()
    _bit(INT_MAX)   // set in start_*()
{
    DbgGpio::init({0});

    // Do not do PWM setup here since this might be a static object, and
    // other stuff is not fully initialized. In particular, clock_get_hz()
    // might return the wrong value (125 MHz, should be 200 MHz). Getting
    // _slice and _channel in the initialization above is okay since that
    // does not require anything else to be initialized.

    gpio_set_function(sig_gpio, GPIO_FUNC_PWM);

    // track power off
    gpio_init(_pwr_gpio);
    power(false);
    gpio_set_dir(_pwr_gpio, GPIO_OUT);
}


DccBitstream::~DccBitstream()
{
    stop(); // track power off, pwm output low
}


void DccBitstream::power(bool on)
{
    gpio_put(_pwr_gpio, on ? 1 : 0);
}


void DccBitstream::start_ops()
{
    start(DccPkt::ops_preamble_bits, _pkt_idle);
}


void DccBitstream::start_svc()
{
    start(DccPkt::svc_preamble_bits, _pkt_reset);
}


void DccBitstream::start(uint preamble_bits, DccPkt& first)
{
    uint sys_hz = clock_get_hz(clk_sys);
    const uint pwm_hz = 1000000; // 1 MHz; 1 usec/count
    uint32_t pwm_div = sys_hz / pwm_hz;

    // If this is a start after a previous stop, the pwm is not disabled,
    // it's just running a 0% duty cycle waveform.
    pwm_set_enabled(_slice, false);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, pwm_div);
    pwm_init(_slice, &config, false);

    // RP2040 has one pwm with interrupt number PWM_IRQ_WRAP.
    // RP2350 has two pwms with interrupt numbers PWM_IRQ_WRAP_[01],
    // and PWM_IRQ_WRAP is PWM_IRQ_WRAP_0.
    pwm_irq_mux_connect(_slice, pwm_handler, this);
    pwm_clear_irq(_slice);
    pwm_set_irq_enabled(_slice, true);

    _preamble_bits = preamble_bits;

    _current = &first;
    _next = &_pkt_idle;

    _byte = -1;                 // sending preamble
    _bit = _preamble_bits - 1;  // there's no previous packet, so no
                                // stop bit as part of the preamble;
                                // set _bit to len-1

    power(true);                // track power on

    next_bit();

    pwm_set_enabled(_slice, true);

    // The first bit has started going out. Program for second bit when
    // first bit finishes done. This takes advantage of the RP2040's
    // double-buffering of TOP and LEVEL.
    next_bit();
}


void DccBitstream::stop()
{
    power(false);               // track power off
    pwm_set_irq_enabled(_slice, false);
    // stop with output low (0% duty)
    pwm_set_chan_level(_slice, _channel, 0);
    // Let the pwm keep running so it gets to the end of the current bit and
    // switches to the 0% duty cycle. If the bitstream starts again, it'll be
    // disabled while it is initialized.
}


void DccBitstream::send_packet(const DccPkt& pkt)
{
    pwm_set_irq_enabled(_slice, false);

    // make sure the irq is really disabled before changing _next and _pkt_a/b
    __dmb();

    if (_current == &_pkt_a) {
        _pkt_b = pkt; // copy packet (only 12 bytes)
        _next = &_pkt_b;
    } else {
        _pkt_a = pkt; // copy
        _next = &_pkt_a;
    }

    // make sure _next and _pkt_a/b are in memory before enabling the irq
    __dmb();

    pwm_set_irq_enabled(_slice, true);
}


void DccBitstream::next_bit()
{
    // byte -1 is the preamble, then byte=0,1,...msg_len-1 for the message bytes
    if (_byte == -1) {
        // sending preamble
        // _bit=[pre_len-1...0], then -1 when preamble is done
        if (_bit == -1) {
            // end of preamble, send packet start bit
            prog_bit(0);
            _byte = 0;
            _bit = 8 - 1;
        } else {
            // continue preamble
            prog_bit(1);
            _bit--;
        }
    } else {
        int msg_len = _current->msg_len();
        // _bit = 7...0, then -1 means stop bit
        if (_bit == -1) {
            // send stop bit
            if ((_byte + 1) == msg_len) {
                // end of message, send 1
                prog_bit(1);
                // next message
                _current = _next;
                _next = &_pkt_idle;
                _byte = -1; // preamble
                // stop bit counts as first bit of next preamble
                // set _bit to pre_len-1, minus one more for the stop bit
                _bit = _preamble_bits - 2;
            } else {
                // more bytes in message, send 0
                prog_bit(0);
                _byte++;
                _bit = 8 - 1;
            }
        } else {
            int b = (_current->data(_byte) >> _bit) & 1;
            prog_bit(b);
            _bit--;
        }
    }
}


void DccBitstream::pwm_handler(void *arg)
{
    //DbgGpio g(0);

    DccBitstream *me = (DccBitstream *)arg;

    me->next_bit();
}
