#pragma once

#include <Arduino.h>
#include "hardware/pwm.h"
#include "dcc_pkt.h"


class DccBitstream
{

    public:

        DccBitstream(int sig_gpio, int pwr_gpio);
        ~DccBitstream();

        void power(bool on);

        void start_ops();
        void start_svc();
        void stop();

        inline bool need_packet()
        {
            return (_next == &_pkt_idle);
        }

        void send_packet(const DccPkt& pkt);

        inline void send_reset()
        {
            send_packet(_pkt_reset);
        }

    private:

        int _pwr_gpio;

        DccPktIdle _pkt_idle;
        DccPktReset _pkt_reset;
        DccPkt _pkt_a;
        DccPkt _pkt_b;

        // ISR sends packet at _current. When it is done:
        //   _current = _next   // _pkt_a, _pkt_b, _pkt_idle, _pkt_reset
        //   _next = _pkt_idle
        //   start packet at _current with _preamble_bits

        DccPkt *_first;     // first _current to go
        DccPkt *_current;   // never nullptr
        DccPkt *_next;      // _pkt_idle if nothing to send

        uint _preamble_bits;

        uint _slice;
        uint _channel;

        int _byte; // -1 for preamble, then index in _current
        int _bit;  // counts down bit in preamble or _byte

        void start(uint preamble_bits, DccPkt& first);

        inline void prog_bit(int b)
        {
            // Period is wrap+1 usec (pwm_hz = 1 MHz), and output is high for
            // count=[0...level-1], low for count=[level...wrap].
            // Set wrap to bit_us-1 and level to bit_us/2.
            // E.g. for square wave with period 4 us, wrap=3 and level=2.

            uint half_us = (b == 0 ? 100 : 58); // half-bit times
            pwm_set_wrap(_slice, 2 * half_us - 1);
            pwm_set_chan_level(_slice, _channel, half_us);
        }

        void next_bit();

        static void pwm_handler(void *arg);

}; // class DccBitstream
