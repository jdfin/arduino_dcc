#pragma once

#include <Arduino.h>


class DccBit
{

public:

    DccBit(int verbosity=0);

    ~DccBit();

    void begin();

    // saw an edge at edge_us
    void edge(uint64_t edge_us);

    // convert an interval into a half-bit
    static int to_half(int d_us)
    {
        if (d_us >= tr0_min_us && d_us <= tr0_max_us)
            return 0;
        else if (d_us >= tr1_min_us && d_us <= tr1_max_us)
            return 1;
        else
            return 2;
    }

    // process a valid half-bit
    void half_bit(int half);

    // packet-receive function type
    typedef void pkt_recv_t(const uint8_t *pkt, int pkt_len, int preamble_len,
                            uint64_t start_us, int bad_cnt);

    // install function to be called on complete packet received
    void on_pkt_recv(pkt_recv_t *pkt_recv);

private:

    // verbosity:
    // 0 - silent (xassert messages only)
    // message per-packet would be in the callback
    // 2 - byte received (one message per byte)
    // 3 - bit received (one message per bit)
    // 4 - synchronization state changes (one message per edge)
    int _verbosity;

    // Bitstream receive state proceeds as follows:
    //
    // UNSYNC - Waiting for preamble.
    // When a valid half-one is seen, go to PREAMBLE.
    // 
    // PREAMBLE - Counting ones in preamble.
    // Count half-ones forever. When a half-zero is seen, reset for a packet
    // and go to BIT_H if there have been enough half-ones (preamble long
    // enough), or go to UNSYNC if not enough preamble half-ones.
    //
    // BIT_H - First half of a bit has been seen (0 or 1).
    // If the same valid half-bit is seen, call rx_bit() and go to either BIT
    // if a packet is still in progress, or to PREAMBLE if a complete packet
    // was received. If the second half of the bit is different from the first
    // half, go to UNSYNC if the new half-bit is a zero, or to PREAMBLE if the
    // new half-bit is a one (it is the first half-bit of the next preamble).
    //
    // BIT - A complete bit was received (both halves).
    // On the next half-bit, set _bit (the half-bit received) and go to BIT_H
    // to wait for the second half of the bit.

    enum BitState {
        UNSYNC,     // waiting for a half-one to start the preamble
        PREAMBLE,   // waiting for a half-zero, counting half-ones in preamble
        BIT_H,      // saw first half of _bit (0 or 1)
        BIT,        // got a complete bit, waiting for next bit
    } _bit_state;

    int _preamble; // count of half-ones in preamble

    // 10 complete one-bits required = 20 half-bits
    static const int preamble_min = 20;

    int _bit;  // bit we're in the middle of (0 or 1)

    uint64_t _edge_us; // time of last edge

    uint64_t _start_us; // packet start time (start of 0 at end of preamble)

    uint64_t _zero_us; // first packet start time

    int _bad_cnt; // something not a valid zero or one

    // byte receive

    uint8_t _byte;
    int _bit_num;

    // decoded a valid bit in the bitstream (_bit is 0 or 1)
    bool bit_rx();

    // packet receive

    static const int pkt_max = 16;
    uint8_t _pkt[pkt_max];

    // 0..pkt_max-1; increments from zero as bytes are received
    int _pkt_len;

    pkt_recv_t *_pkt_recv;

public:

    // DCC spec

    // Used when transmitting bits

    static const int t1_min_us = 55;
    static const int t1_nom_us = 58;
    static const int t1_max_us = 61;

    static const int t1d_max_us = 3;

    static const int t0_min_us = 95;
    static const int t0_nom_us = 100;
    static const int t0_max_us = 9900;

    // Used when receiving bits

    static const int tr1_min_us = 52;
    static const int tr1_nom_us = 58;
    static const int tr1_max_us = 64;

    static const int tr1d_max_us = 6;

    static const int tr0_min_us = 90;
    static const int tr0_nom_us = 100;
    static const int tr0_max_us = 10000;

}; // class DccBit
