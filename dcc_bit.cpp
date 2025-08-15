#include <Arduino.h>
#include "xassert.h"
#include "dcc_bit.h"


DccBit::DccBit(int verbosity) :
    _verbosity(verbosity),
    _bit_state(UNSYNC),
    _preamble(0),
    _bit(0),
    _edge_us(UINT64_MAX),
    _zero_us(UINT64_MAX),
    _bad_cnt(0),
    _byte(0),
    _bit_num(0),
    _pkt_len(0),
    _pkt_recv(nullptr)
{
}


DccBit::~DccBit()
{
}


void DccBit::on_pkt_recv(pkt_recv_t *pkt_recv)
{
    xassert(pkt_recv != nullptr);
    _pkt_recv = pkt_recv;
}


void DccBit::begin()
{
    if (_verbosity > 0)
        Serial.printf("verbosity=%d\n", _verbosity);
    if (_verbosity >= 4)
        Serial.printf(">UNSYNC\n");
}


// saw an edge at edge_us
void DccBit::edge(uint64_t edge_us)
{
    // _edge_us is UINT64_MAX on the first edge ever seen
    if (_edge_us == UINT64_MAX) {
        _edge_us = edge_us;
        return;
    }

    int us = int(edge_us - _edge_us);

    _edge_us = edge_us;

    half_bit(to_half(us));
}


void DccBit::half_bit(int half)
{
    if (half != 0 && half != 1) {
        if (_bit_state != UNSYNC && _verbosity >= 4)
            Serial.printf(" >UNSYNC");
        _bit_state = UNSYNC;
        _bad_cnt++;
        return;
    }

    switch (_bit_state) {

        case UNSYNC:
            if (half == 1) {
                _preamble = 1;
                _bit_state = PREAMBLE;
                if (_verbosity >= 4)
                    Serial.printf(" >PREAMBLE");
            } else {
                // stay in UNSYNC
            }
            break;

        case PREAMBLE:
            if (half == 0) {
                if (_preamble >= preamble_min) {
                    // this is the first half of a start bit for the first
                    // byte in a packet and the preamble is long enough;
                    // need second half of start bit
                    _pkt_len = 0;
                    _bit_num = 0;
                    _bit = 0;
                    _bit_state = BIT_H;
                    if (_verbosity >= 4)
                        Serial.printf(" %d >BIT_H", _preamble);
                    _start_us = _edge_us;
                    if (_zero_us == UINT64_MAX)
                        _zero_us = _start_us;
                } else {
                    // preamble not long enough
                    _bit_state = UNSYNC;
                    if (_verbosity >= 4)
                        Serial.printf(" %d >UNSYNC", _preamble);
                }
            } else { // half == 1
                _preamble++;
                // stay in PREAMBLE
            }
            break;

        case BIT_H:
            if (half == _bit) {
                // Got a valid bit (0 or 1). bit_rx() returns true when a
                // complete packet has been received, false otherwise.
                if (bit_rx()) {
                    // the final '1' counts in the next preamble
                    _preamble = 2;
                    _bit_state = PREAMBLE;
                    if (_verbosity >= 4)
                        Serial.printf(" >PREAMBLE");
                } else {
                    // still receiving packet
                    _bit_state = BIT;
                    if (_verbosity >= 4)
                        Serial.printf(" >BIT");
                }
            } else { // half != _bit
                // Either got a half-one when expecting a half-zero, or half-
                // zero when expecting half-one. A half-zero means we are no
                // longer synchronized. A half-one counts as part of the next
                // preamble.
                if (half == 0) {
                    _bit_state = UNSYNC;
                    if (_verbosity >= 4)
                        Serial.printf(" >UNSYNC");
                } else { // half == 1
                    _preamble = 1;
                    _bit_state = PREAMBLE;
                    if (_verbosity >= 4)
                        Serial.printf(" >PREAMBLE");
                }
            }
            break;

        case BIT:
            _bit = half;
            _bit_state = BIT_H;
            if (_verbosity >= 4)
                Serial.printf(" >BIT_H");
            break;

        default:
            xassert(false);
            break;

    } // switch (_bit_state)

} // DccBit::half_bit


// decoded a valid bit in the bitstream (_bit is 0 or 1)
// returns true if complete packet received and processed, false otherwise
bool DccBit::bit_rx()
{
    xassert(_bit == 0 || _bit == 1);

    if (_verbosity >= 3)
        Serial.printf(" bit=%d", _bit);

    // _bit_num 0 is the start bit
    // If zero, starting the next byte; if one, emit packet in progress.
    // _bit_num 1..8 are used to build data byte (msb first)

    xassert(0 <= _bit_num && _bit_num <= 8);

    if (_bit_num == 0) {
        if (_bit == 0) {
            // end of byte (or preamble), continue packet
            if (_verbosity >= 3)
                Serial.printf(" start");
            _bit_num++;
        } else {
            // end of packet
            if (_verbosity >= 3)
                Serial.printf(" end");
            if (_pkt_recv != nullptr)
                (*_pkt_recv)(_pkt, _pkt_len, _preamble / 2, _start_us, _bad_cnt);
            _bit_num = 0;
            _bad_cnt = 0;
            return true;
        }
    } else if (_bit_num < 8) {
        // first bit received is the MSB
        // shift up then put the new bit in the LSB
        _byte = (_byte << 1) | _bit;
        _bit_num++;
    } else { // _bit_num == 8
        _byte = (_byte << 1) | _bit;
        // last bit in byte, append to packet
        if (_verbosity >= 2)
            Serial.printf(" byte=%02x", _byte);
        _pkt[_pkt_len++] = _byte;
        _bit_num = 0;
        // pkt_recv() is called when a valid stop bit is seen
    }
    return false;
}
