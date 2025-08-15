#include <Arduino.h>
#include "xassert.h"
#include "dcc_pkt.h"


DccPkt::DccPkt(const uint8_t *msg, int msg_len)
{
    xassert(msg_len >= 0);

    memset(_msg, 0, msg_max);

    if (msg_len > msg_max)
        msg_len = 0;

    if (msg != nullptr)
        memcpy(_msg, msg, msg_len);

    _msg_len = msg_len;
}


void DccPkt::msg_len(int new_len)
{
    xassert(0 <= new_len && new_len <= msg_max);

    _msg_len = new_len;
}


uint8_t DccPkt::data(int idx) const
{
    xassert(0 <= idx && idx < _msg_len);

    return _msg[idx];
}


int DccPkt::address() const
{
    xassert(_msg_len >= 1);

    const uint8_t b0 = _msg[0];

    if (b0 < 128) {
        // broadcast (0) or multi-function decoder with 7-bit address
        return b0;

    } else if (b0 < 192) {
        // 128-191: accessory decoder with 9- or 11-bit address
        xassert(_msg_len >= 2);
        const uint8_t b1 = _msg[1];
        int adrs = (int(b0 & 0x3f) << 2) |
                   (int(~b1 & 0x70) << 4) |
                   (int(b1 & 0x06) >> 1);
        return adrs;

    } else if (b0 < 232) {
        // multi-function decoder with 14-bit address
        xassert(_msg_len >= 2);
        return (int(b0 & 0x3f) << 8) | _msg[1];

    } else if (b0 < 253) {
        // reserved
        return address_invalid;

    } else if (b0 < 255) {
        // advanced extended packet
        return address_invalid;

    } else {
        // idle packet (address = 255)
        return b0;

    }

} // DccPkt::address


// set address in packet
// return address bytes used (1 or 2)
int DccPkt::address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    if (adrs <= address_short_max) {
        // one byte in packet
        xassert(msg_max >= 1);
        _msg[0] = adrs;
        return 1;
    } else {
        // two bytes in packet
        xassert(msg_max >= 2);
        _msg[0] = 0xc0 | ((adrs >> 8) & 0x3f);
        xassert(0xc0 <= _msg[0] && _msg[0] <= 0xe7);
        _msg[1] = adrs & 0xff;
        return 2;
    }
}


// get size of address
// return address bytes used (1 or 2)
int DccPkt::address_size() const
{
    if (_msg[0] <= address_short_max)
        return 1;
    else
        return 2;
}


void DccPkt::set_xor()
{
    xassert(_msg_len > 0);
    xassert(_msg_len < msg_max);

    uint8_t b = 0x00;
    for (int i = 0; i < (_msg_len - 1); i++)
        b ^= _msg[i];
    _msg[_msg_len - 1] = b;
}


// Returns true if the current packet could be a service direct-mode packet
//
// Whether it's a service packet is state-dependent (i.e. decoder has been put
// in service mode). Decoding of the packet might be more state-dependent; it
// looks like there is overlap in the packet bit patterns:
//
// 0 0111_1101 0 0000_0001 0 EEEE_EEEE 1                Page preset
// 0 0111_0000 0 0DDD_DDDD 0 EEEE_EEEE 1                Address only verify
// 0 0111_1000 0 0DDD_DDDD 0 EEEE_EEEE 1                Address only write
// 0 0111_0RRR 0 DDDD_DDDD 0 EEEE_EEEE 1                Phys register verify
// 0 0111_1RRR 0 DDDD_DDDD 0 EEEE_EEEE 1                Phys register write
// 0 0111_0RRR 0 DDDD_DDDD 0 EEEE_EEEE 1                Page mode verify
// 0 0111_1RRR 0 DDDD_DDDD 0 EEEE_EEEE 1                Page mode write
//                                                          RRR: 000..101
// 0 0111_1111 0 0000_1000 0 EEEE_EEEE 1                Decoder factory reset
//
// 0 0111_01AA 0 AAAA_AAAA 0 DDDD_DDDD 0 EEEE_EEEE 1    Direct mode verify
// 0 0111_11AA 0 AAAA_AAAA 0 DDDD_DDDD 0 EEEE_EEEE 1    Direct mode write
// 0 0111_10AA 0 AAAA_AAAA 0 111K_DBBB 0 EEEE_EEEE 1    Direct mode bit manip

bool DccPkt::is_svc_direct(const uint8_t *msg, int msg_len)
{
    if (msg_len != 4)
        return false;

    // 1st byte without the two address bits
    const uint8_t b0 = msg[0] & 0xfc;

    // for write and verify, 2nd and 3rd bytes can be anything

    if (b0 == 0x74 || b0 == 0x7c)
        return true; // write or verify

    // for bit manip, check those 1 bits in 3rd byte

    if (b0 == 0x78 && (msg[2] & 0xe0) == 0xe0)
        return true; // bit manipulation

    return false;
}


char *DccPkt::dump(char *buf, int buf_len) const
{
    xassert(buf != nullptr);
    xassert(buf_len >= 0);

    memset(buf, '\0', buf_len);

    char *b = buf;
    char *e = buf + buf_len;

    b += snprintf(b, e - b, "{");

    if (e < b)
        return buf;

    for (int i = 0; i < _msg_len; i++) {
        b += snprintf(b, e - b, " %02x", _msg[i]);
        if (e < b)
            return buf;
    }

    b += snprintf(b, e - b, " }");

    return buf;
}


char *DccPkt::show(char *buf, int buf_len) const
{
    xassert(buf != nullptr);
    xassert(buf_len >= 0);

    memset(buf, '\0', buf_len);

    char *b = buf;
    char *e = buf + buf_len;

    int idx = 0;

    // need a byte from idx, and it shouldn't be the last xor byte
    if (_msg_len < (idx + 2)) {
        b += snprintf(b, e - b, "out of data at byte %d: ", idx);
        dump(b, e - b);
        return buf;
    }

    uint8_t b0 = _msg[idx++];
    xassert(idx == 1);

    if (b0 < 128 || (192 <= b0 && b0 < 232)) {

        int adrs = b0;

        // check for service mode packet
        if (is_svc_direct(_msg, _msg_len)) {
            b += snprintf(b, e - b, " svc: ");
            // it's 4 bytes long with the correct constant bits

            int op = (_msg[0] & 0x0c) >> 2; // 1, 2, or 3

            int cv = (_msg[0] & 0x03);
            cv = (cv << 8) | _msg[1];
            cv++; // by convention, cv number starts at 1

            if (op == 1) {
                b += snprintf(b, e - b, "verify cv%d=0x%02x", cv, _msg[2]);
            } else if (op == 2) {
                int bit = _msg[2] & 0x07; // 0..7
                int val = (_msg[2] & 0x08) >> 3; // 0..1
                if (_msg[2] & 0x10) {
                    b += snprintf(b, e - b, "write cv%d bit%d=%d", cv, bit, val);
                } else {
                    b += snprintf(b, e - b, "verify cv%d bit%d=%d", cv, bit, val);
                }
            } else {
                b += snprintf(b, e - b, "write cv%d=0x%02x", cv, _msg[2]);
            }
            return buf;
        } else if (b0 >= 128) {
            // long address
            if (_msg_len < (idx + 2)) {
                b += snprintf(b, e - b, "out of data at byte %d: ", idx);
                dump(b, e - b);
                return buf;
            }
            uint8_t b1 = _msg[idx++];
            adrs = ((adrs & 0x3f) << 8) | b1;
        }

        xassert(idx == 1 || idx == 2);

        b += snprintf(b, e - b, "%4d: ", adrs);

        if (_msg_len < (idx + 2)) {
            b += snprintf(b, e - b, "out of data at byte %d: ", idx);
            dump(b, e - b);
            return buf;
        }

        uint8_t instr = _msg[idx++];

        if (instr == 0x00) {

            b += snprintf(b, e - b, "reset");

            if (_msg_len != (idx + 1)) {
                b += snprintf(b, e - b, " extra: ");
                dump(b, e - b);
            }

        } else if (instr == 0x3f) {

            if (_msg_len < (idx + 2)) {
                b += snprintf(b, e - b, "out of data at byte %d: ", idx);
                dump(b, e - b);
                return buf;
            }

            int speed = _msg[idx++];

            b += snprintf(b, e - b, "%s %d/128",
                            speed & 0x80 ? "fwd" : "rev", speed & 0x7f);

            if (_msg_len != (idx + 1)) {
                b += snprintf(b, e - b, " extra: ");
                dump(b, e - b);
            }

        } else if ((instr & 0xe0) == 0x80) {

            b += snprintf(b, e - b, "f0%c f1%c f2%c f3%c f4%c",
                          instr & 0x10 ? '+' : '-',
                          instr & 0x01 ? '+' : '-', instr & 0x02 ? '+' : '-',
                          instr & 0x04 ? '+' : '-', instr & 0x08 ? '+' : '-');

            if (_msg_len != (idx + 1)) {
                b += snprintf(b, e - b, " extra: ");
                dump(b, e - b);
            }

        } else if ((instr & 0xf0) == 0xb0) {

            b += snprintf(b, e - b, "f5%c f6%c f7%c f8%c",
                          instr & 0x01 ? '+' : '-', instr & 0x02 ? '+' : '-',
                          instr & 0x04 ? '+' : '-', instr & 0x08 ? '+' : '-');

            if (_msg_len != (idx + 1)) {
                b += snprintf(b, e - b, " extra: ");
                dump(b, e - b);
            }

        } else if ((instr & 0xf0) == 0xa0) {

            b += snprintf(b, e - b, "f9%c f10%c f11%c f12%c",
                          instr & 0x01 ? '+' : '-', instr & 0x02 ? '+' : '-',
                          instr & 0x04 ? '+' : '-', instr & 0x08 ? '+' : '-');

            if (_msg_len != (idx + 1)) {
                b += snprintf(b, e - b, " extra: ");
                dump(b, e - b);
            }

        } else if (instr == 0xde) {

            if (_msg_len < (idx + 2)) {
                b += snprintf(b, e - b, "out of data at byte %d: ", idx);
                dump(b, e - b);
                return buf;
            }

            uint8_t f = _msg[idx++];

            b += snprintf(b, e - b, "f13%c f14%c f15%c f16%c f17%c f18%c f19%c f20%c",
                          f & 0x01 ? '+' : '-', f & 0x02 ? '+' : '-',
                          f & 0x04 ? '+' : '-', f & 0x08 ? '+' : '-',
                          f & 0x10 ? '+' : '-', f & 0x20 ? '+' : '-',
                          f & 0x40 ? '+' : '-', f & 0x80 ? '+' : '-');

            if (_msg_len != (idx + 1)) {
                b += snprintf(b, e - b, " extra: ");
                dump(b, e - b);
            }

        } else if (instr == 0xdf) {

            if (_msg_len < (idx + 2)) {
                b += snprintf(b, e - b, "out of data at byte %d: ", idx);
                dump(b, e - b);
                return buf;
            }

            uint8_t f = _msg[idx++];

            b += snprintf(b, e - b, "f21%c f22%c f23%c f24%c f25%c f26%c f27%c f28%c",
                          f & 0x01 ? '+' : '-', f & 0x02 ? '+' : '-',
                          f & 0x04 ? '+' : '-', f & 0x08 ? '+' : '-',
                          f & 0x10 ? '+' : '-', f & 0x20 ? '+' : '-',
                          f & 0x40 ? '+' : '-', f & 0x80 ? '+' : '-');

            if (_msg_len != (idx + 1)) {
                b += snprintf(b, e - b, " extra: ");
                dump(b, e - b);
            }

        }

    } else if (128 <= b0 && b0 < 192) {

        xassert(idx == 1);

        // 2.4.1 Basic Accessory Decoder Packet Format
        // [preamble] 0 10AAAAAA 0 1AAADAAR 0 EEEEEEEE 1
        // Packet length == 3

        // 2.4.3.1 Basic Accessory Decoder Operations Mode Programming
        // [preamble] 0 10AAAAAA 0 1AAA1AA0 0 1110GGVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
        // Packet length == 6; maybe 5 if DDDDDDDD not required?

        // 2.4.4 Basic Accessory Decoder XPOM
        // [preamble] 0 10AAAAAA 0 1AAA1AA0 0 1110GGSS 0
        //              VVVVVVVV 0 VVVVVVVV 0 VVVVVVVV 0
        //              { DDDDDDDD 0 { DDDDDDDD 0 { DDDDDDDD 0 { DDDDDDDD 0 }}}} EEEEEEEE 1
        // Packet length == 7, 8, 9, 10, or 11

        // 2.4.2 Extended Accessory Decoder Control Packet Format
        // [preamble] 0 10AAAAAA 0 0AAA0AA1 0 XXXXXXXX 0 EEEEEEEE 1
        // Packet length == 4

        // 2.4.3.2 Extended Accessory Decoder Operations Mode Programming
        // [preamble] 0 10AAAAAA 0 0AAA0AA1 0 1110GGVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
        // Packet length == 6; maybe 5 if DDDDDDDD not required?

        // 2.4.5 Extended Accessory Decoder XPOM
        // [preamble] 0 10AAAAAA 0 0AAA0AA1 0 1110GGSS 0
        //              VVVVVVVV 0 VVVVVVVV 0 VVVVVVVV 0
        //              { DDDDDDDD 0 { DDDDDDDD 0 { DDDDDDDD 0 { DDDDDDDD 0 }}}} EEEEEEEE 1
        // Zero to four DDDDDDDD bytes
        // Packet length == 7, 8, 9, 10, or 11

        // 2.4.6 NOP
        // [preamble] 0 10AAAAAA 0 0AAA1AAT 0 EEEEEEEE 1
        // Packet length = 3

        if (_msg_len < 3) {
            b += snprintf(b, e - b, "out of data at byte %d: ", idx);
            dump(b, e - b);
            return buf;
        }

        uint8_t b1 = _msg[1];
        int adrs = (int(b0 & 0x3f) << 2) |
                   (int(~b1 & 0x70) << 4) |
                   (int(b1 & 0x06) >> 1);

        int m = (b1 >> 7) & 1;
        int d = (b1 >> 3) & 1;
        int r = (b1 >> 0) & 1;

        b += snprintf(b, e - b, "%4d: acc m=%d d=%d r=%d: ", adrs, m, d, r);

        dump(b, e - b);

    } else if (b0 == 255) {

        b += snprintf(b, e - b, "      idle");

        if (_msg_len != 3) {
            b += snprintf(b, e - b, ": ");
            dump(b, e - b);
        }

    } else {

        // "reserved" (232-252) or "advanced extended" (253-254)
        dump(b, e - b);

    } // if (b0...)

    return buf;

} // DccPkt::show

//----------------------------------------------------------------------------

DccPktIdle::DccPktIdle()
{
    _msg[0] = 0xff;
    _msg[1] = 0x00;
    _msg_len = 3;
    set_xor();
}

//----------------------------------------------------------------------------

DccPktReset::DccPktReset()
{
    _msg[0] = 0x00;
    _msg[1] = 0x00;
    _msg_len = 3;
    set_xor();
}

//----------------------------------------------------------------------------

DccPktSpeed128::DccPktSpeed128(int adrs, int speed)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(speed_min <= speed && speed <= speed_max);

    refresh(adrs, speed);
}


int DccPktSpeed128::address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, speed());
    return address_size();
}


int DccPktSpeed128::speed() const
{
    int idx = address_size() + 1; // skip address and inst byte (0x3f)
    return dcc_to_int(_msg[idx]);
}


void DccPktSpeed128::speed(int speed)
{
    xassert(speed_min <= speed && speed <= speed_max);

    int idx = address_size() + 1; // skip address and inst byte (0x3f)
    _msg[idx] = int_to_dcc(speed);
    set_xor();
}


void DccPktSpeed128::refresh(int adrs, int speed)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(speed_min <= speed && speed <= speed_max);

    int idx = DccPkt::address(adrs); // 1 or 2 bytes
    _msg[idx++] = 0x3f; // CCC=001 GGGGG=11111
    _msg[idx++] = int_to_dcc(speed);
    _msg_len = idx + 1; // 4 or 5
    set_xor();
}


// DCC speed: msb: 1 is forward, 0 is reverse, remaining bits are magnitude

uint8_t DccPktSpeed128::int_to_dcc(int speed_int)
{
    if (speed_int < 0)
        return -speed_int;
    else
        return speed_int | 0x80;
}


int DccPktSpeed128::dcc_to_int(uint8_t speed_dcc)
{
    if (speed_dcc & 0x80)
        return speed_dcc & ~0x80; // forward, just clear msb
    else
        return -int(speed_dcc); // reverse, return negative of magnitude
}

//----------------------------------------------------------------------------

DccPktFunc0::DccPktFunc0(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs);
}


int DccPktFunc0::address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, funcs());
    return address_size();
}


bool DccPktFunc0::f(int num) const
{
    xassert(f_min <= num && num <= f_max);

    int idx = address_size(); // skip address
    uint8_t f_bit = ((num == 0) ? 0x10 : (0x01 << (num - 1))); // bit for function
    return (_msg[idx] & f_bit) != 0;
}


void DccPktFunc0::f(int num, bool on)
{
    xassert(f_min <= num && num <= f_max);

    int idx = address_size(); // skip address
    uint8_t f_bit = ((num == 0) ? 0x10 : (0x01 << (num - 1))); // bit for function
    if (on)
        _msg[idx] |= f_bit;
    else
        _msg[idx] &= ~f_bit;
    set_xor();
}


void DccPktFunc0::refresh(int adrs, uint8_t funcs)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert((funcs & ~0x1f) == 0);

    int idx = DccPkt::address(adrs); // 1 or 2 bytes
    _msg[idx++] = 0x80 | funcs; // CCC=100, then f0:f4:f3:f2:f1
    _msg_len = idx + 1; // 3 or 4
    set_xor();
}


uint8_t DccPktFunc0::funcs() const
{
    int idx = address_size(); // skip address
    return _msg[idx] & 0x1f; // lower 5 bits
}

//----------------------------------------------------------------------------

DccPktFunc5::DccPktFunc5(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs);
}


int DccPktFunc5::address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, funcs());
    return address_size();
}


bool DccPktFunc5::f(int num) const
{
    xassert(f_min <= num && num <= f_max);

    int idx = address_size(); // skip address
    uint8_t f_bit = 0x01 << (num - f_min); // bit for function
    return (_msg[idx] & f_bit) != 0;
}


void DccPktFunc5::f(int num, bool on)
{
    xassert(f_min <= num && num <= f_max);

    int idx = address_size(); // skip address
    uint8_t f_bit = 0x01 << (num - f_min); // bit for function
    if (on)
        _msg[idx] |= f_bit;
    else
        _msg[idx] &= ~f_bit;
    set_xor();
}


void DccPktFunc5::refresh(int adrs, uint8_t funcs)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert((funcs & ~0x0f) == 0);

    int idx = DccPkt::address(adrs); // 1 or 2 bytes
    _msg[idx++] = 0xb0 | funcs; // CCC=101, S=1, then f8:f7:f6:f5
    _msg_len = idx + 1; // 3 or 4
    set_xor();
}


uint8_t DccPktFunc5::funcs() const
{
    int idx = address_size(); // skip address
    return _msg[idx] & 0x0f; // lower 4 bits
}

//----------------------------------------------------------------------------

DccPktFunc9::DccPktFunc9(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs);
}


int DccPktFunc9::address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, funcs());
    return address_size();
}


bool DccPktFunc9::f(int num) const
{
    xassert(f_min <= num && num <= f_max);

    int idx = address_size(); // skip address
    uint8_t f_bit = 0x01 << (num - f_min); // bit for function
    return (_msg[idx] & f_bit) != 0;
}


void DccPktFunc9::f(int num, bool on)
{
    xassert(f_min <= num && num <= f_max);

    int idx = address_size(); // skip address
    uint8_t f_bit = 0x01 << (num - f_min); // bit for function
    if (on)
        _msg[idx] |= f_bit;
    else
        _msg[idx] &= ~f_bit;
    set_xor();
}


void DccPktFunc9::refresh(int adrs, uint8_t funcs)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert((funcs & ~0x0f) == 0);

    int idx = DccPkt::address(adrs); // 1 or 2 bytes
    _msg[idx++] = 0xa0 | funcs; // CCC=101, S=0, then f12:f11:f10:f9
    _msg_len = idx + 1; // 3 or 4
    set_xor();
}


uint8_t DccPktFunc9::funcs() const
{
    int idx = address_size(); // skip address
    return _msg[idx] & 0x0f; // lower 4 bits
}

//----------------------------------------------------------------------------

DccPktFunc13::DccPktFunc13(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs);
}


int DccPktFunc13::address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, funcs());
    return address_size();
}


bool DccPktFunc13::f(int num) const
{
    xassert(f_min <= num && num <= f_max);

    int idx = address_size() + 1; // skip address and inst byte
    uint8_t f_bit = 1 << (num - f_min); // bit for function
    return (_msg[idx] & f_bit) != 0;
}


void DccPktFunc13::f(int num, bool on)
{
    xassert(f_min <= num && num <= f_max);

    int idx = address_size() + 1; // skip address and inst byte
    uint8_t f_bit = 1 << (num - f_min); // bit for function
    if (on)
        _msg[idx] |= f_bit;
    else
        _msg[idx] &= ~f_bit;
    set_xor();
}


void DccPktFunc13::refresh(int adrs, uint8_t funcs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    int idx = DccPkt::address(adrs); // 1 or 2 bytes
    _msg[idx++] = 0xde;     // CCC=110 GGGGG=11110
    _msg[idx++] = funcs;    // f20:f19:f18:f17:f16:f15:f14:f13
    _msg_len = idx + 1;     // 4 or 5
    set_xor();
}


uint8_t DccPktFunc13::funcs() const
{
    int idx = address_size() + 1; // skip address and inst byte
    return _msg[idx]; // all 8 bits
}

//----------------------------------------------------------------------------

DccPktFunc21::DccPktFunc21(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs);
}


int DccPktFunc21::address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, funcs());
    return address_size();
}


bool DccPktFunc21::f(int num) const
{
    xassert(f_min <= num && num <= f_max);

    int idx = address_size() + 1;       // skip address and inst byte
    uint8_t f_bit = 1 << (num - f_min); // bit for function
    return (_msg[idx] & f_bit) != 0;
}


void DccPktFunc21::f(int num, bool on)
{
    xassert(f_min <= num && num <= f_max);

    int idx = address_size() + 1;       // skip address and inst byte
    uint8_t f_bit = 1 << (num - f_min); // bit for function
    if (on)
        _msg[idx] |= f_bit;
    else
        _msg[idx] &= ~f_bit;
    set_xor();
}


void DccPktFunc21::refresh(int adrs, uint8_t funcs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    int idx = DccPkt::address(adrs);    // 1 or 2 bytes
    _msg[idx++] = 0xdf;                 // CCC=110 GGGGG=11111
    _msg[idx++] = funcs;                // f28:f27:f26:f25:f24:f23:f22:f21
    _msg_len = idx + 1;                 // 4 or 5
    set_xor();
}


uint8_t DccPktFunc21::funcs() const
{
    int idx = address_size() + 1;       // skip address and inst byte
    return _msg[idx];                   // all 8 bits
}

//----------------------------------------------------------------------------

DccPktOpsWriteCv::DccPktOpsWriteCv(int adrs, int cv_num, uint8_t cv_val)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);

    refresh(adrs, cv_num, cv_val);
}


int DccPktOpsWriteCv::address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, cv_num(), cv_val());
    return address_size();
}


void DccPktOpsWriteCv::cv(int cv_num, uint8_t cv_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024

    cv_num--; // cv_num is encoded in messages as 0..1023
    int idx = address_size();           // skip address (1 or 2 bytes)
    _msg[idx++] = 0xec | (cv_num >> 8); // 111011vv
    _msg[idx++] = cv_num;               // vvvvvvvv
    _msg[idx++] = cv_val;               // dddddddd
    _msg_len = idx + 1;                 // total (with xor) 5 or 6 bytes
    set_xor();
}


void DccPktOpsWriteCv::refresh(int adrs, int cv_num, uint8_t cv_val)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024

    (void)DccPkt::address(adrs);        // insert address (1 or 2 bytes)
    cv(cv_num, cv_val);                 // insert everything else
}


int DccPktOpsWriteCv::cv_num() const
{
    int idx = address_size();               // skip address (1 or 2 bytes)
    int cv_hi = _msg[idx++] & 0x03;         // get 2 hi bits
    int cv_num = (cv_hi << 8) | _msg[idx];  // get 8 lo bits
    return cv_num + 1; // cv_num is 0..1023 in message, return 1..1024
}


uint8_t DccPktOpsWriteCv::cv_val() const
{
    int idx = address_size() + 2; // skip address, instruction, cv_num
    return _msg[idx];
}

//----------------------------------------------------------------------------

DccPktOpsWriteBit::DccPktOpsWriteBit(int adrs, int cv_num, int bit_num, int bit_val)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    refresh(adrs, cv_num, bit_num, bit_val);
}


// constructor for when we know we'll be setting fields later
DccPktOpsWriteBit::DccPktOpsWriteBit()
{
    refresh(3, 8, 0, 0);
}


int DccPktOpsWriteBit::address(int adrs)
{
    xassert(address_min <= adrs && adrs <= address_max);

    refresh(adrs, cv_num(), bit_num(), bit_val());
    return address_size();
}


// set cv_num, bit_num, and bit_val in message
void DccPktOpsWriteBit::cv_bit(int cv_num, int bit_num, int bit_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    cv_num--; // cv_num is encoded in messages as 0..1023
    int idx = address_size();                       // skip address (1 or 2 bytes)
    _msg[idx++] = 0xe8 | (cv_num >> 8);             // 111010vv
    _msg[idx++] = cv_num;                           // vvvvvvvv
    _msg[idx++] = 0xf0 | (bit_val << 3) | bit_num;  // dddddddd
    _msg_len = idx + 1;                             // total (with xor) 5 or 6 bytes
    set_xor();
}


// update message where bytes in address (1 or 2) might be changing
void DccPktOpsWriteBit::refresh(int adrs, int cv_num, int bit_num, int bit_val)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    (void)DccPkt::address(adrs);        // insert address (1 or 2 bytes)
    cv_bit(cv_num, bit_num, bit_val);   // insert everything else
}


// get cv_num from message
int DccPktOpsWriteBit::cv_num() const
{
    int idx = address_size();               // skip address (1 or 2 bytes)
    int cv_hi = _msg[idx++] & 0x03;         // get 2 hi bits
    int cv_num = (cv_hi << 8) | _msg[idx];  // get 8 lo bits
    return cv_num + 1; // cv_num is 0..1023 in message, return 1..1024
}


// get bit_num from message
int DccPktOpsWriteBit::bit_num() const
{
    int idx = address_size() + 2;   // skip address, instruction, cv_num
    return _msg[idx] & 0x07;        // return lo 3 bits
}


// get bit_val from message
int DccPktOpsWriteBit::bit_val() const
{
    int idx = address_size() + 2;   // skip address, instruction, cv_num
    return (_msg[idx] >> 3) & 1;    // return bit 3
}

//----------------------------------------------------------------------------

DccPktSvcWriteCv::DccPktSvcWriteCv(int cv_num, uint8_t cv_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024

    set_cv(cv_num, cv_val);
}


void DccPktSvcWriteCv::set_cv(int cv_num, uint8_t cv_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024

    cv_num--; // cv_num is encoded in messages as 0..1023
    _msg[0] = 0x7c | (cv_num >> 8); // 0111CCAA, CC=11 "write byte"
    _msg[1] = cv_num;               // AAAAAAAA
    _msg[2] = cv_val;               // DDDDDDDD
    _msg_len = 4;                   // total (with xor) 4 bytes
    set_xor();
}

//----------------------------------------------------------------------------

DccPktSvcWriteBit::DccPktSvcWriteBit(int cv_num, int bit_num, int bit_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024

    set_cv_bit(cv_num, bit_num, bit_val);
}


void DccPktSvcWriteBit::set_cv_bit(int cv_num, int bit_num, int bit_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    cv_num--; // cv_num is encoded in messages as 0..1023
    _msg[0] = 0x78 | (cv_num >> 8);
    _msg[1] = cv_num;
    _msg[2] = 0xf0 | (bit_val << 3) | bit_num;
    _msg_len = 4;
    set_xor();
}

//----------------------------------------------------------------------------

DccPktSvcVerifyCv::DccPktSvcVerifyCv(int cv_num, uint8_t cv_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024

    set_cv_num(cv_num);
    set_cv_val(cv_val);
}


void DccPktSvcVerifyCv::set_cv_num(int cv_num)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024

    cv_num--; // cv_num is encoded in messages as 0..1023
    _msg[0] = 0x74 | (cv_num >> 8); // 0111CCAA, CC=01 "verify byte"
    _msg[1] = cv_num;               // AAAAAAAA
  //_msg[2] = cv_val;               // DDDDDDDD
    _msg_len = 4;                   // total (with xor) 4 bytes
    set_xor();
}


void DccPktSvcVerifyCv::set_cv_val(uint8_t cv_val)
{
  //_msg[0] = 0x74 | (cv_num >> 8); // 0111CCAA, CC=01 "verify byte"
  //_msg[1] = cv_num;               // AAAAAAAA
    _msg[2] = cv_val;               // DDDDDDDD
    _msg_len = 4;                   // total (with xor) 4 bytes
    set_xor();
}

//----------------------------------------------------------------------------

DccPktSvcVerifyBit::DccPktSvcVerifyBit(int cv_num, int bit_num, int bit_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    set_cv_bit(cv_num, bit_num, bit_val);
}


void DccPktSvcVerifyBit::set_cv_bit(int cv_num, int bit_num, int bit_val)
{
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max); // 1..1024
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    cv_num--; // cv_num is encoded in messages as 0..1023
    _msg[0] = 0x78 | (cv_num >> 8);
    _msg[1] = cv_num;
    _msg[2] = 0xe0 | (bit_val << 3) | bit_num;
    _msg_len = 4;
    set_xor();
}


void DccPktSvcVerifyBit::set_bit(int bit_num, int bit_val)
{
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    //_msg[0] = 0x78 | (cv_num >> 8);
    //_msg[1] = cv_num;
    _msg[2] = 0xe0 | (bit_val << 3) | bit_num;
    //_msg_len = 4;
    set_xor();
}
