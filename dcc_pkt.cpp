#include <Arduino.h>
#include "xassert.h"
#include "dcc_pkt.h"


void DccPkt::msg_len(int new_len)
{
    xassert(new_len <= msg_max);
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
        const uint16_t b1 = _msg[1];
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
    xassert(0 <= adrs && adrs <= address_max);

    if (adrs <= 127) {
        xassert(msg_max >= 1);
        _msg[0] = adrs;
        return 1;
    } else { // adrs <= adrs_max
        xassert(msg_max >= 2);
        _msg[0] = 0xc0 | ((adrs >> 8) & 0x3f);
        xassert(0xc0 <= _msg[0] && _msg[0] <= 0xe7);
        _msg[1] = adrs & 0xff;
        return 2;
    }
}


// get size of address
// return address bytes used (1 or 2)
int DccPkt::address_size()
{
    if (_msg[0] <= 127)
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


// This decodes messages that might be in a decoder's refresh list
// so we can update a message instead of adding a new one.
//
// It does not decode all messages, e.g. CV access.
//
// It also does not decode accessory messages.
DccPkt::Command DccPkt::command() const
{
    if (_msg_len < 3)
        return UNKNOWN;

    uint8_t b0 = _msg[0];

    if (b0 < 128 || (192 <= b0 && b0 < 232)) {
        // multi-function decoder
        uint8_t cmd;
        if (b0 < 128)
            cmd = _msg[1];
        else
            cmd = _msg[2];

        if (cmd == 0x00) {
            return RESET;
        } else if (cmd == 0x3f) {
            return SPEED_128;
        } else if ((cmd & 0xe0) == 0x80) {
            return FUNC_0_4;
        } else if ((cmd & 0xf0) == 0xb0) {
            return FUNC_5_8;
        } else if ((cmd & 0xf0) == 0xa0) {
            return FUNC_9_12;
        } else if (cmd == 0xde) {
            return FUNC_13_20;
        } else if (cmd == 0xdf) {
            return FUNC_21_28;
        } else if (cmd == 0xd8) {
            return FUNC_29_36;
        } else if (cmd == 0xd9) {
            return FUNC_37_44;
        } else if (cmd == 0xda) {
            return FUNC_45_52;
        } else if (cmd == 0xdb) {
            return FUNC_53_60;
        } else if (cmd == 0xdc) {
            return FUNC_61_68;
        }
    }

    return UNKNOWN;

} // DccPkt::command()


uint8_t DccPkt::dcc_speed(int int_speed)
{
    xassert(speed_min <= int_speed && int_speed <= speed_max);

    if (int_speed < 0)
        return -int_speed;
    else // int_speed >= 0
        return int_speed | 0x80;
}


int DccPkt::int_speed(uint8_t dcc_speed)
{
    if (dcc_speed & 0x80)
        return dcc_speed & ~0x80;
    else
        return -int(dcc_speed);
}


DccPkt::Command DccPkt::dcc_function(int function)
{
    if (function < 0) {
        return UNKNOWN;
    } else if (function <= 4) {
        return FUNC_0_4;
    } else if (function <= 8) {
        return FUNC_5_8;
    } else if (function <= 12) {
        return FUNC_9_12;
    } else if (function <= 20) {
        return FUNC_13_20;
    } else if (function <= 28) {
        return FUNC_21_28;
    } else if (function <= 36) {
        return FUNC_29_36;
    } else if (function <= 44) {
        return FUNC_37_44;
    } else if (function <= 52) {
        return FUNC_45_52;
    } else if (function <= 60) {
        return FUNC_53_60;
    } else if (function <= 68) {
        return FUNC_61_68;
    } else {
        return UNKNOWN;
    }
}


bool DccPkt::make_raw(const uint8_t *msg, int msg_len)
{
    xassert(msg != nullptr);

    if (msg_len > msg_max) {
        memset(_msg, 0, msg_max);
        _msg_len = 0;
        return false;
    }

    memcpy(_msg, msg, msg_len);
    _msg_len = msg_len;
    return true;
}


void DccPkt::make_reset()
{
    xassert(msg_max >= 3);
    _msg[0] = 0x00;
    _msg[1] = 0x00;
    _msg_len = 3;
    set_xor();
}


void DccPkt::make_idle()
{
    xassert(msg_max >= 3);
    _msg[0] = 0xff;
    _msg[1] = 0x00;
    _msg_len = 3;
    set_xor();
}


void DccPkt::make_speed(int adrs, int speed)
{
    xassert(msg_max >= 5);
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(speed_min <= speed && speed <= speed_max);

    int idx = address(adrs);
    xassert(idx == 1 || idx == 2);

    _msg[idx++] = 0x3f;
    xassert(idx == 2 || idx == 3);

    _msg[idx++] = dcc_speed(speed);
    xassert(idx == 3 || idx == 4);

    _msg_len = idx + 1; // 4 or 5

    set_xor();
}


// create a function message with a single function turned on (or off)
void DccPkt::make_func(int adrs, int func, bool on)
{
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(function_min <= func && func <= function_max);

    int idx = address(adrs);
    xassert(idx == 1 || idx == 2);

    int f_bit = (on ? 1 : 0);

    if (func <= 12) {

        if (func == 0)
            _msg[idx++] = 0x80 | (f_bit << 4);
        else if (func <= 4)
            _msg[idx++] = 0x80 | (f_bit << (func - 1));
        else if (func <= 8)
            _msg[idx++] = 0xb0 | (f_bit << (func - 5));
        else if (func <= 12)
            _msg[idx++] = 0xa0 | (f_bit << (func - 9));
        xassert(idx == 2 || idx == 3);

    } else { // (func <= 68)

        if (func <= 20) {
            _msg[idx++] = 0xde;
            _msg[idx++] = f_bit << (func - 13);
        } else if (func <= 28) {
            _msg[idx++] = 0xdf;
            _msg[idx++] = f_bit << (func - 21);
        } else if (func <= 36) {
            _msg[idx++] = 0xd8;
            _msg[idx++] = f_bit << (func - 29);
        } else if (func <= 44) {
            _msg[idx++] = 0xd9;
            _msg[idx++] = f_bit << (func - 37);
        } else if (func <= 52) {
            _msg[idx++] = 0xda;
            _msg[idx++] = f_bit << (func - 45);
        } else if (func <= 60) {
            _msg[idx++] = 0xdb;
            _msg[idx++] = f_bit << (func - 53);
        } else if (func <= 68) {
            _msg[idx++] = 0xdc;
            _msg[idx++] = f_bit << (func - 61);
        }
        xassert(function_max == 68);
        xassert(idx == 3 || idx == 4);

    }

    // _msg[idx] is where the xor byte will go
    xassert(2 <= idx && idx <= 4);
    _msg_len = idx + 1;
    set_xor();
}


// set or clear a bit in a function message
void DccPkt::set_func(int func, bool on)
{
    xassert(function_min <= func && func <= function_max);

    int idx = address_size();
    xassert(idx == 1 || idx == 2);

    uint8_t bit = 0;

    if (func <= 12) {

        if (func == 0)
            bit = 1 << 4;
        else if (func <= 4)
            bit = 1 << (func - 1);
        else if (func <= 8)
            bit = 1 << (func - 5);
        else if (func <= 12)
            bit = 1 << (func - 9);

    } else { // (func <= 68)

        if (func <= 20)
            bit = 1 << (func - 13);
        else if (func <= 28)
            bit = 1 << (func - 21);
        else if (func <= 36)
            bit = 1 << (func - 29);
        else if (func <= 44)
            bit = 1 << (func - 37);
        else if (func <= 52)
            bit = 1 << (func - 45);
        else if (func <= 60)
            bit = 1 << (func - 53);
        else if (func <= 68)
            bit = 1 << (func - 61);

        idx++;

    }
    xassert(function_max == 68);

    if (on)
        _msg[idx] |= bit;
    else
        _msg[idx] &= ~bit;

    set_xor();
}


void DccPkt::make_ops_write_cv(int adrs, int cv_num, uint8_t cv_val)
{
    xassert(msg_max >= 6); // for long address

    int idx = 0;

    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);

    // cv_num is 1..1024, will be encoded in messages as 0..1023
    cv_num--;

    idx = address(adrs);
    xassert(idx == 1 || idx == 2);

    _msg[idx++] = 0xec | (cv_num >> 8);
    _msg[idx++] = cv_num;
    _msg[idx++] = cv_val;
    xassert(idx == 4 || idx == 5);

    _msg_len = idx + 1;
    set_xor();
}


void DccPkt::make_ops_write_cv_bit(int adrs, int cv_num, int bit_num, int bit_val)
{
    xassert(msg_max >= 6); // for long address
    xassert(address_min <= adrs && adrs <= address_max);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    // cv_num is 1..1024, will be encoded in messages as 0..1023
    cv_num--;

    int idx = address(adrs);
    xassert(idx == 1 || idx == 2);

    _msg[idx++] = 0xe8 | (cv_num >> 8);
    _msg[idx++] = cv_num;
    _msg[idx++] = 0xf0 | (bit_val << 3) | bit_num;
    xassert(idx == 4 || idx == 5);

    _msg_len = idx + 1;
    set_xor();
}


void DccPkt::make_svc_write_cv(int cv_num, uint8_t cv_val)
{
    xassert(msg_max >= 4);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);

    // cv_num is 1..1024, will be encoded in messages as 0..1023
    cv_num--;

    _msg[0] = 0x7c | (cv_num >> 8);
    _msg[1] = cv_num;
    _msg[2] = cv_val;
    _msg_len = 4;
    set_xor();
}


void DccPkt::make_svc_write_cv_bit(int cv_num, int bit_num, int bit_val)
{
    xassert(msg_max >= 4);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    // cv_num is 1..1024, will be encoded in messages as 0..1023
    cv_num--;

    _msg[0] = 0x78 | (cv_num >> 8);
    _msg[1] = cv_num;
    _msg[2] = 0xf0 | (bit_val << 3) | bit_num;
    _msg_len = 4;
    set_xor();
}


void DccPkt::make_svc_verify_cv(int cv_num, uint8_t cv_val)
{
    xassert(msg_max >= 4);

    // cv_num is 1..1024, will be encoded in messages as 0..1023
    cv_num--;

    _msg[0] = 0x74 | (cv_num >> 8);
    _msg[1] = cv_num;
    _msg[2] = cv_val;
    _msg_len = 4;
    set_xor();
}


void DccPkt::make_svc_verify_cv_bit(int cv_num, int bit_num, int bit_val)
{
    xassert(msg_max >= 4);
    xassert(cv_num_min <= cv_num && cv_num <= cv_num_max);
    xassert(0 <= bit_num && bit_num <= 7);
    xassert(bit_val == 0 || bit_val == 1);

    // cv_num is 1..1024, will be encoded in messages as 0..1023
    cv_num--;

    _msg[0] = 0x78 | (cv_num >> 8);
    _msg[1] = cv_num;
    _msg[2] = 0xe0 | (bit_val << 3) | bit_num;
    _msg_len = 4;
    set_xor();
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
