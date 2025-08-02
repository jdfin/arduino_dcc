
#include <Arduino.h>
#include "xassert.h"

#include "dcc_pkt.h"
#include "dcc_throttle.h"


DccThrottle::DccThrottle() :
    _pkt_speed(),
    _pkt_func_0(),
    _pkt_func_5(),
    _pkt_func_9(),
    _pkt_func_13(),
    _pkt_func_21(),
    _seq(0),
    _pkt_write_cv(),
    _write_cv_cnt(0)
{
}


DccThrottle::~DccThrottle()
{
}


void DccThrottle::address(uint address)
{
    _pkt_speed.address(address);
    _pkt_func_0.address(address);
    _pkt_func_5.address(address);
    _pkt_func_9.address(address);
    _pkt_func_13.address(address);
    _pkt_func_21.address(address);
    _seq = 0;
    _pkt_write_cv.address(address);
}


void DccThrottle::speed(int speed)
{
    _pkt_speed.speed(speed);
    _seq &= ~1; // back up one if a function packet is next
}


void DccThrottle::function(uint num, bool on)
{
    xassert(DccPkt::function_min <= num && num <= DccPkt::function_max);

    if (num <= 4) {
        _pkt_func_0.f(num, on);
        _seq = 1;
    } else if (num <= 8) {
        _pkt_func_5.f(num, on);
        _seq = 3;
    } else if (num <= 12) {
        _pkt_func_9.f(num, on);
        _seq = 5;
    } else if (num <= 20) {
        _pkt_func_13.f(num, on);
        _seq = 7;
    } else { // num <= 28
        _pkt_func_21.f(num, on);
        _seq = 9;
    }
}


void DccThrottle::write_cv(uint cv_num, uint8_t cv_val)
{
    _pkt_write_cv.cv(cv_num, cv_val);
    _write_cv_cnt = write_cv_send_cnt;
}


// 0. Speed     1. F0-F4
// 2. Speed     3. F5-F8
// 4. Speed     5. F9-F12
// 6. Speed     7. F13-F20
// 8. Speed     9. F21-F28
DccPkt DccThrottle::next_packet()
{
    xassert(/* 0 <= _seq && */ _seq < seq_max);

    if (_write_cv_cnt > 0) {
        _write_cv_cnt--;
        return _pkt_write_cv;
    }

    uint seq = _seq;

    if (++_seq >= seq_max)
        _seq = 0;

    if ((seq & 1) == 0) // if _seq even
        return _pkt_speed;
    else if (seq == 1)
        return _pkt_func_0;
    else if (seq == 3)
        return _pkt_func_5;
    else if (seq == 5)
        return _pkt_func_9;
    else if (seq == 7)
        return _pkt_func_13;
    else
        return _pkt_func_21;
}


void DccThrottle::show()
{
    char buf[80];
    Serial.printf("%s\n", _pkt_speed.show(buf, sizeof(buf)));
    Serial.printf("%s\n", _pkt_func_0.show(buf, sizeof(buf)));
    Serial.printf("%s\n", _pkt_func_5.show(buf, sizeof(buf)));
    Serial.printf("%s\n", _pkt_func_9.show(buf, sizeof(buf)));
    Serial.printf("%s\n", _pkt_func_13.show(buf, sizeof(buf)));
    Serial.printf("%s\n", _pkt_func_21.show(buf, sizeof(buf)));
}
