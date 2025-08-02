#include <Arduino.h>
#include <list>

#include "hardware/adc.h"

#include "xassert.h"

#include "dcc_adc.h"
#include "dcc_throttle.h"
#include "dcc_command.h"


DccCommand::DccCommand(int sig_gpio, int pwr_gpio, DccAdc& adc) :
    _bitstream(sig_gpio, pwr_gpio),
    _adc(adc),
    _mode(MODE_OFF),
    // _throttles uses default initializer
    _next_throttle(_throttles.begin()),
    // _svc_status set when needed
    // _ack_ma set when needed
    _reset1_cnt(0),
    _reset2_cnt(0),
    _pkt_svc_write_cv(),
    _write_cnt(0),
    _pkt_svc_verify_bit(),
    _pkt_svc_verify_cv(),
    _verify_bit(0),
    _verify_cnt(0),
    _cv_val(0)
{
}


DccCommand::~DccCommand()
{
    for (DccThrottle *throttle : _throttles) {
        _throttles.remove(throttle);
        delete throttle;
    }
    _next_throttle = _throttles.begin();
}


void DccCommand::mode_off()
{
    _mode = MODE_OFF;
    _adc.stop();
    _bitstream.stop();
}


void DccCommand::mode_ops()
{
    _mode = MODE_OPS;
    _bitstream.start_ops();
}


void DccCommand::mode_svc_write_cv(uint cv_num, uint8_t cv_val)
{
    _mode = MODE_SVC_WRITE_CV;

    _svc_status = -1; // "not done"

    _reset1_cnt = 20;
    _write_cnt = 5;
    _reset2_cnt = 5;
    _pkt_svc_write_cv.set_cv(cv_num, cv_val); // validates cv_num

    _adc.start();

    _bitstream.start_svc();
}


void DccCommand::mode_svc_write_bit(uint cv_num, uint bit_num, uint bit_val)
{
    // XXX
}


void DccCommand::mode_svc_read_cv(uint cv_num)
{
    _mode = MODE_SVC_READ_CV;

    _svc_status = -1; // "not done"

    _reset1_cnt = 20;
    _cv_val = 0;
    _pkt_svc_verify_bit.set_cv_bit(cv_num);
    _pkt_svc_verify_cv.set_cv_num(cv_num);

    _adc.start();

    _bitstream.start_svc();
}


bool DccCommand::svc_done(bool& result)
{
    if (_svc_status == -1)
        return false;

    result = (_svc_status == 1);
    return true;
}


bool DccCommand::svc_done(bool& result, uint8_t& val)
{
    if (_svc_status == -1)
        return false;

    result = (_svc_status == 1);

    if (result)
        val = _cv_val;

    return true;
}


void DccCommand::loop()
{
    if (_mode == MODE_OFF) {
        ; // nop
    } else if (_mode == MODE_OPS) {
        loop_ops();
    } else if (_mode == MODE_SVC_WRITE_CV) {
        _adc.loop();
        loop_svc_write();
    } else if (_mode == MODE_SVC_READ_CV) {
        _adc.loop();
        loop_svc_read();
    }
}


void DccCommand::loop_ops()
{
    if (_bitstream.need_packet()) {
        if (_next_throttle != _throttles.end()) {
            _bitstream.send_packet((*_next_throttle)->next_packet());
            _next_throttle++;
            if (_next_throttle == _throttles.end())
                _next_throttle = _throttles.begin();
        }
    }
}


void DccCommand::loop_svc_write()
{
    if (_reset1_cnt > 0) {
        if (_bitstream.need_packet()) {
            _bitstream.send_reset();
            _reset1_cnt--;
            if (_reset1_cnt == 0) {
                // Use the long average adc reading as the baseline for
                // detecting an ack pulse.
                _ack_ma = _adc.long_ma() + ack_inc_ma;
                //Serial.printf("long_ma = %u, ack_ma = %u\n", long_ma, _ack_ma);
            }
        }
    } else {
        // Checking the current needs to be outside the need_packet clauses
        // and is done while the write and subsequent reset packets are going.
        // Use the short average adc reading.
        uint16_t short_ma = _adc.short_ma();
        if (short_ma >= _ack_ma) {
            // Ack! Don't send any more packets, and power off.
#ifndef INCLUDE_ADC_LOG
            _write_cnt = 0;
            _reset2_cnt = 0;
#endif
            _svc_status = 1;
        }
        if (_write_cnt > 0) {
            if (_bitstream.need_packet()) {
                _bitstream.send_packet(_pkt_svc_write_cv);
                _write_cnt--;
            }
        } else if (_reset2_cnt > 0) {
            if (_bitstream.need_packet()) {
                _bitstream.send_reset();
                _reset2_cnt--;
            }
        } else {
            if (_svc_status == -1)
                _svc_status = 0; // failed, timeout
            mode_off();
        }
    }
}


// Before the first call (when starting the read), mode_svc_read_cv() sets:
//   _reset1_cnt to the number of initial resets (20)
//   _cv_val = 0, so this loop can OR-in one bits as they are discovered
//   _svc_status = -1, to indicate the read is in progress
//
// As the loop is repeatedly called:
//   1. it will send out the initial resets
//   2. it will, for each bit 7...0:
//      a. send out five bit-verifies (that the bit is one)
//      b. send out five resets
//      c. and if an ack is received during any of those 10 packets, a one bit
//         is ORed into _cv_val
//   3. after the verify-bit for bit 0, it sends out five byte-verifies for
//      the cv with the built-up _cv_val, then five more resets
//      a. if an ack is received during any of those 10 packets, we are done,
//         _svc_status is set to 1 (success), and calling svc_done() will
//         return "done/success" and the cv_val
//      b. if no ack has been received when the last reset goes out, we are
//         done, _svc_status is set to 0 (failed), and calling svc_done() will
//         return "done/error"

void DccCommand::loop_svc_read()
{
    xassert(_mode == MODE_SVC_READ_CV);

    if (_reset1_cnt > 0) {
        // first 20 resets are going out
        if (_bitstream.need_packet()) {
            _bitstream.send_reset();
            _reset1_cnt--;
            if (_reset1_cnt == 0) {
                // Use the long average adc reading as the baseline for
                // detecting an ack pulse.
                _ack_ma = _adc.long_ma() + ack_inc_ma;
                //Serial.printf("long_ma = %u, ack_ma = %u\n", long_ma, _ack_ma);
                _verify_bit = 7;
                _pkt_svc_verify_bit.set_bit(_verify_bit, 1);
                _verify_cnt = 5;
#ifdef INCLUDE_ACK_DBG
                _ack_dbg_ma[_verify_bit] = _ack_ma;
#endif
            }
        }
        return;
    }

    uint16_t short_ma = _adc.short_ma();
    if (short_ma >= _ack_ma) {
        // Ack!
        if (_verify_bit == 8) {
            // This is the ack for the byte-verify at the end
            _svc_status = 1;
            // If logging adc (for analysis), we keep going to see the
            // full ack. If not logging, we're done.
            if constexpr (!_adc.logging()) {
                _verify_cnt = 0;
                _reset2_cnt = 0;
            }
        } else {
            // This is an ack for a bit-verify
            _cv_val |= (1 << _verify_bit);
        }
    }

    if (_verify_cnt > 0) {

        if (_bitstream.need_packet()) {
            if (_verify_bit == 8)
                _bitstream.send_packet(_pkt_svc_verify_cv);
            else
                _bitstream.send_packet(_pkt_svc_verify_bit);
            _verify_cnt--;
            if (_verify_cnt == 0)
                _reset2_cnt = 5;
        }

    } else if (_reset2_cnt > 0) {

        if (_bitstream.need_packet()) {
            _bitstream.send_reset();
            _reset2_cnt--;
        }

        if (_reset2_cnt == 0) {
            // Get a new long average adc reading and a new ack threshold each
            // time just before sending out the verify packets. The current
            // does not always hold steady through the whole sequence.
            _ack_ma = _adc.long_ma() + ack_inc_ma;
            //Serial.printf("long_ma = %u, ack_ma = %u\n", long_ma, _ack_ma);
        }

    } else {

        // done with 5 verifies and 5 resets for _verify_bit
        if (_verify_bit == 8) {
            // byte verify done
            if (_svc_status == -1)
                _svc_status = 0; // failed, timeout
            mode_off();
        } else if (_verify_bit > 0) {
            _verify_bit--;
            _pkt_svc_verify_bit.set_bit(_verify_bit, 1);
            _verify_cnt = 5;
#ifdef INCLUDE_ACK_DBG
            _ack_dbg_ma[_verify_bit] = _ack_ma;
#endif
        } else { // (_verify_bit == 0)
            // start byte verify
            _verify_bit = 8; // signifies verify byte
            _pkt_svc_verify_cv.set_cv_val(_cv_val);
            _verify_cnt = 5;
#ifdef INCLUDE_ACK_DBG
            _ack_dbg_ma[_verify_bit] = _ack_ma;
#endif
        }
    }

} // void DccCommand::loop_svc_read


DccThrottle *DccCommand::create_throttle()
{
    DccThrottle* throttle = new DccThrottle();
    _throttles.push_back(throttle);
    _next_throttle = _throttles.begin();
    return throttle;
}


void DccCommand::delete_throttle(DccThrottle *throttle)
{
    _throttles.remove(throttle);
    delete throttle;
    _next_throttle = _throttles.begin();
}


void DccCommand::show()
{
    if (_throttles.empty()) {
        Serial.printf("no throttles\n");
    } else {
        for (DccThrottle *throttle : _throttles) {
            Serial.printf("throttle:\n");
            throttle->show();
        }
    }
}


void DccCommand::show_ack_ma()
{
#ifdef INCLUDE_ACK_DBG
    Serial.printf("ack_ma =");
    for (int bit = 7; bit >= 0; bit--)
        Serial.printf(" %u", _ack_dbg_ma[bit]);
    Serial.printf(" %u\n", _ack_dbg_ma[8]);
#else
    //Serial.printf("ACK_DBG not enabled in dcc_command.h\n");
#endif
}
