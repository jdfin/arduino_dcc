#pragma once

#include <Arduino.h>
#include <list>

#include "dcc_adc.h"
#include "dcc_bitstream.h"

#undef INCLUDE_ACK_DBG


class DccThrottle;


class DccCommand
{

    public:

        DccCommand(int sig_gpio, int pwr_gpio, DccAdc& adc);
        ~DccCommand();

        void mode_off();
        void mode_ops();
        void mode_svc_write_cv(uint cv_num, uint8_t cv_val);
        void mode_svc_write_bit(uint cv_num, uint bit_num, uint bit_val);
        void mode_svc_read_cv(uint cv_num);

        enum Mode {
            MODE_OFF,
            MODE_OPS,
            MODE_SVC_WRITE_CV,
            MODE_SVC_READ_CV,
        };

        Mode mode() const { return _mode; }

        // Returns true if service mode operation is done, and
        // result is set true (success) or false (failed)
        bool svc_done(bool& result);
        bool svc_done(bool& result, uint8_t& val);

        void loop();

        DccThrottle *create_throttle();
        void delete_throttle(DccThrottle *throttle);

        void show();

        void show_ack_ma();

    private:

        DccBitstream _bitstream;

        DccAdc& _adc;

        Mode _mode;

        // for MODE_OPS
        std::list<DccThrottle*> _throttles;
        std::list<DccThrottle*>::iterator _next_throttle;
        void loop_ops();

        // for MODE_SVC_*
        int _svc_status; // -1 not done, 0 failed, 1 success
        uint16_t _ack_ma;
        static const uint16_t ack_inc_ma = 60;
#ifdef INCLUDE_ACK_DBG
        uint16_t _ack_dbg_ma[9]; // 0..7 are bits, 8 is byte
#endif

        uint _reset1_cnt;
        uint _reset2_cnt;

        // for MODE_SVC_WRITE_CV
        DccPktSvcWriteCv _pkt_svc_write_cv;
        uint _write_cnt;
        void loop_svc_write();

        // for MODE_SVC_READ_CV
        DccPktSvcVerifyBit _pkt_svc_verify_bit;
        DccPktSvcVerifyCv _pkt_svc_verify_cv;
        uint _verify_bit;
        uint _verify_cnt;
        uint8_t _cv_val;
        void loop_svc_read();
};
