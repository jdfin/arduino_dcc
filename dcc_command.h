#pragma once

#include <Arduino.h>
#include <list>
#include "dcc_adc.h"
#include "dcc_bitstream.h"
#include "dcc_cv.h"

#undef INCLUDE_ACK_DBG


class DccThrottle;


class DccCommand
{

    public:

        DccCommand(int sig_gpio, int pwr_gpio, DccAdc& adc);
        ~DccCommand();

        void mode_off();
        void mode_ops();
        void mode_svc_write_cv(int cv_num, uint8_t cv_val);
        void mode_svc_write_bit(int cv_num, int bit_num, int bit_val);
        void mode_svc_read_cv(int cv_num);
        void mode_svc_read_bit(int cv_num, int bit_num);

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

        int _reset1_cnt;
        int _reset2_cnt;

        // for MODE_SVC_WRITE_CV
        DccPktSvcWriteCv _pkt_svc_write_cv;
        int _write_cnt;
        DccPktSvcWriteBit _pkt_svc_write_bit;
        int _write_bit_cnt;
        void loop_svc_write();

        // for MODE_SVC_READ_CV
        DccPktSvcVerifyBit _pkt_svc_verify_bit;
        DccPktSvcVerifyCv _pkt_svc_verify_cv;
        int _verify_bit;
        int _verify_bit_val; // 0 or 1
        int _verify_cnt;
        int _read_bit; // -1 when doing a byte read, or 0..7 when doing a bit read
        uint8_t _cv_val;
        void loop_svc_read();
};
