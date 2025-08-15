#pragma once

#include <Arduino.h>
#include "dcc_pkt.h"
#include "dcc_cv.h"


class DccThrottle
{

    public:

        DccThrottle();
        ~DccThrottle();

        void address(int address);
        void speed(int speed);
        void function(int func, bool on);

        void write_cv(int cv_num, uint8_t cv_val);
        void write_bit(int cv_num, int bit_num, int bit_val);

        DccPkt next_packet();

        void show();

    private:

        DccPktSpeed128  _pkt_speed;
        DccPktFunc0     _pkt_func_0;
        DccPktFunc5     _pkt_func_5;
        DccPktFunc9     _pkt_func_9;
        DccPktFunc13    _pkt_func_13;
        DccPktFunc21    _pkt_func_21;

        // where in packet sequence we are
        static const int seq_max = 10;
        int _seq; // _seq = 0..9

        DccPktOpsWriteCv _pkt_write_cv;
        static const int write_cv_send_cnt = 5; // how many times to send it
        int _write_cv_cnt; // times left to send it (5, 4, ... 1, 0)

        DccPktOpsWriteBit _pkt_write_bit;
        static const int write_bit_send_cnt = 5; // how many times to send it
        int _write_bit_cnt; // times left to send it (5, 4, ... 1, 0)

}; // class DccThrottle
