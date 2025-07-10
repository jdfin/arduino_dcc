#pragma once

#include <Arduino.h>


class DccPkt
{

    public:

        DccPkt() : _msg_len(0)
        {
        }

        ~DccPkt()
        {
        }

        void msg_len(int new_len);

        int msg_len() const
        {
            return _msg_len;
        }

        uint8_t data(int idx) const;

        static const int address_invalid = -1;
        int address() const;
        int address(int adrs);
        int address_size();

        void set_xor();

        enum Command {
            RESET,
            SPEED_128,
            FUNC_0_4,
            FUNC_5_8,
            FUNC_9_12,
            FUNC_13_20,
            FUNC_21_28,
            FUNC_29_36,
            FUNC_37_44,
            FUNC_45_52,
            FUNC_53_60,
            FUNC_61_68,
            UNKNOWN
        };

        Command command() const;

        static const int address_min = 1; // 0 is broadcast
        static const int address_max = 10239; // 0x27ff

        static const int speed_min = -127;
        static const int speed_max = 127;

        static const int function_min = 0;
        static const int function_max = 68;

        static const int cv_num_min = 1;
        static const int cv_num_max = 1024;

        // dcc speed is sign+magnitude
        static uint8_t dcc_speed(int int_speed);
        static int int_speed(uint8_t dcc_speed);

        static Command dcc_function(int function);

        bool make_raw(const uint8_t *msg, int msg_len);
        void make_reset();
        void make_idle();
        void make_speed(int adrs, int speed);

        void make_func(int adrs, int func, bool on);
        void set_func(int func, bool on);

        void make_ops_write_cv(int adrs, int cv_num, uint8_t cv_val);
        void make_ops_write_cv_bit(int adrs, int cv_num, int bit_num, int bit_val);

        void make_svc_write_cv(int cv_num, uint8_t cv_val);
        void make_svc_write_cv_bit(int cv_num, int bit_num, int bit_val);
        void make_svc_verify_cv(int cv_num, uint8_t cv_val);
        void make_svc_verify_cv_bit(int cv_num, int bit_num, int bit_val);

        static bool is_svc_direct(const uint8_t *msg, int msg_len);

        char *dump(char *buf, int buf_len) const;
        char *show(char *buf, int buf_len) const;

        static const int ops_preamble_bits = 16;

        // DCC Spec 9.2.3, section E ("long preamble")
        static const int svc_preamble_bits = 20;

    private:

        static const int msg_max = 8;

        uint8_t _msg[msg_max];

        int _msg_len;

}; // DccPkt
