#pragma once

#include <Arduino.h>


class DccPkt
{

    public:

        DccPkt(const uint8_t *msg=nullptr, int msg_len=0);

        ~DccPkt()
        {
            memset(_msg, 0, msg_max);
            _msg_len = 0;
        }

        void msg_len(int new_len);

        int msg_len() const
        {
            return _msg_len;
        }

        uint8_t data(int idx) const;

        static const int address_invalid = INT_MAX;
        int address() const;
        virtual int address(int adrs);
        int address_size() const;

        void set_xor();

        static const int address_min = 1;           // 0 is broadcast
        static const int address_short_max = 127;   // 0x7f
        static const int address_max = 10239;       // 0x27ff
        static const int address_inv = INT_MAX;

        static const int speed_min = -127;
        static const int speed_max = 127;

        static const int function_min = 0;
        static const int function_max = 28;

        static const int cv_num_min = 1;
        static const int cv_num_max = 1024;
        static const int cv_num_inv = INT_MAX;

        // -127..128, or 0..255
        static const int cv_val_min = -127;
        static const int cv_val_max = 255;
        static const int cv_val_inv = INT_MAX;

        static bool is_svc_direct(const uint8_t *msg, int msg_len);

        char *dump(char *buf, int buf_len) const;
        char *show(char *buf, int buf_len) const;

        // DCC Spec 9.2, section A ("preamble")
        static const int ops_preamble_bits = 14;

        // DCC Spec 9.2.3, section E ("long preamble")
        static const int svc_preamble_bits = 20;

    protected:

        static const int msg_max = 8;

        uint8_t _msg[msg_max];

        int _msg_len;

}; // DccPkt


// 2.1 - Address Partitions - Idle Packet
class DccPktIdle : public DccPkt
{
    public:
        DccPktIdle();
};


// 2.3.1.1 - Decoder Control
class DccPktReset : public DccPkt
{
    public:
        DccPktReset();
};


// 2.3.2.1 - 128 Speed Step Control
class DccPktSpeed128 : public DccPkt
{
    public:
        DccPktSpeed128(int adrs=3, int speed=0);
        virtual int address(int adrs) override;
        int speed() const;
        void speed(int speed);
    private:
        void refresh(int adrs, int speed);
        static uint8_t int_to_dcc(int speed_int);
        static int dcc_to_int(uint8_t speed_dcc);
};


// 2.3.4 - Function Group One (F0-F4)
class DccPktFunc0 : public DccPkt
{
    public:
        DccPktFunc0(int adrs=3);
        virtual int address(int adrs) override;
        bool f(int num) const;
        void f(int num, bool on);
    private:
        static const int f_min = 0;
        static const int f_max = 4;
        void refresh(int adrs, uint8_t funcs=0);
        uint8_t funcs() const;
};


// 2.3.5 - Function Group Two (S-bit=1, F5-F8)
class DccPktFunc5 : public DccPkt
{
    public:
        DccPktFunc5(int adrs=3);
        virtual int address(int adrs) override;
        bool f(int num) const;
        void f(int num, bool on);
    private:
        static const int f_min = 5;
        static const int f_max = 8;
        void refresh(int adrs, uint8_t funcs=0);
        uint8_t funcs() const;
};


// 2.3.5 - Function Group Two (S-bit=0, F9-F12)
class DccPktFunc9 : public DccPkt
{
    public:
        DccPktFunc9(int adrs=3);
        virtual int address(int adrs) override;
        bool f(int num) const;
        void f(int num, bool on);
    private:
        static const int f_min = 9;
        static const int f_max = 12;
        void refresh(int adrs, uint8_t funcs=0);
        uint8_t funcs() const;
};


// 2.3.6.5 - F13-F20 Function Control
class DccPktFunc13 : public DccPkt
{
    public:
        DccPktFunc13(int adrs=3);
        virtual int address(int adrs) override;
        bool f(int num) const;
        void f(int num, bool on);
    private:
        static const int f_min = 13;
        static const int f_max = 20;
        void refresh(int adrs, uint8_t funcs=0);
        uint8_t funcs() const;
};


// 2.3.6.6 - F21-F28 Function Control
class DccPktFunc21 : public DccPkt
{
    public:
        DccPktFunc21(int adrs=3);
        virtual int address(int adrs) override;
        bool f(int num) const;
        void f(int num, bool on);
    private:
        static const int f_min = 21;
        static const int f_max = 28;
        void refresh(int adrs, uint8_t funcs=0);
        uint8_t funcs() const;
};


// 2.3.7.3 - Configuration Variable Access - Long Form (write byte)
class DccPktOpsWriteCv : public DccPkt
{
    public:
        DccPktOpsWriteCv(int adrs=3, int cv_num=1, uint8_t cv_val=0);
        virtual int address(int adrs) override;
        void cv(int cv_num, uint8_t cv_val); // set in message
    private:
        void refresh(int adrs, int cv_num, uint8_t cv_val);
        int cv_num() const; // get from message
        uint8_t cv_val() const; // get from message
};


// 2.3.7.3 - Configuration Variable Access - Long Form (bit manipulation)
class DccPktOpsWriteBit : public DccPkt
{
    public:
        DccPktOpsWriteBit();
        DccPktOpsWriteBit(int adrs, int cv_num, int bit_num, int bit_val);
        virtual int address(int adrs) override;
        void cv_bit(int cv_num, int bit_num, int bit_val);
    private:
        void refresh(int adrs, int cv_num, int bit_num, int bit_val);
        int cv_num() const;
        int bit_num() const;
        int bit_val() const;
};


// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcWriteCv : public DccPkt
{
    public:
        DccPktSvcWriteCv(int cv_num=1, uint8_t cv_val=0);
        void set_cv(int cv_num, uint8_t cv_val);
};


// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcWriteBit : public DccPkt
{
    public:
        DccPktSvcWriteBit(int cv_num=1, int bit_num=0, int bit_val=0);
        void set_cv_bit(int cv_num, int bit_num, int bit_val);
};


// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcVerifyCv : public DccPkt
{
    public:
        DccPktSvcVerifyCv(int cv_num=1, uint8_t cv_val=0);
        void set_cv_num(int cv_num);
        void set_cv_val(uint8_t cv_val);
};


// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcVerifyBit : public DccPkt
{
    public:
        DccPktSvcVerifyBit(int cv_num=1, int bit_num=0, int bit_val=0);
        void set_cv_bit(int cv_num, int bit_num=0, int bit_val=0);
        void set_bit(int bit_num, int bit_val);
};
