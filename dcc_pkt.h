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

        static const uint address_invalid = UINT_MAX;
        uint address() const;
        virtual uint address(uint adrs);
        uint address_size() const;

        void set_xor();

        static const int address_min = 1; // 0 is broadcast
        static const int address_max = 10239; // 0x27ff

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
        DccPktSpeed128(uint adrs=3, int speed=0);
        virtual uint address(uint adrs);
        int speed() const;
        void speed(int speed);
    private:
        void refresh(uint adrs, int speed);
        static uint8_t int_to_dcc(int speed_int);
        static int dcc_to_int(uint8_t speed_dcc);
};


// 2.3.4 - Function Group One (F0-F4)
class DccPktFunc0 : public DccPkt
{
    public:
        DccPktFunc0(uint adrs=3);
        virtual uint address(uint adrs);
        bool f(uint num) const;
        void f(uint num, bool on);
    private:
        void refresh(uint adrs, uint8_t funcs=0);
        uint8_t funcs() const;
};


// 2.3.5 - Function Group Two (S-bit=1, F5-F8)
class DccPktFunc5 : public DccPkt
{
    public:
        DccPktFunc5(uint adrs=3);
        virtual uint address(uint adrs);
        bool f(uint num) const;
        void f(uint num, bool on);
    private:
        void refresh(uint adrs, uint8_t funcs=0);
        uint8_t funcs() const;
};


// 2.3.5 - Function Group Two (S-bit=0, F9-F12)
class DccPktFunc9 : public DccPkt
{
    public:
        DccPktFunc9(uint adrs=3);
        virtual uint address(uint adrs);
        bool f(uint num) const;
        void f(uint num, bool on);
    private:
        void refresh(uint adrs, uint8_t funcs=0);
        uint8_t funcs() const;
};


// 2.3.6.5 - F13-F20 Function Control
class DccPktFunc13 : public DccPkt
{
    public:
        DccPktFunc13(uint adrs=3);
        virtual uint address(uint adrs);
        bool f(uint num) const;
        void f(uint num, bool on);
    private:
        void refresh(uint adrs, uint8_t funcs=0);
        uint8_t funcs() const;
};


// 2.3.6.6 - F21-F28 Function Control
class DccPktFunc21 : public DccPkt
{
    public:
        DccPktFunc21(uint adrs=3);
        virtual uint address(uint adrs);
        bool f(uint num) const;
        void f(uint num, bool on);
    private:
        void refresh(uint adrs, uint8_t funcs=0);
        uint8_t funcs() const;
};


// 2.3.7.3 - Configuration Variable Access - Long Form (write byte)
class DccPktOpsWriteCv : public DccPkt
{
    public:
        DccPktOpsWriteCv(uint adrs=3, uint cv_num=1, uint8_t cv_val=0);
        virtual uint address(uint adrs);
        void cv(uint cv_num, uint8_t cv_val); // set in message
    private:
        void refresh(uint adrs, uint cv_num, uint8_t cv_val);
        uint cv_num() const; // get from message
        uint8_t cv_val() const; // get from message
};


// 2.3.7.3 - Configuration Variable Access - Long Form (bit manipulation)
class DccPktOpsWriteBit : public DccPkt
{
    public:
        DccPktOpsWriteBit(uint adrs, uint cv_num, uint bit_num, uint bit_val);
        virtual uint address(uint adrs);
        void cv_bit(uint cv_num, uint bit_num, uint bit_val);
    private:
        void refresh(uint adrs, uint cv_num, uint bit_num, uint bit_val);
        uint cv_num() const;
        uint bit_num() const;
        uint bit_val() const;
};


// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcWriteCv : public DccPkt
{
    public:
        DccPktSvcWriteCv(uint cv_num=1, uint8_t cv_val=0);
        void set_cv(uint cv_num, uint8_t cv_val);
};


// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcWriteBit : public DccPkt
{
    public:
        DccPktSvcWriteBit(uint cv_num=1, uint bit_num=0, uint bit_val=0);
        void set_cv_bit(uint cv_num, uint bit_num, uint bit_val);
};


// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcVerifyCv : public DccPkt
{
    public:
        DccPktSvcVerifyCv(uint cv_num=1, uint8_t cv_val=0);
        void set_cv_num(uint cv_num);
        void set_cv_val(uint8_t cv_val);
};


// Std 9.2.3, Section E, Service Mode Instruction Packets for Direct Mode
class DccPktSvcVerifyBit : public DccPkt
{
    public:
        DccPktSvcVerifyBit(uint cv_num=1, uint bit_num=0, uint bit_val=0);
        void set_cv_bit(uint cv_num, uint bit_num=0, uint bit_val=0);
        void set_bit(uint bit_num, uint bit_val);
};
