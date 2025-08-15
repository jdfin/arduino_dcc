#pragma once

#include <Arduino.h>

#undef INCLUDE_LOG


class DccAdc
{

    public:

        DccAdc(int gpio);
        ~DccAdc();

        void start();
        void stop();

        void loop();

        uint16_t short_ma() const;
        uint16_t long_ma() const;

        static constexpr bool logging()
        {
#ifdef INCLUDE_LOG
            return true;
#else
            return false;
#endif
        }

        void log_reset();
        void log_show();

    private:

        int _gpio;
        
        uint16_t avg_raw(int cnt) const;
        uint16_t short_raw() const;
        uint16_t long_raw() const;

        static uint16_t raw_to_mv(uint16_t raw);
        static uint16_t mv_to_ma(uint16_t mv);

        static const uint32_t clock_rate = 48000000;
        static const uint32_t sample_rate = 10000; // 10 KHz = 100 usec per sample

        static const int avg_max = sample_rate / 60; // 1 cycle of 60 Hz noise
        uint16_t _avg[avg_max];
        int _avg_idx;

        static const int short_cnt = 16;

        static const int long_cnt = avg_max;

        int _err_cnt;

#ifdef INCLUDE_LOG
        static const int log_max = 1 * sample_rate; // 1 sec
        uint16_t _log[log_max];
        int _log_idx;
#endif

}; // class DccAdc
