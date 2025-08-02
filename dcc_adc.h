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
        
        uint16_t avg_raw(uint cnt) const;
        uint16_t short_raw() const;
        uint16_t long_raw() const;

        static uint16_t raw_to_mv(uint16_t raw);
        static uint16_t mv_to_ma(uint16_t mv);

        static const uint32_t clock_rate = 48000000;
        static const uint32_t sample_rate = 10000; // 100 usec

        static const uint avg_max = sample_rate / 60; // 1 cycle of 60 Hz noise
        uint16_t _avg[avg_max];
        uint _avg_idx;

        static const uint short_cnt = 16;

        static const uint long_cnt = avg_max;

        uint _err_cnt;

#ifdef INCLUDE_LOG
        static const uint log_max = sample_rate; // 1 sec
        uint16_t _log[log_max];
        uint _log_idx;
#endif

}; // class DccAdc
