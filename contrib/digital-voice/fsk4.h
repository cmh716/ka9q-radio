#ifndef KA9Q_P25_FSK4_H
#define KA9Q_P25_FSK4_H

#include <cstddef>

class p25_fsk4_demod {
public:
    p25_fsk4_demod(float sample_rate_hz, float symbol_rate_hz);

    void reset();
    /* symbols must have room for at least count floats. */
    size_t process(const float *input, size_t count, float *symbols);
    bool take_frequency_correction(float *correction);
    double fine_frequency_correction() const {
        return fine_frequency_correction_;
    }

private:
    double symbol_time_;
    double symbol_clock_;
    double symbol_spread_;
    double fine_frequency_correction_;
    double coarse_frequency_correction_;
    float history_[8]{};
    size_t history_last_ = 0;
};

#endif
