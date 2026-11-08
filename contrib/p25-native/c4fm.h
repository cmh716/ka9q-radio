#ifndef KA9Q_P25_C4FM_H
#define KA9Q_P25_C4FM_H

#include "fsk4.h"
#include "phase1.h"
#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>

class p25_band_edge_fll {
public:
    explicit p25_band_edge_fll(float bandwidth_divisor = 350.0f);

    void reset();
    std::complex<float> process(std::complex<float> sample);
    float frequency_offset_hz() const;

private:
    static constexpr size_t kFilterSize = 11;
    std::array<std::complex<float>, kFilterSize> lower_taps_{};
    std::array<std::complex<float>, kFilterSize> upper_taps_{};
    std::array<std::complex<float>, kFilterSize> history_{};
    size_t position_ = 0;
    float phase_ = 0.0f;
    float frequency_ = 0.0f;
    float alpha_ = 0.0f;
    float beta_ = 0.0f;
};

class p25_dibit_mapper {
public:
    uint8_t process(uint8_t raw_dibit);
    void reset();
    uint64_t sync_count() const { return sync_count_; }

private:
    uint64_t raw_dibit_accum_ = 0;
    uint64_t sync_count_ = 0;
    size_t map_index_ = 0;
};

class p25_c4fm_decoder_impl {
public:
    p25_c4fm_decoder_impl(p25_phase1_pcm_callback callback, void *user_data);
    ~p25_c4fm_decoder_impl();

    p25_c4fm_decoder_impl(const p25_c4fm_decoder_impl&) = delete;
    p25_c4fm_decoder_impl& operator=(const p25_c4fm_decoder_impl&) = delete;

    int reset();
    int set_expected_nac(uint16_t nac);
    /* Accepts 48 kHz complex float IQ; calls on one instance must be serialized. */
    int process(const std::complex<float> *samples, size_t count);
    int process_interleaved_f32(const float *samples, size_t count);
    /* Accepts count interleaved I/Q samples as signed 16-bit big-endian. */
    int process_s16be(const uint8_t *samples, size_t count);
    int get_stats(uint64_t *complex_samples, uint64_t *symbols,
                  p25_phase1_stats *phase1_stats) const;
    uint64_t sync_count() const { return dibit_mapper_.sync_count(); }
    float frequency_offset_hz() const;

private:
    p25_fsk4_demod fsk4_;
    p25_band_edge_fll fll_;
    p25_dibit_mapper dibit_mapper_;
    p25_phase1_decoder *phase1_;
    std::complex<float> decimator_history_[9]{};
    std::complex<float> if_history_[49]{};
    std::complex<float> previous_sample_{};
    float c4fm_history_[45]{};
    size_t decimator_position_ = 0;
    size_t if_position_ = 0;
    size_t c4fm_position_ = 0;
    uint64_t input_samples_ = 0;
    uint64_t symbols_ = 0;
    double agc_power_ = 0.0;
    bool have_previous_sample_ = false;
};

#endif
