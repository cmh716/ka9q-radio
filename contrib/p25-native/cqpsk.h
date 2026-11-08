#ifndef KA9Q_P25_CQPSK_H
#define KA9Q_P25_CQPSK_H

#include "c4fm.h"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>

class p25_cqpsk_decoder_impl {
public:
    p25_cqpsk_decoder_impl(p25_phase1_pcm_callback callback, void *user_data);
    ~p25_cqpsk_decoder_impl();

    p25_cqpsk_decoder_impl(const p25_cqpsk_decoder_impl&) = delete;
    p25_cqpsk_decoder_impl& operator=(const p25_cqpsk_decoder_impl&) = delete;

    int reset();
    int set_expected_nac(uint16_t nac);
    int process(const std::complex<float> *samples, size_t count);
    int process_interleaved_f32(const float *samples, size_t count);
    int process_s16be(const uint8_t *samples, size_t count);
    int get_stats(uint64_t *complex_samples, uint64_t *symbols,
                  p25_phase1_stats *phase1_stats) const;
    uint64_t sync_count() const { return dibit_mapper_.sync_count(); }
    float frequency_offset_hz() const { return fll_.frequency_offset_hz(); }

private:
    static constexpr size_t kTimingSamplesPerSymbol = 5;
    static constexpr size_t kTimingDelaySize = 2 * kTimingSamplesPerSymbol;

    std::complex<float> timing_interpolate(const std::complex<float> *input,
                                           float mu) const;
    std::complex<float> timing_update();
    std::complex<float> costas_process(std::complex<float> sample);
    int process_symbol(std::complex<float> sample);

    p25_band_edge_fll fll_;
    p25_dibit_mapper dibit_mapper_;
    p25_phase1_decoder *phase1_;
    std::complex<float> decimator_history_[9]{};
    std::complex<float> if_history_[49]{};
    std::array<std::complex<float>, 2 * kTimingDelaySize> timing_delay_{};
    std::complex<float> previous_diff_sample_{};
    size_t decimator_position_ = 0;
    size_t if_position_ = 0;
    size_t timing_position_ = 0;
    uint64_t input_samples_ = 0;
    uint64_t symbols_ = 0;
    double agc_power_ = 0.0;
    float mu_ = 0.0f;
    float omega_ = 5.0f;
    float last_symbol_i_ = 0.0f;
    float last_symbol_q_ = 0.0f;
    float costas_phase_ = 0.0f;
    float costas_frequency_ = 0.0f;
    float costas_alpha_ = 0.0f;
    float costas_beta_ = 0.0f;
    bool have_previous_diff_sample_ = false;
};

#endif
