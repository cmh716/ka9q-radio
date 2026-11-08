#include "p25_native_api.h"

#include "c4fm.h"
#include "cqpsk.h"

#include <new>
#include <stdexcept>
#include <type_traits>
#include <variant>

using decoder_implementation =
    std::variant<p25_c4fm_decoder_impl, p25_cqpsk_decoder_impl>;

struct p25_native_decoder {
    template <typename Decoder>
    p25_native_decoder(std::in_place_type_t<Decoder> type,
                       p25_phase1_pcm_callback callback, void *user_data)
        : implementation(type, callback, user_data)
    {
    }

    decoder_implementation implementation;
};

extern "C" int p25_native_decoder_create(
    p25_native_modulation modulation, p25_phase1_pcm_callback pcm_callback,
    void *user_data, p25_native_decoder **decoder)
{
    if (decoder == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    *decoder = nullptr;
    try {
        switch (modulation) {
        case P25_NATIVE_C4FM:
            *decoder = new p25_native_decoder(
                std::in_place_type<p25_c4fm_decoder_impl>, pcm_callback,
                user_data);
            break;
        case P25_NATIVE_CQPSK:
            *decoder = new p25_native_decoder(
                std::in_place_type<p25_cqpsk_decoder_impl>, pcm_callback,
                user_data);
            break;
        default:
            return P25_PHASE1_INVALID_ARGUMENT;
        }
    } catch (const std::bad_alloc&) {
        return P25_PHASE1_INTERNAL_ERROR;
    } catch (const std::exception&) {
        return P25_PHASE1_INTERNAL_ERROR;
    }
    return P25_PHASE1_OK;
}

extern "C" void p25_native_decoder_destroy(p25_native_decoder *decoder)
{
    delete decoder;
}

extern "C" int p25_native_decoder_reset(p25_native_decoder *decoder)
{
    if (decoder == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    return std::visit([](auto& implementation) {
        return implementation.reset();
    }, decoder->implementation);
}

extern "C" int p25_native_decoder_set_expected_nac(
    p25_native_decoder *decoder, uint16_t nac)
{
    if (decoder == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    return std::visit([nac](auto& implementation) {
        return implementation.set_expected_nac(nac);
    }, decoder->implementation);
}

extern "C" int p25_native_decoder_process_f32(
    p25_native_decoder *decoder, const float *samples, size_t count)
{
    if (decoder == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    return std::visit([samples, count](auto& implementation) {
        return implementation.process_interleaved_f32(samples, count);
    }, decoder->implementation);
}

extern "C" int p25_native_decoder_process_s16be(
    p25_native_decoder *decoder, const uint8_t *samples, size_t count)
{
    if (decoder == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    return std::visit([samples, count](auto& implementation) {
        return implementation.process_s16be(samples, count);
    }, decoder->implementation);
}

extern "C" int p25_native_decoder_get_stats(
    const p25_native_decoder *decoder, uint64_t *complex_samples,
    uint64_t *symbols, p25_phase1_stats *phase1_stats)
{
    if (decoder == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    return std::visit([complex_samples, symbols, phase1_stats](const auto& implementation) {
        return implementation.get_stats(complex_samples, symbols, phase1_stats);
    }, decoder->implementation);
}

extern "C" int p25_native_decoder_get_sync_count(
    const p25_native_decoder *decoder, uint64_t *sync_count)
{
    if (decoder == nullptr || sync_count == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    *sync_count = std::visit([](const auto& implementation) {
        return implementation.sync_count();
    }, decoder->implementation);
    return P25_PHASE1_OK;
}

extern "C" int p25_native_decoder_get_frequency_offset(
    const p25_native_decoder *decoder, float *offset_hz)
{
    if (decoder == nullptr || offset_hz == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    *offset_hz = std::visit([](const auto& implementation) {
        return implementation.frequency_offset_hz();
    }, decoder->implementation);
    return P25_PHASE1_OK;
}
