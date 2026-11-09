// P25 Phase 1 frame layer: NID framing (OP25 p25_framer), encryption
// detection from the HDU and LDU2 ESS, and IMBE voice decoding
// Adapted from OP25 p25p1_fdma.cc; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include "p25_voice.h"

#include <vector>

#include "ezpwd/rs"
#include "imbe_vocoder/imbe_vocoder.h"
#include "op25_golay.h"
#include "op25_hamming.h"
#include "op25_imbe_frame.h"
#include "p25_framer.h"

namespace {

enum : uint32_t { DUID_HDU = 0, DUID_TDU = 3, DUID_LDU1 = 5, DUID_LDU2 = 10, DUID_TDULC = 15 };
constexpr uint8_t ALGID_CLEAR = 0x80;

// Shortened Golay(18,6,8): only 64 codewords, so nearest-codeword search is cheap
uint8_t golay_18_6_decode(uint32_t cw)
{
  uint8_t best = 0;
  int best_distance = 99;
  for (uint32_t d = 0; d < 64; d++) {
    int const distance = __builtin_popcount(golay_24_encode(d) ^ cw);
    if (distance < best_distance) {
      best_distance = distance;
      best = static_cast<uint8_t>(d);
    }
  }
  return best;
}

// Selects a decoder's fixed-point Overflow/Carry flags for the current thread
class basic_op_scope {
public:
  explicit basic_op_scope(BasicOpContext *context) : previous_(basic_op_set_context(context)) {}
  ~basic_op_scope() { basic_op_set_context(previous_); }
private:
  BasicOpContext *previous_;
};

} // namespace

struct p25_voice::impl {
  p25_framer framer;
  BasicOpContext arithmetic{0, 0};
  std::unique_ptr<imbe_vocoder> vocoder;
  ezpwd::RS<63, 55> rs_ess;   // RS(24,16,9) shortened
  ezpwd::RS<63, 47> rs_hdu;   // RS(36,20,17) shortened
  p25_native_pcm_callback callback;
  void *user_data;
  p25_native_stats stats{};
  // Assume clear (as OP25 does) until an HDU or LDU2 says otherwise
  bool encrypted = false;

  void set_algid(uint8_t algid) { encrypted = algid != ALGID_CLEAR; }

  void read_hdu(bit_vector const& frame)
  {
    std::vector<uint8_t> hb(63, 0);
    size_t k = 0;
    for (size_t i = 0; i < 36; i++) {
      uint32_t cw = 0;
      for (size_t j = 0; j < 18; j++)
        cw = (cw << 1) | frame[hdu_codeword_bits[k++]];
      hb[27 + i] = golay_18_6_decode(cw);
    }
    int const errors = rs_hdu.decode(hb);
    if (errors < 0 || errors > 8)
      return;
    // MI is hexbits 27-38; MFID 8 bits, then ALGID 8 bits
    set_algid(static_cast<uint8_t>(((hb[40] & 0x0f) << 4) | (hb[41] >> 2)));
  }

  void read_ess(bit_vector const& frame)
  {
    std::vector<uint8_t> hb(63, 0);
    size_t index = 0;
    for (size_t block = 0; block < 24; block++) {
      uint32_t cw = 0;
      for (size_t bit = 0; bit < 10; bit++)
        cw = (cw << 1) + frame[imbe_ldu_ls_data_bits[index++]];
      hb[39 + block] = hmg1063Dec(cw >> 4, cw & 0x0f);
    }
    int const errors = rs_ess.decode(hb);
    if (errors < 0 || errors > 4)
      return;
    // MI is hexbits 39-50; ALGID follows
    set_algid(static_cast<uint8_t>((hb[51] << 2) + (hb[52] >> 4)));
  }

  void decode_voice(bit_vector const& frame)
  {
    if (encrypted) {
      stats.voice_encrypted += nof_voice_codewords;
      return;
    }
    basic_op_scope scope(&arithmetic);
    voice_codeword bits(voice_codeword_sz);
    int16_t pcm[P25_NATIVE_PCM_SAMPLES];
    for (size_t n = 0; n < nof_voice_codewords; n++) {
      imbe_deinterleave(frame, bits, n);
      uint32_t u[8], first_errors, total_errors;
      imbe_header_decode(bits, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7],
                         first_errors, total_errors);
      int16_t params[8];
      for (size_t i = 0; i < 8; i++)
        params[i] = static_cast<int16_t>(u[i]);
      params[7] >>= 1;
      vocoder->imbe_decode(params, pcm);
      stats.voice_decoded++;
      if (callback != nullptr)
        callback(user_data, pcm);
    }
  }

  void process_frame()
  {
    uint32_t const duid = framer.duid;
    bit_vector const& frame = framer.frame_body;
    stats.frames++;
    stats.last_nac = static_cast<uint16_t>(framer.nac);
    stats.last_bch_errors = framer.bch_errors;
    switch (duid) {
    case DUID_HDU:
      read_hdu(frame);
      break;
    case DUID_TDU:
    case DUID_TDULC:
      encrypted = false; // end of call
      break;
    case DUID_LDU1:
      stats.ldu1++;
      decode_voice(frame);
      break;
    case DUID_LDU2:
      stats.ldu2++;
      read_ess(frame);
      decode_voice(frame);
      break;
    default:
      break;
    }
  }
};

p25_voice::p25_voice(uint16_t nac, p25_native_pcm_callback callback, void *user_data)
  : impl_(new impl)
{
  impl_->callback = callback;
  impl_->user_data = user_data;
  impl_->framer.set_nac(nac);
  basic_op_scope scope(&impl_->arithmetic);
  impl_->vocoder.reset(new imbe_vocoder);
}

p25_voice::~p25_voice() = default;

void p25_voice::rx_dibit(uint8_t dibit)
{
  if (impl_->framer.rx_sym(dibit))
    impl_->process_frame();
}

void p25_voice::get_stats(p25_native_stats *stats) const
{
  stats->frames = impl_->stats.frames;
  stats->ldu1 = impl_->stats.ldu1;
  stats->ldu2 = impl_->stats.ldu2;
  stats->voice_decoded = impl_->stats.voice_decoded;
  stats->voice_encrypted = impl_->stats.voice_encrypted;
  stats->last_nac = impl_->stats.last_nac;
  stats->last_bch_errors = impl_->stats.last_bch_errors;
}
