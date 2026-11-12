// P25 Phase 1 frame layer: NID framing (OP25 p25_framer), encryption
// detection from the HDU and LDU2 ESS, and IMBE voice decoding
// Adapted from OP25 p25p1_fdma.cc; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include "p25_voice.h"

#include <cstdio>
#include <cstring>
#include <vector>

#include "ezpwd/rs"
#include "imbe_vocoder/imbe_vocoder.h"
#include "op25_golay.h"
#include "op25_hamming.h"
#include "op25_imbe_frame.h"
#include "p25_framer.h"
#include "p25_trunk.h"

namespace {

enum : uint32_t { DUID_HDU = 0, DUID_TDU = 3, DUID_LDU1 = 5, DUID_LDU2 = 10, DUID_TSDU = 7, DUID_TDULC = 15 };
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
  p25_trunk trunk;
  uint64_t symbols = 0;
  BasicOpContext arithmetic{0, 0};
  std::unique_ptr<imbe_vocoder> vocoder;
  ezpwd::RS<63, 55> rs_ess;   // RS(24,16,9) shortened
  ezpwd::RS<63, 47> rs_hdu;   // RS(36,20,17) shortened
  ezpwd::RS<63, 51> rs_lc;    // RS(24,12,13) shortened
  dv_pcm_callback callback;
  void *user_data;
  dv_stats stats{};
  // Assume clear (as OP25 does) until an HDU or LDU2 says otherwise
  bool encrypted = false;

  uint64_t last_frame = 0;    // symbol count of the last frame

  // A call ends with a TDU; when that is missed, with the signal
  void end_call()
  {
    stats.call_serial++;
    encrypted = false;
    stats.src_call[0] = stats.dst_call[0] = '\0';
  }

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

  // The 24 Hamming(10,6,3) protected hexbits an LDU carries (LC or ESS),
  // placed at hb[39..62] for the shortened RS decoders
  std::vector<uint8_t> ldu_hexbits(bit_vector const& frame)
  {
    std::vector<uint8_t> hb(63, 0);
    size_t index = 0;
    for (size_t block = 0; block < 24; block++) {
      uint32_t cw = 0;
      for (size_t bit = 0; bit < 10; bit++)
        cw = (cw << 1) + frame[imbe_ldu_ls_data_bits[index++]];
      hb[39 + block] = hmg1063Dec(cw >> 4, cw & 0x0f);
    }
    return hb;
  }

  void read_ess(bit_vector const& frame)
  {
    std::vector<uint8_t> hb = ldu_hexbits(frame);
    int const errors = rs_ess.decode(hb);
    if (errors < 0 || errors > 4)
      return;
    // MI is hexbits 39-50; ALGID follows
    set_algid(static_cast<uint8_t>((hb[51] << 2) + (hb[52] >> 4)));
  }

  // LDU1 link control: the talkgroup (or target unit) and source of the call
  void read_lc(bit_vector const& frame)
  {
    std::vector<uint8_t> hb = ldu_hexbits(frame);
    int const errors = rs_lc.decode(hb);
    if (errors < 0 || errors > 6)
      return;
    uint8_t lc[9];
    for (int i = 0, j = 0; i < 9; j += 4) {
      lc[i++] = static_cast<uint8_t>((hb[j + 39] << 2) + (hb[j + 40] >> 4));
      lc[i++] = static_cast<uint8_t>(((hb[j + 40] & 0x0f) << 4) + (hb[j + 41] >> 2));
      lc[i++] = static_cast<uint8_t>(((hb[j + 41] & 0x03) << 6) + hb[j + 42]);
    }
    uint8_t const lco = lc[0] & 0x3f;
    bool const standard = (lc[0] & 0x40) == 0 && (lc[1] == 0x00 || lc[1] == 0x01 || lc[1] == 0x90);
    if (!standard || (lco != 0x00 && lco != 0x03))
      return; // only group and unit-to-unit voice name the call
    uint32_t const dest = lco == 0x00 ? static_cast<uint32_t>(lc[4] << 8 | lc[5])
                                      : static_cast<uint32_t>(lc[3] << 16 | lc[4] << 8 | lc[5]);
    uint32_t const src = static_cast<uint32_t>(lc[6] << 16 | lc[7] << 8 | lc[8]);
    snprintf(stats.dst_call, sizeof stats.dst_call, "%u", dest);
    if (src != 0)
      snprintf(stats.src_call, sizeof stats.src_call, "%u", src);
  }

  void decode_voice(bit_vector const& frame)
  {
    if (encrypted) {
      stats.voice_muted += nof_voice_codewords;
      return;
    }
    basic_op_scope scope(&arithmetic);
    voice_codeword bits(voice_codeword_sz);
    int16_t pcm[DV_PCM_SAMPLES];
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
      end_call();
      break;
    case DUID_LDU1:
      stats.ldu1++;
      read_lc(frame);
      decode_voice(frame);
      break;
    case DUID_LDU2:
      stats.ldu2++;
      read_ess(frame);
      decode_voice(frame);
      break;
    case DUID_TSDU:
      trunk.tsdu(frame, framer.frame_size, static_cast<uint16_t>(framer.nac));
      break;
    default:
      break;
    }
  }
};

p25_voice::p25_voice(uint16_t nac, dv_pcm_callback callback, void *user_data)
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
  impl_->trunk.tick(++impl_->symbols);
  if (impl_->framer.rx_sym(dibit)) {
    impl_->last_frame = impl_->symbols;
    impl_->process_frame();
  } else if (impl_->symbols - impl_->last_frame == 4800) {
    impl_->end_call(); // 1 s without a frame: the call faded out without its TDU
  }
}

void p25_voice::get_stats(dv_stats *stats) const
{
  stats->call_serial = impl_->stats.call_serial;
  stats->frames = impl_->stats.frames;
  stats->ldu1 = impl_->stats.ldu1;
  stats->ldu2 = impl_->stats.ldu2;
  stats->voice_decoded = impl_->stats.voice_decoded;
  stats->voice_muted = impl_->stats.voice_muted;
  stats->last_nac = impl_->stats.last_nac;
  stats->last_bch_errors = impl_->stats.last_bch_errors;
  memcpy(stats->src_call, impl_->stats.src_call, sizeof stats->src_call);
  memcpy(stats->dst_call, impl_->stats.dst_call, sizeof stats->dst_call);
  impl_->trunk.get_stats(stats);
}

void p25_voice::signal_lost()
{
  impl_->end_call();
}

void p25_voice::set_event_callback(dv_event_callback callback, void *user_data)
{
  impl_->trunk.set_callback(callback, user_data);
}
