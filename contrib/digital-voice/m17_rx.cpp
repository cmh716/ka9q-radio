// M17 voice receiver: 4FSK demodulation with sync-word timing, LSF/LICH
// handling and Codec2 decoding. Frame decoding (Viterbi, Golay,
// interleaving, CRC) is libm17 (M17 Foundation, GPL-2.0-or-later); see
// README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <new>

#include <codec2/codec2.h>

#include "dv_decoder.h"
#include "dv_frontend.h"
#include "upstream/libm17/m17.h"

namespace {

// The front end delivers 24 kHz: 5 samples per 4800 Bd symbol. There's no
// symbol timing loop: as in the M17 reference receivers, every frame's sync
// word picks the sample phase, gain and DC offset used for its payload.
// (OP25's FSK4 timing loop, used for P25 and DMR, can settle on false
// equilibria on M17's alternating +3/-3 preamble.)
constexpr int kSps = 5;
constexpr float kDeviation = 800.0f;     // Hz per symbol level (+/-2400 Hz outer)
constexpr float kQuadratureGain = kDvSampleRate / (2.0f * kDvPi * kDeviation);
constexpr float kSyncThreshold = 2.0f;   // residual distance to accept a sync while hunting
constexpr float kSyncThresholdLocked = 4.0f; // ...near where the next frame's sync is due
constexpr uint64_t kLockWindow = 3;      // samples either side of the expected sync position
constexpr int kLockFrames = 3;           // missed syncs tolerated before giving up on a transmission
constexpr uint64_t kRing = 2048;         // sample history, > one frame (960 samples)
// Noise matches a stream sync now and then; measured on noise, such frames
// almost never pass LICH Golay decoding and have Viterbi error rates >= 0.137,
// while a real first frame at 6 dB SNR has 0.076. A transmission starts on a
// valid LSF, or on a stream frame with a good LICH and a low error rate
constexpr float kStartErrorRate = 0.12f;
constexpr float kMaxErrorRate = 0.16f;   // frames worse than this are muted, even mid-transmission

enum frame_kind { FRAME_NONE, FRAME_LSF_K, FRAME_STREAM_K };

uint16_t lsf_type(lsf_t const& lsf) { return static_cast<uint16_t>((lsf.type[0] << 8) | lsf.type[1]); }
bool lsf_crc_ok(lsf_t const& lsf)
{
  return LSF_CRC(&lsf) == static_cast<uint16_t>((lsf.crc[0] << 8) | lsf.crc[1]);
}

// Least-squares fit rx ~ gain * pattern + dc over a sync word; returns the
// residual distance in symbol units (huge if the fit is implausible)
float fit_sync(float const rx[SYM_PER_SWD], int8_t const pattern[SYM_PER_SWD], float& gain, float& dc)
{
  float sp = 0, sr = 0, spp = 0, spr = 0;
  for (int i = 0; i < SYM_PER_SWD; i++) {
    sp += pattern[i];
    sr += rx[i];
    spp += pattern[i] * pattern[i];
    spr += pattern[i] * rx[i];
  }
  float const n = SYM_PER_SWD;
  gain = (n * spr - sp * sr) / (n * spp - sp * sp);
  dc = (sr - gain * sp) / n;
  if (!(gain > 0.6f && gain < 1.6f) || std::fabs(dc) > 1.5f)
    return 1e9f;
  float r = 0;
  for (int i = 0; i < SYM_PER_SWD; i++) {
    float const e = (rx[i] - dc) / gain - pattern[i];
    r += e * e;
  }
  return std::sqrt(r);
}

class m17_decoder : public dv_decoder {
public:
  m17_decoder(int can, dv_pcm_callback callback, void *user_data)
    : wanted_can_(can), callback_(callback), user_data_(user_data), frontend_(350.0f)
  {
    c2_3200_ = codec2_create(CODEC2_MODE_3200);
    c2_1600_ = codec2_create(CODEC2_MODE_1600);
    if (c2_3200_ == nullptr || c2_1600_ == nullptr) {
      if (c2_3200_ != nullptr)
        codec2_destroy(c2_3200_);
      if (c2_1600_ != nullptr)
        codec2_destroy(c2_1600_);
      throw std::bad_alloc();
    }
    viterbi_reset_ctx(&viterbi_);
    end_call();
  }

  ~m17_decoder() override
  {
    codec2_destroy(c2_3200_);
    codec2_destroy(c2_1600_);
  }

  void process(cfloat const *samples, size_t count) override
  {
    for (size_t i = 0; i < count; i++) {
      cfloat lev;
      if (frontend_.process(samples[i], lev))
        leveled(lev);
    }
  }

  void process_leveled(cfloat const *samples, size_t count) override
  {
    for (size_t i = 0; i < count; i++)
      leveled(samples[i]);
  }

  void leveled(cfloat lev) { rx_sample(discriminate(lev)); }

  void get_stats(dv_stats *stats) const override
  {
    *stats = {};
    stats->samples = frontend_.samples;
    stats->symbols = n_ / kSps;
    stats->syncs = syncs_;
    stats->frames = frames_;
    stats->voice_decoded = voice_decoded_;
    stats->voice_muted = voice_muted_;
    stats->freq_offset = frontend_.frequency_offset_hz() + dc_ * kDeviation;
    stats->error_rate = error_rate_;
    if (lsf_valid_) {
      std::memcpy(stats->src_call, src_, sizeof stats->src_call);
      std::memcpy(stats->dst_call, dst_, sizeof stats->dst_call);
      stats->m17_can = (lsf_type(lsf_) >> 7) & 0xf;
      stats->m17_encrypted = encrypted_;
    } else {
      stats->m17_can = -1;
    }
  }

private:
  // Quadrature discriminator and RRC receive filter, scaled to symbol levels
  float discriminate(cfloat s)
  {
    float const re = s.real() * prev_.real() + s.imag() * prev_.imag();
    float const im = s.imag() * prev_.real() - s.real() * prev_.imag();
    prev_ = s;
    rrc_history_[rrc_position_] = std::atan2(im, re) * kQuadratureGain;
    float const out = dv_fir(DV_RRC_05_TAPS, rrc_history_, rrc_position_);
    rrc_position_ = (rrc_position_ + 1) % std::size(rrc_history_);
    return out;
  }

  float at(uint64_t index) const { return ring_[index % kRing]; }

  void rx_sample(float v)
  {
    ring_[n_ % kRing] = v;
    uint64_t const n = n_++;
    if (n < kSps * SYM_PER_FRA)
      return; // not enough history yet

    // Collecting the payload of a frame whose sync was found?
    if (kind_ != FRAME_NONE) {
      if (n >= payload_end_) {
        decode_frame();
        kind_ = FRAME_NONE;
      }
      return;
    }

    // While a transmission is in progress the next sync is due one frame
    // after the last one: look only near there, more leniently
    bool const locked = lock_ > 0;
    if (locked && n + kLockWindow < next_sync_)
      return;
    bool const window_over = locked && n >= next_sync_ + kLockWindow;
    float const threshold = locked ? kSyncThresholdLocked : kSyncThreshold;

    float syms[SYM_PER_SWD]; // the 8 symbols of a sync word ending at sample n
    for (int k = 0; k < SYM_PER_SWD; k++)
      syms[k] = at(n - kSps * (SYM_PER_SWD - 1 - k));
    float g_str, c_str, g_lsf, c_lsf, g_eot, c_eot;
    float const d_str = fit_sync(syms, str_sync_symbols, g_str, c_str);
    float const d_lsf = fit_sync(syms, lsf_sync_symbols, g_lsf, c_lsf);
    float const d_eot = fit_sync(syms, eot_symbols, g_eot, c_eot);
    if (locked && d_eot < kSyncThreshold && d_eot < d_str && d_eot < d_lsf) {
      end_call(); // end of transmission marker
      return;
    }
    bool const is_lsf = d_lsf < d_str;
    float const d = is_lsf ? d_lsf : d_str;

    // Follow the match to its best (lowest residual) sample phase
    if (d < threshold && (cand_kind_ == FRAME_NONE || d < cand_d_)) {
      cand_kind_ = is_lsf ? FRAME_LSF_K : FRAME_STREAM_K;
      cand_d_ = d;
      cand_end_ = n;
      cand_gain_ = is_lsf ? g_lsf : g_str;
      cand_dc_ = is_lsf ? c_lsf : c_str;
      if (!window_over)
        return;
    }
    if (cand_kind_ != FRAME_NONE && (d >= cand_d_ || window_over)) {
      // Past the best point: take it
      syncs_++;
      kind_ = cand_kind_;
      sync_end_ = cand_end_;
      gain_ = cand_gain_;
      dc_ = cand_dc_;
      payload_end_ = sync_end_ + kSps * SYM_PER_PLD;
      cand_kind_ = FRAME_NONE;
      return;
    }
    if (window_over) {
      // A frame was due but its sync wasn't found
      next_sync_ += kSps * SYM_PER_FRA;
      if (--lock_ == 0)
        end_call();
    }
  }

  void decode_frame()
  {
    float pld[SYM_PER_PLD];
    for (int k = 0; k < SYM_PER_PLD; k++)
      pld[k] = (at(sync_end_ + kSps * (k + 1)) - dc_) / gain_;
    if (kind_ == FRAME_LSF_K) {
      lsf_t lsf;
      uint32_t const e = decode_LSF_ctx(&viterbi_, &lsf, pld);
      if (!lsf_crc_ok(lsf))
        return; // a false match, or too damaged to use
      error_rate_ = e / static_cast<float>(0xFFFF) / (2 * SYM_PER_PLD);
      accept_frame();
      set_lsf(lsf);
      return;
    }

    uint8_t data[16];
    uint8_t lich[5];
    uint16_t fn;
    uint8_t lich_cnt;
    uint32_t const e = decode_str_frame_ctx(&viterbi_, data, lich, &fn, &lich_cnt, pld);
    float const error_rate = e / static_cast<float>(0xFFFF) / (2 * SYM_PER_PLD - 96);
    bool const starting = lock_ == 0;
    if (starting ? (lich_cnt >= 6 || error_rate >= kStartErrorRate) : error_rate > kMaxErrorRate)
      return; // ignored; mid-transmission this counts as a missed frame
    error_rate_ = error_rate;
    accept_frame();

    // Late entry: rebuild the LSF from the six LICH chunks
    if (!lsf_valid_ && lich_cnt < 6) {
      std::memcpy(lich_buf_ + 5 * lich_cnt, lich, 5);
      lich_have_ |= 1 << lich_cnt;
      if (lich_have_ == 0x3f) {
        lsf_t lsf;
        std::memcpy(&lsf, lich_buf_, sizeof lsf);
        if (lsf_crc_ok(lsf))
          set_lsf(lsf);
        else
          lich_have_ = 0;
      }
    }

    // Voice, unless known to be data-only, encrypted, or another CAN
    uint16_t const type = lsf_valid_ ? lsf_type(lsf_) : (M17_TYPE_STREAM | M17_TYPE_VOICE);
    unsigned const data_type = (type >> 1) & 3; // 1 data, 2 voice, 3 voice + data
    bool const can_ok = wanted_can_ < 0 || !lsf_valid_ || static_cast<int>((type >> 7) & 0xf) == wanted_can_;
    if (data_type == 1 || encrypted_ || !can_ok) {
      voice_muted_ += 2;
    } else {
      int16_t pcm[2 * DV_PCM_SAMPLES];
      if (data_type == 3) { // Codec2 1600 voice + 64 bits of data
        codec2_decode(c2_1600_, pcm, data);
      } else {              // Codec2 3200, two 20 ms frames
        codec2_decode(c2_3200_, pcm, data);
        codec2_decode(c2_3200_, pcm + DV_PCM_SAMPLES, data + 8);
      }
      for (int i = 0; i < 2; i++) {
        voice_decoded_++;
        if (callback_ != nullptr)
          callback_(user_data_, pcm + i * DV_PCM_SAMPLES);
      }
    }
    if (fn & 0x8000)
      end_call(); // last frame of the stream
  }

  void accept_frame()
  {
    frames_++;
    lock_ = kLockFrames;
    next_sync_ = sync_end_ + kSps * SYM_PER_FRA;
  }

  void set_lsf(lsf_t const& lsf)
  {
    lsf_ = lsf;
    lsf_valid_ = true;
    encrypted_ = ((lsf_type(lsf) >> 3) & 3) != 0; // scrambler or AES
    decode_callsign_bytes(src_, lsf.src);
    decode_callsign_bytes(dst_, lsf.dst);
  }

  void end_call()
  {
    lsf_valid_ = false;
    encrypted_ = false;
    lich_have_ = 0;
    lock_ = 0;
    kind_ = FRAME_NONE;
    cand_kind_ = FRAME_NONE;
  }

  int const wanted_can_;   // -1 = any
  dv_pcm_callback const callback_;
  void * const user_data_;
  dv_frontend frontend_;
  viterbi_ctx_t viterbi_;
  CODEC2 *c2_3200_ = nullptr;
  CODEC2 *c2_1600_ = nullptr;

  // Discriminator and receive filter
  cfloat prev_{};
  float rrc_history_[45]{};
  size_t rrc_position_ = 0;

  // Samples and framing
  float ring_[kRing]{};
  uint64_t n_ = 0;          // samples received
  frame_kind kind_ = FRAME_NONE; // frame being collected
  uint64_t sync_end_ = 0;   // sample index of its sync word's last symbol
  uint64_t payload_end_ = 0;
  float gain_ = 1, dc_ = 0; // from its sync word
  frame_kind cand_kind_ = FRAME_NONE; // best sync match so far
  float cand_d_ = 0;
  uint64_t cand_end_ = 0;
  float cand_gain_ = 1, cand_dc_ = 0;
  int lock_ = 0;            // missed frames left before giving up on a transmission
  uint64_t next_sync_ = 0;  // sample index where the next sync word should end

  lsf_t lsf_{};
  bool lsf_valid_ = false;
  bool encrypted_ = false;
  uint8_t lich_buf_[30]{};
  unsigned lich_have_ = 0;
  char src_[12]{};
  char dst_[12]{};

  uint64_t syncs_ = 0;
  uint64_t frames_ = 0;
  uint64_t voice_decoded_ = 0;
  uint64_t voice_muted_ = 0;
  float error_rate_ = 0;
};

} // namespace

dv_decoder *dv_create_m17(int can, dv_pcm_callback callback, void *user_data)
{
  return new m17_decoder(can, callback, user_data);
}
