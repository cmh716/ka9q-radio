// D-STAR voice receiver: GMSK demodulation, frame sync, slow-data header
// (callsigns) and AMBE (2400) decoding
// Clock recovery follows GNU Radio clock_recovery_mm_ff as OP25 uses it for
// D-STAR (filter_type 'gmsk'); voice decoding follows OP25 rx_sync.cc;
// frame layout and slow data as OP25's dstar_tx_sb / dstar_header.h
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <iterator>

#include "dv_decoder.h"
#include "dv_frontend.h"
#include "upstream/gnuradio_mmse_taps.h"
#include "upstream/mbelib.h"
#include "upstream/ambe.h" // needs mbelib.h first
#include "upstream/p25p2_vf.h"
#include "upstream/software_imbe_decoder.h"

namespace {

constexpr int kSps = 5;                 // 24 kHz / 4800 bit/s
constexpr float kDeviation = 1200.0f;   // GMSK, h = 0.5: +/-1200 Hz
constexpr float kDiscGain = kDvSampleRate / (2.0f * kDvPi * kDeviation);

// OP25's GMSK path has no filter after the discriminator, which leaves the
// slicer and timing loop all of its noise (rising with frequency) across the
// 24 kHz band. A root raised cosine (alpha 0.2, ~2.9 kHz), the DMR receive
// filter, measured on a live D-STAR capture with added noise: 0 -> 2628 of
// ~2700 voice frames at 8 dB SNR, and more frames decoded on the clean signal
// too (wider RRC 0.5 and a 5-sample average lost lock at times)

// Mueller & Muller timing (OP25 p25_demodulator_dev.py, filter_type 'gmsk')
constexpr float kGainMu = 0.175f;
constexpr float kGainOmega = 0.25f * kGainMu * kGainMu; // critically damped
constexpr float kOmegaRelativeLimit = 0.005f;

constexpr int kFrameBits = 96;          // 72 voice + 24 slow data, 20 ms
constexpr int kVoiceBits = 72;
constexpr int kSuperframe = 21;         // frames per sync (frame 0 carries it)

// Frame sync in frame 0's slow data, and the end-of-transmission pattern,
// as transmitted (bytes LSB first)
constexpr uint8_t kFs[24] = {1,0,1,0,1,0,1,0, 1,0,1,1,0,1,0,0, 0,1,1,0,1,0,0,0};
constexpr uint8_t kEot[48] = {1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0,
                              1,0,1,0,1,0,1,0, 0,0,0,1,0,0,1,1, 0,1,0,1,1,1,1,0};
// Header frame sync after the preamble: 111011001010000
constexpr uint8_t kHeaderSync[15] = {1,1,1,0,1,1,0,0,1,0,1,0,0,0,0};
constexpr int kHeaderBits = 660;        // convolutionally coded radio header

constexpr uint8_t kScramble[3] = {0x70, 0x4f, 0x93}; // slow data

// On live signals OP25's synthesizer renders D-STAR AMBE about 14 dB louder
// than P25 IMBE and clips inside decode_tap() on about 5% of frames. It's
// linear in the spectral magnitudes, so scale those down before synthesis
constexpr float kMagnitudeScale = 0.25f; // -12 dB

uint64_t pattern(uint8_t const *bits, int n)
{
  uint64_t r = 0;
  for (int i = 0; i < n; i++)
    r = (r << 1) | bits[i];
  return r;
}

// The radio header at the start of a transmission (JARL): 41 bytes + 2 tail
// bits, K=3 R=1/2 convolutional code (G1 7, G2 5), 24-way interleave, x^7 +
// x^4 + 1 scrambler. Verified on a live capture (CRC passes, no bit errors).
// Returns false if the CRC fails
bool decode_radio_header(uint8_t const in[kHeaderBits], uint8_t out[41]);

// D-STAR header CRC: CRC-CCITT, reflected (0x8408), init 0xffff, inverted
uint16_t dstar_crc(uint8_t const *data, int len)
{
  uint16_t crc = 0xffff;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
  }
  return static_cast<uint16_t>(~crc);
}

bool decode_radio_header(uint8_t const in[kHeaderBits], uint8_t out[41])
{
  // Descramble
  uint8_t s[kHeaderBits];
  unsigned reg = 0x7f;
  for (int i = 0; i < kHeaderBits; i++) {
    unsigned const fb = ((reg >> 3) ^ (reg >> 6)) & 1;
    s[i] = in[i] ^ fb;
    reg = ((reg << 1) | fb) & 0x7f;
  }
  // De-interleave
  uint8_t d[kHeaderBits];
  for (int i = 0, k = 0; i < kHeaderBits; i++) {
    d[k] = s[i];
    k += 24;
    if (k >= 672)
      k -= 671;
    else if (k >= 660)
      k -= 647;
  }
  // Viterbi, K=3, hard decisions, from state 0
  constexpr int kSteps = kHeaderBits / 2; // 330: 328 header bits + 2 tail
  int metric[4] = {0, 1 << 20, 1 << 20, 1 << 20};
  uint8_t from[kSteps][4];
  for (int t = 0; t < kSteps; t++) {
    int next[4] = {1 << 30, 1 << 30, 1 << 30, 1 << 30};
    for (int st = 0; st < 4; st++) {
      for (int b = 0; b < 2; b++) {
        int const r = (b << 2) | st;                       // newest bit on top
        int const o1 = __builtin_popcount(r & 7) & 1;
        int const o2 = __builtin_popcount(r & 5) & 1;
        int const m = metric[st] + (o1 != d[2 * t]) + (o2 != d[2 * t + 1]);
        int const ns = r >> 1;
        if (m < next[ns]) {
          next[ns] = m;
          from[t][ns] = static_cast<uint8_t>(st);
        }
      }
    }
    std::copy(next, next + 4, metric);
  }
  int st = 0; // the tail bits bring it back to state 0
  uint8_t bits[kSteps];
  for (int t = kSteps - 1; t >= 0; t--) {
    bits[t] = static_cast<uint8_t>(st >> 1); // the input bit is the new state's top bit
    st = from[t][st];
  }
  for (int i = 0; i < 41; i++) {
    out[i] = 0;
    for (int j = 0; j < 8; j++)
      out[i] |= bits[8 * i + j] << j; // LSB first
  }
  uint16_t const crc = dstar_crc(out, 39);
  return (crc & 0xff) == out[39] && (crc >> 8) == out[40];
}

class dstar_decoder : public dv_decoder {
public:
  dstar_decoder(dv_pcm_callback callback, void *user_data)
    : callback_(callback), user_data_(user_data), frontend_(350.0f)
  {
    mbe_initMbeParms(&cur_mp_, &prev_mp_, &enh_mp_);
    mbe_initErrParms(&errs_mp_);
    fs_ = pattern(kFs, 24);
    eot_ = pattern(kEot, 48);
    hs_ = pattern(kHeaderSync, 15);
    end_call();
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
    stats->call_serial = call_serial_;
    stats->samples = frontend_.samples;
    stats->symbols = bits_;
    stats->syncs = syncs_;
    stats->frames = frames_;
    stats->voice_decoded = voice_decoded_;
    stats->voice_muted = voice_muted_;
    stats->freq_offset = frontend_.frequency_offset_hz() + dc_ * kDeviation;
    std::memcpy(stats->src_call, src_, sizeof src_);
    std::memcpy(stats->dst_call, dst_, sizeof dst_);
    stats->m17_can = -1;
  }

private:
  float discriminate(cfloat s)
  {
    float const re = s.real() * prev_.real() + s.imag() * prev_.imag();
    float const im = s.imag() * prev_.real() - s.real() * prev_.imag();
    prev_ = s;
    filt_history_[filt_position_] = std::atan2(im, re) * kDiscGain;
    float const out = dv_fir(DV_RRC_TAPS, filt_history_, filt_position_);
    filt_position_ = (filt_position_ + 1) % std::size(filt_history_);
    return out;
  }

  // GNU Radio clock_recovery_mm_ff, streaming
  void rx_sample(float x)
  {
    hist_.push_back(x);
    while (static_cast<int>(hist_.size()) >= ii_ + 8) {
      size_t const phase = static_cast<size_t>(std::clamp(std::lrint(mu_ * 128.0f), 0L, 128L));
      float out = 0;
      for (int t = 0; t < 8; t++)
        out += P25_GNURADIO_MMSE_TAPS[phase][t] * (hist_[ii_ + t] - dc_);
      float const mm = slice(last_) * out - slice(out) * last_;
      last_ = out;
      omega_ += kGainOmega * mm;
      omega_ = kSps + std::clamp(omega_ - kSps, -kSps * kOmegaRelativeLimit, kSps * kOmegaRelativeLimit);
      mu_ += omega_ + kGainMu * mm;
      int const adv = static_cast<int>(std::floor(mu_));
      ii_ += adv;
      mu_ -= adv;
      symbol(out);
    }
    if (ii_ > 64) { // drop consumed history
      hist_.erase(hist_.begin(), hist_.begin() + ii_);
      ii_ = 0;
    }
  }

  static float slice(float x) { return x < 0 ? -1.0f : 1.0f; }

  // Decision-directed tracking of the symbol amplitude and residual DC
  // (frequency offset), so the slicer and timing work on a centred signal
  void symbol(float s)
  {
    float const e = s; // already DC-corrected by rx_sample
    float const level = e > 0 ? amp_ : -amp_;
    dc_ += 0.004f * (e - level);
    amp_ += 0.004f * (std::fabs(e) - amp_);
    amp_ = std::clamp(amp_, 0.3f, 2.0f);
    dc_ = std::clamp(dc_, -1.0f, 1.0f);
    rx_bit(e > 0 ? 1 : 0);
  }

  void rx_bit(uint8_t raw)
  {
    // Collecting the radio header after its sync?
    if (header_left_ > 0) {
      radio_header_[kHeaderBits - header_left_] = raw ^ (header_invert_ ? 1 : 0);
      if (--header_left_ == 0) {
        uint8_t h[41];
        if (!locked_ && decode_radio_header(radio_header_, h)) {
          set_call(dst_, h + 19);   // UR
          set_call(src_, h + 27);   // MY
          header_valid_ = true;
          invert_ = header_invert_;
          expect_sync_ = bits_ + 1 + kFrameBits; // the end of voice frame 0
        }
      }
    }
    bits_++;
    reg_ = (reg_ << 1) | raw;
    std::memmove(buf_, buf_ + 1, sizeof buf_ - 1);
    buf_[sizeof buf_ - 1] = raw;

    // End of transmission (either polarity)
    uint64_t const r48 = reg_ & 0xffffffffffffULL;
    if (locked_ && (r48 == eot_ || r48 == (~eot_ & 0xffffffffffffULL))) {
      call_serial_++;
      end_call();
      return;
    }

    // Start of transmission: header frame sync (usually after a 1010
    // preamble, but an openSPOT 4 also starts some transmissions after
    // zeros), then the radio header. Decode a header after every such sync
    // while unlocked; its CRC rejects false matches. Voice frame 0 (with its
    // slow-data sync) comes right after the header
    if (!locked_ && header_left_ == 0) {
      uint64_t const r15 = reg_ & 0x7fff;
      if (r15 == hs_ || r15 == (~hs_ & 0x7fff)) {
        header_invert_ = r15 != hs_;
        header_left_ = kHeaderBits;
      }
    }

    uint32_t const r24 = static_cast<uint32_t>(reg_ & 0xffffff);
    uint32_t const fs = invert_ ? (~fs_ & 0xffffff) : fs_;
    if (locked_ || expect_sync_ != 0) {
      // Near where the next frame-0 sync is due: allow bit errors and a
      // small timing slip
      if (bits_ + 2 >= expect_sync_ && bits_ <= expect_sync_ + 2
          && __builtin_popcount(r24 ^ fs) <= 3) {
        sync(bits_);
        return;
      }
      if (bits_ > expect_sync_ + 2) {
        // Missed it: carry on by count, up to two superframes
        if (locked_ && ++missed_ <= 2) {
          expect_sync_ += kSuperframe * kFrameBits;
        } else if (locked_ || bits_ > expect_sync_ + kFrameBits) {
          end_call();
          return;
        }
      }
    } else {
      // Hunting (late entry): an exact sync, either polarity
      if (r24 == fs_ || r24 == (~fs_ & 0xffffff)) {
        invert_ = r24 != fs_;
        sync(bits_);
        return;
      }
    }
    if (locked_ && bits_ == next_frame_)
      frame();
  }

  void sync(uint64_t at)
  {
    syncs_++;
    if (!locked_) {
      header_.clear();
      if (!header_valid_)
        src_[0] = dst_[0] = '\0';
    }
    locked_ = true;
    missed_ = 0;
    expect_sync_ = at + kSuperframe * kFrameBits;
    frame_index_ = 0;
    next_frame_ = at; // this frame (frame 0) is complete now
    frame();
  }

  // One 96-bit frame ends at bit bits_
  void frame()
  {
    frames_++;
    uint8_t bits[kFrameBits];
    for (int i = 0; i < kFrameBits; i++)
      bits[i] = buf_[sizeof buf_ - kFrameBits + i] ^ (invert_ ? 1 : 0);
    voice(bits);
    if (frame_index_ == 0) {
      header_.clear(); // slow data restarts with each superframe
      block_len_ = 0;
    } else
      slow_data(bits + kVoiceBits);
    frame_index_ = (frame_index_ + 1) % kSuperframe;
    next_frame_ = bits_ + kFrameBits;
  }

  // Frames 1-20 carry 3 scrambled bytes each; pairs make 6-byte blocks.
  // Type 0x5n blocks carry the 41-byte header 5 bytes at a time
  void slow_data(uint8_t const *bits)
  {
    for (int k = 0; k < 3; k++) {
      uint8_t b = 0;
      for (int i = 0; i < 8; i++)
        b |= bits[8 * k + i] << i; // LSB first
      block_[block_len_++] = b ^ kScramble[k];
    }
    if (block_len_ < 6)
      return;
    block_len_ = 0;
    uint8_t const type = block_[0] & 0xf0;
    int const n = std::min(block_[0] & 0x0f, 5);
    if (type != 0x50)
      return;
    for (int i = 0; i < n && header_.size() < 41; i++)
      header_.push_back(block_[1 + i]);
    if (header_.size() == 41) {
      uint8_t h[41];
      std::copy(header_.begin(), header_.end(), h);
      uint16_t const crc = dstar_crc(h, 39);
      if ((crc & 0xff) == h[39] && (crc >> 8) == h[40]) {
        set_call(dst_, h + 19);   // UR
        set_call(src_, h + 27);   // MY
      }
      header_.clear();
    }
  }

  static void set_call(char out[12], uint8_t const *c)
  {
    int n = 0;
    for (int i = 0; i < 8; i++)
      out[n++] = (c[i] >= 0x20 && c[i] < 0x7f) ? static_cast<char>(c[i]) : ' ';
    while (n > 0 && out[n - 1] == ' ')
      n--;
    out[n] = '\0';
  }

  // rx_sync::codeword(), CODEWORD_DSTAR case and common tail
  void voice(uint8_t const *cw)
  {
    int b[9];
    size_t const errs = interleaver_.decode_dstar(cw, b, false);
    bool do_silence = false;
    if (b[0] < 120 && errs <= kMaxGolayErrors) // TODO in OP25: frame repeats and tones
      mbe_dequantizeAmbe2400Parms(&cur_mp_, &prev_mp_, &errs_mp_, b);
    else
      do_silence = true;
    mbe_moveMbeParms(&cur_mp_, &prev_mp_);
    if (!do_silence) {
      float ml[56];
      int const n = std::min(cur_mp_.L, 56);
      for (int l = 0; l < n; l++)
        ml[l] = cur_mp_.Ml[l + 1] * kMagnitudeScale;
      synth_.decode_tap(cur_mp_.L, 0, cur_mp_.w0, &cur_mp_.Vl[1], ml);
    }
    audio_samples *samples = synth_.audio();
    int16_t pcm[DV_PCM_SAMPLES];
    for (int i = 0; i < DV_PCM_SAMPLES; i++) {
      float snd = 0;
      if (!do_silence && !samples->empty()) {
        snd = samples->front();
        samples->pop_front();
      }
      pcm[i] = static_cast<int16_t>(std::clamp(snd, -32768.0f, 32767.0f));
    }
    if (do_silence)
      voice_muted_++;
    else
      voice_decoded_++;
    if (callback_ != nullptr)
      callback_(user_data_, pcm);
  }

  void signal_lost() override
  {
    call_serial_++;
    end_call();
  }

  // call_serial_ advances only on a definitive end (an end-of-transmission
  // marker, or the signal lost); a lost sync lock ends the call's state but
  // not its identity, which radiod holds through a brief fade
  void end_call()
  {
    header_valid_ = false;
    header_left_ = 0;
    locked_ = false;
    expect_sync_ = 0;
    missed_ = 0;
    header_.clear();
    block_len_ = 0;
    src_[0] = dst_[0] = '\0';
  }

  static constexpr size_t kMaxGolayErrors = 6; // two Golay(24,12) words, 3 each
  uint64_t call_serial_ = 0;

  dv_pcm_callback const callback_;
  void * const user_data_;
  dv_frontend frontend_;
  cfloat prev_{};
  float filt_history_[45]{};
  size_t filt_position_ = 0;

  // Clock recovery
  std::deque<float> hist_;
  int ii_ = 0;
  float mu_ = 0.5f;
  float omega_ = kSps;
  float last_ = 0;
  float dc_ = 0, amp_ = 1;

  // Framing
  uint64_t reg_ = 0;
  uint8_t buf_[kFrameBits + 64]{};
  uint64_t bits_ = 0;
  uint64_t fs_ = 0, eot_ = 0, hs_ = 0;
  bool locked_ = false;
  bool invert_ = false;
  uint64_t expect_sync_ = 0;
  uint64_t next_frame_ = 0;
  int frame_index_ = 0;
  int missed_ = 0;

  // Radio header
  uint8_t radio_header_[kHeaderBits]{};
  int header_left_ = 0;
  bool header_invert_ = false;
  bool header_valid_ = false;

  // Slow data
  uint8_t block_[6]{};
  int block_len_ = 0;
  std::deque<uint8_t> header_;
  char src_[12]{};
  char dst_[12]{};

  // Vocoder
  p25p2_vf interleaver_;
  mbe_parms cur_mp_, prev_mp_, enh_mp_;
  mbe_errs errs_mp_;
  software_imbe_decoder synth_;

  uint64_t syncs_ = 0;
  uint64_t frames_ = 0;
  uint64_t voice_decoded_ = 0;
  uint64_t voice_muted_ = 0;
};

} // namespace

dv_decoder *dv_create_dstar(dv_pcm_callback callback, void *user_data)
{
  return new dstar_decoder(callback, user_data);
}
