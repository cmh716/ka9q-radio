// NXDN (NXDN48 / NXDN96) conventional voice receiver: 4FSK demodulation,
// frame sync, descrambling, LICH, SACCH / FACCH1 (VCALL: who, cipher) and
// AMBE+2 voice
// Framing and channel coding follow trunk-recorder-pro's nxdn module
// (TrunkRecorder/trunk-recorder-pro, GPL-3.0-or-later; NXDN Forum TS 1-A)
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>

#include "dv_ambe.h"
#include "dv_decoder.h"
#include "dv_frontend.h"
#include "upstream/libm17/m17.h"

namespace {

constexpr int kFrameSymbols = 192;      // 80 ms (NXDN48) or 40 ms (NXDN96)
constexpr int kFswSymbols = 10;
constexpr int kBodySymbols = kFrameSymbols - kFswSymbols;
constexpr uint32_t kFsw = 0xcdf59;      // -3 +1 -3 +3 -3 -3 +3 +3 -1 +3
constexpr uint32_t kFswInverted = kFsw ^ 0xaaaaa;
constexpr int kLockErrs = 1;            // FSW bit errors to lock anywhere
constexpr int kGridErrs = 6;            // ...and to count as the sync where one is due
constexpr int kUnlockFrames = 4;        // frames with neither sync nor LICH before letting go

// Scrambler (TS 1-A 4.6): PN9 x^9 + x^4 + 1 from 0x0e4, one bit per body
// symbol; a 1 inverts the symbol
struct scramble_table {
  uint8_t t[kBodySymbols];
  constexpr scramble_table() : t{}
  {
    uint32_t l = 0x0e4;
    for (int i = 0; i < kBodySymbols; i++) {
      t[i] = l & 1;
      uint32_t const fb = (l ^ (l >> 4)) & 1;
      l = (l >> 1) | (fb << 8);
    }
  }
};
constexpr scramble_table kScramble;

// Channel coding (TS 1-A 4.5): info -> CRC (preset ones) -> 4 tail bits ->
// K=5 R=1/2 convolutional code (G1 1+D^3+D^4, G2 1+D+D^2+D^4: M17's code,
// decoded by libm17) -> puncturing -> block interleave (rows of cols)
struct coding {
  int info, crc_bits;
  uint32_t crc_poly;
  int punct_period;
  int punct[2];
  int npunct;
  int cols;
  int coded() const { return (info + crc_bits + 4) * 2; }
  bool kept(int i) const
  {
    for (int k = 0; k < npunct; k++)
      if (i % punct_period == punct[k])
        return false;
    return true;
  }
  int air() const
  {
    int n = 0;
    for (int i = 0; i < coded(); i++)
      n += kept(i);
    return n;
  }
};
constexpr coding kSacch = {26, 6, 0x27, 12, {5, 11}, 2, 12};   // SR 8 + 18 data -> 60 air bits
constexpr coding kFacch1 = {80, 12, 0x80f, 4, {1, 0}, 1, 16};  // 80 bits layer 3 -> 144 air bits
constexpr int kSacchAt = 16;               // body bit offsets
constexpr int kHalfAt[2] = {76, 220};      // FACCH1 or two VCHs
constexpr int kVchAt = 38;                 // body dibit of the first VCH (36 dibits each)

uint32_t crc(uint8_t const *bits, int n, int width, uint32_t poly)
{
  uint32_t const mask = (1u << width) - 1;
  uint32_t const top = 1u << (width - 1);
  uint32_t r = mask;
  for (int i = 0; i < n; i++) {
    uint32_t const fb = (bits[i] & 1) ^ ((r & top) != 0);
    r = (r << 1) & mask;
    if (fb)
      r ^= poly;
  }
  return r;
}

// Decode one channel from its air bits (hard decisions); false if the CRC fails
bool decode_channel(viterbi_ctx_t *ctx, coding const& c, uint8_t const *air, uint8_t *info)
{
  int const nair = c.air();
  int const rows = nair / c.cols;
  std::vector<uint16_t> soft(c.coded());
  int k = 0;
  for (int i = 0; i < c.coded(); i++) {
    if (!c.kept(i)) {
      soft[i] = 0x7fff; // erasure
      continue;
    }
    int const at = (k % c.cols) * rows + k / c.cols; // de-interleave
    soft[i] = air[at] ? 0xffff : 0;
    k++;
  }
  int const steps = c.coded() / 2;
  std::vector<uint8_t> out((steps + 4 + 7) / 8 + 2);
  viterbi_decode_ctx(ctx, out.data(), soft.data(), c.coded());
  std::vector<uint8_t> all(steps);
  for (int i = 0; i < steps; i++) // libm17 places the decoded bits one byte in
    all[i] = (out[(i + 8) / 8] >> (7 - (i + 8) % 8)) & 1;
  uint32_t rx = 0;
  for (int i = 0; i < c.crc_bits; i++)
    rx = (rx << 1) | all[c.info + i];
  if (crc(all.data(), c.info, c.crc_bits, c.crc_poly) != rx)
    return false;
  for (int i = c.info + c.crc_bits; i < steps; i++)
    if (all[i] != 0)
      return false; // the tail must be zeros
  std::memcpy(info, all.data(), c.info);
  return true;
}

class nxdn_decoder : public dv_decoder {
public:
  nxdn_decoder(bool nxdn96, dv_pcm_callback callback, void *user_data)
    : callback_(callback), user_data_(user_data),
      frontend_(350.0f, nxdn96 ? 4800.0f : 2400.0f),
      // RRC alpha 0.2 like DMR (the spec's x/sin x pre-emphasis and its
      // inverse cancel), 8 symbols long; NXDN48 +/-1050 Hz, NXDN96 +/-2400 Hz outer
      fsk4_(dv_rrc_taps(nxdn96 ? 4800.0f : 2400.0f, 0.2f, nxdn96 ? 41 : 81),
            nxdn96 ? 800.0f : 350.0f, nxdn96 ? 4800.0f : 2400.0f)
  {
    viterbi_reset_ctx(&viterbi_);
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

  void leveled(cfloat lev)
  {
    uint8_t dibit;
    if (fsk4_.process(lev, dibit))
      rx_dibit(dibit);
  }

  void get_stats(dv_stats *stats) const override
  {
    *stats = {};
    stats->call_serial = call_serial_;
    stats->samples = frontend_.samples;
    stats->symbols = n_;
    stats->syncs = syncs_;
    stats->frames = frames_;
    stats->voice_decoded = voice_decoded_;
    stats->voice_muted = voice_muted_;
    stats->freq_offset = frontend_.frequency_offset_hz() + fsk4_.fine_offset_hz();
    std::memcpy(stats->src_call, src_, sizeof src_);
    std::memcpy(stats->dst_call, dst_, sizeof dst_);
    stats->m17_can = -1;
    stats->error_rate = ambe_.error_rate();
  }

private:
  // Framer (trunk-recorder-pro frame.rs Framer::push)
  void rx_dibit(uint8_t dibit)
  {
    buf_.push_back(dibit);
    if (buf_.size() > kFrameSymbols)
      buf_.pop_front();
    win_ = ((win_ << 2) | (dibit & 3)) & 0xfffff;
    uint64_t const i = n_++;
    int const errs = __builtin_popcount(win_ ^ kFsw);
    int const errs_inv = __builtin_popcount(win_ ^ kFswInverted);
    bool const best_inv = errs_inv < errs;
    int const best = best_inv ? errs_inv : errs;
    bool const full = i + 1 >= kFswSymbols;
    if (locked_ && next_fsw_ == i) {
      int const e = inverted_ ? errs_inv : errs;
      pending_sync_ = e <= kGridErrs;
      frame_end_ = i + kBodySymbols;
      next_fsw_ = i + kFrameSymbols;
    } else if (locked_ && best <= kLockErrs && bad_ > 0 && full) {
      lock(i, best_inv); // a clean FSW off the grid after a missed one: the grid moved
    } else if (!locked_ && best <= kLockErrs && full) {
      lock(i, best_inv);
    }
    if (locked_ && i == frame_end_)
      emit();
  }

  void lock(uint64_t i, bool inverted)
  {
    locked_ = true;
    inverted_ = inverted;
    pending_sync_ = true;
    frame_end_ = i + kBodySymbols;
    next_fsw_ = i + kFrameSymbols;
    fresh_ = true;
    bad_ = 0;
  }

  void unlock()
  {
    locked_ = false;
    end_call();
  }

  void emit()
  {
    bool const sync = pending_sync_;
    bool const fresh = fresh_;
    pending_sync_ = fresh_ = false;
    if (buf_.size() < kFrameSymbols)
      return;
    uint8_t body[kBodySymbols];
    uint8_t const flip = inverted_ ? 2 : 0;
    for (int k = 0; k < kBodySymbols; k++)
      body[k] = buf_[kFswSymbols + k] ^ flip ^ (kScramble.t[k] << 1);

    int lich = -1;
    {
      uint8_t bits = 0;
      int inner = 0;
      for (int k = 0; k < 8; k++) {
        bits = (bits << 1) | (body[k] >> 1);
        inner += (body[k] & 1) == 0;
      }
      int const raw = bits >> 1;
      int const p = ((raw >> 6) ^ (raw >> 5) ^ (raw >> 4) ^ (raw >> 3)) & 1;
      if (p == (bits & 1) && inner <= 1)
        lich = raw;
    }
    if (fresh && lich < 0) {
      unlock(); // a 20-bit FSW is short: a lock off noise doesn't survive its LICH
      return;
    }
    // The LICH has no error correction. Within a voice call it doesn't
    // change, so when this one fails but the sync was where it belonged,
    // carry the last good one forward (only ever a voice LICH)
    if (lich >= 0)
      last_lich_ = lich;
    else if (sync && last_lich_ >= 0 && ((last_lich_ >> 3) & 3) >= 2)
      lich = last_lich_;
    if (sync || lich >= 0) {
      bad_ = 0;
    } else if (++bad_ >= kUnlockFrames) {
      unlock();
      return;
    }
    if (sync)
      syncs_++;
    frames_++;
    if (lich >= 0)
      frame(lich, body);
  }

  // A traffic or conventional frame's body, by its LICH (frame.rs Lich::body)
  void frame(int lich, uint8_t const *body)
  {
    int const rf = lich >> 5, fct = (lich >> 3) & 3, option = (lich >> 1) & 3;
    if (rf == 0)
      return; // control channel (CAC)
    if (rf == 3 && fct == 0)
      return; // Type-D single frame: FACCH1s only
    if (fct == 1)
      return; // UDCH / FACCH2: data
    bool const superframe = fct >= 2;
    bool const idle = fct == 3;
    bool const facch[2] = {(option & 2) == 0, (option & 1) == 0};

    uint8_t bits[2 * kBodySymbols];
    for (int k = 0; k < kBodySymbols; k++) {
      bits[2 * k] = body[k] >> 1;
      bits[2 * k + 1] = body[k] & 1;
    }

    // SACCH: SR (structure 2, RAN 6) and 18 bits; in a superframe four of
    // them (structure 3..0) make one 72-bit message
    uint8_t sacch[26];
    if (decode_channel(&viterbi_, kSacch, bits + kSacchAt, sacch)) {
      int const structure = (sacch[0] << 1) | sacch[1];
      uint32_t data = 0;
      for (int k = 8; k < 26; k++)
        data = (data << 1) | sacch[k];
      if (superframe) {
        int const part = 3 - structure;
        if (part == 0)
          sacch_have_ = 0;
        sacch_parts_[part] = data;
        sacch_have_ |= 1 << part;
        if (part == 3 && sacch_have_ == 0xf) {
          uint8_t msg[9];
          unsigned __int128 all = 0;
          for (int k = 0; k < 4; k++)
            all = (all << 18) | sacch_parts_[k];
          for (int k = 0; k < 9; k++)
            msg[k] = static_cast<uint8_t>(all >> (64 - 8 * k));
          message(msg);
        }
      }
    } else if (superframe) {
      sacch_have_ = 0;
    }

    for (int h = 0; h < 2; h++) {
      if (facch[h]) {
        uint8_t info[80];
        if (decode_channel(&viterbi_, kFacch1, bits + kHalfAt[h], info)) {
          uint8_t msg[10];
          for (int k = 0; k < 10; k++) {
            msg[k] = 0;
            for (int b = 0; b < 8; b++)
              msg[k] = (msg[k] << 1) | info[8 * k + b];
          }
          message(msg);
        }
      } else if (!idle) {
        voice(body + kVchAt + 36 * (2 * h));
        voice(body + kVchAt + 36 * (2 * h + 1));
      }
    }
  }

  // Layer 3 (layer3.rs): VCALL says who is talking to whom, and the cipher;
  // TX_REL ends the call
  void message(uint8_t const *o)
  {
    int const type = o[0] & 0x3f;
    if (type == 0x01) { // VCALL
      unsigned const source = (o[3] << 8) | o[4];
      unsigned const destination = (o[5] << 8) | o[6];
      int const call_type = o[2] >> 5;
      encrypted_ = (o[7] >> 6) != 0;
      std::snprintf(src_, sizeof src_, "%u", source);
      std::snprintf(dst_, sizeof dst_, "%s%u", call_type == 1 ? "TG " : "", destination);
    } else if (type == 0x08) { // TX_REL
      call_serial_++;
      end_call();
    }
  }

  void voice(uint8_t const *dibits)
  {
    int16_t pcm[DV_PCM_SAMPLES];
    bool const ok = ambe_.decode(dibits, pcm);
    if (encrypted_ || !ok) {
      std::memset(pcm, 0, sizeof pcm);
      voice_muted_++;
    } else {
      voice_decoded_++;
    }
    if (callback_ != nullptr)
      callback_(user_data_, pcm);
  }

  void signal_lost() override
  {
    call_serial_++;
    unlock(); // and the call
  }

  // call_serial_ advances only on a definitive end (an end-of-transmission
  // marker, or the signal lost); a lost sync lock ends the call's state but
  // not its identity, which radiod holds through a brief fade
  void end_call()
  {
    last_lich_ = -1;
    encrypted_ = false;
    sacch_have_ = 0;
    src_[0] = dst_[0] = '\0';
  }

  dv_pcm_callback const callback_;
  uint64_t call_serial_ = 0;
  void * const user_data_;
  dv_frontend frontend_;
  dv_fsk4 fsk4_;
  viterbi_ctx_t viterbi_;
  dv_ambe2450 ambe_;

  std::deque<uint8_t> buf_;
  uint32_t win_ = 0;
  uint64_t n_ = 0;
  bool locked_ = false;
  bool inverted_ = false;
  uint64_t next_fsw_ = 0;
  uint64_t frame_end_ = 0;
  bool pending_sync_ = false;
  bool fresh_ = false;
  int bad_ = 0;
  int last_lich_ = -1;

  uint32_t sacch_parts_[4]{};
  unsigned sacch_have_ = 0;
  bool encrypted_ = false;
  char src_[12]{};
  char dst_[12]{};

  uint64_t syncs_ = 0;
  uint64_t frames_ = 0;
  uint64_t voice_decoded_ = 0;
  uint64_t voice_muted_ = 0;
};

} // namespace

dv_decoder *dv_create_nxdn(bool nxdn96, dv_pcm_callback callback, void *user_data)
{
  return new nxdn_decoder(nxdn96, callback, user_data);
}
