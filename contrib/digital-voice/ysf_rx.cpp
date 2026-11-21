// Yaesu System Fusion (YSF/C4FM) voice receiver: 4FSK demodulation, frame
// sync, FICH decoding, DN (AMBE+2) and VW (IMBE) voice, DN callsigns
// Framing, FICH and voice decoding follow OP25 (boatbod/op25 718e589a)
// rx_sync.cc; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cassert>
#include <cstring>

#include "dv_decoder.h"
#include "ysf_gps.h"
#include "dv_frontend.h"
#include "upstream/bit_utils.h"
#include "upstream/check_frame_sync.h"
#include "upstream/crc16.h"
#include "upstream/frame_sync_magics.h"
#include "upstream/libm17/m17.h"
#include "upstream/mbelib.h"
#include "upstream/ambe.h" // needs mbelib.h first
#include "upstream/op25_imbe_frame.h"
#include "upstream/rs.h"
#include "upstream/software_imbe_decoder.h"
#include "upstream/ysf_const.h"

namespace {

constexpr int kFrameDibits = 480;   // 100 ms: sync 20, FICH 100, five 72-dibit voice/data blocks
static_assert(kFrameDibits == ysf_gps::kFrameSymbols, "ysf_gps times frames by it");
constexpr int kSyncDibits = 20;
constexpr int kExpiration = 960;    // dibits without a sync before giving up (two frames)
// Fusion deviation isn't pinned down here: OP25's transmitter uses DMR's
// (648 Hz per level), some radios may run wider. Centred at 750 Hz the FSK4
// level tracking decodes about 560-1000 Hz per level (1.7-3 kHz outer)
constexpr float kDeviation = 750.0f;

// FICH fields (32 bits)
constexpr int kFiHeader = 0, kFiCommunication = 1, kFiTerminator = 2;
constexpr int kDtVd1 = 0, kDtData = 1, kDtVd2 = 2, kDtVoiceFr = 3;

// De-interleave 100 dibits (20 x 5) and decode the K=5 rate 1/2 trellis
// code (G1 0x19, G2 0x17: the same code as M17's, so libm17's Viterbi
// decoder does it). OP25's ysf_decode_fich() instead walks the trellis with
// hard decisions and fails on any channel bit error. The callers' CRC
// checks reject wrong decodes
void ysf_trellis_decode(viterbi_ctx_t *ctx, uint8_t const src[100], uint8_t ans[96])
{
  uint16_t soft[200];
  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 5; j++) {
      uint8_t const sym = src[i + j * 20];
      soft[2 * (j + i * 5)] = (sym & 2) ? 0xffff : 0;     // G1 output
      soft[2 * (j + i * 5) + 1] = (sym & 1) ? 0xffff : 0; // G2 output
    }
  }
  uint8_t out[(100 + 4 + 7) / 8 + 1];
  viterbi_decode_ctx(ctx, out, soft, 200);
  for (int i = 0; i < 96; i++) // libm17 places the decoded bits one byte in
    ans[i] = (out[(i + 8) / 8] >> (7 - (i + 8) % 8)) & 1;
}

// rx_sync.cc ysf_decode_fich(): 100 dibits to the 32-bit FICH
bool ysf_decode_fich(viterbi_ctx_t *ctx, uint8_t const src[100], uint32_t& fich)
{
  uint8_t ans[96];
  ysf_trellis_decode(ctx, src, ans);
  uint8_t fich_bits[12 * 4];
  for (int k = 0; k < 4; k++)
    store_i(gly24128Dec(load_i(ans + 24 * k, 24)), fich_bits + 12 * k, 12);
  if (crc16(fich_bits, 48) != 0)
    return false;
  fich = static_cast<uint32_t>(load_i(fich_bits, 32));
  return true;
}

// The V/D mode 2 data channel (inverse of OP25 ysf_tx generate_dch_vd2):
// 100 dibits gathered from the five blocks, to 10 bytes of callsign text
bool ysf_decode_dch_vd2(viterbi_ctx_t *ctx, uint8_t const src[100], char out[11])
{
  uint8_t ans[96];
  ysf_trellis_decode(ctx, src, ans);
  if (crc16(ans, 96) != 0)
    return false;
  ysf_scramble(ans, 80);
  for (int c = 0; c < 10; c++)
    out[c] = static_cast<char>(load_i(ans + 8 * c, 8));
  out[10] = '\0';
  return true;
}

// Trim trailing blanks; reject anything that isn't printable
void tidy_callsign(char s[11])
{
  for (int i = 0; i < 10; i++)
    if (s[i] < 0x20 || s[i] > 0x7e)
      s[i] = ' ';
  for (int i = 9; i >= 0 && s[i] == ' '; i--)
    s[i] = '\0';
}

class ysf_decoder : public dv_decoder {
  friend struct ysf_via_test;
public:
  ysf_decoder(dv_pcm_callback callback, void *user_data)
    : callback_(callback), user_data_(user_data), frontend_(350.0f), fsk4_(DV_RRC_TAPS, kDeviation)
  {
    mbe_initMbeParms(&cur_mp_, &prev_mp_, &enh_mp_);
    mbe_initErrParms(&errs_mp_);
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
    stats->symbols = symbols_;
    stats->syncs = syncs_;
    stats->frames = frames_;
    stats->voice_decoded = voice_decoded_;
    stats->voice_muted = voice_muted_;
    stats->freq_offset = frontend_.frequency_offset_hz() + fsk4_.fine_offset_hz();
    std::memcpy(stats->src_call, src_, sizeof src_);
    std::memcpy(stats->dst_call, dst_, sizeof dst_);
    via(stats->message, sizeof stats->message);
    // The position is its sender's, as with D-STAR
    if (gps_.valid() && src_[0] != '\0' && std::strcmp(src_, position_owner_) == 0) {
      stats->has_position = true;
      stats->latitude = gps_.latitude();
      stats->longitude = gps_.longitude();
    }
    if (gps_.last_message()[0] != '\0')
      std::snprintf(stats->gps_line, sizeof stats->gps_line, "%s%s%s", gps_.last_message(),
                    gps_.radio()[0] ? " " : "", gps_.radio());
    stats->m17_can = -1;
  }

private:
  // rx_sync::rx_sym(), YSF only
  void rx_dibit(uint8_t dibit)
  {
    symbols_++;
    symbol_count_++;
    sync_reg_ = (sync_reg_ << 2) | (dibit & 3);
    bool const sync_detected = check_frame_sync(YSF_FRAME_SYNC_MAGIC ^ sync_reg_,
                                                locked_ ? threshold_ : 0, 2 * kSyncDibits);
    std::memmove(buf_, buf_ + 1, sizeof buf_ - 1);
    buf_[sizeof buf_ - 1] = dibit;
    if (!locked_ && !sync_detected)
      return;
    rx_count_++;
    if (sync_detected) {
      syncs_++;
      if (!locked_) {
        locked_ = true;
        rx_count_ = kSyncDibits;
      } else if (rx_count_ != kSyncDibits) {
        sync_reset();
        locked_ = true;
        rx_count_ = kSyncDibits;
      } else {
        threshold_ = std::min(threshold_ + 1, 2);
      }
      expires_ = symbol_count_ + kExpiration;
    }
    if (locked_ && symbol_count_ >= expires_) {
      sync_reset();
      return;
    }
    if (rx_count_ < kFrameDibits)
      return;
    rx_count_ = 0;
    frame(buf_);
  }

  void signal_lost() override
  {
    call_serial_++;
    sync_reset();
  }

  void sync_reset()
  {
    symbol_count_ = 0;
    rx_count_ = 0;
    threshold_ = 0;
    sync_reg_ = 0;
    expires_ = 0;
    locked_ = false;
    end_call();
  }

  // call_serial_ advances only on a definitive end (an end-of-transmission
  // marker, or the signal lost); a lost sync lock ends the call's state but
  // not its identity, which radiod holds through a brief fade
  void end_call()
  {
    dt_ = kDtVd1; // OP25 starts muted until a FICH says voice
    new_transmission();
  }

  // What a frame's FICH says of the transmission. A terminator ends it (true:
  // nothing more to do with the frame). A header (its FICH checked) starts
  // one: one still under way lost its terminator, and its callers and
  // position are not the new one's
  bool boundary(bool fich_ok, int fi)
  {
    if (fi == kFiTerminator) {
      call_serial_++;
      end_call();
      return true;
    }
    if (fich_ok && fi == kFiHeader) {
      // A transmission was under way if anything of one is known: it ends
      if (src_[0] != '\0' || dst_[0] != '\0' || down_[0] != '\0' || up_[0] != '\0' || gps_.valid())
        call_serial_++;
      new_transmission(); // always: nothing of the last carries into this one
    }
    return false;
  }

  // Forget the transmission's callers and position
  void new_transmission()
  {
    src_[0] = dst_[0] = down_[0] = up_[0] = '\0';
    gps_.reset();
    position_owner_[0] = '\0';
  }

  // A callsign field of the data channel, frame fn 0-3: destination,
  // source, downlink, uplink. A call to everyone has its destination '*'
  // fill, all of it or the first five and the sender's 5-character radio ID
  // ("*****EAFAJ"): "ALL", as D-STAR's is "CQCQCQ", however a superframe
  // has it, lest the call seem to change. A field with characters no
  // callsign, room or ID has passed its CRC by chance ("3~^bl q"): left out
  void store_field(int fn, char call[11])
  {
    // The field as sent, all 10 bytes, before any tidying: a control byte
    // must not become a blank that trims away ("W1AW" + a stray byte)
    static char const allowed[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789 */-.#";
    bool padding = false; // NUL padding may end a field (blanks or NULs after it only)
    for (int i = 0; i < 10; i++) {
      if (call[i] == '\0')
        padding = true;
      else if (std::strchr(allowed, call[i]) == nullptr || (padding && call[i] != ' '))
        return;
    }
    tidy_callsign(call);
    if (fn == 0 && std::strncmp(call, "*****", 5) == 0)
      std::strcpy(call, "ALL");
    char *const field[4] = {dst_, src_, down_, up_};
    std::strncpy(field[fn], call, sizeof src_ - 1);
    // A position that came before any callsign is the first one's, once
    if (fn == 1 && gps_.valid() && position_owner_[0] == '\0')
      std::memcpy(position_owner_, src_, sizeof src_);
  }

  // The way the call came: "via UPLINK/DOWNLINK" (the repeater or node it
  // went up through, the gateway or room it came down from), blank fields
  // and the radios' "*****" fill left out; "" when neither is known
  void via(char *out, size_t size) const
  {
    auto known = [](char const *s) { return s[0] != '\0' && std::strspn(s, "*") != std::strlen(s); };
    bool const up = known(up_), down = known(down_);
    if (up && down && std::strcmp(up_, down_) != 0)
      std::snprintf(out, size, "via %s/%s", up_, down_);
    else if (up || down)
      std::snprintf(out, size, "via %s", up ? up_ : down_);
    else
      out[0] = '\0';
  }

  void frame(uint8_t const *dibits)
  {
    frames_++;
    // rx_sync::ysf_sync(): the data type persists until a FICH decodes
    uint32_t fich;
    int fi = kFiCommunication;
    int fn = -1, ft = -1;
    bool const fich_ok = ysf_decode_fich(&viterbi_, dibits + kSyncDibits, fich);
    if (fich_ok) {
      fi = (fich >> 30) & 3;
      fn = (fich >> 19) & 7;
      ft = (fich >> 16) & 7; // the superframe's last frame number
      dt_ = (fich >> 8) & 3;
    }
    if (boundary(fich_ok, fi))
      return; // a terminator
    bool const unmute = dt_ == kDtVd2 || dt_ == kDtVoiceFr;
    bool const fullrate = dt_ == kDtVoiceFr;

    // V/D mode 2 communication frames carry callsigns in their data channel,
    // one field per frame number: destination, source, downlink, uplink;
    // frames 6 and 7 a GPS message
    if (dt_ == kDtVd2 && fi == kFiCommunication && fn == 0)
      gps_.superframe();
    if (dt_ == kDtVd2 && fi == kFiCommunication && ((fn >= 0 && fn <= 3) || fn == 6 || fn == 7)) {
      uint8_t dch[100];
      for (int v = 0; v < 5; v++)
        std::memcpy(dch + 20 * v, dibits + v * 72 + 120, 20);
      char call[11];
      if (ysf_decode_dch_vd2(&viterbi_, dch, call)) {
        if (fn <= 3)
          store_field(fn, call);
        else if (gps_.frame(fn, ft, reinterpret_cast<uint8_t const *>(call), symbols_))
          std::memcpy(position_owner_, src_, sizeof src_); // whose: "" if not yet known (store_field binds it)
      }
    }

    // Header frames carry no voice
    if (!unmute || fi == kFiHeader) {
      if (unmute)
        voice_muted_ += 5;
      return;
    }
    uint8_t bits[2 * kFrameDibits];
    dibits_to_bits(bits, dibits, kFrameDibits);
    for (int v = 0; v < 5; v++) {
      if (fullrate)
        codeword_fullrate(bits + 2 * (v * 72 + 120));      // 144 bits
      else
        codeword_halfrate(bits + 2 * (v * 72 + 120 + 20)); // 104 bits
    }
  }

  // rx_sync::codeword(), CODEWORD_YSF_HALFRATE case and common tail
  void codeword_halfrate(uint8_t const *cw)
  {
    static constexpr int x = 4, y = 26;
    static constexpr uint8_t majority[8] = {0, 0, 0, 1, 0, 1, 1, 1};
    uint8_t buf[4 * 26];
    uint8_t tmp_codeword[49];
    int b[9];
    for (int i = 0; i < x; i++)
      for (int j = 0; j < y; j++)
        buf[j + i * y] = cw[i + j * x];
    ysf_scramble(buf, 104);
    for (int i = 0; i < 27; i++)
      tmp_codeword[i] = majority[(buf[0 + i * 3] << 2) | (buf[1 + i * 3] << 1) | buf[2 + i * 3]];
    std::memcpy(tmp_codeword + 27, buf + 81, 22);
    decode_49bit(b, tmp_codeword);
    bool do_silence = false;
    if (b[0] < 120) // TODO in OP25: frame repeats and tones
      mbe_dequantizeAmbe2250Parms(&cur_mp_, &prev_mp_, &errs_mp_, b);
    else
      do_silence = true;
    mbe_moveMbeParms(&cur_mp_, &prev_mp_);
    if (!do_silence)
      synth_.decode_tap(cur_mp_.L, 0, cur_mp_.w0, &cur_mp_.Vl[1], &cur_mp_.Ml[1]);
    output(do_silence, 1.0f);
  }

  // rx_sync::codeword(), CODEWORD_YSF_FULLRATE case
  void codeword_fullrate(uint8_t const *cw)
  {
    voice_codeword fullrate_cw(voice_codeword_sz);
    for (int i = 0; i < 144; i++)
      fullrate_cw[i] = cw[ysf_permutation[i]];
    mbe_moveMbeParms(&cur_mp_, &prev_mp_);
    synth_.decode(fullrate_cw);
    // OP25 multiplies this by 32768, but software_imbe_decoder::decode()
    // already produces 16-bit-scale samples (as decode_tap() does), so that
    // clips every sample
    output(false, 1.0f);
  }

  void output(bool do_silence, float scale)
  {
    audio_samples *samples = synth_.audio();
    int16_t pcm[DV_PCM_SAMPLES];
    for (int i = 0; i < DV_PCM_SAMPLES; i++) {
      float snd = 0;
      if (!do_silence && !samples->empty()) {
        snd = samples->front() * scale;
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

  dv_pcm_callback const callback_;
  uint64_t call_serial_ = 0;
  void * const user_data_;
  dv_frontend frontend_;
  dv_fsk4 fsk4_;

  // Framing (rx_sync)
  uint8_t buf_[kFrameDibits]{};
  uint64_t sync_reg_ = 0;
  uint64_t symbol_count_ = 0;
  uint64_t expires_ = 0;
  int rx_count_ = 0;
  int threshold_ = 0;
  bool locked_ = false;
  int dt_ = kDtVd1;

  viterbi_ctx_t viterbi_;

  // Vocoder (rx_sync)
  mbe_parms cur_mp_, prev_mp_, enh_mp_;
  mbe_errs errs_mp_;
  software_imbe_decoder synth_;

  char src_[12]{};
  char dst_[12]{};
  char down_[12]{};  // downlink: the gateway or room the call came down from
  char up_[12]{};    // uplink: the repeater or node it went up through
  ysf_gps gps_;      // the position, from frames 6 and 7
  char position_owner_[12]{}; // whose: src_ when it came
  uint64_t symbols_ = 0;
  uint64_t syncs_ = 0;
  uint64_t frames_ = 0;
  uint64_t voice_decoded_ = 0;
  uint64_t voice_muted_ = 0;
};

} // namespace

dv_decoder *dv_create_ysf(dv_pcm_callback callback, void *user_data)
{
  return new ysf_decoder(callback, user_data);
}
