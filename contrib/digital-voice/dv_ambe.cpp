// AMBE+2 (3600x2450) voice decoding: OP25 rx_sync.cc codeword(), CODEWORD_DMR case
// SPDX-License-Identifier: GPL-3.0-or-later
#include "dv_ambe.h"

#include <algorithm>

#include "upstream/ambe.h" // needs mbelib.h first

dv_ambe2450::dv_ambe2450()
{
  mbe_initMbeParms(&cur_, &prev_, &enh_);
  mbe_initErrParms(&errs_);
  mbe_initToneParms(&tone_);
}

bool dv_ambe2450::decode(uint8_t const dibits[36], int16_t pcm[DV_PCM_SAMPLES])
{
  int b[9];
  int U[4];
  bool do_silence = false;
  bool do_tone = false;

  interleaver_.process_vcw(&errs_, dibits, b, U);
  int rc = mbe_dequantizeAmbeTone(&tone_, &errs_, U);
  if (rc >= 0) { // Tone frame
    if (rc == 0) {
      do_tone = true;
      err_cnt_ = 0;
    } else if ((++err_cnt_ < 4) && do_tone) {
      mbe_useLastMbeParms(&cur_, &prev_);
    } else {
      do_tone = false;
      do_silence = true;
    }
  } else {
    rc = mbe_dequantizeAmbe2250Parms(&cur_, &prev_, &errs_, b);
    if (rc == 0) { // Voice frame
      do_tone = false;
      err_cnt_ = 0;
    } else if ((++err_cnt_ < 4) && !do_tone) { // Erasure with frame repeat per TIA-102.BABA.5.6
      mbe_useLastMbeParms(&cur_, &prev_);
    } else {
      do_tone = false; // Mute after 3 successive frame repeats
      do_silence = true;
    }
  }
  if (errs_.ER > 0.096) { // Mute if error rate exceeds threshold
    do_tone = false;
    do_silence = true;
  }

  if (do_tone) {
    synth_.decode_tone(tone_.ID, tone_.AD, &tone_.n);
  } else {
    mbe_moveMbeParms(&cur_, &prev_);
    if (!do_silence)
      synth_.decode_tap(cur_.L, 0, cur_.w0, &cur_.Vl[1], &cur_.Ml[1]);
  }
  audio_samples *samples = synth_.audio();
  for (int i = 0; i < DV_PCM_SAMPLES; i++) {
    float snd = 0;
    if (!do_silence && !samples->empty()) {
      snd = samples->front();
      samples->pop_front();
    }
    pcm[i] = static_cast<int16_t>(std::clamp(snd, -32768.0f, 32767.0f));
  }
  return !do_silence;
}
