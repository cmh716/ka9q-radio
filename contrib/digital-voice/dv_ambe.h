// AMBE+2 (3600x2450) voice decoding of one 72-bit codeword, as DMR and NXDN
// carry it: OP25 rx_sync.cc codeword(), CODEWORD_DMR case
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_DV_AMBE_H
#define KA9Q_DV_AMBE_H

#include <cstdint>

#include "dv.h"
#include "upstream/mbelib.h"
#include "upstream/p25p2_vf.h"
#include "upstream/software_imbe_decoder.h"

class dv_ambe2450 {
public:
  dv_ambe2450();
  // Decode one codeword (36 dibits, as received) into 20 ms of audio;
  // returns false if the frame was muted (too many errors, or repeats ran out)
  bool decode(uint8_t const dibits[36], int16_t pcm[DV_PCM_SAMPLES]);
  float error_rate() const { return errs_.ER; } // running bit error rate estimate

private:
  p25p2_vf interleaver_;
  mbe_parms cur_, prev_, enh_;
  mbe_errs errs_;
  mbe_tone tone_;
  int err_cnt_ = 0;
  software_imbe_decoder synth_;
};

#endif
