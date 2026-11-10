// Internal base class behind the dv.h C API
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_DV_DECODER_H
#define KA9Q_DV_DECODER_H

#include <complex>
#include <cstddef>

#include "dv.h"

struct dv_decoder {
  virtual ~dv_decoder() = default;
  // 48 kHz IQ through the decoder's own front end
  virtual void process(std::complex<float> const *samples, size_t count) = 0;
  // Or 24 kHz samples from a dv_frontend shared with other decoders (DV_AUTO);
  // the decoder's own front end then sits idle (its stats show 0 samples and
  // no FLL offset)
  virtual void process_leveled(std::complex<float> const *samples, size_t count) = 0;
  virtual void get_stats(dv_stats *stats) const = 0;
};

dv_decoder *dv_create_p25(bool cqpsk, uint16_t nac, dv_pcm_callback callback, void *user_data);
dv_decoder *dv_create_dmr(int slot, dv_pcm_callback callback, void *user_data);
dv_decoder *dv_create_m17(int can, dv_pcm_callback callback, void *user_data); // can -1 = any
dv_decoder *dv_create_ysf(dv_pcm_callback callback, void *user_data);
dv_decoder *dv_create_dstar(dv_pcm_callback callback, void *user_data);
dv_decoder *dv_create_nxdn(bool nxdn96, dv_pcm_callback callback, void *user_data);
dv_decoder *dv_create_auto(dv_config const& config, dv_pcm_callback callback, void *user_data);

#endif
