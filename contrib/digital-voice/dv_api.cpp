// C API for the digital voice receivers
// SPDX-License-Identifier: GPL-3.0-or-later
#include "dv_decoder.h"

extern "C" dv_decoder *dv_create(dv_mode mode, dv_config const *config,
                                 dv_pcm_callback callback, void *user_data)
{
  dv_config const defaults{};
  if (config == nullptr)
    config = &defaults;
  try {
    switch (mode) {
    case DV_P25_C4FM:
    case DV_P25_CQPSK:
      if (config->p25_nac > 0xfff)
        return nullptr;
      return dv_create_p25(mode == DV_P25_CQPSK, config->p25_nac, callback, user_data);
    case DV_DMR:
      if (config->dmr_slot < 0 || config->dmr_slot > 2)
        return nullptr;
      return dv_create_dmr(config->dmr_slot, callback, user_data);
    case DV_M17:
      if (config->m17_can_filter && (config->m17_can < 0 || config->m17_can > 15))
        return nullptr;
      return dv_create_m17(config->m17_can_filter ? config->m17_can : -1, callback, user_data);
    case DV_YSF:
      return dv_create_ysf(callback, user_data);
    case DV_DSTAR:
      return dv_create_dstar(callback, user_data);
    case DV_NXDN48:
    case DV_NXDN96:
      return dv_create_nxdn(mode == DV_NXDN96, callback, user_data);
    case DV_AUTO:
      return dv_create_auto(*config, callback, user_data);
    }
  } catch (...) {
  }
  return nullptr;
}

extern "C" void dv_destroy(dv_decoder *decoder)
{
  delete decoder;
}

extern "C" void dv_process(dv_decoder *decoder, float const *iq, size_t count)
{
  // std::complex<float> is layout-compatible with float[2]
  decoder->process(reinterpret_cast<std::complex<float> const *>(iq), count);
}

extern "C" void dv_get_stats(dv_decoder const *decoder, dv_stats *stats)
{
  decoder->get_stats(stats);
}

extern "C" char const *dv_mode_name(int mode)
{
  switch (mode) {
  case DV_P25_C4FM: return "P25";
  case DV_P25_CQPSK: return "P25";
  case DV_DMR: return "DMR";
  case DV_M17: return "M17";
  case DV_YSF: return "YSF";
  case DV_DSTAR: return "DSTAR";
  case DV_NXDN48: return "NXDN";
  case DV_NXDN96: return "NXDN";
  case DV_AUTO: return "AUTO";
  default: return "";
  }
}
