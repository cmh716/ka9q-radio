// P25 Phase 1 frame layer: dibits in, IMBE speech out
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_P25_VOICE_H
#define KA9Q_P25_VOICE_H

#include <cstdint>
#include <memory>

#include "dv.h"

class p25_voice {
public:
  p25_voice(uint16_t nac, dv_pcm_callback callback, void *user_data);
  ~p25_voice();
  p25_voice(const p25_voice&) = delete;
  p25_voice& operator=(const p25_voice&) = delete;

  void rx_dibit(uint8_t dibit);
  // Fills the frame-layer fields of stats
  void get_stats(dv_stats *stats) const;
  // Control channel events (P25 trunking)
  void set_event_callback(dv_event_callback callback, void *user_data);
  // End the current call: encryption and IDs are forgotten
  void signal_lost();

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

#endif
