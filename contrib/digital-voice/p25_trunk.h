// P25 Phase 1 control channel: TSDU trunking signalling blocks (TSBKs),
// the site's band plan and identity, and the voice channel grants
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_P25_TRUNK_H
#define KA9Q_P25_TRUNK_H

#include <array>
#include <cstdint>
#include <map>
#include <vector>

#include "dv.h"

class p25_trunk {
public:
  // One TSDU from the framer: frame_body (status bits included) and its length in bits
  void tsdu(std::vector<bool> const& frame, size_t bits, uint16_t nac);
  // Advances the clock (in symbols) that ages calls out
  void tick(uint64_t symbols) { now_ = symbols; }
  void get_stats(dv_stats *stats) const;
  void set_callback(dv_event_callback callback, void *user_data)
  {
    callback_ = callback;
    user_data_ = user_data;
  }

  // Decodes one interleaved 196-bit block (one bit per byte) into 12 octets;
  // returns the number of bit errors the trellis decoder corrected
  static int block_decode(uint8_t const *bits, uint8_t out[12]);
  static bool crc_ok(uint8_t const block[12]);

private:
  struct iden {
    bool valid = false;
    uint64_t base_hz = 0;
    uint32_t spacing_hz = 0;
    int64_t tx_offset_hz = 0;
    uint32_t bandwidth_hz = 0;
    int slots = 1;            // TDMA slots per carrier
  };
  struct call {
    uint32_t group = 0;
    uint32_t source = 0;
    uint64_t seen = 0;
    uint8_t opts = 0;         // service options of the grant (updates carry none)
    uint32_t transmission = 0; // sequence number of its explicit grant
  };

  void tsbk(uint8_t const t[12], uint16_t nac);
  void set_iden(uint8_t opcode, uint8_t const t[12]);
  // explicit: a channel grant message (a new transmission), not an update
  void grant(uint16_t ch, uint32_t group, uint32_t source, uint8_t opts, bool unit, bool explicit_grant);
  // Channel number to (downlink frequency, TDMA slot); 0 Hz if its band plan isn't known yet
  uint64_t channel_hz(uint16_t ch, int *slot) const;
  void emit(dv_event& e);

  std::array<iden, 16> idens_{};
  std::map<uint16_t, call> calls_; // by channel
  dv_event_callback callback_ = nullptr;
  void *user_data_ = nullptr;
  uint64_t now_ = 0;
  uint64_t tsbks_ = 0;
  uint64_t tsbk_errors_ = 0;
  uint32_t wacn_ = 0;
  uint16_t sysid_ = 0;
  uint8_t rfss_ = 0, site_ = 0;
  bool site_known_ = false;
  uint64_t grants_ = 0;
  uint32_t transmissions_ = 0; // explicit grants seen
};

#endif
