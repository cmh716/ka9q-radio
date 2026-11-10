// Digital voice receivers (P25 Phase 1, DMR, M17, YSF, D-STAR, NXDN): 48 kHz complex IQ in, 8 kHz speech out
// Adapted from OP25; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_DV_H
#define KA9Q_DV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DV_IQ_SAMPRATE 48000
#define DV_PCM_SAMPRATE 8000
#define DV_PCM_SAMPLES 160   // One vocoder frame, 20 ms

typedef enum {
  DV_P25_C4FM = 0,
  DV_P25_CQPSK = 1,  // P25 simulcast (LSM/CQPSK)
  DV_DMR = 2,
  DV_M17 = 3,
  DV_YSF = 4,        // Yaesu System Fusion (C4FM): DN and VW voice
  DV_DSTAR = 5,      // D-STAR (GMSK) voice
  DV_NXDN48 = 6,     // NXDN, 2400 baud (6.25 kHz)
  DV_NXDN96 = 7,     // NXDN, 4800 baud (12.5 kHz)
  DV_AUTO = 8,       // all of the above but P25 CQPSK, whichever is on the air
  DV_P25P2 = 9,      // P25 Phase 2 (TDMA) voice channel; needs p25_wacn, p25_sysid, p25_nac
} dv_mode;

typedef struct {
  uint16_t p25_nac;  // P25: NAC to accept, 0 = any; Phase 2: the system's NAC (for descrambling)
  uint32_t p25_wacn; // P25 Phase 2: WACN and system ID (for descrambling)
  uint16_t p25_sysid;
  int p25_slot;      // P25 Phase 2: TDMA slot 0 or 1
  int dmr_slot;      // DMR: timeslot 1 or 2; 0 = whichever starts a call first
  bool m17_can_filter; // M17: play only transmissions with this CAN
  int m17_can;       // M17: Channel Access Number 0-15, if m17_can_filter
} dv_config;

// Called synchronously from dv_process() for each 20 ms frame of speech
typedef void (*dv_pcm_callback)(void *user_data, int16_t const pcm[DV_PCM_SAMPLES]);

typedef struct {
  uint64_t samples;          // complex input samples
  uint64_t symbols;
  uint64_t syncs;            // frame sync words seen
  uint64_t frames;           // frames decoded (P25: passed NID/NAC check; DMR: bursts)
  uint64_t voice_decoded;    // vocoder frames played
  uint64_t voice_muted;      // vocoder frames muted (P25: encrypted; DMR: other slot or too many errors)
  float freq_offset;         // Hz, estimated carrier offset
  // P25
  uint64_t ldu1, ldu2;
  uint16_t last_nac;
  uint32_t last_bch_errors;
  // DMR
  int active_slot;           // slot being played, 1 or 2; 0 = none
  int dmr_cc;                // color code, -1 if not yet known
  float error_rate;          // DMR: AMBE bit error rate estimate of the active slot; M17: Viterbi
  // M17, YSF and D-STAR callsigns, NXDN unit IDs, of the current transmission; "" if not yet known
  char src_call[12];
  char dst_call[12];
  // M17, from the current transmission's link setup frame
  int m17_can;               // Channel Access Number, -1 if not yet known
  bool m17_encrypted;        // M17, and DMR (privacy bit or PI header)
  // DV_AUTO: the mode whose call is being played, -1 if none (the other
  // fields are then that decoder's)
  int active_mode;
  // P25 control channel (TSDU): trunking blocks that passed / failed their CRC,
  // and the site, once broadcast (wacn 0 = not yet known)
  uint64_t tsbks, tsbk_errors;
  uint32_t wacn;
  uint16_t sysid;
  uint8_t rfss, site;
} dv_stats;

// P25 control channel events
typedef enum {
  DV_EVENT_GRANT = 0,   // a new call on a voice channel (or a new talkgroup on it)
  DV_EVENT_SITE = 1,    // site identity or band plan changed
  DV_EVENT_UPDATE = 2,  // a call already granted is still going (fields as GRANT; text empty)
} dv_event_type;

typedef struct {
  dv_event_type type;
  uint64_t freq_hz;     // GRANT: voice channel downlink, 0 if its band plan isn't known yet
  int slot;             // GRANT: TDMA slot 0 or 1 (Phase 2), -1 for FDMA (Phase 1)
  uint16_t channel;     // GRANT: identifier (4 bits) and channel number (12 bits)
  uint32_t group;       // GRANT: talkgroup, or target unit for a unit-to-unit call
  uint32_t source;      // GRANT: source unit, 0 if not given
  bool unit;            // GRANT: unit-to-unit call
  bool encrypted;       // GRANT: service options say protected
  bool emergency;
  char text[128];       // readable summary for logging
} dv_event;

typedef void (*dv_event_callback)(void *user_data, dv_event const *event);

typedef struct dv_decoder dv_decoder;

// Returns NULL on failure (bad mode or config)
dv_decoder *dv_create(dv_mode mode, dv_config const *config,
                      dv_pcm_callback callback, void *user_data);
void dv_destroy(dv_decoder *decoder);

// iq: count interleaved I/Q float pairs at DV_IQ_SAMPRATE
// (a C 'float complex' array may be passed directly)
void dv_process(dv_decoder *decoder, float const *iq, size_t count);
void dv_get_stats(dv_decoder const *decoder, dv_stats *stats);
// P25: called synchronously from dv_process() for control channel events
void dv_set_event_callback(dv_decoder *decoder, dv_event_callback callback, void *user_data);
// Short display name of a mode ("P25", "DMR", "M17", "YSF", "DSTAR", "NXDN"), "" if none
char const *dv_mode_name(int mode);

#ifdef __cplusplus
}
#endif

#endif
