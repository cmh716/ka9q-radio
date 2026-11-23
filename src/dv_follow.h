// Trunking (P25, NXDN Type-C, LTR): follow a control channel's voice grants
// from the control channel's own thread, on one or more voice lanes
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef _DV_FOLLOW_H
#define _DV_FOLLOW_H 1

#include <stdbool.h>
#include <stddef.h>
#include "radio.h"
#include "dv.h"

#define DV_FOLLOW_LANES 8      // voice-channels: at most this many calls at once

struct dv_follow;

// chan: the control channel. Lane 0 plays on it; lanes 1..nlanes-1 on
// lane_chans[i] (output channels without threads of their own: the control
// channel's thread sends their audio and status). Speech from the call lane
// i follows goes to pcm(pcm_users[i], ...)
struct dv_follow *dv_follow_create(chan_t *chan, int nlanes, chan_t *const *lane_chans,
				   dv_pcm_callback pcm, void *const *pcm_users);
void dv_follow_destroy(struct dv_follow *f);
int dv_follow_lanes(struct dv_follow const *f);
// Control channel events (from dv_set_event_callback)
void dv_follow_event(struct dv_follow *f, dv_event const *e);
// After each block of the control channel: count samples, with its stats
void dv_follow_block(struct dv_follow *f, dv_stats const *cc, int count);
// For monitor: "TG<group> <MHz>" of the call lane follows; between calls,
// lane 0 says how the control channel is ("CC N469", "no CC", "CC search").
// false if there's nothing to say (an idle lane but the first)
bool dv_follow_source(struct dv_follow const *f, int lane, char *buf, size_t len);
// The call lane follows: talkgroup (or unit) and source as decimal strings,
// its voice channel frequency and TDMA slot (-1 for FDMA); false if none.
// generation identifies the voice decoder; serial counts its call endings
bool dv_follow_call(struct dv_follow const *f, int lane, char *dest, size_t dlen, char *src, size_t slen,
		    double *freq, int *slot, uint64_t *generation, uint64_t *serial);
void dv_follow_print(struct dv_follow const *f);

#endif
