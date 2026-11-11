// P25 trunking: follow a control channel's voice grants on the same channel
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef _DV_FOLLOW_H
#define _DV_FOLLOW_H 1

#include <stdbool.h>
#include <stddef.h>
#include "radio.h"
#include "dv.h"

struct dv_follow;

// Speech from the followed calls goes to pcm(pcm_user, ...)
struct dv_follow *dv_follow_create(chan_t *chan, dv_pcm_callback pcm, void *pcm_user);
void dv_follow_destroy(struct dv_follow *f);
// Control channel events (from dv_set_event_callback)
void dv_follow_event(struct dv_follow *f, dv_event const *e);
// After each block of the control channel: count samples, with its stats
void dv_follow_block(struct dv_follow *f, dv_stats const *cc, int count);
// "TG<group> <MHz>" of the call being followed, for monitor; false if none
bool dv_follow_source(struct dv_follow const *f, char *buf, size_t len);
// The followed call's talkgroup (or unit) and source as decimal strings; false if none
bool dv_follow_call(struct dv_follow const *f, char *dest, size_t dlen, char *src, size_t slen);
void dv_follow_print(struct dv_follow const *f);

#endif
