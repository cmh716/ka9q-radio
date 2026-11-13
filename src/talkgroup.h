// Talkgroup names for trunked calls, from CSV files
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef _TALKGROUP_H
#define _TALKGROUP_H
#include <stdint.h>

// Where monitor and pcmrecord look by default
#define TALKGROUPS "/var/lib/ka9q-radio/talkgroups.d"

struct talkgroup {
  uint32_t id;
  char *tag;         // short name ("Alpha Tag"), never NULL
  char *description; // longer name, "" if none
  char *category;    // agency or group of talkgroups, "" if none
};

// Read a CSV file, or every *.csv in a directory. The first line names the
// columns: "Decimal" (the talkgroup number) and "Alpha Tag", optionally
// "Description" and "Category", in any order among others, as in a
// radioreference.com talkgroup download. Adds to what is already loaded.
// Returns the number of talkgroups now known, -1 if the path can't be read
int load_talkgroups(char const *path);

// The talkgroup a call is to: mode and dest as radiod reports them
// (CALL_MODE, CALL_DEST). NULL if it isn't a P25 talkgroup with a name
struct talkgroup const *lookup_talkgroup(char const *mode, char const *dest);
#endif
