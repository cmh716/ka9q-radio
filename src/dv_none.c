// radiod built without the digital voice modes (make ENABLE_DV=0): their
// channels are refused, and nothing needs C++ or libcodec2
// SPDX-License-Identifier: GPL-3.0-or-later
#include <stdio.h>

#include "radio.h"

int demod_dv(void *arg){
  chan_t * const chan = arg;
  fprintf(stderr,"%s: this radiod was built without digital voice (ENABLE_DV=0)\n",chan->name);
  chan->demod_type = INVALID_DEMOD;
  return -1;
}
