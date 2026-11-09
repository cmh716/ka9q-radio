// Native P25 Phase 1 demodulator for radiod
// The decoder library (contrib/p25-native) takes 48 kHz complex IQ and
// emits 8 kHz IMBE speech in bursts of 9 frames per LDU; a small queue
// paces that out at real time
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "misc.h"
#include "radio.h"
#include "sched.h"
#include "p25_native.h"

#define P25_AUDIO_QUEUE_DEPTH 45 // 900 ms

struct p25_audio_queue {
  int16_t samples[P25_AUDIO_QUEUE_DEPTH][P25_NATIVE_PCM_SAMPLES];
  unsigned int head;
  unsigned int count;
  uint64_t dropped;
};

static void p25_pcm_callback(void *user_data, int16_t const pcm[P25_NATIVE_PCM_SAMPLES]){
  struct p25_audio_queue * const queue = user_data;
  if(queue->count == P25_AUDIO_QUEUE_DEPTH){
    queue->dropped++;
    return;
  }
  unsigned int const tail = (queue->head + queue->count) % P25_AUDIO_QUEUE_DEPTH;
  memcpy(queue->samples[tail],pcm,sizeof queue->samples[tail]);
  queue->count++;
}

int demod_p25(void *arg){
  chan_t * const chan = arg;
  assert(chan != NULL);
  if(chan == NULL)
    return -1;

  // Like WFM, the output rate is fixed by the mode, whatever was set before
  _Static_assert(P25_IQ_SAMPRATE == P25_NATIVE_IQ_SAMPRATE, "P25 IQ rate mismatch");
  _Static_assert(P25_AUDIO_SAMPRATE == P25_NATIVE_PCM_SAMPRATE, "P25 audio rate mismatch");
  chan->output.samprate = P25_AUDIO_SAMPRATE;
  chan->output.channels = 1;
  {
    int const pt = pt_from_info(chan->output.samprate,chan->output.channels,chan->output.encoding);
    if(pt >= 0)
      chan->output.rtp.type = (uint8_t)pt;
  }
  int const blocksize = (int)lrint(P25_IQ_SAMPRATE * Blocktime);
  if(create_filter_output(&chan->filter.out,&chan->frontend->in,blocksize,COMPLEX) != 0){
    fprintf(stderr,"%s: couldn't create P25 channel filter\n",chan->name);
    chan->demod_type = INVALID_DEMOD;
    return -1;
  }
  set_channel_filter(chan);

  struct p25_audio_queue audio_queue = {0};
  p25_native_decoder * const decoder =
    p25_native_create(chan->p25.modulation == P25_CQPSK ? P25_NATIVE_CQPSK : P25_NATIVE_C4FM,
		      chan->p25.nac,p25_pcm_callback,&audio_queue);
  if(decoder == NULL){
    fprintf(stderr,"%s: couldn't create native P25 decoder\n",chan->name);
    chan->demod_type = INVALID_DEMOD;
    return -1;
  }

  // Emit one 20 ms audio frame per 20 ms of IQ, in integer arithmetic
  uint64_t output_clock = 0;
  uint64_t const audio_frame_clock = (uint64_t)P25_IQ_SAMPRATE * P25_NATIVE_PCM_SAMPLES;
  uint64_t reported_drops = 0;
  int telemetry_samples = 0;
  chan->tp1 = 0; // detected P25 sync words
  chan->tp2 = 0; // decoded P25 frames
  chan->p25.have_nac = false;
  bool restart_needed = false;
  realtime(chan->prio);

  while(!restart_needed){
    restart_needed = handle_commands(chan);
    int r;
    if(restart_needed || (r = downconvert(chan)) == -1)
      break; // restart or terminate
    else if(r == 1)
      continue; // channel inactive; poll for commands

    int const count = chan->sampcount;
    // float complex is layout-compatible with interleaved I/Q floats
    p25_native_process(decoder,(float const *)chan->baseband,(size_t)count);

    telemetry_samples += count;
    if(telemetry_samples >= P25_IQ_SAMPRATE){
      telemetry_samples -= P25_IQ_SAMPRATE;
      p25_native_stats stats;
      p25_native_get_stats(decoder,&stats);
      chan->tp1 = (double)stats.syncs;
      chan->tp2 = (double)stats.frames;
      chan->sig.foffset = stats.freq_offset;
      if(stats.frames != 0){
	chan->p25.last_nac = stats.last_nac;
	chan->p25.have_nac = true;
      }
      if(Verbose > 0)
	fprintf(stderr,"%s P25 samples=%" PRIu64 " symbols=%" PRIu64
		" syncs=%" PRIu64 " frames=%" PRIu64
		" LDU1=%" PRIu64 " LDU2=%" PRIu64
		" voice=%" PRIu64 " encrypted=%" PRIu64
		" NAC=%03x BCH=%" PRIu32 " offset=%+.1f Hz\n",
		chan->name,stats.samples,stats.symbols,stats.syncs,stats.frames,
		stats.ldu1,stats.ldu2,stats.voice_decoded,stats.voice_encrypted,
		stats.last_nac,stats.last_bch_errors,stats.freq_offset);
    }
    if(audio_queue.dropped != reported_drops){
      fprintf(stderr,"%s: P25 audio queue overflowed; dropped %" PRIu64 " frames\n",
	      chan->name,audio_queue.dropped - reported_drops);
      reported_drops = audio_queue.dropped;
    }

    output_clock += (uint64_t)count * P25_AUDIO_SAMPRATE;
    while(output_clock >= audio_frame_clock){
      output_clock -= audio_frame_clock;
      if(audio_queue.count == 0){
	chan->output.power = 0;
	send_output(chan,NULL,P25_NATIVE_PCM_SAMPLES,true); // keep RTP timestamps moving
	continue;
      }
      float audio[P25_NATIVE_PCM_SAMPLES];
      int16_t const * const pcm = audio_queue.samples[audio_queue.head];
      float power = 0;
      for(int i = 0; i < P25_NATIVE_PCM_SAMPLES; i++){
	audio[i] = (pcm[i] / 32768.0f) * chan->output.gain;
	power += audio[i] * audio[i];
      }
      chan->output.power = power / P25_NATIVE_PCM_SAMPLES;
      audio_queue.head = (audio_queue.head + 1) % P25_AUDIO_QUEUE_DEPTH;
      audio_queue.count--;
      if(send_output(chan,audio,P25_NATIVE_PCM_SAMPLES,false) < 0){
	restart_needed = true;
	break;
      }
    }
  }
  response(chan); // in case one is pending as we're restarting
  p25_native_destroy(decoder);
  return chan->demod_type == INVALID_DEMOD ? -1 : 0;
}
