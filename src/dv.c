// Digital voice demodulator for radiod: P25 Phase 1, DMR, M17 and YSF
// The decoder library (contrib/digital-voice) takes 48 kHz complex IQ and
// emits 8 kHz speech in bursts (P25: 9 frames per LDU; DMR: 3 per burst;
// M17: 2 per frame; YSF: 5 per frame);
// a small queue paces that out at real time
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "misc.h"
#include "radio.h"
#include "sched.h"
#include "dv.h"

#define DV_AUDIO_QUEUE_DEPTH 45 // 900 ms

struct dv_audio_queue {
  int16_t samples[DV_AUDIO_QUEUE_DEPTH][DV_PCM_SAMPLES];
  unsigned int head;
  unsigned int count;
  uint64_t dropped;
};

static void queue_pcm(void *user_data, int16_t const pcm[DV_PCM_SAMPLES]){
  struct dv_audio_queue * const queue = user_data;
  if(queue->count == DV_AUDIO_QUEUE_DEPTH){
    queue->dropped++;
    return;
  }
  unsigned int const tail = (queue->head + queue->count) % DV_AUDIO_QUEUE_DEPTH;
  memcpy(queue->samples[tail],pcm,sizeof queue->samples[tail]);
  queue->count++;
}

int demod_dv(void *arg){
  chan_t * const chan = arg;
  assert(chan != NULL);
  if(chan == NULL)
    return -1;

  _Static_assert(DV_IQ_SAMPRATE_RADIOD == DV_IQ_SAMPRATE, "digital voice IQ rate mismatch");
  _Static_assert(DV_AUDIO_SAMPRATE == DV_PCM_SAMPRATE, "digital voice audio rate mismatch");
  bool const is_dmr = chan->demod_type == DMR_DEMOD;
  bool const is_m17 = chan->demod_type == M17_DEMOD;
  bool const is_ysf = chan->demod_type == YSF_DEMOD;
  char const * const label = is_dmr ? "DMR" : is_m17 ? "M17" : is_ysf ? "YSF" : "P25";

  // Like WFM, the output rate is fixed by the mode, whatever was set before
  chan->output.samprate = DV_AUDIO_SAMPRATE;
  chan->output.channels = 1;
  {
    int const pt = pt_from_info(chan->output.samprate,chan->output.channels,chan->output.encoding);
    if(pt >= 0)
      chan->output.rtp.type = (uint8_t)pt;
  }
  int const blocksize = (int)lrint(DV_IQ_SAMPRATE_RADIOD * Blocktime);
  if(create_filter_output(&chan->filter.out,&chan->frontend->in,blocksize,COMPLEX) != 0){
    fprintf(stderr,"%s: couldn't create %s channel filter\n",chan->name,label);
    chan->demod_type = INVALID_DEMOD;
    return -1;
  }
  set_channel_filter(chan);

  struct dv_audio_queue audio_queue = {0};
  dv_config const config = {
    .p25_nac = chan->p25.nac,
    .dmr_slot = chan->dmr.slot,
    .m17_can_filter = chan->m17.can >= 0,
    .m17_can = chan->m17.can,
  };
  dv_mode const mode = is_dmr ? DV_DMR : is_m17 ? DV_M17 : is_ysf ? DV_YSF
    : chan->p25.modulation == P25_CQPSK ? DV_P25_CQPSK : DV_P25_C4FM;
  dv_decoder * const decoder = dv_create(mode,&config,queue_pcm,&audio_queue);
  if(decoder == NULL){
    fprintf(stderr,"%s: couldn't create %s decoder\n",chan->name,label);
    chan->demod_type = INVALID_DEMOD;
    return -1;
  }

  // Emit one 20 ms audio frame per 20 ms of IQ, in integer arithmetic
  uint64_t output_clock = 0;
  uint64_t const audio_frame_clock = (uint64_t)DV_IQ_SAMPRATE_RADIOD * DV_PCM_SAMPLES;
  uint64_t reported_drops = 0;
  int telemetry_samples = 0;
  chan->tp1 = 0; // detected sync words
  chan->tp2 = 0; // decoded frames (P25) or bursts (DMR)
  chan->p25.have_nac = false;
  chan->dv.source[0] = '\0';
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
    dv_process(decoder,(float const *)chan->baseband,(size_t)count);

    telemetry_samples += count;
    if(telemetry_samples >= DV_IQ_SAMPRATE_RADIOD){
      telemetry_samples -= DV_IQ_SAMPRATE_RADIOD;
      dv_stats stats;
      dv_get_stats(decoder,&stats);
      chan->tp1 = (double)stats.syncs;
      chan->tp2 = (double)stats.frames;
      chan->sig.foffset = stats.freq_offset;
      if(is_m17 || is_ysf)
	strlcpy(chan->dv.source,stats.src_call,sizeof chan->dv.source);
      if(!is_dmr && !is_m17 && !is_ysf && stats.frames != 0){
	chan->p25.last_nac = stats.last_nac;
	chan->p25.have_nac = true;
      }
      if(Verbose > 0){
	if(is_ysf)
	  fprintf(stderr,"%s YSF samples=%" PRIu64 " syncs=%" PRIu64 " frames=%" PRIu64
		  " voice=%" PRIu64 " muted=%" PRIu64 " src=%s dst=%s offset=%+.1f Hz\n",
		  chan->name,stats.samples,stats.syncs,stats.frames,
		  stats.voice_decoded,stats.voice_muted,
		  stats.src_call[0] ? stats.src_call : "-",stats.dst_call[0] ? stats.dst_call : "-",
		  stats.freq_offset);
	else if(is_m17)
	  fprintf(stderr,"%s M17 samples=%" PRIu64 " syncs=%" PRIu64 " frames=%" PRIu64
		  " voice=%" PRIu64 " muted=%" PRIu64 " src=%s dst=%s can=%d%s"
		  " errors=%.1f%% offset=%+.1f Hz\n",
		  chan->name,stats.samples,stats.syncs,stats.frames,
		  stats.voice_decoded,stats.voice_muted,
		  stats.src_call[0] ? stats.src_call : "-",stats.dst_call[0] ? stats.dst_call : "-",
		  stats.m17_can,stats.m17_encrypted ? " encrypted" : "",
		  100.0f * stats.error_rate,stats.freq_offset);
	else if(is_dmr)
	  fprintf(stderr,"%s DMR samples=%" PRIu64 " symbols=%" PRIu64
		  " syncs=%" PRIu64 " bursts=%" PRIu64
		  " voice=%" PRIu64 " muted=%" PRIu64
		  " slot=%d errors=%.1f%% offset=%+.1f Hz\n",
		  chan->name,stats.samples,stats.symbols,stats.syncs,stats.frames,
		  stats.voice_decoded,stats.voice_muted,
		  stats.active_slot,100.0f * stats.error_rate,stats.freq_offset);
	else
	  fprintf(stderr,"%s P25 samples=%" PRIu64 " symbols=%" PRIu64
		  " syncs=%" PRIu64 " frames=%" PRIu64
		  " LDU1=%" PRIu64 " LDU2=%" PRIu64
		  " voice=%" PRIu64 " encrypted=%" PRIu64
		  " NAC=%03x BCH=%" PRIu32 " offset=%+.1f Hz\n",
		  chan->name,stats.samples,stats.symbols,stats.syncs,stats.frames,
		  stats.ldu1,stats.ldu2,stats.voice_decoded,stats.voice_muted,
		  stats.last_nac,stats.last_bch_errors,stats.freq_offset);
      }
    }
    if(audio_queue.dropped != reported_drops){
      fprintf(stderr,"%s: %s audio queue overflowed; dropped %" PRIu64 " frames\n",
	      chan->name,label,audio_queue.dropped - reported_drops);
      reported_drops = audio_queue.dropped;
    }

    output_clock += (uint64_t)count * DV_AUDIO_SAMPRATE;
    while(output_clock >= audio_frame_clock){
      output_clock -= audio_frame_clock;
      if(audio_queue.count == 0){
	chan->output.power = 0;
	send_output(chan,NULL,DV_PCM_SAMPLES,true); // keep RTP timestamps moving
	continue;
      }
      float audio[DV_PCM_SAMPLES];
      int16_t const * const pcm = audio_queue.samples[audio_queue.head];
      float power = 0;
      for(int i = 0; i < DV_PCM_SAMPLES; i++){
	audio[i] = (pcm[i] / 32768.0f) * chan->output.gain;
	power += audio[i] * audio[i];
      }
      chan->output.power = power / DV_PCM_SAMPLES;
      audio_queue.head = (audio_queue.head + 1) % DV_AUDIO_QUEUE_DEPTH;
      audio_queue.count--;
      if(send_output(chan,audio,DV_PCM_SAMPLES,false) < 0){
	restart_needed = true;
	break;
      }
    }
  }
  response(chan); // in case one is pending as we're restarting
  dv_destroy(decoder);
  return chan->demod_type == INVALID_DEMOD ? -1 : 0;
}
