// Native P25 Phase 1 demodulator for radiod
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "radio.h"
#include "sched.h"
#include "p25_native_api.h"

#define P25_AUDIO_QUEUE_DEPTH 45

struct p25_audio_queue {
  int16_t samples[P25_AUDIO_QUEUE_DEPTH][P25_IMBE_PCM_SAMPLES];
  unsigned int head;
  unsigned int count;
  uint64_t dropped;
};

static int p25_pcm_callback(void *user_data, uint16_t nac, uint8_t duid,
                            uint32_t bch_errors, uint32_t imbe_errors,
                            int16_t const pcm[P25_IMBE_PCM_SAMPLES]){
  struct p25_audio_queue *queue = user_data;
  (void)nac;
  (void)duid;
  (void)bch_errors;
  (void)imbe_errors;
  if(queue->count == P25_AUDIO_QUEUE_DEPTH){
    queue->dropped++;
    return 0;
  }
  unsigned int const tail = (queue->head + queue->count) % P25_AUDIO_QUEUE_DEPTH;
  memcpy(queue->samples[tail],pcm,sizeof queue->samples[tail]);
  queue->count++;
  return 0;
}

int demod_p25(void *arg){
  chan_t * const chan = arg;
  assert(chan != NULL);
  if(chan == NULL)
    return -1;

  int const iq_samprate = channel_input_samprate(chan);
  if(iq_samprate != P25_IQ_SAMPRATE ||
     chan->output.samprate != P25_AUDIO_SAMPRATE ||
     chan->output.channels != 1){
    fprintf(stderr,"%s: native P25 requires %d Hz IQ and mono %d Hz audio\n",
            chan->name,P25_IQ_SAMPRATE,P25_AUDIO_SAMPRATE);
    chan->demod_type = INVALID_DEMOD;
    return -1;
  }

  int const blocksize = (int)lrint(iq_samprate * Blocktime);
  if(blocksize <= 0 ||
     create_filter_output(&chan->filter.out,&chan->frontend->in,
                          blocksize,COMPLEX) != 0){
    fprintf(stderr,"%s: couldn't create P25 channel filter\n",chan->name);
    chan->demod_type = INVALID_DEMOD;
    return -1;
  }
  set_channel_filter(chan);

  struct p25_audio_queue audio_queue = {0};
  p25_native_decoder *decoder = NULL;
  p25_native_modulation const modulation =
      chan->p25.modulation == P25_CQPSK ?
      P25_NATIVE_CQPSK : P25_NATIVE_C4FM;
  if(p25_native_decoder_create(modulation,p25_pcm_callback,&audio_queue,
                               &decoder) != P25_PHASE1_OK){
    fprintf(stderr,"%s: couldn't create native P25 decoder\n",chan->name);
    chan->demod_type = INVALID_DEMOD;
    return -1;
  }
  if(p25_native_decoder_set_expected_nac(decoder,chan->p25.nac) !=
     P25_PHASE1_OK){
    fprintf(stderr,"%s: couldn't set P25 NAC filter\n",chan->name);
    p25_native_decoder_destroy(decoder);
    chan->demod_type = INVALID_DEMOD;
    return -1;
  }

  uint64_t output_clock = 0;
  uint64_t const audio_frame_clock =
      (uint64_t)iq_samprate * P25_IMBE_PCM_SAMPLES;
  uint64_t reported_drops = 0;
  uint64_t telemetry_samples = 0;
  float iq_components[2 * 512];
  chan->tp1 = 0; // detected P25 sync words
  chan->tp2 = 0; // decoded P25 frames
  bool restart_needed = false;
  realtime(chan->prio);

  while(!restart_needed){
    restart_needed = handle_commands(chan);
    int r;
    if(restart_needed || (r = downconvert(chan)) == -1)
      break;
    else if(r == 1)
      continue;

    float complex const * const input = chan->baseband;
    int const count = chan->sampcount;
    int const process_blocksize = 512;
    for(int offset = 0; offset < count; offset += process_blocksize){
      int const block_count = count - offset < process_blocksize ?
          count - offset : process_blocksize;
      for(int i = 0; i < block_count; i++){
        iq_components[2 * i] = crealf(input[offset + i]);
        iq_components[2 * i + 1] = cimagf(input[offset + i]);
      }
      int const decoder_result =
          p25_native_decoder_process_f32(decoder,iq_components,block_count);
      if(decoder_result != P25_PHASE1_OK){
        fprintf(stderr,"%s: native P25 decoder failed (%d)\n",
                chan->name,decoder_result);
        chan->demod_type = INVALID_DEMOD;
        restart_needed = false;
        break;
      }
    }
    if(chan->demod_type == INVALID_DEMOD)
      break;

    telemetry_samples += (uint64_t)count;
    if(telemetry_samples >= (uint64_t)iq_samprate){
      telemetry_samples %= (uint64_t)iq_samprate;
      p25_phase1_stats stats;
      uint64_t sync_count;
      uint64_t complex_samples;
      uint64_t symbols;
      float frequency_offset;
      if(p25_native_decoder_get_stats(decoder,&complex_samples,&symbols,&stats) !=
             P25_PHASE1_OK ||
         p25_native_decoder_get_sync_count(decoder,&sync_count) != P25_PHASE1_OK ||
         p25_native_decoder_get_frequency_offset(decoder,&frequency_offset) !=
             P25_PHASE1_OK){
        fprintf(stderr,"%s: couldn't read P25 decoder statistics\n",chan->name);
        chan->demod_type = INVALID_DEMOD;
        break;
      }
      chan->tp1 = (double)sync_count;
      chan->tp2 = (double)stats.complete_frames;
      chan->sig.foffset = frequency_offset;
      if(Verbose > 0)
        fprintf(stderr,"%s P25 samples=%" PRIu64 " symbols=%" PRIu64
                " syncs=%" PRIu64 " frames=%" PRIu64
                " LDU1=%" PRIu64 " LDU2=%" PRIu64
                " voice=%" PRIu64 " skipped-unknown=%" PRIu64
                " skipped-encrypted=%" PRIu64 " NAC=%03x BCH=%u"
                " offset=%+.1f Hz\n",
                chan->name,complex_samples,symbols,sync_count,
                stats.complete_frames,stats.ldu1_frames,stats.ldu2_frames,
                stats.voice_codewords_decoded,
                stats.voice_codewords_skipped_unknown_encryption,
                stats.voice_codewords_skipped_encrypted,
                stats.last_nac,stats.last_bch_errors,frequency_offset);
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
        send_output(chan,NULL,P25_IMBE_PCM_SAMPLES,true);
        continue;
      }

      float audio[P25_IMBE_PCM_SAMPLES];
      int16_t const * const pcm = audio_queue.samples[audio_queue.head];
      float power = 0;
      for(size_t i = 0; i < P25_IMBE_PCM_SAMPLES; i++){
        audio[i] = (pcm[i] / 32768.0f) * chan->output.gain;
        power += audio[i] * audio[i];
      }
      chan->output.power = power / P25_IMBE_PCM_SAMPLES;
      audio_queue.head = (audio_queue.head + 1) % P25_AUDIO_QUEUE_DEPTH;
      audio_queue.count--;
      if(send_output(chan,audio,P25_IMBE_PCM_SAMPLES,false) < 0){
        restart_needed = true;
        break;
      }
    }
    response(chan);
  }

  p25_native_decoder_destroy(decoder);
  return chan->demod_type == INVALID_DEMOD ? -1 : 0;
}
