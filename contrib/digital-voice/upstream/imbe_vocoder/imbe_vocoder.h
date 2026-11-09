/*
 * Project 25 IMBE Encoder/Decoder Fixed-Point implementation
 * Developed by Pavel Yazev E-mail: pyazev@gmail.com
 * Version 1.0 (c) Copyright 2009
 */
/* -*- c++ -*- */
#ifndef INCLUDED_IMBE_VOCODER_H
#define INCLUDED_IMBE_VOCODER_H

#include <stdint.h>
#include <string.h>

#include "imbe.h"
#include "dsp_sub.h"
#include "basic_op.h"
#include "math_sub.h"
#include "encode.h"
#include "decode.h"

class imbe_vocoder
{
public:
	imbe_vocoder(void);	// constructor
	~imbe_vocoder() {}	// destructor
	// imbe_decode decodes IMBE codewords (frame_vector),
	// outputs the resulting 160 audio samples (snd)
	void imbe_decode(int16_t *frame_vector, int16_t *snd) {
		decode(&my_imbe_param, frame_vector, snd);
	}
private:
	IMBE_PARAM my_imbe_param;

	/* data items originally static (moved from individual c++ sources) */
	UWord32 seed ;
	Word16 num_harms_prev1;
	Word32 sa_prev1[NUM_HARMS_MAX + 2];
	Word16 uv_mem[105];
	UWord32 ph_mem[NUM_HARMS_MAX];
	Word16 num_harms_prev3;
	Word32 fund_freq_prev;
	Word16 vu_dsn_prev[NUM_HARMS_MAX];
	Word16 sa_prev3[NUM_HARMS_MAX];
	Word16 v_uv_dsn[NUM_BANDS_MAX];
	Word16 wr_array[FFTLENGTH / 2 + 1];
	Word16 wi_array[FFTLENGTH / 2 + 1];

	/* member functions */
	void idct(Word16 *in, Word16 m_lim, Word16 i_lim, Word16 *out);
	void fft_init(void);
	void fft(Word16 *datam1, Word16 nn, Word16 isign);
	void sa_decode_init(void);
	void sa_decode(IMBE_PARAM *imbe_param);
	void uv_synt_init(void);
	void uv_synt(IMBE_PARAM *imbe_param, Word16 *snd);
	void v_synt_init(void);
	void v_synt(IMBE_PARAM *imbe_param, Word16 *snd);
	void decode_init(IMBE_PARAM *imbe_param);
	void decode(IMBE_PARAM *imbe_param, Word16 *frame_vector, Word16 *snd);
};

#endif /* INCLUDED_IMBE_VOCODER_H */
