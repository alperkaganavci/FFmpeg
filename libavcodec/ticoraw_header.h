/*
 * TICO RAW (Nikon N-RAW, R3D NE) codestream header parser
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVCODEC_TICORAW_HEADER_H
#define AVCODEC_TICORAW_HEADER_H

#include <stdint.h>

#include "ticoraw_nraw.h"

#define TICORAW_NB_COMPONENTS 4  /* Nc of the supported profile */
#define TICORAW_NB_BANDS      25 /* bands per precinct for NLx = 5, NLy = 1, see the format note 5.3 */
#define TICORAW_VENDOR_BYTES  13 /* PIH vendor bytes V0 ... V12 */

/**
 * Parsed codestream header: PIH, CDT and WGT, plus the position of the first slice.
 *
 * The first block holds the raw PIH fields (names as in the format note, section 3.2).
 * Sizes are in samples, lengths in bytes, offsets in bytes from the first byte (SOC) of
 * the codestream.
 */
typedef struct TicoRawHeader {
    /* PIH */
    int width, height;          /* Wf, Hf: full mosaic size in samples (both even) */
    int nc;                     /* Nc: number of components */
    int ng;                     /* Ng: coefficients per code group */
    int ss;                     /* Ss: code groups per significance group */
    int bw;                     /* Bw: nominal coefficient bit depth */
    int fq;                     /* Fq: fractional bits; dequantized values are scaled by 2^fq */
    int br;                     /* Br: bits per raw GCLI */
    int cpih;                   /* Cpih: colour transform field (0 for Star-Tetrix streams) */
    int nlx, nly;               /* NLx, NLy: global decomposition levels */
    int slice_height;           /* Hsl: precincts per slice */
    int lcod;                   /* Lcod: codestream length, SOC to EOC inclusive */

    /* CDT: one entry per component, in component order */
    uint8_t comp_depth[TICORAW_NB_COMPONENTS]; /* bit depth B[c] */
    uint8_t comp_sx[TICORAW_NB_COMPONENTS];    /* horizontal sampling factor Sx */
    uint8_t comp_sy[TICORAW_NB_COMPONENTS];    /* vertical sampling factor Sy */
    uint8_t comp_nlx[TICORAW_NB_COMPONENTS];   /* own horizontal decomposition levels (0 = no wavelet) */
    uint8_t comp_nly[TICORAW_NB_COMPONENTS];   /* own vertical decomposition levels */

    /* WGT: one entry per band, in the band order of the format note 5.3 */
    int nb_bands;                              /* always TICORAW_NB_BANDS */
    uint8_t band_gain[TICORAW_NB_BANDS];       /* G[b]: gain, lowers the GTLI */
    uint8_t band_priority[TICORAW_NB_BANDS];   /* P[b]: priority, compared with R */

    /* Transfer curve, from the PIH vendor bytes (format note 9.1) */
    uint8_t vendor[TICORAW_VENDOR_BYTES];      /* V0 ... V12, unmodified */
    int black_level;            /* Bk: 12 bit, also the curve output at the pivot code */
    int curve_slope;            /* S: 18 bit, 16.16 fixed point slope above the pivot */
    int curve_pivot;            /* K: 16 bit code index where the curve reaches Bk */

    /* Geometry derived from the fields above */
    int cw, ch;                 /* component size: width / 2, height / 2 */
    int nb_precincts;           /* ceil(ch / 2): precincts, each with two lines per component */
    int nb_slices;              /* ceil(nb_precincts / slice_height) */
    int first_slice_offset;     /* offset of the first SLH marker */
} TicoRawHeader;

/**
 * Parse the markers of a TICO RAW codestream up to the first slice header.
 *
 * @param cs      codestream, starting at SOC
 * @param size    codestream size in bytes (Lcod must equal it, and EOC must end it)
 * @param h       filled on success
 * @param logctx  context for av_log, may be NULL
 * @return 0 on success, AVERROR_INVALIDDATA for damaged or short data,
 *         AVERROR_PATCHWELCOME for a valid stream outside the supported set. The error
 *         message names the field and its value.
 */
int ff_ticoraw_parse_header(const uint8_t *cs, int size, TicoRawHeader *h, void *logctx);

/**
 * Parse the Nikon wrapper and the codestream header of one video sample, and check that
 * the two agree on the picture size.
 *
 * @param cs_offset  receives the offset of the codestream inside buf
 * @param cs_size    receives the codestream size in bytes
 * @return 0 on success, or the error of the wrapper or header parser
 */
int ff_ticoraw_parse_frame(const uint8_t *buf, int size, TicoRawMeta *meta,
                           TicoRawHeader *h, int *cs_offset, int *cs_size, void *logctx);

#endif /* AVCODEC_TICORAW_HEADER_H */
