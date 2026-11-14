/*
 * TICO RAW (Nikon N-RAW, R3D NE) precinct entropy decoding and dequantization
 *
 * Copyright (c) 2026 Alper Kagan AVCI
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

#ifndef AVCODEC_TICORAW_ENTROPY_H
#define AVCODEC_TICORAW_ENTROPY_H

#include <stddef.h>
#include <stdint.h>

#include "ticoraw_header.h"

#define TICORAW_NB_LINES          2  /* lines per precinct and component (NLy = 1) */
#define TICORAW_NB_PACKETS        (TICORAW_NB_LINES * TICORAW_NB_COMPONENTS)
#define TICORAW_MAX_PACKET_BANDS  16 /* bands of one component on one line: n + 1 <= 16 */

/** One band of the precinct layout. */
typedef struct TicoRawBand {
    int comp;           /* component */
    int start;          /* first column in the wavelet layout */
    int width;          /* coefficients */
    int nb_groups;      /* code groups: ceil(width / Ng) */
    int nb_sig_groups;  /* significance groups: ceil(nb_groups / Ss) */
} TicoRawBand;

/** One band of one packet, with the precinct line it is coded for. */
typedef struct TicoRawPacketBand {
    uint8_t band;
    uint8_t line;
} TicoRawPacketBand;

/** Band and packet structure of every precinct, derived from the header once. */
typedef struct TicoRawLayout {
    int nb_bands;
    TicoRawBand band[TICORAW_NB_BANDS];
    int nb_packets;
    int packet_nb_bands[TICORAW_NB_PACKETS];
    TicoRawPacketBand packet_band[TICORAW_NB_PACKETS][TICORAW_MAX_PACKET_BANDS];
    int precinct_header_size;   /* 5 + ceil(2 * nb_bands / 8) bytes */
    int total_groups;           /* code groups of all bands */
} TicoRawLayout;

/**
 * Per-slice decoding state: the prediction source of every band index.
 * Slices are independent, so each thread that decodes slices needs its own context.
 */
typedef struct TicoRawSliceContext {
    uint8_t *gcli[TICORAW_NB_BANDS];    /* GCLIs of the last decoded line of band b */
    uint8_t gtli[TICORAW_NB_BANDS];     /* GTLI of that line */
    uint8_t have_src[TICORAW_NB_BANDS]; /* 0 until band b was decoded in this slice */
    uint8_t *gcli_buf;
} TicoRawSliceContext;

/** Start and end offsets of the precinct data of one slice (SLH excluded). */
typedef struct TicoRawSliceSpan {
    int start, end;
} TicoRawSliceSpan;

/**
 * Derive the bands and packets of a precinct from the header.
 * @return 0, or AVERROR_PATCHWELCOME when the structure is outside the supported set
 */
int ff_ticoraw_layout_init(TicoRawLayout *l, const TicoRawHeader *h, void *logctx);

int  ff_ticoraw_slice_context_init(TicoRawSliceContext *sc, const TicoRawLayout *l);
void ff_ticoraw_slice_context_free(TicoRawSliceContext *sc);

/**
 * Walk the slice headers and precinct lengths of the codestream and record
 * where the precinct data of each slice starts and ends.
 *
 * @param spans  h->nb_slices entries
 * @return 0 or AVERROR_INVALIDDATA
 */
int ff_ticoraw_find_slices(const uint8_t *cs, int size, const TicoRawHeader *h,
                           const TicoRawLayout *l, TicoRawSliceSpan *spans, void *logctx);

/**
 * Entropy decode and dequantize the precincts of one slice into the four coefficient
 * planes. Each plane holds the bands of its component side by side in every row, the
 * low-pass band first (the layout that the inverse wavelet starts from).
 *
 * cs must be followed by AV_INPUT_BUFFER_PADDING_SIZE readable bytes. Every read stays
 * inside the slice span plus that padding.
 *
 * @param slice   slice index
 * @param planes  four planes of h->cw x h->ch int32 samples
 * @param stride  plane stride in int32_t elements
 * @return 0 or AVERROR_INVALIDDATA
 */
int ff_ticoraw_decode_slice(TicoRawSliceContext *sc, const TicoRawLayout *l,
                            const TicoRawHeader *h, const uint8_t *cs,
                            const TicoRawSliceSpan *span, int slice,
                            int32_t *const planes[4], ptrdiff_t stride, void *logctx);

#endif /* AVCODEC_TICORAW_ENTROPY_H */
