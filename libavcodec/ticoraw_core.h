/*
 * TICO RAW (Nikon N-RAW, R3D NE) core decoder: codestream to companded CFA planes
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

#ifndef AVCODEC_TICORAW_CORE_H
#define AVCODEC_TICORAW_CORE_H

#include <stddef.h>
#include <stdint.h>

#include "ticoraw_header.h"

/** Four int32 planes of cw x ch samples, in component order. stride is in elements. */
typedef struct TicoRawPlanes {
    int32_t *p[4];
    int cw, ch;
    ptrdiff_t stride;
} TicoRawPlanes;

/** The stages of the format note 10.2; decoding stops after the given one. */
enum TicoRawStage {
    TICORAW_STAGE_COEFFS,       /* dequantized coefficients, canonical layout (10.4) */
    TICORAW_STAGE_PLANES_ST,    /* after the inverse wavelet (10.5) */
    TICORAW_STAGE_CFA,          /* after the inverse Star-Tetrix: cfa_companded (10.6) */
};

struct AVCodecContext;

typedef struct TicoRawCore TicoRawCore;   /* opaque: buffers, per-thread scratch, DSP */

/**
 * Make a core for the codec context avctx. The core runs its jobs through
 * avctx->execute2(); with slice threads it keeps one scratch set per slice thread.
 * avctx is also the log context. One core per codec context.
 */
int  ff_ticoraw_core_init(TicoRawCore **core, const TicoRawHeader *h,
                          struct AVCodecContext *avctx);
void ff_ticoraw_core_free(TicoRawCore **core);

/**
 * Decode one codestream up to the given stage.
 *
 * The slices are entropy decoded in parallel, and the wavelet and Star-Tetrix passes are
 * split into row or column jobs, all through avctx->execute2(). The output does not
 * depend on the number of threads.
 *
 * The planes belong to core and stay valid until the next call. Calls on one core must
 * not overlap. cs must be followed by AV_INPUT_BUFFER_PADDING_SIZE readable bytes (as
 * in an AVPacket). h must be the result of ff_ticoraw_parse_header() on the same
 * codestream; the core keeps its buffers and band layout while the geometry of h
 * (sizes, levels, slices) stays the same, and rebuilds them when it changes.
 *
 * @return 0, AVERROR_INVALIDDATA for damaged data, AVERROR_PATCHWELCOME for an
 *         unsupported structure, or AVERROR(ENOMEM)
 */
int ff_ticoraw_decode_stage(TicoRawCore *core, const uint8_t *cs, int size,
                            const TicoRawHeader *h, enum TicoRawStage stop,
                            TicoRawPlanes *out);

/**
 * Decodes one codestream to the four companded CFA planes (stage cfa_companded).
 * Same as ff_ticoraw_decode_stage() with TICORAW_STAGE_CFA.
 */
int ff_ticoraw_decode_planes(TicoRawCore *core, const uint8_t *cs, int size,
                             const TicoRawHeader *h, TicoRawPlanes *out);

/**
 * Number of jobs to split rows rows over in avctx->execute2(): 1 without slice threads,
 * else a few jobs per thread, so that faster cores take more of them.
 */
int ff_ticoraw_nb_row_jobs(const struct AVCodecContext *avctx, int rows);

#define TICORAW_CURVE_SIZE  65536  /* entries of the transfer curve table: one per 16-bit code */
#define TICORAW_WHITE_LEVEL 16383  /* ceiling of the transfer curve, 2^14 - 1 (note 9.3) */

/**
 * Build the transfer curve table Lin(i) of the format note 9.2 from the curve
 * parameters of h (black_level, curve_slope, curve_pivot).
 *
 * @param lut  TICORAW_CURVE_SIZE entries, filled with linear values 0 to
 *             TICORAW_WHITE_LEVEL
 */
void ff_ticoraw_build_curve(uint16_t *lut, const TicoRawHeader *h);

/**
 * Write quad rows y0 to y1 - 1 of the companded CFA planes (stage cfa_companded) as
 * linear RGGB mosaic rows 2 * y0 to 2 * y1 - 1 (format note 10.7).
 *
 * @param dst     mosaic row 2 * y0; dst_stride is in uint16_t elements, not bytes
 * @param planes  planes Gr, R, Gb, B as returned by ff_ticoraw_decode_planes()
 * @param lut     table from ff_ticoraw_build_curve()
 */
void ff_ticoraw_planes_to_bayer(uint16_t *dst, ptrdiff_t dst_stride,
                                const TicoRawPlanes *planes, const uint16_t *lut,
                                int y0, int y1);

#endif /* AVCODEC_TICORAW_CORE_H */
