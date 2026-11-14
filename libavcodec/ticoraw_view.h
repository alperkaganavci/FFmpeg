/*
 * TICO RAW (Nikon N-RAW, R3D NE) Rec.709 viewing picture
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

#ifndef AVCODEC_TICORAW_VIEW_H
#define AVCODEC_TICORAW_VIEW_H

#include <stdint.h>

#include "libavutil/frame.h"

#include "avcodec.h"
#include "ticoraw_core.h"

enum TicoRawViewSize {
    TICORAW_VIEW_HALF,  /* one RGB pixel per 2x2 quad: cw x ch */
    TICORAW_VIEW_FULL,  /* full demosaic: width x height */
};

/** One axis of a bilinear sample of the full view (12.3). */
typedef struct TicoRawViewTap {
    int i0, i1;     /* the two quad positions */
    float f;        /* weight of i1 */
} TicoRawViewTap;

/**
 * Tables of ff_ticoraw_render_view(), kept between frames by the caller. Start from all
 * zeros; free with ff_ticoraw_view_free(). One per codec context: the frames of one
 * context must not render at the same time.
 */
typedef struct TicoRawViewContext {
    float *norm;            /* 12.2: N(Lin(i)) for every code index i, rebuilt each frame */
    TicoRawViewTap *taps;   /* full view: per-column taps of the phases 0 and 1 */
    int nb_taps;            /* allocated entries of taps */
} TicoRawViewContext;

typedef struct TicoRawViewParams {
    int black_level, white_level;   /* linear sensor values of black and of white */
    double wb_gain[3];              /* recorded R, G, B white-balance gains (G is not used) */
    double cam_to_xyz[9];           /* white-balanced camera RGB to CIE XYZ (D65), row-major */
    double exposure;                /* EV offset on top of the reference brightness */
    enum TicoRawViewSize size;
    int nraw;                       /* 1 for an N-RAW clip, 0 for an R3D NE clip */
} TicoRawViewParams;

/**
 * Writes the GBRP16 view of the decoded planes into frame (planes 0,1,2 = G,B,R).
 *
 * Speed: N(Lin(i)) comes from a table of every code index, the full view's column taps
 * are computed once per frame, and the tone map, shoulder and power of 12.7 come from a
 * table indexed by the float bits of each channel with linear interpolation; its codes
 * are within 1 of the formula of 12.7 and 12.8 step 1 for every float input.
 *
 * The picture follows the viewing transform of the reference renderer, format note
 * section 12, at the view's own size: normalisation (12.2), quad sampling or the full
 * demosaic (12.3), white balance and clip (12.4), k * A * cam_to_xyz (12.5), tone map,
 * shoulder and the power 1 / 2.4 (12.7), 16-bit quantisation (12.8 step 1). Rows are split
 * across avctx->execute2() jobs; the output does not depend on the number of threads.
 *
 * @param vc     tables kept between frames, see TicoRawViewContext
 * @param cfa    planes Gr, R, Gb, B (stage cfa_companded) from ff_ticoraw_decode_planes()
 * @param curve  table from ff_ticoraw_build_curve()
 * @param frame  allocated AV_PIX_FMT_GBRP16 frame of cfa->cw x cfa->ch (half) or
 *               2 * cfa->cw x 2 * cfa->ch (full)
 * @return 0, AVERROR_INVALIDDATA if white_level is not above black_level, or
 *         AVERROR(ENOMEM)
 */
int ff_ticoraw_render_view(TicoRawViewContext *vc, const TicoRawPlanes *cfa,
                           const uint16_t *curve, const TicoRawViewParams *p,
                           AVFrame *frame, AVCodecContext *avctx);

/** Free the tables of vc and reset it to all zeros. */
void ff_ticoraw_view_free(TicoRawViewContext *vc);

/**
 * The colour steps of ff_ticoraw_render_view() (format note 12.4 to 12.8 step 1) for one
 * pixel, for tests.
 *
 * @param cam  normalised camera RGB (12.2, 12.3)
 * @param rgb  receives the 16-bit R, G, B codes
 */
void ff_ticoraw_view_pixel(const TicoRawViewParams *p, const float cam[3], uint16_t rgb[3]);

#endif /* AVCODEC_TICORAW_VIEW_H */
