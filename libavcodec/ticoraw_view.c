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

/*
 * The viewing transform of the reference renderer (format note section 12), per output
 * pixel, in single precision floating point like the reference:
 *  - camera RGB: half size, one pixel per quad, R, the mean of the two normalised greens
 *    and B (12.3 quad sampling at ow = cw); full size, the full demosaic of 12.3 on the
 *    raw values; then N(s) = max((s - black) / (white - black), 0) (12.2);
 *  - white balance with the recorded gains (G gain 1), each channel clamped to 0..1 (12.4);
 *  - lin = k * A * C * w, with k = 11.52 * F * 2^exposure, F = 2^-0.5 for R3D NE and 1 for
 *    N-RAW (12.5); k, A and C are applied as one 3x3 matrix;
 *  - per channel: tone map, shoulder, clamp and the power 1 / 2.4 (12.7);
 *  - Q = floor(V * 65535 + 0.5) (12.8 step 1).
 * For speed, N(Lin(i)) is a table of all 65536 code indices (the same float operations,
 * so the same values), and 12.7 with 12.8 step 1 is a table of V * 65535 at 128 points
 * per octave of the input, read with linear interpolation (within 1 code of the formula,
 * see encode_table_init()).
 * The resize to a requested width, the P010 step and the HEVC step of the reference are
 * not part of the view.
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "libavutil/intfloat.h"
#include "libavutil/log.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/thread.h"

#include "avcodec.h"
#include "ticoraw_core.h"
#include "ticoraw_view.h"

/* A: CIE XYZ (D65) to linear Rec.709, the constants of the format note 12.5. */
static const double xyz_to_rec709[9] = {
     3.24097,  -1.537383, -0.498611,
    -0.969244,  1.875968,  0.041555,
     0.05563,  -0.203977,  1.056972,
};

/* The matrix used when the recorded cam_to_xyz is rejected (12.1). */
static const double default_cam_to_xyz[9] = {
    0.605607,   0.38127,  -0.0364214,
    0.228239,   1.04834,  -0.276578,
    0.0062585, -0.173072,  1.25587,
};

/*
 * The table of 12.7 and 12.8 step 1: entry i holds V * 65535 (before rounding, without
 * the clamp of y to 1) for the float input whose bits are ENC_BITS_LO + (i << ENC_SHIFT),
 * that is 2^ENC_SEG_BITS points per octave from 2^ENC_EXP_LO up to 2. Inputs below
 * 2^ENC_EXP_LO give code 0 (the formula gives 0 below about 2^-36) and inputs of 2 and
 * more give 65535 (y reaches 1 at about 1.5).
 */
#define ENC_EXP_LO   (-40)
#define ENC_SEG_BITS 7
#define ENC_SHIFT    (23 - ENC_SEG_BITS)
#define ENC_SIZE     ((1 - ENC_EXP_LO) << ENC_SEG_BITS)
#define ENC_BITS_LO  ((uint32_t)(127 + ENC_EXP_LO) << 23)

static float encode_table[ENC_SIZE + 1];
static AVOnce encode_table_once = AV_ONCE_INIT;

/*
 * Fill encode_table from the formula in the comment of encode(), in double precision.
 * Checked against that formula in single precision (the code of the view before the
 * table) for every float from 0 to 4, for larger floats, infinities and NaN: no code
 * differs by more than 1.
 */
static av_cold void encode_table_init(void)
{
    int i;

    for (i = 0; i <= ENC_SIZE; i++) {
        const double x = av_int2float(ENC_BITS_LO + ((uint32_t)i << ENC_SHIFT));
        const double t = 1.1 * 0.18 * pow(x / 0.18, 1.16);
        const double y = t * (1.0 + t / 16.0) / (1.0 + t) * 1.25;

        encode_table[i] = pow(y, 1.0 / 2.4) * 65535.0;
    }
}

typedef struct ViewJobs {
    const TicoRawPlanes *cfa;
    const uint16_t *curve;
    const float *norm;      /* 12.2: N(Lin(i)) for every code index i */
    const TicoRawViewTap *tx[2]; /* full view: column taps of the phases 0 and 1 */
    float black, range;     /* 12.2: N(s) = max((s - black) / range, 0) */
    float wb[3];            /* 12.4: R, G, B gains */
    float m[9];             /* 12.5: k * A * C */
    uint16_t *dst[3];       /* output planes G, B, R */
    ptrdiff_t stride[3];    /* in uint16_t elements */
    int rows, nb_jobs;
} ViewJobs;

static int finite_in(double v, double lo, double hi)
{
    return isfinite(v) && v >= lo && v < hi;
}

/* Gains, matrix and exposure gain of 12.1, 12.4 and 12.5. */
static void setup_colour(ViewJobs *j, const TicoRawViewParams *p)
{
    const double k = 11.52 * (p->nraw ? 1.0 : M_SQRT1_2) * exp2(p->exposure);
    const double *c = p->cam_to_xyz;
    double gr = p->wb_gain[0], gb = p->wb_gain[2];
    int r, col, i;

    /* 12.1: the gains are used only if both lie in [0.2, 8) */
    if (!finite_in(gr, 0.2, 8.0) || !finite_in(gb, 0.2, 8.0)) {
        gr = 1.7;
        gb = 1.5;
    }
    j->wb[0] = av_clipd(gr,  0.1, 16.0);    /* 12.4: each gain clamped to [0.1, 16] */
    j->wb[1] = 1.0f;
    j->wb[2] = av_clipd(gb,  0.1, 16.0);

    /* 12.1: the matrix is used only if it is finite and its Y row sums to 1 within 0.05 */
    for (i = 0; i < 9; i++)
        if (!isfinite(c[i]))
            break;
    if (i < 9 || !(fabs(c[3] + c[4] + c[5] - 1.0) < 0.05))
        c = default_cam_to_xyz;

    for (r = 0; r < 3; r++) {
        for (col = 0; col < 3; col++) {
            double v = 0.0;
            for (i = 0; i < 3; i++)
                v += xyz_to_rec709[3 * r + i] * c[3 * i + col];
            j->m[3 * r + col] = k * v;
        }
    }
}

/* Lin(code) of one cfa_companded sample: code index i = clip(v + 32768) (note 9.3). */
static av_always_inline float raw(const ViewJobs *j, int32_t v)
{
    return j->curve[av_clip_uint16(v + 32768)];
}

/* 12.2 */
static av_always_inline float normalise(const ViewJobs *j, float s)
{
    return FFMAX((s - j->black) / j->range, 0.0f);
}

/* N(Lin(code)) of one cfa_companded sample, from the table. */
static av_always_inline float norm_raw(const ViewJobs *j, int32_t v)
{
    return j->norm[av_clip_uint16(v + 32768)];
}

/*
 * 12.7 and 12.8 step 1 for one channel of lin, from encode_table. The formula it
 * follows (the code of the view before the table) is
 *   t = 1.1 * 0.18 * (x / 0.18)^1.16                  tone map
 *   y = t * (1 + t / 16) / (1 + t) * 1.25             shoulder, L = 4
 *   Q = y < 1 ? floor(y^(1 / 2.4) * 65535 + 0.5) : 65535
 * for x > 0, and Q = 0 otherwise. A NaN gives 0, never UB.
 */
static av_always_inline uint16_t encode(float x)
{
    uint32_t b;
    float f, v;
    int i;

    if (!(x >= av_int2float(ENC_BITS_LO)))
        return 0;
    if (!(x < 2.0f))
        return 65535;
    b = av_float2int(x) - ENC_BITS_LO;
    i = b >> ENC_SHIFT;
    f = (b & ((1 << ENC_SHIFT) - 1)) * (1.0f / (1 << ENC_SHIFT));
    v = encode_table[i] + (encode_table[i + 1] - encode_table[i]) * f;
    return v < 65534.5f ? (uint16_t)(v + 0.5f) : 65535;
}

/* 12.4 to 12.8 step 1 for one pixel of normalised camera RGB. */
static av_always_inline void put_pixel(const ViewJobs *j, float r, float g, float b,
                                       uint16_t *dg, uint16_t *db, uint16_t *dr)
{
    const float *m = j->m;

    r = av_clipf(r * j->wb[0], 0.0f, 1.0f);
    g = av_clipf(g * j->wb[1], 0.0f, 1.0f);
    b = av_clipf(b * j->wb[2], 0.0f, 1.0f);
    *dr = encode(m[0] * r + m[1] * g + m[2] * b);
    *dg = encode(m[3] * r + m[4] * g + m[5] * b);
    *db = encode(m[6] * r + m[7] * g + m[8] * b);
}

/*
 * Half size, job i: quad rows y0 to y1 - 1. At ow = cw the quad sampling of 12.3 reads
 * exactly one quad per output pixel: R, the mean of the two normalised greens, B.
 */
static int half_job(AVCodecContext *avctx, void *arg, int job, int threadnr)
{
    const ViewJobs *j = arg;
    const TicoRawPlanes *c = j->cfa;
    const int y0 = (int64_t)j->rows * job       / j->nb_jobs;
    const int y1 = (int64_t)j->rows * (job + 1) / j->nb_jobs;
    int x, y;

    for (y = y0; y < y1; y++) {
        const int32_t *gr = c->p[0] + y * c->stride;
        const int32_t *r  = c->p[1] + y * c->stride;
        const int32_t *gb = c->p[2] + y * c->stride;
        const int32_t *b  = c->p[3] + y * c->stride;
        uint16_t *dg = j->dst[0] + y * j->stride[0];
        uint16_t *db = j->dst[1] + y * j->stride[1];
        uint16_t *dr = j->dst[2] + y * j->stride[2];

        for (x = 0; x < c->cw; x++)
            put_pixel(j, norm_raw(j, r[x]),
                      0.5f * (norm_raw(j, gr[x]) + norm_raw(j, gb[x])),
                      norm_raw(j, b[x]), dg + x, db + x, dr + x);
    }
    return 0;
}

typedef TicoRawViewTap Tap;

/* One axis of a bilinear sample (12.3) at quad coordinate (pos - phase) / 2 of n samples. */
static av_always_inline Tap get_tap(int pos, int phase, int n)
{
    const float u = av_clipf((pos - phase) * 0.5f, 0.0f, n - 1);
    Tap t;

    t.i0 = (int)u;
    t.i1 = FFMIN(t.i0 + 1, n - 1);
    t.f  = u - t.i0;
    return t;
}

/* Bilinear sample of the raw values of one plane (12.3). */
static av_always_inline float sample(const ViewJobs *j, const int32_t *plane, Tap tx, Tap ty)
{
    const int32_t *r0 = plane + ty.i0 * j->cfa->stride;
    const int32_t *r1 = plane + ty.i1 * j->cfa->stride;
    const float p00 = raw(j, r0[tx.i0]), p01 = raw(j, r0[tx.i1]);
    const float p10 = raw(j, r1[tx.i0]), p11 = raw(j, r1[tx.i1]);
    const float top    = p00 + (p01 - p00) * tx.f;
    const float bottom = p10 + (p11 - p10) * tx.f;

    return top + (bottom - top) * ty.f;
}

/*
 * Full size, job i: output rows y0 to y1 - 1, with the full demosaic of 12.3 at
 * ow = 2 * cw (one output pixel per mosaic pixel, mx = x + 0.5, my = y + 0.5). Plane
 * phases: Gr (1, 0), R (0, 0), Gb (0, 1), B (1, 1). The format note marks this path as
 * untested against the reference.
 */
static int full_job(AVCodecContext *avctx, void *arg, int job, int threadnr)
{
    const ViewJobs *j = arg;
    const TicoRawPlanes *c = j->cfa;
    const int32_t *pgr = c->p[0], *pr = c->p[1], *pgb = c->p[2], *pb = c->p[3];
    const int y0 = (int64_t)j->rows * job       / j->nb_jobs;
    const int y1 = (int64_t)j->rows * (job + 1) / j->nb_jobs;
    int x, y;

    for (y = y0; y < y1; y++) {
        const Tap ty0 = get_tap(y, 0, c->ch), ty1 = get_tap(y, 1, c->ch);
        uint16_t *dg = j->dst[0] + y * j->stride[0];
        uint16_t *db = j->dst[1] + y * j->stride[1];
        uint16_t *dr = j->dst[2] + y * j->stride[2];

        for (x = 0; x < 2 * c->cw; x++) {
            const Tap tx0 = j->tx[0][x], tx1 = j->tx[1][x];
            /* q_Gr = (tx1, ty0), q_R = (tx0, ty0), q_Gb = (tx0, ty1), q_B = (tx1, ty1) */
            const float g   = 0.5f * (sample(j, pgr, tx1, ty0) + sample(j, pgb, tx0, ty1));
            const float g_r = 0.5f * (sample(j, pgr, tx0, ty0) + sample(j, pgb, tx0, ty0));
            const float g_b = 0.5f * (sample(j, pgr, tx1, ty1) + sample(j, pgb, tx1, ty1));
            const float rr  = g + sample(j, pr, tx0, ty0) - g_r;
            const float bb  = g + sample(j, pb, tx1, ty1) - g_b;

            put_pixel(j, normalise(j, rr), normalise(j, g), normalise(j, bb),
                      dg + x, db + x, dr + x);
        }
    }
    return 0;
}

int ff_ticoraw_render_view(TicoRawViewContext *vc, const TicoRawPlanes *cfa,
                           const uint16_t *curve, const TicoRawViewParams *p,
                           AVFrame *frame, AVCodecContext *avctx)
{
    ViewJobs j = { 0 };
    int i, k;

    if (p->white_level <= p->black_level) {
        av_log(avctx, AV_LOG_ERROR, "White level %d is not above the black level %d\n",
               p->white_level, p->black_level);
        return AVERROR_INVALIDDATA;
    }
    ff_thread_once(&encode_table_once, encode_table_init);

    j.cfa   = cfa;
    j.curve = curve;
    j.black = p->black_level;
    j.range = p->white_level - p->black_level;
    setup_colour(&j, p);

    if (!vc->norm) {
        vc->norm = av_malloc_array(TICORAW_CURVE_SIZE, sizeof(*vc->norm));
        if (!vc->norm)
            return AVERROR(ENOMEM);
    }
    for (i = 0; i < TICORAW_CURVE_SIZE; i++)
        vc->norm[i] = normalise(&j, curve[i]);
    j.norm = vc->norm;

    if (p->size == TICORAW_VIEW_FULL) {
        const int w = 2 * cfa->cw;

        if (vc->nb_taps < 2 * w) {
            av_freep(&vc->taps);
            vc->nb_taps = 0;
            vc->taps = av_malloc_array(2 * w, sizeof(*vc->taps));
            if (!vc->taps)
                return AVERROR(ENOMEM);
            vc->nb_taps = 2 * w;
        }
        for (i = 0; i < w; i++) {
            vc->taps[i]     = get_tap(i, 0, cfa->cw);
            vc->taps[w + i] = get_tap(i, 1, cfa->cw);
        }
        j.tx[0] = vc->taps;
        j.tx[1] = vc->taps + w;
    }
    for (k = 0; k < 3; k++) {
        j.dst[k]    = (uint16_t *)frame->data[k];
        j.stride[k] = frame->linesize[k] / 2;
    }

    j.rows    = p->size == TICORAW_VIEW_HALF ? cfa->ch : 2 * cfa->ch;
    j.nb_jobs = ff_ticoraw_nb_row_jobs(avctx, j.rows);
    avctx->execute2(avctx, p->size == TICORAW_VIEW_HALF ? half_job : full_job,
                    &j, NULL, j.nb_jobs);
    return 0;
}

void ff_ticoraw_view_free(TicoRawViewContext *vc)
{
    av_freep(&vc->norm);
    av_freep(&vc->taps);
    vc->nb_taps = 0;
}

void ff_ticoraw_view_pixel(const TicoRawViewParams *p, const float cam[3], uint16_t rgb[3])
{
    ViewJobs j = { 0 };

    ff_thread_once(&encode_table_once, encode_table_init);
    setup_colour(&j, p);
    put_pixel(&j, cam[0], cam[1], cam[2], &rgb[1], &rgb[2], &rgb[0]);
}
