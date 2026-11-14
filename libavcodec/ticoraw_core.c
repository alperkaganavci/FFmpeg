/*
 * TICO RAW (Nikon N-RAW, R3D NE) core decoder: codestream to companded CFA planes
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

#include <string.h>

#include "libavutil/attributes.h"
#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "libavutil/error.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"

#include "avcodec.h"
#include "ticoraw_core.h"
#include "ticoraw_dsp.h"
#include "ticoraw_entropy.h"

/* The header fields that the buffers and the band layout depend on. */
typedef struct CoreGeometry {
    int cw, ch, ng, ss, nb_bands, nb_precincts, slice_height, nb_slices;
    uint8_t comp_nlx[TICORAW_NB_COMPONENTS];
    uint8_t comp_nly[TICORAW_NB_COMPONENTS];
} CoreGeometry;

/* Scratch of one slice thread. Jobs that run at the same time never share one. */
typedef struct CoreThread {
    TicoRawSliceContext slice_ctx;
    int32_t *row_tmp;               /* idwt53_h scratch, cw elements */
} CoreThread;

struct TicoRawCore {
    AVCodecContext *avctx;          /* runs the jobs (execute2), log context */
    TicoRawDSPContext dsp;
    int nb_threads;                 /* slice threads of avctx, 1 without slice threads */
    CoreThread *thread;             /* nb_threads entries, indexed by threadnr */

    /* Made for geom; rebuilt when the geometry of a header differs. */
    int have_geom;
    CoreGeometry geom;
    TicoRawLayout layout;
    TicoRawSliceSpan *spans;        /* nb_slices entries */
    int *slice_ret;                 /* nb_slices entries: return value of each slice job */
    int32_t *plane_buf;
    int32_t *plane[4];
    ptrdiff_t stride;               /* in int32_t elements */
    /* L_j: low-pass width after j levels */
    int len[TICORAW_NB_COMPONENTS][TICORAW_MAX_PACKET_BANDS];
    int nb_row_jobs;                /* Star-Tetrix row jobs */
    int nb_strips, strip_width;     /* vertical wavelet column jobs per component */

    /* The call in progress, read by the jobs. */
    const TicoRawHeader *h;
    const uint8_t *cs;
    int st_pass;
};

static void get_geometry(CoreGeometry *g, const TicoRawHeader *h)
{
    memset(g, 0, sizeof(*g));
    g->cw           = h->cw;
    g->ch           = h->ch;
    g->ng           = h->ng;
    g->ss           = h->ss;
    g->nb_bands     = h->nb_bands;
    g->nb_precincts = h->nb_precincts;
    g->slice_height = h->slice_height;
    g->nb_slices    = h->nb_slices;
    memcpy(g->comp_nlx, h->comp_nlx, sizeof(g->comp_nlx));
    memcpy(g->comp_nly, h->comp_nly, sizeof(g->comp_nly));
}

static void core_free_geometry(TicoRawCore *c)
{
    int t;

    for (t = 0; c->thread && t < c->nb_threads; t++) {
        ff_ticoraw_slice_context_free(&c->thread[t].slice_ctx);
        av_freep(&c->thread[t].row_tmp);
    }
    av_freep(&c->spans);
    av_freep(&c->slice_ret);
    av_freep(&c->plane_buf);
    memset(c->plane, 0, sizeof(c->plane));
    c->have_geom = 0;
}

static int core_setup(TicoRawCore *c, const TicoRawHeader *h)
{
    CoreGeometry g;
    int i, j, t, ret;

    get_geometry(&g, h);
    if (c->have_geom && !memcmp(&g, &c->geom, sizeof(g)))
        return 0;

    core_free_geometry(c);
    if ((ret = ff_ticoraw_layout_init(&c->layout, h, c->avctx)) < 0)
        return ret;

    c->stride    = FFALIGN(h->cw, 16);
    c->plane_buf = av_malloc_array(4 * c->stride, h->ch * sizeof(int32_t));
    c->spans     = av_malloc_array(h->nb_slices, sizeof(*c->spans));
    c->slice_ret = av_malloc_array(h->nb_slices, sizeof(*c->slice_ret));
    if (!c->plane_buf || !c->spans || !c->slice_ret) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    for (t = 0; t < c->nb_threads; t++) {
        if ((ret = ff_ticoraw_slice_context_init(&c->thread[t].slice_ctx, &c->layout)) < 0)
            goto fail;
        c->thread[t].row_tmp = av_malloc_array(h->cw, sizeof(int32_t));
        if (!c->thread[t].row_tmp) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }
    for (i = 0; i < 4; i++)
        c->plane[i] = c->plane_buf + i * c->stride * h->ch;

    /* len[c][j] = L_j, the low-pass width after j levels */
    for (i = 0; i < TICORAW_NB_COMPONENTS; i++) {
        c->len[i][0] = h->cw;
        for (j = 1; j <= h->comp_nlx[i]; j++)
            c->len[i][j] = c->len[i][j - 1] - c->len[i][j - 1] / 2;
    }

    c->nb_row_jobs = ff_ticoraw_nb_row_jobs(c->avctx, h->ch);
    c->strip_width = FFALIGN((h->cw + c->nb_threads - 1) / c->nb_threads, 16);
    c->nb_strips   = (h->cw + c->strip_width - 1) / c->strip_width;

    c->geom      = g;
    c->have_geom = 1;
    return 0;

fail:
    core_free_geometry(c);
    return ret;
}

int ff_ticoraw_nb_row_jobs(const AVCodecContext *avctx, int rows)
{
    if (!(avctx->active_thread_type & FF_THREAD_SLICE) || avctx->thread_count <= 1)
        return 1;
    return av_clip(4 * avctx->thread_count, 1, FFMAX(rows, 1));
}

av_cold int ff_ticoraw_core_init(TicoRawCore **core, const TicoRawHeader *h,
                                 AVCodecContext *avctx)
{
    TicoRawCore *c = av_mallocz(sizeof(*c));
    int ret;

    if (!c)
        return AVERROR(ENOMEM);
    c->avctx      = avctx;
    c->nb_threads = avctx->active_thread_type & FF_THREAD_SLICE ?
                    FFMAX(avctx->thread_count, 1) : 1;
    c->thread     = av_calloc(c->nb_threads, sizeof(*c->thread));
    if (!c->thread) {
        av_free(c);
        return AVERROR(ENOMEM);
    }
    ff_ticoraw_dsp_init(&c->dsp);
    if ((ret = core_setup(c, h)) < 0) {
        ff_ticoraw_core_free(&c);
        return ret;
    }
    *core = c;
    return 0;
}

av_cold void ff_ticoraw_core_free(TicoRawCore **core)
{
    if (!*core)
        return;
    core_free_geometry(*core);
    av_freep(&(*core)->thread);
    av_freep(core);
}

/*
 * Horizontal inverse wavelet steps on rows y0 to y1 - 1 of every component.
 * Each row is transformed on its own.
 */
static void horizontal_rows(TicoRawCore *c, int32_t *tmp, int y0, int y1)
{
    int comp, y, j;

    for (comp = 0; comp < TICORAW_NB_COMPONENTS; comp++) {
        const int n = c->geom.comp_nlx[comp];
        const int *len = c->len[comp];

        if (!n)
            continue;   /* no wavelet: planes_st equals coeffs */
        for (y = y0; y < y1; y++) {
            int32_t *row = c->plane[comp] + y * c->stride;
            if (y & 1) {
                c->dsp.idwt53_h(row, tmp, len[0]);
            } else {
                for (j = n - 1; j >= 0; j--)
                    c->dsp.idwt53_h(row, tmp, len[j]);
            }
        }
    }
}

/*
 * Job s: entropy decode slice s (slices are independent), then run the horizontal
 * wavelet steps on its rows: precinct k holds rows 2k and 2k + 1.
 */
static int slice_job(AVCodecContext *avctx, void *arg, int s, int threadnr)
{
    TicoRawCore *c = arg;
    CoreThread *t;
    int ret, y0, y1;

    av_assert1(threadnr < c->nb_threads);
    t   = &c->thread[threadnr];
    ret = ff_ticoraw_decode_slice(&t->slice_ctx, &c->layout, c->h, c->cs, &c->spans[s], s,
                                  c->plane, c->stride, c->avctx);
    if (ret < 0)
        return ret;

    y0 = 2 * s * c->geom.slice_height;
    y1 = FFMIN(y0 + 2 * c->geom.slice_height, c->geom.ch);
    horizontal_rows(c, t->row_tmp, y0, y1);
    return 0;
}

/* Job comp * nb_strips + i: the vertical wavelet step on column strip i of comp. */
static int vertical_job(AVCodecContext *avctx, void *arg, int job, int threadnr)
{
    TicoRawCore *c = arg;
    const int comp = job / c->nb_strips;
    const int x0   = job % c->nb_strips * c->strip_width;

    if (!c->geom.comp_nlx[comp] || x0 >= c->geom.cw)
        return 0;
    c->dsp.idwt53_v(c->plane[comp] + x0, c->stride,
                    FFMIN(c->strip_width, c->geom.cw - x0), c->geom.ch);
    return 0;
}

/* Job i: Star-Tetrix pass st_pass on row band i. */
static int star_tetrix_job(AVCodecContext *avctx, void *arg, int job, int threadnr)
{
    TicoRawCore *c = arg;
    const int ch = c->geom.ch;
    const int y0 = (int64_t)ch * job       / c->nb_row_jobs;
    const int y1 = (int64_t)ch * (job + 1) / c->nb_row_jobs;

    c->dsp.inv_star_tetrix[c->st_pass](c->plane, c->stride, c->geom.cw, ch, y0, y1);
    return 0;
}

int ff_ticoraw_decode_planes(TicoRawCore *c, const uint8_t *cs, int size,
                             const TicoRawHeader *h, TicoRawPlanes *out)
{
    AVCodecContext *avctx = c->avctx;
    int s, i, ret;

    if ((ret = core_setup(c, h)) < 0)
        return ret;
    if ((ret = ff_ticoraw_find_slices(cs, size, h, &c->layout, c->spans, avctx)) < 0)
        return ret;

    c->h    = h;
    c->cs   = cs;
    avctx->execute2(avctx, slice_job, c, c->slice_ret, h->nb_slices);
    for (s = 0; s < h->nb_slices; s++)
        if (c->slice_ret[s] < 0)
            return c->slice_ret[s];

    avctx->execute2(avctx, vertical_job, c, NULL, TICORAW_NB_COMPONENTS * c->nb_strips);
    for (c->st_pass = 0; c->st_pass < TICORAW_ST_PASSES; c->st_pass++)
        avctx->execute2(avctx, star_tetrix_job, c, NULL, c->nb_row_jobs);

    for (i = 0; i < 4; i++)
        out->p[i] = c->plane[i];
    out->cw     = c->geom.cw;
    out->ch     = c->geom.ch;
    out->stride = c->stride;
    return 0;
}

#define CURVE_BIAS (5 << 15)   /* 5/8 of 2^18 */

void ff_ticoraw_build_curve(uint16_t *lut, const TicoRawHeader *h)
{
    int i;

    for (i = 0; i < TICORAW_CURVE_SIZE; i++) {
        int64_t e = i - h->curve_pivot, phi, tau;

        if (e < 0) {
            phi = -e * e;
        } else {
            tau = (e * h->curve_slope + 32768) >> 16;
            phi = tau * tau;
        }
        /*
         * The curve is black_level + (phi + CURVE_BIAS) / 2^18, rounded down: the
         * arithmetic shift floors also for a negative phi. The bias is 5/8 of the
         * divisor, a constant of the curve and not a rounding to the nearest value.
         */
        lut[i] = av_clip64(h->black_level + ((phi + CURVE_BIAS) >> 18),
                           0, TICORAW_WHITE_LEVEL);
    }
}

void ff_ticoraw_planes_to_bayer(uint16_t *dst, ptrdiff_t dst_stride,
                                const TicoRawPlanes *planes, const uint16_t *lut,
                                int y0, int y1)
{
    int x, y;

    for (y = y0; y < y1; y++) {
        const int32_t *gr = planes->p[0] + y * planes->stride;
        const int32_t *r  = planes->p[1] + y * planes->stride;
        const int32_t *gb = planes->p[2] + y * planes->stride;
        const int32_t *b  = planes->p[3] + y * planes->stride;
        uint16_t *row0 = dst + 2 * (y - y0) * dst_stride;
        uint16_t *row1 = row0 + dst_stride;

        /* code index i = clip(v + 32768, 0, 65535) */
        for (x = 0; x < planes->cw; x++) {
            AV_WL16(&row0[2 * x],     lut[av_clip_uint16(r[x]  + 32768)]);
            AV_WL16(&row0[2 * x + 1], lut[av_clip_uint16(gr[x] + 32768)]);
            AV_WL16(&row1[2 * x],     lut[av_clip_uint16(gb[x] + 32768)]);
            AV_WL16(&row1[2 * x + 1], lut[av_clip_uint16(b[x]  + 32768)]);
        }
    }
}
