/*
 * TICO RAW (Nikon N-RAW, R3D NE) inverse wavelet and inverse Star-Tetrix
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

#include "config.h"
#include "libavutil/attributes.h"
#include "libavutil/common.h"

#include "ticoraw_dsp.h"

/* Format note 7.1. Index replacements give the whole-sample symmetric extension. */
static void idwt53_h_c(int32_t *x, int32_t *tmp, int n)
{
    const int ns = (n + 1) >> 1, nd = n >> 1;
    const int32_t *s = tmp, *d = tmp + ns;
    int i;

    if (n < 2)
        return;
    memcpy(tmp, x, n * sizeof(*x));

    /* even samples: d[-1] -> d[0], d[nd] -> d[nd - 1] */
    x[0] = s[0] - ((d[0] + d[0] + 2) >> 2);
    for (i = 1; i < nd; i++)
        x[2 * i] = s[i] - ((d[i - 1] + d[i] + 2) >> 2);
    if (ns > nd)
        x[2 * nd] = s[nd] - ((d[nd - 1] + d[nd - 1] + 2) >> 2);

    /* odd samples, after all even ones: x[n] -> x[n - 2] when n is even */
    for (i = 0; i < nd - 1; i++)
        x[2 * i + 1] = d[i] + ((x[2 * i] + x[2 * i + 2]) >> 1);
    i = nd - 1;
    x[2 * i + 1] = d[i] + ((x[2 * i] + (2 * i + 2 < n ? x[2 * i + 2] : x[2 * i])) >> 1);
}

/*
 * Format note 7.2 step 3: 7.1 down every column, s[i] = row 2i, d[i] = row 2i + 1.
 * Done row by row in place: the even rows first (they read only odd rows), then the
 * odd rows (they read only the finished even rows).
 */
static void idwt53_v_c(int32_t *p, ptrdiff_t stride, int w, int h)
{
    const int ns = (h + 1) >> 1, nd = h >> 1;
    int i, x;

    if (h < 2)
        return;

    for (i = 0; i < ns; i++) {
        int32_t *e = p + 2 * i * stride;
        const int32_t *dm = p + (2 * (i > 0 ? i - 1 : 0) + 1) * stride;
        const int32_t *dp = p + (2 * (i < nd ? i : nd - 1) + 1) * stride;

        for (x = 0; x < w; x++)
            e[x] -= (dm[x] + dp[x] + 2) >> 2;
    }
    for (i = 0; i < nd; i++) {
        int32_t *o = p + (2 * i + 1) * stride;
        const int32_t *a = p + 2 * i * stride;
        const int32_t *b = 2 * i + 2 < h ? a + 2 * stride : a;

        for (x = 0; x < w; x++)
            o[x] += (a[x] + b[x]) >> 1;
    }
}

/*
 * Format note 8: five lifting steps, each over the whole picture before the next. A step
 * reads only planes other than the one it updates (apart from the sample itself), so the
 * order of the samples inside a step does not matter, and the rows of a step may be done
 * in any split. Neighbours repeat the edge sample.
 *
 * Valid streams keep every value near 2^16 (note 7.4). The input is clamped to +-2^24 so
 * that crafted data cannot overflow the four-term sums: five steps grow the magnitude to
 * at most 3.5 * 2^24, and the largest sum stays below 2^28.
 */
#define ST_LIMIT (1 << 24)

#define ST_ARGS int32_t *const p[4], ptrdiff_t stride, int w, int h, int y0, int y1
#define ROW(P, Y) ((P) + (ptrdiff_t)(Y) * stride)
#define XM (x > 0 ? x - 1 : 0)
#define XP (x < w - 1 ? x + 1 : w - 1)

static void st_clamp_c(ST_ARGS)
{
    int x, y, i;

    for (i = 0; i < 4; i++)
        for (y = y0; y < y1; y++) {
            int32_t *r = ROW(p[i], y);
            for (x = 0; x < w; x++)
                r[x] = av_clip(r[x], -ST_LIMIT, ST_LIMIT);
        }
}

/* 1. A -= floor((E(x, y-) + E(x+, y-) + E(x, y) + E(x+, y)) / 8) */
static void st_step1_c(ST_ARGS)
{
    int32_t *const A = p[0];
    const int32_t *const E = p[2];
    int x, y;

    for (y = y0; y < y1; y++) {
        int32_t *a = ROW(A, y);
        const int32_t *e0 = ROW(E, y > 0 ? y - 1 : 0), *e1 = ROW(E, y);
        for (x = 0; x < w; x++)
            a[x] -= (e0[x] + e0[XP] + e1[x] + e1[XP]) >> 3;
    }
}

/* 2. E += -floor((C(x, y) + C(x, y+) + F(x-, y) + F(x, y)) / 8)
 *         + floor((A(x-, y) + A(x, y) + A(x-, y+) + A(x, y+)) / 4) */
static void st_step2_c(ST_ARGS)
{
    const int32_t *const A = p[0], *const C = p[1], *const F = p[3];
    int32_t *const E = p[2];
    int x, y;

    for (y = y0; y < y1; y++) {
        int32_t *e = ROW(E, y);
        const int yp = y < h - 1 ? y + 1 : h - 1;
        const int32_t *c0 = ROW(C, y), *c1 = ROW(C, yp), *f0 = ROW(F, y);
        const int32_t *a0 = ROW(A, y), *a1 = ROW(A, yp);
        for (x = 0; x < w; x++) {
            const int xm = XM;
            e[x] += -((c0[x] + c1[x] + f0[xm] + f0[x]) >> 3) +
                     ((a0[xm] + a0[x] + a1[xm] + a1[x]) >> 2);
        }
    }
}

/* 3. A -= floor((C(x, y) + C(x+, y) + F(x, y-) + F(x, y)) / 8) */
static void st_step3_c(ST_ARGS)
{
    int32_t *const A = p[0];
    const int32_t *const C = p[1], *const F = p[3];
    int x, y;

    for (y = y0; y < y1; y++) {
        int32_t *a = ROW(A, y);
        const int32_t *c0 = ROW(C, y), *f0 = ROW(F, y > 0 ? y - 1 : 0), *f1 = ROW(F, y);
        for (x = 0; x < w; x++)
            a[x] -= (c0[x] + c0[XP] + f0[x] + f1[x]) >> 3;
    }
}

/* 4. C += floor((A(x-, y) + A(x, y) + E(x, y-) + E(x, y)) / 4) */
static void st_step4_c(ST_ARGS)
{
    const int32_t *const A = p[0], *const E = p[2];
    int32_t *const C = p[1];
    int x, y;

    for (y = y0; y < y1; y++) {
        int32_t *c = ROW(C, y);
        const int32_t *a0 = ROW(A, y), *e0 = ROW(E, y > 0 ? y - 1 : 0), *e1 = ROW(E, y);
        for (x = 0; x < w; x++)
            c[x] += (a0[XM] + a0[x] + e0[x] + e1[x]) >> 2;
    }
}

/* 5. F += floor((A(x, y) + A(x, y+) + E(x, y) + E(x+, y)) / 4) */
static void st_step5_c(ST_ARGS)
{
    const int32_t *const A = p[0], *const E = p[2];
    int32_t *const F = p[3];
    int x, y;

    for (y = y0; y < y1; y++) {
        int32_t *f = ROW(F, y);
        const int32_t *a0 = ROW(A, y), *a1 = ROW(A, y < h - 1 ? y + 1 : h - 1);
        const int32_t *e1 = ROW(E, y);
        for (x = 0; x < w; x++)
            f[x] += (a0[x] + a1[x] + e1[x] + e1[XP]) >> 2;
    }
}

#undef ST_ARGS
#undef ROW
#undef XM
#undef XP

av_cold void ff_ticoraw_dsp_init(TicoRawDSPContext *c)
{
    c->idwt53_h           = idwt53_h_c;
    c->idwt53_v           = idwt53_v_c;
    c->inv_star_tetrix[0] = st_clamp_c;
    c->inv_star_tetrix[1] = st_step1_c;
    c->inv_star_tetrix[2] = st_step2_c;
    c->inv_star_tetrix[3] = st_step3_c;
    c->inv_star_tetrix[4] = st_step4_c;
    c->inv_star_tetrix[5] = st_step5_c;

#if ARCH_AARCH64
    ff_ticoraw_dsp_init_aarch64(c);
#endif
}
