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

/*
 * Every read is bounds checked: the bit readers are the checked variant (they never
 * read past the end of their buffer plus the input padding), and a sub-packet whose
 * content runs past its length, or a length that runs past the precinct, is
 * AVERROR_INVALIDDATA. Every loop is bounded by the band and packet geometry.
 */

#define UNCHECKED_BITSTREAM_READER 0

#include <string.h>

#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "libavutil/error.h"
#include "libavutil/intmath.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"

#include "get_bits.h"
#include "jpegxs.h"
#include "ticoraw_entropy.h"

#define SLH_SIZE            6   /* marker, L = 4, slice index */
#define PACKET_HEADER_SIZE  7   /* Dr 1, Ldat 20, Lcnt 20, Lsgn 15 bits */
#define GROUP_SIZE          4   /* Ng of the supported set; the magnitude planes are 4 bits */
#define MAX_GCLI            15
#define MAX_UNARY           31

static int add_band(TicoRawLayout *l, const TicoRawHeader *h, int comp, int start,
                    int width, int packet, int line, void *logctx)
{
    TicoRawBand *b;
    int nb = l->packet_nb_bands[packet];

    if (l->nb_bands >= TICORAW_NB_BANDS || nb >= TICORAW_MAX_PACKET_BANDS) {
        av_log(logctx, AV_LOG_ERROR,
               "Unsupported band structure: more than %d bands\n", TICORAW_NB_BANDS);
        return AVERROR_PATCHWELCOME;
    }
    b = &l->band[l->nb_bands];
    b->comp          = comp;
    b->start         = start;
    b->width         = width;
    b->nb_groups     = (width + GROUP_SIZE - 1) / GROUP_SIZE;
    b->nb_sig_groups = (b->nb_groups + h->ss - 1) / h->ss;
    l->total_groups += b->nb_groups;

    l->packet_band[packet][nb].band = l->nb_bands;
    l->packet_band[packet][nb].line = line;
    l->packet_nb_bands[packet]++;
    l->nb_bands++;
    return 0;
}

av_cold int ff_ticoraw_layout_init(TicoRawLayout *l, const TicoRawHeader *h, void *logctx)
{
    int line0_band[TICORAW_NB_COMPONENTS];
    int c, j, ret;

    memset(l, 0, sizeof(*l));
    l->nb_packets = TICORAW_NB_PACKETS;

    if (h->ng != GROUP_SIZE || h->ss < 1) {
        av_log(logctx, AV_LOG_ERROR, "Unsupported Ng (ng) %d / Ss (ss) %d\n", h->ng, h->ss);
        return AVERROR_PATCHWELCOME;
    }
    for (c = 0; c < TICORAW_NB_COMPONENTS; c++) {
        int nlx = h->comp_nlx[c], nly = h->comp_nly[c];
        if (!(nlx == 0 && nly == 0) &&
            !(nlx >= 1 && nlx < TICORAW_MAX_PACKET_BANDS && nly == 1)) {
            av_log(logctx, AV_LOG_ERROR,
                   "Unsupported decomposition levels %d x %d of component %d\n", nlx, nly, c);
            return AVERROR_PATCHWELCOME;
        }
    }

    /* Line 0 bands of components 0 to 3, packets 0 to 3. */
    for (c = 0; c < TICORAW_NB_COMPONENTS; c++) {
        int n = h->comp_nlx[c], lo = h->cw, hi[TICORAW_MAX_PACKET_BANDS], start;

        line0_band[c] = l->nb_bands;
        for (j = 1; j <= n; j++) {
            hi[j] = lo / 2;
            lo   -= hi[j];
        }
        if ((ret = add_band(l, h, c, 0, lo, c, 0, logctx)) < 0)
            return ret;
        start = lo;
        for (j = n; j >= 1; j--) {
            if ((ret = add_band(l, h, c, start, hi[j], c, 0, logctx)) < 0)
                return ret;
            start += hi[j];
        }
    }

    /*
     * Line 1 bands, packets 4 to 7. A component without a wavelet codes line 1 with its
     * line 0 band (same band index).
     */
    for (c = 0; c < TICORAW_NB_COMPONENTS; c++) {
        int packet = TICORAW_NB_COMPONENTS + c, lo = (h->cw + 1) / 2;

        if (!h->comp_nlx[c]) {
            int nb = l->packet_nb_bands[packet];
            l->packet_band[packet][nb].band = line0_band[c];
            l->packet_band[packet][nb].line = 1;
            l->packet_nb_bands[packet]++;
            continue;
        }
        if ((ret = add_band(l, h, c, 0,  lo,         packet, 1, logctx)) < 0 ||
            (ret = add_band(l, h, c, lo, h->cw - lo, packet, 1, logctx)) < 0)
            return ret;
    }

    if (l->nb_bands != h->nb_bands) {
        av_log(logctx, AV_LOG_ERROR,
               "Unsupported band count %d, the WGT marker has %d entries\n",
               l->nb_bands, h->nb_bands);
        return AVERROR_PATCHWELCOME;
    }
    l->precinct_header_size = 5 + (2 * l->nb_bands + 7) / 8;
    return 0;
}

av_cold int ff_ticoraw_slice_context_init(TicoRawSliceContext *sc, const TicoRawLayout *l)
{
    int b, pos = 0;

    memset(sc, 0, sizeof(*sc));
    sc->gcli_buf = av_mallocz(FFMAX(l->total_groups, 1));
    if (!sc->gcli_buf)
        return AVERROR(ENOMEM);
    for (b = 0; b < l->nb_bands; b++) {
        sc->gcli[b] = sc->gcli_buf + pos;
        pos += l->band[b].nb_groups;
    }
    return 0;
}

av_cold void ff_ticoraw_slice_context_free(TicoRawSliceContext *sc)
{
    av_freep(&sc->gcli_buf);
    memset(sc->gcli, 0, sizeof(sc->gcli));
}

int ff_ticoraw_find_slices(const uint8_t *cs, int size, const TicoRawHeader *h,
                           const TicoRawLayout *l, TicoRawSliceSpan *spans, void *logctx)
{
    const int end = size - 2; /* EOC */
    int pos = h->first_slice_offset, s, k;

    for (s = 0; s < h->nb_slices; s++) {
        int nb_prec = FFMIN(h->slice_height, h->nb_precincts - s * h->slice_height);

        if (pos < 0 || end - pos < SLH_SIZE || AV_RB16(cs + pos) != JPEGXS_MARKER_SLH ||
            AV_RB16(cs + pos + 2) != 4 || AV_RB16(cs + pos + 4) != s) {
            av_log(logctx, AV_LOG_ERROR, "Missing slice header %d at offset %d\n", s, pos);
            return AVERROR_INVALIDDATA;
        }
        pos += SLH_SIZE;
        spans[s].start = pos;
        for (k = 0; k < nb_prec; k++) {
            int lprc;

            if (end - pos < l->precinct_header_size) {
                av_log(logctx, AV_LOG_ERROR,
                       "Precinct header of slice %d runs past the codestream\n", s);
                return AVERROR_INVALIDDATA;
            }
            lprc = AV_RB24(cs + pos);
            if (lprc > end - pos - l->precinct_header_size) {
                av_log(logctx, AV_LOG_ERROR,
                       "Precinct of slice %d (Lprc %d) runs past the codestream\n", s, lprc);
                return AVERROR_INVALIDDATA;
            }
            pos += l->precinct_header_size + lprc;
        }
        spans[s].end = pos;
    }
    /* Any bytes may come between the last precinct and EOC. */
    return 0;
}

/* Count the 1 bits before the next 0 bit, the 0 bit is consumed. */
static av_always_inline int read_unary(GetBitContext *gb)
{
    unsigned v = show_bits_long(gb, 32);
    int u;

    if (v == 0xFFFFFFFFU)
        return -1;
    u = ff_clz(~v);
    if (u > MAX_UNARY)
        return -1;
    skip_bits_long(gb, u + 1);
    return u;
}

/* Uniform reconstruction, m + floor(m / 2^z) + floor(m / 2^2z) + ... */
static av_always_inline int dequant(int m, int t, int gcli)
{
    const int zeta = gcli - t + 1;
    int mh = 0;

    if (!t)
        return m;
    for (; m; m >>= zeta)
        mh += m;
    return mh;
}

typedef struct PrecinctParams {
    int k;                              /* precinct index */
    int q, r;                           /* quantization and refinement values */
    uint8_t mode[TICORAW_NB_BANDS];     /* D[b] */
} PrecinctParams;

static int decode_packet(TicoRawSliceContext *sc, const TicoRawLayout *l,
                         const TicoRawHeader *h, const uint8_t *cs, int *ppos, int pend,
                         int packet, const PrecinctParams *pp,
                         int32_t *const planes[4], ptrdiff_t stride, void *logctx)
{
    const int nb_bands = l->packet_nb_bands[packet];
    GetBitContext gsig, gcnt, gdat, gsgn;
    int pos = *ppos, dr, ldat, lcnt, lsgn, nb_flags = 0, sig_len, i;
    uint64_t hdr;

    if (pend - pos < PACKET_HEADER_SIZE)
        goto overrun;
    hdr  = (uint64_t)AV_RB32(cs + pos) << 24 | AV_RB24(cs + pos + 4);
    dr   = hdr >> 55;
    ldat = (hdr >> 35) & 0xFFFFF;
    lcnt = (hdr >> 15) & 0xFFFFF;
    lsgn = hdr & 0x7FFF;
    pos += PACKET_HEADER_SIZE;

    /* Significance sub-packet: no length field, one flag per significance group. */
    if (!dr)
        for (i = 0; i < nb_bands; i++) {
            int b = l->packet_band[packet][i].band;
            if (pp->mode[b] & 2)
                nb_flags += l->band[b].nb_sig_groups;
        }
    sig_len = (nb_flags + 7) >> 3;

    if (sig_len > pend - pos)
        goto overrun;
    init_get_bits8(&gsig, cs + pos, sig_len);
    pos += sig_len;
    if (lcnt > pend - pos)
        goto overrun;
    init_get_bits8(&gcnt, cs + pos, lcnt);
    pos += lcnt;
    if (ldat > pend - pos)
        goto overrun;
    init_get_bits8(&gdat, cs + pos, ldat);
    pos += ldat;
    if (lsgn > pend - pos)
        goto overrun;
    init_get_bits8(&gsgn, cs + pos, lsgn);
    pos += lsgn;

    /*
     * The four sub-packets are read with their own readers, band by band and group by
     * group. Each sub-packet is still read in its own order (all bands of the packet, in
     * band order), so this gives the same result as decoding every GCLI first.
     */
    for (i = 0; i < nb_bands; i++) {
        const int b = l->packet_band[packet][i].band;
        const int line = l->packet_band[packet][i].line;
        const TicoRawBand *band = &l->band[b];
        const int row = 2 * pp->k + line;
        const int use_sig = !dr && (pp->mode[b] & 2);
        const int vertical = !dr && (pp->mode[b] & 1) && sc->have_src[b];
        const int ts = sc->gtli[b];
        uint8_t *gcli = sc->gcli[b];
        int32_t *dst = row < h->ch ? planes[band->comp] + row * stride + band->start : NULL;
        int t, s, g;

        /* GTLI: priorities are only compared, gains only subtracted. */
        t = pp->q - h->band_gain[b] - (h->band_priority[b] < pp->r);
        t = av_clip(t, 0, MAX_GCLI);

        for (s = 0; s < band->nb_sig_groups; s++) {
            const int flag = use_sig ? get_bits1(&gsig) : 0;
            const int gend = FFMIN((s + 1) * h->ss, band->nb_groups);

            for (g = s * h->ss; g < gend; g++) {
                const int pi = vertical ? FFMAX3(gcli[g], ts, t) : t;
                int a[GROUP_SIZE] = { 0 }, n, j, v;

                if (dr) {
                    v = get_bits(&gcnt, h->br);
                } else if (flag) {
                    v = pi;
                } else {
                    int u = read_unary(&gcnt);
                    if (u < 0) {
                        av_log(logctx, AV_LOG_ERROR,
                               "Unary GCLI value with more than %d ones in precinct %d\n",
                               MAX_UNARY, pp->k);
                        return AVERROR_INVALIDDATA;
                    }
                    if (!vertical) {
                        v = t + u;
                    } else {
                        const int rho = pi - t;
                        int delta;
                        if (u > 2 * rho)
                            delta = u - rho;
                        else if (u & 1)
                            delta = -((u + 1) >> 1);
                        else
                            delta = u >> 1;
                        v = pi + delta;
                    }
                }
                if (v < 0 || v > MAX_GCLI) {
                    av_log(logctx, AV_LOG_ERROR, "GCLI %d out of range in precinct %d\n",
                           v, pp->k);
                    return AVERROR_INVALIDDATA;
                }
                gcli[g] = v;

                /* Magnitudes: n bit planes of 4 bits, most significant first. */
                n = v - t;
                for (; n > 0; n--) {
                    const int bits = get_bits(&gdat, GROUP_SIZE);
                    a[0] = a[0] << 1 | (bits >> 3);
                    a[1] = a[1] << 1 | (bits >> 2 & 1);
                    a[2] = a[2] << 1 | (bits >> 1 & 1);
                    a[3] = a[3] << 1 | (bits      & 1);
                }

                /* Signs and dequantization. */
                for (j = 0; j < GROUP_SIZE; j++) {
                    const int idx = g * GROUP_SIZE + j;
                    int m = a[j] << t;

                    if (m) {
                        m = dequant(m, t, v) << h->fq;
                        if (get_bits1(&gsgn))
                            m = -m;
                    }
                    if (dst && idx < band->width)
                        dst[idx] = m;
                }
            }
        }
        if (get_bits_left(&gcnt) < 0 || get_bits_left(&gdat) < 0 ||
            get_bits_left(&gsgn) < 0) {
            av_log(logctx, AV_LOG_ERROR,
                   "%s sub-packet content runs past its length in precinct %d\n",
                   get_bits_left(&gcnt) < 0 ? "GCLI" :
                   get_bits_left(&gdat) < 0 ? "Data" : "Sign", pp->k);
            return AVERROR_INVALIDDATA;
        }
        /* This band line is now the prediction source of band index b. */
        sc->gtli[b]     = t;
        sc->have_src[b] = 1;
    }

    *ppos = pos;
    return 0;

overrun:
    av_log(logctx, AV_LOG_ERROR, "Packet %d runs past its precinct %d\n", packet, pp->k);
    return AVERROR_INVALIDDATA;
}

int ff_ticoraw_decode_slice(TicoRawSliceContext *sc, const TicoRawLayout *l,
                            const TicoRawHeader *h, const uint8_t *cs,
                            const TicoRawSliceSpan *span, int slice,
                            int32_t *const planes[4], ptrdiff_t stride, void *logctx)
{
    const int first = slice * h->slice_height;
    const int last  = FFMIN(first + h->slice_height, h->nb_precincts);
    int pos = span->start, k, b, p, ret;

    /* Prediction starts fresh in every slice. */
    memset(sc->have_src, 0, sizeof(sc->have_src));
    memset(sc->gtli, 0, sizeof(sc->gtli));

    for (k = first; k < last; k++) {
        PrecinctParams pp;
        int lprc, pend;

        if (span->end - pos < l->precinct_header_size)
            goto overrun;
        lprc = AV_RB24(cs + pos);
        if (lprc > span->end - pos - l->precinct_header_size)
            goto overrun;
        pend = pos + l->precinct_header_size + lprc;

        pp.k = k;
        pp.q = cs[pos + 3];
        pp.r = cs[pos + 4];
        for (b = 0; b < l->nb_bands; b++)
            pp.mode[b] = cs[pos + 5 + (b >> 2)] >> (6 - 2 * (b & 3)) & 3;

        pos += l->precinct_header_size;
        for (p = 0; p < l->nb_packets; p++) {
            ret = decode_packet(sc, l, h, cs, &pos, pend, p, &pp, planes, stride, logctx);
            if (ret < 0)
                return ret;
        }
        /* Zero padding up to Lprc. */
        pos = pend;
    }
    if (pos != span->end)
        goto overrun;
    return 0;

overrun:
    av_log(logctx, AV_LOG_ERROR, "Precinct runs past slice %d\n", slice);
    return AVERROR_INVALIDDATA;
}
