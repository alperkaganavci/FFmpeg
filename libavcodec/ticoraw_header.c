/*
 * TICO RAW (Nikon N-RAW, R3D NE) codestream header parser
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

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "libavutil/common.h"
#include "libavutil/error.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"

#include "jpegxs.h"
#include "ticoraw_header.h"

/*
 * The supported set is the parameter set that Nikon cameras write. Every value
 * outside it is rejected with AVERROR_PATCHWELCOME and a message with the field and value.
 */
#define PIH_BODY_STD_SIZE   24 /* standard PIH body, without the vendor bytes */
#define PIH_BODY_SIZE       (PIH_BODY_STD_SIZE + TICORAW_VENDOR_BYTES) /* L = 39 */
#define CAP_BODY_SIZE       32
#define CDT_BYTES_PER_COMP  3
#define WGT_BYTES_PER_BAND  2
#define SLH_SIZE            6  /* marker, L, slice index */
#define MIN_PRECINCT_SIZE   68 /* precinct header (12) and 8 packet headers (8 * 7) */
#define MIN_CW              64 /* smaller pictures are not supported */
#define MIN_CH              4

#define SUPPORTED_DEPTH     14

static const char cap_vendor_prefix[] = "CONTACT_INTOPIX_";
/* CDT byte 3 per component: horizontal levels in the high nibble, vertical in the low one */
static const uint8_t supported_comp_levels[TICORAW_NB_COMPONENTS] = { 0x51, 0x51, 0x00, 0x51 };

/* Fail with AVERROR_PATCHWELCOME unless value == expected. */
static int require(void *logctx, const char *name, const char *key, int value, int expected)
{
    if (value == expected)
        return 0;
    av_log(logctx, AV_LOG_ERROR,
           "Unsupported %s (%s) %d, only %d is supported\n", name, key, value, expected);
    return AVERROR_PATCHWELCOME;
}

#define REQUIRE(name, key, value, expected) do {                           \
    int ret_ = require(logctx, name, key, value, expected);                \
    if (ret_ < 0)                                                          \
        return ret_;                                                       \
} while (0)

/* The marker segments found before the first slice: body pointers and body lengths. */
typedef struct Segment {
    const uint8_t *body;
    int len;
} Segment;

static int parse_pih(const uint8_t *b, int len, TicoRawHeader *h, void *logctx)
{
    int fslc, ppoc, lh, rl, qpih, fs, rm;

    if (len < PIH_BODY_STD_SIZE) {
        av_log(logctx, AV_LOG_ERROR, "PIH is too short (L = %d)\n", len + 2);
        return AVERROR_INVALIDDATA;
    }
    REQUIRE("PIH length", "Lpih", len + 2, PIH_BODY_SIZE + 2);

    h->lcod = AV_RB32(b);
    REQUIRE("Ppih", "ppih", AV_RB16(b + 4), 0);
    REQUIRE("Plev", "plev", AV_RB16(b + 6), 0);

    h->width  = AV_RB16(b + 8);
    h->height = AV_RB16(b + 10);
    if (!h->width || !h->height) {
        av_log(logctx, AV_LOG_ERROR, "Invalid picture size %dx%d\n", h->width, h->height);
        return AVERROR_INVALIDDATA;
    }
    if ((h->width | h->height) & 1) {
        av_log(logctx, AV_LOG_ERROR,
               "Unsupported %s (%s) %d, only even sizes are supported (size %dx%d)\n",
               h->width & 1 ? "Wf" : "Hf", h->width & 1 ? "width" : "height",
               h->width & 1 ? h->width : h->height, h->width, h->height);
        return AVERROR_PATCHWELCOME;
    }
    h->cw = h->width / 2;
    h->ch = h->height / 2;
    if (h->cw < MIN_CW || h->ch < MIN_CH) {
        av_log(logctx, AV_LOG_ERROR,
               "Unsupported Wf x Hf (width x height) %dx%d, "
               "the component size must be at least %dx%d\n",
               h->width, h->height, MIN_CW, MIN_CH);
        return AVERROR_PATCHWELCOME;
    }

    REQUIRE("Cw", "precinct_width", AV_RB16(b + 12), 0);
    h->slice_height = AV_RB16(b + 14);
    REQUIRE("Hsl", "slice_height", h->slice_height, 16);

    h->nc = b[16];
    REQUIRE("Nc", "nc", h->nc, TICORAW_NB_COMPONENTS);
    h->ng = b[17];
    REQUIRE("Ng", "ng", h->ng, 4);
    h->ss = b[18];
    REQUIRE("Ss", "ss", h->ss, 8);
    h->bw = b[19];
    REQUIRE("Bw", "bw", h->bw, 18);
    h->fq = b[20] >> 4;
    REQUIRE("Fq", "fq", h->fq, 4);
    h->br = b[20] & 15;
    REQUIRE("Br", "br", h->br, 4);

    /*
     * Cpih = 0 is the value the Star-Tetrix streams store. The profile check (CAP
     * vendor string, PIH length, CDT size, Nc) is done by the caller and by the checks
     * above.
     */
    fslc = b[21] >> 7;
    ppoc = (b[21] >> 4) & 7;
    h->cpih = b[21] & 15;
    REQUIRE("Fslc", "fslc", fslc, 0);
    REQUIRE("Ppoc", "ppoc", ppoc, 1);
    REQUIRE("Cpih", "cpih", h->cpih, 0);

    h->nlx = b[22] >> 4;
    h->nly = b[22] & 15;
    REQUIRE("NLx", "nlx", h->nlx, 5);
    REQUIRE("NLy", "nly", h->nly, 1);

    lh   = b[23] >> 7;
    rl   = (b[23] >> 6) & 1;
    qpih = (b[23] >> 4) & 3;
    fs   = (b[23] >> 2) & 3;
    rm   = b[23] & 3;
    REQUIRE("Lh", "lh", lh, 0);
    REQUIRE("Rl", "rl", rl, 0);
    REQUIRE("Qpih", "qpih", qpih, 1);
    REQUIRE("Fs", "fs", fs, 1);
    REQUIRE("Rm", "rm", rm, 0);

    /* Vendor bytes: any value. Only the transfer curve parameters are read. */
    memcpy(h->vendor, b + PIH_BODY_STD_SIZE, TICORAW_VENDOR_BYTES);
    h->black_level = ((h->vendor[3] & 15) << 8) | h->vendor[4];
    h->curve_slope = ((h->vendor[7] & 3) << 16) | (h->vendor[8] << 8) | h->vendor[9];
    h->curve_pivot = (h->vendor[10] << 8) | h->vendor[11];
    return 0;
}

static int parse_cdt(const uint8_t *b, int len, TicoRawHeader *h, void *logctx)
{
    char name[64];
    int c;

    REQUIRE("CDT length", "Lcdt", len + 2, CDT_BYTES_PER_COMP * TICORAW_NB_COMPONENTS + 2);

    for (c = 0; c < TICORAW_NB_COMPONENTS; c++) {
        const uint8_t *e = b + CDT_BYTES_PER_COMP * c;

        h->comp_depth[c] = e[0];
        h->comp_sx[c]    = e[1] >> 4;
        h->comp_sy[c]    = e[1] & 15;
        h->comp_nlx[c]   = e[2] >> 4;
        h->comp_nly[c]   = e[2] & 15;

        snprintf(name, sizeof(name), "bit depth of component %d", c);
        REQUIRE(name, "comp_depth", e[0], SUPPORTED_DEPTH);
        snprintf(name, sizeof(name), "Sx of component %d", c);
        REQUIRE(name, "comp_sx", e[1] >> 4, 1);
        snprintf(name, sizeof(name), "Sy of component %d", c);
        REQUIRE(name, "comp_sy", e[1] & 15, 1);
        snprintf(name, sizeof(name), "decomposition levels (CDT byte 3) of component %d", c);
        REQUIRE(name, "comp_levels", e[2], supported_comp_levels[c]);
    }
    return 0;
}

static int parse_wgt(const uint8_t *b, int len, TicoRawHeader *h, void *logctx)
{
    int i;

    REQUIRE("WGT length", "Lwgt", len + 2, WGT_BYTES_PER_BAND * TICORAW_NB_BANDS + 2);

    h->nb_bands = TICORAW_NB_BANDS;
    for (i = 0; i < TICORAW_NB_BANDS; i++) {
        h->band_gain[i]     = b[WGT_BYTES_PER_BAND * i];
        h->band_priority[i] = b[WGT_BYTES_PER_BAND * i + 1];
    }
    return 0;
}

int ff_ticoraw_parse_header(const uint8_t *cs, int size, TicoRawHeader *h, void *logctx)
{
    Segment cap = { 0 }, pih = { 0 }, cdt = { 0 }, wgt = { 0 };
    int pos, ret;

    memset(h, 0, sizeof(*h));

    if (!cs || size < 4 || AV_RB16(cs) != JPEGXS_MARKER_SOC) {
        av_log(logctx, AV_LOG_ERROR, "Missing SOC marker at the codestream start\n");
        return AVERROR_INVALIDDATA;
    }

    /* Marker segments up to the first slice header. */
    pos = 2;
    for (;;) {
        Segment *seg = NULL;
        int marker, len;

        if (size - pos < 2) {
            av_log(logctx, AV_LOG_ERROR, "Codestream ends inside the header\n");
            return AVERROR_INVALIDDATA;
        }
        marker = AV_RB16(cs + pos);
        if (marker == JPEGXS_MARKER_SLH)
            break;
        if (marker == JPEGXS_MARKER_EOC) {
            av_log(logctx, AV_LOG_ERROR, "EOC marker before the first slice header\n");
            return AVERROR_INVALIDDATA;
        }
        if ((marker >> 8) != 0xFF || size - pos < 4) {
            av_log(logctx, AV_LOG_ERROR, "Invalid marker 0x%04X at offset %d\n", marker, pos);
            return AVERROR_INVALIDDATA;
        }
        len = AV_RB16(cs + pos + 2);
        if (len < 2 || len > size - pos - 2) {
            av_log(logctx, AV_LOG_ERROR,
                   "Invalid length %d of marker 0x%04X at offset %d\n", len, marker, pos);
            return AVERROR_INVALIDDATA;
        }

        switch (marker) {
        case JPEGXS_MARKER_CAP: seg = &cap; break;
        case JPEGXS_MARKER_PIH: seg = &pih; break;
        case JPEGXS_MARKER_CDT: seg = &cdt; break;
        case JPEGXS_MARKER_WGT: seg = &wgt; break;
        case JPEGXS_MARKER_COM: /* extension data: no effect on decoding */
            break;
        case JPEGXS_MARKER_NLT: /* these markers change the decoding process */
        case JPEGXS_MARKER_CWD:
        case JPEGXS_MARKER_CTS:
        case JPEGXS_MARKER_CRG:
        default:
            av_log(logctx, AV_LOG_ERROR, "Unsupported marker 0x%04X (L = %d) in the header\n",
                   marker, len);
            return AVERROR_PATCHWELCOME;
        }
        if (seg) {
            if (seg->body) {
                av_log(logctx, AV_LOG_ERROR, "Marker 0x%04X appears twice\n", marker);
                return AVERROR_INVALIDDATA;
            }
            seg->body = cs + pos + 4;
            seg->len  = len - 2;
        }
        pos += 2 + len;
    }

    /* The first slice header: L = 4, slice index 0. */
    if (size - pos < SLH_SIZE || AV_RB16(cs + pos + 2) != 4 || AV_RB16(cs + pos + 4) != 0) {
        av_log(logctx, AV_LOG_ERROR, "Invalid first slice header at offset %d\n", pos);
        return AVERROR_INVALIDDATA;
    }
    h->first_slice_offset = pos;

    if (!pih.body || !cdt.body || !wgt.body) {
        av_log(logctx, AV_LOG_ERROR, "Missing %s marker in the codestream header\n",
               !pih.body ? "PIH" : !cdt.body ? "CDT" : "WGT");
        return AVERROR_INVALIDDATA;
    }

    /* Lcod, the codestream length, must match the real size. */
    if (pih.len >= 4 && AV_RB32(pih.body) != (uint32_t)size) {
        av_log(logctx, AV_LOG_ERROR,
               "Codestream length Lcod %"PRIu32" differs from the codestream size %d\n",
               AV_RB32(pih.body), size);
        return AVERROR_INVALIDDATA;
    }
    if (AV_RB16(cs + size - 2) != JPEGXS_MARKER_EOC) {
        av_log(logctx, AV_LOG_ERROR, "Missing EOC marker at the codestream end\n");
        return AVERROR_INVALIDDATA;
    }

    /*
     * Star-Tetrix profile: a CAP marker with the vendor string, the PIH length 39,
     * Nc = 4, 3-byte CDT entries and Cpih = 0.
     */
    if (!cap.body || cap.len != CAP_BODY_SIZE ||
        memcmp(cap.body, cap_vendor_prefix, sizeof(cap_vendor_prefix) - 1)) {
        av_log(logctx, AV_LOG_ERROR,
               "Unsupported CAP marker (%s), only a %d byte body that starts with %s "
               "is supported\n",
               cap.body ? "other body" : "missing", CAP_BODY_SIZE, cap_vendor_prefix);
        return AVERROR_PATCHWELCOME;
    }

    if ((ret = parse_pih(pih.body, pih.len, h, logctx)) < 0)
        return ret;
    if ((ret = parse_cdt(cdt.body, cdt.len, h, logctx)) < 0)
        return ret;
    if ((ret = parse_wgt(wgt.body, wgt.len, h, logctx)) < 0)
        return ret;

    h->nb_precincts = (h->ch + 1) / 2;
    h->nb_slices    = (h->nb_precincts + h->slice_height - 1) / h->slice_height;

    /*
     * Every precinct has MIN_PRECINCT_SIZE bytes of headers, and at least one bit
     * for each group of 32 coefficients of the 8 * cw coefficients of its lines: the
     * significance flag, or more bits per group in the other modes. Half of that is
     * required here. A codestream that is shorter cannot hold the picture, so a
     * damaged size field cannot cause a large allocation (the planes need 16 bytes
     * per sample of the component size).
     */
    if ((int64_t)h->nb_precincts * (MIN_PRECINCT_SIZE + h->cw / 64) > size) {
        av_log(logctx, AV_LOG_ERROR,
               "Codestream of %d bytes is too short for %d precincts of %d samples\n",
               size, h->nb_precincts, h->cw);
        return AVERROR_INVALIDDATA;
    }
    return 0;
}

int ff_ticoraw_parse_frame(const uint8_t *buf, int size, TicoRawMeta *meta,
                           TicoRawHeader *h, int *cs_offset, int *cs_size, void *logctx)
{
    int ret = ff_ticoraw_parse_nraw(buf, size, meta, cs_offset, cs_size, logctx);

    if (ret < 0)
        return ret;
    ret = ff_ticoraw_parse_header(buf + *cs_offset, *cs_size, h, logctx);
    if (ret < 0)
        return ret;
    if (meta->width != h->width || meta->height != h->height) {
        av_log(logctx, AV_LOG_ERROR,
               "Wrapper size %dx%d differs from the picture header size %dx%d\n",
               meta->width, meta->height, h->width, h->height);
        return AVERROR_INVALIDDATA;
    }
    return 0;
}
