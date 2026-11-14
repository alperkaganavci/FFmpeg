/*
 * TICO RAW (Nikon N-RAW, R3D NE) frame wrapper parser
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
#include <limits.h>
#include <math.h>
#include <string.h>

#include "libavutil/common.h"
#include "libavutil/error.h"
#include "libavutil/intfloat.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"

#include "ticoraw_nraw.h"

#define BOX_HEADER_SIZE 8   /* u32 size, fourcc */
#define NRMT_HEADER_SIZE 12 /* box header, u32 key */
#define NRMT_VALUE_LEAD 1   /* leading byte of unknown meaning before the payload */

#define NRMT_KEY(group, tag) (((uint32_t)(group) << 16) | (tag))
#define KEY_WIDTH     NRMT_KEY(0x0110, 0x0100)
#define KEY_HEIGHT    NRMT_KEY(0x0110, 0x0101)
#define KEY_BITS      NRMT_KEY(0x0110, 0x0102)
#define KEY_ISO       NRMT_KEY(0x0110, 0x8832)
#define KEY_CFA       NRMT_KEY(0x0110, 0xA302)
#define KEY_WB        NRMT_KEY(0x0190, 0x0009)
#define KEY_CAM2XYZ   NRMT_KEY(0x0190, 0x0012)

#define MAX_DIMENSION 65535 /* Wf and Hf are 16-bit fields in the codestream */
#define SUPPORTED_BITS 14

enum {
    SEEN_WIDTH   = 1 << 0,
    SEEN_HEIGHT  = 1 << 1,
    SEEN_ISO     = 1 << 2,
    SEEN_CFA     = 1 << 3,
    SEEN_WB      = 1 << 4,
    SEEN_CAM2XYZ = 1 << 5,
    SEEN_ALL     = (1 << 6) - 1,
};

static const char *const missing_names[] = {
    "width (0x01100100)", "height (0x01100101)", "iso (0x01108832)",
    "cfa (0x0110A302)", "wb_gain (0x01900009)", "cam_to_xyz (0x01900012)",
};

static int read_float(const uint8_t *p, double *out)
{
    float f = av_int2float(AV_RB32(p));

    if (!isfinite(f))
        return AVERROR_INVALIDDATA;
    *out = f;
    return 0;
}

static int bad_length(void *logctx, uint32_t key, int len, int want)
{
    av_log(logctx, AV_LOG_ERROR,
           "Wrapper tag 0x%08"PRIX32" has a %d byte payload, expected %d\n", key, len, want);
    return AVERROR_INVALIDDATA;
}

static int bad_float(void *logctx, uint32_t key)
{
    av_log(logctx, AV_LOG_ERROR, "Wrapper tag 0x%08"PRIX32" holds a non-finite number\n", key);
    return AVERROR_INVALIDDATA;
}

/* One NRMT tag. payload points after the leading byte, len is its length. */
static int parse_tag(uint32_t key, const uint8_t *payload, int len,
                     TicoRawMeta *meta, unsigned *seen, void *logctx)
{
    int i;

    switch (key) {
    case KEY_WIDTH:
    case KEY_HEIGHT: {
        uint32_t v;

        if (len != 4)
            return bad_length(logctx, key, len, 4);
        v = AV_RB32(payload);
        if (v < 2 || v > MAX_DIMENSION) {
            av_log(logctx, AV_LOG_ERROR, "Invalid coded %s %"PRIu32"\n",
                   key == KEY_WIDTH ? "width" : "height", v);
            return AVERROR_INVALIDDATA;
        }
        if (v & 1) { /* the codestream codes one sample per 2x2 quad: size / 2 */
            av_log(logctx, AV_LOG_ERROR,
                   "Unsupported coded %s %"PRIu32", only even sizes are supported\n",
                   key == KEY_WIDTH ? "width" : "height", v);
            return AVERROR_PATCHWELCOME;
        }
        if (key == KEY_WIDTH) {
            meta->width = v;
            *seen |= SEEN_WIDTH;
        } else {
            meta->height = v;
            *seen |= SEEN_HEIGHT;
        }
        break;
    }
    case KEY_BITS:
        if (len != 2)
            return bad_length(logctx, key, len, 2);
        if (AV_RB16(payload) != SUPPORTED_BITS) {
            av_log(logctx, AV_LOG_ERROR,
                   "Unsupported bits per sample %d, only %d is supported\n",
                   AV_RB16(payload), SUPPORTED_BITS);
            return AVERROR_PATCHWELCOME;
        }
        break;
    case KEY_ISO: {
        uint32_t v;

        if (len != 4)
            return bad_length(logctx, key, len, 4);
        v = AV_RB32(payload);
        if (v > INT_MAX) {
            av_log(logctx, AV_LOG_ERROR, "Invalid ISO value %"PRIu32"\n", v);
            return AVERROR_INVALIDDATA;
        }
        meta->iso = v;
        *seen |= SEEN_ISO;
        break;
    }
    case KEY_CFA: {
        int cols, rows, n_r = 0, n_g = 0, n_b = 0;

        if (len < 4)
            return bad_length(logctx, key, len, 4);
        cols = AV_RB16(payload);
        rows = AV_RB16(payload + 2);
        if (cols != 2 || rows != 2) {
            av_log(logctx, AV_LOG_ERROR,
                   "Unsupported CFA pattern size %dx%d, only 2x2 is supported\n",
                   cols, rows);
            return AVERROR_PATCHWELCOME;
        }
        if (len != 4 + cols * rows)
            return bad_length(logctx, key, len, 4 + cols * rows);
        for (i = 0; i < 4; i++) {
            switch (payload[4 + i]) {
            case 0: meta->cfa[i] = 'R'; n_r++; break;
            case 1: meta->cfa[i] = 'G'; n_g++; break;
            case 2: meta->cfa[i] = 'B'; n_b++; break;
            default:
                av_log(logctx, AV_LOG_ERROR, "Unsupported CFA colour code %d\n",
                       payload[4 + i]);
                return AVERROR_PATCHWELCOME;
            }
        }
        meta->cfa[4] = '\0';
        if (n_r != 1 || n_g != 2 || n_b != 1) {
            av_log(logctx, AV_LOG_ERROR, "Unsupported CFA pattern %s\n", meta->cfa);
            return AVERROR_PATCHWELCOME;
        }
        *seen |= SEEN_CFA;
        break;
    }
    case KEY_WB: {
        double r, b;

        if (len != 8)
            return bad_length(logctx, key, len, 8);
        if (read_float(payload, &r) < 0 || read_float(payload + 4, &b) < 0)
            return bad_float(logctx, key);
        meta->wb_gain[0] = r;
        meta->wb_gain[1] = 1.0;
        meta->wb_gain[2] = b;
        *seen |= SEEN_WB;
        break;
    }
    case KEY_CAM2XYZ:
        if (len != 36)
            return bad_length(logctx, key, len, 36);
        for (i = 0; i < 9; i++)
            if (read_float(payload + 4 * i, &meta->cam_to_xyz[i]) < 0)
                return bad_float(logctx, key);
        *seen |= SEEN_CAM2XYZ;
        break;
    default: /* tags that feed no output */
        break;
    }
    return 0;
}

/* The version string of the NRFM, NRHM and NRTI boxes is "0100" (format note section 2). */
static int check_version(const uint8_t *version, const char *box, void *logctx)
{
    char text[5];
    int i;

    if (!memcmp(version, "0100", 4))
        return 0;
    for (i = 0; i < 4; i++)   /* keep control bytes out of the log */
        text[i] = version[i] >= 0x20 && version[i] < 0x7f ? version[i] : '?';
    text[4] = 0;
    av_log(logctx, AV_LOG_ERROR, "Unsupported version \"%s\" in the %s box, only \"0100\" is known\n",
           text, box);
    return AVERROR_PATCHWELCOME;
}

/* NRFH payload: NRHM box, then NRMT boxes. */
static int parse_frame_header(const uint8_t *p, int size, TicoRawMeta *meta,
                              unsigned *seen, void *logctx)
{
    int pos = 0, ret;

    while (pos < size) {
        uint32_t box_size;

        if (size - pos < BOX_HEADER_SIZE) {
            av_log(logctx, AV_LOG_ERROR, "Truncated box in the NRFH box\n");
            return AVERROR_INVALIDDATA;
        }
        box_size = AV_RB32(p + pos);
        if (box_size < BOX_HEADER_SIZE || box_size > size - pos) {
            av_log(logctx, AV_LOG_ERROR, "Invalid box size %"PRIu32" in the NRFH box\n", box_size);
            return AVERROR_INVALIDDATA;
        }
        if (AV_RL32(p + pos + 4) == MKTAG('N', 'R', 'H', 'M')) {
            if (box_size != 12) {
                av_log(logctx, AV_LOG_ERROR, "NRHM box has %"PRIu32" bytes, expected 12\n",
                       box_size);
                return AVERROR_INVALIDDATA;
            }
            ret = check_version(p + pos + BOX_HEADER_SIZE, "NRHM", logctx);
            if (ret < 0)
                return ret;
        } else if (AV_RL32(p + pos + 4) == MKTAG('N', 'R', 'M', 'T')) {
            if (box_size < NRMT_HEADER_SIZE + NRMT_VALUE_LEAD) {
                av_log(logctx, AV_LOG_ERROR, "NRMT box of %"PRIu32" bytes is too short\n",
                       box_size);
                return AVERROR_INVALIDDATA;
            }
            ret = parse_tag(AV_RB32(p + pos + 8),
                            p + pos + NRMT_HEADER_SIZE + NRMT_VALUE_LEAD,
                            box_size - NRMT_HEADER_SIZE - NRMT_VALUE_LEAD,
                            meta, seen, logctx);
            if (ret < 0)
                return ret;
        }
        pos += box_size;
    }
    return 0;
}

/* NRTH payload: NRTI box (version "0100", u32 codestream length), then NRTA padding. */
static int parse_trailer_header(const uint8_t *p, int size, uint32_t *cs_len, void *logctx)
{
    int pos = 0, ret;

    while (size - pos >= BOX_HEADER_SIZE) {
        uint32_t box_size = AV_RB32(p + pos);

        if (box_size < BOX_HEADER_SIZE || box_size > size - pos) {
            av_log(logctx, AV_LOG_ERROR, "Invalid box size %"PRIu32" in the NRTH box\n", box_size);
            return AVERROR_INVALIDDATA;
        }
        if (AV_RL32(p + pos + 4) == MKTAG('N', 'R', 'T', 'I')) {
            if (box_size != 16) {
                av_log(logctx, AV_LOG_ERROR, "NRTI box has %"PRIu32" bytes, expected 16\n",
                       box_size);
                return AVERROR_INVALIDDATA;
            }
            if ((ret = check_version(p + pos + BOX_HEADER_SIZE, "NRTI", logctx)) < 0)
                return ret;
            *cs_len = AV_RB32(p + pos + 12);
            return 0;
        }
        pos += box_size;
    }
    av_log(logctx, AV_LOG_ERROR, "Missing NRTI box in the NRTH box\n");
    return AVERROR_INVALIDDATA;
}

int ff_ticoraw_parse_nraw(const uint8_t *buf, int size, TicoRawMeta *meta,
                          int *cs_offset, int *cs_size, void *logctx)
{
    uint32_t nraw_size, cs_len = 0;
    unsigned seen = 0;
    int pos, i, ret, have_header = 0, have_trailer = 0;

    if (!buf || size < BOX_HEADER_SIZE) {
        av_log(logctx, AV_LOG_ERROR, "Frame of %d bytes is too short for the NRAW box\n", size);
        return AVERROR_INVALIDDATA;
    }

    nraw_size = AV_RB32(buf);
    if (AV_RL32(buf + 4) != MKTAG('N', 'R', 'A', 'W')) {
        av_log(logctx, AV_LOG_ERROR, "Missing NRAW box at the start of the frame\n");
        return AVERROR_INVALIDDATA;
    }
    if (nraw_size != size) {
        av_log(logctx, AV_LOG_ERROR,
               "NRAW box size %"PRIu32" differs from the frame size %d\n", nraw_size, size);
        return AVERROR_INVALIDDATA;
    }

    memset(meta, 0, sizeof(*meta));

    /* The child boxes follow each other from byte 8; NRTH is the last one. */
    pos = BOX_HEADER_SIZE;
    while (!have_trailer) {
        uint32_t box_size;
        uint32_t tag;

        if (size - pos < BOX_HEADER_SIZE) {
            av_log(logctx, AV_LOG_ERROR, "Frame ends before the NRTH box (offset %d)\n", pos);
            return AVERROR_INVALIDDATA;
        }
        box_size = AV_RB32(buf + pos);
        tag      = AV_RL32(buf + pos + 4);
        if (box_size < BOX_HEADER_SIZE || box_size > size - pos) {
            av_log(logctx, AV_LOG_ERROR, "Invalid wrapper box size %"PRIu32" at offset %d\n",
                   box_size, pos);
            return AVERROR_INVALIDDATA;
        }
        if (tag == MKTAG('N', 'R', 'F', 'M')) {
            if (box_size != 12) {
                av_log(logctx, AV_LOG_ERROR, "NRFM box has %"PRIu32" bytes, expected 12\n",
                       box_size);
                return AVERROR_INVALIDDATA;
            }
            ret = check_version(buf + pos + BOX_HEADER_SIZE, "NRFM", logctx);
            if (ret < 0)
                return ret;
        } else if (tag == MKTAG('N', 'R', 'F', 'H')) {
            ret = parse_frame_header(buf + pos + BOX_HEADER_SIZE,
                                     box_size - BOX_HEADER_SIZE, meta, &seen, logctx);
            if (ret < 0)
                return ret;
            have_header = 1;
        } else if (tag == MKTAG('N', 'R', 'T', 'H')) {
            ret = parse_trailer_header(buf + pos + BOX_HEADER_SIZE,
                                       box_size - BOX_HEADER_SIZE, &cs_len, logctx);
            if (ret < 0)
                return ret;
            have_trailer = 1;
        }
        pos += box_size;
    }

    if (!have_header) {
        av_log(logctx, AV_LOG_ERROR, "Missing NRFH box in the frame wrapper\n");
        return AVERROR_INVALIDDATA;
    }
    if (seen != SEEN_ALL) {
        for (i = 0; i < FF_ARRAY_ELEMS(missing_names); i++)
            if (!(seen & (1 << i)))
                av_log(logctx, AV_LOG_ERROR, "Missing wrapper tag %s\n", missing_names[i]);
        return AVERROR_INVALIDDATA;
    }

    /* The codestream runs to the end of the sample, and NRTI gives its exact length. */
    if (cs_len < 4 || cs_len != size - pos) {
        av_log(logctx, AV_LOG_ERROR,
               "NRTI codestream length %"PRIu32" differs from the %d bytes after the wrapper\n",
               cs_len, size - pos);
        return AVERROR_INVALIDDATA;
    }
    if (buf[pos] != 0xFF || buf[pos + 1] != 0x10) {
        av_log(logctx, AV_LOG_ERROR, "Missing SOC marker at the codestream start (offset %d)\n", pos);
        return AVERROR_INVALIDDATA;
    }

    meta->white_level = (1 << SUPPORTED_BITS) - 1;
    *cs_offset = pos;
    *cs_size   = cs_len;
    return 0;
}
