/*
 * TICO RAW (Nikon N-RAW, R3D NE) test program
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
 * Usage: ticoraw <sample.bin> <stage> <out>
 *
 * Runs one decoder stage on a frame sample (Nikon wrapper + codestream) and writes the
 * result to <out>. Stages:
 *   meta    JSON in the frame JSON format: width, height, cw, ch and the meta section
 *   header  the same JSON with the header section (PIH and CDT fields) added
 *   coeffs, planes_st, cfa_companded
 *           the decoding stage of the format note 10.2: four int32 little-endian planes
 *           of cw x ch samples, component order 0 to 3, no header (note 10.1)
 *   bayer_linear
 *           the linear sensor mosaic of the format note 10.7: width x height uint16
 *           little-endian samples, row-major, no header
 *   view_example
 *           the worked example of the format note 12.10: the colour steps of the Rec.709
 *           view (R3D NE, default settings, black level, gains and matrix of the sample)
 *           on three neutral normalised inputs, as text lines "<name> <R> <G> <B>" with
 *           16-bit codes. The codestream is not decoded.
 *   exposure_nan
 *           opens the tico_raw decoder with view=rec709 and the exposure option set to NaN
 *           through av_opt_set_double() (the option range does not stop a NaN, and option
 *           strings cannot carry one), and writes the line "avcodec_open2 <ret>", where
 *           <ret> is the return value. The decoder must refuse it with AVERROR(EINVAL).
 *           The sample is only parsed.
 *
 * On a damaged or unsupported stream the program prints the reason on stderr, writes no
 * output file and exits with status 1.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "libavutil/error.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/macros.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "libavcodec/avcodec.h"
#include "libavcodec/defs.h"
#include "libavcodec/ticoraw_core.h"
#include "libavcodec/ticoraw_header.h"
#include "libavcodec/ticoraw_nraw.h"
#include "libavcodec/ticoraw_view.h"

static uint8_t *read_file(const char *path, int *size)
{
    FILE *f = fopen(path, "rb");
    uint8_t *buf = NULL;
    long len;

    if (!f)
        return NULL;
    if (fseek(f, 0, SEEK_END) < 0 || (len = ftell(f)) <= 0 || len > INT32_MAX ||
        fseek(f, 0, SEEK_SET) < 0)
        goto end;
    /* zeroed input padding, as in an AVPacket: the bit readers may look past the end */
    buf = av_mallocz(len + AV_INPUT_BUFFER_PADDING_SIZE);
    if (buf && fread(buf, 1, len, f) != (size_t)len) {
        av_freep(&buf);
        goto end;
    }
    *size = len;
end:
    fclose(f);
    return buf;
}

/* Shortest decimal that parses back to the same double. */
static void print_double(FILE *out, double v)
{
    char s[40];
    int prec;

    for (prec = 1; prec <= 17; prec++) {
        snprintf(s, sizeof(s), "%.*g", prec, v);
        if (strtod(s, NULL) == v)
            break;
    }
    if (!strpbrk(s, ".eE"))
        strcat(s, ".0");
    fputs(s, out);
}

static void print_int_list(FILE *out, const uint8_t *v, int n)
{
    int i;

    fputc('[', out);
    for (i = 0; i < n; i++)
        fprintf(out, "%s%d", i ? ", " : "", v[i]);
    fputc(']', out);
}

/* The header section of the frame JSON: the keys of the format note 10.8. */
static void write_header(FILE *out, const TicoRawHeader *h)
{
    fprintf(out, "  \"header\": {\n    \"width\": %d,\n    \"height\": %d,\n"
            "    \"nc\": %d,\n    \"ng\": %d,\n    \"ss\": %d,\n    \"bw\": %d,\n"
            "    \"fq\": %d,\n    \"br\": %d,\n    \"cpih\": %d,\n    \"nlx\": %d,\n"
            "    \"nly\": %d,\n    \"slice_height\": %d,\n    \"comp_depth\": ",
            h->width, h->height, h->nc, h->ng, h->ss, h->bw, h->fq, h->br, h->cpih,
            h->nlx, h->nly, h->slice_height);
    print_int_list(out, h->comp_depth, TICORAW_NB_COMPONENTS);
    fputs(",\n    \"comp_sx\": ", out);
    print_int_list(out, h->comp_sx, TICORAW_NB_COMPONENTS);
    fputs(",\n    \"comp_sy\": ", out);
    print_int_list(out, h->comp_sy, TICORAW_NB_COMPONENTS);
    fputs("\n  }\n", out);
}

/* h is NULL for the meta stage. */
static int write_json(FILE *out, const TicoRawMeta *m, const TicoRawHeader *h)
{
    int i;

    fprintf(out, "{\n  \"width\": %d,\n  \"height\": %d,\n  \"cw\": %d,\n  \"ch\": %d,\n",
            m->width, m->height, m->width / 2, m->height / 2);
    fprintf(out, "  \"meta\": {\n    \"cfa\": \"%s\",\n    \"wb_gain\": [", m->cfa);
    for (i = 0; i < 3; i++) {
        if (i)
            fputs(", ", out);
        print_double(out, m->wb_gain[i]);
    }
    fputs("],\n    \"cam_to_xyz\": [", out);
    for (i = 0; i < 9; i++) {
        if (i)
            fputs(", ", out);
        print_double(out, m->cam_to_xyz[i]);
    }
    fprintf(out, "],\n    \"iso\": %d,\n    \"white_level\": %d\n  }%s\n",
            m->iso, m->white_level, h ? "," : "");
    if (h)
        write_header(out, h);
    fputs("}\n", out);
    return ferror(out) ? -1 : 0;
}

static const struct {
    const char *name;
    enum TicoRawStage stage;
} plane_stages[] = {
    { "coeffs",        TICORAW_STAGE_COEFFS    },
    { "planes_st",     TICORAW_STAGE_PLANES_ST },
    { "cfa_companded", TICORAW_STAGE_CFA       },
    { "bayer_linear",  TICORAW_STAGE_CFA       }, /* then the curve and mosaic of 10.7 */
};

/* Note 10.1: component 0 first, each plane row-major, int32 little-endian. */
static int write_planes(FILE *out, const TicoRawPlanes *pl)
{
    uint8_t *row = av_malloc_array(pl->cw, 4);
    int c, x, y, ret = 0;

    if (!row)
        return -1;
    for (c = 0; c < 4 && !ret; c++)
        for (y = 0; y < pl->ch && !ret; y++) {
            const int32_t *src = pl->p[c] + y * pl->stride;
            for (x = 0; x < pl->cw; x++)
                AV_WL32(row + 4 * x, src[x]);
            if (fwrite(row, 4, pl->cw, out) != (size_t)pl->cw)
                ret = -1;
        }
    av_free(row);
    return ret;
}

/* Note 10.7: width x height uint16 little-endian, row-major. */
static int write_bayer(FILE *out, const TicoRawPlanes *pl, const TicoRawHeader *h)
{
    uint16_t *lut  = av_malloc_array(TICORAW_CURVE_SIZE, sizeof(*lut));
    uint16_t *rows = av_malloc_array(2 * h->width, sizeof(*rows));
    uint8_t *le    = av_malloc_array(2 * h->width, 2);
    int x, y, ret = 0;

    if (!lut || !rows || !le) {
        ret = -1;
        goto end;
    }
    ff_ticoraw_build_curve(lut, h);
    for (y = 0; y < pl->ch && !ret; y++) {
        /* one quad row gives two mosaic rows */
        ff_ticoraw_planes_to_bayer(rows, h->width, pl, lut, y, y + 1);
        for (x = 0; x < 2 * h->width; x++)
            AV_WL16(le + 2 * x, rows[x]);
        if (fwrite(le, 2, 2 * h->width, out) != 2 * (size_t)h->width)
            ret = -1;
    }
end:
    av_free(lut);
    av_free(rows);
    av_free(le);
    return ret;
}

/*
 * Format note 12.10: the white-balanced normalised value N of each column (mid-grey is
 * 0.18 / k), given to the colour steps as camera RGB (N / gain R, N, N / gain B).
 */
static int write_view_example(FILE *out, const TicoRawMeta *m, const TicoRawHeader *h)
{
    static const struct {
        const char *name;
        float n;
    } cols[] = { { "midgrey", 0.0220971f }, { "grey018", 0.18f }, { "white", 1.0f } };
    TicoRawViewParams p = {
        .black_level = h->black_level,
        .white_level = m->white_level,
        .size        = TICORAW_VIEW_HALF,
        .nraw        = 0,
    };
    int i;

    memcpy(p.wb_gain, m->wb_gain, sizeof(p.wb_gain));
    memcpy(p.cam_to_xyz, m->cam_to_xyz, sizeof(p.cam_to_xyz));
    for (i = 0; i < FF_ARRAY_ELEMS(cols); i++) {
        const float cam[3] = { cols[i].n / m->wb_gain[0], cols[i].n, cols[i].n / m->wb_gain[2] };
        uint16_t rgb[3];

        ff_ticoraw_view_pixel(&p, cam, rgb);
        if (fprintf(out, "%s %d %d %d\n", cols[i].name, rgb[0], rgb[1], rgb[2]) < 0)
            return 1;
    }
    return 0;
}

/* Stage exposure_nan: the return value of avcodec_open2() with a NaN exposure. */
static int write_exposure_nan(FILE *out)
{
    const AVCodec *codec = avcodec_find_decoder_by_name("tico_raw");
    AVCodecContext *dec  = codec ? avcodec_alloc_context3(codec) : NULL;
    int ret;

    if (!dec)
        return 1;
    ret = av_opt_set(dec, "view", "rec709", AV_OPT_SEARCH_CHILDREN);
    if (ret >= 0)
        ret = av_opt_set_double(dec, "exposure", NAN, AV_OPT_SEARCH_CHILDREN);
    if (ret < 0) {
        fprintf(stderr, "cannot set the options: %s\n", av_err2str(ret));
        avcodec_free_context(&dec);
        return 1;
    }
    ret = avcodec_open2(dec, codec, NULL);
    avcodec_free_context(&dec);
    return fprintf(out, "avcodec_open2 %d\n", ret) < 0;
}

int main(int argc, char **argv)
{
    TicoRawMeta meta;
    TicoRawHeader hdr;
    AVCodecContext *avctx = NULL;
    TicoRawCore *core = NULL;
    TicoRawPlanes planes;
    uint8_t *buf;
    int size, cs_offset, cs_size, err, ret = 1, stage = -1, bayer, i;
    FILE *out;

    if (argc != 4) {
        fprintf(stderr, "usage: %s <sample.bin> <stage> <out>\n"
                "stages: meta header coeffs planes_st cfa_companded bayer_linear "
                "view_example exposure_nan\n", argv[0]);
        return 2;
    }
    for (i = 0; i < FF_ARRAY_ELEMS(plane_stages); i++)
        if (!strcmp(argv[2], plane_stages[i].name))
            stage = plane_stages[i].stage;
    bayer = !strcmp(argv[2], "bayer_linear");
    if (stage < 0 && strcmp(argv[2], "meta") && strcmp(argv[2], "header") &&
        strcmp(argv[2], "view_example") && strcmp(argv[2], "exposure_nan")) {
        fprintf(stderr, "unknown stage '%s'\n", argv[2]);
        return 2;
    }

    buf = read_file(argv[1], &size);
    if (!buf) {
        fprintf(stderr, "cannot read %s\n", argv[1]);
        return 1;
    }
    err = ff_ticoraw_parse_frame(buf, size, &meta, &hdr, &cs_offset, &cs_size, NULL);
    if (err < 0) {
        fprintf(stderr, "cannot parse %s: %s\n", argv[1], av_err2str(err));
        goto end;
    }
    /* The mosaic placement of note 10.6 is known for RGGB only (note 11.2). */
    if (bayer && strcmp(meta.cfa, "RGGB")) {
        fprintf(stderr, "Unsupported CFA pattern (cfa) %s, only RGGB is supported: %s\n",
                meta.cfa, av_err2str(AVERROR_PATCHWELCOME));
        goto end;
    }
    if (stage >= 0) {
        /* a default context: no slice threads, jobs run one after the other */
        avctx = avcodec_alloc_context3(NULL);
        err   = avctx ? ff_ticoraw_core_init(&core, &hdr, avctx) : AVERROR(ENOMEM);
        if (err >= 0)
            err = ff_ticoraw_decode_stage(core, buf + cs_offset, cs_size, &hdr, stage, &planes);
        if (err < 0) {
            fprintf(stderr, "cannot decode %s: %s\n", argv[1], av_err2str(err));
            goto end;
        }
    }

    out = fopen(argv[3], "wb");
    if (!out) {
        fprintf(stderr, "cannot write %s\n", argv[3]);
        goto end;
    }
    if (bayer)
        ret = write_bayer(out, &planes, &hdr);
    else if (stage >= 0)
        ret = write_planes(out, &planes);
    else if (!strcmp(argv[2], "view_example"))
        ret = write_view_example(out, &meta, &hdr);
    else if (!strcmp(argv[2], "exposure_nan"))
        ret = write_exposure_nan(out);
    else
        ret = write_json(out, &meta, strcmp(argv[2], "header") ? NULL : &hdr);
    if (fclose(out))
        ret = 1;
    if (ret)
        fprintf(stderr, "write error on %s\n", argv[3]);
end:
    ff_ticoraw_core_free(&core);
    avcodec_free_context(&avctx);
    av_free(buf);
    return ret ? 1 : 0;
}
