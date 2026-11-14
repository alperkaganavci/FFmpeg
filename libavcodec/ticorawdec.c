/*
 * TICO RAW (Nikon N-RAW, R3D NE) decoder
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

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "libavutil/dict.h"
#include "libavutil/eval.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixfmt.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "thread.h"
#include "ticoraw_core.h"
#include "ticoraw_header.h"
#include "ticoraw_nraw.h"
#include "ticoraw_view.h"

enum TicoRawView {
    TICORAW_VIEW_BAYER,     /* linear RGGB mosaic, the default */
    TICORAW_VIEW_REC709,    /* Rec.709 RGB viewing picture (ticoraw_view.h) */
};

typedef struct TicoRawContext {
    const AVClass *class;
    int view;               /* enum TicoRawView */
    int view_size;          /* enum TicoRawViewSize */
    double exposure;        /* EV offset of the rec709 view */

    TicoRawCore *core;      /* per decoder context, so one per frame thread; it runs its
                             * jobs on the slice threads of that context */
    uint16_t *curve;        /* TICORAW_CURVE_SIZE entries, built for the parameters below */
    int curve_black_level, curve_slope, curve_pivot;
    TicoRawViewContext view_ctx;    /* tables of the rec709 view */
} TicoRawContext;

/*
 * The plane-to-mosaic placement of the format note 10.6 is known for RGGB only
 * (note 11.2), so only that pattern is output.
 */
static enum AVPixelFormat cfa_to_pix_fmt(const char *cfa)
{
    if (!strcmp(cfa, "RGGB"))
        return AV_PIX_FMT_BAYER_RGGB16;
    return AV_PIX_FMT_NONE;
}

static int update_curve(AVCodecContext *avctx, const TicoRawHeader *h)
{
    TicoRawContext *s = avctx->priv_data;

    if (s->curve && s->curve_black_level == h->black_level &&
        s->curve_slope == h->curve_slope && s->curve_pivot == h->curve_pivot)
        return 0;
    if (!s->curve) {
        s->curve = av_malloc_array(TICORAW_CURVE_SIZE, sizeof(*s->curve));
        if (!s->curve)
            return AVERROR(ENOMEM);
    }
    ff_ticoraw_build_curve(s->curve, h);
    s->curve_black_level = h->black_level;
    s->curve_slope       = h->curve_slope;
    s->curve_pivot       = h->curve_pivot;
    return 0;
}

/* Rows of the companded planes to write as mosaic rows, split into row jobs. */
typedef struct BayerJobs {
    uint16_t *dst;
    ptrdiff_t dst_stride;   /* in uint16_t elements */
    const TicoRawPlanes *planes;
    const uint16_t *lut;
    int nb_jobs;
} BayerJobs;

static int bayer_job(AVCodecContext *avctx, void *arg, int job, int threadnr)
{
    const BayerJobs *b = arg;
    const int y0 = (int64_t)b->planes->ch * job       / b->nb_jobs;
    const int y1 = (int64_t)b->planes->ch * (job + 1) / b->nb_jobs;

    ff_ticoraw_planes_to_bayer(b->dst + 2 * y0 * b->dst_stride, b->dst_stride,
                               b->planes, b->lut, y0, y1);
    return 0;
}

/* Shortest decimal that parses back to the same double, as in the frame JSON (note 10.8). */
static void format_double(char *buf, int size, double v)
{
    int prec;

    for (prec = 1; prec < 17; prec++) {
        snprintf(buf, size, "%.*g", prec, v);
        if (av_strtod(buf, NULL) == v)
            return;
    }
    snprintf(buf, size, "%.17g", v);
}

static int set_list(AVDictionary **dict, const char *key, const double *v, int n)
{
    char buf[9 * 32];
    int i, len = 0;

    for (i = 0; i < n; i++) {
        if (i)
            buf[len++] = ' ';
        format_double(buf + len, sizeof(buf) - len, v[i]);
        len += strlen(buf + len);
    }
    return av_dict_set(dict, key, buf, 0);
}

/* The frame metadata keys read by the view code and by players. */
static int set_metadata(AVFrame *frame, const TicoRawMeta *meta, int black_level)
{
    int ret;

    if ((ret = av_dict_set(&frame->metadata, "ticoraw.cfa", meta->cfa, 0)) < 0 ||
        (ret = set_list(&frame->metadata, "ticoraw.wb_gain", meta->wb_gain, 3)) < 0 ||
        (ret = set_list(&frame->metadata, "ticoraw.cam_to_xyz", meta->cam_to_xyz, 9)) < 0 ||
        (ret = av_dict_set_int(&frame->metadata, "ticoraw.iso", meta->iso, 0)) < 0 ||
        (ret = av_dict_set_int(&frame->metadata, "ticoraw.white_level",
                               meta->white_level, 0)) < 0 ||
        (ret = av_dict_set_int(&frame->metadata, "ticoraw.black_level",
                               black_level, 0)) < 0)
        return ret;
    return 0;
}

static int tico_raw_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                                 int *got_frame, AVPacket *avpkt)
{
    TicoRawContext *s = avctx->priv_data;
    TicoRawMeta meta;
    TicoRawHeader hdr;
    TicoRawPlanes planes;
    BayerJobs bayer;
    enum AVPixelFormat pix_fmt;
    int cs_offset, cs_size, out_w, out_h, ret;

    /* Nothing is allocated before the wrapper and the header are parsed and checked. */
    ret = ff_ticoraw_parse_frame(avpkt->data, avpkt->size, &meta, &hdr,
                                 &cs_offset, &cs_size, avctx);
    if (ret < 0)
        return ret;

    pix_fmt = cfa_to_pix_fmt(meta.cfa);
    if (pix_fmt == AV_PIX_FMT_NONE) {
        av_log(avctx, AV_LOG_ERROR,
               "Unsupported CFA pattern (cfa) %s, only RGGB is supported\n", meta.cfa);
        return AVERROR_PATCHWELCOME;
    }

    /* The size is checked against max_pixels before the core allocates anything. */
    ret = av_image_check_size2(hdr.width, hdr.height, avctx->max_pixels,
                               AV_PIX_FMT_NONE, 0, avctx);
    if (ret < 0)
        return ret;
    out_w = hdr.width;
    out_h = hdr.height;
    if (s->view == TICORAW_VIEW_REC709) {
        pix_fmt = AV_PIX_FMT_GBRP16;
        if (s->view_size == TICORAW_VIEW_HALF) {
            out_w = hdr.cw;
            out_h = hdr.ch;
        }
        avctx->color_primaries = AVCOL_PRI_BT709;
        avctx->color_trc       = AVCOL_TRC_BT709;
        avctx->colorspace      = AVCOL_SPC_RGB;
        avctx->color_range     = AVCOL_RANGE_JPEG;
    }
    if (out_w != avctx->width || out_h != avctx->height) {
        ret = ff_set_dimensions(avctx, out_w, out_h);
        if (ret < 0)
            return ret;
    }
    avctx->pix_fmt = pix_fmt;

    if (!s->core) {
        ret = ff_ticoraw_core_init(&s->core, &hdr, avctx);
        if (ret < 0)
            return ret;
    }
    ret = update_curve(avctx, &hdr);
    if (ret < 0)
        return ret;

    /*
     * Every frame is intra coded and needs nothing from the frames before it. With no
     * update_thread_context, frame threading has already let the next thread start.
     */
    ret = ff_thread_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    ret = ff_ticoraw_decode_planes(s->core, avpkt->data + cs_offset, cs_size, &hdr, &planes);
    if (ret < 0)
        return ret;

    if (s->view == TICORAW_VIEW_REC709) {
        const TicoRawViewParams view = {
            .black_level = hdr.black_level,
            .white_level = meta.white_level,
            .wb_gain     = { meta.wb_gain[0], meta.wb_gain[1], meta.wb_gain[2] },
            .cam_to_xyz  = { meta.cam_to_xyz[0], meta.cam_to_xyz[1], meta.cam_to_xyz[2],
                             meta.cam_to_xyz[3], meta.cam_to_xyz[4], meta.cam_to_xyz[5],
                             meta.cam_to_xyz[6], meta.cam_to_xyz[7], meta.cam_to_xyz[8] },
            .exposure    = s->exposure,
            .size        = s->view_size,
            /* the sample entry fourcc tells N-RAW (NRAW) from R3D NE (NR3D), note 1 */
            .nraw        = avctx->codec_tag == MKTAG('N', 'R', 'A', 'W'),
        };

        ret = ff_ticoraw_render_view(&s->view_ctx, &planes, s->curve, &view, frame, avctx);
        if (ret < 0)
            return ret;
        frame->color_primaries = AVCOL_PRI_BT709;
        frame->color_trc       = AVCOL_TRC_BT709;
        frame->colorspace      = AVCOL_SPC_RGB;
        frame->color_range     = AVCOL_RANGE_JPEG;
    } else {
        bayer.dst        = (uint16_t *)frame->data[0];
        bayer.dst_stride = frame->linesize[0] / 2;
        bayer.planes     = &planes;
        bayer.lut        = s->curve;
        bayer.nb_jobs    = ff_ticoraw_nb_row_jobs(avctx, planes.ch);
        avctx->execute2(avctx, bayer_job, &bayer, NULL, bayer.nb_jobs);
    }

    ret = set_metadata(frame, &meta, hdr.black_level);
    if (ret < 0)
        return ret;

    frame->pict_type = AV_PICTURE_TYPE_I;
    frame->flags    |= AV_FRAME_FLAG_KEY;
    *got_frame = 1;

    return avpkt->size;
}

static av_cold int tico_raw_decode_init(AVCodecContext *avctx)
{
    TicoRawContext *s = avctx->priv_data;

    /* The option range does not stop a NaN: every comparison with it is false. */
    if (!isfinite(s->exposure)) {
        av_log(avctx, AV_LOG_ERROR, "The exposure must be a finite number of EV\n");
        return AVERROR(EINVAL);
    }
    return 0;
}

static av_cold int tico_raw_decode_close(AVCodecContext *avctx)
{
    TicoRawContext *s = avctx->priv_data;

    ff_ticoraw_core_free(&s->core);
    av_freep(&s->curve);
    ff_ticoraw_view_free(&s->view_ctx);
    return 0;
}

#define OFFSET(x) offsetof(TicoRawContext, x)
#define VD AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_VIDEO_PARAM
static const AVOption options[] = {
    { "view", "Output picture", OFFSET(view), AV_OPT_TYPE_INT,
      { .i64 = TICORAW_VIEW_BAYER }, TICORAW_VIEW_BAYER, TICORAW_VIEW_REC709, VD, .unit = "view" },
        { "bayer",  "Linear Bayer mosaic (bayer_rggb16)", 0, AV_OPT_TYPE_CONST,
          { .i64 = TICORAW_VIEW_BAYER },  0, 0, VD, .unit = "view" },
        { "rec709", "Rec.709 RGB viewing picture (gbrp16)", 0, AV_OPT_TYPE_CONST,
          { .i64 = TICORAW_VIEW_REC709 }, 0, 0, VD, .unit = "view" },
    { "view_size", "Size of the rec709 view", OFFSET(view_size), AV_OPT_TYPE_INT,
      { .i64 = TICORAW_VIEW_HALF }, TICORAW_VIEW_HALF, TICORAW_VIEW_FULL, VD, .unit = "view_size" },
        { "half", "One pixel per 2x2 quad: half width and height", 0, AV_OPT_TYPE_CONST,
          { .i64 = TICORAW_VIEW_HALF }, 0, 0, VD, .unit = "view_size" },
        { "full", "Bilinear demosaic: full width and height", 0, AV_OPT_TYPE_CONST,
          { .i64 = TICORAW_VIEW_FULL }, 0, 0, VD, .unit = "view_size" },
    { "exposure", "Exposure offset of the rec709 view in EV", OFFSET(exposure), AV_OPT_TYPE_DOUBLE,
      { .dbl = 0.0 }, -8.0, 8.0, VD },
    { NULL },
};

static const AVClass tico_raw_decoder_class = {
    .class_name = "tico_raw",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_tico_raw_decoder = {
    .p.name         = "tico_raw",
    CODEC_LONG_NAME("intoPIX TICO RAW (Nikon N-RAW, R3D NE)"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_TICO_RAW,
    .priv_data_size = sizeof(TicoRawContext),
    .p.priv_class   = &tico_raw_decoder_class,
    .init           = tico_raw_decode_init,
    .close          = tico_raw_decode_close,
    FF_CODEC_DECODE_CB(tico_raw_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS |
                      AV_CODEC_CAP_SLICE_THREADS,
};
