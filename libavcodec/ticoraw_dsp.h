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

#ifndef AVCODEC_TICORAW_DSP_H
#define AVCODEC_TICORAW_DSP_H

#include <stddef.h>
#include <stdint.h>

#define TICORAW_ST_PASSES 6  /* input clamp, then the five lifting steps of note 8 */

/*
 * All strides are in int32_t elements, not bytes. All arithmetic is integer, with
 * floor division done by arithmetic right shifts (format note, Notation).
 */
typedef struct TicoRawDSPContext {
    /**
     * One-dimensional inverse 5/3 step (format note 7.1), in place on one row.
     * Input: ceil(n / 2) low-pass values, then floor(n / 2) high-pass values.
     * Output: n interleaved samples.
     *
     * @param x    the row, n elements
     * @param tmp  scratch, at least n elements, must not overlap x
     */
    void (*idwt53_h)(int32_t *x, int32_t *tmp, int n);

    /**
     * Vertical inverse 5/3 step (format note 7.2 step 3), in place on a plane in the
     * canonical layout: row 2i holds low-pass value i and row 2i + 1 holds high-pass
     * value i of every column. Columns are independent, so a caller may pass a strip of
     * columns (p + x0, w = strip width).
     */
    void (*idwt53_v)(int32_t *p, ptrdiff_t stride, int w, int h);

    /**
     * Inverse Star-Tetrix (format note 8), in place on the four planes A, C, E, F
     * (components 0 to 3), each w x h. After all passes they hold the CFA planes
     * Gr, R, Gb, B (format note 10.6).
     *
     * Pass 0 clamps the input, passes 1 to 5 are the five lifting steps. Each pass works
     * on rows y0 to y1 - 1 only and reads only planes it does not write, so the rows of
     * one pass may be split across threads; every row of a pass must be done before the
     * next pass starts.
     */
    void (*inv_star_tetrix[TICORAW_ST_PASSES])(int32_t *const p[4], ptrdiff_t stride,
                                               int w, int h, int y0, int y1);
} TicoRawDSPContext;

void ff_ticoraw_dsp_init(TicoRawDSPContext *c);
void ff_ticoraw_dsp_init_aarch64(TicoRawDSPContext *c);

#endif /* AVCODEC_TICORAW_DSP_H */
