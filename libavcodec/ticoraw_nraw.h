/*
 * TICO RAW (Nikon N-RAW, R3D NE) frame wrapper parser
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

#ifndef AVCODEC_TICORAW_NRAW_H
#define AVCODEC_TICORAW_NRAW_H

#include <stdint.h>

/**
 * Per-frame camera metadata from the Nikon frame wrapper.
 */
typedef struct TicoRawMeta {
    int width, height;
    char cfa[5];            /* "RGGB", "GRBG", ... */
    double wb_gain[3];      /* R, G, B; G = 1.0 */
    double cam_to_xyz[9];   /* row-major */
    int iso;
    int white_level;
} TicoRawMeta;

/**
 * Parse the Nikon frame wrapper of one video sample.
 *
 * On success fills meta and the position of the TICO RAW codestream inside buf.
 *
 * @return 0 on success, AVERROR_INVALIDDATA for damaged or short data,
 *         AVERROR_PATCHWELCOME for a valid wrapper with unsupported values.
 */
int ff_ticoraw_parse_nraw(const uint8_t *buf, int size, TicoRawMeta *meta,
                          int *cs_offset, int *cs_size, void *logctx);

#endif /* AVCODEC_TICORAW_NRAW_H */
