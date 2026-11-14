# TICO RAW fork of FFmpeg

This is a fork of [FFmpeg](https://github.com/FFmpeg/FFmpeg). It adds a decoder named
`tico_raw` for the raw video in Nikon clips (`.nev`, and `.r3d` clips that hold TICO RAW).
The decoder was written clean room. It is not part of upstream FFmpeg, and these branches are
not meant for upstream.

| Branch | Based on | Use |
| --- | --- | --- |
| `tico-raw` (default) | FFmpeg `master` (commit `2bf291526e`) | Current work |
| `tico-raw-9.0` | FFmpeg `n9.0.2` | The same decoder on 9.0.2. IINA RAW is built from this branch. |

The decoder commits are the six commits that follow the base:

1. `avcodec: add AV_CODEC_ID_TICO_RAW and the Nikon mov tags`
2. `avcodec/ticoraw: add the Nikon wrapper and codestream header parsers`
3. `avcodec/ticoraw: add entropy decoding, the inverse wavelet and Star-Tetrix, and the core`
4. `avcodec/ticoraw: add the TICO RAW decoder`
5. `avcodec/ticoraw: add an optional Rec.709 view (fork-only, not for upstream)`
6. `avcodec/tests/ticoraw: add a self-test program for the TICO RAW decoder`

The Rec.709 view option (commit 5) exists only in this fork. It is meant for players and is
not part of any upstream plan.

Project page, tests, results and the player that uses this decoder (IINA RAW, for Apple
Silicon Macs): [alperkaganavci/nikon-raw-playback](https://github.com/alperkaganavci/nikon-raw-playback).

FFmpeg's licence applies to this fork (LGPL-2.1-or-later, see `LICENSE.md` and `COPYING.LGPLv2.1`).
This fork is not affiliated with the FFmpeg project, Nikon, intoPIX or RED.

---

# FFmpeg README

FFmpeg is a collection of libraries and tools to process multimedia content
such as audio, video, subtitles and related metadata.

## Libraries

* `libavcodec` provides implementation of a wider range of codecs.
* `libavformat` implements streaming protocols, container formats and basic I/O access.
* `libavutil` includes hashers, decompressors and miscellaneous utility functions.
* `libavfilter` provides means to alter decoded audio and video through a directed graph of connected filters.
* `libavdevice` provides an abstraction to access capture and playback devices.
* `libswresample` implements audio mixing and resampling routines.
* `libswscale` implements color conversion and scaling routines.

## Tools

* [ffmpeg](https://ffmpeg.org/ffmpeg.html) is a command line toolbox to
  manipulate, convert and stream multimedia content.
* [ffplay](https://ffmpeg.org/ffplay.html) is a minimalistic multimedia player.
* [ffprobe](https://ffmpeg.org/ffprobe.html) is a simple analysis tool to inspect
  multimedia content.
* Additional small tools such as `aviocat`, `ismindex` and `qt-faststart`.

## Documentation

The offline documentation is available in the **doc/** directory.

The online documentation is available in the main [website](https://ffmpeg.org)
and in the [wiki](https://trac.ffmpeg.org).

### Examples

Coding examples are available in the **doc/examples** directory.

## License

FFmpeg codebase is mainly LGPL-licensed with optional components licensed under
GPL. Please refer to the LICENSE file for detailed information.

## Contributing

Patches should be submitted to the ffmpeg-devel mailing list using
`git format-patch` or `git send-email`. Github pull requests should be
avoided because they are not part of our review process and will be ignored.
