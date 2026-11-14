FATE_TICORAW-$(call FRAMECRC, MOV, TICO_RAW) += fate-ticoraw-nikon-zr-r3dne
fate-ticoraw-nikon-zr-r3dne: CMD = framecrc -i $(TARGET_SAMPLES)/ticoraw/nikon-zr-r3dne-cut.mov

FATE_TICORAW-$(call FRAMECRC, MOV, TICO_RAW) += fate-ticoraw-nikon-zr-r3dne-slice
fate-ticoraw-nikon-zr-r3dne-slice: CMD = framecrc -i $(TARGET_SAMPLES)/ticoraw/nikon-zr-r3dne-cut.mov
fate-ticoraw-nikon-zr-r3dne-slice: REF = $(SRC_PATH)/tests/ref/fate/ticoraw-nikon-zr-r3dne
fate-ticoraw-nikon-zr-r3dne-slice: THREADS = 4
fate-ticoraw-nikon-zr-r3dne-slice: THREAD_TYPE = slice

FATE_SAMPLES_FFMPEG += $(FATE_TICORAW-yes)
fate-ticoraw: $(FATE_TICORAW-yes)
