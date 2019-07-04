

UGFX_PATH := $(COMPONENT_PATH)/ugfx

COMPONENT_SRCDIRS := . board games
COMPONENT_ADD_INCLUDEDIRS := . board games
COMPONENT_EXTRA_INCLUDES := \
	$(UGFX_PATH) \
	$(UGFX_PATH)/src \
	$(UGFX_PATH)/src/gdisp/mcufont \
	$(UGFX_PATH)/drivers/gdisp/ILI9341 \
	$(IDF_PATH)/components/freertos/include/freertos \
	$(UGFX_PATH) \
	$(UGFX_PATH)/src \
	$(UGFX_PATH)/src/gdisp/mcufont \
	$(UGFX_PATH)/drivers/gdisp/ILI9341 \
	$(IDF_PATH)/components/freertos/include/freertos \

