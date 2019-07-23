#
# "main" pseudo-component makefile.

# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

# COMPONENT_ADD_INCLUDEDIRS := ./app .
# COMPONENT_SRCDIRS :=  ./app .
COMPONENT_SRCDIRS :=  .
COMPONENT_PRIV_INCLUDEDIRS := ./include .

UGFX_PATH := $(PROJECT_PATH)/components/ugfx-esp32/ugfx
# GFXLIB := $(COMPONENT_PATH)/ugfx
# GFXSINGLEMAKE :=
# GFXDRIVERS := gdisp/ILI9341
# OPT_OS := freertos
# GFXBOARD :=
# GFXDEMO :=

# COMPONENT_EXTRA_INCLUDES := \
#       $(UGFX_PATH) \
#       $(UGFX_PATH)/src \
#       $(UGFX_PATH)/src/gdisp/mcufont \

COMPONENT_EXTRA_INCLUDES := \
        $(UGFX_PATH) \
        $(UGFX_PATH)/src \
        $(UGFX_PATH)/src/gdisp/mcufont \
        $(UGFX_PATH)/drivers/gdisp/ILI9341 \
        $(IDF_PATH)/components/freertos/include/freertos \
