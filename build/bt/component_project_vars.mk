# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(PROJECT_PATH)/esp-idf/components/bt/include $(PROJECT_PATH)/esp-idf/components/bt/bluedroid/api/include/api
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/bt -lbt -L $(PROJECT_PATH)/esp-idf/components/bt/lib -lbtdm_app
COMPONENT_LINKER_DEPS += $(PROJECT_PATH)/esp-idf/components/bt/lib/libbtdm_app.a
COMPONENT_SUBMODULES += $(PROJECT_PATH)/esp-idf/components/bt/lib
COMPONENT_LIBRARIES += bt
component-bt-build: 
