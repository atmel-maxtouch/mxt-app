LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(SRC_PATH)
LOCAL_SRC_FILES := \
  mxt_config_loader.c
LOCAL_LDLIBS := -llog
LOCAL_STATIC_LIBRARIES := maxtouch
LOCAL_MODULE := mxt-config-loader

include $(BUILD_EXECUTABLE)
