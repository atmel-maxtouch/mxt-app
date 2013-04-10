LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_CFLAGS += -D__GIT_VERSION=\"$(GIT_VERSION)\"
LOCAL_C_INCLUDES := $(SRC_PATH)
LOCAL_SRC_FILES := \
  mxt_app.c \
  bootloader.c \
  diagnostic_data.c \
  touch_app.c \
  self_test.c \
  bridge.c \
  gr.c \
  serial_data.c
LOCAL_LDLIBS := -llog
LOCAL_STATIC_LIBRARIES := maxtouch
LOCAL_MODULE := mxt-app

include $(BUILD_EXECUTABLE)
