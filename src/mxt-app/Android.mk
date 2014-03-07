LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_CFLAGS += -DMXT_VERSION=\"$(GIT_VERSION)\"
LOCAL_C_INCLUDES := $(SRC_PATH)
LOCAL_SRC_FILES := \
  mxt_app.c \
  menu.c \
  bootloader.c \
  diagnostic_data.c \
  touch_app.c \
  self_test.c \
  bridge.c \
  buffer.c \
  gr.c \
  serial_data.c \
  self_cap.c
LOCAL_LDLIBS := -llog
LOCAL_STATIC_LIBRARIES := maxtouch
LOCAL_MODULE := mxt-app

include $(BUILD_EXECUTABLE)
