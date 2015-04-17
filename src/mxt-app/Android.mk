LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# Bypass PIE security
# LOCAL_CFLAGS += -fPIE –pie
# LOCAL_LDFLAGS += -fPIE –pie

LOCAL_CFLAGS += -DHAVE_LIBUSB -DMXT_VERSION=\"$(GIT_VERSION)\"
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
  self_cap.c \
  signal.c
LOCAL_LDLIBS := -llog
LOCAL_STATIC_LIBRARIES := maxtouch
LOCAL_MODULE := mxt-app

include $(BUILD_EXECUTABLE)
