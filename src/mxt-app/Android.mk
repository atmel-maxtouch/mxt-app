LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_CFLAGS += -DHAVE_LIBUSB -DMXT_VERSION=\"$(GIT_VERSION)\"
LOCAL_C_INCLUDES := $(MXTAPP_TOP_DIR)/src $(MXTAPP_TOP_DIR)/lib/libusbdroid/code/src
LOCAL_SRC_FILES := \
  mxt_app.c \
  broken_line.c \
  sensor_variant.c \
  polyfit.c \
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
LOCAL_STATIC_LIBRARIES := maxtouch libusbdroid
LOCAL_SHARED_LIBRARIES := liblog
LOCAL_MODULE := mxt-app

include $(BUILD_EXECUTABLE)
