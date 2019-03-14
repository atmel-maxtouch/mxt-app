LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(MXTAPP_TOP_DIR)/src
LOCAL_CFLAGS += -DMXT_VERSION=\"$(GIT_VERSION)\"
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
LOCAL_STATIC_LIBRARIES := maxtouch
LOCAL_SHARED_LIBRARIES := liblog
LOCAL_MODULE := mxt-app

ifneq ($(MXTAPP_NO_USB_SUPPORT),true)
    LOCAL_C_INCLUDES += $(MXTAPP_TOP_DIR)/lib/libusbdroid/code/src
    LOCAL_CFLAGS += -DHAVE_LIBUSB
    LOCAL_STATIC_LIBRARIES += libusbdroid
endif

include $(BUILD_EXECUTABLE)
