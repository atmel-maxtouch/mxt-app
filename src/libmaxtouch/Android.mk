LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(MXTAPP_TOP_DIR)/src $(MXTAPP_TOP_DIR)/lib/libusbdroid/code/src
LOCAL_CFLAGS += -DHAVE_LIBUSB -DMXT_VERSION=\"$(GIT_VERSION)\"
LOCAL_SRC_FILES := \
  libmaxtouch.c \
  log.c \
  msg.c \
  config.c \
  utilfuncs.c \
  info_block.c \
  sysfs/sysfs_device.c \
  sysfs/dmesg.c \
  i2c_dev/i2c_dev_device.c \
  hidraw/hidraw_device.c \
  usb/usb_device.c
LOCAL_MODULE := maxtouch
LOCAL_STATIC_LIBRARIES := libusbdroid

include $(BUILD_STATIC_LIBRARY)
