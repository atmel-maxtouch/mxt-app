LOCAL_PATH := $(my-dir)
include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(SRC_PATH)
LOCAL_CFLAGS += -DHAVE_LIBUSB
LOCAL_SRC_FILES := \
  libmaxtouch.c \
  log.c \
  msg.c \
  config.c \
  utilfuncs.c \
  info_block.c \
  sysfs/sysfs_device.c \
  sysfs/dmesg.c \
  sysfs/sysinfo.c \
  i2c_dev/i2c_dev_device.c \
  usb/usb_device.c
LOCAL_MODULE := libmaxtouch
LOCAL_STATIC_LIBRARIES := libusbdroid

include $(BUILD_STATIC_LIBRARY)
