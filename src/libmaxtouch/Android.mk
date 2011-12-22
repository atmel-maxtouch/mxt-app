LOCAL_PATH := $(my-dir)
include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(SRC_PATH)
LOCAL_SRC_FILES := \
  libmaxtouch.c \
  log.c \
  dmesg.c \
  sysinfo.c \
  info_block.c \
  sysfs/sysfs_device.c \
  i2c_dev/i2c_dev_device.c
LOCAL_LDLIBS := -llog
LOCAL_MODULE := libmaxtouch

include $(BUILD_STATIC_LIBRARY)
