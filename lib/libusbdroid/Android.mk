LOCAL_PATH := $(my-dir)/code/src/LibUSBDroid/jni
include $(CLEAR_VARS)

LOCAL_SRC_FILES := libusb_jni.c core.c descriptor.c io.c sync.c os/linux_usbfs.c
LOCAL_CFLAGS := -fPIC -DPIC
LOCAL_MODULE := libusbdroid

include $(BUILD_STATIC_LIBRARY)
