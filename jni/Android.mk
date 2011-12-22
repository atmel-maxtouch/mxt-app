LOCAL_PATH := $(call my-dir)

SRC_PATH := $(LOCAL_PATH)/../src

subdirs := $(addprefix $(SRC_PATH)/,$(addsuffix /Android.mk, \
    libmaxtouch \
    jni \
    tools/mxt_app \
    tools/config_loader \
    tools/mxt_bridge \
    tools/mxt_bootloader \
  ))

include $(subdirs)
