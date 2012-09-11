LOCAL_PATH := $(call my-dir)

SRC_PATH := $(LOCAL_PATH)/../src

GIT_VERSION = $(shell sh -c 'git describe --abbrev=4 --dirty --always --long | sed s/^v//')

subdirs := $(addprefix $(SRC_PATH)/,$(addsuffix /Android.mk, \
    libmaxtouch \
    jni \
    tools/mxt_app \
    tools/mxt_config_loader \
    tools/mxt_bridge \
    tools/mxt_bootloader \
    tools/mxt_debug_dump \
  ))

include $(subdirs)
