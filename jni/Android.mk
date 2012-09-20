LOCAL_PATH := $(call my-dir)

SRC_PATH := $(LOCAL_PATH)/../src

GIT_VERSION = $(shell sh -c 'build-aux/version.sh')

subdirs := $(addprefix $(SRC_PATH)/,$(addsuffix /Android.mk, \
    libmaxtouch \
    jni \
    tools/mxt_app \
    tools/mxt_config_loader \
    tools/mxt_bootloader \
    tools/mxt_debug_dump \
  ))

include $(subdirs)
