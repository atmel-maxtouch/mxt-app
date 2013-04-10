LOCAL_PATH := $(call my-dir)

SRC_PATH := $(LOCAL_PATH)/../src

GIT_VERSION = $(shell sh -c 'build-aux/version.sh')

subdirs := $(addprefix $(SRC_PATH)/,$(addsuffix /Android.mk, \
    libmaxtouch \
    jni \
    tools/mxt_app \
  ))

include $(subdirs)
