LOCAL_PATH := $(call my-dir)

TOP_DIR := $(LOCAL_PATH)/..

GIT_VERSION = `sh -c 'build-aux/version.sh'`

subdirs := $(addprefix $(TOP_DIR)/,$(addsuffix /Android.mk, \
    src/libmaxtouch \
    src/jni \
    src/mxt-app \
    lib/libusbdroid \
  ))

include $(subdirs)
