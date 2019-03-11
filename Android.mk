LOCAL_PATH := $(call my-dir)

GIT_VERSION = `sh -c '${MXTAPP_TOP_DIR}/build-aux/version.sh'`

subdirs := $(addprefix $(MXTAPP_TOP_DIR)/,$(addsuffix /Android.mk, \
    src/libmaxtouch \
    src/mxt-app \
    lib/libusbdroid \
  ))

include $(subdirs)

