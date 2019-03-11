LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CFLAGS += -DMXT_VERSION=\"$(GIT_VERSION)\"
LOCAL_C_INCLUDES += $(MXTAPP_TOP_DIR)/src
LOCAL_SRC_FILES := jniinterface.c
LOCAL_LDLIBS := -llog
LOCAL_STATIC_LIBRARIES := maxtouch libusbdroid
LOCAL_SHARED_LIBRARIES := liblog
LOCAL_MODULE    := libmaxtouch-jni

include $(BUILD_SHARED_LIBRARY)
