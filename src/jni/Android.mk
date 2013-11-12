LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CFLAGS += -D__GIT_VERSION=\"$(GIT_VERSION)\"
LOCAL_C_INCLUDES += $(SRC_PATH)
LOCAL_SRC_FILES := jniinterface.c
LOCAL_LDLIBS := -llog
LOCAL_STATIC_LIBRARIES := maxtouch
LOCAL_MODULE    := libmaxtouch-jni

include $(BUILD_SHARED_LIBRARY)
