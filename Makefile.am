EXTRA_DIST = autogen.sh
AM_MAKEFLAGS = --no-print-directory

OBP_VERSION := $(shell sh -c 'build-aux/version.sh')

.PHONY: force
obp-version: force
	echo '$(OBP_VERSION)' | cmp -s - $@ || echo '$(OBP_VERSION)' > $@

AM_CFLAGS = \
	-I$(top_srcdir)/src \
	-fvisibility=hidden \
	-ffunction-sections \
	-fdata-sections \
  -g -Wall \
  -Wmissing-declarations -Wmissing-prototypes -Wnested-externs \
  -Wpointer-arith -Wsign-compare -Wchar-subscripts -Wstrict-prototypes \
  -Wwrite-strings \
  -Wshadow -Wformat-security -Wtype-limits \
  -Werror \
  -D__GIT_VERSION=\"$(OBP_VERSION)\" \
  -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0

AM_LDFLAGS = \
	-Wl,--gc-sections \
	-Wl,--as-needed

bin_PROGRAMS = mxt-app
noinst_LTLIBRARIES = libmaxtouch.la

libmaxtouch_la_SOURCES =\
	src/libmaxtouch/libmaxtouch.c \
	src/libmaxtouch/info_block.c \
	src/libmaxtouch/log.c \
	src/libmaxtouch/utilfuncs.c \
	src/libmaxtouch/msg.c \
	src/libmaxtouch/config.c \
	src/libmaxtouch/sysfs/sysfs_device.c \
	src/libmaxtouch/sysfs/sysinfo.c \
	src/libmaxtouch/sysfs/dmesg.c \
	src/libmaxtouch/i2c_dev/i2c_dev_device.c

if HAVE_LIBUSB
libmaxtouch_la_SOURCES += src/libmaxtouch/usb/usb_device.c
AM_CFLAGS += -DHAVE_LIBUSB
libmaxtouch_la_LIBS = @USBLIBS@
libmaxtouch_la_LDFLAGS = -lusb-1.0
endif

mxt_app_LDADD = libmaxtouch.la
EXTRA_mxt_app_DEPENDENCIES = obp-version
mxt_app_SOURCES =\
	src/tools/mxt_app/mxt_app.c \
	src/tools/mxt_app/menu.c \
	src/tools/mxt_app/bootloader.c \
	src/tools/mxt_app/diagnostic_data.c \
	src/tools/mxt_app/touch_app.c \
	src/tools/mxt_app/self_test.c \
	src/tools/mxt_app/bridge.c \
	src/tools/mxt_app/gr.c \
	src/tools/mxt_app/serial_data.c
