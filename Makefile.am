EXTRA_DIST = autogen.sh
AM_MAKEFLAGS = --no-print-directory

OBP_VERSION := $(shell build-aux/version.sh)

.PHONY: force
obp-version: force
	echo '$(OBP_VERSION)' | cmp -s - $@ || echo '$(OBP_VERSION)' > $@

AM_CFLAGS = \
	-I$(top_srcdir)/src \
	-fvisibility=hidden \
	-ffunction-sections \
	-fdata-sections \
  -g -Wall -Werror \
  -Wmissing-declarations -Wmissing-prototypes -Wnested-externs \
  -Wpointer-arith -Wsign-compare -Wchar-subscripts -Wstrict-prototypes \
  -Wwrite-strings -Wshadow -Wformat-security -Wtype-limits \
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
	src/mxt-app/mxt_app.c \
	src/mxt-app/menu.c \
	src/mxt-app/bootloader.c \
	src/mxt-app/diagnostic_data.c \
	src/mxt-app/touch_app.c \
	src/mxt-app/self_test.c \
	src/mxt-app/bridge.c \
	src/mxt-app/gr.c \
	src/mxt-app/serial_data.c \
	src/mxt-app/buffer.c

man1_MANS = mxt-app.1

mxt-app.1: README.md
	-pandoc -s -t man README.md -o mxt-app.1

clean-local:
	-rm mxt-app.1
