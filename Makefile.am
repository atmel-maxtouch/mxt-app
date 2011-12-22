EXTRA_DIST = autogen.sh
ACLOCAL_AMFLAGS = -I m4 ${ACLOCAL_FLAGS}
AM_MAKEFLAGS = --no-print-directory

AM_CFLAGS = \
	-I$(top_srcdir)/src \
	-fvisibility=hidden \
	-ffunction-sections \
	-fdata-sections

AM_LDFLAGS = \
	-Wl,--gc-sections \
	-Wl,--as-needed

bin_PROGRAMS = mxt-app mxt-config-loader mxt-bridge mxt-bootloader
noinst_LTLIBRARIES = libmaxtouch.la

libmaxtouch_la_SOURCES =\
	src/libmaxtouch/libmaxtouch.c \
	src/libmaxtouch/info_block.c \
	src/libmaxtouch/dmesg.c \
	src/libmaxtouch/sysinfo.c \
	src/libmaxtouch/log.c \
	src/libmaxtouch/sysfs/sysfs_device.c \
	src/libmaxtouch/i2c_dev/i2c_dev_device.c

if HAVE_LIBUSB
libmaxtouch_la_SOURCES += src/libmaxtouch/usb/usb_device.c
AM_CFLAGS += -DHAVE_LIBUSB
libmaxtouch_la_LIBS = @USBLIBS@
libmaxtouch_la_LDFLAGS = -lusb-1.0
endif

mxt_config_loader_LDADD = libmaxtouch.la
mxt_config_loader_SOURCES = \
	src/tools/config_loader/config_loader.c

mxt_bridge_LDADD = libmaxtouch.la
mxt_bridge_SOURCES = \
	src/tools/mxt_bridge/mxt_bridge.c

mxt_bootloader_LDADD = libmaxtouch.la
mxt_bootloader_SOURCES = \
	src/tools/mxt_bootloader/mxt_bootloader.c

mxt_app_LDADD = libmaxtouch.la
mxt_app_SOURCES =\
	src/tools/mxt_app/mxt_app.c \
	src/tools/mxt_app/touch_app.c \
	src/tools/mxt_app/utilfuncs.c
