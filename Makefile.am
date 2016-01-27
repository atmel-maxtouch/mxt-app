ACLOCAL_AMFLAGS = -I m4

EXTRA_DIST = autogen.sh build-aux/version.sh README.md doc/doxygen.cfg

GIT_VERSION := $(shell $(top_srcdir)/build-aux/version.sh)

.PHONY: force
git-version: force
	echo '$(GIT_VERSION)' | cmp -s - $@ || echo '$(GIT_VERSION)' > $@

AM_MAKEFLAGS = --no-print-directory

AM_CFLAGS = \
	-I$(top_srcdir)/src \
	-fvisibility=hidden \
	-ffunction-sections \
	-fdata-sections \
	-Wall -Werror -Wmissing-declarations -Wmissing-prototypes -Wnested-externs \
	-Wpointer-arith -Wsign-compare -Wchar-subscripts -Wstrict-prototypes \
	-Wwrite-strings -Wshadow -Wformat-security -Wtype-limits \
	-DMXT_VERSION=\"$(GIT_VERSION)\" \
	-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 -D_GNU_SOURCE=1

AM_LDFLAGS = \
	-Wl,--gc-sections \
	-Wl,--as-needed

if !DEBUG
AM_CFLAGS += -DNDEBUG
endif

bin_PROGRAMS = mxt-app
noinst_LTLIBRARIES = libmaxtouch.la

check_PROGRAMS = run_unit_tests

run_unit_tests_SOURCES =\
  src/test/run_unit_tests.c

run_unit_tests_CFLAGS =\
  -I$(top_srcdir)/src \
	-fvisibility=hidden \
	-ffunction-sections \
	-fdata-sections \
	-DMXT_VERSION=\"$(GIT_VERSION)\" \
	-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0

run_unit_tests_LDFLAGS = -lcmocka -lmaxtouch

TESTS = run_unit_tests

libmaxtouch_la_SOURCES =\
	src/libmaxtouch/libmaxtouch.h \
	src/libmaxtouch/libmaxtouch.c \
	src/libmaxtouch/info_block.h \
	src/libmaxtouch/info_block.c \
	src/libmaxtouch/log.h \
	src/libmaxtouch/log.c \
	src/libmaxtouch/utilfuncs.h \
	src/libmaxtouch/utilfuncs.c \
	src/libmaxtouch/msg.h \
	src/libmaxtouch/msg.c \
	src/libmaxtouch/config.c \
	src/libmaxtouch/sysfs/sysfs_device.h \
	src/libmaxtouch/sysfs/sysfs_device.c \
	src/libmaxtouch/sysfs/dmesg.h \
	src/libmaxtouch/sysfs/dmesg.c \
	src/libmaxtouch/i2c_dev/i2c_dev_device.h \
	src/libmaxtouch/i2c_dev/i2c_dev_device.c \
	src/libmaxtouch/hidraw/hidraw_device.h \
	src/libmaxtouch/hidraw/hidraw_device.c

if HAVE_LIBUSB
libmaxtouch_la_SOURCES += \
	src/libmaxtouch/usb/usb_device.h \
	src/libmaxtouch/usb/usb_device.c
AM_CFLAGS += -DHAVE_LIBUSB
libmaxtouch_la_LIBS = @USBLIBS@
libmaxtouch_la_LDFLAGS = -lusb-1.0
endif

mxt_app_LDADD = libmaxtouch.la
EXTRA_mxt_app_DEPENDENCIES = git-version
mxt_app_SOURCES =\
	src/mxt-app/mxt_app.h \
	src/mxt-app/mxt_app.c \
	src/mxt-app/menu.c \
	src/mxt-app/bootloader.c \
	src/mxt-app/diagnostic_data.c \
	src/mxt-app/touch_app.c \
	src/mxt-app/self_test.c \
	src/mxt-app/bridge.c \
	src/mxt-app/gr.c \
	src/mxt-app/serial_data.c \
	src/mxt-app/buffer.c \
	src/mxt-app/buffer.h \
	src/mxt-app/self_cap.c \
	src/mxt-app/signal.c

.PHONY: doc
doc: doc/doxygen.cfg
	doxygen doc/doxygen.cfg

if ENABLE_MAN
man1_MANS = man1/mxt-app.1

man1/mxt-app.1: README.md
	@$(MKDIR_P) man1/
	-pandoc -s -t man \
	-V title="MXT-APP" \
	-V section="1" \
	-V description="\"mxt-app $(GIT_VERSION)\"" \
	$(top_srcdir)/README.md -o man1/mxt-app.1
endif

clean-local:
	-rm git-version
	-rm man1/mxt-app.1
	-rm -Rf doc/html

dist-hook:
	echo $(VERSION) > $(distdir)/VERSION

indent:
	astyle --style=linux --indent=spaces=2 \
	$(mxt_app_SOURCES) $(libmaxtouch_la_SOURCES)
