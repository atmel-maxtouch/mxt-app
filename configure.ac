#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.67])
AC_INIT([obp-utils], [1.1], [nick.dyer@itdev.co.uk])
AM_INIT_AUTOMAKE([foreign])
AC_CONFIG_SRCDIR([src/libmaxtouch/libmaxtouch.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_CC
AC_PROG_LIBTOOL

# Check for usb support
AC_CHECK_LIB([usb-1.0], [libusb_init], [libusb=true])
AM_CONDITIONAL([HAVE_LIBUSB], [test x$libusb = xtrue])

# Checks for header files.
AC_CHECK_HEADERS([arpa/inet.h fcntl.h netinet/in.h stdint.h stdlib.h string.h sys/ioctl.h sys/socket.h sys/statfs.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_INT16_T
AC_TYPE_INT8_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_MALLOC

AC_CONFIG_FILES([Makefile doc/Makefile])

AC_OUTPUT
