#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.63])
AC_INIT([obp-utils], [1.16], [nick.dyer@itdev.co.uk])
AM_INIT_AUTOMAKE([foreign subdir-objects])
AC_CONFIG_SRCDIR([src/libmaxtouch/libmaxtouch.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_CC
LT_INIT

# Check for usb support
AC_CHECK_LIB([usb-1.0], [libusb_init], [libusb=true])
AC_CHECK_LIB([usb-1.0], [libusb_error_name], AC_DEFINE([HAVE_LIBUSB_ERROR_NAME], [1], [ ]))
AM_CONDITIONAL([HAVE_LIBUSB], [test x$libusb = xtrue])

# Handle debug/release build
AC_ARG_ENABLE(debug,
AS_HELP_STRING([--enable-debug],
               [enable debugging, default: no]),
[case "${enableval}" in
             yes) debug=true ;;
             no)  debug=false ;;
             *)   AC_MSG_ERROR([bad value ${enableval} for --enable-debug]) ;;
esac],
[debug=false])

AM_CONDITIONAL(DEBUG, test x"$debug" = x"true")

# Checks for header files.
AC_HEADER_STDBOOL
AC_CHECK_HEADERS([arpa/inet.h fcntl.h netinet/in.h stdint.h stdlib.h])
AC_CHECK_HEADERS([string.h sys/ioctl.h sys/socket.h sys/statfs.h unistd.h])
AC_CHECK_HEADERS([limits.h malloc.h netdb.h sys/time.h ctype.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_INLINE
AC_TYPE_INT16_T
AC_TYPE_INT8_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T
AC_TYPE_SIZE_T

# Checks for library functions.
AC_FUNC_MALLOC
AC_FUNC_REALLOC
AC_CHECK_FUNCS([bzero])
AC_CHECK_FUNCS([gettimeofday])
AC_CHECK_FUNCS([select])
AC_CHECK_FUNCS([setlocale])
AC_CHECK_FUNCS([socket])
AC_CHECK_FUNCS([strerror])
AC_CHECK_FUNCS([strncasecmp])
AC_CHECK_FUNCS([tolower])

AC_CONFIG_FILES([Makefile doc/Makefile])

AC_OUTPUT
