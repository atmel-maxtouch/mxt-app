#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.67])
AC_INIT([obp-utils], [1.6], [nick.dyer@itdev.co.uk])
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
AC_HEADER_STDBOOL
AC_CHECK_HEADERS([arpa/inet.h fcntl.h netinet/in.h stdint.h stdlib.h])
AC_CHECK_HEADERS([string.h sys/ioctl.h sys/socket.h sys/statfs.h unistd.h])
AC_CHECK_HEADERS([limits.h malloc.h netdb.h sys/time.h])

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

AC_CONFIG_FILES([Makefile doc/Makefile])

AC_OUTPUT
