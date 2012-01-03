#!/bin/sh -e

# Remove dependency cache otherwise file renames confuse autoconf
find . -type d -name \.deps | xargs rm -rf

OBPCFLAGS="-g -Wall \
-Wmissing-declarations -Wmissing-prototypes \
-Wnested-externs -Wpointer-arith \
-Wsign-compare -Wchar-subscripts \
-Wstrict-prototypes -Wshadow \
-Wformat-security -Wtype-limits"

autoreconf -v --install

./configure CFLAGS="${OBPCFLAGS} ${CFLAGS}" $@
