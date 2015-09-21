#!/bin/sh -e

# Remove dependency cache otherwise file renames confuse autoconf
find . -type d -name \.deps | xargs rm -rf

[ -d m4 ] || mkdir m4
autoreconf -v --install

./configure $@
