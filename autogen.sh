#!/bin/sh -e

# Remove dependency cache otherwise file renames confuse autoconf
find . -type d -name \.deps | xargs rm -rf

autoreconf -v --install

./configure $@
