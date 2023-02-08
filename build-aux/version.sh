#!/bin/sh

TOPDIR=$(dirname "$0")/..

if [ -d "$TOPDIR/.git" ]
then
  VERSION=$(cd "$TOPDIR" && git describe --tags --dirty=-mod --always | sed s/^v//)
elif [ -z "$VERSION" ]
then
  VERSION=$(tr -d '\n' < "$TOPDIR/VERSION")
fi

echo "$VERSION"