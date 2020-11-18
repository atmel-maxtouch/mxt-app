#!/bin/sh

TOPDIR=$(dirname "$0")/..

if [ -z "$VERSION" ]
then
  VERSION=$(tr -d '\n' < "$TOPDIR/VERSION")
fi

if [ -d "$TOPDIR/.git" ]
then
  VERSION=$(cd "$TOPDIR" && git describe --tags --dirty=-mod --always | sed s/^v//)
fi

echo "$VERSION"