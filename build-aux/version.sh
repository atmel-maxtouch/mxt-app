#!/bin/sh

TOPDIR=$(dirname $0)/..

if [ -d "$TOPDIR/.git" ]
then
  VERSION=`cd $TOPDIR; git describe --abbrev=4 --dirty=-mod --always | sed s/^v//`
fi

if [ -z $VERSION ]
then
  VERSION=`tr -d '\n' < $TOPDIR/VERSION`
fi

echo $VERSION
