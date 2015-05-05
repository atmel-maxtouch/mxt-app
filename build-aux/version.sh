#!/bin/sh

TOPDIR=$(dirname $0)/..

if [ -d "$TOPDIR/.git" ]
then
  VERSION=`cd $TOPDIR; git describe --dirty=-mod --always | sed s/^v//`
fi

if [ -z $VERSION ]
then
  VERSION=`tr -d '\n' < $TOPDIR/VERSION`
fi

echo $VERSION
