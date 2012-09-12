#!/bin/sh

if [ -d ".git" ]
then
  VERSION=`git describe --abbrev=4 --dirty=-mod --always | sed s/^v//`
fi

if [ -z $VERSION ]
then
  VERSION=`tr -d '\n' < VERSION`
fi

echo $VERSION
