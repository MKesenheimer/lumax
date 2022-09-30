#!/bin/bash

platform=linux

# compile the main test application
# macOS
if [ "$platform" == "macOS" ]; then
  # check paths with
  #otool -L main
  #otool -D liblumax.so

  gcc -rpath @executable_path main.c -o main -I.. -L. -lm -llumax_darwin
fi

# linux
if [ "$platform" == "linux" ]; then
  # check paths with
  #readelf -d main | head -20

  gcc -Wl,-rpath='$ORIGIN' main.c -o main -I.. -L. -lm -llumax_linux
fi
