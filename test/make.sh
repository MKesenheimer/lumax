#!/bin/bash

if [ "$1" == "" ]; then
  echo "Usage: ./make.sh <platform: macOS | linux | windows>"
  exit -1
fi

platform=$1

# compile the main test application
# macOS
if [ "$platform" == "macOS" ]; then
  # check paths with
  #otool -L main
  #otool -D liblumax.so

  gcc -rpath @executable_path/libs main.c -o main_darwin -I. -Llibs -lm -llumax_darwin
fi

# linux
if [ "$platform" == "linux" ]; then
  # check paths with
  #readelf -d main | head -20

  gcc -Wl,-rpath='$ORIGIN/libs' main.c -o main_linux -I. -Llibs -lm -llumax_linux
fi

# windows
if [ "$platform" == "windows" ]; then
  i686-w64-mingw32-gcc -Wl,-rpath='$ORIGIN/libs' main.c -o main.exe -I. -Llibs -lm -llumax_windows
fi
