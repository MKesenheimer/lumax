#!/bin/bash

platform=macOS

# compile the main test application
# macOS
if [ "$platform" == "macOS" ]; then
  # check paths with
  #otool -L main
  #otool -D liblumax.so

  gcc -rpath @executable_path main.c -o main_darwin -I.. -L. -lm -llumax_darwin
fi

# linux
if [ "$platform" == "linux" ]; then
  # check paths with
  #readelf -d main | head -20

  gcc -Wl,-rpath='$ORIGIN' main.c -o main_linux -I.. -L. -lm -llumax_linux
fi

# windows
if [ "$platform" == "windows" ]; then
  i686-w64-mingw32-gcc main.c -o main.exe -I.. -L. -lm -llumax_windows
fi