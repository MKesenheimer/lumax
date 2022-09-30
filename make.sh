#!/bin/bash
# install the FTDI drivers!
# All Platforms: https://ftdichip.com/drivers/d2xx-drivers/
# Arch Linux: https://aur.archlinux.org/packages/libftd2xx

# macOS, linux, windows
platform=windows

# macOS
if [ "$platform" == "macOS" ]; then
  gcc -c -fPIC -I/opt/local/include liblumax.c
  gcc -shared     -fPIC -o liblumax_darwin.so    liblumax.o -L/opt/local/lib/ -lftd2xx
  gcc -dynamiclib -fPIC -o liblumax_darwin.dylib liblumax.o -L/opt/local/lib/ -lftd2xx

  # set the install names in the shared libraries (in order to find with rpath)
  install_name_tool -id @rpath/liblumax_darwin.so liblumax_darwin.so
  install_name_tool -id @rpath/liblumax_darwin.dylib liblumax_darwin.dylib

  # check paths with
  #otool -L main
  #otool -D liblumax.so
fi

# Windows
if [ "$platform" == "windows" ]; then
  x86_64-w64-mingw32-cc -DWINDOWS -c -I./FTD2XX/extracted liblumax.c
  x86_64-w64-mingw32-cc -shared -fPIC -o liblumax_win32.so liblumax.o -L./FTD2XX/extracted/amd64 -lwinmm -lftd2xx
fi

# Linux
if [ "$platform" == "linux" ]; then
  gcc -c -fPIC -I/usr/include liblumax.c
  gcc -shared -fPIC -o liblumax_linux.so liblumax.o -L/usr/include -lftd2xx

  # create rules for this usb device
  # /etc/udev/rules.d/87-lumax.rules
  sudo cp 87-lumax.rules /etc/udev/rules.d/87-lumax.rules
  # /etc/udev/rules.d/89-lumax.rules
  sudo cp 89-lumax.rules /etc/udev/rules.d/89-lumax.rules
  sudo udevadm trigger
fi

# move the libraries to the libs folder
mv *.so *.dylib libs
rm *.o

# copy files for the test application
cp libs/*.dylib libs/*.so libs/*.dll test
