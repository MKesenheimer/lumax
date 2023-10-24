#!/bin/bash
# install the FTDI drivers!
# All Platforms: https://ftdichip.com/drivers/d2xx-drivers/
# Arch Linux: https://aur.archlinux.org/packages/libftd2xx


if [ "$1" == "" ]; then
  echo "Usage: ./make.sh <platform: macOS | linux | windows>"
  exit -1
fi

# macOS, linux, windows
platform=$1
dotest=$2

# macOS
if [ "$platform" == "macOS" ]; then
  gcc -c -fPIC -I/opt/local/include -I/opt/local/include/libftdi1 liblumax.c
  gcc -shared     -fPIC -o liblumax_darwin.so    liblumax.o -L/opt/local/lib -lftdi1
  gcc -dynamiclib -fPIC -o liblumax_darwin.dylib liblumax.o -L/opt/local/lib -lftdi1

  # set the install names in the shared libraries (in order to find with rpath)
  install_name_tool -id @rpath/liblumax_darwin.so liblumax_darwin.so
  install_name_tool -id @rpath/liblumax_darwin.dylib liblumax_darwin.dylib

  # check paths with
  #otool -L main
  #otool -D liblumax.so
fi

# Linux
if [ "$platform" == "linux" ]; then
  gcc -c -fPIC -I/usr/include -I/usr/include/libftdi1 liblumax.c
  gcc -shared -fPIC -o liblumax_linux.so liblumax.o -L/usr/include -lftdi1

  # create rules for this usb device
  # /etc/udev/rules.d/87-lumax.rules
  sudo cp 87-lumax.rules /etc/udev/rules.d/87-lumax.rules
  # /etc/udev/rules.d/89-lumax.rules
  sudo cp 89-lumax.rules /etc/udev/rules.d/89-lumax.rules
  sudo udevadm trigger
fi

# move the libraries to the libs folder
mv *.so libs
rm *.o
if [ "$platform" == "macOS" ]; then
  mv *.dylib libs
fi


# copy files for the test application
cp -r libs lumax.h test

# copy files for the python interface
cp -r libs python

if [ "$dotest" != "" ]; then
  cd test
  ./make.sh $platform
  # todo: other platforms
  ./main_darwin
  cd ..
fi
