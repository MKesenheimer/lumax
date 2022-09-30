#!/bin/bash
# install the FTDI drivers!
# All Platforms: https://ftdichip.com/drivers/d2xx-drivers/
# Arch Linux: https://aur.archlinux.org/packages/libftd2xx

platform=linux

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
#gcc -c -I<TODO: path to ftdlib> liblumax.c
#gcc -shared -fPIC -o liblumax_win32.so liblumax.o -L<TODO:path to ftdlib> -lftd2xx

# Linux
if [ "$platform" == "linux" ]; then
  gcc -c -fPIC -I/usr/include liblumax.c
  gcc -shared -fPIC -o liblumax_linux.so liblumax.o -L/usr/include -lftd2xx
fi

# move the libraries to the libs folder
mv *.so *.dylib libs
rm *.o

# copy files for the test application
cp libs/*.dylib libs/*.so test
