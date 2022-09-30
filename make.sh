#!/bin/bash


# macOS
gcc -c -I/opt/local/include liblumax.c
gcc -shared     -fPIC -o liblumax_darwin.so    liblumax.o -L/opt/local/lib/ -lftd2xx
gcc -dynamiclib -fPIC -o liblumax_darwin.dylib liblumax.o -L/opt/local/lib/ -lftd2xx

# Windows
#gcc -c -I<TODO: path to ftdlib> liblumax.c
#gcc -shared -fPIC -o liblumax_win32.so liblumax.o -L<TODO:path to ftdlib> -lftd2xx

# Linux
#gcc -c -I<TODO: path to ftdlib> liblumax.c
#gcc -shared -fPIC -o liblumax_linux.so liblumax.o -L<TODO:path to ftdlib> -lftd2xx


# set the install names in the shared libraries (in order to find with rpath)
install_name_tool -id @rpath/liblumax_darwin.so liblumax_darwin.so
install_name_tool -id @rpath/liblumax_darwin.dylib liblumax_darwin.dylib

# check paths with
#otool -L main
#otool -D liblumax.so

# move the libraries to the libs folder
mv *.so *.dylib libs
rm *.o

# for the test application
cp libs/*.dylib libs/*.so test
