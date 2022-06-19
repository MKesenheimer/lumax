#!/bin/bash

gcc -c -I/opt/local/include liblumax.c
gcc -shared -o liblumax.so liblumax.o -L/opt/local/lib/ -lftd2xx
gcc -dynamiclib -fPIC -o liblumax.dylib liblumax.o -L/opt/local/lib/ -lftd2xx

# set the install names in the shared libraries (in order to find with rpath)
install_name_tool -id @rpath/liblumax.so liblumax.so
install_name_tool -id @rpath/liblumax.dylib liblumax.dylib

# check paths with
#otool -L main
#otool -D liblumax.so

gcc -rpath @executable_path main.c -o main -L. -llumax
