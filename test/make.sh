#!/bin/bash

# check paths with
#otool -L main
#otool -D liblumax.so

# compile the main test application
gcc -rpath @executable_path main.c -o main -I.. -L. -llumax_darwin
