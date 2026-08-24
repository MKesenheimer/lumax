# Makefile for the liblumax shared library.
#
# Builds natively on macOS, Linux, and Windows (MinGW / MSYS2).
#
# Targets:
#   make            build the library into libs/ and refresh the test/ and python/ copies
#   make lib        build the shared library only
#   make check      build and run the protocol unit tests (no device required)
#   make test       build everything, then build and run the C test program (device required)
#   make udev       (Linux only) install the udev rules so userspace can claim the FTDI device
#   make clean      remove all build artifacts
#
# Overrides (examples):
#   make CC=clang
#   make LIBFTDI_CFLAGS=-I/myprefix/include/libftdi1 LIBFTDI_LDFLAGS=-L/myprefix/lib

CC     ?= gcc
CFLAGS ?= -O2
CFLAGS += -fPIC

# --- platform detection -----------------------------------------------------
# Native Windows (cmd) sets OS=Windows_NT; MSYS2/MinGW shells report
# MINGW64_NT-*/MSYS_NT-* via uname.
ifeq ($(OS),Windows_NT)
PLATFORM := windows
else
UNAME_S := $(shell uname -s 2>/dev/null)
ifeq ($(UNAME_S),Darwin)
PLATFORM := macOS
else ifeq ($(UNAME_S),Linux)
PLATFORM := linux
else ifneq (,$(findstring NT,$(UNAME_S)))
PLATFORM := windows
else
$(error Unsupported platform '$(UNAME_S)'. Use macOS, Linux, or Windows (MinGW/MSYS2).)
endif
endif

# --- per-platform settings --------------------------------------------------
ifeq ($(PLATFORM),macOS)
LIBFTDI_CFLAGS  ?= -I/opt/homebrew/include/libftdi1 -I/opt/local/include/libftdi1
LIBFTDI_LDFLAGS ?= -L/opt/homebrew/lib -L/opt/local/lib
LIBS := libs/liblumax_darwin.so libs/liblumax_darwin.dylib
else ifeq ($(PLATFORM),linux)
LIBFTDI_CFLAGS  ?= -I/usr/include/libftdi1
LIBFTDI_LDFLAGS ?=
LIBS := libs/liblumax_linux.so
else
# Windows (MinGW / MSYS2): libftdi may live in the msys (/usr) or mingw64 prefix
LIBFTDI_CFLAGS  ?= -I/usr/include/libftdi1 -I/mingw64/include/libftdi1
LIBFTDI_LDFLAGS ?= -L/usr/lib -L/mingw64/lib
LIBS := libs/liblumax_windows.dll
endif

ifeq ($(PLATFORM),windows)
CHECK_BIN := test/test_protocol.exe
else
CHECK_BIN := test/test_protocol
endif

.PHONY: all lib dist check test udev clean libs

all: lib dist

lib: $(LIBS)

# lumax_protocol.o has no ftdi dependency (pure protocol logic, unit-testable)
lumax_protocol.o: lumax_protocol.c lumax_protocol.h lumax.h
	$(CC) $(CFLAGS) -c $< -o $@

liblumax.o: liblumax.c lumax.h liblumax.h lumax_protocol.h
	$(CC) $(CFLAGS) $(LIBFTDI_CFLAGS) -c $< -o $@

OBJS := liblumax.o lumax_protocol.o

# libs/ is not tracked in git; create it on demand.
libs:
	@mkdir -p libs

# macOS: C consumers link the .so, ctypes loads the .dylib; build both and
# point the install names at @rpath so consumers can find them via rpath.
libs/liblumax_darwin.so: $(OBJS) | libs
	$(CC) -shared $(LIBFTDI_LDFLAGS) -lftdi1 $^ -o $@
	install_name_tool -id @rpath/liblumax_darwin.so $@

libs/liblumax_darwin.dylib: $(OBJS) | libs
	$(CC) -dynamiclib $(LIBFTDI_LDFLAGS) -lftdi1 $^ -o $@
	install_name_tool -id @rpath/liblumax_darwin.dylib $@

libs/liblumax_linux.so: $(OBJS) | libs
	$(CC) -shared $(LIBFTDI_LDFLAGS) -lftdi1 $^ -o $@

libs/liblumax_windows.dll: $(OBJS) | libs
	$(CC) -shared $(LIBFTDI_LDFLAGS) -lftdi1 $^ -o $@

# protocol unit tests: lumax_protocol.c has no ftdi dependency
check:
	$(CC) $(CFLAGS) -I. test/test_protocol.c lumax_protocol.c -o $(CHECK_BIN)
	./$(CHECK_BIN)

# keep the C test program and the Python bindings in sync with the fresh build
dist: lib
	rm -rf test/libs python/libs
	cp -R libs test/libs
	cp -R libs python/libs
	cp -f lumax.h test/lumax.h

test: all
	$(MAKE) -C test run

udev:
ifeq ($(PLATFORM),linux)
	sudo cp 87-lumax.rules /etc/udev/rules.d/87-lumax.rules
	sudo cp 89-lumax.rules /etc/udev/rules.d/89-lumax.rules
	sudo udevadm trigger
else
	@echo "udev rules are Linux-only; nothing to do."
endif

clean:
	rm -f liblumax.o lumax_protocol.o
	rm -f $(CHECK_BIN)
	rm -f libs/liblumax_*
	rm -f test/main_darwin test/main_linux test/main.exe
	rm -f test/lumax.h
	rm -f test/libs/* python/libs/*
