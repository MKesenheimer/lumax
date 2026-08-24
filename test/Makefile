# Makefile for the C test program (main.c).
#
# Links against the liblumax library copied into test/libs - build it first
# from the parent directory:  make
#
# Targets:
#   make        build the test program (main_darwin / main_linux / main.exe)
#   make run    build and run it (the laser device must be connected)
#   make clean  remove the binary
#
# On Windows this expects a MinGW / MSYS2 environment (gcc, make, sh).

CC ?= gcc

# --- platform detection (mirrors the top-level Makefile) --------------------
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
TARGET  := main_darwin
LIB     := -llumax_darwin
LDFLAGS += -rpath @executable_path/libs
else ifeq ($(PLATFORM),linux)
TARGET  := main_linux
LIB     := -llumax_linux
LDFLAGS += -Wl,-rpath,'$$ORIGIN/libs'
else
TARGET  := main.exe
LIB     := -llumax_windows
endif

.PHONY: all run clean

all: $(TARGET)

$(TARGET): main.c lumax.h
	@test -d libs || { echo "error: libs/ not found - run 'make' in the parent directory first"; exit 1; }
	$(CC) $(CFLAGS) $(LDFLAGS) -I. -Llibs -lm $(LIB) main.c -o $@

run: $(TARGET)
ifeq ($(PLATFORM),windows)
	# next to the executable so the loader finds it
	cp -f libs/liblumax_windows.dll .
endif
	./$(TARGET)

clean:
	rm -f main_darwin main_linux main.exe liblumax_windows.dll
