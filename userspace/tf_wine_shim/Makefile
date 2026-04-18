#
# Build the Wine PE shim DLL for the Windows Trueforce SDK ABI.
#
# Requires winegcc + winebuild (Fedora: wine-devel; Debian/Ubuntu:
# wine-dev). Produces trueforce_sdk.dll.so, a Wine-style PE .so that
# Wine loads when WINEDLLOVERRIDES="trueforce_sdk=n,b" redirects the
# Windows SDK name to our shim.
#
# The shim forwards every named export to libtrueforce.so (in
# ../libtrueforce). Build that first: (cd ../libtrueforce && make).
#

WINEGCC   ?= winegcc
DLL       := trueforce_sdk.dll.so
SPEC      := trueforce_sdk.spec
LIBTF_DIR := ../libtrueforce

CFLAGS    ?= -O2 -g -Wall -Wextra -Wno-unused-parameter
CPPFLAGS  ?= -I$(LIBTF_DIR)/include
LDFLAGS   ?= -m64 -L$(LIBTF_DIR)
LDLIBS    := -ltrueforce

SRCS := src/shim.c
OBJS := $(SRCS:.c=.o)

.PHONY: all clean

all: $(DLL)

$(DLL): $(OBJS) $(SPEC)
	$(WINEGCC) -shared -m64 $(SPEC) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@

%.o: %.c
	$(WINEGCC) -m64 $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(DLL) $(DLL:.dll.so=.dll) $(DLL:.dll.so=.exe.so) $(DLL:.dll.so=.dll.fake)
