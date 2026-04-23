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
LDFLAGS   ?= -m64
# Link libtrueforce statically. The shim's .dll.so otherwise carries a
# DT_NEEDED for libtrueforce.so.1, which Proton's pressure-vessel can't
# resolve from the host's /usr/lib. Per-prefix install lives inside
# drive_c, so a self-contained shim is the simplest fix.
LDLIBS    := -lpthread

SRCS := src/shim.c
OBJS := $(SRCS:.c=.o)

LIBTF_STATIC := $(LIBTF_DIR)/libtrueforce.a

.PHONY: all clean

all: $(DLL)

$(LIBTF_STATIC):
	$(MAKE) -C $(LIBTF_DIR) libtrueforce.a

$(DLL): $(OBJS) $(SPEC) $(LIBTF_STATIC)
	$(WINEGCC) -shared -m64 $(SPEC) $(OBJS) $(LIBTF_STATIC) $(LDFLAGS) $(LDLIBS) -o $@

%.o: %.c
	$(WINEGCC) -m64 $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(DLL) $(DLL:.dll.so=.dll) $(DLL:.dll.so=.exe.so) $(DLL:.dll.so=.dll.fake)
