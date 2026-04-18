PREFIX ?= /usr/local
LIBDIR ?= $(PREFIX)/lib
INCDIR ?= $(PREFIX)/include
UDEVDIR ?= /etc/udev/rules.d

CC ?= cc
CFLAGS ?= -O2 -g -Wall -Wextra -Wno-unused-parameter -fPIC
CPPFLAGS ?= -Iinclude
LDFLAGS ?=
LDLIBS := -lpthread

SRCS := src/discovery.c src/exports.c src/session.c src/stream.c
OBJS := $(SRCS:.c=.o)

SONAME := libtrueforce.so.1
FULL := $(SONAME).3.11
LIB  := libtrueforce.so

TESTS := tests/discover tests/init_session tests/sine

.PHONY: all clean install udev-install tests

all: $(FULL) tests

$(FULL): $(OBJS)
	$(CC) $(LDFLAGS) -shared -Wl,-soname,$(SONAME) -o $@ $^ $(LDLIBS)
	ln -sf $(FULL) $(SONAME)
	ln -sf $(SONAME) $(LIB)

%.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

tests: $(TESTS)

tests/discover: tests/discover.c $(FULL)
	$(CC) $(CFLAGS) $(CPPFLAGS) -Isrc -L. -o $@ $< -ltrueforce -Wl,-rpath,'$$ORIGIN/..'

tests/init_session: tests/init_session.c $(FULL)
	$(CC) $(CFLAGS) $(CPPFLAGS) -Isrc -L. -o $@ $< -ltrueforce -Wl,-rpath,'$$ORIGIN/..'

tests/sine: tests/sine.c $(FULL)
	$(CC) $(CFLAGS) $(CPPFLAGS) -L. -o $@ $< -ltrueforce -lm -Wl,-rpath,'$$ORIGIN/..'

install: all
	install -d $(DESTDIR)$(LIBDIR) $(DESTDIR)$(INCDIR)
	install -m 0644 $(FULL) $(DESTDIR)$(LIBDIR)/
	ln -sf $(FULL) $(DESTDIR)$(LIBDIR)/$(SONAME)
	ln -sf $(SONAME) $(DESTDIR)$(LIBDIR)/$(LIB)
	install -m 0644 include/trueforce.h $(DESTDIR)$(INCDIR)/

udev-install:
	install -d $(DESTDIR)$(UDEVDIR)
	install -m 0644 udev/99-logitech-rs50-trueforce.rules $(DESTDIR)$(UDEVDIR)/

clean:
	rm -f $(OBJS) $(FULL) $(SONAME) $(LIB) $(TESTS)
