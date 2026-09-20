# Built on the QNX Pi itself (the image ships clang); no SDP or cross-compiler.
CC = clang
CFLAGS ?= -O2 -Wall -Wextra
LDLIBS = -lsocket

all: lineguard camtap

lineguard: lineguard.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

# needs `apk add qnx-sf-base-dev` for camera/camera_api.h
camtap: camtap.c
	$(CC) $(CFLAGS) -o $@ $< -lcamapi

clean:
	rm -f lineguard camtap

.PHONY: all clean
