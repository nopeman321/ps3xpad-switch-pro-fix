# Host-side unit tests for the ps3xpad Switch Pro protocol code.
#
# These are plain native tests: no PS3 hardware, no Cell SDK, no signing.
# They run anywhere with a C compiler (host gcc/clang), including inside the
# ps3dev container used for the real PRX build.
#
#   make -C tests test     build and run
#   make -C tests run      run an already-built binary
#   make -C tests clean

CC      ?= cc
CFLAGS  ?= -std=c11 -O2
WARN     = -Wall -Wextra -pedantic
TARGETS  = test_switch_pro test_bt_dongle test_hen_integration

.PHONY: all test run clean

all: $(TARGETS)

test_switch_pro: test_switch_pro.c
	$(CC) $(CFLAGS) $(WARN) -o $@ $<

test_bt_dongle: test_bt_dongle.c
	$(CC) $(CFLAGS) $(WARN) -o $@ $<

test_hen_integration: test_hen_integration.c
	$(CC) $(CFLAGS) $(WARN) -o $@ $<

test: $(TARGETS)
	./test_switch_pro
	./test_bt_dongle
	./test_hen_integration

run: $(TARGETS)
	./test_switch_pro
	./test_bt_dongle
	./test_hen_integration

clean:
	rm -f $(TARGETS) test_switch_pro.exe test_bt_dongle.exe test_hen_integration.exe