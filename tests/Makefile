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
TARGET   = test_switch_pro
SRC      = test_switch_pro.c

.PHONY: all test run clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(WARN) -o $@ $<

test: $(TARGET)
	./$(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe