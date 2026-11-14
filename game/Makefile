CELL_MK_DIR = $(CELL_SDK)/samples/mk
include $(CELL_MK_DIR)/sdk.makedef.mk

CELL_BUILD_TOOLS = GCC

PPU_AR = ppu-lv2-ar
PPU_RANLIB = ppu-lv2-ranlib
PPU_STRIP = ppu-lv2-strip
PPU_CC = ppu-lv2-gcc
PPU_CXX = ppu-lv2-g++
PPU_LD = ppu-lv2-ld
PPU_CCLD = ppu-lv2-gcc
PPU_CXXLD = ppu-lv2-g++
PPU_PRX_CCLD = ppu-lv2-gcc
PPU_PRX_CXXLD = ppu-lv2-g++
PPU_PRX_EXPORTPICKUP = ppu-lve-prx-exportpickup
PPU_PRX_STRIP = ppu-lv2-prx-strip

PPU_PRX_FLAGS = -mprx -mno-sn-ld -Os -ffunction-sections -fdata-sections -fno-builtin-printf -nodefaultlibs -std=gnu99 -Wno-shadow -Wno-unused-parameter
PPU_CFLAGS = $(PPU_PRX_FLAGS)
PPU_CXXFLAGS += $(PPU_PRX_FLAGS)
PPU_LDFLAGS += $(PPU_PRX_FLAGS)
PPU_PRX_LDFLAGS += -mprx -ffunction-sections -fdata-sections -Wl,--strip-unused-data

PPU_PRX_STRIPFLAGS += --strip-debug --strip-section-header

# libc.c (shared with the VSH plugin) provides memcpy/memset since the PRX is
# built with -nodefaultlibs. -lio_stub resolves the cellPadSetActDirect symbol
# we hook; the sys_event_port / sys_mutex / sys_prx calls are lv2 syscalls and
# need no stub library.
PPU_SRCS = ../src/libc.c xpad_game.c
PPU_PRX_LDLIBS = -lio_stub
PPU_PRX_TARGET = xpad_game.prx

CLEANFILES = $(PRX_DIR)/$(PPU_SPRX_TARGET)

# Signs a CFW-loadable SPRX. This mirrors the VSH plugin's signing; a module
# loaded into a game process by PS3MAPI LOAD_PROC_MODULE uses the same plain
# APP SELF format. Adjust the scetool key/auth flags if your CFW rejects it.
all:
	$(MAKE) $(PPU_OBJS_DEPENDS)
	$(PPU_PRX_STRIP) $(PPU_PRX_STRIPFLAGS) $(PPU_PRX_TARGET)
	scetool -0 SELF -1 TRUE -s FALSE -2 0A -3 1070000052000001 -4 01000002 -5 APP -6 0003004000000000 -A 0001000000000000 -e xpad_game.prx xpad_game.sprx

xpad_game.o: ../src/xpad_rumble.h

include $(CELL_MK_DIR)/sdk.target.mk
