// xpad_game.sprx - game-process half of ps3xpad's rumble support.
//
// The VSH driver (xpad.sprx) registers USB Xbox pads as virtual "LDD" pads.
// The OS never delivers cellPadSetActDirect() actuator (rumble) values to an
// LDD pad, so the driver cannot see them from VSH. This plugin is loaded into
// the game's own process (manually through webMAN MOD's "Game Plugins" page,
// or automatically by the VSH driver via PS3MAPI LOAD_PROC_MODULE). Inside the
// game process it hooks cellPadSetActDirect(), reads the motor values the game
// requested, and forwards them to the VSH driver over the named lv2 event
// queue defined in xpad_rumble.h. The driver then issues the USB rumble
// command to the pad on that port.
//
// Only the rumble path lives here; input is entirely handled by the VSH
// driver. If anything in this plugin fails, it fails closed - the game keeps
// running normally, just without rumble on xpad controllers.
//
// NOTE: the function-hook installer (install_hook / patch of the resolved
// cellPadSetActDirect entry) is the one piece that can only be validated on
// real hardware; see TODO.md section 8. The IPC contract, actuator mapping and
// module lifecycle below are self-contained and do not depend on it.

#include <stdint.h>
#include <string.h>
#include <sys/prx.h>
#include <sys/process.h>
#include <sys/event.h>
#include <sys/synchronization.h>
#include <cell/pad.h>
#include "../src/xpad_rumble.h"

SYS_MODULE_INFO(xpad_game, 0, 1, 0);
SYS_MODULE_START(xpad_game_start);
SYS_MODULE_STOP(xpad_game_stop);

int xpad_game_start(uint64_t arg);
int xpad_game_stop(void);

// ---------------------------------------------------------------------------
// Rumble IPC sender (game process -> VSH driver)
// ---------------------------------------------------------------------------

static sys_event_port_t g_port;
static int32_t          g_connected;      // 1 once the port is connected to the queue
static int32_t          g_send_fails;     // consecutive send failures on g_port
static sys_mutex_t      g_send_mutex;     // serializes (re)connect + send
static int32_t          g_send_mutex_ok;

// tear down and reconnect after this many consecutive send failures - handles
// the driver being unloaded/reloaded without churning on a transiently full
// queue (a full queue just drops the update and is retried on the next change)
#define RUMBLE_MAX_SEND_FAILS 4

// last values forwarded per pad port, so we only send on change (the game may
// call cellPadSetActDirect every frame with the same values)
static uint8_t g_last_l[CELL_PAD_MAX_PORT_NUM];
static uint8_t g_last_r[CELL_PAD_MAX_PORT_NUM];
static uint8_t g_have_last[CELL_PAD_MAX_PORT_NUM];

// (re)connect an event port to the VSH driver's named queue. The driver
// creates the queue at boot, so this normally succeeds on the first try; if
// the driver is not loaded (or was reloaded) the connect fails and we simply
// retry on the next change. Caller holds g_send_mutex.
static int32_t rumble_connect(void) {
  int32_t r;

  if (g_connected) {
    return CELL_OK;
  }
  if ((r = sys_event_port_create(&g_port, SYS_EVENT_PORT_LOCAL, SYS_EVENT_PORT_NO_NAME)) != CELL_OK) {
    return r;
  }
  if ((r = sys_event_port_connect_ipc(g_port, RUMBLE_IPC_KEY)) != CELL_OK) {
    // queue not present yet - drop the port and try again next change
    sys_event_port_destroy(g_port);
    return r;
  }
  g_connected = 1;
  g_send_fails = 0;
  return CELL_OK;
}

static void rumble_disconnect(void) {
  if (g_connected) {
    g_connected = 0;
    g_send_fails = 0;
    sys_event_port_disconnect(g_port);
    sys_event_port_destroy(g_port);
  }
}

// forward one actuator update to the VSH driver. Best-effort and non-fatal:
// a full queue or a missing driver just drops the update.
static void rumble_send(int32_t port, uint8_t lval, uint8_t rval) {
  int32_t r;

  if (port < 0 || port >= CELL_PAD_MAX_PORT_NUM) {
    return;
  }
  if (!g_send_mutex_ok) {
    return;
  }
  if (sys_mutex_lock(g_send_mutex, 0) != CELL_OK) {
    return;
  }

  // skip unchanged values (per port) to keep the queue and the pad thread quiet
  if (g_have_last[port] && g_last_l[port] == lval && g_last_r[port] == rval) {
    sys_mutex_unlock(g_send_mutex);
    return;
  }

  if (rumble_connect() == CELL_OK) {
    r = sys_event_port_send(g_port, (uint64_t)port, (uint64_t)lval, (uint64_t)rval);
    if (r == CELL_OK) {
      g_send_fails = 0;
      g_last_l[port] = lval;
      g_last_r[port] = rval;
      g_have_last[port] = 1;
    } else if (++g_send_fails >= RUMBLE_MAX_SEND_FAILS) {
      // a transiently full queue just drops a few updates; persistent failures
      // mean the queue went away (driver unloaded), so reconnect to a fresh one
      rumble_disconnect();
    }
  }
  sys_mutex_unlock(g_send_mutex);
}

// translate a PS3 CellPadActParam into the (left,right) pair the xpad USB
// rumble commands expect. PS3 exposes motor[0] as the small (right,
// high-frequency) motor as an on/off flag and motor[1] as the large (left,
// low-frequency) motor as a 0..255 magnitude. See xpad_rumble.h.
static void forward_actuator(uint32_t port_no, const CellPadActParam *param) {
  uint8_t lval, rval;

  if (param == NULL) {
    return;
  }
  lval = param->motor[1];               // large / left, already 0..255
  rval = param->motor[0] ? 0xFF : 0x00; // small / right is on/off on PS3
  rumble_send((int32_t)port_no, lval, rval);
}

// ---------------------------------------------------------------------------
// cellPadSetActDirect hook
//
// A game-agnostic detour: overwrite the first few instructions of the resolved
// cellPadSetActDirect entry with a jump to our thunk, so every caller in the
// process is redirected regardless of which module issued the call. The thunk
// fixes up the TOC (r2) for our C hook; the trampoline replays the displaced
// instructions and jumps back so the original still runs.
// ---------------------------------------------------------------------------

typedef int32_t (*padact_fn)(uint32_t, CellPadActParam *);

#define STEAL 4 // instructions displaced from the target prologue

static uint32_t *g_target;              // resolved cellPadSetActDirect entry (code)
static uint32_t  g_saved[STEAL];        // original instructions, for restore
// g_thunk/g_tramp hold generated code executed via bctr. They live in the
// module's data; PS3 lv2 processes execute data/heap freely, but if a target
// CFW enforces W^X these two need to move to exec memory - the one seam beyond
// the code-write below that a hardware bring-up may have to adjust.
static uint32_t  g_thunk[8];            // entry fixup: set our TOC, jump to hook
static uint32_t  g_tramp[16];           // set orig TOC, replay prologue, jump back
static uint32_t  g_orig_opd[2] __attribute__((aligned(8))); // {g_tramp, orig TOC}
static int32_t   g_hook_installed;

// emit a 4-instruction absolute jump to 'addr' using the volatile r0:
//   lis r0,addr@h ; ori r0,r0,addr@l ; mtctr r0 ; bctr
// r0 is scratch under the PPC64 ABI so this is safe at a function entry. Valid
// for PS3 user addresses (< 0x8000_0000, so lis does not sign-extend high).
static void emit_jump_r0(uint32_t *out, uint32_t addr) {
  out[0] = 0x3C000000u | (addr >> 16);    // lis   r0, addr@h
  out[1] = 0x60000000u | (addr & 0xFFFF); // ori   r0, r0, addr@l
  out[2] = 0x7C0903A6u;                   // mtctr r0
  out[3] = 0x4E800420u;                   // bctr
}

// emit lis/ori loading 'val' into r2 (the TOC pointer) at out[0..1]
static void emit_load_r2(uint32_t *out, uint32_t val) {
  out[0] = 0x3C400000u | (val >> 16);     // lis r2, val@h
  out[1] = 0x60420000u | (val & 0xFFFF);  // ori r2, r2, val@l
}

// flush the instruction cache over a range we just wrote code into - the
// correct data-cache-store / instruction-cache-invalidate sequence for
// self-modifying code on PPC.
static void icache_flush(void *addr, int32_t len) {
  uint8_t *p = (uint8_t *)addr;
  int32_t i;

  for (i = 0; i < len; i += 4) {
    __asm__ __volatile__("dcbst 0,%0" :: "r"(p + i) : "memory");
  }
  __asm__ __volatile__("sync" ::: "memory");
  for (i = 0; i < len; i += 4) {
    __asm__ __volatile__("icbi 0,%0" :: "r"(p + i) : "memory");
  }
  __asm__ __volatile__("isync" ::: "memory");
}

// The hook. Runs with the correct TOC (set by g_thunk). Forwards the rumble
// values, then calls the original through the trampoline so the game's pad
// handling is unchanged.
static int32_t hook_padSetActDirect(uint32_t port_no, CellPadActParam *param) {
  forward_actuator(port_no, param);
  return ((padact_fn)(void *)g_orig_opd)(port_no, param);
}

// install the detour. Returns 0 on success. Fails closed: on any error the
// original function is left untouched and rumble is simply unavailable.
static int32_t install_hook(void) {
  uint32_t *tgt_opd, *hook_opd;
  uint32_t  orig_toc, our_toc, hook_entry;

  // OPDs: a PS3 function pointer is {entry, toc}. Our resolved import gives us
  // the real cellPadSetActDirect entry/TOC; &hook gives our own.
  tgt_opd  = (uint32_t *)(void *)cellPadSetActDirect;
  hook_opd = (uint32_t *)(void *)hook_padSetActDirect;
  if (tgt_opd == NULL || hook_opd == NULL) {
    return -1;
  }
  g_target   = (uint32_t *)tgt_opd[0];
  orig_toc   = tgt_opd[1];
  hook_entry = hook_opd[0];
  our_toc    = hook_opd[1];
  if (g_target == NULL) {
    return -1;
  }

  // save the instructions we are about to overwrite (for the trampoline and
  // for restore on unload)
  memcpy(g_saved, g_target, sizeof(g_saved));

  // thunk: land here from the patched entry, set our TOC, jump to the C hook
  emit_load_r2(g_thunk, our_toc);
  emit_jump_r0(g_thunk + 2, hook_entry);

  // trampoline (the "original"): set the original TOC, replay the displaced
  // prologue, then jump back into the function just past the patch
  emit_load_r2(g_tramp, orig_toc);
  memcpy(g_tramp + 2, g_saved, sizeof(g_saved));
  emit_jump_r0(g_tramp + 2 + STEAL, (uint32_t)(uintptr_t)(g_target + STEAL));
  g_orig_opd[0] = (uint32_t)(uintptr_t)g_tramp;
  g_orig_opd[1] = orig_toc;

  // publish the thunk/trampoline code before anything can branch into it
  // (g_orig_opd is plain data read by the call sequence, so it needs no flush)
  icache_flush(g_thunk, sizeof(g_thunk));
  icache_flush(g_tramp, sizeof(g_tramp));

  // redirect the entry to the thunk. This writes into the resolved function's
  // code; on a CFW that allows in-process code writes (the norm - the plugin
  // itself was injected the same way) this takes effect immediately.
  {
    uint32_t patch[STEAL];
    emit_jump_r0(patch, (uint32_t)(uintptr_t)g_thunk);
    memcpy(g_target, patch, sizeof(patch));
    icache_flush(g_target, sizeof(patch));
  }

  g_hook_installed = 1;
  return 0;
}

static void remove_hook(void) {
  if (g_hook_installed && g_target != NULL) {
    memcpy(g_target, g_saved, sizeof(g_saved));
    icache_flush(g_target, sizeof(g_saved));
    g_hook_installed = 0;
  }
}

// ---------------------------------------------------------------------------
// Module lifecycle
// ---------------------------------------------------------------------------

int xpad_game_start(uint64_t arg) {
  sys_mutex_attribute_t mattr;
  (void)arg;

  memset(g_have_last, 0, sizeof(g_have_last));
  sys_mutex_attribute_initialize(mattr);
  g_send_mutex_ok = (sys_mutex_create(&g_send_mutex, &mattr) == CELL_OK) ? 1 : 0;

  // connect lazily on the first actuator change (rumble_send), so load order
  // versus the VSH driver does not matter. Install the hook now.
  install_hook();

  return SYS_PRX_RESIDENT;
}

int xpad_game_stop(void) {
  remove_hook();
  if (g_send_mutex_ok) {
    sys_mutex_lock(g_send_mutex, 0);
    rumble_disconnect();
    sys_mutex_unlock(g_send_mutex);
    sys_mutex_destroy(g_send_mutex);
    g_send_mutex_ok = 0;
  }
  return SYS_PRX_STOP_OK;
}
