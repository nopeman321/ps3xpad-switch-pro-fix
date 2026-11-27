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

// Proper prototype for lv2 IPC event port connection to prevent implicit declaration
int32_t sys_event_port_connect_ipc(sys_event_port_t event_port_id, uint64_t ipc_key);

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
// PPC64 OPD-table detour helpers & pad hooks
//
// A game-agnostic detour: resolve the target function's real module OPD
// {entry, toc} via its .sceStub.text stub, save the original {entry, toc} into
// h->orig_opd, and atomically overwrite the target module's OPD with our C
// hook's {entry, toc}.
//
// Why OPD patching instead of .text patching:
//   1. Every caller in the process (EBOOT.BIN + all PRXs) dispatches through
//      the target module's OPD, so patching it redirects all callers.
//   2. A PRX's .opd lives in its RW- data segment, whereas .text is R-E
//      (write-protected on PS3HEN, causing a DSI fault if written via memcpy).
//   3. No code is generated in .bss (avoiding NX/ISI faults) and no icache
//      flush is needed.
// ---------------------------------------------------------------------------

// Standard PS3 DualShock 3 capability flags advertised for spoofed LDD ports
#define DS3_SPOOF_CAPABILITIES \
  (CELL_PAD_CAPABILITY_PS3_CONFORMITY | \
   CELL_PAD_CAPABILITY_PRESS_MODE | \
   CELL_PAD_CAPABILITY_SENSOR_MODE | \
   CELL_PAD_CAPABILITY_HP_ANALOG_STICK | \
   CELL_PAD_CAPABILITY_ACTUATOR)

typedef int32_t (*padact_fn)(uint32_t, CellPadActParam *);
typedef int32_t (*padgetinfo2_fn)(CellPadInfo2 *);
typedef int32_t (*padgetdataextra_fn)(uint32_t, uint32_t *, CellPadData *);

// Reusable hook state.
typedef struct {
  uint32_t *target_opd;                              // resolved module OPD in RW- .opd
  uint32_t  orig_opd[2] __attribute__((aligned(8))); // saved {orig_entry, orig_toc}
  int32_t   installed;
} pad_hook_t;

static pad_hook_t g_setActDirect_hook;
static pad_hook_t g_getInfo2_hook;
static pad_hook_t g_getDataExtra_hook;

// Resolve a target function pointer to its real module OPD {entry, toc}.
// In a PS3 PRX linked against *_stub, taking &imported_func yields the address
// of its .sceStub.text stub rather than an OPD:
//   li   r12, seg@l       (0x3980xxxx)
//   oris r12, r12, seg@h  (0x658Cxxxx)
//   lwz  r12, disp(r12)   (0x818Cxxxx)
// which loads the resolved module OPD pointer from .data.sceFStub.
// Returns NULL (fail-closed) if the pointer is NULL or the import is unresolved.
static uint32_t *resolve_target_opd(void *target_func) {
  uint32_t *p = (uint32_t *)target_func;

  if (p == NULL) {
    return NULL;
  }
  if ((p[0] & 0xFFFF0000u) == 0x39800000u &&
      (p[1] & 0xFFFF0000u) == 0x658C0000u &&
      (p[2] & 0xFFFF0000u) == 0x818C0000u) {
    uint32_t r12 = (uint32_t)(int32_t)(int16_t)(p[0] & 0xFFFFu);
    uint32_t stub_entry_addr;
    uint32_t *opd;

    r12 |= ((uint32_t)(p[1] & 0xFFFFu)) << 16;
    stub_entry_addr = r12 + (uint32_t)(int32_t)(int16_t)(p[2] & 0xFFFFu);
    if (stub_entry_addr == 0) {
      return NULL;
    }
    opd = *(uint32_t **)(uintptr_t)stub_entry_addr;
    if (opd == NULL || opd == p) {
      return NULL;
    }
    return opd;
  }
  return p;
}

// Write an {entry, toc} pair into an OPD slot (atomically when 8-byte aligned).
static void write_opd(uint32_t *dst_opd, uint32_t entry, uint32_t toc) {
  if (((uintptr_t)dst_opd & 7u) == 0) {
    *(volatile uint64_t *)dst_opd = ((uint64_t)entry << 32) | (uint64_t)toc;
  } else {
    dst_opd[0] = entry;
    dst_opd[1] = toc;
  }
  __asm__ __volatile__("sync" ::: "memory");
}

// Install a PPC64 OPD detour. Returns 0 on success, -1 on failure.
// Fails closed: on any error the target OPD is left untouched.
static int32_t install_pad_hook(pad_hook_t *h, void *target_func, void *hook_func) {
  uint32_t *tgt_opd, *hook_opd;

  if (h == NULL || target_func == NULL || hook_func == NULL) {
    return -1;
  }
  if (h->installed) {
    return 0;
  }

  tgt_opd  = resolve_target_opd(target_func);
  hook_opd = (uint32_t *)hook_func;
  if (tgt_opd == NULL || hook_opd == NULL) {
    return -1;
  }
  if (tgt_opd[0] == 0 || tgt_opd[1] == 0 || hook_opd[0] == 0 || hook_opd[1] == 0) {
    return -1;
  }

  h->target_opd  = tgt_opd;
  h->orig_opd[0] = tgt_opd[0];
  h->orig_opd[1] = tgt_opd[1];

  write_opd(tgt_opd, hook_opd[0], hook_opd[1]);

  h->installed = 1;
  return 0;
}

static void remove_pad_hook(pad_hook_t *h) {
  if (h != NULL && h->installed && h->target_opd != NULL) {
    write_opd(h->target_opd, h->orig_opd[0], h->orig_opd[1]);
    h->installed = 0;
  }
}

// Forward rumble actuator values, then call original cellPadSetActDirect.
static int32_t hook_padSetActDirect(uint32_t port_no, CellPadActParam *param) {
  forward_actuator(port_no, param);
  return ((padact_fn)(void *)g_setActDirect_hook.orig_opd)(port_no, param);
}

// Spoof connected LDD pads as standard DualShock 3 controllers in CellPadInfo2.
// Real hardware DS3/STANDARD, BD remote, and disconnected ports are untouched.
static int32_t hook_padGetInfo2(CellPadInfo2 *info) {
  int32_t ret = ((padgetinfo2_fn)(void *)g_getInfo2_hook.orig_opd)(info);
  if (ret == CELL_OK && info != NULL) {
    uint32_t i;
    for (i = 0; i < CELL_PAD_MAX_PORT_NUM; i++) {
      if ((info->port_status[i] & CELL_PAD_STATUS_CONNECTED) &&
          info->device_type[i] == CELL_PAD_DEV_TYPE_LDD) {
        info->device_type[i] = CELL_PAD_DEV_TYPE_STANDARD;
        info->device_capability[i] = DS3_SPOOF_CAPABILITIES;
      }
    }
  }
  return ret;
}

// Spoof LDD device_type reported by cellPadGetDataExtra as standard DS3.
static int32_t hook_padGetDataExtra(uint32_t port_no, uint32_t *device_type, CellPadData *data) {
  int32_t ret = ((padgetdataextra_fn)(void *)g_getDataExtra_hook.orig_opd)(port_no, device_type, data);
  if (ret >= 0 && device_type != NULL && *device_type == CELL_PAD_DEV_TYPE_LDD) {
    *device_type = CELL_PAD_DEV_TYPE_STANDARD;
  }
  return ret;
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
  // versus the VSH driver does not matter. Install the hooks now; each hook
  // fails closed independently if its target cannot be patched.
  install_pad_hook(&g_setActDirect_hook, (void *)cellPadSetActDirect, (void *)hook_padSetActDirect);
  install_pad_hook(&g_getInfo2_hook, (void *)cellPadGetInfo2, (void *)hook_padGetInfo2);
  install_pad_hook(&g_getDataExtra_hook, (void *)cellPadGetDataExtra, (void *)hook_padGetDataExtra);

  return SYS_PRX_RESIDENT;
}

int xpad_game_stop(void) {
  remove_pad_hook(&g_getDataExtra_hook);
  remove_pad_hook(&g_getInfo2_hook);
  remove_pad_hook(&g_setActDirect_hook);
  if (g_send_mutex_ok) {
    sys_mutex_lock(g_send_mutex, 0);
    rumble_disconnect();
    sys_mutex_unlock(g_send_mutex);
    sys_mutex_destroy(g_send_mutex);
    g_send_mutex_ok = 0;
  }
  return SYS_PRX_STOP_OK;
}
