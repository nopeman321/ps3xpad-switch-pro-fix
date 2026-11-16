#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ppu_thread.h>
#include <sys/process.h>
#include <sys/prx.h>
#include <sys/synchronization.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <sys/types.h>
#include <sys/time.h>
#include <cell/sysmodule.h>
#include <cell/pad.h>
#include <cell/pad/libpad_dbg.h>
#include <cell/usbd.h>
#include <cell/cell_fs.h>
#include "ControlStruct.h"
#include "xpad_rumble.h"
#include "bt_dongle.h"

#define THREAD_NAME "xpaddt"
#define STOP_THREAD_NAME "xpadds"
#define INJECT_THREAD_NAME "xpaddi"
#define MAX_XPAD_DEV_NUM ((int32_t)(sizeof(xpad_info) / sizeof(xpad_info[0])))
#define MAX_XPADW_DEV_NUM ((int32_t)(sizeof(xpadw_info) / sizeof(xpadw_info[0])))
#define MAX_SWITCH_DEV_NUM ((int32_t)(sizeof(switch_info) / sizeof(switch_info[0])))
#define MAX_XPAD_NUM CELL_PAD_MAX_PORT_NUM
#define MAX_XPADW_NUM 4 // endpoint units on an Xbox 360 wireless receiver (1 per paired pad)
#define MAX_UNIT_NUM (MAX_XPAD_NUM + MAX_XPADW_NUM)
#define XPAD_DATA_LEN 14+2 // +2 for count and size fields
#define XPADW_DATA_LEN 0x13+2
// Largest wMaxPacketSize we accept. Xbox 360 pads (wired and the wireless
// receiver) report 32; the Switch Pro full-mode report is 54 bytes, so this
// must cover both or swpro reports would be truncated before parsing.
#define MAX_XPAD_PAYLOAD 64
#define MAX_XPAD_DATA_LEN (MAX_XPAD_PAYLOAD+2) // +2 for count and size fields
// Largest LED/rumble command we send: 12 bytes for wireless Xbox rumble, 17 for
// the Switch Pro rumble-only report (id + 8 left + 8 right).
#define XPAD_OUT_LEN 17
#define RINGBUF_SIZE  10
#define XPAD_DEVICES_PATH "/dev_hdd0/plugins/xpad_devices.txt"
// debug log; low-volume lifecycle events only (registration, attach/detach,
// transfer errors, game-process injection), see xlog(). Rotated on every
// plugin start (see xlog_init): xpad.log -> xpad.log.1 -> xpad.log.2, so the
// two previous boots are preserved. This matters because a console that
// freezes when launching a game gets rebooted by the user, and without
// rotation that reboot would truncate the very log that captured the freeze.
#define XPAD_LOG_PATH   "/dev_hdd0/plugins/xpad.log"
#define XPAD_LOG_PATH_1 "/dev_hdd0/plugins/xpad.log.1"
#define XPAD_LOG_PATH_2 "/dev_hdd0/plugins/xpad.log.2"
// cellUsbd error codes (cell/usbd_error.h). Earlier revisions of this plugin
// mislabeled 0x80110006 as "not initialized" and retried the whole
// registration pass on it, which can never succeed: 0x80110006 actually
// means an identical LDD is already registered (duplicate name or VID/PID -
// e.g. from a partial earlier pass, a duplicate table entry, or another
// plugin claiming the same device).
#define CELL_USBD_ERROR_NOT_INITIALIZED        ((int32_t)0x80110001)
#define CELL_USBD_ERROR_LDD_ALREADY_REGISTERED ((int32_t)0x80110006)
// 0x801100FF is returned by cellUsbdRegisterExtraLdd() once cellUsbd's
// fixed-size LDD table is full (retail hardware accepts only ~22 entries,
// shared across every plugin on the system). This built-in device table is
// larger than that budget, so hitting it is EXPECTED, not fatal: the entries
// that registered keep working and the plugin stays active - it just cannot
// claim any further VID/PIDs. The device table is ordered so the controllers
// people actually use register before the budget runs out (see xpad_info).
#define CELL_USBD_ERROR_LDD_TABLE_FULL         ((int32_t)0x801100FF)

// Rumble IPC (game process -> this VSH plugin). The key, queue depth and wire
// format are defined in xpad_rumble.h, shared with the game-side sender so the
// two plugins can never disagree on them. See TODO.md section 8 for the design.
#define RUMBLE_DRAIN_MAX 16     // events drained per poll tick

// Auto-injection of the game-process plugin (xpad_game.sprx) via PS3MAPI.
// The PS3MAPI syscall-8 opcode/argument ABI below was verified against the
// authoritative Cobra/Mamba kernel source (aldostools/Mamba stage2/
// ps3mapi_core.{h,c}) and the ArtemisPS3 userland reference (ps3mapi_ps3_lib.c):
// the syscall-8 dispatch opcode, the syscall variant/argument order, and the
// kernel buffer sizes (MAX_PROCESS 16, MAX_FILE_LEN 256) all match. The earlier
// research-pass values for GET_ALL_PROC_PID (0x0011) and GET_PROC_NAME_BY_PID
// (0x0013) were WRONG and have been corrected to 0x0021 / 0x0022.
//
// LAUNCH-FREEZE FIX — the ABI is correct; the injection *timing* was the bug.
// An earlier enable-by-default froze the console on every game launch. The
// rotated debug log pinned it: the poll thread caught the game process the
// instant multiMAN spawned it and injected immediately —
//
//   [66.648] auto-inject: loading xpad_game.sprx into ..._main_EBOOT.BIN...
//   <no matching "loaded OK" - console frozen on a black screen from here on>
//
// ps3mapi_load_module() for the game EBOOT never returned. Loading a module
// into a process that is still booting (before it has mapped its own segments,
// resolved its prx imports and become schedulable) hangs the kernel prx loader,
// and xpad_game.sprx's start hook then runs against a half-constructed process.
// A manual webMAN "Game Plugins" injection never hits this because by the time
// a human triggers it the game has long since settled into running.
//
// The fix reproduces that safe timing automatically: try_auto_inject() no
// longer injects a freshly-seen process. It waits until a candidate has been
// observed alive across XPAD_INJECT_SETTLE_POLLS consecutive polls — i.e. the
// game has finished booting and is running — before loading the module. The
// same delay also skips multiMAN's transient loader helpers (prepNTFS.self,
// RELOAD.SELF), which exit before they reach the threshold. Input is unaffected
// either way; rumble just begins a few seconds into the game.
//
// INPUT-DEATH FIX — the settle delay stopped the console freeze, but a second
// field log showed the pad still going dead the moment a game starts:
//
//   [86.496] auto-inject: loading xpad_game.sprx into ..._main_EBOOT.BIN...
//   <no "loaded OK" and no "load_module failed" — but the console kept running>
//
// ps3mapi_load_module() hung in the kernel again (it can, even against a
// settled process, depending on CFW/game), and try_auto_inject() used to run
// INLINE on the 10 ms poll thread — the very thread that drains the USB ring
// buffers into cellPadLddDataInsert(). A hung injection therefore no longer
// froze the console, it silently froze THIS PLUGIN's input pump instead, which
// the user experiences as "the controller stops working when the game starts".
//
// Injection therefore runs on its own dedicated low-priority thread now. If
// ps3mapi_load_module() hangs, only that expendable thread blocks: input keeps
// flowing and only rumble auto-setup is lost for the session. The poll thread
// additionally watchdogs the injection thread (see inject_call_t0) and drops a
// breadcrumb in the log when an injection call has been stuck for a while.
//
// LOADER-WEDGE FIX (auto-injection now OPT-IN, default off) — the dedicated
// thread saved the input pump, but a third field report showed the real cost
// of a hung LOAD_PROC_MODULE: after the first injection attempt wedged, the
// console froze on STARTING ANYTHING — multiMAN, games, all of it. The thread
// the plugin sacrifices is expendable; the kernel state it is wedged in is
// not. The hung syscall sits inside lv2's process/PRX loader, and every
// subsequent launch (any process start, any module load) funnels through that
// same loader path and blocks behind it forever. So on a CFW where
// LOAD_PROC_MODULE hangs even post-settle - which this console demonstrably
// is - no amount of plugin-side thread isolation can make auto-injection
// safe: the only safe move is not to issue the syscall at all.
//
// Auto-injection is therefore OPT-IN at runtime: the injection thread is only
// started when the user has created XPAD_INJECT_OPT_IN_PATH (content
// irrelevant, existence is the switch). Without it the plugin makes no
// PS3MAPI calls whatsoever and rumble is set up manually instead via webMAN
// MOD's "Game Plugins" page, which injects xpad_game.sprx long after the game
// has settled and under the user's control. Users whose CFW handles
// LOAD_PROC_MODULE fine can restore the old convenience by creating the file.
#define XPAD_AUTO_INJECT 1
#define XPAD_GAME_SPRX_PATH "/dev_hdd0/plugins/xpad_game.sprx"
// existence of this file enables auto-injection (see LOADER-WEDGE FIX above)
#define XPAD_INJECT_OPT_IN_PATH "/dev_hdd0/plugins/xpad_auto_inject.txt"

#if XPAD_AUTO_INJECT
// Cobra/Mamba syscall-8 dispatch and PS3MAPI opcodes (see TODO.md section 8).
// Values verified against aldostools/Mamba stage2/ps3mapi_core.h.
#define SYSCALL8_OPCODE_PS3MAPI            0x7777ULL
#define PS3MAPI_OPCODE_GET_ALL_PROC_PID    0x0021ULL
#define PS3MAPI_OPCODE_GET_PROC_NAME_BY_PID 0x0022ULL
#define PS3MAPI_OPCODE_LOAD_PROC_MODULE    0x0044ULL
#define PS3MAPI_MAX_PROC 16 // == Mamba MAX_PROCESS; kernel fills exactly this many
#endif
#define MAX_EXTRA_DEV_NUM 24
#define EXTRA_NAME_LEN 48
#define DESCRIPTOR_TABLE_SIZE (sizeof(descriptor_table)/sizeof(descriptor_table_t))
#define SWAP16(x) ((uint16_t)((((x) & 0x00FF) << 8) | (((x) & 0xFF00) >> 8)))
// some Cell SDK versions misspell UsbInterfaceDescriptor's subclass field
// ("bInterfaceSublass"), so read it by its USB-spec offset instead of by name
#define USB_IF_SUBCLASS(ifd) (((const uint8_t *)(ifd))[6])

enum XTYPES {
  XTYPE_XBOX360 = 1,
  XTYPE_XBOX360W = 2,
  XTYPE_SWITCH_PRO = 3,
  XTYPE_BTD = 4
};

// ---------------------------------------------------------------------------
// Nintendo Switch Pro controller (USB, Nintendo protocol)
//
// Protocol reference: SWITCH_PRO_PROTOCOL.md, derived from
// BetterJoy/BetterJoyForCemu/Joycon.cs. This is NOT XInput - a Switch pad in
// USB mode speaks Nintendo's own HID protocol with 0x01/0x10 output reports and
// 0x30/0x21/0x3F input reports.
// ---------------------------------------------------------------------------
#define SWPRO_OUT_SUBCOMMAND  0x01 // subcommand + arguments
#define SWPRO_OUT_RUMBLE      0x10 // rumble only
#define SWPRO_IN_SIMPLE       0x3F // 12 bytes, initial mode, no IMU
#define SWPRO_IN_FULL         0x30 // full mode + IMU, 60 Hz
#define SWPRO_IN_SUBCMD_REPLY 0x21 // subcommand ack + input

#define SWPRO_SC_SET_MODE     0x03 // arg 0x30 = full mode, 60 Hz
#define SWPRO_SC_ENABLE_IMU   0x40 // arg 0x01
#define SWPRO_SC_ENABLE_VIB   0x48 // arg 0x01
#define SWPRO_SC_SET_PLAYER   0x30 // arg = LED pattern
#define SWPRO_SC_SPI_READ     0x10 // args: 0x80/0x60 + address

#define SWPRO_OUT_REPORT_LEN 16 // id + counter + 8 rumble + subcmd + args
#define SWPRO_FULL_REPORT_LEN 54
#define SWPRO_IMU_FRAME_BYTES 12
#define SWPRO_IMU_FRAMES      3

// button bytes of the 0x30 report sit at offset 3 and are three packed bitfields
#define SWPRO_BTN_RIGHT_CLUSTER 3
#define SWPRO_BTN_SHARED        4
#define SWPRO_BTN_LEFT_CLUSTER  5

// right cluster (byte 3)
#define SWPRO_BTN_Y     (1 << 0)
#define SWPRO_BTN_X     (1 << 1)
#define SWPRO_BTN_B     (1 << 2)
#define SWPRO_BTN_A     (1 << 3)
#define SWPRO_BTN_R     (1 << 6)
#define SWPRO_BTN_ZR    (1 << 7)

// shared cluster (byte 4)
#define SWPRO_BTN_MINUS   (1 << 0)
#define SWPRO_BTN_PLUS    (1 << 1)
#define SWPRO_BTN_RSTICK  (1 << 2)
#define SWPRO_BTN_LSTICK  (1 << 3)

// left cluster (byte 5)
#define SWPRO_BTN_DOWN  (1 << 0)
#define SWPRO_BTN_UP    (1 << 1)
#define SWPRO_BTN_RIGHT (1 << 2)
#define SWPRO_BTN_LEFT  (1 << 3)
#define SWPRO_BTN_L     (1 << 6)
#define SWPRO_BTN_ZL    (1 << 7)

// Neutral rumble: low = 0x00, high = 0x01 for both motors. Sending anything
// else makes the pad vibrate continuously on some third-party models, so this
// is what every report carries until a real rumble value arrives.
static const uint8_t swpro_rumble_neutral[8] = {0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};

// Full-mode input report (0x30), 54 bytes.
//
//   [0]      report id
//   [1..2]   global counter
//   [3..5]   buttons, three packed bitfields
//   [6..12]  unused in this mode
//   [13..48] IMU, 3 frames of 12 bytes (accel xyz + gyro xyz, LE int16)
//   [49..52] sticks: left X, left Y, right X, right Y
//   [53]     trailing byte
typedef struct {
  uint8_t report_id;
  uint8_t counter[2];
  uint8_t buttons[10];
  uint8_t imu[SWPRO_IMU_FRAMES][SWPRO_IMU_FRAME_BYTES];
  uint8_t stick_lx, stick_ly, stick_rx, stick_ry;
  uint8_t trailing;
} PACKED SWPRO_FULL_REPORT_t;

// Compile-time proof the struct matches the wire format. An earlier revision of
// SWITCH_PRO_PROTOCOL.md claimed 16-byte IMU frames; that overruns the 54-byte
// report, so these offsets are the corrected ones (see the document).
#define SWPRO_OFF_BUTTONS 3
#define SWPRO_OFF_IMU     13
#define SWPRO_OFF_STICKS  49

typedef struct xpad_device {
	uint16_t vid;
	uint16_t pid;
	const char *name;
} XPAD_INFO_t;

// defined below, next to the wireless table
static XPAD_INFO_t switch_info[];

// xpad device info from linux xpad driver.
//
// ORDER MATTERS. cellUsbd holds registered LDDs in a fixed-size table and
// runs out of slots after a couple dozen entries (retail hardware has been
// observed to accept ~22 before cellUsbdRegisterExtraLdd() returns
// CELL_USBD_ERROR_LDD_TABLE_FULL, 0x801100FF). This table has more entries
// than that, so entries past the budget are silently skipped (non-fatal, see
// register_one()). The controllers this plugin is most used with - the
// standard Xbox 360 pad, the Logitech wired pads, and especially the modern
// 8BitDo / GameSir 2.4GHz dongles - are therefore listed FIRST so they always
// register within the budget. The long tail of legacy arcade sticks, guitars
// and clones follows and is what gets dropped when the table fills up.
static XPAD_INFO_t xpad_info[] = {
	{0x045e, 0x028e, "Microsoft X-Box 360 pad"},
	{0x046d, 0xc21d, "Logitech Gamepad F310"},
	{0x046d, 0xc21e, "Logitech Gamepad F510"},
	{0x046d, 0xc21f, "Logitech Gamepad F710"},
	{0x046d, 0xc242, "Logitech Chillstream Controller"},
	// 8BitDo pads/dongles in X mode speak the plain wired XInput protocol,
	// even the 2.4GHz dongles - they are NOT the Microsoft wireless receiver.
	// Listed high up so this plugin's headline controllers always fit within
	// the cellUsbd LDD budget (see the ORDER MATTERS note above).
	// NOTE: every name in these tables doubles as the USBD LDD name and must
	// be unique - registering two LDDs with the same name fails with
	// CELL_USBD_ERROR_LDD_ALREADY_REGISTERED (0x80110006)
	{0x2dc8, 0x301c, "8BitDo Ultimate 2C Wireless (2.4GHz dongle)"},
	{0x2dc8, 0x3106, "8BitDo Ultimate Wireless / Pro 2 Wired"},
	{0x2dc8, 0x3109, "8BitDo Ultimate Wireless (Bluetooth dongle)"},
	{0x2dc8, 0x310a, "8BitDo Ultimate 2C Wireless (2.4GHz dongle v2)"},
	{0x2dc8, 0x310b, "8BitDo Ultimate 2 Wireless (2.4GHz dongle)"},
	{0x2dc8, 0x6001, "8BitDo SN30 Pro"},
	// GameSir XInput pads; their 2.4GHz dongles also present as wired XInput
	{0x3537, 0x1004, "GameSir T4 Kaleid"},
	{0x3537, 0x100f, "GameSir Nova 2 Lite"},
	{0x044f, 0xb326, "Thrustmaster Gamepad GP XID"},
	{0x0738, 0x4716, "Mad Catz Wired Xbox 360 Controller" },
	{0x0738, 0x4718, "Mad Catz Street Fighter IV FightStick SE"},
	{0x0738, 0x4728, "Mad Catz Street Fighter IV FightPad"},
	{0x0738, 0x4736, "Mad Catz MicroCon Gamepad"},
	{0x0738, 0x4738, "Mad Catz Wired Xbox 360 Controller (SFIV)"},
	{0x0738, 0xbeef, "Mad Catz JOYTECH NEO SE Advanced GamePad"},
	{0x0e6f, 0x0113, "Afterglow AX.1 Gamepad for Xbox 360"},
	{0x0e6f, 0x011f, "Rock Candy Gamepad Wired Controller"},
	{0x0e6f, 0x0201, "Pelican PL-3601 'TSZ' Wired Xbox 360 Controller"},
	{0x0e6f, 0x0213, "Afterglow Gamepad for Xbox 360" },
	{0x0e6f, 0x0301, "Logic3 Controller"},
	{0x0e6f, 0x0401, "Logic3 Controller (0401)"},
	{0x0e6f, 0x0501, "PDP Xbox 360 Controller"},
	{0x0e6f, 0xf900, "PDP Afterglow AX.1"},
	{0x0f0d, 0x000d, "Hori Fighting Stick EX2"},
	{0x0f0d, 0x0016, "Hori Real Arcade Pro.EX"},
	{0x0f0d, 0x001b, "Hori Real Arcade Pro VX"},
	{0x11c9, 0x55f0, "Nacon GC-100XF"},
	{0x12ab, 0x0301, "PDP AFTERGLOW AX.1 (12ab)"},
	{0x1430, 0x4748, "RedOctane Guitar Hero X-plorer"},
	{0x1430, 0xf801, "RedOctane Controller"},
	{0x146b, 0x0601, "BigBen Interactive XBOX 360 Controller"},
	{0x1532, 0x0037, "Razer Sabertooth"},
	{0x15e4, 0x3f00, "Power A Mini Pro Elite"},
	{0x15e4, 0x3f0a, "Xbox Airflo wired controller"},
	{0x162e, 0xbeef, "Joytech Neo-Se Take2"},
	{0x1689, 0xfd00, "Razer Onza Tournament Edition"},
	{0x1689, 0xfd01, "Razer Onza Classic Edition"},
	// same product under a second VID/PID; the "(hex)" suffixes keep LDD
	// names unique (see the 0x80110006 note below)
	{0x1689, 0xfe00, "Razer Sabertooth (fe00)"},
	{0x1bad, 0x0003, "Harmonix Rock Band Drumkit"},
	{0x1bad, 0xf016, "Mad Catz Xbox 360 Controller"},
	{0x1bad, 0xf028, "Street Fighter IV FightPad"},
	{0x1bad, 0xf038, "Street Fighter IV FightStick TE"},
	{0x1bad, 0xf900, "Harmonix Xbox 360 Controller"},
	{0x1bad, 0xf901, "Gamestop Xbox 360 Controller"},
	{0x1bad, 0xf903, "Tron Xbox 360 controller"},
	{0x24c6, 0x5000, "Razer Atrox Arcade Stick"},
	{0x24c6, 0x5300, "PowerA MINI PROEX Controller"},
	{0x24c6, 0x530a, "Xbox 360 Pro EX Controller"},
	{0x24c6, 0x531a, "PowerA Pro Ex"},
	{0x24c6, 0x5397, "FUS1ON Tournament Controller"},
	{0x24c6, 0x5500, "Hori XBOX 360 EX 2 with Turbo"},
	{0x24c6, 0x5501, "Hori Real Arcade Pro VX-SA"},
	{0x24c6, 0x5503, "Hori Fighting Edge"},
	{0x24c6, 0x5b02, "Thrustmaster GPX Controller"},
	{0x24c6, 0x5d04, "Razer Sabertooth (5d04)"},
	{0x24c6, 0xfafe, "Rock Candy Gamepad for Xbox 360"},
};

static XPAD_INFO_t xpadw_info[] = {
 {0x045e, 0x0291, "Xbox 360 Wireless Receiver (XBOX)"},
 {0x045e, 0x0719, "Xbox 360 Wireless Receiver"},
};

// Nintendo Switch Pro controllers in USB mode. These speak Nintendo's own HID
// protocol (SWITCH_PRO_PROTOCOL.md), not XInput, so they need their own LDD
// registration and their own report parsing. Only the plain USB Pro Controller
// is listed: 8BitDo pads in Switch mode use the same protocol but their VID/PID
// varies per firmware, so users should add them through xpad_devices.txt with
// XTYPE_SWITCH_PRO rather than have them fail to register here.
static XPAD_INFO_t switch_info[] = {
 {0x057e, 0x2009, "Nintendo Switch Pro Controller"},
};

// user-supplied devices loaded from XPAD_DEVICES_PATH at plugin start
typedef struct {
	uint16_t vid;
	uint16_t pid;
	uint8_t xtype;
	char name[EXTRA_NAME_LEN];
} XPAD_EXTRA_t;

static XPAD_EXTRA_t extra_info[MAX_EXTRA_DEV_NUM];
static int32_t extra_count = 0;
static int32_t extra_skipped = 0; // device list lines rejected by the parser

// which table entries currently have an extra LDD registered. This makes
// register_devices() idempotent (a retry pass must not re-register entries
// that already succeeded - re-registering fails with 0x80110006) and lets
// shutdown_usb() unregister exactly what was registered.
static uint8_t xpad_lddreg[sizeof(xpad_info) / sizeof(xpad_info[0])];
static uint8_t xpadw_lddreg[sizeof(xpadw_info) / sizeof(xpadw_info[0])];
static uint8_t switch_lddreg[sizeof(switch_info) / sizeof(switch_info[0])];
static uint8_t extra_lddreg[MAX_EXTRA_DEV_NUM];
static int32_t reg_conflicts;      // entries skipped: LDD already registered
static int32_t reg_budget_full;    // cellUsbd LDD table filled up mid-pass
static int32_t reg_skipped_full;   // entries left unregistered once it filled up
static uint16_t reg_fail_vid, reg_fail_pid; // entry a fatal error occurred on

typedef struct {
  uint8_t bDescriptorType;
  int32_t (*dump_descriptor)(int32_t dev_id, void *desc);
} descriptor_table_t;

typedef struct xpad_unit {
  int32_t dev_id; /* Device id */
  uint16_t vid; /* USB vendor id */
  uint16_t pid; /* USB product id */
  const char *name; /* Device name for notifications (points into a device table) */
  int32_t unit_idx; /* Index into XPAD.units[] */
  int32_t number; /* Pad slot (index into handle[]), -1 while no LDD pad is registered */
  int32_t last_port; /* Last known cellPadLddGetPortNo() result, -1 if unknown */
  int32_t c_pipe; /* Control pipe id */
  int32_t i_pipe; /* In pipe id */
  int32_t o_pipe; /* Out pipe id */
  int32_t payload; /* Size of payload */
  uint8_t ifnum; /* Interface number */
  uint8_t as; /* Alternate setting number */
  int32_t tcount; /* Transfer counts */
  uint8_t xtype;
  uint8_t is_bt;

  // methods to their respective controllers
  int32_t (*read_input)(struct xpad_unit *unit, void *data);
  int32_t (*set_led)(struct xpad_unit *unit, uint8_t led);
  int32_t (*set_rumble)(struct xpad_unit *unit, uint8_t lval, uint8_t rval);

  /* Ring buffer (one per unit so pads do not contend with each other) */
  sys_mutex_t rb_mutex; /* Guards rp/wp/rblen/ringbuf and the fields below */
  int32_t rp; /* Read pointer   */
  int32_t wp; /* Write pointer  */
  int32_t rblen; /* Buffer length  */
  unsigned char ringbuf[RINGBUF_SIZE][MAX_XPAD_DATA_LEN]; /* Ring buffer */

  /* Out-transfer buffer: cellUsbdInterruptTransfer() is asynchronous, so
     LED/rumble payloads must live here, not in the caller's stack frame.
     out_busy is set while a transfer using out_data is in flight. */
  uint8_t out_data[XPAD_OUT_LEN];
  uint8_t out_busy;

  /* Lifecycle guards: 'closing' stops completion callbacks from touching
     the unit or resubmitting transfers; 'pending' counts async USBD
     requests whose callbacks have not finished yet. unit_free() sets
     'closing', closes the pipes and waits for 'pending' to drain. */
  volatile int32_t pending;
  volatile uint8_t closing;

  /* Debug-log state: first successful input report logged once, transfer
     errors logged up to a small cap so a dying device cannot flood the log */
  uint8_t dbg_first_in;
  uint8_t dbg_err_count;

  /* Switch Pro state: the output-report counter wraps 0x00..0x0F and must
     advance once per subcommand sent (SWITCH_PRO_PROTOCOL.md section 3).
     swpro_inited latches the mode/IMU/vibration enable sequence so it is
     sent exactly once per attach. */
  uint8_t swpro_counter;
  uint8_t swpro_inited;
  uint8_t swpro_state;
  int32_t swpro_timer;
  uint32_t swpro_keepalive_timer;

  /* Buffer for interrupt transfer */
  unsigned char data[0];

} XPAD_UNIT_t;

typedef struct {
  int32_t n; /* Number of claimed pad slots */
  XPAD_UNIT_t *units[MAX_UNIT_NUM]; /* Every allocated endpoint unit (wired or wireless) */
  XPAD_UNIT_t *pad_unit[MAX_XPAD_NUM]; /* Which unit owns each pad slot, NULL = free */
} XPAD_t;

int xpadd_start(uint64_t arg);
int xpadd_stop(void);

// wired Xbox 360 controller methods
static int32_t xpad_probe(int32_t dev_id);
static int32_t xpad_attach(int32_t dev_id);
static int32_t xpad_detach(int32_t dev_id);
static int32_t xpad_detach_all(void);
static int32_t xpad_read_input(XPAD_UNIT_t *unit, void *data);
static void xpad_read_report(int32_t id, uint8_t *readBuf);
static int32_t xpad_set_led(XPAD_UNIT_t *unit, uint8_t led);
static int32_t xpad_set_rumble(XPAD_UNIT_t *unit, uint8_t lval, uint8_t rval);

// wireless Xbox 360 controller methods
static int32_t xpadw_probe(int32_t dev_id);
static int32_t xpadw_attach(int32_t dev_id);
static int32_t xpadw_detach(int32_t dev_id);
static int32_t xpadw_detach_all(void);
static int32_t xpadw_read_input(XPAD_UNIT_t *unit, void *data);
static void xpadw_read_report(int32_t id, uint8_t *readBuf);
static int32_t xpadw_set_led(XPAD_UNIT_t *unit, uint8_t led);
static int32_t xpadw_set_rumble(XPAD_UNIT_t *unit, uint8_t lval, uint8_t rval);

typedef enum {
  SWPRO_STATE_IDLE = 0,
  SWPRO_STATE_REQUEST_MAC,
  SWPRO_STATE_HANDSHAKE_1,
  SWPRO_STATE_BAUDRATE,
  SWPRO_STATE_HANDSHAKE_2,
  SWPRO_STATE_FORCE_HID,
  SWPRO_STATE_SET_MODE,
  SWPRO_STATE_ENABLE_IMU,
  SWPRO_STATE_ENABLE_VIB,
  SWPRO_STATE_SET_LED,
  SWPRO_STATE_DONE
} swpro_state_t;
// Nintendo Switch Pro controller methods
static int32_t swpro_probe(int32_t dev_id);
static int32_t swpro_attach(int32_t dev_id);
static int32_t swpro_detach(int32_t dev_id);
static int32_t swpro_detach_all(void);
static int32_t swpro_read_input(XPAD_UNIT_t *unit, void *data);
static void swpro_read_report(int32_t id, uint8_t *readBuf);
static int32_t swpro_set_led(XPAD_UNIT_t *unit, uint8_t led);
static int32_t swpro_set_rumble(XPAD_UNIT_t *unit, uint8_t lval, uint8_t rval);
static int32_t swpro_send_subcommand(XPAD_UNIT_t *unit, uint8_t subcmd, uint8_t arg);
static int32_t swpro_init(XPAD_UNIT_t *unit);

// common methods
static void data_transfer_done(int32_t result, int32_t count, void *arg);
static void data_transfer(XPAD_UNIT_t *unit);
static void set_config_done(int32_t result, int32_t count, void *arg);
static void set_interface_done(int32_t result, int32_t count, void *arg);
static XPAD_UNIT_t *unit_alloc(int32_t dev_id, int32_t payload, uint8_t ifnum, uint8_t as, uint8_t xtype);
static void unit_free(XPAD_UNIT_t *unit);
static int32_t check_pad_status(void);
static int32_t register_ldd_controller(XPAD_UNIT_t *unit);
static int32_t unregister_ldd_controller(XPAD_UNIT_t *unit);

// rumble IPC
static int32_t create_rumble_queue(void);
static void destroy_rumble_queue(void);
static void route_rumble(int32_t port, uint8_t lval, uint8_t rval);
static void drain_rumble_queue(void);
#if XPAD_AUTO_INJECT
static void try_auto_inject(void);
#endif

// notifications
static void notify_pad_connected(XPAD_UNIT_t *unit);
static void notify_pad_disconnected(XPAD_UNIT_t *unit);

// debug logging to XPAD_LOG_PATH (no-ops if the log file cannot be created)
void xlog(const char *text);
void xlog_code(const char *what, int32_t code);
void xlog_dev(const char *what, uint16_t vid, uint16_t pid, int32_t code);
static void xlog_proc(const char *what, const char *name, uint32_t pid);

// vsh methods
void *getNIDfunc(const char *vsh_module, uint32_t fnid, int32_t offset);
static void show_msg(char *msg);
int (*vshtask_notify)(int, const char *) = NULL;
void *(*vsh_malloc)(unsigned int size) = NULL;
int (*vsh_free)(void *ptr) = NULL;

// usb methods
static int32_t get_device_desc(int32_t dev_id, void *p);
static int32_t get_configration_desc(int32_t dev_id, void *p);
static int32_t get_interface_desc(int32_t dev_id, void *p);
static int32_t get_endpoint_desc(int32_t dev_id, void *p);

descriptor_table_t descriptor_table[] = {
  {USB_DESCRIPTOR_TYPE_DEVICE, get_device_desc},
  {USB_DESCRIPTOR_TYPE_CONFIGURATION, get_configration_desc},
  {USB_DESCRIPTOR_TYPE_INTERFACE, get_interface_desc},
  {USB_DESCRIPTOR_TYPE_ENDPOINT, get_endpoint_desc},
};

static CellUsbdLddOps xpad_ops = {
  0,
  xpad_probe,
  xpad_attach,
  xpad_detach
};

static CellUsbdLddOps xpadw_ops = {
  0,
  xpadw_probe,
  xpadw_attach,
  xpadw_detach
};

static CellUsbdLddOps swpro_ops = {
  0,
  swpro_probe,
  swpro_attach,
  swpro_detach
};

static CellUsbdLddOps btd_ops = {
  0,
  btd_probe,
  btd_attach,
  btd_detach
};

static XPAD_INFO_t btd_info[] = {
  {0x0BDA, 0xA729, "Realtek Bluetooth 5.3 Adapter"},
  {0x0A12, 0x0001, "CSR8510 Bluetooth Dongle"},
  {0x0A5C, 0x21E8, "Broadcom BCM20702 Bluetooth Dongle"}
};
#define MAX_BTD_DEV_NUM ((int32_t)(sizeof(btd_info) / sizeof(btd_info[0])))
static uint8_t btd_lddreg[sizeof(btd_info) / sizeof(btd_info[0])];
static uint8_t btd_class_registered = 0;

static XPAD_t XPAD;
static uint8_t xpad_led[4] = {ledOn1, ledOn2, ledOn3, ledOn4};
static sys_ppu_thread_t thread_id = 1;
static sys_mutex_t xpad_mutex;
static int32_t handle[CELL_PAD_MAX_PORT_NUM];
static volatile uint8_t running;
static sys_event_queue_t rumble_queue;
static uint8_t rumble_queue_ready; // set once the queue exists

SYS_MODULE_INFO(XPADD, 0, 1, 0);
SYS_MODULE_START(xpadd_start);
SYS_MODULE_STOP(xpadd_stop);

static inline void _sys_ppu_thread_exit(uint64_t val) {
  system_call_1(41, val);
}

static inline sys_prx_id_t prx_get_module_id_by_address(void *addr) {
  system_call_1(461, (uint64_t)(uint32_t)addr);
  return((int)p1);
}

// lv2 sys_time_get_current_time (syscall 145); used only to timestamp the
// debug log, so relative accuracy between two log lines is all that matters
static inline uint64_t time_now_usec(void) {
  uint64_t sec = 0, nsec = 0;

  system_call_2(145, (uint64_t)(uint32_t)&sec, (uint64_t)(uint32_t)&nsec);
  return(sec * 1000000ULL + nsec / 1000ULL);
}

static inline void sys_pad_dbg_ldd_register_controller(uint8_t *data, int32_t *handle, uint8_t addr, uint32_t capability) {

  // syscall for registering a virtual controller with custom capabilities
  system_call_4(574, (uint8_t *)data, (int32_t *)handle, (uint8_t)addr, (uint32_t)capability);
}

static inline void sys_pad_dbg_ldd_set_data_insert_mode(int32_t handle, uint16_t addr, uint32_t *mode, uint8_t addr2) {

  // syscall for controlling button data filter (allows a virtual controller to be used in games)
  system_call_4(573, handle, addr, mode, addr2);
}

void *getNIDfunc(const char * vsh_module, uint32_t fnid, int32_t offset) {

  // from webman-MOD source
  // used to find malloc, free, and show notification

  // 0x10000 = ELF
  // 0x10080 = segment 2 start
  // 0x10200 = code start

  uint32_t table = (*(uint32_t *)0x1008C) + 0x984; // vsh table address
  //  uint32_t table = (*(uint32_t*)0x1002C) + 0x214 - 0x10000; // vsh table address
  //  uint32_t table = 0x63A9D4;

  while (((uint32_t)*(uint32_t *)table) != 0) {
    uint32_t *export_stru_ptr = (uint32_t *)*(uint32_t *)table; // ptr to export stub, size 2C, "sys_io" usually... Exports:0000000000635BC0 stru_635BC0:    ExportStub_s <0x1C00, 1, 9, 0x39, 0, 0x2000000, aSys_io, ExportFNIDTa
    const char *lib_name_ptr =  (const char *)*(uint32_t *)((char *)export_stru_ptr + 0x10);
    if(strncmp(vsh_module, lib_name_ptr, strlen(lib_name_ptr)) == 0) {
      // we got the proper export struct
      uint32_t lib_fnid_ptr = *(uint32_t *)((char *)export_stru_ptr + 0x14);
      uint32_t lib_func_ptr = *(uint32_t *)((char *)export_stru_ptr + 0x18);
      uint16_t count = *(uint16_t *)((char *)export_stru_ptr + 6); // number of exports
      for (int i = 0; i < count; i++) {
        if (fnid == *(uint32_t *)((char *)lib_fnid_ptr + i*4)) {
          // take address from OPD
          return (void **)*((uint32_t *)(lib_func_ptr) + i) + offset;
        }
      }
    }
    table = table + 4;
  }
  return(0);
}

static void show_msg(char* msg) {

  // from webman-MOD
  // displays a notification on the PS3
  if (!vshtask_notify) {
    vshtask_notify = (void *)((int)getNIDfunc("vshtask", 0xA02D46E7, 0));
  }
  if (strlen(msg) > 200) {
    msg[200] = 0;
  }
  xlog(msg); // every on-screen notification also lands in the debug log
  if (vshtask_notify) {
    vshtask_notify(0, msg);
  }
}

// tiny formatting helpers (no printf in this PRX)
static char *append_str(char *dst, const char *src) {
  while (*src) {
    *dst++ = *src++;
  }
  *dst = 0;
  return(dst);
}

static char *append_hex16(char *dst, uint16_t v) {
  static const char hexdig[] = "0123456789ABCDEF";
  int32_t i;

  for (i = 12; i >= 0; i -= 4) {
    *dst++ = hexdig[(v >> i) & 0xF];
  }
  *dst = 0;
  return(dst);
}
static char *append_hex8(char *dst, uint8_t v) {
  static const char hexdig[] = "0123456789ABCDEF";
  *dst++ = hexdig[(v >> 4) & 0xF];
  *dst++ = hexdig[v & 0xF];
  *dst++ = ' ';
  *dst = 0;
  return(dst);
}

static char *append_int(char *dst, int32_t v) {
  char tmp[12];
  int32_t i = 0;

  if (v < 0) {
    *dst++ = '-';
    v = -v;
  }
  do {
    tmp[i++] = (char)('0' + (v % 10));
    v /= 10;
  } while (v > 0);
  while (i > 0) {
    *dst++ = tmp[--i];
  }
  *dst = 0;
  return(dst);
}

static char *append_hex32(char *dst, uint32_t v) {
  static const char hexdig[] = "0123456789ABCDEF";
  int32_t i;

  for (i = 28; i >= 0; i -= 4) {
    *dst++ = hexdig[(v >> i) & 0xF];
  }
  *dst = 0;
  return(dst);
}

// bounded append so a long message cannot overrun a log line buffer
static char *append_strn(char *dst, const char *src, int32_t max) {
  while (*src && max-- > 0) {
    *dst++ = *src++;
  }
  *dst = 0;
  return(dst);
}

// ---------------------------------------------------------------------------
// debug log. One short line per lifecycle event (registration, attach,
// detach, transfer errors, game-process injection), timestamped in seconds
// since plugin start, to XPAD_LOG_PATH. Each line is written with its own
// open/append/close so a VSH crash loses at most one line and no file
// descriptor is shared between the plugin thread and USBD callbacks.
static sys_mutex_t log_mutex;
static uint8_t log_ok;    // log file created and mutex usable
static uint64_t log_t0;   // timestamp origin

// Rotate the previous boot's log out of the way before starting a fresh one,
// keeping the two most recent old logs (xpad.log.1 = last boot, xpad.log.2 =
// the boot before that). Renames are cheap and best-effort: a missing source
// (first ever boot, or a boot that never created a log) just makes that rename
// a no-op error we ignore. Done before the fresh XPAD_LOG_PATH is truncated so
// the current file still holds last boot's contents at rename time.
static void xlog_rotate(void) {
  cellFsUnlink(XPAD_LOG_PATH_2);            // drop the oldest kept log
  cellFsRename(XPAD_LOG_PATH_1, XPAD_LOG_PATH_2); // last boot -> older slot
  cellFsRename(XPAD_LOG_PATH, XPAD_LOG_PATH_1);   // this file -> last-boot slot
}

static void xlog_init(void) {
  sys_mutex_attribute_t mutex_attr;
  int32_t fd;

  sys_mutex_attribute_initialize(mutex_attr);
  if (sys_mutex_create(&log_mutex, &mutex_attr) != CELL_OK) {
    return;
  }
  // preserve the last two boots' logs so a freeze-then-reboot does not wipe
  // the log that captured the freeze
  xlog_rotate();
  if (cellFsOpen(XPAD_LOG_PATH, CELL_FS_O_WRONLY | CELL_FS_O_CREAT | CELL_FS_O_TRUNC, &fd, NULL, 0) != CELL_FS_SUCCEEDED) {
    sys_mutex_destroy(log_mutex);
    return; // no log (e.g. no space); the plugin works fine without it
  }
  cellFsClose(fd);
  log_t0 = time_now_usec();
  log_ok = 1;
  xlog("xpad debug log start (timestamps: seconds since plugin start)");
}

void xlog(const char *text) {
  char buf[224], *m;
  int32_t fd;
  uint64_t nw, dt;
  uint32_t ms;

  if (!log_ok) {
    return;
  }
  dt = time_now_usec() - log_t0;
  ms = (uint32_t)((dt / 1000ULL) % 1000ULL);
  m = buf;
  *m++ = '[';
  m = append_int(m, (int32_t)(dt / 1000000ULL));
  *m++ = '.';
  *m++ = (char)('0' + (ms / 100) % 10);
  *m++ = (char)('0' + (ms / 10) % 10);
  *m++ = (char)('0' + ms % 10);
  m = append_str(m, "] ");
  m = append_strn(m, text, 200);
  *m++ = '\n';
  // plain lock/unlock (not block()/unblock()): those exit the calling thread
  // on failure, which must never happen on a USBD callback thread
  if (sys_mutex_lock(log_mutex, 0) != CELL_OK) {
    return;
  }
  if (cellFsOpen(XPAD_LOG_PATH, CELL_FS_O_WRONLY | CELL_FS_O_APPEND, &fd, NULL, 0) == CELL_FS_SUCCEEDED) {
    cellFsWrite(fd, buf, (uint64_t)(m - buf), &nw);
    cellFsClose(fd);
  }
  sys_mutex_unlock(log_mutex);
}

// "<what> (code 0x00000000)"
void xlog_code(const char *what, int32_t code) {
  char msg[192], *m;

  if (!log_ok) {
    return;
  }
  m = msg;
  m = append_strn(m, what, 140);
  m = append_str(m, " (code 0x");
  m = append_hex32(m, (uint32_t)code);
  m = append_str(m, ")");
  xlog(msg);
}

// "<what> VID:PID (code 0x00000000)"; code 0 is omitted
void xlog_dev(const char *what, uint16_t vid, uint16_t pid, int32_t code) {
  char msg[192], *m;

  if (!log_ok) {
    return;
  }
  m = msg;
  m = append_strn(m, what, 120);
  m = append_str(m, " ");
  m = append_hex16(m, vid);
  m = append_str(m, ":");
  m = append_hex16(m, pid);
  if (code != 0) {
    m = append_str(m, " (code 0x");
    m = append_hex32(m, (uint32_t)code);
    m = append_str(m, ")");
  }
  xlog(msg);
}

// "<what> <name> (pid N)"; used for game-process (injection) logging where
// the process name is the identifying detail, not a VID/PID
static void xlog_proc(const char *what, const char *name, uint32_t pid) {
  char msg[256], *m;

  if (!log_ok) {
    return;
  }
  m = msg;
  m = append_strn(m, what, 100);
  m = append_str(m, " ");
  m = append_strn(m, name, 120);
  m = append_str(m, " (pid ");
  m = append_int(m, (int32_t)pid);
  m = append_str(m, ")");
  xlog(msg);
}
// ---------------------------------------------------------------------------

// find the display name for a VID/PID across the built-in and user tables
static const char *find_device_name(uint16_t vid, uint16_t pid) {
  int32_t i;

  for (i = 0; i < MAX_XPAD_DEV_NUM; i++) {
    if (xpad_info[i].vid == vid && xpad_info[i].pid == pid) {
      return(xpad_info[i].name);
    }
  }
  for (i = 0; i < MAX_XPADW_DEV_NUM; i++) {
    if (xpadw_info[i].vid == vid && xpadw_info[i].pid == pid) {
      return(xpadw_info[i].name);
    }
  }
  for (i = 0; i < MAX_SWITCH_DEV_NUM; i++) {
    if (switch_info[i].vid == vid && switch_info[i].pid == pid) {
      return(switch_info[i].name);
    }
  }
  for (i = 0; i < extra_count; i++) {
    if (extra_info[i].vid == vid && extra_info[i].pid == pid) {
      return(extra_info[i].name);
    }
  }
  return(NULL);
}

// parse up to 4 hex digits (optional 0x prefix); advances *pp past the number
static int32_t parse_hex16(char **pp, uint16_t *out) {
  char *p = *pp;
  uint32_t v = 0;
  int32_t digits = 0, d;

  while (*p == ' ' || *p == '\t') {
    p++;
  }
  if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
    p += 2;
  }
  for (;;) {
    char c = *p;
    if (c >= '0' && c <= '9') {
      d = c - '0';
    } else if (c >= 'a' && c <= 'f') {
      d = c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
      d = c - 'A' + 10;
    } else {
      break;
    }
    if (++digits > 4) {
      return(-1);
    }
    v = (v << 4) | (uint32_t)d;
    p++;
  }
  if (digits == 0) {
    return(-1);
  }
  *pp = p;
  *out = (uint16_t)v;
  return(0);
}

// trim whitespace and stray quotes from both ends, in place
static char *trim_field(char *s) {
  char *end;

  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '"') {
    s++;
  }
  end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '"')) {
    *--end = 0;
  }
  return(s);
}

static int32_t builtin_known(uint16_t vid, uint16_t pid) {
  int32_t i;

  for (i = 0; i < MAX_XPAD_DEV_NUM; i++) {
    if (xpad_info[i].vid == vid && xpad_info[i].pid == pid) {
      return(1);
    }
  }
  for (i = 0; i < MAX_XPADW_DEV_NUM; i++) {
    if (xpadw_info[i].vid == vid && xpadw_info[i].pid == pid) {
      return(1);
    }
  }
  for (i = 0; i < MAX_SWITCH_DEV_NUM; i++) {
    if (switch_info[i].vid == vid && switch_info[i].pid == pid) {
      return(1);
    }
  }
  return(0);
}

static int32_t extra_known(uint16_t vid, uint16_t pid) {
  int32_t i;

  for (i = 0; i < extra_count; i++) {
    if (extra_info[i].vid == vid && extra_info[i].pid == pid) {
      return(1);
    }
  }
  return(0);
}

static void devlist_warn(int32_t line_no, const char *reason) {
  char msg[96], *m;

  extra_skipped++;
  m = msg;
  m = append_str(m, "XPAD: devices.txt line ");
  m = append_int(m, line_no);
  m = append_str(m, " skipped (");
  m = append_str(m, reason);
  m = append_str(m, ")");
  show_msg(msg);
}

// load user devices from XPAD_DEVICES_PATH; format per line:
//   VID, PID, NAME, XTYPE     (# comments, blank lines and quotes tolerated)
static void load_extra_devices(void) {
  static char fbuf[8192];
  int32_t fd, line_no = 0;
  uint64_t nread = 0;
  char *line, *next, *p, *name, *xt, *last_comma, *scan;
  uint16_t vid, pid;
  uint8_t xtype;

  if (cellFsOpen(XPAD_DEVICES_PATH, CELL_FS_O_RDONLY, &fd, NULL, 0) != CELL_FS_SUCCEEDED) {
    xlog("no xpad_devices.txt, using built-in device table only");
    return; // no device list installed - built-ins only
  }
  cellFsRead(fd, fbuf, sizeof(fbuf) - 1, &nread);
  cellFsClose(fd);
  fbuf[nread] = 0;

  line = fbuf;
  while (line != NULL && *line != 0) {
    line_no++;
    if ((next = strchr(line, '\n')) != NULL) {
      *next++ = 0;
    }

    // '#' starts a comment anywhere on the line
    if ((p = strchr(line, '#')) != NULL) {
      *p = 0;
    }
    p = trim_field(line);
    if (*p == 0) {
      line = next;
      continue;
    }
    if (parse_hex16(&p, &vid) < 0) {
      devlist_warn(line_no, "bad VID");
      line = next;
      continue;
    }
    while (*p == ' ' || *p == '\t') p++;
    if (*p++ != ',') {
      devlist_warn(line_no, "bad format");
      line = next;
      continue;
    }
    if (parse_hex16(&p, &pid) < 0) {
      devlist_warn(line_no, "bad PID");
      line = next;
      continue;
    }
    while (*p == ' ' || *p == '\t') p++;
    if (*p++ != ',') {
      devlist_warn(line_no, "bad format");
      line = next;
      continue;
    }

    // the name runs up to the LAST comma so it may itself contain commas
    last_comma = NULL;
    for (scan = p; *scan; scan++) {
      if (*scan == ',') {
        last_comma = scan;
      }
    }
    if (last_comma == NULL) {
      devlist_warn(line_no, "missing XTYPE");
      line = next;
      continue;
    }
    *last_comma = 0;
    name = trim_field(p);
    xt = trim_field(last_comma + 1);

    if (strcasecmp(xt, "XTYPE_XBOX360") == 0) {
      xtype = XTYPE_XBOX360;
    } else if (strcasecmp(xt, "XTYPE_XBOX360W") == 0) {
      // XTYPE_XBOX360W selects the Microsoft 4-pad wireless receiver
      // protocol; 8BitDo/GameSir 2.4GHz dongles speak plain wired XInput.
      // treat non-Microsoft VIDs as wired to avoid the classic misconfig
      if (vid != 0x045e) {
        show_msg((char *)"XPAD: XBOX360W is only for MS receivers, using XBOX360");
        xtype = XTYPE_XBOX360;
      } else {
        xtype = XTYPE_XBOX360W;
      }
    } else if (strcasecmp(xt, "XTYPE_SWITCH_PRO") == 0 || strcasecmp(xt, "XTYPE_SWITCH") == 0 ||
               strcasecmp(xt, "PTYPE_SWITCH") == 0) {
      // Nintendo Switch Pro protocol (Nintendo HID, not XInput). Use this for
      // 8BitDo/GameSir pads in Switch mode whose VID/PID is not in switch_info.
      xtype = XTYPE_SWITCH_PRO;
    } else if (strcasecmp(xt, "XTYPE_BTD") == 0 || strcasecmp(xt, "XTYPE_BT_DONGLE") == 0 ||
               strcasecmp(xt, "PTYPE_BT") == 0) {
      xtype = XTYPE_BTD;
    } else if (strcasecmp(xt, "XTYPE_XBOX") == 0 || strcasecmp(xt, "XTYPE_XBOXONE") == 0 ||
               strcasecmp(xt, "PTYPE_PS3") == 0 || strcasecmp(xt, "PTYPE_PS4") == 0) {
      devlist_warn(line_no, "type not supported yet");
      line = next;
      continue;
    } else {
      devlist_warn(line_no, "unknown XTYPE");
      line = next;
      continue;
    }

    // registering the same VID/PID twice with different LDD ops is
    // undefined, so keep the first entry (built-ins win over the file).
    // built-in duplicates are skipped silently - listing an already
    // supported pad in the file is harmless
    if (builtin_known(vid, pid)) {
      line = next;
      continue;
    }
    if (extra_known(vid, pid)) {
      devlist_warn(line_no, "duplicate VID/PID");
      line = next;
      continue;
    }
    if (extra_count >= MAX_EXTRA_DEV_NUM) {
      devlist_warn(line_no, "list full");
      line = next;
      continue;
    }
    extra_info[extra_count].vid = vid;
    extra_info[extra_count].pid = pid;
    extra_info[extra_count].xtype = xtype;
    if (*name == 0) {
      name = (char *)"Custom controller";
    }
    strncpy(extra_info[extra_count].name, name, EXTRA_NAME_LEN - 1);
    extra_info[extra_count].name[EXTRA_NAME_LEN - 1] = 0;
    xlog_dev("xpad_devices.txt entry accepted", vid, pid, 0);
    extra_count++;
    line = next;
  }
  xlog_code("xpad_devices.txt entries loaded", extra_count);
}

static void *_malloc(unsigned int size) {

  // vsh export for malloc
  if (!vsh_malloc) {
    vsh_malloc = getNIDfunc("allocator", 0x759E0635, 0);
  }
  if (vsh_malloc) {
    return vsh_malloc(size);
  }
  return(NULL);
}

static void _free(void *ptr) {

  // vsh export for free
  if (!vsh_free) {
    vsh_free = (void *)((int)getNIDfunc("allocator", 0x77A602DD, 0));
  }
  if (vsh_free) {
    vsh_free(ptr);
  }
}

static void block(sys_mutex_t mutex) {
  int32_t r;

  if ((r = sys_mutex_lock(mutex, 0)) != CELL_OK) {
    sys_ppu_thread_exit(0);
  }
}

static void unblock(sys_mutex_t mutex) {
  int32_t r;

  if ((r = sys_mutex_unlock(mutex)) != CELL_OK) {
    sys_ppu_thread_exit(0);
  }
}

// take a 'pending' reference for an async USBD request about to be
// submitted; refused (returns 0) once the unit is being freed
static int32_t unit_submit_begin(XPAD_UNIT_t *unit) {
  int32_t ok = 0;

  block(unit->rb_mutex);
  if (!unit->closing) {
    unit->pending++;
    ok = 1;
  }
  unblock(unit->rb_mutex);
  return(ok);
}

// drop a 'pending' reference: after a completion callback is done with the
// unit, or when submitting the request failed (no callback will fire)
static void unit_submit_end(XPAD_UNIT_t *unit) {
  block(unit->rb_mutex);
  unit->pending--;
  unblock(unit->rb_mutex);
}

static void data_transfer_done(int32_t result, int32_t count, void *arg) {
  XPAD_UNIT_t *unit = (XPAD_UNIT_t *)arg;
  unsigned char *xpadbuf;
  int32_t log_first = 0, log_err = 0;
  uint16_t vid, pid;

  block(unit->rb_mutex);
  if (unit->closing) {
    unit->pending--;
    unblock(unit->rb_mutex);
    return;
  }
  if (result == CELL_OK) {
    if (unit->xtype == XTYPE_SWITCH_PRO && count >= 10 && unit->data[0] == 0x81 && unit->data[1] == 0x01) {
      btd_learn_controller_mac(&unit->data[4]);
    }
    if (!unit->dbg_first_in) {
      unit->dbg_first_in = 1;
      log_first = 1;
      // LOG HEXDUMP (first report)
      char hex[160], *h = hex;
      for (int i=0; i<16; i++) h = append_hex8(h, unit->data[i]);
      xlog_dev("FIRST INPUT REPORT (16 bytes) for", unit->vid, unit->pid, 0);
      xlog(hex);
    }
  } else if (unit->dbg_err_count < 5) {
    // a failing or resetting device shows up here; cap the log lines so a
    // dying dongle cannot flood the log
    unit->dbg_err_count++;
    log_err = 1;
  }
  if (result == CELL_OK && unit->rblen < RINGBUF_SIZE) {
    xpadbuf = &unit->ringbuf[unit->wp][0];
    xpadbuf[0] = (unsigned char)(++unit->tcount & 0xFF);
    count = (count <= MAX_XPAD_DATA_LEN - 2) ? count : MAX_XPAD_DATA_LEN - 2;
    if (count < 0) {
      count = 0;
    }
    xpadbuf[1] = (unsigned char)(count & 0xFF); // store the clamped size, readers copy exactly this many bytes
    memcpy(&xpadbuf[2], unit->data, count);
    if (++unit->wp >= RINGBUF_SIZE) {
      unit->wp = 0;
    }
    unit->rblen++;
  }
  unblock(unit->rb_mutex);
  vid = unit->vid;
  pid = unit->pid;
  data_transfer(unit); // resubmit (takes its own reference)
  unit_submit_end(unit); // this callback's reference
  // log only after dropping the reference (file I/O can be slow and must
  // not extend the window unit_free() waits on); the unit may be freed by
  // now, hence the captured vid/pid locals
  if (log_first) {
    xlog_dev("first input report received from", vid, pid, count);
  }
  if (log_err) {
    xlog_dev("input transfer error on", vid, pid, result);
  }
}

static void data_transfer(XPAD_UNIT_t *unit) {
  int32_t r;
  uint16_t vid, pid;
  if (!unit_submit_begin(unit)) {
    return;
  }
  vid = unit->vid;
  pid = unit->pid;
  if ((r = cellUsbdInterruptTransfer(unit->i_pipe, unit->data, unit->payload, data_transfer_done, unit)) != CELL_OK) {
    unit_submit_end(unit);
    xlog_dev("input transfer submit failed for", vid, pid, r); // after the ref is dropped, see above
  }
}

static void set_interface_done(int32_t result, int32_t count, void *arg) {
  XPAD_UNIT_t *unit = (XPAD_UNIT_t *)arg;
  (void)count;
  xlog_dev("set_interface completed for", unit->vid, unit->pid, result);
  if (!unit->closing) {
    if (unit->xtype == XTYPE_SWITCH_PRO && !unit->swpro_inited) {
        unit->swpro_state = SWPRO_STATE_REQUEST_MAC;
        unit->swpro_timer = 15; // 150ms
    }
    data_transfer(unit);
  }
  unit_submit_end(unit);
}

static void set_config_done(int32_t result, int32_t count, void *arg) {
  XPAD_UNIT_t *unit = (XPAD_UNIT_t *)arg;
  (void)count;
  xlog_dev("set_configuration completed for", unit->vid, unit->pid, result);
  xlog_code("set_config_done ifnum", unit->ifnum);
  xlog_code("set_config_done as", unit->as);
  if (unit->closing) {
    unit_submit_end(unit);
    return;
  }
  if (unit->as > 0 || unit->xtype == XTYPE_SWITCH_PRO) {
    if (unit_submit_begin(unit) &&
        cellUsbdSetInterface(unit->c_pipe, unit->ifnum, unit->as, set_interface_done, unit) != CELL_OK) {
      unit_submit_end(unit);
    }
  } else {
    data_transfer(unit);
  }

  // some 2.4GHz dongles only start streaming input after receiving an
  // LED packet once configured, so re-send the assigned LED here
  if (unit->number >= 0 && handle[unit->number] >= 0 && unit->last_port >= 0) {
    unit->set_led(unit, xpad_led[unit->last_port % 4]);
  }
  unit_submit_end(unit);
}

static void unit_free(XPAD_UNIT_t *unit) {
  int32_t i;

  if (unit) {
    block(unit->rb_mutex);
    unit->closing = 1;
    unblock(unit->rb_mutex);

    // closing the pipes cancels outstanding transfers so their completion
    // callbacks (which see 'closing' and bail out) fire promptly, and
    // releases the pipe resources this driver used to leak on shutdown
    if (unit->i_pipe >= 0) {
      cellUsbdClosePipe(unit->i_pipe);
    }
    if (unit->o_pipe >= 0) {
      cellUsbdClosePipe(unit->o_pipe);
    }
    if (unit->c_pipe >= 0) {
      cellUsbdClosePipe(unit->c_pipe);
    }

    // wait (bounded) for queued completion callbacks to finish with the
    // unit before freeing it; if USBD drops canceled callbacks entirely
    // this exits early via pending == 0. The bound is generous because a
    // callback can be blocked on a debug-log write to the HDD.
    for (i = 0; i < 500 && unit->pending > 0; i++) {
      sys_timer_usleep(1000);
    }

    // barrier: a callback that just dropped the last reference may still
    // be inside rb_mutex; taking it once ensures the handover completed
    block(unit->rb_mutex);
    unblock(unit->rb_mutex);
    sys_mutex_destroy(unit->rb_mutex);
    _free(unit);
  }
}

// return this unit's pad slot to the pool; callers must hold xpad_mutex
static void release_pad_slot(XPAD_UNIT_t *unit) {
  if (unit->number >= 0 && unit->number < MAX_XPAD_NUM) {
    if (XPAD.pad_unit[unit->number] == unit) {
      XPAD.pad_unit[unit->number] = NULL;
      XPAD.n--;
    }
    unit->number = -1;
  }
}

// release the unit-list entry reserved by unit_alloc() and free the unit;
// for attach error paths (callers must NOT hold xpad_mutex)
static void unit_release(XPAD_UNIT_t *unit) {
  if (unit) {
    block(xpad_mutex);
    if (unit->unit_idx >= 0 && unit->unit_idx < MAX_UNIT_NUM && XPAD.units[unit->unit_idx] == unit) {
      XPAD.units[unit->unit_idx] = NULL;
    }
    release_pad_slot(unit);
    unblock(xpad_mutex);
    unit_free(unit);
  }
}

static XPAD_UNIT_t *unit_alloc(int32_t dev_id, int32_t payload, uint8_t ifnum, uint8_t as, uint8_t xtype) {
  XPAD_UNIT_t *unit;
  int32_t i;
  sys_mutex_attribute_t mutex_attr;

  if ((unit = (XPAD_UNIT_t *)_malloc(sizeof(XPAD_UNIT_t) + payload)) != NULL) {
    memset(unit, 0, sizeof(XPAD_UNIT_t));
    unit->dev_id = dev_id;
    unit->c_pipe = -1; // so unit_free() only closes pipes this unit opened
    unit->i_pipe = -1;
    unit->o_pipe = -1;
    unit->payload = payload;
    unit->ifnum = ifnum;
    unit->as = as;
    unit->tcount = 0;
    unit->rp = 0;
    unit->wp = 0;
    unit->rblen = 0;
    unit->xtype = xtype;
    if (xtype == XTYPE_XBOX360) {
      unit->read_input = xpad_read_input;
      unit->set_led = xpad_set_led;
      unit->set_rumble = xpad_set_rumble;
    } else if (xtype == XTYPE_XBOX360W) {
      unit->read_input = xpadw_read_input;
      unit->set_led = xpadw_set_led;
      unit->set_rumble = xpadw_set_rumble;
    } else if (xtype == XTYPE_SWITCH_PRO) {
      unit->read_input = swpro_read_input;
      unit->set_led = swpro_set_led;
      unit->set_rumble = swpro_set_rumble;
    }
    sys_mutex_attribute_initialize(mutex_attr);
    if (sys_mutex_create(&unit->rb_mutex, &mutex_attr) != CELL_OK) {
      _free(unit);
      return(NULL);
    }

    // claim the first free unit-list entry; fail cleanly when full.
    // no pad slot is claimed here - that happens in
    // register_ldd_controller() when the pad is actually usable
    unit->number = -1;
    unit->last_port = -1;
    block(xpad_mutex);
    for (i = 0; i < MAX_UNIT_NUM; i++) {
      if (XPAD.units[i] == NULL) {
        break;
      }
    }
    if (i >= MAX_UNIT_NUM) {
      unblock(xpad_mutex);
      unit_free(unit);
      return(NULL);
    }
    unit->unit_idx = i;
    XPAD.units[i] = unit; // reserve the entry so a concurrent alloc cannot claim it
    unblock(xpad_mutex);
  }
  return(unit);
}

// callers must hold xpad_mutex
static int32_t register_ldd_controller(XPAD_UNIT_t *unit) {
  uint8_t data[0x114];
  int32_t port, s, r;
  uint32_t capability, mode, port_setting;

  // claim a pad slot on first registration; endpoint units (e.g. the four
  // per wireless receiver) exist without a slot until a pad is really there
  if (unit->number < 0) {
    for (s = 0; s < MAX_XPAD_NUM; s++) {
      if (XPAD.pad_unit[s] == NULL) {
        break;
      }
    }
    if (s >= MAX_XPAD_NUM) {
      return(-1); // no free pad slot
    }
    XPAD.pad_unit[s] = unit;
    unit->number = s;
    XPAD.n++;
  }

  // register ldd controller with custom device capability
  if (handle[unit->number] < 0) {
    capability = 0xFFFF; // CELL_PAD_CAPABILITY_PS3_CONFORMITY | CELL_PAD_CAPABILITY_PRESS_MODE | CELL_PAD_CAPABILITY_HP_ANALOG_STICK | CELL_PAD_CAPABILITY_ACTUATOR;
    sys_pad_dbg_ldd_register_controller(data, (int32_t *)&(handle[unit->number]), 5, (uint32_t)capability << 1);
    //handle[unit->number] = cellPadLddRegisterController();
    sys_timer_usleep(1000*10); // allow some time for ps3 to register ldd controller
    if (handle[unit->number] < 0) {
      r = handle[unit->number];
      release_pad_slot(unit);
      return(r);
    }

    // all pad data into games
    mode = CELL_PAD_LDD_INSERT_DATA_INTO_GAME_MODE_ON; // = (1)
    sys_pad_dbg_ldd_set_data_insert_mode((int32_t)handle[unit->number], 0x100, (uint32_t *)&mode, 4);

    // set press and sensor mode on
    port_setting = CELL_PAD_SETTING_PRESS_ON | CELL_PAD_SETTING_SENSOR_ON;
    port = cellPadLddGetPortNo(handle[unit->number]);
    if (port < 0) {
      return(port);
    }
    cellPadSetPortSetting(port, port_setting);

    // set Xbox led corresponding to port number; with more than 4 pads two
    // controllers will show the same LED pattern (there are only 4 LEDs)
    unit->last_port = port;
    unit->set_led(unit, xpad_led[port%4]);
  }
  return(CELL_PAD_OK);
}

// callers must hold xpad_mutex
static int32_t unregister_ldd_controller(XPAD_UNIT_t *unit) {
  int32_t r = CELL_PAD_OK;

  if (unit->number >= 0 && handle[unit->number] >= 0) {
    if ((r = cellPadLddUnregisterController(handle[unit->number])) == CELL_OK) {
      //xpad_set_led(unit->number, xpad_led[ledBlinkingAll]);
      handle[unit->number] = -1;
    }
    // if unregistering failed, keep the handle so the next unit claiming
    // this slot adopts the still-registered LDD pad instead of leaking it
  }

  // always release the slot - the unit may be about to be freed and
  // XPAD.pad_unit[] must never point at a freed unit
  release_pad_slot(unit);
  unit->last_port = -1;
  return(r);
}

// create the named event queue the game-process plugin sends rumble events
// to. Best-effort: if it cannot be created the plugin keeps working, just
// without rumble (input is unaffected).
static int32_t create_rumble_queue(void) {
  sys_event_queue_attribute_t attr;
  int32_t r;

  rumble_queue_ready = 0;
  sys_event_queue_attribute_initialize(attr);
  r = sys_event_queue_create(&rumble_queue, &attr, RUMBLE_IPC_KEY, RUMBLE_QUEUE_DEPTH);
  if (r != CELL_OK) {
    return(r);
  }
  rumble_queue_ready = 1;
  return(CELL_OK);
}

static void destroy_rumble_queue(void) {
  if (rumble_queue_ready) {
    rumble_queue_ready = 0;
    // SYS_EVENT_QUEUE_DESTROY_FORCE tears the queue down even if the game
    // side still has a port connected (e.g. plugin unloaded mid-game)
    sys_event_queue_destroy(rumble_queue, SYS_EVENT_QUEUE_DESTROY_FORCE);
  }
}

// deliver an actuator value to the pad currently assigned to 'port'.
// 'port' is a libpad port number: the game side reads it straight from the
// cellPadSetActDirect(port_no, ...) call it hooks, and unit->last_port is the
// same value read back here via cellPadLddGetPortNo(). Both name the one
// system-wide libpad pad array - an LDD (virtual) pad is inserted into that
// array and addressed by its index, so cellPadLddGetPortNo() returns exactly
// the index cellPadSetActDirect() uses. There is no separate LDD port
// namespace for the two to drift against, so this equality match is correct
// by construction (verified against the libpad LDD semantics in RPCS3).
static void route_rumble(int32_t port, uint8_t lval, uint8_t rval) {
  int32_t i;
  XPAD_UNIT_t *unit;

  block(xpad_mutex);
  for (i = 0; i < MAX_UNIT_NUM; i++) {
    if ((unit = XPAD.units[i]) != NULL && unit->number >= 0 &&
        handle[unit->number] >= 0 && unit->last_port == port && unit->set_rumble) {
      unit->set_rumble(unit, lval, rval);
      break; // ports are unique per pad
    }
  }
  unblock(xpad_mutex);
}

// drain any rumble events the game plugin has posted since the last tick and
// forward them to the USB pads. Non-blocking so the poll loop never stalls.
static void drain_rumble_queue(void) {
  sys_event_t ev[RUMBLE_DRAIN_MAX];
  int32_t n = 0, i;

  if (!rumble_queue_ready) {
    return;
  }
  if (sys_event_queue_tryreceive(rumble_queue, ev, RUMBLE_DRAIN_MAX, &n) != CELL_OK) {
    return;
  }
  for (i = 0; i < n; i++) {
    int32_t port = RUMBLE_EV_PORT(ev[i].data1);
    uint8_t lval = RUMBLE_EV_LVAL(ev[i].data2);
    uint8_t rval = RUMBLE_EV_RVAL(ev[i].data3);
    route_rumble(port, lval, rval);
  }
}

#if XPAD_AUTO_INJECT
// --- PS3MAPI auto-injection of the game-process plugin --------------------
// Loads xpad_game.sprx into running game processes so rumble needs no per-game
// setup. Best-effort: any failure just means that game has no rumble; input is
// unaffected. Compiled out until the PS3MAPI ABI is confirmed (see the
// XPAD_AUTO_INJECT note above and TODO.md section 8).

static uint32_t injected_pids[PS3MAPI_MAX_PROC];

// Settle tracking. A newly-created game process must NOT be injected
// immediately (that mid-boot injection is the launch freeze - see the
// XPAD_AUTO_INJECT note above). pending_pids[] holds candidate processes we
// have seen but not yet injected, and pending_age[] counts how many consecutive
// polls each has stayed alive; a candidate is only injected once its age
// reaches XPAD_INJECT_SETTLE_POLLS, by which point it has settled into a running
// game. Slots are kept in lockstep (pending_age[i] belongs to pending_pids[i]).
// try_auto_inject() runs every ~2 s (see the poll loop), so the settle window
// below is ~2 s * XPAD_INJECT_SETTLE_POLLS.
#define XPAD_INJECT_SETTLE_POLLS 6 // ~12 s: comfortably past a game's own boot
static uint32_t pending_pids[PS3MAPI_MAX_PROC];
static int32_t  pending_age[PS3MAPI_MAX_PROC];

// Budget for auto-inject log lines. A process whose injection keeps failing is
// not marked injected, so try_auto_inject() retries it every ~2s; without a cap
// that would flood the rotated log and push the interesting boot-time events
// (including whatever preceded a freeze) out of the two kept files. The budget
// is generous enough to capture every launch during a normal session but stops
// a stuck process from filling the disk.
#define XPAD_INJECT_LOG_BUDGET 80
static int32_t inject_log_budget = XPAD_INJECT_LOG_BUDGET;

// Injection runs on its own thread (see the INPUT-DEATH FIX note above):
// ps3mapi_load_module() can hang in the kernel, and anything that hangs must
// not share a thread with the input pump. inject_call_t0 is the lv2 timestamp
// of the load_module call currently in flight (0 when idle); the poll thread
// watchdogs it so a hang leaves a breadcrumb in the debug log instead of
// vanishing silently. Written only by the injection thread, read by the poll
// thread; a uint64_t load/store is atomic on the PPU so no lock is needed.
static sys_ppu_thread_t inject_thread_id = (sys_ppu_thread_t)-1;
static volatile uint64_t inject_call_t0;
// how long an in-flight load_module call may run before the watchdog logs it;
// a healthy call returns in well under a second
#define XPAD_INJECT_HANG_USEC 15000000ULL
static uint8_t inject_hang_logged;

// never inject into VSH or the system processes - only into an actual game/app
static int32_t is_system_process(const char *name) {
  static const char *sys[] = {"vsh", "sys_", "default.self"};
  int32_t i;

  for (i = 0; i < (int32_t)(sizeof(sys) / sizeof(sys[0])); i++) {
    if (strstr(name, sys[i]) != NULL) {
      return 1;
    }
  }
  return 0;
}

static int32_t ps3mapi_get_all_pids(uint32_t *pids) {
  memset(pids, 0, sizeof(uint32_t) * PS3MAPI_MAX_PROC);
  system_call_3(8, SYSCALL8_OPCODE_PS3MAPI, PS3MAPI_OPCODE_GET_ALL_PROC_PID, (uint64_t)(uint32_t)pids);
  return (int32_t)p1;
}

static void ps3mapi_get_proc_name(uint32_t pid, char *name) {
  name[0] = 0;
  system_call_4(8, SYSCALL8_OPCODE_PS3MAPI, PS3MAPI_OPCODE_GET_PROC_NAME_BY_PID, (uint64_t)pid, (uint64_t)(uint32_t)name);
}

static int32_t ps3mapi_load_module(uint32_t pid, const char *path) {
  system_call_6(8, SYSCALL8_OPCODE_PS3MAPI, PS3MAPI_OPCODE_LOAD_PROC_MODULE, (uint64_t)pid, (uint64_t)(uint32_t)path, 0, 0);
  return (int32_t)p1;
}

static int32_t pid_in_list(uint32_t pid, const uint32_t *pids) {
  int32_t i;

  for (i = 0; i < PS3MAPI_MAX_PROC; i++) {
    if (pids[i] == pid) {
      return 1;
    }
  }
  return 0;
}

static int32_t already_injected(uint32_t pid) {
  return pid_in_list(pid, injected_pids);
}

static void mark_injected(uint32_t pid) {
  int32_t i;

  for (i = 0; i < PS3MAPI_MAX_PROC; i++) {
    if (injected_pids[i] == 0) {
      injected_pids[i] = pid;
      return;
    }
  }
}

// Record that pid was seen alive this poll and return its consecutive-seen age
// (capped at the settle threshold), adding it to the pending table on first
// sight. Returns 0 if the table is full - an untracked candidate is treated as
// not-yet-settled and therefore not injected, which fails safe.
static int32_t pending_age_bump(uint32_t pid) {
  int32_t i, free = -1;

  for (i = 0; i < PS3MAPI_MAX_PROC; i++) {
    if (pending_pids[i] == pid) {
      if (pending_age[i] < XPAD_INJECT_SETTLE_POLLS) {
        pending_age[i]++;
      }
      return pending_age[i];
    }
    if (free < 0 && pending_pids[i] == 0) {
      free = i;
    }
  }
  if (free >= 0) {
    pending_pids[free] = pid;
    pending_age[free] = 1;
    return 1;
  }
  return 0;
}

static void pending_forget(uint32_t pid) {
  int32_t i;

  for (i = 0; i < PS3MAPI_MAX_PROC; i++) {
    if (pending_pids[i] == pid) {
      pending_pids[i] = 0;
      pending_age[i] = 0;
      return;
    }
  }
}

static void try_auto_inject(void) {
  uint32_t pids[PS3MAPI_MAX_PROC];
  char name[256]; // generous: PS3MAPI writes the process self-path here
  int32_t i, r, age;

  if ((r = ps3mapi_get_all_pids(pids)) < 0) {
    if (inject_log_budget > 0) {
      inject_log_budget--;
      xlog_code("auto-inject: get_all_pids failed", r);
    }
    return;
  }

  // forget pids that have exited so their slots (and a possibly-reused pid)
  // can be injected again next time
  for (i = 0; i < PS3MAPI_MAX_PROC; i++) {
    if (injected_pids[i] != 0 && !pid_in_list(injected_pids[i], pids)) {
      injected_pids[i] = 0;
    }
    // a candidate that exited before it settled (e.g. multiMAN's prepNTFS /
    // RELOAD loader helpers) drops out of the pending table, and a reused pid
    // restarts its settle window from scratch
    if (pending_pids[i] != 0 && !pid_in_list(pending_pids[i], pids)) {
      pending_pids[i] = 0;
      pending_age[i] = 0;
    }
  }

  for (i = 0; i < PS3MAPI_MAX_PROC; i++) {
    if (pids[i] == 0 || already_injected(pids[i])) {
      continue;
    }
    ps3mapi_get_proc_name(pids[i], name);
    name[sizeof(name) - 1] = 0;
    if (name[0] == 0 || is_system_process(name)) {
      continue;
    }
    // Do NOT inject a process the first time we see it. Injecting into a game
    // that is still booting hangs the kernel prx loader (the launch freeze -
    // see the XPAD_AUTO_INJECT note above). Age the candidate instead and only
    // inject once it has stayed alive long enough to have finished booting.
    age = pending_age_bump(pids[i]);
    if (age < XPAD_INJECT_SETTLE_POLLS) {
      continue;
    }
    // Log the process and the attempt BEFORE the syscall. Because each xlog()
    // line is flushed with its own open/write/close and the log survives a
    // reboot, this line is the breadcrumb that names which game we were
    // injecting into if the console ever does lock up here.
    if (inject_log_budget > 0) {
      inject_log_budget--;
      xlog_proc("auto-inject: loading xpad_game.sprx into", name, pids[i]);
    }
    inject_call_t0 = time_now_usec();
    r = ps3mapi_load_module(pids[i], XPAD_GAME_SPRX_PATH);
    inject_call_t0 = 0;
    if (r >= 0) {
      mark_injected(pids[i]);
      pending_forget(pids[i]); // settled and injected: stop tracking it
      if (inject_log_budget > 0) {
        inject_log_budget--;
        xlog_proc("auto-inject: loaded xpad_game.sprx OK into", name, pids[i]);
      }
    } else if (inject_log_budget > 0) {
      // not marked injected: left in the pending table with its age pinned at
      // the threshold, so it retries next pass. The budget keeps a persistent
      // failure from flooding the log
      inject_log_budget--;
      xlog_code("auto-inject: load_module failed", r);
    }
  }
}

// Auto-injection is enabled only when the user has created the opt-in file
// (see the LOADER-WEDGE FIX note above). Existence is the whole switch; the
// content is ignored.
static int32_t auto_inject_opted_in(void) {
  int32_t fd;

  if (cellFsOpen(XPAD_INJECT_OPT_IN_PATH, CELL_FS_O_RDONLY, &fd, NULL, 0) != CELL_FS_SUCCEEDED) {
    return 0;
  }
  cellFsClose(fd);
  return 1;
}

// Dedicated injection thread: scans for game processes every ~2 s, same
// cadence the poll loop used when it did this inline. All the injection state
// (injected_pids/pending_*/inject_log_budget) is touched only from this thread.
// If ps3mapi_load_module() hangs, this thread blocks forever and injection is
// simply lost for the session — input, which lives on the poll thread, is
// untouched. On a normal shutdown the thread notices 'running' going to 0
// within one sleep period and exits; xpadd_stop_thread joins it.
static void inject_thread(uint64_t arg) {
  (void)arg;
  while (running) {
    sys_timer_sleep(2);
    if (!running) {
      break;
    }
    try_auto_inject();
  }
  sys_ppu_thread_exit(0);
}
#endif // XPAD_AUTO_INJECT

void usb_done_cb(int32_t result, int32_t count, void *arg) {
  XPAD_UNIT_t *unit = (XPAD_UNIT_t *)arg;
  (void)count;

  // LED/rumble out-transfer finished; unit->out_data is free again
  block(unit->rb_mutex);
  if (result != CELL_OK) {
    xlog_dev("OUT transfer failed for", unit->vid, unit->pid, result);
  }
  unit->out_busy = 0;
  unit->pending--;
  unblock(unit->rb_mutex);
}

// send an LED/rumble command; the payload is copied into unit->out_data
// because the transfer is asynchronous and would otherwise read the
// caller's stack frame after it returned
static int32_t write_xpad(XPAD_UNIT_t *unit, uint8_t *data, int32_t len) {
  int32_t r;

  if (unit->is_bt) {
    return btd_send_hid_output(unit, data, len);
  }

  if (len > XPAD_OUT_LEN) {
    xlog_code("write_xpad failed: len > XPAD_OUT_LEN", len);
    return(-1);
  }
  block(unit->rb_mutex);
  if (unit->closing || unit->out_busy) {
    // a previous command is still in flight and owns out_data; commands
    // are rare (LED changes) and callers tolerate the occasional drop
    unblock(unit->rb_mutex);
    return(-1);
  }
  unit->out_busy = 1;
  unit->pending++;
  memcpy(unit->out_data, data, len);
  unblock(unit->rb_mutex);
  if ((r = cellUsbdInterruptTransfer(unit->o_pipe, unit->out_data, len, usb_done_cb, unit)) != CELL_OK) {
    block(unit->rb_mutex);
    unit->out_busy = 0;
    unit->pending--;
    unblock(unit->rb_mutex);
    if (unit->dbg_err_count < 5) {
      unit->dbg_err_count++;
      xlog_dev("LED/rumble transfer submit failed for", unit->vid, unit->pid, r);
    }
  }
  return(r);
}


#define SWPRO_USB_CMD_STATUS      0x01
#define SWPRO_USB_CMD_HANDSHAKE   0x02
#define SWPRO_USB_CMD_BAUDRATE_3M 0x03
#define SWPRO_USB_CMD_NO_TIMEOUT  0x04

static int32_t swpro_send_usb(XPAD_UNIT_t *unit, uint8_t cmd) {
  uint8_t out[2] = {0x80, cmd};
  int32_t r = write_xpad(unit, out, 2);
  xlog_code("Switch USB 0x80 cmd sent, result", (cmd << 16) | (r & 0xFFFF));
  return(r);
}

static void process_swpro_init(XPAD_UNIT_t *unit) {
  static const uint8_t led_map[4] = {0x01, 0x02, 0x04, 0x08};
  uint8_t pat = 0x01;

  if (unit->swpro_state == SWPRO_STATE_IDLE || unit->swpro_state == SWPRO_STATE_DONE) {
    return;
  }
  if (unit->swpro_timer > 0) {
    unit->swpro_timer--;
    return;
  }
  if (unit->out_busy) {
    return;
  }

  switch (unit->swpro_state) {
    case SWPRO_STATE_REQUEST_MAC:
      if (swpro_send_usb(unit, SWPRO_USB_CMD_STATUS) == CELL_OK) {
        unit->swpro_state = SWPRO_STATE_HANDSHAKE_1;
        unit->swpro_timer = 15;
      }
      break;
    case SWPRO_STATE_HANDSHAKE_1:
      if (swpro_send_usb(unit, SWPRO_USB_CMD_HANDSHAKE) == CELL_OK) {
        unit->swpro_state = SWPRO_STATE_BAUDRATE;
        unit->swpro_timer = 15;
      }
      break;
    case SWPRO_STATE_BAUDRATE:
      if (swpro_send_usb(unit, SWPRO_USB_CMD_BAUDRATE_3M) == CELL_OK) {
        unit->swpro_state = SWPRO_STATE_HANDSHAKE_2;
        unit->swpro_timer = 15;
      }
      break;
    case SWPRO_STATE_HANDSHAKE_2:
      if (swpro_send_usb(unit, SWPRO_USB_CMD_HANDSHAKE) == CELL_OK) {
        unit->swpro_state = SWPRO_STATE_FORCE_HID;
        unit->swpro_timer = 15;
      }
      break;
    case SWPRO_STATE_FORCE_HID:
      if (swpro_send_usb(unit, SWPRO_USB_CMD_NO_TIMEOUT) == CELL_OK) {
        unit->swpro_state = SWPRO_STATE_SET_MODE;
        unit->swpro_timer = 15;
      }
      break;
    case SWPRO_STATE_SET_MODE:
      if (swpro_send_subcommand(unit, SWPRO_SC_SET_MODE, 0x30) == CELL_OK) {
        xlog("Switch init: SET_MODE (0x30) sent");
        unit->swpro_state = SWPRO_STATE_ENABLE_IMU;
        unit->swpro_timer = 15;
      }
      break;
    case SWPRO_STATE_ENABLE_IMU:
      if (swpro_send_subcommand(unit, SWPRO_SC_ENABLE_IMU, 0x01) == CELL_OK) {
        xlog("Switch init: ENABLE_IMU sent");
        unit->swpro_state = SWPRO_STATE_ENABLE_VIB;
        unit->swpro_timer = 15;
      }
      break;
    case SWPRO_STATE_ENABLE_VIB:
      if (swpro_send_subcommand(unit, SWPRO_SC_ENABLE_VIB, 0x01) == CELL_OK) {
        xlog("Switch init: ENABLE_VIB sent");
        unit->swpro_state = SWPRO_STATE_SET_LED;
        unit->swpro_timer = 15;
      }
      break;
    case SWPRO_STATE_SET_LED:
      if (unit->last_port >= 0) {
        pat = led_map[unit->last_port % 4];
      }
      if (swpro_send_subcommand(unit, SWPRO_SC_SET_PLAYER, pat) == CELL_OK) {
        xlog_code("Switch init: DONE, player LED set to", pat);
        unit->swpro_state = SWPRO_STATE_DONE;
        unit->swpro_inited = 1;
      }
      break;
    default:
      unit->swpro_state = SWPRO_STATE_IDLE;
      break;
  }
}

static void notify_pad_connected(XPAD_UNIT_t *unit) {
  char msg[96], *m;

  m = msg;
  m = append_str(m, "XPAD: ");
  m = append_str(m, unit->name ? unit->name : "Controller");
  m = append_str(m, " (");
  m = append_hex16(m, unit->vid);
  m = append_str(m, ":");
  m = append_hex16(m, unit->pid);
  m = append_str(m, ")");
  if (unit->last_port >= 0) {
    m = append_str(m, " on port ");
    m = append_int(m, unit->last_port + 1);
  }
  show_msg(msg);
}

static void notify_pad_disconnected(XPAD_UNIT_t *unit) {
  char msg[96], *m;

  m = msg;
  m = append_str(m, "XPAD: ");
  m = append_str(m, unit->name ? unit->name : "Controller");
  m = append_str(m, " disconnected");
  show_msg(msg);
}

// start of wired controller specific methods
static int32_t xpad_probe(int32_t dev_id) {
  uint16_t idVendor, idProduct;
  uint32_t i;
  UsbDeviceDescriptor *ddesc;
  UsbInterfaceDescriptor *idesc;

  block(xpad_mutex);
  if (XPAD.n >= MAX_XPAD_NUM) { // no free pad slot
    unblock(xpad_mutex);
    xlog("probe rejected: no free pad slot");
    return(CELL_USBD_PROBE_FAILED);
  }
  unblock(xpad_mutex);
  if ((ddesc = (UsbDeviceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_DEVICE)) == NULL) {
    xlog("probe rejected: no device descriptor");
    return(CELL_USBD_PROBE_FAILED);
  }
  idesc = (UsbInterfaceDescriptor *)ddesc;
  if ((idesc = (UsbInterfaceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, idesc, USB_DESCRIPTOR_TYPE_INTERFACE)) == NULL) {
    xlog("probe rejected: no interface descriptor");
    return(CELL_USBD_PROBE_FAILED);
  }

  // make sure product id and vendor id are valid
  idVendor = SWAP16(ddesc->idVendor);
  idProduct = SWAP16(ddesc->idProduct);
  for (i = 0; i < MAX_XPAD_DEV_NUM; i++) {
    if (xpad_info[i].vid == idVendor && xpad_info[i].pid == idProduct) {
      xlog_dev("probe matched (wired)", idVendor, idProduct, 0);
      return(CELL_USBD_PROBE_SUCCEEDED);
    }
  }
  for (i = 0; i < (uint32_t)extra_count; i++) {
    if (extra_info[i].xtype == XTYPE_XBOX360 && extra_info[i].vid == idVendor && extra_info[i].pid == idProduct) {
      xlog_dev("probe matched (wired, devices.txt)", idVendor, idProduct, 0);
      return(CELL_USBD_PROBE_SUCCEEDED);
    }
  }
  xlog_dev("probe rejected: VID/PID not in wired table", idVendor, idProduct, 0);
  return(CELL_USBD_PROBE_FAILED);
}

static int32_t xpad_attach(int32_t dev_id) {
  int32_t payload;
  uint8_t *desc = NULL;
  UsbConfigurationDescriptor *cdesc;
  UsbInterfaceDescriptor *ifd, *cur_if = NULL, *xin_if = NULL, *first_if = NULL;
  UsbEndpointDescriptor *ed, *ep_in = NULL, *ep_out = NULL, *first_in = NULL, *first_out = NULL;
  XPAD_UNIT_t *unit;

  UsbDeviceDescriptor *ddesc;
  uint16_t idVendor = 0, idProduct = 0;

  if ((ddesc = (UsbDeviceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_DEVICE)) != NULL) {
    idVendor = SWAP16(ddesc->idVendor);
    idProduct = SWAP16(ddesc->idProduct);
  }
  xlog_dev("attach starting for", idVendor, idProduct, 0);
  if ((cdesc = (UsbConfigurationDescriptor *) cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_CONFIGURATION)) == NULL) {
    xlog("attach failed: no configuration descriptor");
    return (CELL_USBD_ATTACH_FAILED);
  }

  // some dongles put audio/HID interfaces first or use in endpoints other
  // than 0x81, so scan every interface for the XInput class triple
  // (class 0xFF, subclass 0x5D, protocol 0x01) and take that interface's
  // interrupt IN/OUT endpoints, whatever their addresses
  while ((desc = (uint8_t *)cellUsbdScanStaticDescriptor(dev_id, desc, 0)) != NULL) {
    if (desc[1] == USB_DESCRIPTOR_TYPE_INTERFACE) {
      ifd = (UsbInterfaceDescriptor *)desc;
      if (first_if == NULL) {
        first_if = ifd;
      }
      if (xin_if == NULL &&
          ifd->bInterfaceClass == 0xFF && USB_IF_SUBCLASS(ifd) == 0x5D && ifd->bInterfaceProtocol == 0x01) {
        xin_if = ifd;
      }
      cur_if = ifd;
    } else if (desc[1] == USB_DESCRIPTOR_TYPE_ENDPOINT) {
      ed = (UsbEndpointDescriptor *)desc;
      if ((ed->bmAttributes & 0x03) != 0x03) { // only interrupt endpoints
        continue;
      }
      if (cur_if != NULL && cur_if == xin_if) {
        if ((ed->bEndpointAddress & 0x80) && ep_in == NULL) {
          ep_in = ed;
        } else if (!(ed->bEndpointAddress & 0x80) && ep_out == NULL) {
          ep_out = ed;
        }
      }
      if (cur_if != NULL && cur_if == first_if) {
        if ((ed->bEndpointAddress & 0x80) && first_in == NULL) {
          first_in = ed;
        } else if (!(ed->bEndpointAddress & 0x80) && first_out == NULL) {
          first_out = ed;
        }
      }
    }
  }
  if (xin_if == NULL || ep_in == NULL) {
    // no XInput interface advertised; fall back to the first interface
    // (the pre-scan behavior, minus the hard 0x81 requirement)
    xlog("attach: no XInput class interface found, falling back to first interface");
    xin_if = first_if;
    ep_in = first_in;
    ep_out = first_out;
  }
  if (xin_if == NULL || ep_in == NULL) {
    xlog("attach failed: no usable interface/interrupt-IN endpoint");
    return(CELL_USBD_ATTACH_FAILED);
  }
  payload = SWAP16(ep_in->wMaxPacketSize);
  {
    char dbg[128], *m = dbg;
    m = append_str(m, "attach: interface ");
    m = append_int(m, xin_if->bInterfaceNumber);
    m = append_str(m, " alt ");
    m = append_int(m, xin_if->bAlternateSetting);
    m = append_str(m, ", ep_in 0x");
    m = append_hex16(m, ep_in->bEndpointAddress);
    m = append_str(m, ", ep_out ");
    if (ep_out != NULL) {
      m = append_str(m, "0x");
      m = append_hex16(m, ep_out->bEndpointAddress);
    } else {
      m = append_str(m, "none");
    }
    m = append_str(m, ", payload ");
    m = append_int(m, payload);
    xlog(dbg);
  }
  if ((unit = unit_alloc(dev_id, payload, xin_if->bInterfaceNumber, xin_if->bAlternateSetting, XTYPE_XBOX360)) == NULL) {
    xlog("attach failed: unit alloc (out of memory or unit list full)");
    return(CELL_USBD_ATTACH_FAILED);
  }
  unit->vid = idVendor;
  unit->pid = idProduct;
  unit->name = find_device_name(idVendor, idProduct);
  if ((unit->c_pipe = cellUsbdOpenPipe(dev_id, NULL)) < 0) {
    xlog_code("attach failed: control pipe open", unit->c_pipe);
    unit_release(unit);
    return(CELL_USBD_ATTACH_FAILED);
  }
  if ((unit->i_pipe = cellUsbdOpenPipe(dev_id, ep_in)) < 0) {
    xlog_code("attach failed: IN pipe open", unit->i_pipe);
    unit_release(unit);
    return(CELL_USBD_ATTACH_FAILED);
  }
  if (ep_out != NULL) {
    if ((unit->o_pipe = cellUsbdOpenPipe(dev_id, ep_out)) < 0) {
      xlog_code("attach failed: OUT pipe open", unit->o_pipe);
      unit_release(unit);
      return(CELL_USBD_ATTACH_FAILED);
    }
  } else {
    // no OUT endpoint descriptor found; try the classic addresses
    ep_in->bEndpointAddress = 0x01; // XBox 360 controller out endpoint
    if ((unit->o_pipe = cellUsbdOpenPipe(dev_id, ep_in)) < 0) {
      ep_in->bEndpointAddress = 0x02; // It is 0x02 for some controllers
      if ((unit->o_pipe = cellUsbdOpenPipe(dev_id, ep_in)) < 0) {
        xlog_code("attach failed: fallback OUT pipe open", unit->o_pipe);
        unit_release(unit);
        return(CELL_USBD_ATTACH_FAILED);
      }
    }
  }

  // endpoint found; a wired pad is only useful with an LDD pad behind it,
  // so claim the pad slot and register before starting configuration
  block(xpad_mutex);
  if (register_ldd_controller(unit) != CELL_PAD_OK) {
    unblock(xpad_mutex);
    unit_release(unit);
    show_msg((char *)"XPAD: no free pad slot for new controller");
    return(CELL_USBD_ATTACH_FAILED);
  }
  unblock(xpad_mutex);
  cellUsbdSetPrivateData(dev_id, unit);
  if (unit_submit_begin(unit)) {
    int32_t cr = cellUsbdSetConfiguration(unit->c_pipe, cdesc->bConfigurationValue, set_config_done, unit);
    if (cr != CELL_OK) {
      xlog_code("attach: set_configuration submit failed", cr);
      unit_submit_end(unit);
    }
  }
  {
    char dbg[64], *m = dbg;
    m = append_str(m, "attach OK, LDD pad port ");
    m = append_int(m, unit->last_port);
    xlog(dbg);
  }
  notify_pad_connected(unit);
  return(CELL_USBD_ATTACH_SUCCEEDED);
}

static int32_t xpad_detach(int32_t dev_id) {
  XPAD_UNIT_t *unit;

  // Xbox controller has been unplugged
  // disconnect virtual controller associated to it
  if ((unit = (XPAD_UNIT_t *)cellUsbdGetPrivateData(dev_id)) == NULL) {
    xlog("USB detach for unknown device (no private data)");
    return(CELL_USBD_DETACH_FAILED);
  }
  xlog_dev("USB detach:", unit->vid, unit->pid, 0);
  block(xpad_mutex);
  if (unit->unit_idx >= 0 && unit->unit_idx < MAX_UNIT_NUM && XPAD.units[unit->unit_idx] == unit) {
    XPAD.units[unit->unit_idx] = NULL;
  }
  unregister_ldd_controller(unit);
  unblock(xpad_mutex);
  notify_pad_disconnected(unit);
  unit_free(unit);
  return(CELL_USBD_DETACH_SUCCEEDED);
}

static int32_t xpad_detach_all(void) {
  int32_t i;
  XPAD_UNIT_t *unit;

  // detach all wired controllers
  block(xpad_mutex);
  for (i = 0; i < MAX_UNIT_NUM; i++) {
    if ((unit = XPAD.units[i]) != NULL && unit->xtype == XTYPE_XBOX360) {
      XPAD.units[i] = NULL;
      unregister_ldd_controller(unit);
      unit_free(unit);
    }
  }
  unblock(xpad_mutex);
  return(CELL_USBD_DETACH_SUCCEEDED);
}

static void xpad_read_report(int32_t id, uint8_t *readBuf) {
  uint16_t *digit0, *digit1, *digit2,
           *analog_rx, *analog_ry, *analog_lx, *analog_ly,
           *press_l2, *press_r2, *press_l1, *press_r1,
           *press_up, *press_down, *press_left, *press_right,
           *press_cross, *press_square, *press_circle, *press_triangle;
  XBOX360_IN_REPORT *report;
  CellPadData data;

  report = (XBOX360_IN_REPORT *)readBuf;
  memset(&data, 0, sizeof(CellPadData));
  data.len = 24;

  // map location of each button in virtual pad's data
  digit0 = &data.button[0];
  digit1 = &data.button[CELL_PAD_BTN_OFFSET_DIGITAL1];
  digit2 = &data.button[CELL_PAD_BTN_OFFSET_DIGITAL2];
  analog_rx = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X];
  analog_ry = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y];
  analog_lx = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X];
  analog_ly = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y];
  press_l2 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_L2];
  press_r2 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_R2];
  press_l1 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_L1];
  press_r1 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_R1];
  press_right = &data.button[CELL_PAD_BTN_OFFSET_PRESS_RIGHT];
  press_left = &data.button[CELL_PAD_BTN_OFFSET_PRESS_LEFT];
  press_up = &data.button[CELL_PAD_BTN_OFFSET_PRESS_UP];
  press_down = &data.button[CELL_PAD_BTN_OFFSET_PRESS_DOWN];
  press_cross = &data.button[CELL_PAD_BTN_OFFSET_PRESS_CROSS];
  press_square = &data.button[CELL_PAD_BTN_OFFSET_PRESS_SQUARE];
  press_circle = &data.button[CELL_PAD_BTN_OFFSET_PRESS_CIRCLE];
  press_triangle = &data.button[CELL_PAD_BTN_OFFSET_PRESS_TRIANGLE];

  // set default controller values
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_X] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_Y] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_Z] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_G] = 0x0200;

  // read values from Xbox controller and map it to the virtual pad
  *digit0 = (report->buttons & btnXbox) ? *digit0 | CELL_PAD_CTRL_LDD_PS : *digit0 & ~CELL_PAD_CTRL_LDD_PS;
  *digit1 = (report->buttons & btnDigiLeft) ? *digit1 | CELL_PAD_CTRL_LEFT : *digit1 & ~CELL_PAD_CTRL_LEFT;
  *digit1 = (report->buttons & btnDigiDown) ? *digit1 | CELL_PAD_CTRL_DOWN : *digit1 & ~CELL_PAD_CTRL_DOWN;
  *digit1 = (report->buttons & btnDigiRight) ? *digit1 | CELL_PAD_CTRL_RIGHT : *digit1 & ~CELL_PAD_CTRL_RIGHT;
  *digit1 = (report->buttons & btnDigiUp) ? *digit1 | CELL_PAD_CTRL_UP : *digit1 & ~CELL_PAD_CTRL_UP;
  *digit1 = (report->buttons & btnStart) ? *digit1 | CELL_PAD_CTRL_START : *digit1 & ~CELL_PAD_CTRL_START;
  *digit1 = (report->buttons & btnHatRight) ? *digit1 | CELL_PAD_CTRL_R3 : *digit1 & ~CELL_PAD_CTRL_R3;
  *digit1 = (report->buttons & btnHatLeft) ? *digit1 | CELL_PAD_CTRL_L3 : *digit1 & ~CELL_PAD_CTRL_L3;
  *digit1 = (report->buttons & btnBack) ? *digit1 | CELL_PAD_CTRL_SELECT : *digit1 & ~CELL_PAD_CTRL_SELECT;
  *digit2 = (report->buttons & btnX) ? *digit2 | CELL_PAD_CTRL_SQUARE : *digit2 & ~CELL_PAD_CTRL_SQUARE;
  *digit2 = (report->buttons & btnA) ? *digit2 | CELL_PAD_CTRL_CROSS : *digit2 & ~CELL_PAD_CTRL_CROSS;
  *digit2 = (report->buttons & btnB) ? *digit2 | CELL_PAD_CTRL_CIRCLE : *digit2 & ~CELL_PAD_CTRL_CIRCLE;
  *digit2 = (report->buttons & btnY) ? *digit2 | CELL_PAD_CTRL_TRIANGLE : *digit2 & ~CELL_PAD_CTRL_TRIANGLE;
  *digit2 = (report->buttons & btnShoulderRight) ? *digit2 | CELL_PAD_CTRL_R1 : *digit2 & ~CELL_PAD_CTRL_R1;
  *digit2 = (report->buttons & btnShoulderLeft) ? *digit2 | CELL_PAD_CTRL_L1 : *digit2 & ~CELL_PAD_CTRL_L1;
  *digit2 = (report->trigL > 0) ? *digit2 | CELL_PAD_CTRL_L2 : *digit2 & ~CELL_PAD_CTRL_L2;
  *digit2 = (report->trigR > 0) ? *digit2 | CELL_PAD_CTRL_R2 : *digit2 & ~CELL_PAD_CTRL_R2;

  // emulate pressure values except for L2 and R2, button presses correspond to max sensitivity value
  *press_l2 = (report->trigL);
  *press_r2 = (report->trigR);
  *press_l1 = (report->buttons & btnShoulderLeft) ? 0xFF : 0;
  *press_r1 = (report->buttons & btnShoulderRight) ? 0xFF : 0;
  *press_up = (report->buttons & btnDigiUp) ? 0xFF : 0;
  *press_down = (report->buttons & btnDigiDown) ? 0xFF : 0;
  *press_left = (report->buttons & btnDigiLeft) ? 0xFF : 0;
  *press_right = (report->buttons & btnDigiRight) ? 0xFF : 0;
  *press_square = (report->buttons & btnX) ? 0xFF : 0;
  *press_circle = (report->buttons & btnB) ? 0xFF : 0;
  *press_cross = (report->buttons & btnA) ? 0xFF : 0;
  *press_triangle = (report->buttons & btnY) ? 0xFF : 0;

  // PS3 pads use 8 bit values for each axis while Xbox pads use 16 bit
  // convert Xbox analog values for PS3 compatibility
  *analog_rx = (report->right.x-0x80) & 0x00FF;
  *analog_ry = ((report->right.y ^ 0xFF)-0x80) & 0x00FF;
  *analog_lx = (report->left.x-0x80) & 0x00FF;
  *analog_ly = ((report->left.y ^ 0xFF)-0x80) & 0x00FF;

  // send pad data to virtual pad
  cellPadLddDataInsert(handle[id], &data);
}

static int32_t xpad_read_input(XPAD_UNIT_t *unit, void *data) {
  unsigned char *p;
  unsigned char *xpadbuf;
  int32_t have_report = 0;
  XBOX360_IN_REPORT *report;

  p = (unsigned char *)data;
  if (unit == NULL) {
    return(-1);
  }

  // drain the whole ring buffer each tick so a pad reporting faster than
  // the 10 ms poll rate does not accumulate latency; only the newest
  // movement report is parsed
  block(unit->rb_mutex);
  while (unit->rblen > 0) {
    xpadbuf = &unit->ringbuf[unit->rp][0];
    report = (XBOX360_IN_REPORT *)&xpadbuf[2];
    if ((xpadbuf[1] >= sizeof(XBOX360_IN_REPORT)) &&
        (report->header.command == inReport) && (report->header.size == sizeof(XBOX360_IN_REPORT))) {
      memcpy(p, &xpadbuf[2], xpadbuf[1]);
      have_report = 1;
    }
    if (++unit->rp >= RINGBUF_SIZE) {
      unit->rp = 0;
    }
    unit->rblen--;
  }
  unblock(unit->rb_mutex);

  // parse outside the ring-buffer lock so USB completion callbacks
  // are never blocked on cellPadLddDataInsert()
  if (have_report && unit->number >= 0 && handle[unit->number] >= 0) {
    xpad_read_report(unit->number, p);
  }
  return(0);
}

static int32_t xpad_set_led(XPAD_UNIT_t *unit, uint8_t led) {
  uint8_t out[3] = {0x01, 0x03, led};

  if (write_xpad(unit, out, 3) < 0) {
    return(-1);
  }
  return(CELL_OK);
}

static int32_t xpad_set_rumble(XPAD_UNIT_t *unit, uint8_t lval, uint8_t rval) {
  uint8_t out[8] = {0x00, 0x08, 0x00, lval, rval, 0x00, 0x00, 0x00};

  if (write_xpad(unit, out, 8) < 0) {
    return(-1);
  }
  return(CELL_OK);
}
// end of wired controller specific methods 

// start of wireless controller specific methods
static int32_t get_device_desc(int32_t dev_id, void *p) {
  (void) dev_id;
  UsbDeviceDescriptor *desc = (UsbDeviceDescriptor *)p;

  // do nothing
  return(CELL_OK);
}

static int32_t get_configration_desc(int32_t dev_id, void *p) {
  (void) dev_id;
  UsbConfigurationDescriptor *desc = (UsbConfigurationDescriptor *)p;
  uint16_t wTotalLength;
  wTotalLength = SWAP16(desc->wTotalLength);

  // do nothing
  return(CELL_OK);
}

static int32_t get_interface_desc(int32_t dev_id, void *p) {
  (void) dev_id;
  UsbInterfaceDescriptor *desc = (UsbInterfaceDescriptor *)p;

  // do nothing
  return(CELL_OK);
}

static int32_t get_endpoint_desc(int32_t dev_id, void *p) {
  (void) dev_id;
  UsbEndpointDescriptor *edesc = (UsbEndpointDescriptor *)p;
  int32_t payload;
  XPAD_UNIT_t *unit;

  if (edesc->bEndpointAddress == 0x81 || edesc->bEndpointAddress == 0x83 || edesc->bEndpointAddress == 0x85 || edesc->bEndpointAddress == 0x87) {
    payload = SWAP16(edesc->wMaxPacketSize);
    if ((unit = unit_alloc(dev_id, payload, (edesc->bEndpointAddress - 0x01) & 0x0f, 0, XTYPE_XBOX360W)) == NULL) {
      return(CELL_USBD_ATTACH_FAILED);
    }
    unit->name = "Wireless pad";
    if ((unit->c_pipe = cellUsbdOpenPipe(dev_id, NULL)) < 0) {
      unit_release(unit);
      return(CELL_USBD_ATTACH_FAILED);
    }
    if ((unit->i_pipe = cellUsbdOpenPipe(dev_id, edesc)) < 0) {
      unit_release(unit);
      return(CELL_USBD_ATTACH_FAILED);
    }
    edesc->bEndpointAddress &= 0x0f; // XBox controller out endpoint, ex: 0x81 & 0x0f == 0x01
    if ((unit->o_pipe = cellUsbdOpenPipe(dev_id, edesc)) < 0) {
      unit_release(unit);
      return(CELL_USBD_ATTACH_FAILED);
    }

    // endpoint found; no pad slot or LDD pad is claimed yet - that only
    // happens when the receiver reports a controller connection (0x08 0x80)
    if (unit_submit_begin(unit) &&
        cellUsbdSetConfiguration(unit->c_pipe, 1, set_config_done, unit) != CELL_OK) {
      unit_submit_end(unit);
    }
  }
  return(CELL_USBD_ATTACH_SUCCEEDED);
}

static int32_t xpadw_probe(int32_t dev_id) {
  uint16_t idVendor, idProduct;
  uint32_t i;
  UsbDeviceDescriptor *ddesc;
  UsbInterfaceDescriptor *idesc;

  int32_t free_units = 0;

  // the receiver needs one endpoint unit per possible pad (no pad slots yet)
  block(xpad_mutex);
  for (i = 0; i < MAX_UNIT_NUM; i++) {
    if (XPAD.units[i] == NULL) {
      free_units++;
    }
  }
  unblock(xpad_mutex);
  if (free_units < MAX_XPADW_NUM) {
    return(CELL_USBD_PROBE_FAILED);
  }
  if ((ddesc = (UsbDeviceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_DEVICE)) == NULL) {
    return(CELL_USBD_PROBE_FAILED);
  }
  idesc = (UsbInterfaceDescriptor *)ddesc;
  if ((idesc = (UsbInterfaceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, idesc, USB_DESCRIPTOR_TYPE_INTERFACE)) == NULL) {
    return(CELL_USBD_PROBE_FAILED);
  }

  // make sure product id and vendor id are valid
  idVendor = SWAP16(ddesc->idVendor);
  idProduct = SWAP16(ddesc->idProduct);
  for (i = 0; i < MAX_XPADW_DEV_NUM; i++) {
    if (xpadw_info[i].vid == idVendor && xpadw_info[i].pid == idProduct) {
      return(CELL_USBD_PROBE_SUCCEEDED);
    }
  }
  for (i = 0; i < (uint32_t)extra_count; i++) {
    if (extra_info[i].xtype == XTYPE_XBOX360W && extra_info[i].vid == idVendor && extra_info[i].pid == idProduct) {
      return(CELL_USBD_PROBE_SUCCEEDED);
    }
  }
  return(CELL_USBD_PROBE_FAILED);
}

static int xpadw_attach(int32_t dev_id) {
  uint8_t* desc = 0;
  uint32_t i;
  UsbDeviceDescriptor *ddesc;
  uint16_t idVendor = 0, idProduct = 0;
  const char *rname;
  char msg[96], *m;

  if ((ddesc = (UsbDeviceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_DEVICE)) != NULL) {
    idVendor = SWAP16(ddesc->idVendor);
    idProduct = SWAP16(ddesc->idProduct);
  }
  rname = find_device_name(idVendor, idProduct);
  m = msg;
  m = append_str(m, "XPAD: ");
  m = append_str(m, rname ? rname : "Wireless receiver");
  m = append_str(m, " (");
  m = append_hex16(m, idVendor);
  m = append_str(m, ":");
  m = append_hex16(m, idProduct);
  m = append_str(m, ") connected");
  show_msg(msg);

  // Xbox 360 wireless receivers have 4 endpoints (1 per controller)
  // all 4 need to be listened to at all times in case of controller connection/disconnection
  // traverse through its usb device descriptor and find the endpoints
  while (1) {
    if ((desc = cellUsbdScanStaticDescriptor(dev_id, desc, 0)) == 0) {
        break;
    }
    for (i = 0; i < DESCRIPTOR_TABLE_SIZE; i++) {
      if (descriptor_table[i].bDescriptorType == desc[1]) {
        break;
      }
    }
    if (i != DESCRIPTOR_TABLE_SIZE) {
      descriptor_table[i].dump_descriptor(dev_id, desc);
    }
  }
  return(CELL_USBD_ATTACH_SUCCEEDED);
}

static int32_t xpadw_detach(int32_t dev_id) {
  int32_t r;

  // Xbox wireless receiver has been unplugged
  // disconnect all virtual controllers associated to it
  r = xpadw_detach_all();
  show_msg((char *)"XPAD: Wireless receiver disconnected");
  return(r);
}

static int32_t xpadw_detach_all(void) {
  int32_t i;
  XPAD_UNIT_t *unit;

  // detach all wireless controllers
  block(xpad_mutex);
  for (i = 0; i < MAX_UNIT_NUM; i++) {
    if ((unit = XPAD.units[i]) != NULL && unit->xtype == XTYPE_XBOX360W) {
      XPAD.units[i] = NULL;
      unregister_ldd_controller(unit);
      unit_free(unit);
    }
  }
  unblock(xpad_mutex);
  return(CELL_USBD_DETACH_SUCCEEDED);
}

static void xpadw_read_report(int32_t id, uint8_t *readBuf) {
  uint16_t *digit0, *digit1, *digit2,
           *analog_rx, *analog_ry, *analog_lx, *analog_ly,
           *press_l2, *press_r2, *press_l1, *press_r1,
           *press_up, *press_down, *press_left, *press_right,
           *press_cross, *press_square, *press_circle, *press_triangle;
  XBOX360W_IN_REPORT *report;
  CellPadData data;

  report = (XBOX360W_IN_REPORT *)readBuf;
  memset(&data, 0, sizeof(CellPadData));
  data.len = 24;

  // map location of each button in virtual pad's data
  digit0 = &data.button[0];
  digit1 = &data.button[CELL_PAD_BTN_OFFSET_DIGITAL1];
  digit2 = &data.button[CELL_PAD_BTN_OFFSET_DIGITAL2];
  analog_rx = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X];
  analog_ry = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y];
  analog_lx = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X];
  analog_ly = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y];
  press_l2 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_L2];
  press_r2 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_R2];
  press_l1 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_L1];
  press_r1 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_R1];
  press_right = &data.button[CELL_PAD_BTN_OFFSET_PRESS_RIGHT];
  press_left = &data.button[CELL_PAD_BTN_OFFSET_PRESS_LEFT];
  press_up = &data.button[CELL_PAD_BTN_OFFSET_PRESS_UP];
  press_down = &data.button[CELL_PAD_BTN_OFFSET_PRESS_DOWN];
  press_cross = &data.button[CELL_PAD_BTN_OFFSET_PRESS_CROSS];
  press_square = &data.button[CELL_PAD_BTN_OFFSET_PRESS_SQUARE];
  press_circle = &data.button[CELL_PAD_BTN_OFFSET_PRESS_CIRCLE];
  press_triangle = &data.button[CELL_PAD_BTN_OFFSET_PRESS_TRIANGLE];

  // set default controller values
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_X] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_Y] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_Z] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_G] = 0x0200;

  // read values from Xbox controller and map it to the virtual pad
  *digit0 = (report->buttons & btnXbox) ? *digit0 | CELL_PAD_CTRL_LDD_PS : *digit0 & ~CELL_PAD_CTRL_LDD_PS;
  *digit1 = (report->buttons & btnDigiLeft) ? *digit1 | CELL_PAD_CTRL_LEFT : *digit1 & ~CELL_PAD_CTRL_LEFT;
  *digit1 = (report->buttons & btnDigiDown) ? *digit1 | CELL_PAD_CTRL_DOWN : *digit1 & ~CELL_PAD_CTRL_DOWN;
  *digit1 = (report->buttons & btnDigiRight) ? *digit1 | CELL_PAD_CTRL_RIGHT : *digit1 & ~CELL_PAD_CTRL_RIGHT;
  *digit1 = (report->buttons & btnDigiUp) ? *digit1 | CELL_PAD_CTRL_UP : *digit1 & ~CELL_PAD_CTRL_UP;
  *digit1 = (report->buttons & btnStart) ? *digit1 | CELL_PAD_CTRL_START : *digit1 & ~CELL_PAD_CTRL_START;
  *digit1 = (report->buttons & btnHatRight) ? *digit1 | CELL_PAD_CTRL_R3 : *digit1 & ~CELL_PAD_CTRL_R3;
  *digit1 = (report->buttons & btnHatLeft) ? *digit1 | CELL_PAD_CTRL_L3 : *digit1 & ~CELL_PAD_CTRL_L3;
  *digit1 = (report->buttons & btnBack) ? *digit1 | CELL_PAD_CTRL_SELECT : *digit1 & ~CELL_PAD_CTRL_SELECT;
  *digit2 = (report->buttons & btnX) ? *digit2 | CELL_PAD_CTRL_SQUARE : *digit2 & ~CELL_PAD_CTRL_SQUARE;
  *digit2 = (report->buttons & btnA) ? *digit2 | CELL_PAD_CTRL_CROSS : *digit2 & ~CELL_PAD_CTRL_CROSS;
  *digit2 = (report->buttons & btnB) ? *digit2 | CELL_PAD_CTRL_CIRCLE : *digit2 & ~CELL_PAD_CTRL_CIRCLE;
  *digit2 = (report->buttons & btnY) ? *digit2 | CELL_PAD_CTRL_TRIANGLE : *digit2 & ~CELL_PAD_CTRL_TRIANGLE;
  *digit2 = (report->buttons & btnShoulderRight) ? *digit2 | CELL_PAD_CTRL_R1 : *digit2 & ~CELL_PAD_CTRL_R1;
  *digit2 = (report->buttons & btnShoulderLeft) ? *digit2 | CELL_PAD_CTRL_L1 : *digit2 & ~CELL_PAD_CTRL_L1;
  *digit2 = (report->trigL > 0) ? *digit2 | CELL_PAD_CTRL_L2 : *digit2 & ~CELL_PAD_CTRL_L2;
  *digit2 = (report->trigR > 0) ? *digit2 | CELL_PAD_CTRL_R2 : *digit2 & ~CELL_PAD_CTRL_R2;

  // emulate pressure values except for L2 and R2, button presses correspond to max sensitivity value
  *press_l2 = (report->trigL);
  *press_r2 = (report->trigR);
  *press_l1 = (report->buttons & btnShoulderLeft) ? 0xFF : 0;
  *press_r1 = (report->buttons & btnShoulderRight) ? 0xFF : 0;
  *press_up = (report->buttons & btnDigiUp) ? 0xFF : 0;
  *press_down = (report->buttons & btnDigiDown) ? 0xFF : 0;
  *press_left = (report->buttons & btnDigiLeft) ? 0xFF : 0;
  *press_right = (report->buttons & btnDigiRight) ? 0xFF : 0;
  *press_square = (report->buttons & btnX) ? 0xFF : 0;
  *press_circle = (report->buttons & btnB) ? 0xFF : 0;
  *press_cross = (report->buttons & btnA) ? 0xFF : 0;
  *press_triangle = (report->buttons & btnY) ? 0xFF : 0;

  // PS3 pads use 8 bit values for each axis while Xbox pads use 16 bit
  // convert Xbox analog values for PS3 compatibility 
  *analog_rx = (report->right.x-0x80) & 0x00FF;
  *analog_ry = ((report->right.y ^ 0xFF)-0x80) & 0x00FF;
  *analog_lx = (report->left.x-0x80) & 0x00FF;
  *analog_ly = ((report->left.y ^ 0xFF)-0x80) & 0x00FF;

  // send pad data to virtual pad
  cellPadLddDataInsert(handle[id], &data);
}

static int32_t xpadw_read_input(XPAD_UNIT_t *unit, void *data) {
  unsigned char *p;
  unsigned char *pkt;
  unsigned char *xpadbuf;
  int32_t have_report = 0;
  int32_t pending = 0; /* +1 connect, -1 disconnect; last event wins */
  XBOX360W_IN_REPORT *report;

  p = (unsigned char *)data;
  if (unit == NULL) {
    return(-1);
  }

  // drain the whole ring buffer each tick; keep only the newest movement
  // report but honor every connect/disconnect event in order
  block(unit->rb_mutex);
  while (unit->rblen > 0) {
    xpadbuf = &unit->ringbuf[unit->rp][0];
    pkt = &xpadbuf[2];
    if (xpadbuf[1] >= 2 && pkt[0] == 0x08) {
      if (pkt[1] == 0x80) {
        pending = 1; // controller connected to receiver
      } else if (pkt[1] == 0x00) {
        pending = -1; // controller disconnected from receiver
        have_report = 0; // drop any report from before the disconnect
      }
    }
    report = (XBOX360W_IN_REPORT *)pkt;
    if ((xpadbuf[1] >= sizeof(XBOX360W_IN_REPORT)) && (pkt[1] == 0x01) &&
        (report->header.command == inReport) && (report->header.size == sizeof(XBOX360W_IN_REPORT))) {
      memcpy(p, pkt, xpadbuf[1]);
      have_report = 1;
    }
    if (++unit->rp >= RINGBUF_SIZE) {
      unit->rp = 0;
    }
    unit->rblen--;
  }
  unblock(unit->rb_mutex);

  // register/unregister outside the ring-buffer lock: registration sleeps
  // 10 ms and must not stall USB completion callbacks for other pads
  if (pending > 0) {
    if (unit->number < 0) { // newly connected pad
      if (register_ldd_controller(unit) == CELL_PAD_OK) {
        notify_pad_connected(unit);
      } else {
        show_msg((char *)"XPAD: no free pad slot for wireless pad");
      }
    }
  } else if (pending < 0) {
    if (unit->number >= 0) { // pad was actually registered
      unregister_ldd_controller(unit);
      notify_pad_disconnected(unit);
    }
  }
  if (have_report && unit->number >= 0 && handle[unit->number] >= 0) {
    xpadw_read_report(unit->number, p);
  }
  return(0);
}

static int32_t xpadw_set_led(XPAD_UNIT_t *unit, uint8_t led) {
  uint8_t out[4] = {0x00, 0x00, 0x08, led | 0x40};

  if (write_xpad(unit, out, 4) < 0) {
    return(-1);
  }
  return(CELL_OK);
}

static int32_t xpadw_set_rumble(XPAD_UNIT_t *unit, uint8_t lval, uint8_t rval) {
  // full 12-byte wireless rumble command, matching the Linux xpad driver and
  // xboxdrv; a truncated payload leaves the motors unset on many receivers
  uint8_t out[12] = {0x00, 0x01, 0x0f, 0xc0, 0x00, lval, rval, 0x00, 0x00, 0x00, 0x00, 0x00};

  if (write_xpad(unit, out, 12) < 0) {
    return(-1);
  }
  return(CELL_OK);
}
// end of wireless controller specific methods

// ---------------------------------------------------------------------------
// start of Nintendo Switch Pro controller specific methods
// ---------------------------------------------------------------------------

// Send one subcommand. The report is always the full 16 bytes: report id,
// counter, 8 rumble bytes, subcommand code, then arguments (unused here).
// The counter advances 0x00..0x0F and wraps; the pad echoes it back.
static int32_t swpro_send_subcommand(XPAD_UNIT_t *unit, uint8_t subcmd, uint8_t arg) {
  uint8_t out[SWPRO_OUT_REPORT_LEN];
  int32_t r;

  memset(out, 0, sizeof(out));
  out[0] = SWPRO_OUT_SUBCOMMAND;
  out[1] = unit->swpro_counter & 0x0F;
  memcpy(&out[2], swpro_rumble_neutral, 8);
  out[10] = subcmd;
  out[11] = arg;

  // advance the counter for the next subcommand (wraps at 0x0F, per protocol)
  unit->swpro_counter = (uint8_t)((unit->swpro_counter + 1) & 0x0F);

  r = write_xpad(unit, out, SWPRO_OUT_REPORT_LEN);
  if (r < 0) {
      xlog_code("Switch send_subcommand write_xpad failed", r);
  }
  return(r);
}

static int32_t swpro_probe(int32_t dev_id) {
  uint16_t idVendor, idProduct;
  uint32_t i;
  UsbDeviceDescriptor *ddesc;

  if ((ddesc = (UsbDeviceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_DEVICE)) == NULL) {
    return(CELL_USBD_PROBE_FAILED);
  }
  idVendor = SWAP16(ddesc->idVendor);
  idProduct = SWAP16(ddesc->idProduct);

  for (i = 0; i < MAX_SWITCH_DEV_NUM; i++) {
    if (switch_info[i].vid == idVendor && switch_info[i].pid == idProduct) {
      xlog_dev("probe matched (Switch Pro)", idVendor, idProduct, 0);
      return(CELL_USBD_PROBE_SUCCEEDED);
    }
  }
  for (i = 0; i < (uint32_t)extra_count; i++) {
    if (extra_info[i].xtype == XTYPE_SWITCH_PRO && extra_info[i].vid == idVendor && extra_info[i].pid == idProduct) {
      xlog_dev("probe matched (Switch Pro, devices.txt)", idVendor, idProduct, 0);
      return(CELL_USBD_PROBE_SUCCEEDED);
    }
  }
  xlog_dev("probe rejected: VID/PID not a Switch Pro pad", idVendor, idProduct, 0);
  return(CELL_USBD_PROBE_FAILED);
}

static int32_t swpro_attach(int32_t dev_id) {
  int32_t payload;
  uint8_t *desc = NULL;
  UsbConfigurationDescriptor *cdesc;
  UsbInterfaceDescriptor *ifd, *cur_if = NULL, *hid_if = NULL;
  UsbEndpointDescriptor *ed, *ep_in = NULL, *ep_out = NULL;
  XPAD_UNIT_t *unit;
  UsbDeviceDescriptor *ddesc;
  uint16_t idVendor = 0, idProduct = 0;

  if ((ddesc = (UsbDeviceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_DEVICE)) != NULL) {
    idVendor = SWAP16(ddesc->idVendor);
    idProduct = SWAP16(ddesc->idProduct);
  }
  xlog_dev("Switch attach starting for", idVendor, idProduct, 0);
  if ((cdesc = (UsbConfigurationDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_CONFIGURATION)) == NULL) {
    xlog("Switch attach failed: no configuration descriptor");
    return(CELL_USBD_ATTACH_FAILED);
  }

  // Nintendo pads present a HID interface (class 0x03). Take its interrupt
  // IN and OUT endpoints; the OUT endpoint is mandatory because every mode
  // change and rumble command is sent on it.
  while ((desc = (uint8_t *)cellUsbdScanStaticDescriptor(dev_id, desc, 0)) != NULL) {
    if (desc[1] == USB_DESCRIPTOR_TYPE_INTERFACE) {
      ifd = (UsbInterfaceDescriptor *)desc;
      if (hid_if == NULL && ifd->bInterfaceClass == 0x03) {
        hid_if = ifd;
      }
      cur_if = ifd;
    } else if (desc[1] == USB_DESCRIPTOR_TYPE_ENDPOINT) {
      ed = (UsbEndpointDescriptor *)desc;
      if (cur_if != NULL && cur_if == hid_if) {
        char dbg[96], *m = dbg;
        m = append_str(m, "Switch ep 0x");
        m = append_hex16(m, ed->bEndpointAddress);
        m = append_str(m, " attr 0x");
        m = append_hex16(m, ed->bmAttributes);
        m = append_str(m, " maxpkt ");
        m = append_int(m, SWAP16(ed->wMaxPacketSize));
        xlog(dbg);
        if ((ed->bmAttributes & 0x03) != 0x03) { // interrupt endpoints only
          continue;
        }
        if ((ed->bEndpointAddress & 0x80) && ep_in == NULL) {
          ep_in = ed;
        } else if (!(ed->bEndpointAddress & 0x80) && ep_out == NULL) {
          ep_out = ed;
        }
      }
    }
  }
  if (hid_if == NULL || ep_in == NULL) {
    xlog("Switch attach failed: no HID interface or interrupt-IN endpoint");
    return(CELL_USBD_ATTACH_FAILED);
  }
  if (ep_out == NULL) {
    // no OUT endpoint: subcommands and rumble are impossible, so the pad
    // could only ever work in its default simple-HID mode. Reject it rather
    // than registering a controller that can never be configured.
    xlog("Switch attach failed: no interrupt-OUT endpoint (needed for rumble/mode)");
    return(CELL_USBD_ATTACH_FAILED);
  }
  payload = SWAP16(ep_in->wMaxPacketSize);
  // the endpoint must carry the whole 54-byte full report; a smaller endpoint
  // means this is not a full-size pad (and the sticks would be truncated)
  if (payload < SWPRO_FULL_REPORT_LEN) {
    xlog_code("Switch attach failed: IN endpoint too small", payload);
    return(CELL_USBD_ATTACH_FAILED);
  }

  if ((unit = unit_alloc(dev_id, payload, hid_if->bInterfaceNumber, hid_if->bAlternateSetting, XTYPE_SWITCH_PRO)) == NULL) {
    xlog("Switch attach failed: unit alloc (out of memory or unit list full)");
    return(CELL_USBD_ATTACH_FAILED);
  }
  unit->vid = idVendor;
  unit->pid = idProduct;
  unit->name = find_device_name(idVendor, idProduct);
  if ((unit->c_pipe = cellUsbdOpenPipe(dev_id, NULL)) < 0) {
    xlog_code("Switch attach failed: control pipe open", unit->c_pipe);
    unit_release(unit);
    return(CELL_USBD_ATTACH_FAILED);
  }
  if ((unit->i_pipe = cellUsbdOpenPipe(dev_id, ep_in)) < 0) {
    xlog_code("Switch attach failed: IN pipe open", unit->i_pipe);
    unit_release(unit);
    return(CELL_USBD_ATTACH_FAILED);
  }
  if ((unit->o_pipe = cellUsbdOpenPipe(dev_id, ep_out)) < 0) {
    xlog_code("Switch attach failed: OUT pipe open", unit->o_pipe);
    unit_release(unit);
    return(CELL_USBD_ATTACH_FAILED);
  }

  // A Switch pad is only useful with an LDD pad behind it, so claim the pad
  // slot and register before starting configuration.
  block(xpad_mutex);
  if (register_ldd_controller(unit) != CELL_PAD_OK) {
    unblock(xpad_mutex);
    unit_release(unit);
    show_msg((char *)"XPAD: no free pad slot for Switch Pro controller");
    return(CELL_USBD_ATTACH_FAILED);
  }
  unblock(xpad_mutex);
  cellUsbdSetPrivateData(dev_id, unit);

  unit->swpro_state = SWPRO_STATE_IDLE;
  unit->swpro_timer = 0;
  unit->swpro_inited = 0;
  // switch the pad into full mode + IMU + vibration before streaming starts;
  // the set_configuration callback re-sends the LED, so do the init after it
  if (unit_submit_begin(unit)) {
    int32_t cr = cellUsbdSetConfiguration(unit->c_pipe, cdesc->bConfigurationValue, set_config_done, unit);
    if (cr != CELL_OK) {
      xlog_code("Switch attach: set_configuration submit failed", cr);
      unit_submit_end(unit);
    }
  }

  // full mode + IMU + vibration are enabled in set_config_done() once the
  // device is configured; nothing to do here
  notify_pad_connected(unit);
  return(CELL_USBD_ATTACH_SUCCEEDED);
}

static int32_t swpro_detach(int32_t dev_id) {
  XPAD_UNIT_t *unit;

  if ((unit = (XPAD_UNIT_t *)cellUsbdGetPrivateData(dev_id)) == NULL) {
    xlog("Switch USB detach for unknown device (no private data)");
    return(CELL_USBD_DETACH_FAILED);
  }
  xlog_dev("Switch USB detach:", unit->vid, unit->pid, 0);
  block(xpad_mutex);
  if (unit->unit_idx >= 0 && unit->unit_idx < MAX_UNIT_NUM && XPAD.units[unit->unit_idx] == unit) {
    XPAD.units[unit->unit_idx] = NULL;
  }
  unregister_ldd_controller(unit);
  unblock(xpad_mutex);
  notify_pad_disconnected(unit);
  unit_free(unit);
  return(CELL_USBD_DETACH_SUCCEEDED);
}

static int32_t swpro_detach_all(void) {
  int32_t i;
  XPAD_UNIT_t *unit;

  block(xpad_mutex);
  for (i = 0; i < MAX_UNIT_NUM; i++) {
    if ((unit = XPAD.units[i]) != NULL && unit->xtype == XTYPE_SWITCH_PRO) {
      XPAD.units[i] = NULL;
      unregister_ldd_controller(unit);
      unit_free(unit);
    }
  }
  unblock(xpad_mutex);
  return(CELL_USBD_DETACH_SUCCEEDED);
}

#define SWPRO_BTN_HOME    (1 << 4)
#define SWPRO_BTN_CAPTURE (1 << 5)

static uint8_t swpro_scale_axis_12bit(uint16_t raw, int invert) {
  int32_t d = (int32_t)raw - 2048;
  int32_t v;
  // Deadzone around 2048 (~7%)
  if (d > -140 && d < 140) {
    return 0x80;
  }
  // Useful factory travel on Switch Pro / 8BitDo is ~ +/- 1400 around 2048
  v = 128 + ((d * 127) / 1400);
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  if (invert) v = 255 - v;
  return (uint8_t)v;
}

// Translate one Switch Pro input report into a libpad data block.
static void swpro_read_report(int32_t id, uint8_t *readBuf) {
  uint16_t *digit0, *digit1, *digit2,
           *analog_rx, *analog_ry, *analog_lx, *analog_ly,
           *press_l2, *press_r2, *press_l1, *press_r1,
           *press_up, *press_down, *press_left, *press_right,
           *press_cross, *press_square, *press_circle, *press_triangle;
  CellPadData data;

  memset(&data, 0, sizeof(CellPadData));
  data.len = 24;

  digit0 = &data.button[0];
  digit1 = &data.button[CELL_PAD_BTN_OFFSET_DIGITAL1];
  digit2 = &data.button[CELL_PAD_BTN_OFFSET_DIGITAL2];
  analog_rx = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X];
  analog_ry = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y];
  analog_lx = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X];
  analog_ly = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y];
  press_l2 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_L2];
  press_r2 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_R2];
  press_l1 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_L1];
  press_r1 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_R1];
  press_up = &data.button[CELL_PAD_BTN_OFFSET_PRESS_UP];
  press_down = &data.button[CELL_PAD_BTN_OFFSET_PRESS_DOWN];
  press_left = &data.button[CELL_PAD_BTN_OFFSET_PRESS_LEFT];
  press_right = &data.button[CELL_PAD_BTN_OFFSET_PRESS_RIGHT];
  press_cross = &data.button[CELL_PAD_BTN_OFFSET_PRESS_CROSS];
  press_square = &data.button[CELL_PAD_BTN_OFFSET_PRESS_SQUARE];
  press_circle = &data.button[CELL_PAD_BTN_OFFSET_PRESS_CIRCLE];
  press_triangle = &data.button[CELL_PAD_BTN_OFFSET_PRESS_TRIANGLE];

  // centred defaults
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_X] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_Y] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_Z] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_G] = 0x0200;

  if (readBuf[0] == SWPRO_IN_FULL || readBuf[0] == SWPRO_IN_SUBCMD_REPLY) {
    if (readBuf[0] == SWPRO_IN_SUBCMD_REPLY) {
      static uint8_t s_subcmd_ack_logged = 0;
      if (!s_subcmd_ack_logged) {
        s_subcmd_ack_logged = 1;
        xlog_code("Switch: Received subcmd reply for subcmd", (int32_t)readBuf[14]);
      }
    }
    uint8_t btn_right  = readBuf[3];
    uint8_t btn_shared = readBuf[4];
    uint8_t btn_left   = readBuf[5];

    uint16_t lx_12 = (uint16_t)readBuf[6] | ((uint16_t)(readBuf[7] & 0x0F) << 8);
    uint16_t ly_12 = (uint16_t)(readBuf[7] >> 4) | ((uint16_t)readBuf[8] << 4);
    uint16_t rx_12 = (uint16_t)readBuf[9] | ((uint16_t)(readBuf[10] & 0x0F) << 8);
    uint16_t ry_12 = (uint16_t)(readBuf[10] >> 4) | ((uint16_t)readBuf[11] << 4);

    *analog_lx = swpro_scale_axis_12bit(lx_12, 0);
    *analog_ly = swpro_scale_axis_12bit(ly_12, 1); // invert Y for PS3
    *analog_rx = swpro_scale_axis_12bit(rx_12, 0);
    *analog_ry = swpro_scale_axis_12bit(ry_12, 1); // invert Y for PS3

    // Physical layout matches PlayStation layout:
    // Y (left)   -> Square
    // X (top)    -> Triangle
    // B (bottom) -> Cross
    // A (right)  -> Circle
    *digit2 = (btn_right & SWPRO_BTN_Y) ? *digit2 | CELL_PAD_CTRL_SQUARE : *digit2 & ~CELL_PAD_CTRL_SQUARE;
    *digit2 = (btn_right & SWPRO_BTN_X) ? *digit2 | CELL_PAD_CTRL_TRIANGLE : *digit2 & ~CELL_PAD_CTRL_TRIANGLE;
    *digit2 = (btn_right & SWPRO_BTN_B) ? *digit2 | CELL_PAD_CTRL_CROSS : *digit2 & ~CELL_PAD_CTRL_CROSS;
    *digit2 = (btn_right & SWPRO_BTN_A) ? *digit2 | CELL_PAD_CTRL_CIRCLE : *digit2 & ~CELL_PAD_CTRL_CIRCLE;

    *digit1 = (btn_left & SWPRO_BTN_UP) ? *digit1 | CELL_PAD_CTRL_UP : *digit1 & ~CELL_PAD_CTRL_UP;
    *digit1 = (btn_left & SWPRO_BTN_DOWN) ? *digit1 | CELL_PAD_CTRL_DOWN : *digit1 & ~CELL_PAD_CTRL_DOWN;
    *digit1 = (btn_left & SWPRO_BTN_LEFT) ? *digit1 | CELL_PAD_CTRL_LEFT : *digit1 & ~CELL_PAD_CTRL_LEFT;
    *digit1 = (btn_left & SWPRO_BTN_RIGHT) ? *digit1 | CELL_PAD_CTRL_RIGHT : *digit1 & ~CELL_PAD_CTRL_RIGHT;
    *digit1 = (btn_shared & SWPRO_BTN_PLUS) ? *digit1 | CELL_PAD_CTRL_START : *digit1 & ~CELL_PAD_CTRL_START;
    *digit1 = (btn_shared & SWPRO_BTN_MINUS) ? *digit1 | CELL_PAD_CTRL_SELECT : *digit1 & ~CELL_PAD_CTRL_SELECT;
    *digit1 = (btn_shared & SWPRO_BTN_RSTICK) ? *digit1 | CELL_PAD_CTRL_R3 : *digit1 & ~CELL_PAD_CTRL_R3;
    *digit1 = (btn_shared & SWPRO_BTN_LSTICK) ? *digit1 | CELL_PAD_CTRL_L3 : *digit1 & ~CELL_PAD_CTRL_L3;

    if (btn_shared & SWPRO_BTN_HOME) {
      *digit0 |= CELL_PAD_CTRL_LDD_PS;
    }

    *digit2 = (btn_right & SWPRO_BTN_R) ? *digit2 | CELL_PAD_CTRL_R1 : *digit2 & ~CELL_PAD_CTRL_R1;
    *digit2 = (btn_left & SWPRO_BTN_L) ? *digit2 | CELL_PAD_CTRL_L1 : *digit2 & ~CELL_PAD_CTRL_L1;
    *digit2 = (btn_right & SWPRO_BTN_ZR) ? *digit2 | CELL_PAD_CTRL_R2 : *digit2 & ~CELL_PAD_CTRL_R2;
    *digit2 = (btn_left & SWPRO_BTN_ZL) ? *digit2 | CELL_PAD_CTRL_L2 : *digit2 & ~CELL_PAD_CTRL_L2;

    *press_r1 = (btn_right & SWPRO_BTN_R) ? 0xFF : 0;
    *press_l1 = (btn_left & SWPRO_BTN_L) ? 0xFF : 0;
    *press_r2 = (btn_right & SWPRO_BTN_ZR) ? 0xFF : 0;
    *press_l2 = (btn_left & SWPRO_BTN_ZL) ? 0xFF : 0;

    *press_up = (btn_left & SWPRO_BTN_UP) ? 0xFF : 0;
    *press_down = (btn_left & SWPRO_BTN_DOWN) ? 0xFF : 0;
    *press_left = (btn_left & SWPRO_BTN_LEFT) ? 0xFF : 0;
    *press_right = (btn_left & SWPRO_BTN_RIGHT) ? 0xFF : 0;
    *press_square = (btn_right & SWPRO_BTN_Y) ? 0xFF : 0;
    *press_triangle = (btn_right & SWPRO_BTN_X) ? 0xFF : 0;
    *press_cross = (btn_right & SWPRO_BTN_B) ? 0xFF : 0;
    *press_circle = (btn_right & SWPRO_BTN_A) ? 0xFF : 0;

  } else if (readBuf[0] == SWPRO_IN_SIMPLE) {
    // 0x3F simple HID report: readBuf[1..2] buttons, readBuf[3] hat, readBuf[4..11] 4 axes 16-bit LE
    uint16_t b = (uint16_t)readBuf[1] | ((uint16_t)readBuf[2] << 8);
    uint8_t hat = readBuf[3];

    // Hat to D-pad
    if (hat == 0 || hat == 1 || hat == 7) *digit1 |= CELL_PAD_CTRL_UP;
    if (hat == 1 || hat == 2 || hat == 3) *digit1 |= CELL_PAD_CTRL_RIGHT;
    if (hat == 3 || hat == 4 || hat == 5) *digit1 |= CELL_PAD_CTRL_DOWN;
    if (hat == 5 || hat == 6 || hat == 7) *digit1 |= CELL_PAD_CTRL_LEFT;

    // Face buttons: Simple HID: bit0=A, bit1=B, bit2=X, bit3=Y
    if (b & (1 << 3)) *digit2 |= CELL_PAD_CTRL_SQUARE;   // Y (left) -> Square
    if (b & (1 << 2)) *digit2 |= CELL_PAD_CTRL_TRIANGLE; // X (top) -> Triangle
    if (b & (1 << 1)) *digit2 |= CELL_PAD_CTRL_CROSS;    // B (bottom) -> Cross
    if (b & (1 << 0)) *digit2 |= CELL_PAD_CTRL_CIRCLE;   // A (right) -> Circle

    if (b & (1 << 9)) *digit1 |= CELL_PAD_CTRL_START;    // Plus
    if (b & (1 << 8)) *digit1 |= CELL_PAD_CTRL_SELECT;   // Minus
    if (b & (1 << 10)) *digit1 |= CELL_PAD_CTRL_L3;      // L3
    if (b & (1 << 11)) *digit1 |= CELL_PAD_CTRL_R3;      // R3
    if (b & (1 << 12)) *digit0 |= CELL_PAD_CTRL_LDD_PS;  // Home

    if (b & (1 << 5)) { *digit2 |= CELL_PAD_CTRL_R1; *press_r1 = 0xFF; }
    if (b & (1 << 4)) { *digit2 |= CELL_PAD_CTRL_L1; *press_l1 = 0xFF; }
    if (b & (1 << 7)) { *digit2 |= CELL_PAD_CTRL_R2; *press_r2 = 0xFF; }
    if (b & (1 << 6)) { *digit2 |= CELL_PAD_CTRL_L2; *press_l2 = 0xFF; }

    *analog_lx = readBuf[5]; // take high byte of 16-bit LE (or middle)
    *analog_ly = 255 - readBuf[7];
    *analog_rx = readBuf[9];
    *analog_ry = 255 - readBuf[11];
  } else {
    return;
  }

  cellPadLddDataInsert(handle[id], &data);
}

static int32_t swpro_read_input(XPAD_UNIT_t *unit, void *data) {
  unsigned char *p;
  unsigned char *xpadbuf;
  int32_t have_report = 0;
  uint8_t first_report = 0;
  int32_t len;

  p = (unsigned char *)data;
  if (unit == NULL) {
    return(-1);
  }

  // drain the whole ring buffer each tick, keeping only the newest report
  block(unit->rb_mutex);
  while (unit->rblen > 0) {
    xpadbuf = &unit->ringbuf[unit->rp][0];
    len = xpadbuf[1];
    if (len >= 3 && (xpadbuf[2] == SWPRO_IN_FULL || xpadbuf[2] == SWPRO_IN_SUBCMD_REPLY || xpadbuf[2] == SWPRO_IN_SIMPLE)) {
      if (len > MAX_XPAD_DATA_LEN - 2) {
        len = MAX_XPAD_DATA_LEN - 2;
      }
      memcpy(p, &xpadbuf[2], len);
      have_report = 1;
      first_report = xpadbuf[2];
    }
    if (++unit->rp >= RINGBUF_SIZE) {
      unit->rp = 0;
    }
    unit->rblen--;
  }
  unblock(unit->rb_mutex);

  if (have_report && unit->number >= 0 && handle[unit->number] >= 0) {
    swpro_read_report(unit->number, p);
    (void)first_report;
  }
  return(0);
}

static int32_t swpro_set_led(XPAD_UNIT_t *unit, uint8_t led) {
  uint8_t pattern;
  static const uint8_t led_map[4] = {0x01, 0x02, 0x04, 0x08}; // solid player 1..4 LEDs

  if (unit == NULL) {
    return(-1);
  }
  // Do NOT send 0x01 subcommands on the USB pipe before the 0x80 USB handshake
  // and mode initialization state machine has finished! process_swpro_init()
  // will send the assigned port LED at SWPRO_STATE_SET_LED.
  if (unit->swpro_state != SWPRO_STATE_DONE) {
    return(CELL_OK);
  }
  if (led >= 0x06 && led <= 0x09) {
    pattern = led_map[(led - 0x06) % 4];
  } else {
    pattern = 0x01;
  }
  return(swpro_send_subcommand(unit, SWPRO_SC_SET_PLAYER, pattern));
}

static int32_t swpro_set_rumble(XPAD_UNIT_t *unit, uint8_t lval, uint8_t rval) {
  uint8_t out[SWPRO_OUT_REPORT_LEN];

  if (unit == NULL || unit->swpro_state != SWPRO_STATE_DONE) {
    return(CELL_OK);
  }
  memset(out, 0, sizeof(out));
  out[0] = SWPRO_OUT_RUMBLE;
  out[1] = unit->swpro_counter & 0x0F;
  unit->swpro_counter = (uint8_t)((unit->swpro_counter + 1) & 0x0F);

  if (lval == 0 && rval == 0) {
    memcpy(&out[2], swpro_rumble_neutral, 8);
  } else {
    // Left motor (4 bytes: out[2..5])
    if (lval > 0) {
      out[2] = 0x88;
      out[3] = 0x00;
      out[4] = 0x62;
      out[5] = (uint8_t)(0x40 + ((lval >> 2) & 0x3F));
    } else {
      memcpy(&out[2], &swpro_rumble_neutral[0], 4);
    }
    // Right motor (4 bytes: out[6..9])
    if (rval > 0) {
      out[6] = 0x10;
      out[7] = 0x88;
      out[8] = 0x62;
      out[9] = (uint8_t)(0x40 + ((rval >> 2) & 0x3F));
    } else {
      memcpy(&out[6], &swpro_rumble_neutral[4], 4);
    }
  }
  if (write_xpad(unit, out, 10) < 0) {
    return(-1);
  }
  return(CELL_OK);
}
// end of Switch Pro controller specific methods

// Bluetooth Dongle bridge methods
int32_t btd_is_custom_dongle(uint16_t vid, uint16_t pid) {
  int32_t i;
  for (i = 0; i < extra_count; i++) {
    if (extra_info[i].xtype == XTYPE_BTD && extra_info[i].vid == vid && extra_info[i].pid == pid) {
      return 1;
    }
  }
  return 0;
}

void *btd_create_switch_unit(void) {
  XPAD_UNIT_t *unit;
  unit = unit_alloc(-1, SWPRO_FULL_REPORT_LEN, 0, 0, XTYPE_SWITCH_PRO);
  if (unit == NULL) {
    xlog("BTD: unit_alloc failed for virtual Switch Pro");
    return NULL;
  }
  unit->vid = 0x057E;
  unit->pid = 0x2009;
  unit->name = "Switch Pro Controller (Bluetooth)";
  unit->is_bt = 1;
  unit->read_input = swpro_read_input;
  unit->set_led = swpro_set_led;
  unit->set_rumble = swpro_set_rumble;
  unit->o_pipe = -2;

  block(xpad_mutex);
  if (register_ldd_controller(unit) != CELL_PAD_OK) {
    unblock(xpad_mutex);
    unit_free(unit);
    xlog("BTD: register_ldd_controller failed for virtual Switch Pro");
    return NULL;
  }
  unblock(xpad_mutex);
  xlog("BTD: virtual Switch Pro registered successfully!");
  notify_pad_connected(unit);
  return (void *)unit;
}

void btd_destroy_switch_unit(void *u) {
  XPAD_UNIT_t *unit = (XPAD_UNIT_t *)u;
  if (unit == NULL) return;
  block(xpad_mutex);
  if (unit->unit_idx >= 0 && unit->unit_idx < MAX_UNIT_NUM && XPAD.units[unit->unit_idx] == unit) {
    XPAD.units[unit->unit_idx] = NULL;
  }
  unregister_ldd_controller(unit);
  unblock(xpad_mutex);
  notify_pad_disconnected(unit);
  unit_free(unit);
}

void btd_start_swpro_init(void *u) {
  XPAD_UNIT_t *unit = (XPAD_UNIT_t *)u;
  if (unit == NULL) return;
  unit->swpro_state = SWPRO_STATE_SET_MODE;
  unit->swpro_timer = 10;
  unit->swpro_inited = 0;
}

void btd_push_report(void *u, const uint8_t *report, int32_t len) {
  XPAD_UNIT_t *unit = (XPAD_UNIT_t *)u;
  unsigned char *xpadbuf;
  if (unit == NULL || report == NULL || len <= 0) return;

  block(unit->rb_mutex);
  if (unit->closing || unit->rblen >= RINGBUF_SIZE) {
    unblock(unit->rb_mutex);
    return;
  }
  xpadbuf = &unit->ringbuf[unit->wp][0];
  xpadbuf[0] = (unsigned char)(++unit->tcount & 0xFF);
  int32_t count = (len <= MAX_XPAD_DATA_LEN - 2) ? len : MAX_XPAD_DATA_LEN - 2;
  xpadbuf[1] = (unsigned char)(count & 0xFF);
  memcpy(&xpadbuf[2], report, count);
  if (++unit->wp >= RINGBUF_SIZE) {
    unit->wp = 0;
  }
  unit->rblen++;
  unblock(unit->rb_mutex);
}

static int32_t check_pad_status(void) {
  int32_t i, cr, port, pad;
  XPAD_UNIT_t *unit;
  CellPadInfo2 pad_info2;

  cr = cellPadGetInfo2(&pad_info2);
  if (cr != CELL_PAD_OK) {
    return(-1);
  }
  for (pad = 0; pad < CELL_PAD_MAX_PORT_NUM; ++pad) {
    if (pad_info2.port_status[pad] & CELL_PAD_STATUS_ASSIGN_CHANGES) {

      // port assignments changed; re-send LEDs only for pads whose port
      // actually moved (only 4 LED patterns exist, so with more than 4
      // pads two controllers will show the same pattern)
      for (i = 0; i < MAX_UNIT_NUM; i++) {
        if ((unit = XPAD.units[i]) != NULL && unit->number >= 0 && handle[unit->number] >= 0) {
          port = cellPadLddGetPortNo(handle[unit->number]);
          if (port >= 0 && port != unit->last_port) {
            unit->last_port = port;
            unit->set_led(unit, xpad_led[port%4]);
          }
        }
      }
      break; // one refresh pass covers all pads
    }
  }
  return(0);
}

// Register one VID/PID with USBD, tolerating duplicates. Returns CELL_OK
// when registered or when this exact identity is already registered (a
// conflicting entry must not take the whole plugin down - the device it
// describes still works through whoever registered it first). Any other
// error is passed to the caller; CELL_USBD_ERROR_NOT_INITIALIZED means
// "retry the pass later", everything else is fatal.
static int32_t register_one(CellUsbdLddOps *ops, uint16_t vid, uint16_t pid, const char *name, uint8_t *lddreg) {
  int32_t r;

  if (*lddreg) {
    return(CELL_OK); // registered by an earlier pass; passes are idempotent
  }
  if (reg_budget_full) {
    // the LDD table already filled up earlier in this pass; every further
    // cellUsbdRegisterExtraLdd() would just return the same error, so skip
    // the call and record the entry as dropped
    reg_skipped_full++;
    return(CELL_OK);
  }
  ops->name = name;
  r = cellUsbdRegisterExtraLdd(ops, vid, pid);
  if (r == CELL_OK) {
    *lddreg = 1;
    xlog_dev("registered", vid, pid, 0);
    return(CELL_OK);
  }
  if (r == CELL_USBD_ERROR_LDD_ALREADY_REGISTERED) {
    reg_conflicts++;
    xlog_dev("skipped, LDD already registered", vid, pid, r);
    return(CELL_OK);
  }
  if (r == CELL_USBD_ERROR_LDD_TABLE_FULL) {
    // cellUsbd is out of LDD slots. This is non-fatal: keep every device
    // that already registered working and leave the plugin active. Latch a
    // flag so the rest of the pass stops hammering USBD, and count what we
    // had to drop for the summary line in init_usb().
    reg_budget_full = 1;
    reg_skipped_full++;
    xlog_dev("LDD table full, entry not registered", vid, pid, r);
    return(CELL_OK);
  }
  reg_fail_vid = vid;
  reg_fail_pid = pid;
  xlog_dev("register FAILED", vid, pid, r);
  return(r);
}

// Register every built-in and user-supplied controller VID/PID with USBD.
// Safe to call repeatedly: entries that already registered are skipped.
static int32_t register_devices(void) {
  int32_t r, i;

  // register Bluetooth Dongle device types FIRST
  for (i = 0; i < MAX_BTD_DEV_NUM; i++) {
    if ((r = register_one(&btd_ops, btd_info[i].vid, btd_info[i].pid, btd_info[i].name, &btd_lddreg[i])) != CELL_OK) {
      return(r);
    }
  }

  // register Nintendo Switch Pro device types FIRST: the LDD table has only
  // ~22 slots and the wired Xbox table below has ~35 entries, so a Switch
  // Pro pad registered after the Xbox table is silently dropped when the
  // table fills. Putting it first guarantees a slot.
  for (i = 0; i < MAX_SWITCH_DEV_NUM; i++) {
    if ((r = register_one(&swpro_ops, switch_info[i].vid, switch_info[i].pid, switch_info[i].name, &switch_lddreg[i])) != CELL_OK) {
      return(r);
    }
  }

  // register wired Xbox controller device types
  for (i = 0; i < MAX_XPAD_DEV_NUM; i++) {
    if ((r = register_one(&xpad_ops, xpad_info[i].vid, xpad_info[i].pid, xpad_info[i].name, &xpad_lddreg[i])) != CELL_OK) {
      return(r);
    }
  }

  // register wireless Xbox controller device types
  for (i = 0; i < MAX_XPADW_DEV_NUM; i++) {
    if ((r = register_one(&xpadw_ops, xpadw_info[i].vid, xpadw_info[i].pid, xpadw_info[i].name, &xpadw_lddreg[i])) != CELL_OK) {
      return(r);
    }
  }

  // register user-supplied device types
  for (i = 0; i < extra_count; i++) {
    if (extra_info[i].xtype == XTYPE_XBOX360W) {
      r = register_one(&xpadw_ops, extra_info[i].vid, extra_info[i].pid, extra_info[i].name, &extra_lddreg[i]);
    } else if (extra_info[i].xtype == XTYPE_SWITCH_PRO) {
      r = register_one(&swpro_ops, extra_info[i].vid, extra_info[i].pid, extra_info[i].name, &extra_lddreg[i]);
    } else if (extra_info[i].xtype == XTYPE_BTD) {
      r = register_one(&btd_ops, extra_info[i].vid, extra_info[i].pid, extra_info[i].name, &extra_lddreg[i]);
    } else {
      r = register_one(&xpad_ops, extra_info[i].vid, extra_info[i].pid, extra_info[i].name, &extra_lddreg[i]);
    }
    if (r != CELL_OK) {
      return(r);
    }
  }
  return(CELL_OK);
}

// undo every registration register_devices() made (and only those)
static void unregister_devices(void) {
  int32_t i;

  for (i = 0; i < MAX_BTD_DEV_NUM; i++) {
    if (btd_lddreg[i]) {
      cellUsbdUnregisterExtraLdd(&btd_ops);
      btd_lddreg[i] = 0;
    }
  }

  for (i = 0; i < MAX_XPAD_DEV_NUM; i++) {
    if (xpad_lddreg[i]) {
      cellUsbdUnregisterExtraLdd(&xpad_ops);
      xpad_lddreg[i] = 0;
    }
  }
  for (i = 0; i < MAX_XPADW_DEV_NUM; i++) {
    if (xpadw_lddreg[i]) {
      cellUsbdUnregisterExtraLdd(&xpadw_ops);
      xpadw_lddreg[i] = 0;
    }
  }
  for (i = 0; i < MAX_SWITCH_DEV_NUM; i++) {
    if (switch_lddreg[i]) {
      cellUsbdUnregisterExtraLdd(&swpro_ops);
      switch_lddreg[i] = 0;
    }
  }
  for (i = 0; i < extra_count; i++) {
    if (extra_lddreg[i]) {
      if (extra_info[i].xtype == XTYPE_XBOX360W) {
        cellUsbdUnregisterExtraLdd(&xpadw_ops);
      } else {
        cellUsbdUnregisterExtraLdd(&xpad_ops);
      }
      extra_lddreg[i] = 0;
    }
  }
}

// number of times to retry registration when USBD is not ready yet
// (CELL_USBD_ERROR_NOT_INITIALIZED, 0x80110001), and the delay between
// tries. The pre-init boot sleep usually makes USBD ready by the first try;
// the retry is a safety net for slow/cold boots where a single fixed delay
// is not enough (up to ~30 s of extra grace here). Retry passes only touch
// entries that have not registered yet.
#define USB_INIT_RETRIES 10
#define USB_INIT_RETRY_DELAY_SEC 3

static int32_t init_usb(void) {
  int32_t r, attempt;
  sys_mutex_attribute_t mutex_attr;

  sys_mutex_attribute_initialize(mutex_attr);
  if ((r = sys_mutex_create(&xpad_mutex, &mutex_attr)) != CELL_OK) {
    xlog_code("init_usb: mutex create failed", r);
    return(r);
  }

  // initialize all controller handlers
  memset(handle, -1, sizeof(int32_t) * CELL_PAD_MAX_PORT_NUM);

  // pick up user-supplied VID/PIDs before registering anything
  load_extra_devices();

  memset(&XPAD, 0, sizeof(XPAD));

  // Register with USBD. If the subsystem is still coming up at boot the
  // call returns CELL_USBD_ERROR_NOT_INITIALIZED (0x80110001); wait and
  // retry rather than depend on the pre-init delay being exactly long
  // enough. Any other error is a real failure and aborts. Note attach
  // callbacks for connected controllers start firing mid-pass, as soon as
  // their own VID/PID is registered.
  for (attempt = 0; ; attempt++) {
    r = register_devices();
    if (r != CELL_USBD_ERROR_NOT_INITIALIZED || attempt >= USB_INIT_RETRIES) {
      break;
    }
    xlog_code("init_usb: USBD not ready, retrying in 3s", r);
    sys_timer_sleep(USB_INIT_RETRY_DELAY_SEC);
  }
  if (r != CELL_OK) {
    // roll back: earlier entries may have registered - and a controller may
    // already have attached through one of them - before the failing entry
    xlog_code("init_usb: registration aborted", r);
    xpad_detach_all();
    xpadw_detach_all();
    swpro_detach_all();
    unregister_devices();
    sys_mutex_destroy(xpad_mutex);
    return(r);
  }
  if (reg_conflicts > 0) {
    xlog_code("init_usb: device IDs skipped (already registered elsewhere)", reg_conflicts);
  }
  if (reg_skipped_full > 0) {
    // not an error: the cellUsbd LDD table ran out of room. Everything up to
    // that point registered and works; only the low-priority tail was dropped
    xlog_code("init_usb: LDD table full, low-priority device IDs skipped", reg_skipped_full);
  }

  // best-effort: a failure here only means rumble is unavailable, so it must
  // not abort USB init (input still works)
  create_rumble_queue();
  return(CELL_OK);
}

static int32_t shutdown_usb(void) {
  int32_t r;

  destroy_rumble_queue();

  // unregister exactly the registrations register_devices() made so
  // reloading the plugin neither leaks LDD registrations nor unregisters
  // entries that were never registered (e.g. skipped conflicts)
  unregister_devices();
  if ((r = sys_mutex_destroy(xpad_mutex)) != CELL_OK) {
    return(r);
  }
  return(CELL_OK);
}

static int xpadd_thread(uint64_t arg) {
  unsigned char xpad_data[MAX_XPAD_DATA_LEN];
  int32_t i, r;
  XPAD_UNIT_t *unit;
#if XPAD_AUTO_INJECT
  uint64_t t0;
#endif

  xlog_init();

  // wait for the system to finish booting: when this plugin is loaded at
  // startup (via boot_plugins.txt), the USBD subsystem and VSH notification
  // service may not be ready yet. Without this delay, cellUsbdRegisterExtraLdd
  // can return CELL_USBD_ERROR_NOT_INITIALIZED (0x80110001). 10 seconds is
  // the established safe margin used by other webMAN-era VSH plugins; it also
  // lets the VSH notification service come up so the show_msg() calls below
  // are visible. This delay alone is not a guarantee, though (a cold/slow
  // boot can need longer), so init_usb() additionally retries registration
  // while USBD reports not-initialized.
  xlog("plugin thread started, waiting 10s for boot to settle");
  sys_timer_sleep(10);

  xlog("starting USB init");
  r = init_usb();

  if (r != CELL_OK) {
    // do not fail silently: without this the user just sees a plugin
    // that does nothing
    char msg[96], *m;
    m = msg;
    m = append_str(m, "XPAD: USB init failed (0x");
    m = append_hex32(m, (uint32_t)r);
    if (reg_fail_vid != 0 || reg_fail_pid != 0) {
      m = append_str(m, " at ");
      m = append_hex16(m, reg_fail_vid);
      m = append_str(m, ":");
      m = append_hex16(m, reg_fail_pid);
    }
    m = append_str(m, "), plugin inactive");
    show_msg(msg);
    sys_ppu_thread_exit(0);
  }
  show_msg((char *)"XPAD Loaded!");
  xlog_code("USB init OK, entering poll loop; conflicts skipped", reg_conflicts);
  if (extra_count > 0 || extra_skipped > 0) {
    char msg[96], *m;
    m = msg;
    m = append_str(m, "XPAD: ");
    m = append_int(m, extra_count);
    m = append_str(m, " custom device(s) loaded");
    if (extra_skipped > 0) {
      m = append_str(m, ", ");
      m = append_int(m, extra_skipped);
      m = append_str(m, " line(s) skipped");
    }
    show_msg(msg);
  }
  running = 1;
#if XPAD_AUTO_INJECT
  // Auto-injection is opt-in (the LOADER-WEDGE FIX note above): a hung
  // LOAD_PROC_MODULE wedges the lv2 loader and freezes every subsequent
  // launch console-wide, so unless the user has explicitly created the opt-in
  // file the plugin must not issue PS3MAPI calls at all. When opted in, the
  // work still runs on its own expendable thread so a hang at least cannot
  // stall the input pump below (the INPUT-DEATH FIX note above). Low
  // priority: injection is a background convenience, input is not. If the
  // thread cannot be created, run without auto-injection rather than risk it.
  if (!auto_inject_opted_in()) {
    xlog("auto-inject: disabled (no /dev_hdd0/plugins/xpad_auto_inject.txt); use webMAN Game Plugins for rumble");
  } else if (sys_ppu_thread_create(&inject_thread_id, inject_thread, NULL, 1000, 0x2000,
                                   SYS_PPU_THREAD_CREATE_JOINABLE, INJECT_THREAD_NAME) != CELL_OK) {
    inject_thread_id = (sys_ppu_thread_t)-1;
    xlog("auto-inject: thread create failed, auto-injection disabled");
  } else {
    xlog("auto-inject: enabled (opt-in file present)");
  }
#endif
  while (running) {
    sys_timer_usleep(1000 * 10);
    btd_poll_tick();
    block(xpad_mutex);
    for (i = 0; i < MAX_UNIT_NUM; i++) {
      if ((unit = XPAD.units[i]) != NULL) {
        if (unit->xtype == XTYPE_SWITCH_PRO) {
            process_swpro_init(unit);
            if (unit->is_bt && unit->swpro_state == SWPRO_STATE_DONE) {
              if (++unit->swpro_keepalive_timer >= 100) { /* Keepalive every 1.0 second */
                unit->swpro_keepalive_timer = 0;
                swpro_set_rumble(unit, 0, 0);
              }
            }
        }
        unit->read_input(unit, xpad_data);
      }
    }
    check_pad_status();
    unblock(xpad_mutex);

    // forward rumble events from the game-process plugin to the USB pads
    // (route_rumble() takes xpad_mutex itself, so drain outside the lock)
    drain_rumble_queue();

#if XPAD_AUTO_INJECT
    // watchdog the injection thread: if a load_module call has been in flight
    // far longer than any healthy call takes, it is hung in the kernel. Input
    // (this thread) is unaffected; log the breadcrumb once so the debug log
    // explains why rumble auto-setup went missing this session.
    t0 = inject_call_t0;
    if (!inject_hang_logged && t0 != 0 && time_now_usec() - t0 > XPAD_INJECT_HANG_USEC) {
      inject_hang_logged = 1;
      xlog("auto-inject: load_module stuck >15s (kernel hang); input unaffected, rumble auto-inject lost this session");
    }
#endif
  }

  // exiting...
  xpad_detach_all();
  xpadw_detach_all();
  swpro_detach_all();
  btd_detach_all();
  shutdown_usb();
  sys_ppu_thread_exit(0);
  return(0);
}

int xpadd_start(uint64_t arg) {
  sys_ppu_thread_create(&thread_id, xpadd_thread, NULL, -0x1d8, 0x2000, SYS_PPU_THREAD_CREATE_JOINABLE, THREAD_NAME);
  _sys_ppu_thread_exit(0);
  return(SYS_PRX_RESIDENT);
}

static void xpadd_stop_thread(uint64_t arg) {
  uint64_t exit_code;
#if XPAD_AUTO_INJECT
  int32_t i;
#endif

  running = 0;
  sys_timer_usleep(500000);
#if XPAD_AUTO_INJECT
  if (inject_thread_id != (sys_ppu_thread_t)-1) {
    // the injection thread sees running == 0 on its next 2 s wake and exits.
    // Give an in-flight load_module call a bounded window to return first: if
    // it is hung in the kernel the thread can never exit, and joining it would
    // turn a lost-rumble session into a plugin unload that hangs forever.
    for (i = 0; i < 50 && inject_call_t0 != 0; i++) {
      sys_timer_usleep(100000);
    }
    if (inject_call_t0 == 0) {
      sys_ppu_thread_join(inject_thread_id, &exit_code);
    }
    inject_thread_id = (sys_ppu_thread_t)-1;
  }
#endif
  if (thread_id != (sys_ppu_thread_t)-1) {
    sys_ppu_thread_join(thread_id, &exit_code);
  }
  show_msg("XPAD Unloaded!");
  sys_ppu_thread_exit(0);
}

int xpadd_stop(void) {
  sys_ppu_thread_t t;
  uint64_t exit_code;

  sys_ppu_thread_create(&t, xpadd_stop_thread, 0, 0, 0x2000, SYS_PPU_THREAD_CREATE_JOINABLE, STOP_THREAD_NAME);
  sys_ppu_thread_join(t, &exit_code);
  _sys_ppu_thread_exit(0);
  return(SYS_PRX_STOP_OK);
}
