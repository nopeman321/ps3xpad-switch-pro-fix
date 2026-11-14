# ps3xpad — Nintendo Switch Pro Controller support for PS3

`xpad.sprx` is a VSH plugin for PS3 (CFW with PS3HEN / webMAN MOD) that registers USB
gamepads with the system through the `cellUsbd` LDD interface, so games and the XMB see
them as first-party pads.

This fork adds a complete, non-blocking driver for the **Nintendo Switch Pro Controller
(USB VID:PID `057E:2009`)** — including third-party pads that speak the same protocol,
such as the 8BitDo Ultimate C in Switch mode — and remaps its face buttons to the
**PlayStation layout**.

---

## Why this fork exists

The stock `xpad` driver only handled Xbox 360 / XInput pads. The initial Switch Pro
support in this tree had three defects that made the pad unusable:

1. **Blocking init.** The init sequence ran inside the 10 ms poll thread with
   `sys_timer_usleep()`, stalling the USB input pump and deadlocking the endpoint.
   The pad never finished its handshake and stopped sending reports.
2. **Premature subcommands.** `register_ldd_controller()` and the LED refresh path
   pushed subcommands (`0x01`) onto the OUT pipe *before* the `0x80` handshake and the
   mode switch had completed. Those writes failed with `-1` and left `out_busy` stuck
   at `1`, which blocked every later OUT transfer.
3. **Missing cleanup on unload.** `swpro_detach_all()` was never called from
   `xpadd_thread()` or the `init_usb()` rollback. After an unload the kernel kept
   interrupt transfers pointing at freed memory, which crashed the console when an ISO
   was mounted or when the plugin was reloaded.

All three are fixed here.

---

## Features

| Feature | Status |
|---|---|
| USB HID enumeration (`057E:2009`, 8BitDo Switch mode) | ✅ |
| Asynchronous init state machine (`0x80` → `0x03` → IMU → vibration → LED) | ✅ |
| Full report `0x30` / subcommand reply `0x21` decoding | ✅ |
| Simple HID report `0x3F` fallback | ✅ |
| 12-bit little-endian analog sticks → 8-bit PS3 axes | ✅ |
| PlayStation face-button layout | ✅ |
| Rumble (`0x10`) and player LED (`0x30`) | ✅ |
| Clean detach, no leaked pipes on unload | ✅ |

---

## Installation

1. Copy `xpad.sprx` to `/dev_hdd0/plugins/` on your PS3.
2. Add this line to `/dev_hdd0/boot_plugins.txt`:

   ```
   /dev_hdd0/plugins/xpad.sprx
   ```

3. Reboot the console (or hot-reload via webMAN / PS3MAPI).

> **Do not** load `xpad_game.sprx` from `boot_plugins.txt`. It is a game-process
> plugin for rumble forwarding and must be injected per-game (webMAN → *Game Plugins*),
> not loaded globally in VSH. Having it in `boot_plugins.txt` is what caused the
> crashes when mounting ISO images.

### Button layout

The Pro Controller's physical layout is rotated relative to PlayStation's, so the
driver maps by **position**, not by Nintendo's letter:

| Physical position | Nintendo label | PS3 button |
|---|---|---|
| Left    | Y | **Square** |
| Top     | X | **Triangle** |
| Bottom  | B | **Cross** |
| Right   | A | **Circle** |
| —       | Home | **PS** |

This matches what a player expects when holding the pad: the bottom face button is
Cross, the right one is Circle.

---

## Protocol notes

### USB handshake (`0x80` commands)

Sent on the interrupt OUT endpoint, in this order, each gated on the previous one
returning `CELL_OK`:

| Command | Meaning |
|---|---|
| `0x80 0x01` | Request MAC / status |
| `0x80 0x02` | Handshake 1 |
| `0x80 0x03` | Switch to 3 Mbit/s baudrate |
| `0x80 0x02` | Handshake 2 (required after the baud change) |
| `0x80 0x04` | Force HID — disables the internal USB timeout so the pad does not revert to Bluetooth |

`0x80 0x04` is mandatory: without it the Pro Controller times out and drops the USB
session after a few seconds.

### Subcommands (report `0x01`)

Full 16-byte output report:

```
[0]      0x01            report ID
[1]      counter & 0x0F  wraps 0x00..0x0F, echoed by the pad
[2..9]   rumble[8]       neutral = 00 01 40 40 00 01 40 40
[10]     subcommand
[11..]   arguments
```

Subcommands used during init: `0x03` (input mode `0x30` full), `0x40` (enable IMU),
`0x48` (enable vibration), `0x30` (player LEDs).

See [SWITCH_PRO_PROTOCOL.md](SWITCH_PRO_PROTOCOL.md) for the full reference.

### Input reports

**`0x30` / `0x21` — full report** (49 bytes on the wire):

| Byte(s) | Field |
|---|---|
| 3 | Right cluster: `Y=0x01 X=0x02 B=0x04 A=0x08 R=0x40 ZR=0x80` |
| 4 | Shared: `MINUS=0x01 PLUS=0x02 RSTICK=0x04 LSTICK=0x08 HOME=0x10 CAPTURE=0x20` |
| 5 | Left cluster: `DOWN=0x01 UP=0x02 RIGHT=0x04 LEFT=0x08 L=0x40 ZL=0x80` |
| 6..8 | Left stick, 12-bit LE |
| 9..11 | Right stick, 12-bit LE |

```c
lx =  buf[6]        | ((buf[7] & 0x0F) << 8);
ly = (buf[7] >> 4)  |  (buf[8]        << 4);
rx =  buf[9]        | ((buf[10] & 0x0F) << 8);
ry = (buf[10] >> 4) |  (buf[11]       << 4);
```

12-bit raw values are scaled to the PS3's 8-bit range with a ±140 deadzone around
the 2048 centre and ±1400 usable travel, and the Y axis is inverted:

```c
static uint8_t swpro_scale_axis_12bit(uint16_t raw, int invert) {
  int32_t d = (int32_t)raw - 2048;
  int32_t v;
  if (d > -140 && d < 140) return 0x80;          // deadzone
  v = 128 + ((d * 127) / 1400);
  if (v < 0)   v = 0;
  if (v > 255) v = 255;
  return invert ? (uint8_t)(255 - v) : (uint8_t)v;
}
```

**`0x3F` — simple HID** (12 bytes): buttons in bytes 1–2, HAT switch in byte 3,
four 16-bit LE axes in bytes 4–11. Decoded as a fallback for pads that never leave
simple-HID mode.

> **Endianness note.** All multi-byte fields are unpacked byte-by-byte with explicit
> shifts. Never cast the USB buffer to a packed struct on the PPU — it is
> big-endian, and a direct struct read silently returns byte-swapped values.

---

## Building

Requires the Sony Cell SDK (PPU toolchain) and `scetool`.

```sh
CELL_SDK=C:/usr/local/cell sh tools/build_windows.sh
```

`tools/build_windows.sh` compiles both plugins, then runs `scetool` from its own
directory (scetool resolves its `data/` key directory relative to the *current working
directory*, so invoking it from `src/` produces an unsigned SELF).

Verify the result:

```sh
scetool -i src/xpad.sprx
```

You want `Category [SELF]`, `Encrypted [YES]` on all segments, and
`Signature status: OK`.

### Layout

| Path | Purpose |
|---|---|
| `src/main.c` | VSH driver: LDD registration, USB pumps, report decoding |
| `src/libc.c` | Minimal libc shims (`memcpy`, `strlen`, …) for the PRX environment |
| `src/ControlStruct.h`, `src/xpad_rumble.h` | Controller tables and rumble encoding |
| `game/xpad_game.c` | Game-process plugin: forwards rumble via an lv2 event queue |
| `tools/build_windows.sh` | Build + sign script |

---

## Debugging

The driver writes a timestamped log to `/dev_hdd0/plugins/xpad.log`
(rotated to `xpad_1.log` on each boot). Useful markers:

```
Switch USB 0x80 cmd sent, result (code 0x00010000)
FIRST INPUT REPORT (16 bytes) for 057E:2009
81 01 00 03 28 E1 F7 D8 17 E4 00 00 00 00 00 00
Switch init: SET_MODE (0x30) sent
Switch init: ENABLE_IMU sent
Switch init: ENABLE_VIB sent
Switch init: DONE, player LED set to (code 0x00000001)
```

A healthy init reaches `DONE` within ~1.5 s of attach. If it stops at
`SET_MODE`, the OUT pipe is still busy from an earlier write.

---

## Credits

Built on [Osirix/ps3xpad](https://github.com/Osirix/ps3xpad). Protocol details
cross-checked against
[dekuNukem/Nintendo_Switch_Reverse_Engineering](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering)
and BetterJoy.

## License

GPLv3 — see the original project for the full text.