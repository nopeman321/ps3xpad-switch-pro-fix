# Switch Pro — USB Protocol Reference

Source: `BetterJoy/BetterJoyForCemu/Joycon.cs` (current `master`).

## 1. MAC / Status Handshake

| # | Byte | Meaning |
|---|------|---------|
| 1 | `0x80` | Sync byte (always the first byte of every command/ack) |
| 2 | `0x01` | MAC/status — tells the controller to send its MAC address |

Controller replies with MAC (6 bytes) + status. BetterJoy then sends:

| # | Byte | Meaning |
|---|------|---------|
| 1 | `0x80` | Sync |
| 2 | `0x02` | Handshake 1 — acknowledge receipt of MAC |
| 1 | `0x80` | Sync |
| 2 | `0x03` | Baudrate 3 Mbit — set 0x40 in the 3 Mbit register (byte 2) |
| 1 | `0x80` | Sync |
| 2 | `0x02` | Handshake 2 — acknowledge baud change |

## 2. Disable / Enable USB Timeout

After the handshake sequence:

* **Disable timeout / Force HID** → `{ 0x80, 0x04 }` (byte 0x04 tells the controller to enter Force HID mode and disables the internal USB timeout).
* **Enable timeout on disconnect** → `{ 0x80, 0x05 }` sent when the pad is disconnected.

## 3. Output Report 0x01 — Subcommand

Full 16-byte report (byte 0 = report ID):

| Byte(s) | Field | Notes |
|---------|-------|-------|
| 0 | `0x01` | Report ID for subcommand |
| 1 | `global_count & 0x0F` | Wraps 0x00…0x0F after each send (`if (global_count == 0xf) global_count = 0; else ++global_count;`) |
| 2–9 | Rumble bytes | **Neutral**: `00 01 40 40 00 01 40 40` (default_buf in Joycon.cs). The rumble data is shared across subcommands — it stays at neutral until a specific rumble subcommand is sent. |
| 10 | Subcommand code | See table below |
| 11–15+ | Arguments | Optional, depending on subcommand |

### Key Subcommands

| Code (byte 10) | Purpose | Args (bytes 11+) |
|----------------|---------|-------------------|
| `0x10` | SPI read | 2 bytes: command (0x80 or 0x60) + address offset; response returned in next input report |
| `0x03` | Set Input Mode 0x30 (Full) | 0x30 |
| `0x40` | Enable IMU (gyro/accel) | 0x01 |
| `0x48` | Enable Vibration | 0x01 |
| `0x30` | Set Player LEDs | up to 5 bytes for LED pattern |

### Initialization Sequence (third-party controllers)

```
Send { 0x01, cnt, rumble[8], 0x10, cmd, addr }   // SPI read (cmd=0x80 or 0x60)
Send { 0x01, cnt, rumble[8], 0x03, 0x30 }         // Full Mode 60 Hz
Send { 0x01, cnt, rumble[8], 0x40, 0x01 }         // Enable IMU
Send { 0x01, cnt, rumble[8], 0x48, 0x01 }         // Enable Vibration
Send { 0x01, cnt, rumble[8], 0x30, leds[] }       // Set LEDs
```

> **SPI fallback (third-party / 8BitDo):** When `thirdParty == true`, `dump_calibration_data()` skips the SPI read entirely and uses hardcoded calibration values from `App.config` (`acc_sensiti`, `gyr_sensiti`, `stick_cal`, `stick2_cal`, `deadzone`, `deadzone2`). This avoids the fragile SPI timeout that causes initialization to fail on some third-party pads.

## 4. Output Report 0x10 — Rumble Only

| Byte(s) | Field | Notes |
|---------|-------|-------|
| 0 | `0x10` | Report ID for rumble-only |
| 1–16 | Rumble data | Two sets of 8 bytes (left motor, right motor, repeated) |

This is used when the controller only needs a rumble update without changing any mode or parameters.

## 5. Input Reports

### Report 0x30 — Full Mode + IMU (54 bytes)

| Byte(s) | Field |
|---------|-------|
| 0 | Report ID = `0x30` |
| 1–2 | Global counter (same as output) |
| 3–12 | Buttons (10 bytes, one byte per button) |
| 13–48 | IMU data — **3 frames of IMU**, each 12 bytes |
| 49–52 | Stick X/Y left & right |
| 53 | Trailing/padding byte |

**IMU layout per frame (12 bytes):**
- Bytes 0–5: Accelerometer X, Y, Z (signed 16-bit little-endian × 3)
- Bytes 6–11: Gyroscope X, Y, Z (signed 16-bit little-endian × 3)

The three frames are consecutive, so frame *n* starts at byte `13 + 12*n`
(13, 25, 37) and the sticks start at byte 49. That is what makes the total add
up: `1 + 2 + 10 + 3*12 + 4 + 1 = 54` bytes.

> **Correction:** an earlier revision of this document said each IMU frame was
> 16 bytes. That is inconsistent with the 54-byte report stated above —
> `13 + 3*16 = 61` already overruns it. Nintendo/BetterJoy frames carry exactly
> six int16 samples (3 accel + 3 gyro) with no temperature field, i.e. 12 bytes.
> The corrected offsets are the ones used by `src/main.c` and asserted in
> `tests/test_switch_pro.c`.

### Report 0x21 — Subcommand Reply + Input (54 bytes)

| Byte(s) | Field |
|---------|-------|
| 0 | Report ID = `0x21` |
| 1 | SPI data count (for SPI reads) |
| 2–10 | Response data (9 bytes for SPI payload) |
| 11 | Subcommand that was acknowledged |
| 12+ | Input report (same layout as 0x30) |

The code waits for `response[0] == 0x21 && response[11] == sc` (up to 10 retries
with 100 ms timeout).

> **Correction:** the reply carries the acknowledged subcommand code at byte 11,
> not byte 14, and the 9-byte SPI payload occupies bytes 2–10. Byte 14 lands
> inside the input portion, which is why matching on `response[14]` never
> confirms a subcommand. `src/main.c` matches on byte 11.

### Report 0x3F — Simple HID Mode (12 bytes)

| Byte(s) | Field |
|---------|-------|
| 0 | Report ID = `0x3F` |
| 1–11 | Button + stick data (simple, no IMU) |

This is the initial mode the controller enters on connection before the Full Mode (0x30) subcommand.

## 5b. Button Byte Layout

The 10 button bytes of the 0x30 report are **not** one byte per button — they
are three packed bitfields (bytes 3, 4 and 5) followed by seven unused bytes.
The three groups are laid out so that a whole group can be read as one bitmask.

### Byte 3 — right cluster

| Bit | Button | PS3 mapping |
|-----|--------|-------------|
| 0 | Y | Triangle |
| 1 | X | Square |
| 2 | B | Circle |
| 3 | A | Cross |
| 4 | SR (right) | — |
| 5 | SL (right) | — |
| 6 | R | R1 |
| 7 | ZR | R2 |

### Byte 4 — shared cluster

| Bit | Button | PS3 mapping |
|-----|--------|-------------|
| 0 | Minus | Select |
| 1 | Plus | Start |
| 2 | Right stick click | R3 |
| 3 | Left stick click | L3 |
| 4 | Home | — |
| 5 | Capture | — |

### Byte 5 — left cluster

| Bit | Button | PS3 mapping |
|-----|--------|-------------|
| 0 | Down | Down |
| 1 | Up | Up |
| 2 | Right | Right |
| 3 | Left | Left |
| 4 | SR (left) | — |
| 5 | SL (left) | — |
| 6 | L | L1 |
| 7 | ZL | L2 |

**Face-button mapping is diagonal.** On Nintendo hardware A sits where B is on
an Xbox pad, so A maps to Cross, B to Circle, X to Square and Y to Triangle.
Getting this wrong produces a pad whose face buttons are rotated 180°.

### Sticks (bytes 49–52)

Left X, left Y, right X, right Y — one byte each, centred on `0x08` with a
working range of roughly `0x00`–`0xFF`. PS3 pads use the same centred 8-bit
convention, so no scaling is required beyond clamping.

## 6. SPI Calibration — Key Addresses

| Address | Field | Size |
|---------|-------|------|
| `0x80 + offset` | User stick calibration (per-stick) | 9 bytes |
| `0x60 + offset` | Factory stick calibration | 9 bytes |
| `0x603D` / `0x6046` | Stick calibration (older notation) | — |
| `0x604E` / `0x6057` | IMU calibration | — |

Offset values depend on left/right and Joycon vs Pro Controller:

| Controller | Side | User SPI offset | Factory SPI offset |
|------------|------|-----------------|--------------------|
| Joycon | Left  | `0x12` | `0x3D` |
| Joycon | Right | `0x1D` | `0x46` |
| Pro (left stick) | Left  | `0x1D` | `0x3D` |
| Pro (right stick) | Right | `0x12` | `0x46` |

Third-party controllers skip the SPI reads and use the `App.config` defaults directly.

## 7. USB Pipe Addresses

| Direction | Endpoint Address | Description |
|-----------|-----------------|-------------|
| IN (pad → host) | `0x81` | Interrupt pipe for input reports |
| OUT (host → pad) | `0x01` or `0x02` | Interrupt pipe for output reports |

Both pipes must be opened for full Switch Pro functionality. The OUT pipe is required to send subcommands and rumble updates; the IN pipe receives the 54-byte Full Mode input reports.

## 8. Rumble Byte Reference

### Neutral (default)
```
00 01 40 40  00 01 40 40
```
Low byte = `0x00`, high byte = `0x01` for both motors (no vibration).

### Full rumble
```
FF 00 FF 00  FF 00 FF 00
```
Maximum rumble on both motors.

### Byte order in the 8-byte rumble field:
| Byte | Motor | Meaning |
|------|-------|---------|
| 0 | Left low | Low-frequency motor, left |
| 1 | Left high | High-frequency motor, left |
| 2 | Right low | Low-frequency motor, right |
| 3 | Right high | High-frequency motor, right |
| 4 | Left low (repeat) | — |
| 5 | Left high (repeat) | — |
| 6 | Right low (repeat) | — |
| 7 | Right high (repeat) | — |
