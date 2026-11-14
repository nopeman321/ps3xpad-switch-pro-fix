# PS3XPad Code Review

## Architecture Overview

ps3xpad is a two-part plugin for PS3 running webMAN/Mamba CFW. It bridges Xbox 360 / XInput USB controllers into the PS3's `cellPad` subsystem so games see them as native pads (with pressure buttons, PS button, etc).

| Component | Path | Role |
|---|---|---|
| **VSH driver** (`xpad.sprx`) | `src/main.c` (~2807 lines) | LDD plugin, registers USB controllers, owns rumble events, auto-injection |
| **Game plugin** (`xpad_game.sprx`) | `game/xpad_game.c` (~303 lines) | Loaded into game process, hooks `cellPadSetActDirect()`, sends rumble via IPC |

The two communicate through an lv2 named event queue (key `0x7870616472756d62ULL`) — the game plugin pushes actuator values, the VSH driver drains them and issues USB rumble to the controller.

---

## 1. VSH Driver (`src/main.c`)

### Strengths

- **Device table ordering is deliberate.** `xpad_info[]` is ordered by popularity (Xbox 360 → third-party) so the most-used controllers register before cellUsbd's fixed-size LDD table (~22 entries on retail) fills up. The comment at line 170 explicitly documents this.

- **Self-healing boot delay.** The plugin waits 10s for USBD to be ready (line 2669), and retries registration if `CELL_USBD_ERROR_NOT_INITIALIZED` is seen. This handles cold boots without user intervention.

- **Idempotent registration.** The `xpad_lddreg[]` arrays track which entries were already registered, preventing re-registration failures (0x80110006) on retry.

- **Detached USB device cleanup.** The `closing`/`pending` guards (line 324-329) in `XPAD_UNIT_t` prevent use-after-free when devices are unplugged mid-transfer.

- **Log rotation.** The three-file log rotation (`xpad.log`, `.1`, `.2`) means a frozen game-reboot cycle doesn't lose diagnostic data — a thoughtful touch for the most failure-prone scenario.

- **Auto-injection is opt-in and safe.** The `LOAD_PROC_MODULE` call can hang the lv2 loader on some CFWs, so the plugin only issues PS3MAPI calls when `xpad_auto_inject.txt` exists (line 129). A separate low-priority thread runs injection; even if it hangs, input processing continues.

### Issues

#### ISSUE 1 — Hardcoded 10s boot delay
Lines 2669–2670: `sys_timer_sleep(10)`. This is the "established safe margin" used by other webMAN-era plugins, but on a slow console (HDD failing, large HDD, cold boot), 10s may not be enough. The retry loop in `init_usb()` catches the case where registration itself fails before USBD is ready, but the initial 10s wait is a fixed floor — there's no way to skip it or configure it.

**Severity:** Low. Only affects cold boots on slow consoles.
**Fix:** Add a second file like `/dev_hdd0/plugins/xpad_fast_boot.txt` that sets `sys_timer_sleep(2)`.

#### ISSUE 2 — `write_xpad()` drops LED/rumble commands silently
Lines 1646–1674: If `out_busy` is true (a previous transfer still in flight), the function returns -1 without logging. This is correct for correctness but means a user could have rumble "stop working" on a controller without seeing any log entry or notification.

**Severity:** Low. The rate of LED/rumble commands is low enough that contention is rare.
**Fix:** Log a single "LED drop" per transfer failure burst (already done in lines 1668–1671 for submit failures, but the `out_busy` case at line 1653 falls through silently).

#### ISSUE 3 — `xpad_detach_all()` / `xpadw_detach_all()` are called twice during stop
Lines 2756–2758: `xpadd_stop_thread()` calls `xpad_detach_all()` and `xpadw_detach_all()`. But if the user unloads the plugin manually (e.g., from webMAN), `xpadd_stop()` → `xpadd_stop_thread()` does the same. Meanwhile, the poll thread's while loop may still be running when `running = 0` is set at line 2775 — the loop exits after one more 10ms sleep, then calls these functions. **But `xpad_detach_all()` iterates `XPAD.units[]` and calls `cellUsbdClosePipe()` on each unit; if a detach has already happened in a previous pass (e.g., during plugin unload), `cellUsbdClosePipe()` on a closed pipe is idempotent.** So this is safe, but worth noting.

**Severity:** Negligible. Idempotent by design.

#### ISSUE 4 — `inject_thread_t0` race during shutdown
Lines 2778–2789: The shutdown code checks `inject_call_t0 != 0` to decide whether the injection thread's in-flight call has returned. But `inject_call_t0` is written by the injection thread and read by the stop thread. There's no explicit memory barrier between them — though on PS3 (PowerPC), loads are not reordered with respect to other loads, so this should be fine.

**Severity:** Low. Correct on PowerPC; would need a fence if ported to x86.

#### ISSUE 5 — `xpadd_stop()` creates a new thread to join the main thread
Lines 2799–2807: `xpadd_stop()` spawns `xpadd_stop_thread` which sets `running = 0`, sleeps 500ms, then joins the main thread. This is a bit of a dance — it could just call `xpadd_stop_thread()` directly from the stopping PRX.

**Severity:** Cosmetic. Works correctly; the extra thread adds ~500ms to unload time.

### Technical Details

- **PID tracking with settle-time:** `pending_pids[]` / `pending_age[]` (lines 140–141) track unregistered processes. Auto-injection only happens after `XPAD_INJECT_SETTLE_POLLS` (6 polls ≈ 12s), preventing injection of still-booting games.

- **Rumble wire format** (`xpad_rumble.h`): `data1=port(0..6)`, `data2=left motor(0..255)`, `data3=right motor(0 or 255)`. Simple, efficient, no wasted bytes.

- **Auto-inject watchdog:** `inject_call_t0` + `XPAD_INJECT_HANG_USEC` (line 2748–2752). If a `load_module` call takes >15s, the poll thread logs a breadcrumb without blocking input.

---

## 2. Game Plugin (`game/xpad_game.c`)

### Strengths

- **Detour-based hook of `cellPadSetActDirect()`.** Lines 86–104: The plugin saves the original function pointer and installs a trampoline that calls into the rumble sender. Clean, no patching of the game's code.

- **Simple IPC protocol.** One event queue, one push per actuator call. No shared memory, no locks (the kernel handles it).

- **Module lifecycle awareness.** `unload_module()` properly uninstalls the hook and closes the event port.

### Issues

#### ISSUE 6 — Single-point-of-failure: game plugin only
Lines 8–11 of `game/readme.md`: "Rumble needs both plugins running." If the game-process plugin fails to load (PS3MAPI hang, memory pressure), rumble stops working until the user manually reloads. There's no fallback in the VSH driver.

**Severity:** Medium. The most common failure mode: a single bad `LOAD_PROC_MODULE` call wedges rumble for the session.
**Fix:** Add a "manual rumble" mode where the game plugin's event queue is polled from the VSH poll thread (already partially done — `drain_rumble_queue()` at line 2740).

#### ISSUE 7 — `xpad_game.c` doesn't handle multiple controllers
Line 86: The hook saves `act_direct_orig`, a single function pointer. If two controllers are active, both calls go through the same detour. The rumble sender (`rumble_send`) uses `data1=port` to route, so this is correct — but if a game uses `cellPadGetData()` on multiple pads simultaneously and the hook doesn't preserve thread-local state, there's a potential race.

**Severity:** Low. Only matters for games that read pads in parallel threads (rare).

#### ISSUE 8 — No error handling for event queue creation failure
Line 104: `sys_event_port_create()` returns an event port ID; if it fails, the plugin continues running with an invalid port ID. The subsequent `rumble_send()` calls will fail silently.

**Severity:** Low. Event port creation rarely fails, but a silent failure means rumble is just dead.

---

## 3. Build System (`src/Makefile`, `game/Makefile`)

### Strengths

- Both use the same GCC toolchain (`ppu-lv2-gcc`) and flags (`-mprx -mno-sn-ld -Os`).
- `-lio_stub` resolves `cellPadSetActDirect` in the game plugin.
- `scetool` signs both as APP SELF — consistent with PS3MAPI expectations.

### Issues

#### ISSUE 9 — No dependency on header changes
Neither Makefile lists `xpad_rumble.h` or `ControlStruct.h` as dependencies. If either header changes, the `.c` files won't be recompiled automatically.

**Severity:** Low. The headers are small and rarely change.
**Fix:** Add `PPU_PRX_SRCDIRS += $(CELL_TARGET_PATH)/ppu/include/cell/net` or a simple `xpad_rumble.h: xpad_game.c` rule.

---

## 4. BetterJoy Integration

### Strengths

- **BetterJoy's controller detection** is comprehensive (3rdPartyControllers.cs lists dozens of HID controllers).
- **BetterJoyForCemu/Joycon.cs** handles both wired and wireless controllers, including complex calibration data (lines 1193–1245).
- **3rdPartyControllers.cs** stores per-controller metadata in `Program.thirdPartyCons` and persists to file.

### Integration Points

1. **Controller list overlap:** ps3xpad's `xpad_info[]` (~30 entries) and BetterJoy's 3rd party list cover different devices — ps3xpad is wired XInput-focused, BetterJoy adds Bluetooth and wireless USB adapters. No overlap issues.

2. **Rumble protocol alignment:** Both use the same rumble wire format (left/right motor intensity), so a unified rumble pipeline is straightforward.

3. **PID matching:** ps3xpad's auto-inject PID tracking matches BetterJoy's controller enumeration by VID/PID — no conflicts.

### Issues in BetterJoy integration

#### ISSUE 10 — BetterJoy's stick calibration parsing is fragile
Lines 1199–1206: `stick_cal[isLeft ? 0 : 2] = (UInt16)((buf_[1] << 8) & 0xF00 | buf_[0])`. The `& 0xF00` mask is correct, but if `buf_` contains garbage (uninitialized SPI), the calibration values will be wrong. The `ReadSPI()` call (line 1211) doesn't validate return length before accessing `buf_[i]`.

**Severity:** Medium. Can cause stick drift or calibration reset on cold boot.
**Fix:** Add a bounds check on `ReadSPI()` return value.

#### ISSUE 11 — Trigger button mapping in Joycon.cs
Lines 1725–1732: The trigger left/right mapping uses `output.trigger_left_value = (byte)(buttons[(int)(isLeft ? Button.SHOULDER_2 : Button.SHOULDER1_2)] ? Byte.MaxValue : 0)`. This looks correct, but the `trigger_left` assignment on line 1731 (`output.trigger_left = output.trigger_left_value > 0 ? output.trigger_left = true`) uses a compound expression with side effects. It works in C# but is slightly unusual — could be clearer as a simple `if/else`.

**Severity:** Cosmetic. Works correctly.

---

## Summary of Findings

| # | Issue | Severity | Component |
|---|-------|----------|-----------|
| 1 | Hardcoded 10s boot delay | Low | VSH driver |
| 2 | Silent LED drop in write_xpad | Low | VSH driver |
| 3 | Double detach during stop | Negligible | VSH driver |
| 4 | inject_thread_t0 race | Low | VSH driver |
| 5 | Extra thread for stop | Cosmetic | VSH driver |
| **6** | **Single-point-of-failure rumble** | **Medium** | **Game plugin** |
| 7 | Single-hook multi-controller | Low | Game plugin |
| 8 | No event queue error handling | Low | Game plugin |
| 9 | Header dependency missing | Low | Build system |
| **10** | **BetterJoy SPI calibration fragility** | **Medium** | **BetterJoy** |
| 11 | Trigger button compound expression | Cosmetic | BetterJoy |

### Top Recommendations

1. **Rumble resilience (Medium):** Add a fallback rumble mode in the VSH driver that detects when the game plugin has gone away and can recover it automatically (or at least log a notification).

2. **BetterJoy SPI validation (Medium):** Add length checks to `ReadSPI()` before accessing `buf_[i]`.

3. **Boot delay config (Low):** Add an optional fast-boot file to skip the 10s wait on consoles with fast HDDs.

4. **Auto-inject watchdog improvement (Low):** Increase `XPAD_INJECT_HANG_USEC` from 15s to 20s for safety margin, since `LOAD_PROC_MODULE` can occasionally take longer under memory pressure.
