# xpad_game.sprx — game-process rumble sender

This is the second half of ps3xpad's rumble support. It is **not** loaded at
boot like the VSH driver; it is loaded **into a running game's process**, where
it hooks `cellPadSetActDirect()` and forwards the actuator (rumble) values the
game requests to the VSH driver (`xpad.sprx`) over the shared lv2 event queue
defined in [`../src/xpad_rumble.h`](../src/xpad_rumble.h). The driver then
issues the USB rumble command to the xpad controller on that port.

Rumble needs both plugins running: `xpad.sprx` in VSH (owns the USB pads and
the event queue) and `xpad_game.sprx` in the game (sees the actuator values).

## Loading it into a game

- **Manual (recommended):** use webMAN MOD's PS3MAPI **"Game Plugins"** page to
  load `/dev_hdd0/plugins/xpad_game.sprx` into the running game yourself. Also
  the fallback for firmware where PS3MAPI `LOAD_PROC_MODULE` is unavailable or
  partial (e.g. some PS3HEN setups).
- **Automatic (opt-in, off by default):** the VSH driver can inject it via
  PS3MAPI `LOAD_PROC_MODULE`. Enable it by creating an (empty) file
  `/dev_hdd0/plugins/xpad_auto_inject.txt`; without that file the driver makes
  no PS3MAPI calls at all. When enabled, a dedicated low-priority thread scans
  the PS3MAPI process list every ~2 s, skips VSH/system processes, and loads
  `/dev_hdd0/plugins/xpad_game.sprx` once into each game process — but only
  **after that process has stayed alive across several polls** (a settle
  delay, ~12 s), because injecting into a still-booting game hangs the kernel
  prx loader. **Why it is opt-in:** on some CFWs `LOAD_PROC_MODULE` hangs in
  the kernel even against a settled process, and the wedged lv2 loader then
  blocks every subsequent launch — the console freezes on starting anything
  until reboot. Only enable it if manual injection already works reliably on
  your CFW. Needs the file present in `/dev_hdd0/plugins/` and a CFW with
  PS3MAPI (Cobra/Mamba; webMAN MOD). The opcode/argument ABI is verified
  against the Cobra/Mamba kernel source (`aldostools/Mamba`
  `stage2/ps3mapi_core.{h,c}`) and the ArtemisPS3 userland reference — see
  TODO.md section 8.

## Building

Same toolchain as the VSH plugin (Sony Cell SDK + scetool). From this
directory:

```
cd game
make
```

produces `xpad_game.prx` and the signed `xpad_game.sprx`. Copy
`xpad_game.sprx` to `/dev_hdd0/plugins/`.

## Status

The IPC sender, actuator mapping and module lifecycle are complete, and the
PS3MAPI auto-injection opcode ABI in the VSH driver has been verified against
the Cobra/Mamba kernel source. The `cellPadSetActDirect` detour is implemented
but, like the USB rumble packets, can only be validated on real hardware; see
TODO.md section 8.
