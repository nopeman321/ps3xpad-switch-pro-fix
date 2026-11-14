# ps3xpad-switch-pro-fix

A custom build of the ps3xpad plugin with critical stability fixes and native PlayStation-layout remapping for the Nintendo Switch Pro Controller on PS3.

## Improvements
- **PlayStation Layout Mapping**: Remaps the Switch Pro physical buttons to match the standard PS3 controller layout:
  - **Y (Left) -> Square**
  - **X (Top) -> Triangle**
  - **B (Bottom) -> Cross**
  - **A (Right) -> Circle**
- **USB Stability Fixes**: Implements robust `swpro_detach_all()` routines to prevent kernel hangs and system freezes during ISO mounting or plugin reloads.
- **Improved Initializer**: Synchronized state machine for controller handshake, mode setting, IMU, and rumble initialization to ensure a 100% success rate on boot.

## Installation
1. Compile the source using the provided `tools/build_windows.sh` script or download the pre-compiled `xpad.sprx`.
2. Connect to your PS3 via FTP.
3. Place `xpad.sprx` into `/dev_hdd0/plugins/`.
4. Edit your `/dev_hdd0/boot_plugins.txt` and add this line:
   `/dev_hdd0/plugins/xpad.sprx`
5. **Important**: Ensure `xpad_game.sprx` is NOT present in your `boot_plugins.txt` to avoid system conflicts.
6. Restart your PS3.

## Credits
Based on the original [ps3xpad](https://github.com/Osirix/ps3xpad) by Osirix.

## License
Licensed under GPLv3.
