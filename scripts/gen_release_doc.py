#!/usr/bin/env python3
"""
Generate build/README.txt with Git commit hash, build date, and tool documentation.
"""

import subprocess
import datetime
import os
import sys

def get_git_info():
    try:
        h = subprocess.check_output(["git", "rev-parse", "--short=8", "HEAD"], stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        h = "unknown"
    try:
        status = subprocess.check_output(["git", "status", "--porcelain"], stderr=subprocess.DEVNULL).decode().strip()
        dirty = "-dirty" if len(status) > 0 else ""
    except Exception:
        dirty = ""
    date_str = datetime.datetime.now().strftime("%Y-%m-%d")
    return f"{h}{dirty}", date_str

def main():
    repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    build_dir = os.path.join(repo_dir, "build")
    os.makedirs(build_dir, exist_ok=True)
    readme_path = os.path.join(build_dir, "README.txt")

    git_hash, date_str = get_git_info()

    doc = f"""===============================================================================
       PiStorm32-lite 200.00 MHz Gateware Release Package
===============================================================================
Firmware Version : Commit {git_hash}
Build Date       : {date_str}
Target Hardware  : Commodore Amiga 1200 + PiStorm32-lite (Efinix Trion T20)
PLL Clock Rate   : 14x Multiplier (~198.63 MHz PAL / ~200.45 MHz NTSC)
Sampling Period  : 5.000 ns Resolution
===============================================================================

1. PACKAGE CONTENTS
-------------------
  * firmware.bin.gz        : Gzip-compressed FPGA bitstream for PiStorm32-lite.
  * efinix_firmware_ps32.h : C-header include file containing the bitstream
                             byte array for direct compilation into Emu68.
  * firmware_ps32.h        : Symlink/alias copy of efinix_firmware_ps32.h.
  * PS32Scope              : Live hardware bus scope & logic analyzer GUI.
  * PS32_BusCtrl           : CLI bus control & hardware telemetry tool.
  * PS32_Turbo_ON          : Double-clickable shortcut: Enables Ultra Turbo mode.
  * PS32_Turbo_OFF         : Double-clickable shortcut: Disables accelerations.
  * PS32_NoFastDSACK       : Double-clickable shortcut: Enables Mediator-safe mode.
  * PS32_StressTest        : 6-stage hardware quality & stress test suite.
  * BenchSRAM              : High-precision memory and PCMCIA benchmark tool.
  * README.txt             : This documentation file.


2. FIRMWARE INSTALLATION (EMU68)
--------------------------------
Method A: SD-Card Boot Partition (Recommended)
  1. Mount the FAT32 boot partition of your Emu68 SD card (e.g. 'emu68boot:').
  2. Copy 'firmware.bin.gz' into the root of the boot partition.
  3. Ensure your 'config.txt' contains the initramfs directive, for example:
       initramfs firmware.bin.gz,A1200.47.96.rom
  4. Perform a cold reboot of the Amiga. Emu68 will automatically load the
     new bitstream into the FPGA at boot.

Method B: Compile into Emu68
  1. Copy 'efinix_firmware_ps32.h' into 'Emu68/src/pistorm/efinix_firmware_ps32.h'.
  2. Rebuild the Emu68 kernel.


3. AMIGA TOOLS & UTILITIES GUIDE
--------------------------------

[PS32Scope]
  Real-time oscilloscope displaying Amiga 1200 bus signals (/AS, /DS, /DSACK,
  R/W, and DATA bus multiplexing) captured by the FPGA at 200 MHz.
  
  Features:
  - Motherboard Crystal Calibration: Accurately detects and displays PAL
    (14.19 MHz) vs NTSC (14.32 MHz) via AmigaOS timer.device E-Clock.
  - Sub-nanosecond phase alignment and jitter-free display.
  - Live display of firmware Git commit hash and build date.

  Usage:
    Run 'PS32Scope' from Shell or double-click its icon.
    Keys:
      [SPC]   Freeze / unfreeze live waveform display
      [Z]     Cycle zoom (AUTO fit cycle, 450ns, 800ns, 1400ns)
      [W]     Lock trigger to WRITE cycles
      [R]     Lock trigger to READ cycles
      [1]     Switch FPGA to Ultra Turbo mode (0x87)
      [2]     Switch FPGA to Safe / No Fast DSACK mode (0x81)
      [3]     Switch FPGA to Stock reference mode (0x00)
      [Q]     Quit program


[PS32_BusCtrl]
  Command-line tool to query telemetry or dynamically adjust bus timing modes.

  Usage:
    PS32_BusCtrl status        Display live FPGA settings and firmware version
    PS32_BusCtrl on            Enable Ultra Turbo mode (0x87: FastDSACK+Prefetch+CCK)
    PS32_BusCtrl nofastdsack   Disable Fast DSACK (0x81: Mediator PCI bridge safe)
    PS32_BusCtrl off           Stock timing (0x00: Golden reference parity)
    PS32_BusCtrl <hex_val>     Apply custom 8-bit BUS_CTRL register value


[PS32_Turbo_ON / PS32_Turbo_OFF / PS32_NoFastDSACK]
  Pre-configured executables for Workbench users. Simply double-click in
  Workbench to immediately apply the corresponding bus mode without Shell.


[PS32_StressTest]
  Exhaustive hardware validation suite testing 6 key areas:
    1. Virtual Zorro Wishbone register stress (1,000,000 consecutive R/W ops)
    2. Motherboard Chip RAM pattern sweep (Solid 0/1, Checkerboard, PRNG)
    3. PCMCIA SRAM Gayle bus integrity
    4. PiStorm Fast RAM 32-bit ARM LPDDR stress
    5. FPGA bus control mode stability sweep (all 5 modes)
    6. Filesystem block I/O & checksum integrity
  Usage:
    Run 'PS32_StressTest' from Shell. Exits with return code 0 on pass.


[BenchSRAM]
  High-precision assembly kernel benchmarking tool for PCMCIA SRAM and RAM.
  Usage:
    BenchSRAM                  Benchmark default PCMCIA SRAM at $00610000
    BenchSRAM <addr> <size_kb> Benchmark arbitrary memory range


4. FPGA HARDWARE OPERATING MODES
--------------------------------
* Ultra Turbo (0x87):
    Enables speculative 32-bit & 16-bit prefetch, Fast DSACK (-88ns dead time),
    and 7.09 MHz CCK phase alignment. Delivers maximum Chip RAM speed
    (up to 5.0 MB/s read, 7.0 MB/s write). Default mode.

* Mediator Safe (0x81):
    Keeps speculative prefetch active but leaves standard Motorola S4 hold
    times intact. Recommended if using Elbox Mediator PCI bus boards.

* Stock Golden Parity (0x00):
    Disables all accelerations. Operates with 100% stock 68020 bus timing.
===============================================================================
"""

    with open(readme_path, "w") as f:
        f.write(doc)

    print(f"[gen_release_doc] Generated {readme_path} (commit: {git_hash}, date: {date_str})")

if __name__ == "__main__":
    main()
