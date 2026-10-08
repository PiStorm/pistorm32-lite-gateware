#!/usr/bin/env python3
import subprocess
import re
import sys
import time

AMIGA_IP = "192.168.178.33"

MODES = [
    ("Stock Golden Ref", "0x00", "All OFF (Stock 68020 timing)"),
    ("FastDSACK Only",   "0x02", "Fast DSACK ON, Prefetch OFF, CCK OFF"),
    ("CCK Sync Only",    "0x06", "Fast DSACK ON, CCK Sync ON, Prefetch OFF"),
    ("Mediator Safe",    "0x81", "Fast DSACK OFF, CCK OFF, Prefetch 16/32 ON"),
    ("Turbo (32b Pref)", "0x07", "Fast DSACK ON, CCK ON, Prefetch 32b ON"),
    ("Full Ultra Turbo", "0x87", "Fast DSACK ON, CCK ON, Prefetch 16b/32b ON"),
]

def run_cmd(cmd):
    p = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return p.stdout

results = {}

for name, val, desc in MODES:
    print(f"Setting mode {name} ({val})...", flush=True)
    run_cmd(f"squirt_exec {AMIGA_IP} 'Work:PS32_BusCtrl {val}'")
    time.sleep(0.5)

    print(f"Running bustest chip size=16k for {name}...", flush=True)
    out = run_cmd(f"squirt_exec {AMIGA_IP} 'bustest chip size=16k'")
    
    # Parse output:
    # chip      $00010000  readw     656.1 ns   normal       3.0 * 10^6 byte/s
    ops = {}
    for line in out.splitlines():
        line = line.strip()
        m = re.match(r"chip\s+\$[0-9A-Fa-f]+\s+([a-z]+)\s+([0-9.]+)\s+ns\s+\S+\s+([0-9.]+)\s+\*\s+10\^6\s+byte/s", line)
        if m:
            op = m.group(1)
            cyc = float(m.group(2))
            bw = float(m.group(3))
            ops[op] = (cyc, bw)
    results[name] = (val, desc, ops)
    time.sleep(0.5)

# Reset to full turbo at the end
run_cmd(f"squirt_exec {AMIGA_IP} 'Work:PS32_BusCtrl 0x87'")

print("\n" + "="*80)
print("RESULTS SUMMARY")
print("="*80)

# Markdown Table 1: Bandwidth (MB/s)
print("\n### Bandwidth Comparison (MB/s = 10^6 byte/s)\n")
print("| Mode | BUS_CTRL | Read Word (16-bit) | Read Long (32-bit) | Read Multiple | Write Word (16-bit) | Write Long (32-bit) | Write Multiple |")
print("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")

for name, (val, desc, ops) in results.items():
    rw = f"{ops.get('readw', (0,0))[1]:.2f}" if 'readw' in ops else "-"
    rl = f"{ops.get('readl', (0,0))[1]:.2f}" if 'readl' in ops else "-"
    rm = f"{ops.get('readm', (0,0))[1]:.2f}" if 'readm' in ops else "-"
    ww = f"{ops.get('writew', (0,0))[1]:.2f}" if 'writew' in ops else "-"
    wl = f"{ops.get('writel', (0,0))[1]:.2f}" if 'writel' in ops else "-"
    wm = f"{ops.get('writem', (0,0))[1]:.2f}" if 'writem' in ops else "-"
    print(f"| **{name}** | `{val}` | {rw} MB/s | {rl} MB/s | {rm} MB/s | {ww} MB/s | {wl} MB/s | {wm} MB/s |")

# Markdown Table 2: Cycle Latency (ns)
print("\n### Cycle Latency Comparison (Nanoseconds)\n")
print("| Mode | BUS_CTRL | Read Word | Read Long | Read Multiple | Write Word | Write Long | Write Multiple |")
print("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")

for name, (val, desc, ops) in results.items():
    rw = f"{ops.get('readw', (0,0))[0]:.1f} ns" if 'readw' in ops else "-"
    rl = f"{ops.get('readl', (0,0))[0]:.1f} ns" if 'readl' in ops else "-"
    rm = f"{ops.get('readm', (0,0))[0]:.1f} ns" if 'readm' in ops else "-"
    ww = f"{ops.get('writew', (0,0))[0]:.1f} ns" if 'writew' in ops else "-"
    wl = f"{ops.get('writel', (0,0))[0]:.1f} ns" if 'writel' in ops else "-"
    wm = f"{ops.get('writem', (0,0))[0]:.1f} ns" if 'writem' in ops else "-"
    print(f"| **{name}** | `{val}` | {rw} | {rl} | {rm} | {ww} | {wl} | {wm} |")

