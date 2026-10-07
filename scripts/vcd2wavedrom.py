#!/usr/bin/env python3
"""
vcd2wavedrom.py - Convert PiStorm32-lite simulation VCD traces to WaveDrom JSON & SVG diagrams.

Usage:
    python3 scripts/vcd2wavedrom.py [--preset all] [--vcd sim.vcd]
    python3 scripts/vcd2wavedrom.py --preset read
    python3 scripts/vcd2wavedrom.py --preset write
    python3 scripts/vcd2wavedrom.py --preset filter
    python3 scripts/vcd2wavedrom.py --preset wishbone
"""

import sys
import os
import json
import subprocess
import argparse

STATE_NAMES = {
    1: 'S0_IDLE',
    2: 'S0_START',
    4: 'S1_AS',
    8: 'S2_OPEN',
    16: 'S3_WAIT',
    32: 'S4_NOP',
    64: 'S5_LATCH',
    128: 'UPDATE',
    256: 'TERM',
    512: 'Z2_FIN'
}

DSACK_NAMES = {
    '11': 'NONE',
    '00': '32-BIT',
    '10': '16-BIT',
    '01': '8-BIT'
}

def parse_vcd_header(f):
    vars_by_id = {}
    id_by_name = {}
    for line in f:
        line = line.strip()
        if line.startswith('$var'):
            parts = line.split()
            # $var wire <size> <id> <name> ... $end
            size = int(parts[2])
            var_id = parts[3]
            name = parts[4]
            vars_by_id[var_id] = {'name': name, 'size': size}
            # Also index by unique path or name
            if name not in id_by_name:
                id_by_name[name] = var_id
        elif line.startswith('$enddefinitions'):
            break
    return vars_by_id, id_by_name

def extract_timeline(vcd_path, target_signals, t_start, t_end):
    """
    Extracts time-value events for specified signals between t_start and t_end.
    Returns: dict of signal_name -> list of (time, value)
    """
    with open(vcd_path, 'r') as f:
        vars_by_id, id_by_name = parse_vcd_header(f)
        
        # Resolve target signal IDs
        sig_id_map = {}
        for sig in target_signals:
            if sig in id_by_name:
                sig_id_map[id_by_name[sig]] = sig
            else:
                matched = False
                for vid, info in vars_by_id.items():
                    if info['name'] == sig:
                        sig_id_map[vid] = sig
                        matched = True
                        break
                if not matched:
                    print(f"Warning: Signal '{sig}' not found in VCD definitions!")

        current_time = 0
        current_values = {name: 'x' for name in target_signals}
        initial_values = {name: 'x' for name in target_signals}
        timeline = {name: [] for name in target_signals}
        captured_initial = False

        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith('#'):
                current_time = int(line[1:])
                if current_time >= t_start and not captured_initial:
                    initial_values = dict(current_values)
                    captured_initial = True
                if current_time > t_end:
                    break
                continue

            # Format: '0<id>', '1<id>', 'b<bits> <id>', 'r<real> <id>'
            if line[0] in ('0', '1', 'x', 'z'):
                val = line[0]
                var_id = line[1:]
            elif line.startswith('b'):
                parts = line.split()
                val = parts[0][1:]
                var_id = parts[1]
            else:
                continue

            if var_id in sig_id_map:
                sig_name = sig_id_map[var_id]
                current_values[sig_name] = val
                if current_time >= t_start:
                    timeline[sig_name].append((current_time, val))

    return timeline, initial_values

def sample_signals(timeline, initial_values, target_signals, t_start, t_end, t_step):
    """
    Samples signals uniformly at discrete steps t_start, t_start + t_step, ...
    """
    samples = {name: [] for name in target_signals}
    num_steps = (t_end - t_start) // t_step + 1

    sig_ptrs = {name: 0 for name in target_signals}
    sig_curr = dict(initial_values)

    for step_idx in range(num_steps):
        sample_time = t_start + step_idx * t_step

        for name in target_signals:
            events = timeline[name]
            ptr = sig_ptrs[name]
            while ptr < len(events) and events[ptr][0] <= sample_time:
                sig_curr[name] = events[ptr][1]
                ptr += 1
            sig_ptrs[name] = ptr
            samples[name].append(sig_curr[name])

    return samples

def build_wavejson(samples, config, title):
    """
    Converts sampled arrays into WaveDrom JSON format.
    """
    signal_list = []

    for cfg in config:
        name = cfg['name']
        sig_type = cfg.get('type', 'bit')
        disp_name = cfg.get('label', name)

        raw_vals = samples[name]
        wave_chars = []
        data_vals = []
        last_val = None

        for val in raw_vals:
            if sig_type == 'clk':
                wave_chars.append('p' if val == '1' else 'P')
            elif sig_type == 'bit':
                if val in ('0', '1'):
                    if val == last_val:
                        wave_chars.append('.')
                    else:
                        wave_chars.append(val)
                else:
                    wave_chars.append('x')
                last_val = val
            elif sig_type in ('bus', 'state', 'dsack', 'hex'):
                if val == 'x' or not val:
                    formatted = 'x'
                    wave_val = 'x'
                else:
                    wave_val = '='
                    if sig_type == 'state':
                        try:
                            int_val = int(val, 2)
                            formatted = STATE_NAMES.get(int_val, hex(int_val))
                        except ValueError:
                            formatted = val
                    elif sig_type == 'dsack':
                        formatted = DSACK_NAMES.get(val, val)
                    elif sig_type == 'hex':
                        try:
                            formatted = f"0x{int(val, 2):X}"
                        except ValueError:
                            formatted = val
                    else:
                        formatted = val

                if wave_val == 'x':
                    if last_val == 'x':
                        wave_chars.append('.')
                    else:
                        wave_chars.append('x')
                    last_val = 'x'
                else:
                    if formatted == last_val:
                        wave_chars.append('.')
                    else:
                        wave_chars.append('=')
                        data_vals.append(formatted)
                    last_val = formatted

        entry = {
            "name": disp_name,
            "wave": "".join(wave_chars)
        }
        if data_vals:
            entry["data"] = data_vals
        signal_list.append(entry)

    wave_doc = {
        "signal": signal_list,
        "head": {
            "text": title,
            "tick": 0
        },
        "config": {
            "hscale": 1
        }
    }
    return wave_doc

def render_wavedrom(wave_json_path, svg_out_path):
    cmd = ["npx", "--yes", "wavedrom-cli", "-i", wave_json_path, "-s", svg_out_path]
    print(f"Rendering: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)
    print(f"Generated SVG: {svg_out_path}")

def generate_preset_read(vcd_path, out_dir):
    t_start = 1735
    t_end = 1795
    t_step = 2

    target_signals = [
        'AMIPLL_CLKOUT0', 'MC_CLK', 'state', 'address', 'MC_RW_OUT',
        'MC_AS_n_OUT', 'MC_DS_n_OUT', 'MC_DSACK_n', 'DA_IN'
    ]

    timeline, initial_values = extract_timeline(vcd_path, target_signals, t_start, t_end)
    samples = sample_signals(timeline, initial_values, target_signals, t_start, t_end, t_step)

    config = [
        {'name': 'AMIPLL_CLKOUT0', 'label': 'sys_clk (182 MHz)', 'type': 'clk'},
        {'name': 'MC_CLK',         'label': 'MC_CLK (14.18 MHz)', 'type': 'bit'},
        {'name': 'state',          'label': 'FSM State',          'type': 'state'},
        {'name': 'address',        'label': 'MC_A[23:0]',         'type': 'hex'},
        {'name': 'MC_RW_OUT',      'label': 'MC_RW (1=Read)',     'type': 'bit'},
        {'name': 'MC_AS_n_OUT',    'label': 'MC_AS_n',            'type': 'bit'},
        {'name': 'MC_DS_n_OUT',    'label': 'MC_DS_n',            'type': 'bit'},
        {'name': 'MC_DSACK_n',     'label': 'MC_DSACK[1:0]_n',    'type': 'dsack'},
        {'name': 'DA_IN',          'label': 'DA_IN (Read Data)',  'type': 'hex'}
    ]

    wave_doc = build_wavejson(samples, config, "Amiga 1200 Fast DSACK 32-Bit Read Cycle (68EC020)")

    json_path = os.path.join(out_dir, "m68k_read_cycle.json")
    svg_path = os.path.join(out_dir, "m68k_read_cycle.svg")
    with open(json_path, 'w') as f:
        json.dump(wave_doc, f, indent=2)

    render_wavedrom(json_path, svg_path)

def generate_preset_write(vcd_path, out_dir):
    t_start = 1615
    t_end = 1675
    t_step = 2

    target_signals = [
        'AMIPLL_CLKOUT0', 'MC_CLK', 'state', 'address', 'MC_RW_OUT',
        'MC_AS_n_OUT', 'MC_DS_n_OUT', 'MC_DSACK_n', 'DA_OUT'
    ]

    timeline, initial_values = extract_timeline(vcd_path, target_signals, t_start, t_end)
    samples = sample_signals(timeline, initial_values, target_signals, t_start, t_end, t_step)

    config = [
        {'name': 'AMIPLL_CLKOUT0', 'label': 'sys_clk (182 MHz)', 'type': 'clk'},
        {'name': 'MC_CLK',         'label': 'MC_CLK (14.18 MHz)', 'type': 'bit'},
        {'name': 'state',          'label': 'FSM State',          'type': 'state'},
        {'name': 'address',        'label': 'MC_A[23:0]',         'type': 'hex'},
        {'name': 'MC_RW_OUT',      'label': 'MC_RW (0=Write)',    'type': 'bit'},
        {'name': 'MC_AS_n_OUT',    'label': 'MC_AS_n',            'type': 'bit'},
        {'name': 'MC_DS_n_OUT',    'label': 'MC_DS_n',            'type': 'bit'},
        {'name': 'MC_DSACK_n',     'label': 'MC_DSACK[1:0]_n',    'type': 'dsack'},
        {'name': 'DA_OUT',         'label': 'DA_OUT (Write Data)','type': 'hex'}
    ]

    wave_doc = build_wavejson(samples, config, "Amiga 1200 Pipelined 32-Bit Write Cycle (68EC020)")

    json_path = os.path.join(out_dir, "m68k_write_cycle.json")
    svg_path = os.path.join(out_dir, "m68k_write_cycle.svg")
    with open(json_path, 'w') as f:
        json.dump(wave_doc, f, indent=2)

    render_wavedrom(json_path, svg_path)

def generate_preset_filter(vcd_path, out_dir):
    t_start = 28415
    t_end = 28465
    t_step = 2

    target_signals = [
        'AMIPLL_CLKOUT0', 'MC_CLK', 'mc_clk_raw_sync', 'mc_clk_lockout', 'mc_clk_filtered'
    ]

    timeline, initial_values = extract_timeline(vcd_path, target_signals, t_start, t_end)
    samples = sample_signals(timeline, initial_values, target_signals, t_start, t_end, t_step)

    config = [
        {'name': 'AMIPLL_CLKOUT0',  'label': 'sys_clk (182 MHz)',          'type': 'clk'},
        {'name': 'MC_CLK',          'label': 'MC_CLK (1.8V Ringing Raw)',  'type': 'bit'},
        {'name': 'mc_clk_raw_sync', 'label': 'CDC Sync [1:0]',              'type': 'bus'},
        {'name': 'mc_clk_lockout',  'label': 'Lockout Counter (3 ticks)',  'type': 'bus'},
        {'name': 'mc_clk_filtered', 'label': 'mc_clk_filtered (Cleaned)',  'type': 'bit'}
    ]

    wave_doc = build_wavejson(samples, config, "A1200 Motherboard Inductive Ringing (1.8V Dip) & Digital Lockout Filter")

    json_path = os.path.join(out_dir, "clock_glitch_filter.json")
    svg_path = os.path.join(out_dir, "clock_glitch_filter.svg")
    with open(json_path, 'w') as f:
        json.dump(wave_doc, f, indent=2)

    render_wavedrom(json_path, svg_path)

def generate_preset_wishbone(vcd_path, out_dir):
    t_start = 31665
    t_end = 31685
    t_step = 2

    target_signals = [
        'AMIPLL_CLKOUT0', 'wb_cyc', 'wb_stb', 'wb_we', 'wb_adr',
        'wb_dat_m2s', 'wb_dat_s2m', 'wb_ack', 'MC_AS_n_OUT'
    ]

    timeline, initial_values = extract_timeline(vcd_path, target_signals, t_start, t_end)
    samples = sample_signals(timeline, initial_values, target_signals, t_start, t_end, t_step)

    config = [
        {'name': 'AMIPLL_CLKOUT0', 'label': 'sys_clk (182 MHz Wishbone)', 'type': 'clk'},
        {'name': 'wb_cyc',         'label': 'wb_cyc',                     'type': 'bit'},
        {'name': 'wb_stb',         'label': 'wb_stb',                     'type': 'bit'},
        {'name': 'wb_we',          'label': 'wb_we (1=Write)',            'type': 'bit'},
        {'name': 'wb_adr',         'label': 'wb_adr [15:0]',              'type': 'hex'},
        {'name': 'wb_dat_m2s',     'label': 'wb_dat_m2s (Write Data)',    'type': 'hex'},
        {'name': 'wb_dat_s2m',     'label': 'wb_dat_s2m (Read Data)',     'type': 'hex'},
        {'name': 'wb_ack',         'label': 'wb_ack (0 WS)',              'type': 'bit'},
        {'name': 'MC_AS_n_OUT',    'label': 'Amiga MC_AS_n (Isolated)',   'type': 'bit'}
    ]

    wave_doc = build_wavejson(samples, config, "Virtual Zorro-II Internal Wishbone Access (182 MHz, 0-WS, Isolated)")

    json_path = os.path.join(out_dir, "wishbone_bus_cycle.json")
    svg_path = os.path.join(out_dir, "wishbone_bus_cycle.svg")
    with open(json_path, 'w') as f:
        json.dump(wave_doc, f, indent=2)

    render_wavedrom(json_path, svg_path)

def generate_preset_prefetch(vcd_path, out_dir):
    t_start = 2135
    t_end = 2165
    t_step = 2

    target_signals = [
        'AMIPLL_CLKOUT0', 'MC_CLK', 'state', 'address', 'cur_prefetch_hit',
        'prefetch_hit_terminate', 'MC_AS_n_OUT', 'DA_IN'
    ]

    timeline, initial_values = extract_timeline(vcd_path, target_signals, t_start, t_end)
    samples = sample_signals(timeline, initial_values, target_signals, t_start, t_end, t_step)

    config = [
        {'name': 'AMIPLL_CLKOUT0',        'label': 'sys_clk (182 MHz)',           'type': 'clk'},
        {'name': 'MC_CLK',                'label': 'MC_CLK (14.18 MHz)',          'type': 'bit'},
        {'name': 'state',                 'label': 'FSM State',                   'type': 'state'},
        {'name': 'address',               'label': 'Requested Address',           'type': 'hex'},
        {'name': 'cur_prefetch_hit',      'label': 'Prefetch Tag Match (Hit)',    'type': 'bit'},
        {'name': 'prefetch_hit_terminate','label': 'prefetch_hit_terminate (0-WS)','type': 'bit'},
        {'name': 'MC_AS_n_OUT',           'label': 'Amiga MC_AS_n (0 Bus Cycles)','type': 'bit'},
        {'name': 'DA_IN',                 'label': 'Returned Data (Prefetch Buffer)','type': 'hex'}
    ]

    wave_doc = build_wavejson(samples, config, "Speculative Prefetch Cache Hit (0 Wait States, 0 Bus Cycles)")

    json_path = os.path.join(out_dir, "prefetch_cache_hit.json")
    svg_path = os.path.join(out_dir, "prefetch_cache_hit.svg")
    with open(json_path, 'w') as f:
        json.dump(wave_doc, f, indent=2)

    render_wavedrom(json_path, svg_path)

def main():
    parser = argparse.ArgumentParser(description="Convert VCD to WaveDrom JSON/SVG")
    parser.add_argument("--vcd", default="sim.vcd", help="Path to input VCD file")
    parser.add_argument("--outdir", default="DOCS/waveforms", help="Output directory for JSON/SVG")
    parser.add_argument("--preset", choices=["all", "read", "write", "filter", "wishbone", "prefetch"], default="all")

    args = parser.parse_args()

    if not os.path.exists(args.vcd):
        print(f"Error: VCD file '{args.vcd}' not found! Run 'make trace' first.")
        sys.exit(1)

    os.makedirs(args.outdir, exist_ok=True)

    if args.preset in ("all", "read"):
        print("\n--- Generating Preset: Read Cycle ---")
        generate_preset_read(args.vcd, args.outdir)

    if args.preset in ("all", "write"):
        print("\n--- Generating Preset: Write Cycle ---")
        generate_preset_write(args.vcd, args.outdir)

    if args.preset in ("all", "filter"):
        print("\n--- Generating Preset: Clock Glitch Filter ---")
        generate_preset_filter(args.vcd, args.outdir)

    if args.preset in ("all", "wishbone"):
        print("\n--- Generating Preset: Wishbone Access ---")
        generate_preset_wishbone(args.vcd, args.outdir)

    if args.preset in ("all", "prefetch"):
        print("\n--- Generating Preset: Prefetch Hit ---")
        generate_preset_prefetch(args.vcd, args.outdir)

    print("\n[SUCCESS] Waveforms extracted and rendered to SVG successfully!")

if __name__ == "__main__":
    main()
