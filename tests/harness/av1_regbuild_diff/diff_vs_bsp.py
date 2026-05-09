#!/usr/bin/env python3
"""Compare our reg_N_in.txt dumps against BSP captures.
Usage: python diff_vs_bsp.py <our_dump_dir> [kick_numbers...]"""
import sys, os, re

DMA_LSB = {65,67,69,71,73,75,77,79,81,83,85,87,89,91,93,95,
           97,99,101,103,105,107,109,111,113,133,135,137,139,
           141,143,145,147,167,169,171,173,175,177,179,183,190,
           192,194,196,198,200,202,204,224,226,228,230,232,234,
           236,238,326,328,339,341,348,350,505,507}
DMA_MSB = {v-1 for v in DMA_LSB}
SKIP = DMA_LSB | DMA_MSB

def read_bsp(path):
    regs = {}
    for line in open(path):
        m = re.match(r'reg\[\s*(\d+)\]\s*=\s*([0-9a-fA-F]+)', line)
        if m:
            regs[int(m.group(1))] = int(m.group(2), 16)
    return regs

def read_ours(path):
    regs = {}
    for line in open(path):
        m = re.match(r'reg\[\s*(\d+)\]\s*=\s*([0-9a-fA-F]+)', line)
        if m:
            regs[int(m.group(1))] = int(m.group(2), 16)
    return regs

our_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
bsp_dir = os.path.join(os.path.dirname(__file__), '../../data/av1/av1capture')
kicks = [int(x) for x in sys.argv[2:]] if len(sys.argv) > 2 else [0, 1]

for k in kicks:
    bsp_f = os.path.join(bsp_dir, f'reg_{k}_in.txt')
    our_f = os.path.join(our_dir, f'reg_{k}_in.txt')
    if not os.path.exists(bsp_f):
        print(f'kick {k}: no BSP reference'); continue
    if not os.path.exists(our_f):
        print(f'kick {k}: no our dump'); continue
    bsp = read_bsp(bsp_f)
    ours = read_ours(our_f)
    diffs = []
    for idx in range(512):
        if idx in SKIP: continue
        b = bsp.get(idx, 0)
        o = ours.get(idx, 0)
        if b != o:
            diffs.append((idx, b, o))
    if diffs:
        print(f'\nkick {k}: {len(diffs)} non-DMA register differences:')
        for idx, b, o in diffs:
            print(f'  reg[{idx:3d}]: bsp={b:08x}  ours={o:08x}  diff={b^o:08x}')
    else:
        print(f'kick {k}: all non-DMA registers match BSP exactly!')
