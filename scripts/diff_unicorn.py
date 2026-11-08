#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Thumb-2 指令级差分对拍：dtwin（GDB RSP） vs Unicorn（QEMU 派生参考模型）
#
# 原理：
#   - 生成器按指令类产出 constrained-random Thumb-2 编码流（字节级）。语义由两侧
#     解码器各自决定——编码器只需产出合法编码，即使标错指令类别也只影响覆盖率
#     统计，不影响对拍有效性。
#   - 每个用例：双方重置为相同寄存器/内存现场（r0-r12 含边界值、xpsr 随机 NZCV、
#     数据窗/栈窗填充相同随机字节），随后逐指令锁步执行。
#   - 每条指令后比对 r0-r12 / sp / lr / pc / xpsr(NZCVQ+T) / fpscr，以及 Unicorn
#     写钩子记录到的内存地址（从 dtwin 读回同地址比对）。
#   - 失配输出：PC、编码字节、capstone 反汇编（可选）、双方寄存器/内存差异。
#
# 用法：
#   python3 scripts/diff_unicorn.py --host 127.0.0.1 --port 33333 \
#       [--count 500] [--instrs 6] [--seed 1] [--classes all|alu16,dp16,...]
#       [--max-mismatches 20] [-v]
#
# 依赖：pip install unicorn（必需）、capstone（可选，用于反汇编失配指令）
# =============================================================================

import argparse
import random
import socket
import struct
import sys

try:
    import capstone
    _MD = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB | capstone.CS_MODE_MCLASS)
except ImportError:  # 反汇编仅为诊断增强，缺失时降级为纯 hex 输出
    _MD = None

from unicorn import (
    Uc, UcError, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB, UC_HOOK_MEM_WRITE,
)
from unicorn.arm_const import (
    UC_ARM_REG_R0, UC_ARM_REG_R13, UC_ARM_REG_LR, UC_ARM_REG_PC,
    UC_ARM_REG_MSP, UC_ARM_REG_PSP, UC_ARM_REG_XPSR, UC_ARM_REG_CONTROL,
    UC_ARM_REG_FPSCR,
)
import unicorn.arm_const as _ac

# ---------------------------------------------------------------- 内存布局
SRAM_BASE   = 0x2000_0000
SRAM_SIZE   = 0x2_0000          # dtwin S32K312 模型：128KB SRAM
STREAM_BASE = SRAM_BASE         # 指令流放置处（SRAM 可执行）
STREAM_MAX  = 0x100             # 单用例指令流最大字节数
DATA_WIN    = 0x2000_1000       # 数据窗口（load/store 基址指向这里）
DATA_SIZE   = 0x1000
STACK_WIN   = 0x2000_E000       # 栈窗口（push/pop/sp 相对寻址）
STACK_TOP   = 0x2000_EC00       # 初始 SP：下移为 [sp,+imm] 读留出窗内上探空间
STACK_END   = 0x2000_F000       # 栈窗口实际边界（初始 SP 之上仍属窗口）

# 供 load/store 生成器使用的基址寄存器（setup 阶段装载数据窗内地址）
BASE_REGS = (5, 6)

# 边界值语料：寄存器初始化时高频采样
EDGE_U32 = (0, 1, 2, 0x7FFF_FFFF, 0x8000_0000, 0xFFFF_FFFF, 0x5555_5555,
            0xAAAA_AAAA, 0x0000_00FF, 0xFF00_0000)

XPSR_MASK = 0xF900_01FF         # NZCVQ + T + IPSR（线程模式 IPSR=0）


# ============================================================ RSP 客户端
class Rsp:
    """最小 GDB RSP 客户端（ack 模式 + RLE 解压）。"""

    def __init__(self, host: str, port: int):
        self.s = socket.create_connection((host, port), timeout=10)
        self.s.settimeout(10)

    def _enc(self, payload: bytes) -> bytes:
        return b'$' + payload + b'#' + f'{sum(payload) & 0xFF:02x}'.encode()

    @staticmethod
    def _dec_rle(data: bytes) -> bytes:
        out = bytearray()
        i = 0
        while i < len(data):
            if data[i:i + 1] == b'*' and out and i + 1 < len(data):
                out.extend([out[-1]] * ((data[i + 1] - 29) & 0xFF))
                i += 2
            else:
                out.append(data[i])
                i += 1
        return bytes(out)

    def cmd(self, payload: bytes) -> bytes:
        self.s.sendall(self._enc(payload))
        while True:
            b = self.s.recv(1)
            if not b:
                raise ConnectionError("dtwin 关闭了 RSP 连接")
            if b == b'$':
                break
        p = bytearray()
        while True:
            b = self.s.recv(1)
            if not b:
                raise ConnectionError("RSP 报文截断")
            if b == b'#':
                break
            p += b
        self.s.recv(2)
        self.s.sendall(b'+')
        return self._dec_rle(bytes(p))

    def handshake(self):
        self.cmd(b'qSupported:xmlRegisters=arm')
        self.cmd(b'?')

    # ---- 寄存器 ----
    def read_g(self) -> bytes:
        r = self.cmd(b'g')
        if len(r) != 448:
            raise RuntimeError(f"g 报文长度异常: {len(r)}（期望 448 hex）")
        return bytes.fromhex(r.decode())

    def write_g(self, regs224: bytes):
        assert len(regs224) == 224
        r = self.cmd(b'G' + regs224.hex().encode())
        if r != b'OK':
            raise RuntimeError(f"G 报文被拒: {r!r}")

    # ---- 内存 ----
    def write_mem(self, addr: int, data: bytes):
        r = self.cmd(f'M{addr:x},{len(data):x}:'.encode() + data.hex().encode())
        if r != b'OK':
            raise RuntimeError(f"M 报文被拒 @{addr:#x}: {r!r}")

    def read_mem(self, addr: int, length: int) -> bytes:
        r = self.cmd(f'm{addr:x},{length:x}'.encode())
        if len(r) != length * 2:
            raise RuntimeError(f"m 报文长度异常 @{addr:#x}+{length}: {r!r}")
        return bytes.fromhex(r.decode())

    def step(self) -> bytes:
        """单步一条指令，返回 stop 报文（如 b'S05'）。"""
        return self.cmd(b's')

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


# ============================================================ 寄存器快照
REG_NAMES = (
    [f'r{i}' for i in range(13)] + ['sp', 'lr', 'pc', 'xpsr',
    'msp', 'psp', 'primask', 'basepri', 'faultmask', 'control']
    + [f'd{i}' for i in range(16)] + ['fpscr']
)


def unpack_g(buf: bytes) -> dict:
    """解析 224 字节 g 报文为命名 dict（d 寄存器按 u64，其余 u32）。"""
    vals = {}
    off = 0
    for i, name in enumerate(REG_NAMES):
        if name.startswith('d') and name[1:].isdigit():
            vals[name] = struct.unpack_from('<Q', buf, off)[0]
            off += 8
        else:
            vals[name] = struct.unpack_from('<I', buf, off)[0]
            off += 4
    return vals


def pack_g(vals: dict) -> bytes:
    out = bytearray()
    for name in REG_NAMES:
        v = vals[name]
        if name.startswith('d') and name[1:].isdigit():
            out += struct.pack('<Q', v & 0xFFFF_FFFF_FFFF_FFFF)
        else:
            out += struct.pack('<I', v & 0xFFFF_FFFF)
    assert len(out) == 224
    return bytes(out)


# ============================================================ Unicorn 参考侧
class UcRef:
    def __init__(self):
        self.mu = Uc(UC_ARCH_ARM, UC_MODE_MCLASS | UC_MODE_THUMB)
        self.mu.mem_map(0x0000_0000, 0x8_0000)              # FLASH
        self.mu.mem_map(SRAM_BASE, SRAM_SIZE)               # SRAM
        self.mu.mem_map(0xE000_0000, 0x10_0000)             # SYSTEM（兜底）
        self.mu.mem_write(0, b'\xff' * 0x8_0000)            # Flash 擦除态
        self.writes = []                                    # (addr, size, val)
        self.mu.hook_add(UC_HOOK_MEM_WRITE,
                         lambda mu, acc, addr, size, val, ud:
                         self.writes.append((addr, size, val)))
        self.faulted = None

    def reset(self, vals: dict):
        for i in range(13):
            self.mu.reg_write(UC_ARM_REG_R0 + i, vals[f'r{i}'])
        self.mu.reg_write(UC_ARM_REG_MSP, vals['msp'])
        self.mu.reg_write(UC_ARM_REG_PSP, vals['psp'])
        self.mu.reg_write(UC_ARM_REG_R13, vals['sp'])
        self.mu.reg_write(UC_ARM_REG_LR, vals['lr'])
        self.mu.reg_write(UC_ARM_REG_CONTROL, vals['control'])
        self.mu.reg_write(UC_ARM_REG_XPSR, vals['xpsr'])
        self.mu.reg_write(UC_ARM_REG_FPSCR, vals['fpscr'])
        self.mu.reg_write(UC_ARM_REG_PC, vals['pc'])
        for i in range(32):
            sreg = getattr(_ac, f'UC_ARM_REG_S{i}')
            d = vals[f'd{i // 2}']
            self.mu.reg_write(sreg, (d >> (32 * (i & 1))) & 0xFFFF_FFFF)
        self.writes.clear()
        self.faulted = None

    def load_mem(self, addr: int, data: bytes):
        self.mu.mem_write(addr, data)

    def step(self):
        pc = self.mu.reg_read(UC_ARM_REG_PC)
        try:
            self.mu.emu_start(pc | 1, STREAM_BASE + 0x1000, count=1)
        except UcError as e:
            self.faulted = str(e)

    def snapshot(self) -> dict:
        vals = {}
        for i in range(13):
            vals[f'r{i}'] = self.mu.reg_read(UC_ARM_REG_R0 + i)
        vals['sp'] = self.mu.reg_read(UC_ARM_REG_R13)
        vals['lr'] = self.mu.reg_read(UC_ARM_REG_LR)
        vals['pc'] = self.mu.reg_read(UC_ARM_REG_PC) & ~1
        vals['xpsr'] = self.mu.reg_read(UC_ARM_REG_XPSR)
        vals['fpscr'] = self.mu.reg_read(UC_ARM_REG_FPSCR)
        for i in range(16):
            lo = self.mu.reg_read(getattr(_ac, f'UC_ARM_REG_S{2 * i}'))
            hi = self.mu.reg_read(getattr(_ac, f'UC_ARM_REG_S{2 * i + 1}'))
            vals[f'd{i}'] = (hi << 32) | lo
        return vals


# ============================================================ Thumb-2 编码器
# 约定：生成器返回 (code: bytes, overrides: dict)。overrides 目前支持
# {'base5': True / 'base6': True} —— setup 阶段把 r5/r6 装载为数据窗内地址。

def h16(v: int) -> bytes:
    return struct.pack('<H', v)


def h32(hw1: int, hw2: int) -> bytes:
    return struct.pack('<HH', hw1, hw2)


def _lo3(rng) -> tuple:
    return (rng.randrange(8), rng.randrange(8), rng.randrange(8))  # rd, rn, rm


def gen_alu_imm3(rng):
    """adds/subs rd, rn, #imm3（16 位）"""
    rd, rn, _ = _lo3(rng)
    imm3 = rng.randrange(8)
    op = rng.choice((0b0001110, 0b0001111))
    return h16((op << 9) | (imm3 << 6) | (rn << 3) | rd), {}


def gen_alu_reg3(rng):
    """adds/subs rd, rn, rm（16 位）"""
    rd, rn, rm = _lo3(rng)
    op = rng.choice((0b0001100, 0b0001101))
    return h16((op << 9) | (rm << 6) | (rn << 3) | rd), {}


def gen_mov_imm8(rng):
    """movs/adds/subs/cmp rn, #imm8（16 位）"""
    rdn = rng.randrange(8)
    op = rng.choice((0b00100, 0b00101, 0b00110, 0b00111))
    return h16((op << 11) | (rdn << 8) | rng.randrange(256)), {}


def gen_shift_imm5(rng):
    """lsls/lsrs/asrs rd, rm, #imm5（16 位）"""
    rd, _, rm = _lo3(rng)
    imm5 = rng.randrange(32)
    op = rng.choice((0b000, 0b001, 0b010))
    return h16((op << 11) | (imm5 << 6) | (rm << 3) | rd), {}


_DP_OPS = tuple(range(16))  # and/eor/lsl/lsr/asr/adc/sbc/ror/tst/rsb/cmp/cmn/orr/mul/bic/mvn

def gen_dp16(rng):
    """010000 xxxx rm rn 低寄存器数据操作（16 位）"""
    rdn = rng.randrange(8)
    rm = rng.randrange(8)
    op = rng.choice(_DP_OPS)
    return h16(0x4000 | (op << 6) | (rm << 3) | rdn), {}


def gen_hireg(rng):
    """mov/add/cmp 高寄存器（16 位），排除写 PC 的形式"""
    op = rng.choice((0b01000101, 0b01000110, 0b01000100))  # cmp/mov/add
    while True:
        rm = rng.randrange(16)
        rdn = rng.randrange(16)
        if op == 0b01000110 and rdn == 15:   # mov pc, rm → 分支，排除
            continue
        if op == 0b01000100 and rdn == 15:   # add pc, rm → 排除
            continue
        if rm == 15:                          # 读 PC 两边语义一致但噪音大，先排除
            continue
        break
    d = (rdn & 8) >> 3
    return h16((op << 8) | (d << 7) | (rm << 3) | (rdn & 7)), {}


def _mem_overrides(rng, width: int, imm_max: int) -> dict:
    """为访存指令选基址寄存器与窗内偏移。

    基址固定（r5=DATA_WIN+0x400, r6=DATA_WIN+0x800，见 make_regs），imm 按
    "基址+imm 不越出窗口" 钳制——流内多条访存指令共享基址寄存器也不会互相
    覆盖约束。
    """
    b = rng.choice(BASE_REGS)
    base = DATA_WIN + (0x400 if b == 5 else 0x800)
    hi = min(imm_max, DATA_WIN + DATA_SIZE - width - base)
    imm = rng.randrange(0, max(hi, width), width)
    return ({}, b, imm)


# 流内安全目标寄存器：load 不得写基址/偏移寄存器（r3/r4=偏移, r5/r6=基址），
# 否则同流后续指令的地址被污染为野性地址
SAFE_RT = tuple(r for r in range(8) if r not in (3, 4, 5, 6))


def gen_ldst_imm5(rng):
    """ldr/str/ldrb/strb/ldrh/strh rd, [rn, #imm5]（16 位）"""
    width = rng.choice((1, 2, 4))
    load = rng.random() < 0.5
    ov, b, imm = _mem_overrides(rng, width, 31 * width)
    imm5 = imm // width
    # load 的目标寄存器避开基址/偏移寄存器，防止污染同流后续地址
    rt = rng.choice(SAFE_RT) if load else rng.randrange(8)
    if width == 4:
        code = h16((0b0110 << 12) | ((1 if load else 0) << 11) | (imm5 << 6) | (b << 3) | rt)
    elif width == 2:
        code = h16((0b1000 << 12) | ((1 if load else 0) << 11) | (imm5 << 6) | (b << 3) | rt)
    else:
        code = h16((0b0111 << 12) | ((1 if load else 0) << 11) | (imm5 << 6) | (b << 3) | rt)
    return code, ov


def gen_ldst_reg(rng):
    """ldr/str/b/h/sb/sh rd, [rn, rm]（16 位）"""
    op = rng.randrange(8)  # str,strh,strb,ldrsb,ldr,ldrh,ldrb,ldrsh
    width = (4, 2, 1, 1, 4, 2, 1, 2)[op]
    _, b, _ = _mem_overrides(rng, width, 0)
    rm = rng.choice((3, 4))                 # 偏移寄存器：setup 装载窗内小偏移
    # load 目标避开基址/偏移寄存器
    rt = rng.choice(SAFE_RT) if op >= 3 else rng.randrange(8)
    # 固定 4 字节对齐：同寄存器被流内多条指令重复指定时任意覆盖顺序都保持对齐
    off = rng.randrange(0, 0x400, 4)
    code = h16((0b0101 << 12) | (op << 9) | (rm << 6) | (b << 3) | rt)
    return code, {f'fix{rm}': off}


def gen_ldr_literal(rng):
    """ldr rd, [pc, #imm8*4]（16 位，读指令流后方的字面量池）"""
    rt = rng.randrange(8)
    imm8 = rng.randrange(0, 32)  # 目标落在流后 128B 内（初始化相同随机数据）
    return h16(0x4800 | (rt << 8) | imm8), {}


def gen_ldst_sprel(rng):
    """ldr/str rd, [sp, #imm8*4]（16 位）"""
    load = rng.random() < 0.5
    rt = rng.randrange(8)
    imm8 = rng.randrange(0, 64)
    code = h16((0b1001 << 12) | ((1 if load else 0) << 11) | (rt << 8) | imm8)
    return code, {}


def gen_push_pop(rng):
    """push/pop 寄存器子集（16 位）"""
    pop = rng.random() < 0.5
    regs = rng.randrange(1, 256)
    code = h16((0b1011 << 12) | ((1 if pop else 0) << 11) | 0b010 << 8 | regs)
    return code, {}


def gen_extend_rev16(rng):
    """sxth/sxtb/uxth/uxtb/rev/rev16/revsh（16 位）"""
    rd, _, rm = _lo3(rng)
    op = rng.choice((0b1011001001, 0b1011001000, 0b1011001011, 0b1011001010,
                     0b1011101000, 0b1011101001, 0b1011101011))
    return h16((op << 6) | (rm << 3) | rd), {}


def gen_add_sp(rng):
    """add rd, sp, #imm8*4 / add/sub sp, #imm7*4（16 位）"""
    kind = rng.randrange(3)
    if kind == 0:
        return h16(0xA800 | (rng.randrange(8) << 8) | rng.randrange(64)), {}
    if kind == 1:
        return h16(0xB000 | rng.randrange(64)), {}
    return h16(0xB080 | rng.randrange(64)), {}


def gen_cbz(rng):
    """cbz/cbnz rn, +2（跳过下一条 movs 标记，验证条件分支判定）"""
    rn = rng.randrange(8)
    nonzero = rng.random() < 0.5
    cb = h16((0xB900 if nonzero else 0xB100) | (1 << 3) | rn)
    marker = h16(0x2000 | (rn << 8) | 0xA5)   # movs rn, #0xA5（被跳过则不执行）
    return cb + marker, {}


def gen_bcond(rng):
    """b<cond> +2（跳过 movs 标记；cond != AL/NV）"""
    cond = rng.randrange(14)
    rd = rng.randrange(8)
    b = h16(0xD000 | (cond << 8) | 1)
    marker = h16(0x2000 | (rd << 8) | 0x5A)
    return b + marker, {}


def gen_nop16(rng):
    return h16(0xBF00), {}


# ---- 32 位（Thumb-2）----

def gen_dp_modimm(rng):
    """and/bic/orr/orn/eor/add/adc/sbc/sub/rsb .w[.s] 修改立即数"""
    op = rng.choice((0b0000, 0b0001, 0b0010, 0b0011, 0b0100,
                     0b1000, 0b1010, 0b1011, 0b1101, 0b1110))
    s = rng.random() < 0.5
    if op in (0b0010, 0b0011) and rng.random() < 0.3:
        rn = 0xF                       # mov.w / mvn.w 形式
    else:
        rn = rng.randrange(15)
    # rd=15 仅对 ANDS/EORS/ADDS/SUBS 合法（= TST/TEQ/CMN/CMP 别名，丢弃结果）；
    # 其余 op + rd=15 属 UNPREDICTABLE（QEMU 当 ALU 写 PC 分支，dtwin 不建模）
    if s and op in (0b0000, 0b0100, 0b1000, 0b1101) and rng.random() < 0.3:
        rd = 0xF                       # tst/teq/cmn/cmp 形式
    else:
        rd = rng.randrange(15)
    i = rng.randrange(2)
    imm3 = rng.randrange(8)
    imm8 = rng.randrange(256)
    hw1 = 0xF000 | (i << 10) | (op << 5) | ((1 if s else 0) << 4) | rn
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return h32(hw1, hw2), {}


def gen_movw_movt(rng):
    rd = rng.randrange(15)
    imm16 = rng.randrange(0x10000)
    i = (imm16 >> 11) & 1
    imm4 = (imm16 >> 12) & 0xF
    imm3 = (imm16 >> 8) & 7
    imm8 = imm16 & 0xFF
    top = rng.random() < 0.5
    hw1 = (0xF2C0 if top else 0xF240) | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return h32(hw1, hw2), {}


def gen_addw_subw(rng):
    rd = rng.randrange(15)
    rn = rng.randrange(15)
    imm12 = rng.randrange(0x1000)
    i = (imm12 >> 11) & 1
    imm3 = (imm12 >> 8) & 7
    imm8 = imm12 & 0xFF
    sub = rng.random() < 0.5
    hw1 = (0xF2A0 if sub else 0xF200) | (i << 10) | rn
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return h32(hw1, hw2), {}


_LDST_W = (  # (hw1_base, width)——imm12（T3/T4）形式 top，as/QEMU 实测
    (0xF880, 1), (0xF890, 1), (0xF990, 1),   # strb / ldrb / ldrsb
    (0xF8A0, 2), (0xF8B0, 2), (0xF9B0, 2),   # strh / ldrh / ldrsh
    (0xF8C0, 4), (0xF8D0, 4),                # str / ldr
)

def gen_ldst_w(rng):
    """ldr/str 系列 .w [rn, #imm12]"""
    base_enc, width = rng.choice(_LDST_W)
    ov, b, imm = _mem_overrides(rng, width, 0xFFF)
    is_load = base_enc in (0xF890, 0xF990, 0xF8B0, 0xF9B0, 0xF8D0)
    rt = rng.choice(SAFE_RT) if is_load else rng.randrange(15)
    code = h32(base_enc | b, (rt << 12) | imm)
    return code, ov


def gen_shift_w(rng):
    """lsl/lsr/asr/ror .w[.s] rd, rm, #imm5（含 mov.w 别名与 rrx；0xEA4F 空间）"""
    typ = rng.randrange(4)
    s = rng.random() < 0.5
    rd = rng.randrange(15)
    rm = rng.randrange(15)
    imm5 = rng.randrange(32)  # LSL imm5=0 → mov.w；LSR/ASR imm5=0 → #32；ROR 0 → rrx
    hw1 = 0xEA4F | ((1 if s else 0) << 4)
    hw2 = ((imm5 >> 2) << 12) | (rd << 8) | ((imm5 & 3) << 6) | (typ << 4) | rm
    return h32(hw1, hw2), {}


def gen_shift_reg_w(rng):
    """lsl/lsr/asr/ror .w[.s] rd, rn, rm（寄存器移位）"""
    typ = rng.randrange(4)
    s = rng.random() < 0.5
    rd = rng.randrange(15)
    rn = rng.randrange(15)
    rm = rng.randrange(15)
    hw1 = 0xFA00 | (typ << 5) | ((1 if s else 0) << 4) | rm
    hw2 = 0xF000 | (rd << 8) | rn
    return h32(hw1, hw2), {}


def gen_mul_mla(rng):
    """mul / mla / mls（32 位）"""
    kind = rng.randrange(3)
    rd, rn, rm = rng.randrange(15), rng.randrange(15), rng.randrange(15)
    if kind == 0:
        hw1, hw2 = 0xFB00 | rn, 0xF000 | (rd << 8) | rm
    else:
        ra = rng.randrange(15)
        # mla：低半字 bit4=0；mls：低半字 bit4=1（QEMU/capstone 实测布局；
        # 0xFB10 高半字族是 SMLABB，属未实现的 DSP 缺口，不在此类生成）
        hw1 = 0xFB00 | rn
        hw2 = (ra << 12) | (rd << 8) | (0x10 if kind == 2 else 0) | rm
    return h32(hw1, hw2), {}


def gen_div(rng):
    """sdiv / udiv"""
    rd, rn, rm = rng.randrange(15), rng.randrange(15), rng.randrange(15)
    hw1 = (0xFB90 if rng.random() < 0.5 else 0xFBB0) | rn
    hw2 = 0xF0F0 | (rd << 8) | rm
    return h32(hw1, hw2), {}


def gen_mull_long(rng):
    """smull / umull / smlal / umlal"""
    op = rng.choice((0xFB80, 0xFBA0, 0xFBC0, 0xFBE0))
    rn = rng.randrange(15)
    rdlo = rng.randrange(15)
    while True:
        rdhi = rng.randrange(15)
        if rdhi != rdlo:
            break
    rm = rng.randrange(15)
    return h32(op | rn, (rdlo << 12) | (rdhi << 8) | rm), {}


def gen_bitfield(rng):
    """ubfx / sbfx / bfi / bfc"""
    op = rng.choice((0xF3C0, 0xF340, 0xF360))
    rn = rng.randrange(15)
    if op == 0xF360 and rng.random() < 0.5:
        rn = 0xF                       # bfc
    rd = rng.randrange(15)
    lsb = rng.randrange(32)
    width = rng.randrange(1, 33 - lsb)
    # ubfx/sbfx 低 5 位 = width-1；bfi/bfc 低 5 位 = msb = lsb+width-1
    low5 = (lsb + width - 1) if op == 0xF360 else (width - 1)
    hw2 = ((lsb >> 2) << 12) | (rd << 8) | ((lsb & 3) << 6) | low5
    return h32(op | rn, hw2), {}


def gen_sat(rng):
    """ssat / usat（sh 在 hw1 bit5；sh=1 时 imm5 须 ≥1，imm5=0 落入 SAT16 空间）"""
    usat = rng.random() < 0.5
    sh = rng.randrange(2)
    rd = rng.randrange(15)
    rn = rng.randrange(15)
    if usat:
        satimm = rng.randrange(32)
        hw1 = 0xF380 | (sh << 5) | rn
    else:
        satimm = rng.randrange(1, 32)
        hw1 = 0xF300 | (sh << 5) | rn
    imm5 = rng.randrange(1, 32) if sh else rng.randrange(32)
    hw2 = ((imm5 >> 2) << 12) | (rd << 8) | ((imm5 & 3) << 6) | satimm
    return h32(hw1, hw2), {}


def gen_sat16(rng):
    """ssat16 / usat16 rd, #n, rn（位宽字段 bits[5:0]，0-63；hw2 bits[15:12]=0）"""
    usat = rng.random() < 0.5
    hw1 = (0xF3A0 if usat else 0xF320) | rng.randrange(15)
    hw2 = (rng.randrange(15) << 8) | rng.randrange(64)
    return h32(hw1, hw2), {}


def gen_clz_rev_w(rng):
    """clz / rbit / rev / rev16 / revsh（32 位）"""
    # QEMU/capstone 实测 op 映射：rev=0xF080、rev16=0xF090、rbit=0xF0A0、revsh=0xF0B0
    hw1_op, hw2_op = rng.choice(((0xFAB0, 0xF080),   # clz
                                 (0xFA90, 0xF0A0),   # rbit
                                 (0xFA90, 0xF080),   # rev
                                 (0xFA90, 0xF090),   # rev16
                                 (0xFA90, 0xF0B0)))  # revsh
    rd = rng.randrange(15)
    rm = rng.randrange(15)
    return h32(hw1_op | rm, hw2_op | (rd << 8) | rm), {}


def gen_extend_rot(rng):
    """sxth/sxtb/uxth/uxtb .w（含 rotate 变体）"""
    hw1_op = rng.choice((0xFA0F, 0xFA4F, 0xFA1F, 0xFA5F))
    rd = rng.randrange(15)
    rm = rng.randrange(15)
    rot = rng.randrange(4)
    return h32(hw1_op, 0xF080 | (rd << 8) | (rot << 4) | rm), {}


def gen_pkh(rng):
    """pkhbt / pkhtb"""
    tb = rng.random() < 0.5
    rd, rn, rm = rng.randrange(15), rng.randrange(15), rng.randrange(15)
    imm5 = rng.randrange(32)
    hw1 = (0xEAE0 if tb else 0xEAC0) | rn
    hw2 = ((imm5 >> 2) << 12) | (rd << 8) | ((imm5 & 3) << 6) | rm
    return h32(hw1, hw2), {}


# 指令类注册表：名字 → 生成器
CLASSES = {
    # ---- Tier A：16 位整数 ----
    'alu_imm3':    gen_alu_imm3,
    'alu_reg3':    gen_alu_reg3,
    'mov_imm8':    gen_mov_imm8,
    'shift_imm5':  gen_shift_imm5,
    'dp16':        gen_dp16,
    'hireg':       gen_hireg,
    'ldst_imm5':   gen_ldst_imm5,
    'ldst_reg':    gen_ldst_reg,
    'ldr_literal': gen_ldr_literal,
    'ldst_sprel':  gen_ldst_sprel,
    'push_pop':    gen_push_pop,
    'extend_rev16': gen_extend_rev16,
    'add_sp':      gen_add_sp,
    'cbz':         gen_cbz,
    'bcond':       gen_bcond,
    'nop16':       gen_nop16,
    # ---- Tier B：32 位整数 ----
    'dp_modimm':   gen_dp_modimm,
    'movw_movt':   gen_movw_movt,
    'addw_subw':   gen_addw_subw,
    'ldst_w':      gen_ldst_w,
    'shift_w':     gen_shift_w,
    'shift_reg_w': gen_shift_reg_w,
    'mul_mla':     gen_mul_mla,
    'div':         gen_div,
    'mull_long':   gen_mull_long,
    'bitfield':    gen_bitfield,
    'sat':         gen_sat,
    'sat16':       gen_sat16,
    'clz_rev_w':   gen_clz_rev_w,
    'extend_rot':  gen_extend_rot,
    'pkh':         gen_pkh,
}


# ============================================================ 用例现场构造
def random_u32(rng) -> int:
    if rng.random() < 0.45:
        return rng.choice(EDGE_U32)
    return rng.getrandbits(32)


def make_regs(rng, overrides: dict) -> dict:
    vals = {}
    for i in range(13):
        vals[f'r{i}'] = random_u32(rng)
    # 访存基址寄存器：固定在数据窗内（偏移由生成器按窗口钳制）
    vals['r5'] = DATA_WIN + 0x400
    vals['r6'] = DATA_WIN + 0x800
    for key, addr in overrides.items():
        if key.startswith('fix'):
            vals[f'r{int(key[3:])}'] = addr
    vals['sp'] = STACK_TOP
    vals['lr'] = random_u32(rng) | 1
    vals['pc'] = STREAM_BASE
    nzcv = rng.randrange(16)
    vals['xpsr'] = 0x0100_0000 | (nzcv << 28)
    vals['msp'] = STACK_TOP
    vals['psp'] = STACK_TOP
    vals['primask'] = 0
    vals['basepri'] = 0
    vals['faultmask'] = 0
    vals['control'] = 0
    for i in range(16):
        vals[f'd{i}'] = 0
    vals['fpscr'] = 0
    return vals


# ============================================================ 对拍主循环
def disasm(code: bytes, addr: int) -> str:
    if _MD is None:
        return code.hex()
    out = []
    for insn in _MD.disasm(code, addr):
        out.append(f'{insn.address:#x}: {insn.mnemonic} {insn.op_str}')
        return out[0]          # 只反汇编首条（生成粒度即单条）
    return code.hex()


def run_case(rsp: Rsp, rng, cls_name: str, n_instrs: int,
             verbose: bool) -> list:
    """执行一个用例（同类指令 n_instrs 条锁步），返回失配描述列表。

    注意：Unicorn 对每个用例重建实例——其 TB 翻译缓存不会因 mem_write 失效，
    跨用例复用会执行陈旧代码（已实测确认）。
    """
    uc = UcRef()
    gen = CLASSES[cls_name]
    code = b''
    overrides = {}
    while len(code) < STREAM_MAX - 8 and len(_count_hw(code)) < n_instrs:
        chunk, ov = gen(rng)
        code += chunk
        overrides.update(ov)

    vals = make_regs(rng, overrides)
    data_bytes = bytes(rng.getrandbits(8) for _ in range(DATA_SIZE))
    stack_bytes = bytes(rng.getrandbits(8) for _ in range(0x1000))
    # 分支类用 nop 垫后（分支可能跳入流后区域），其余类用随机字节作字面量池
    if cls_name in ('bcond', 'cbz'):
        lit_bytes = b'\x00\xbf' * 0x80
    else:
        lit_bytes = bytes(rng.getrandbits(8) for _ in range(0x100))

    # dtwin 侧装载
    rsp.write_g(pack_g(vals))
    rsp.write_mem(STREAM_BASE, code + lit_bytes[:STREAM_MAX - len(code)])
    rsp.write_mem(DATA_WIN, data_bytes)
    rsp.write_mem(STACK_WIN, stack_bytes)

    # Unicorn 侧装载
    uc.reset(vals)
    uc.load_mem(STREAM_BASE, code + lit_bytes[:STREAM_MAX - len(code)])
    uc.load_mem(DATA_WIN, data_bytes)
    uc.load_mem(STACK_WIN, stack_bytes)

    mismatches = []
    steps = len(_count_hw(code))
    for idx in range(steps):
        pc_before = unpack_g(rsp.read_g())['pc']
        stop = rsp.step()
        uc.step()

        d = unpack_g(rsp.read_g())
        u = uc.snapshot()

        diff = {}
        for i in range(13):
            n = f'r{i}'
            if d[n] != u[n]:
                diff[n] = (d[n], u[n])
        for n in ('sp', 'lr'):
            if d[n] != u[n]:
                diff[n] = (d[n], u[n])
        if d['pc'] != u['pc']:
            diff['pc'] = (d['pc'], u['pc'])
        if (d['xpsr'] & XPSR_MASK) != (u['xpsr'] & XPSR_MASK):
            diff['xpsr'] = (d['xpsr'] & XPSR_MASK, u['xpsr'] & XPSR_MASK)
        if d['fpscr'] != u['fpscr']:
            diff['fpscr'] = (d['fpscr'], u['fpscr'])

        # 内存写比对（以 Unicorn 写钩子为触集）
        # 窗口外写入 = 野性地址（基址被流内前序 load 污染时可能发生）：
        # unicorn 的 Flash 区可写不报错，dtwin/硬件会 fault——两者都算"拒绝"，
        # 记为一致；窗口外写入但 dtwin 未 fault 才算真失配。
        def in_window(a: int, size: int) -> bool:
            return (DATA_WIN <= a and a + size <= DATA_WIN + DATA_SIZE) or \
                   (STACK_WIN <= a and a + size <= STACK_END)
        wild = [(a, s, v) for a, s, v in uc.writes if not in_window(a, s)]
        mem_diff = []
        for addr, size, val in uc.writes:
            if not in_window(addr, size):
                continue
            try:
                dv = int.from_bytes(rsp.read_mem(addr, size), 'little')
            except RuntimeError as e:
                mem_diff.append((addr, size, f'读取失败({e})', val))
                continue
            if dv != val:
                mem_diff.append((addr, size, dv, val))
        uc.writes.clear()

        stop_sig = stop.decode(errors='replace')
        dtwin_fault = stop_sig not in ('S05', 'T05')
        uc_fault = uc.faulted is not None

        if dtwin_fault and (uc_fault or wild):
            # 双方均以各自方式拒绝了越界访问（fault 类型级比对留待专项）
            if verbose:
                print(f'    [{cls_name}#{idx}] pc={pc_before:#010x} 双方均拒绝，视为一致')
            continue
        if wild and not dtwin_fault:
            mem_diff.append((wild[0][0], wild[0][1], 'dtwin 未 fault', wild[0][2]))

        if diff or mem_diff or dtwin_fault or uc_fault:
            # 定位本条指令字节：pc_before 相对 STREAM_BASE
            off = pc_before - STREAM_BASE
            insn_bytes = code[off:off + 4] if 0 <= off < len(code) else b'?'
            mismatches.append({
                'class': cls_name, 'idx': idx, 'pc': pc_before,
                'bytes': insn_bytes, 'dis': disasm(insn_bytes, pc_before),
                'regs': diff, 'mem': mem_diff,
                'stop': stop_sig, 'uc_fault': uc.faulted,
                'seed_regs': vals, 'code': code,
            })
            break        # 状态已发散，本用例后续指令无意义
        if verbose:
            print(f'    [{cls_name}#{idx}] pc={pc_before:#010x} OK')
    return mismatches


def _count_hw(code: bytes) -> list:
    """粗略按 16 位单元计数（32 位指令占 2 个单元）。"""
    units = []
    i = 0
    while i + 1 < len(code):
        hw = struct.unpack_from('<H', code, i)[0]
        if (hw & 0xF800) in (0xE800, 0xF000, 0xF800) and (hw & 0xE000) == 0xE000:
            units.append(4)
            i += 4
        else:
            units.append(2)
            i += 2
    return units


def replay(rsp: Rsp, code_hex: str, reg_overrides: dict):
    """定向复现：单条/小段指令流 + 指定初始寄存器，逐步锁步并打印双方状态。"""
    code = bytes.fromhex(code_hex)
    vals = {}
    for i in range(13):
        vals[f'r{i}'] = 0
    vals['r5'] = DATA_WIN + 0x400
    vals['r6'] = DATA_WIN + 0x800
    vals.update(sp=STACK_TOP, lr=0xDEADBEEF, pc=STREAM_BASE, xpsr=0x0100_0000,
                msp=STACK_TOP, psp=STACK_TOP, primask=0, basepri=0,
                faultmask=0, control=0, fpscr=0)
    for i in range(16):
        vals[f'd{i}'] = 0
    vals.update(reg_overrides)

    pad = b'\x00\xbf' * ((STREAM_MAX - len(code)) // 2)
    rsp.write_g(pack_g(vals))
    rsp.write_mem(STREAM_BASE, code + pad)
    rsp.write_mem(DATA_WIN, bytes(DATA_SIZE))
    rsp.write_mem(STACK_WIN, bytes(0x1000))
    uc = UcRef()
    uc.reset(vals)
    uc.load_mem(STREAM_BASE, code + pad)
    uc.load_mem(DATA_WIN, bytes(DATA_SIZE))
    uc.load_mem(STACK_WIN, bytes(0x1000))

    print('反汇编:', disasm(code, STREAM_BASE))
    print(f'初始: { {k: hex(v) for k, v in vals.items() if not k.startswith("d")} }')
    steps = max(1, len(_count_hw(code)))
    for idx in range(steps):
        pc0 = unpack_g(rsp.read_g())['pc']
        stop = rsp.step().decode(errors='replace')
        uc.step()
        d = unpack_g(rsp.read_g())
        u = uc.snapshot()
        print(f'-- step {idx} pc={pc0:#010x} stop={stop} uc_fault={uc.faulted}')
        for i in range(13):
            n = f'r{i}'
            mark = ' <<' if d[n] != u[n] else ''
            print(f'   {n:4s} d={d[n]:#010x} u={u[n]:#010x}{mark}')
        for n in ('sp', 'lr', 'pc'):
            mark = ' <<' if d[n] != u[n] else ''
            print(f'   {n:4s} d={d[n]:#010x} u={u[n]:#010x}{mark}')
        mark = ' <<' if (d['xpsr'] & XPSR_MASK) != (u['xpsr'] & XPSR_MASK) else ''
        print(f'   xpsr d={d["xpsr"]:#010x} u={u["xpsr"]:#010x}{mark}')
        mark = ' <<' if d['fpscr'] != u['fpscr'] else ''
        print(f'   fpscr d={d["fpscr"]:#010x} u={u["fpscr"]:#010x}{mark}')
        if uc.writes:
            print('   unicorn 内存写:', [(hex(a), s, hex(v)) for a, s, v in uc.writes])
            uc.writes.clear()
        if stop not in ('S05', 'T05') or uc.faulted:
            break


def main() -> int:
    ap = argparse.ArgumentParser(description='dtwin vs Unicorn Thumb-2 指令级差分对拍')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=33333)
    ap.add_argument('--count', type=int, default=500, help='用例数（每用例 instrs 条指令）')
    ap.add_argument('--instrs', type=int, default=6, help='每用例指令数')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--classes', default='all',
                    help='逗号分隔的指令类，或 all / tierA / tierB')
    ap.add_argument('--max-mismatches', type=int, default=20)
    ap.add_argument('-v', '--verbose', action='store_true')
    ap.add_argument('--replay', metavar='HEX',
                    help='定向复现模式：指令流 hex，如 111d741f')
    ap.add_argument('--regs', default='', metavar='K=V,...',
                    help='replay 初始寄存器，如 r2=0x9a38ffca,xpsr=0xa1000000')
    args = ap.parse_args()

    if args.replay:
        overrides = {}
        for kv in args.regs.split(','):
            if not kv:
                continue
            k, v = kv.split('=')
            overrides[k.strip()] = int(v, 0)
        rsp = Rsp(args.host, args.port)
        rsp.handshake()
        replay(rsp, args.replay, overrides)
        rsp.close()
        return 0

    if args.classes == 'all':
        classes = sorted(CLASSES)
    elif args.classes == 'tierA':
        classes = [c for c in sorted(CLASSES) if not c.endswith('_w')
                   and c not in ('dp_modimm', 'movw_movt', 'addw_subw',
                                 'mul_mla', 'div', 'mull_long', 'bitfield',
                                 'sat', 'sat16', 'clz_rev_w', 'extend_rot', 'pkh',
                                 'shift_reg_w')]
    elif args.classes == 'tierB':
        classes = ['dp_modimm', 'movw_movt', 'addw_subw', 'ldst_w', 'shift_w',
                   'shift_reg_w', 'mul_mla', 'div', 'mull_long', 'bitfield',
                   'sat', 'sat16', 'clz_rev_w', 'extend_rot', 'pkh']
    else:
        classes = args.classes.split(',')
    for c in classes:
        if c not in CLASSES:
            ap.error(f'未知指令类: {c}（可选: {sorted(CLASSES)}）')

    rng = random.Random(args.seed)
    rsp = Rsp(args.host, args.port)
    rsp.handshake()

    total_instr = 0
    all_mm = []
    print(f'== 差分对拍: seed={args.seed} 用例={args.count}×{args.instrs} 类={len(classes)} ==')
    for case_no in range(args.count):
        cls = classes[rng.randrange(len(classes))]
        mm = run_case(rsp, rng, cls, args.instrs, args.verbose)
        total_instr += args.instrs
        if mm:
            all_mm.extend(mm)
            m = mm[0]
            print(f'\n[失配 #{len(all_mm)}] 类={m["class"]} 用例={case_no} '
                  f'指令序号={m["idx"]} pc={m["pc"]:#010x}')
            print(f'  编码: {m["bytes"].hex()}  反汇编: {m["dis"]}')
            if m['stop'] not in ('S05', 'T05'):
                print(f'  dtwin 异常停止: {m["stop"]}')
            if m['uc_fault']:
                print(f'  unicorn 异常: {m["uc_fault"]}')
            for name, (dv, uv) in sorted(m['regs'].items()):
                print(f'  {name:8s} dtwin={dv:#010x}  unicorn={uv:#010x}')
            for addr, size, dv, uv in m['mem']:
                dv_s = f'{dv:#x}' if isinstance(dv, int) else str(dv)
                print(f'  mem[{addr:#x}:{size}] dtwin={dv_s} unicorn={uv:#x}')
            print(f'  用例初始寄存器: {m["seed_regs"]}')
            print(f'  用例指令流: {m["code"].hex()}')
            if len(all_mm) >= args.max_mismatches:
                print(f'\n== 达到最大失配数 {args.max_mismatches}，提前终止 ==')
                break

    print(f'\n== 完成: {total_instr} 条指令, {len(all_mm)} 个失配 ==')
    rsp.close()
    return 1 if all_mm else 0


if __name__ == '__main__':
    sys.exit(main())
