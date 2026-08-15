//! 指令执行 — ARMv7E-M (Cortex-M4F) 执行器
//!
//! 基于 Decoder 输出的统一指令表示执行，更新 CPU 状态。
//! Phase 1: 核心整数指令（数据传送/算术逻辑/移位/分支/压栈）

use super::decode::{
    AccessWidth, Cond, DspShiftKind, Instruction, LoadStoreOffset, QAddKind, ShiftAmount, ShiftKind,
};
use super::dsp;
use crate::memory::Memory;
use crate::CpuState;

/// 执行结果
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecOutcome {
    /// 正常执行下一条
    Continue,
    /// 分支跳转
    Branch { target: u32 },
    /// 异常返回（BX LR 特殊形式）
    ExceptionReturn,
    /// 触发硬件异常
    Fault { reason: super::FaultReason },
}

/// 指令执行器
#[derive(Debug, Default)]
pub struct Executor {
    /// 已执行指令数
    pub executed_count: u64,
    /// 周期计数（模拟时钟）
    pub cycle_count: u64,
}

impl Executor {
    pub fn new() -> Self {
        Self::default()
    }

    /// 执行一条指令，返回下一步行为
    pub fn execute(
        &mut self,
        cpu: &mut CpuState,
        memory: &mut Memory,
        instr: &Instruction,
    ) -> ExecOutcome {
        self.executed_count += 1;
        self.cycle_count += 1;
        match instr {
            Instruction::Nop => ExecOutcome::Continue,
            Instruction::Mov { rd, rm, imm } => {
                let val = match imm {
                    Some(v) => *v,
                    None => cpu.regs[*rm as usize],
                };
                cpu.regs[*rd as usize] = val;
                ExecOutcome::Continue
            }
            Instruction::MovImm32 { rd, imm16, top } => {
                let val = *imm16 as u32;
                if *top {
                    cpu.regs[*rd as usize] = (cpu.regs[*rd as usize] & 0xFFFF) | (val << 16);
                } else {
                    cpu.regs[*rd as usize] = (cpu.regs[*rd as usize] & 0xFFFF0000) | val;
                }
                ExecOutcome::Continue
            }
            Instruction::Add {
                rd,
                rn,
                rm,
                imm,
                flags,
            } => {
                let a = cpu.regs[*rn as usize];
                let b = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let (result, carry) = a.overflowing_add(b);
                if *flags {
                    self.update_flags_add(cpu, a, b, result, carry);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Sub {
                rd,
                rn,
                rm,
                imm,
                flags,
            } => {
                let a = cpu.regs[*rn as usize];
                let b = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let (result, borrow) = a.overflowing_sub(b);
                if *flags {
                    self.update_flags_sub(cpu, a, b, result, borrow);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::And { rd, rn, rm, flags } => {
                let result = cpu.regs[*rn as usize] & cpu.regs[*rm as usize];
                if *flags {
                    self.update_flags_logical(cpu, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Orr { rd, rn, rm, flags } => {
                let result = cpu.regs[*rn as usize] | cpu.regs[*rm as usize];
                if *flags {
                    self.update_flags_logical(cpu, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Eor { rd, rn, rm, flags } => {
                let result = cpu.regs[*rn as usize] ^ cpu.regs[*rm as usize];
                if *flags {
                    self.update_flags_logical(cpu, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Bic { rd, rn, rm, flags } => {
                let result = cpu.regs[*rn as usize] & !cpu.regs[*rm as usize];
                if *flags {
                    self.update_flags_logical(cpu, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Mul { rd, rn, rm, flags } => {
                let result = cpu.regs[*rn as usize].wrapping_mul(cpu.regs[*rm as usize]);
                if *flags {
                    self.update_flags_logical(cpu, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Udiv { rd, rn, rm } => {
                let divisor = cpu.regs[*rm as usize];
                cpu.regs[*rd as usize] = if divisor == 0 {
                    0
                } else {
                    cpu.regs[*rn as usize] / divisor
                };
                ExecOutcome::Continue
            }
            Instruction::Sdiv { rd, rn, rm } => {
                let divisor = cpu.regs[*rm as usize] as i32;
                cpu.regs[*rd as usize] = if divisor == 0 {
                    0
                } else {
                    (cpu.regs[*rn as usize] as i32 / divisor) as u32
                };
                ExecOutcome::Continue
            }
            Instruction::Shift {
                rd,
                rm,
                kind,
                amount,
                flags,
            } => {
                let val = cpu.regs[*rm as usize];
                let result = match amount {
                    ShiftAmount::Immediate(n) => self.shift_val(val, *kind, *n),
                    ShiftAmount::Register(r) => {
                        self.shift_val(val, *kind, (cpu.regs[*r as usize] & 0xFF) as u8)
                    }
                };
                if *flags {
                    self.update_flags_logical(cpu, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Cmp { rn, rm, imm } => {
                let a = cpu.regs[*rn as usize];
                let b = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let (result, borrow) = a.overflowing_sub(b);
                self.update_flags_sub(cpu, a, b, result, borrow);
                ExecOutcome::Continue
            }
            Instruction::Cmn { rn, rm } => {
                let a = cpu.regs[*rn as usize];
                let b = cpu.regs[*rm as usize];
                let (result, carry) = a.overflowing_add(b);
                self.update_flags_add(cpu, a, b, result, carry);
                ExecOutcome::Continue
            }
            Instruction::Tst { rn, rm } => {
                let result = cpu.regs[*rn as usize] & cpu.regs[*rm as usize];
                self.update_flags_logical(cpu, result);
                ExecOutcome::Continue
            }
            Instruction::Branch { cond, target } => {
                if let Some(c) = cond {
                    if self.cond_holds(cpu, *c) {
                        ExecOutcome::Branch { target: *target }
                    } else {
                        ExecOutcome::Continue
                    }
                } else {
                    ExecOutcome::Branch { target: *target }
                }
            }
            Instruction::BranchLink { target } => {
                cpu.regs[14] = cpu.regs[15] - 1; // LR = PC | 1 (Thumb)
                ExecOutcome::Branch { target: *target }
            }
            Instruction::BranchExchange { rm } => {
                let target = cpu.regs[*rm as usize];
                if target & 1 == 0 {
                    ExecOutcome::Fault {
                        reason: super::FaultReason::UsageFault { address: target },
                    }
                } else {
                    ExecOutcome::Branch {
                        target: target & !1,
                    }
                }
            }
            Instruction::BranchLinkExchange { rm } => {
                let target = cpu.regs[*rm as usize];
                cpu.regs[14] = cpu.regs[15] - 1;
                if target & 1 == 0 {
                    ExecOutcome::Fault {
                        reason: super::FaultReason::UsageFault { address: target },
                    }
                } else {
                    ExecOutcome::Branch {
                        target: target & !1,
                    }
                }
            }
            Instruction::CompareBranch { rn, target, zero } => {
                let val = cpu.regs[*rn as usize];
                if (val == 0) == *zero {
                    ExecOutcome::Branch { target: *target }
                } else {
                    ExecOutcome::Continue
                }
            }
            Instruction::TableBranch { .. } => ExecOutcome::Fault {
                reason: super::FaultReason::UnimplementedInstr,
            },
            Instruction::Push { regs, lr } => {
                // PUSH {reglist} — 递减 SP 并压栈
                let mut sp = cpu.regs[13];
                let mut count = 0u32;
                for i in 0..8 {
                    if regs & (1 << i) != 0 {
                        count += 1;
                    }
                }
                if *lr {
                    count += 1;
                }
                sp = sp.wrapping_sub(count * 4);
                let mut addr = sp;
                for i in 0..8 {
                    if regs & (1 << i) != 0 {
                        let val = cpu.regs[i];
                        if let Err(f) = memory.write_u32(addr, val) {
                            return ExecOutcome::Fault {
                                reason: super::FaultReason::MemManage { address: addr },
                            };
                        }
                        addr += 4;
                    }
                }
                if *lr {
                    let val = cpu.regs[14];
                    if let Err(_f) = memory.write_u32(addr, val) {
                        return ExecOutcome::Fault {
                            reason: super::FaultReason::MemManage { address: addr },
                        };
                    }
                }
                cpu.regs[13] = sp;
                ExecOutcome::Continue
            }
            Instruction::Pop { regs, pc } => {
                // POP {reglist} — 出栈并递增 SP
                let mut sp = cpu.regs[13];
                let mut addr = sp;
                for i in 0..8 {
                    if regs & (1 << i) != 0 {
                        let val = match memory.read_u32(addr) {
                            Ok(v) => v,
                            Err(_f) => {
                                return ExecOutcome::Fault {
                                    reason: super::FaultReason::BusFault { address: addr },
                                }
                            }
                        };
                        cpu.regs[i] = val;
                        addr += 4;
                    }
                }
                let mut count = 0u32;
                for i in 0..8 {
                    if regs & (1 << i) != 0 {
                        count += 1;
                    }
                }
                if *pc {
                    let val = match memory.read_u32(addr) {
                        Ok(v) => v,
                        Err(_f) => {
                            return ExecOutcome::Fault {
                                reason: super::FaultReason::BusFault { address: addr },
                            }
                        }
                    };
                    cpu.regs[15] = val & !1; // 清 Thumb 位
                    count += 1;
                }
                cpu.regs[13] = sp.wrapping_add(count * 4);
                ExecOutcome::Continue
            }
            Instruction::Ldm {
                rn,
                regs,
                writeback,
            } => {
                let mut addr = cpu.regs[*rn as usize];
                let mut last = 0u32;
                for i in 0..16 {
                    if regs & (1 << i) != 0 {
                        let val = match memory.read_u32(addr) {
                            Ok(v) => v,
                            Err(_f) => {
                                return ExecOutcome::Fault {
                                    reason: super::FaultReason::BusFault { address: addr },
                                }
                            }
                        };
                        cpu.regs[i] = val;
                        addr += 4;
                        last = addr;
                    }
                }
                if *writeback {
                    cpu.regs[*rn as usize] = last;
                }
                ExecOutcome::Continue
            }
            Instruction::Stm {
                rn,
                regs,
                writeback,
            } => {
                let base = cpu.regs[*rn as usize];
                let mut addr = base;
                for i in 0..16 {
                    if regs & (1 << i) != 0 {
                        let val = cpu.regs[i];
                        if let Err(_f) = memory.write_u32(addr, val) {
                            return ExecOutcome::Fault {
                                reason: super::FaultReason::MemManage { address: addr },
                            };
                        }
                        addr += 4;
                    }
                }
                if *writeback {
                    cpu.regs[*rn as usize] = addr;
                }
                ExecOutcome::Continue
            }
            Instruction::Ldr {
                rt,
                rn,
                offset,
                width,
            } => {
                let base = cpu.regs[*rn as usize];
                let addr = match offset {
                    LoadStoreOffset::Immediate(imm) => base.wrapping_add(*imm),
                    LoadStoreOffset::Register(rm) => base.wrapping_add(cpu.regs[*rm as usize]),
                };
                let val = match width {
                    AccessWidth::Byte => memory.read_u8(addr).map(|v| v as u32),
                    AccessWidth::HalfWord => memory.read_u16(addr).map(|v| v as u32),
                    AccessWidth::Word => memory.read_u32(addr),
                };
                match val {
                    Ok(v) => {
                        cpu.regs[*rt as usize] = v;
                        ExecOutcome::Continue
                    }
                    Err(_f) => ExecOutcome::Fault {
                        reason: super::FaultReason::BusFault { address: addr },
                    },
                }
            }
            Instruction::Str {
                rt,
                rn,
                offset,
                width,
            } => {
                let base = cpu.regs[*rn as usize];
                let addr = match offset {
                    LoadStoreOffset::Immediate(imm) => base.wrapping_add(*imm),
                    LoadStoreOffset::Register(rm) => base.wrapping_add(cpu.regs[*rm as usize]),
                };
                let val = cpu.regs[*rt as usize];
                let result = match width {
                    AccessWidth::Byte => memory.write_u8(addr, val as u8),
                    AccessWidth::HalfWord => memory.write_u16(addr, val as u16),
                    AccessWidth::Word => memory.write_u32(addr, val),
                };
                match result {
                    Ok(()) => ExecOutcome::Continue,
                    Err(_f) => ExecOutcome::Fault {
                        reason: super::FaultReason::MemManage { address: addr },
                    },
                }
            }
            Instruction::LdrLiteral { rt, imm } => {
                let addr = cpu.regs[15].wrapping_add(*imm) & !3;
                match memory.read_u32(addr) {
                    Ok(v) => {
                        cpu.regs[*rt as usize] = v;
                        ExecOutcome::Continue
                    }
                    Err(_f) => ExecOutcome::Fault {
                        reason: super::FaultReason::BusFault { address: addr },
                    },
                }
            }
            Instruction::MsrMrs { .. } => {
                // MRS/MSR 尚未在 decode 中实现（32-bit Thumb-2 0xF3EF/0xF380），保持诚实 Unimplemented
                ExecOutcome::Fault {
                    reason: super::FaultReason::UnimplementedInstr,
                }
            }
            Instruction::Svc { imm8 } => {
                // SVC → SVCall 异常（异常号 11）
                let _ = imm8;
                ExecOutcome::Fault {
                    reason: super::FaultReason::UnimplementedInstr,
                } // 异常入口由上层调度
            }
            Instruction::ExceptionReturn => ExecOutcome::ExceptionReturn,

            // ================= Phase 3: DSP =================
            Instruction::Sat {
                rd,
                rn,
                sat_imm,
                signed,
                shift_kind,
                shift_imm,
            } => {
                let t = cpu.regs[*rn as usize];
                let shifted = match shift_kind {
                    DspShiftKind::Lsl => {
                        let n = *shift_imm & 0x1F;
                        if n == 0 {
                            t
                        } else {
                            t << n
                        }
                    }
                    DspShiftKind::Asr => {
                        let n = *shift_imm & 0x1F;
                        if *signed {
                            // SSAT: ASR（n=0 → 移 32 位，符号填充）
                            if n == 0 {
                                ((t as i32) >> 31) as u32
                            } else {
                                ((t as i32) >> n) as u32
                            }
                        } else {
                            // USAT: LSR（n=0 → 移 32 位 → 0）
                            if n == 0 {
                                0
                            } else {
                                t >> n
                            }
                        }
                    }
                };
                let (result, sat) = if *signed {
                    let r = dsp::ssat(shifted as i32, *sat_imm as u32);
                    (r as u32, r != shifted as i32)
                } else {
                    let r = dsp::usat(shifted as i32, *sat_imm as u32);
                    (r, r != shifted)
                };
                if sat {
                    self.set_q(cpu);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::QAddSub { rd, rn, rm, kind } => {
                let a = cpu.regs[*rm as usize] as i32;
                let b = cpu.regs[*rn as usize] as i32;
                let (result, sat) = match kind {
                    QAddKind::Qadd => dsp::qadd_q(a, b),
                    QAddKind::Qsub => dsp::qsub_q(a, b),
                    QAddKind::Qdadd => dsp::qdadd_q(a, b),
                    QAddKind::Qdsub => dsp::qdsub_q(a, b),
                };
                if sat {
                    self.set_q(cpu);
                }
                cpu.regs[*rd as usize] = result as u32;
                ExecOutcome::Continue
            }
            Instruction::Simd16 {
                rd,
                rn,
                rm,
                kind,
                unsigned,
            } => {
                let a = cpu.regs[*rn as usize];
                let b = cpu.regs[*rm as usize];
                let (result, ge) = dsp::simd16(a, b, *kind, *unsigned);
                cpu.regs[*rd as usize] = result;
                // GE[1:0] 更新（GE[3:2] 不变）
                cpu.xpsr = (cpu.xpsr & !(0x3 << 16)) | (((ge as u32) & 0x3) << 16);
                ExecOutcome::Continue
            }
            Instruction::DualHalfMul {
                rd,
                rn,
                rm,
                swap,
                sub,
            } => {
                let (bl, bh) = dsp::dual_half_operands(cpu.regs[*rm as usize], *swap);
                let al = cpu.regs[*rn as usize] as i16 as i32;
                let ah = (cpu.regs[*rn as usize] >> 16) as i16 as i32;
                let sum = if *sub {
                    al * bl - ah * bh
                } else {
                    al * bl + ah * bh
                };
                cpu.regs[*rd as usize] = sum as u32;
                ExecOutcome::Continue
            }
            Instruction::DualHalfMulAcc {
                rd,
                rn,
                rm,
                ra,
                swap,
                sub,
            } => {
                let (bl, bh) = dsp::dual_half_operands(cpu.regs[*rm as usize], *swap);
                let al = cpu.regs[*rn as usize] as i16 as i32;
                let ah = (cpu.regs[*rn as usize] >> 16) as i16 as i32;
                let sum = if *sub {
                    al * bl - ah * bh
                } else {
                    al * bl + ah * bh
                };
                cpu.regs[*rd as usize] = cpu.regs[*ra as usize].wrapping_add(sum as u32);
                ExecOutcome::Continue
            }
            Instruction::DualHalfMulLong {
                rdlo,
                rdhi,
                rn,
                rm,
                swap,
                sub,
            } => {
                let (bl, bh) = dsp::dual_half_operands(cpu.regs[*rm as usize], *swap);
                let al = cpu.regs[*rn as usize] as i16 as i64;
                let ah = (cpu.regs[*rn as usize] >> 16) as i16 as i64;
                let sum = if *sub {
                    al * bl as i64 - ah * bh as i64
                } else {
                    al * bl as i64 + ah * bh as i64
                };
                let acc =
                    ((cpu.regs[*rdhi as usize] as u64) << 32) | cpu.regs[*rdlo as usize] as u64;
                let result = acc.wrapping_add(sum as u64);
                cpu.regs[*rdlo as usize] = result as u32;
                cpu.regs[*rdhi as usize] = (result >> 32) as u32;
                ExecOutcome::Continue
            }
            Instruction::Mla {
                rd,
                rn,
                rm,
                ra,
                sub,
            } => {
                let prod = cpu.regs[*rn as usize].wrapping_mul(cpu.regs[*rm as usize]);
                cpu.regs[*rd as usize] = if *sub {
                    cpu.regs[*ra as usize].wrapping_sub(prod)
                } else {
                    cpu.regs[*ra as usize].wrapping_add(prod)
                };
                ExecOutcome::Continue
            }
            Instruction::Pkh {
                rd,
                rn,
                rm,
                tb,
                shift_imm,
            } => {
                let rn_val = cpu.regs[*rn as usize];
                let rm_val = cpu.regs[*rm as usize];
                let n = *shift_imm & 0x1F;
                if *tb {
                    // PKHTB: 高半字取 Rn，低半字取 ASR(Rm, n)
                    let shifted = if n == 0 {
                        ((rm_val as i32) >> 31) as u32
                    } else {
                        ((rm_val as i32) >> n) as u32
                    };
                    cpu.regs[*rd as usize] = (rn_val & 0xFFFF_0000) | (shifted & 0xFFFF);
                } else {
                    // PKHBT: 低半字取 Rn，高半字取 LSL(Rm, n)
                    let shifted = if n == 0 { rm_val } else { rm_val << n };
                    cpu.regs[*rd as usize] = (rn_val & 0xFFFF) | (shifted & 0xFFFF_0000);
                }
                ExecOutcome::Continue
            }

            Instruction::Unimplemented { .. } => ExecOutcome::Fault {
                reason: super::FaultReason::UnimplementedInstr,
            },
            Instruction::Invalid { address } => ExecOutcome::Fault {
                reason: super::FaultReason::IllegalInstruction { pc: *address },
            },
        }
    }

    /// 置位 DSP Q 标志（APSR bit27，粘性）
    fn set_q(&self, cpu: &mut CpuState) {
        cpu.xpsr |= 1 << 27;
    }

    /// 移位计算
    fn shift_val(&self, val: u32, kind: ShiftKind, n: u8) -> u32 {
        let n = n & 0x1F;
        match kind {
            ShiftKind::Lsl => val.wrapping_shl(n as u32),
            ShiftKind::Lsr => val.wrapping_shr(n as u32),
            ShiftKind::Asr => ((val as i32) >> n) as u32,
            ShiftKind::Ror => val.rotate_right(n as u32),
            ShiftKind::Rrx => (val >> 1) | ((self.carry_bit() as u32) << 31),
        }
    }

    /// 逻辑操作更新标志（N/Z，C/V 由调用方处理）
    fn update_flags_logical(&self, cpu: &mut CpuState, result: u32) {
        // APSR N/Z 位（bit31/bit30）
        if result & 0x8000_0000 != 0 {
            cpu.xpsr |= 1 << 31; // N
        } else {
            cpu.xpsr &= !(1 << 31);
        }
        if result == 0 {
            cpu.xpsr |= 1 << 30; // Z
        } else {
            cpu.xpsr &= !(1 << 30);
        }
    }

    /// 加法更新标志
    fn update_flags_add(&self, cpu: &mut CpuState, a: u32, b: u32, result: u32, carry: bool) {
        self.update_flags_logical(cpu, result);
        // C = carry out
        if carry {
            cpu.xpsr |= 1 << 29;
        } else {
            cpu.xpsr &= !(1 << 29);
        }
        // V = overflow (signed)
        let v = ((a ^ result) & (b ^ result) & 0x8000_0000) != 0;
        if v {
            cpu.xpsr |= 1 << 28;
        } else {
            cpu.xpsr &= !(1 << 28);
        }
    }

    /// 减法更新标志
    fn update_flags_sub(&self, cpu: &mut CpuState, a: u32, b: u32, result: u32, borrow: bool) {
        self.update_flags_logical(cpu, result);
        // C = NOT borrow
        if !borrow {
            cpu.xpsr |= 1 << 29;
        } else {
            cpu.xpsr &= !(1 << 29);
        }
        let v = ((a ^ b) & (a ^ result) & 0x8000_0000) != 0;
        if v {
            cpu.xpsr |= 1 << 28;
        } else {
            cpu.xpsr &= !(1 << 28);
        }
    }

    /// 读取进位标志
    fn carry_bit(&self) -> bool {
        false // 简化：RRX 的进位由 exec 状态管理，Phase 1 先置 false
    }

    /// 条件码评估
    fn cond_holds(&self, cpu: &CpuState, cond: Cond) -> bool {
        let n = cpu.xpsr & (1 << 31) != 0;
        let z = cpu.xpsr & (1 << 30) != 0;
        let c = cpu.xpsr & (1 << 29) != 0;
        let v = cpu.xpsr & (1 << 28) != 0;
        match cond {
            Cond::Eq => z,
            Cond::Ne => !z,
            Cond::Cs => c,
            Cond::Cc => !c,
            Cond::Mi => n,
            Cond::Pl => !n,
            Cond::Vs => v,
            Cond::Vc => !v,
            Cond::Hi => c && !z,
            Cond::Ls => !c || z,
            Cond::Ge => n == v,
            Cond::Lt => n != v,
            Cond::Gt => !z && (n == v),
            Cond::Le => z || (n != v),
            Cond::Al => true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::decode::{AccessWidth, LoadStoreOffset};
    use crate::memory::Memory;

    fn setup() -> (Executor, CpuState, Memory) {
        (Executor::new(), CpuState::default(), Memory::test_ram())
    }

    #[test]
    fn ldr_str_word_roundtrip() {
        let (mut ex, mut cpu, mut mem) = setup();
        // R0 = 0x2000_0000 (SRAM)，R1 = 0x1234_5678
        cpu.regs[0] = 0x2000_0000;
        cpu.regs[1] = 0x1234_5678;
        // STR R1, [R0]
        let instr = Instruction::Str {
            rt: 1,
            rn: 0,
            offset: LoadStoreOffset::Immediate(0),
            width: AccessWidth::Word,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        // LDR R2, [R0]
        let instr = Instruction::Ldr {
            rt: 2,
            rn: 0,
            offset: LoadStoreOffset::Immediate(0),
            width: AccessWidth::Word,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[2], 0x1234_5678);
    }

    #[test]
    fn ldr_byte() {
        let (mut ex, mut cpu, mut mem) = setup();
        cpu.regs[0] = 0x2000_0000;
        mem.write_u8(0x2000_0004, 0xAB).unwrap();
        let instr = Instruction::Ldr {
            rt: 3,
            rn: 0,
            offset: LoadStoreOffset::Immediate(4),
            width: AccessWidth::Byte,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[3], 0xAB);
    }

    #[test]
    fn push_pop_roundtrip() {
        let (mut ex, mut cpu, mut mem) = setup();
        cpu.regs[13] = 0x2000_1000; // SP
        cpu.regs[0] = 0x1111_1111;
        cpu.regs[1] = 0x2222_2222;
        cpu.regs[14] = 0x0800_0001; // LR
                                    // PUSH {R0, R1, LR}
        let instr = Instruction::Push {
            regs: 0b11,
            lr: true,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[13], 0x2000_0FF4); // SP -= 12
                                               // POP {R0, R1, PC}
        cpu.regs[0] = 0;
        cpu.regs[1] = 0;
        let instr = Instruction::Pop {
            regs: 0b11,
            pc: true,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0x1111_1111);
        assert_eq!(cpu.regs[1], 0x2222_2222);
        assert_eq!(cpu.regs[15], 0x0800_0000); // PC 清 Thumb 位
        assert_eq!(cpu.regs[13], 0x2000_1000);
    }

    #[test]
    fn ldm_stm_writeback() {
        let (mut ex, mut cpu, mut mem) = setup();
        cpu.regs[0] = 0x2000_0000;
        cpu.regs[1] = 0xAA;
        cpu.regs[2] = 0xBB;
        // STM R0!, {R1, R2}
        let instr = Instruction::Stm {
            rn: 0,
            regs: 0b110,
            writeback: true,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0x2000_0008); // writeback
                                              // LDM R0!, {R3, R4}
        cpu.regs[0] = 0x2000_0000;
        let instr = Instruction::Ldm {
            rn: 0,
            regs: 0b11000,
            writeback: true,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[3], 0xAA);
        assert_eq!(cpu.regs[4], 0xBB);
        assert_eq!(cpu.regs[0], 0x2000_0008);
    }
}
