//! 指令执行 — ARMv7E-M (Cortex-M4F) 执行器
//!
//! 基于 Decoder 输出的统一指令表示执行，更新 CPU 状态。
//! Phase 1: 核心整数指令（数据传送/算术逻辑/移位/分支/压栈）

use super::decode::{
    AccessWidth, Cond, DspShiftKind, FpArithOp, FpCvtOp, FpUnaryOp, Instruction, LoadStoreOffset,
    QAddKind, ShiftAmount, ShiftKind, SpecialReg,
};
use super::{dsp, fpu};
use crate::memory::{Memory, MemoryFault};
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
            Instruction::MsrMrs { rt, reg, read } => {
                if *read {
                    // MRS：特殊寄存器 → 核心寄存器
                    let v = match reg {
                        SpecialReg::Apsr => cpu.xpsr & 0xF800_0000,
                        SpecialReg::ApsrGe => cpu.xpsr & 0xF8FF_0000,
                        SpecialReg::Ipsr => cpu.xpsr & 0x1FF,
                        SpecialReg::Epsr => cpu.xpsr & 0x01FF_0000,
                        SpecialReg::Msp => cpu.msp,
                        SpecialReg::Psp => cpu.psp,
                        SpecialReg::Primask => cpu.primask as u32,
                        SpecialReg::Faultmask => cpu.faultmask as u32,
                        SpecialReg::Basepri => cpu.basepri as u32,
                        // BASEPRI_MAX 读为 UNPREDICTABLE（解码已拒绝，不可达）
                        SpecialReg::BasepriMax => cpu.basepri as u32,
                        SpecialReg::Control => cpu.control as u32,
                    };
                    cpu.regs[*rt as usize] = v;
                } else {
                    // MSR：核心寄存器 → 特殊寄存器
                    let v = cpu.regs[*rt as usize];
                    match reg {
                        SpecialReg::Apsr => {
                            cpu.xpsr = (cpu.xpsr & !0xF800_0000) | (v & 0xF800_0000)
                        }
                        SpecialReg::ApsrGe => {
                            cpu.xpsr = (cpu.xpsr & !0xF8FF_0000) | (v & 0xF8FF_0000)
                        }
                        // IPSR/EPSR 只读：写被忽略（ARM 语义 UNPREDICTABLE，保守忽略）
                        SpecialReg::Ipsr | SpecialReg::Epsr => {}
                        SpecialReg::Msp => cpu.msp = v,
                        SpecialReg::Psp => cpu.psp = v,
                        SpecialReg::Primask => cpu.primask = (v & 1) as u8,
                        SpecialReg::Faultmask => cpu.faultmask = (v & 1) as u8,
                        SpecialReg::Basepri => cpu.basepri = (v & 0xFF) as u8,
                        // BASEPRI_MAX：仅当新值更小（提高屏蔽）时生效 → min
                        SpecialReg::BasepriMax => {
                            cpu.basepri = cpu.basepri.min((v & 0xFF) as u8)
                        }
                        SpecialReg::Control => cpu.control = (v & 0x3) as u8,
                    }
                }
                ExecOutcome::Continue
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
            Instruction::Simd8 {
                rd,
                rn,
                rm,
                unsigned,
                halving,
                sub,
            } => {
                let a = cpu.regs[*rn as usize];
                let b = cpu.regs[*rm as usize];
                let (result, ge) = dsp::simd8(a, b, *unsigned, *halving, *sub);
                cpu.regs[*rd as usize] = result;
                // GE[3:0] 更新（bits[19:16]）
                cpu.xpsr = (cpu.xpsr & !(0xF << 16)) | (((ge as u32) & 0xF) << 16);
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

            // ================= Phase 4: FPU =================
            Instruction::FpVmovReg { sd, sm, double } => {
                let fpu = &mut cpu.fpu;
                if *double {
                    let v = fpu.read_d(*sm as usize);
                    fpu.write_d(*sd as usize, v);
                } else {
                    let v = fpu.read_s(*sm as usize);
                    fpu.write_s(*sd as usize, v);
                }
                ExecOutcome::Continue
            }
            Instruction::FpVmovCore { rt, sn, to_core } => {
                if *to_core {
                    cpu.regs[*rt as usize] = cpu.fpu.read_s(*sn as usize);
                } else {
                    cpu.fpu.write_s(*sn as usize, cpu.regs[*rt as usize]);
                }
                ExecOutcome::Continue
            }
            Instruction::FpVmovCoreD {
                rt,
                rt2,
                dn,
                to_core,
            } => {
                if *to_core {
                    let v = cpu.fpu.read_d(*dn as usize);
                    cpu.regs[*rt as usize] = v as u32;
                    cpu.regs[*rt2 as usize] = (v >> 32) as u32;
                } else {
                    let v =
                        (cpu.regs[*rt as usize] as u64) | ((cpu.regs[*rt2 as usize] as u64) << 32);
                    cpu.fpu.write_d(*dn as usize, v);
                }
                ExecOutcome::Continue
            }
            Instruction::FpVmovImm { sd, imm, double } => {
                if *double {
                    cpu.fpu.write_d(*sd as usize, *imm);
                } else {
                    cpu.fpu.write_s(*sd as usize, *imm as u32);
                }
                ExecOutcome::Continue
            }
            Instruction::FpArith3 {
                op,
                vd,
                vn,
                vm,
                double,
            } => {
                let fpu = &mut cpu.fpu;
                if *double {
                    let a = fpu.read_d(*vn as usize);
                    let b = fpu.read_d(*vm as usize);
                    let c = fpu.read_d(*vd as usize);
                    let (res, flags) = match op {
                        FpArithOp::Vadd => fpu::f64_add(fpu, a, b),
                        FpArithOp::Vsub => fpu::f64_sub(fpu, a, b),
                        FpArithOp::Vmul => fpu::f64_mul(fpu, a, b),
                        FpArithOp::Vnmul => {
                            let (r, f) = fpu::f64_mul(fpu, a, b);
                            (r ^ (1 << 63), f)
                        }
                        FpArithOp::Vdiv => fpu::f64_div(fpu, a, b),
                        FpArithOp::Vmla => fpu::f64_mul_add(fpu, a, b, c, false, false),
                        FpArithOp::Vmls => fpu::f64_mul_add(fpu, a, b, c, true, false),
                        FpArithOp::Vnmls => fpu::f64_mul_add(fpu, a, b, c, true, true),
                        FpArithOp::Vnmla => fpu::f64_mul_add(fpu, a, b, c, false, true),
                    };
                    self.apply_fpu_flags(fpu, &flags);
                    fpu.write_d(*vd as usize, res);
                } else {
                    let a = fpu.read_s(*vn as usize);
                    let b = fpu.read_s(*vm as usize);
                    let c = fpu.read_s(*vd as usize);
                    let (res, flags) = match op {
                        FpArithOp::Vadd => fpu::f32_add(fpu, a, b),
                        FpArithOp::Vsub => fpu::f32_sub(fpu, a, b),
                        FpArithOp::Vmul => fpu::f32_mul(fpu, a, b),
                        FpArithOp::Vnmul => {
                            let (r, f) = fpu::f32_mul(fpu, a, b);
                            (r ^ (1 << 31), f)
                        }
                        FpArithOp::Vdiv => fpu::f32_div(fpu, a, b),
                        FpArithOp::Vmla => fpu::f32_mul_add(fpu, a, b, c, false, false),
                        FpArithOp::Vmls => fpu::f32_mul_add(fpu, a, b, c, true, false),
                        FpArithOp::Vnmls => fpu::f32_mul_add(fpu, a, b, c, true, true),
                        FpArithOp::Vnmla => fpu::f32_mul_add(fpu, a, b, c, false, true),
                    };
                    self.apply_fpu_flags(fpu, &flags);
                    fpu.write_s(*vd as usize, res);
                }
                ExecOutcome::Continue
            }
            Instruction::FpUnary { op, vd, vm, double } => {
                let fpu = &mut cpu.fpu;
                if *double {
                    let v = fpu.read_d(*vm as usize);
                    match op {
                        FpUnaryOp::Vabs => fpu.write_d(*vd as usize, v & !(1 << 63)),
                        FpUnaryOp::Vneg => fpu.write_d(*vd as usize, v ^ (1 << 63)),
                        FpUnaryOp::Vsqrt => {
                            let f = f64::from_bits(v);
                            if f.is_nan() {
                                // NaN 传播（安静化），无异常
                                fpu.write_d(*vd as usize, v | 0x0008_0000_0000_0000);
                            } else if f < 0.0 {
                                // 负数开方 → 默认 NaN + IOC
                                self.apply_fpu_flags(
                                    fpu,
                                    &fpu::FpOpFlags {
                                        ioc: true,
                                        ..Default::default()
                                    },
                                );
                                fpu.write_d(*vd as usize, fpu::DEFAULT_NAN_F64);
                            } else {
                                let r = f.sqrt();
                                let mut flags = fpu::FpOpFlags::default();
                                if fpu::is_denormal_f64(r.to_bits()) {
                                    flags.ufc = true;
                                    flags.ixc = true;
                                }
                                self.apply_fpu_flags(fpu, &flags);
                                fpu.write_d(*vd as usize, r.to_bits());
                            }
                        }
                    }
                } else {
                    let v = fpu.read_s(*vm as usize);
                    match op {
                        FpUnaryOp::Vabs => fpu.write_s(*vd as usize, v & !(1 << 31)),
                        FpUnaryOp::Vneg => fpu.write_s(*vd as usize, v ^ (1 << 31)),
                        FpUnaryOp::Vsqrt => {
                            let f = f32::from_bits(v);
                            if f.is_nan() {
                                fpu.write_s(*vd as usize, fpu::quiet_nan(v));
                            } else if f < 0.0 {
                                self.apply_fpu_flags(
                                    fpu,
                                    &fpu::FpOpFlags {
                                        ioc: true,
                                        ..Default::default()
                                    },
                                );
                                fpu.write_s(*vd as usize, fpu::DEFAULT_NAN_F32);
                            } else {
                                let r = f.sqrt();
                                let mut flags = fpu::FpOpFlags::default();
                                if fpu::is_denormal_f32(r.to_bits()) {
                                    flags.ufc = true;
                                    flags.ixc = true;
                                }
                                self.apply_fpu_flags(fpu, &flags);
                                fpu.write_s(*vd as usize, r.to_bits());
                            }
                        }
                    }
                }
                ExecOutcome::Continue
            }
            Instruction::FpCmp {
                vd,
                vm,
                double,
                e,
                zero,
            } => {
                let fpu = &mut cpu.fpu;
                if *double {
                    let a = fpu.read_d(*vd as usize);
                    let b = if *zero { 0 } else { fpu.read_d(*vm as usize) };
                    let (n, z, c, v, ioc) = self.fpu_compare_f64(a, b, *e);
                    fpu.set_nzcv(n, z, c, v);
                    if ioc {
                        self.apply_fpu_flags(
                            fpu,
                            &fpu::FpOpFlags {
                                ioc: true,
                                ..Default::default()
                            },
                        );
                    }
                } else {
                    let a = fpu.read_s(*vd as usize);
                    let b = if *zero { 0 } else { fpu.read_s(*vm as usize) };
                    let (n, z, c, v, ioc) = self.fpu_compare_f32(a, b, *e);
                    fpu.set_nzcv(n, z, c, v);
                    if ioc {
                        self.apply_fpu_flags(
                            fpu,
                            &fpu::FpOpFlags {
                                ioc: true,
                                ..Default::default()
                            },
                        );
                    }
                }
                ExecOutcome::Continue
            }
            Instruction::FpCvt { op, vd, vm } => {
                let fpu = &mut cpu.fpu;
                match op {
                    FpCvtOp::S32ToF32 => {
                        let x = fpu.read_s(*vm as usize) as i32 as i64;
                        let r = fpu::cvt_int_to_f32(fpu, x);
                        fpu.write_s(*vd as usize, r.to_bits());
                    }
                    FpCvtOp::U32ToF32 => {
                        let x = fpu.read_s(*vm as usize) as i64;
                        let r = fpu::cvt_int_to_f32(fpu, x);
                        fpu.write_s(*vd as usize, r.to_bits());
                    }
                    FpCvtOp::F32ToS32 => {
                        let (r, flags) =
                            fpu::cvt_f32_to_int(fpu, fpu.read_s(*vm as usize), true, false);
                        self.apply_fpu_flags(fpu, &flags);
                        fpu.write_s(*vd as usize, r);
                    }
                    FpCvtOp::F32ToU32 => {
                        let (r, flags) =
                            fpu::cvt_f32_to_int(fpu, fpu.read_s(*vm as usize), false, false);
                        self.apply_fpu_flags(fpu, &flags);
                        fpu.write_s(*vd as usize, r);
                    }
                    FpCvtOp::F32ToS32R => {
                        let (r, flags) =
                            fpu::cvt_f32_to_int(fpu, fpu.read_s(*vm as usize), true, true);
                        self.apply_fpu_flags(fpu, &flags);
                        fpu.write_s(*vd as usize, r);
                    }
                    FpCvtOp::F32ToU32R => {
                        let (r, flags) =
                            fpu::cvt_f32_to_int(fpu, fpu.read_s(*vm as usize), false, true);
                        self.apply_fpu_flags(fpu, &flags);
                        fpu.write_s(*vd as usize, r);
                    }
                    FpCvtOp::F32ToF64 => {
                        // VCVT.F32.F64 Sd, Dm：源 Dm（f64），目标 Sd（f32）
                        let v = fpu.read_d(*vm as usize);
                        let (r, flags) = fpu::cvt_f64_to_f32(fpu, v);
                        self.apply_fpu_flags(fpu, &flags);
                        fpu.write_s(*vd as usize, r);
                    }
                    FpCvtOp::F64ToF32 => {
                        // VCVT.F64.F32 Dd, Sm：源 Sm（f32），目标 Dd（f64，精确）
                        let v = fpu.read_s(*vm as usize);
                        let r = (f32::from_bits(v) as f64).to_bits();
                        fpu.write_d(*vd as usize, r);
                    }
                    FpCvtOp::S32ToF64 => {
                        let x = fpu.read_s(*vm as usize) as i32 as i64;
                        let r = fpu::cvt_int_to_f64(x);
                        fpu.write_d(*vd as usize, r.to_bits());
                    }
                    FpCvtOp::U32ToF64 => {
                        let x = fpu.read_s(*vm as usize) as i64;
                        let r = fpu::cvt_int_to_f64(x);
                        fpu.write_d(*vd as usize, r.to_bits());
                    }
                    FpCvtOp::F64ToS32 => {
                        let (r, flags) =
                            fpu::cvt_f64_to_int(fpu, fpu.read_d(*vm as usize), true, false);
                        self.apply_fpu_flags(fpu, &flags);
                        fpu.write_s(*vd as usize, r);
                    }
                    FpCvtOp::F64ToU32 => {
                        let (r, flags) =
                            fpu::cvt_f64_to_int(fpu, fpu.read_d(*vm as usize), false, false);
                        self.apply_fpu_flags(fpu, &flags);
                        fpu.write_s(*vd as usize, r);
                    }
                    FpCvtOp::F64ToS32R => {
                        let (r, flags) =
                            fpu::cvt_f64_to_int(fpu, fpu.read_d(*vm as usize), true, true);
                        self.apply_fpu_flags(fpu, &flags);
                        fpu.write_s(*vd as usize, r);
                    }
                    FpCvtOp::F64ToU32R => {
                        let (r, flags) =
                            fpu::cvt_f64_to_int(fpu, fpu.read_d(*vm as usize), false, true);
                        self.apply_fpu_flags(fpu, &flags);
                        fpu.write_s(*vd as usize, r);
                    }
                }
                ExecOutcome::Continue
            }
            Instruction::FpLoadStore {
                rt,
                rn,
                offset,
                load,
                double,
            } => {
                let base = cpu.regs[*rn as usize];
                let addr = base.wrapping_add(*offset);
                let fpu = &mut cpu.fpu;
                // 对齐：单精度 4 字节，双精度 8 字节
                let align = if *double { 7 } else { 3 };
                if addr & align != 0 {
                    return ExecOutcome::Fault {
                        reason: super::FaultReason::UnalignedAccess { address: addr },
                    };
                }
                if *double {
                    if *load {
                        let lo = match memory.read_u32(addr) {
                            Ok(v) => v,
                            Err(e) => {
                                return ExecOutcome::Fault {
                                    reason: self.map_mem_fault(e),
                                }
                            }
                        };
                        let hi = match memory.read_u32(addr + 4) {
                            Ok(v) => v,
                            Err(e) => {
                                return ExecOutcome::Fault {
                                    reason: self.map_mem_fault(e),
                                }
                            }
                        };
                        fpu.write_d(*rt as usize, (lo as u64) | ((hi as u64) << 32));
                    } else {
                        let v = fpu.read_d(*rt as usize);
                        if let Err(e) = memory.write_u32(addr, v as u32) {
                            return ExecOutcome::Fault {
                                reason: self.map_mem_fault(e),
                            };
                        }
                        if let Err(e) = memory.write_u32(addr + 4, (v >> 32) as u32) {
                            return ExecOutcome::Fault {
                                reason: self.map_mem_fault(e),
                            };
                        }
                    }
                } else if *load {
                    match memory.read_u32(addr) {
                        Ok(v) => fpu.write_s(*rt as usize, v),
                        Err(e) => {
                            return ExecOutcome::Fault {
                                reason: self.map_mem_fault(e),
                            }
                        }
                    }
                } else {
                    match memory.write_u32(addr, fpu.read_s(*rt as usize)) {
                        Ok(()) => {}
                        Err(e) => {
                            return ExecOutcome::Fault {
                                reason: self.map_mem_fault(e),
                            }
                        }
                    }
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

    /// 汇总 FPU 累积异常标志到 FPSCR
    fn apply_fpu_flags(&self, fpu: &mut super::fpu::FpuRegisters, flags: &super::fpu::FpOpFlags) {
        fpu.set_cumulative(
            flags.ioc, flags.dzc, flags.ofc, flags.ufc, flags.ixc, flags.idc,
        );
        if flags.qc {
            fpu.set_qc();
        }
    }

    /// f32 比较（VCMP/VCMPE）：返回 (N, Z, C, V, IOC)
    fn fpu_compare_f32(&self, a: u32, b: u32, e: bool) -> (bool, bool, bool, bool, bool) {
        let (af, bf) = (f32::from_bits(a), f32::from_bits(b));
        if af.is_nan() || bf.is_nan() {
            // 无序：C=1, V=1；VCMPE 或信号 NaN → IOC
            let snan = fpu::is_signaling_nan_f32(a) || fpu::is_signaling_nan_f32(b);
            return (false, false, true, true, e || snan);
        }
        if af == bf {
            (false, true, true, false, false)
        } else if af < bf {
            (true, false, false, false, false)
        } else {
            (false, false, true, false, false)
        }
    }

    /// f64 比较
    fn fpu_compare_f64(&self, a: u64, b: u64, e: bool) -> (bool, bool, bool, bool, bool) {
        let (af, bf) = (f64::from_bits(a), f64::from_bits(b));
        if af.is_nan() || bf.is_nan() {
            let snan = fpu::is_signaling_nan_f64(a) || fpu::is_signaling_nan_f64(b);
            return (false, false, true, true, e || snan);
        }
        if af == bf {
            (false, true, true, false, false)
        } else if af < bf {
            (true, false, false, false, false)
        } else {
            (false, false, true, false, false)
        }
    }

    /// 内存故障 → 引擎故障
    fn map_mem_fault(&self, f: MemoryFault) -> super::FaultReason {
        match f {
            MemoryFault::UnalignedAccess { address } => {
                super::FaultReason::UnalignedAccess { address }
            }
            MemoryFault::MemManage { address } => super::FaultReason::MemManage { address },
            MemoryFault::BusFault { address } => super::FaultReason::BusFault { address },
            MemoryFault::ReadOnlyWrite { address } => super::FaultReason::MemManage { address },
        }
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

    // ============ P1-补：16-bit LDR/STR golden（编码经 arm-none-eabi-as 实测） ============

    use crate::engine::decode::Decoder;

    /// 全链路：16-bit STR/LDR word 立即数（0x60C8 STR / 0x695A LDR）
    #[test]
    fn golden_16bit_ldr_str_word_imm() {
        let (mut ex, mut cpu, mut mem) = setup();
        let mut dec = Decoder::new();
        cpu.regs[1] = 0x2000_0000;
        cpu.regs[0] = 0x1122_3344;
        // STR r0, [r1, #12]（0x60C8）
        let instr = dec.decode_halfword(0x60C8, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(mem.read_u32(0x2000_000C).unwrap(), 0x1122_3344);
        // LDR r2, [r3, #20]（0x695A：rn=r3，偏移 20）
        cpu.regs[3] = 0x2000_0000;
        mem.write_u32(0x2000_0014, 0x1122_3344).unwrap();
        let instr = dec.decode_halfword(0x695A, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[2], 0x1122_3344);
    }

    /// 全链路：16-bit STRB/LDRB（0x71EC / 0x78FE）与 STRH/LDRH（0x80C8 / 0x895A）
    #[test]
    fn golden_16bit_ldr_str_byte_half() {
        let (mut ex, mut cpu, mut mem) = setup();
        let mut dec = Decoder::new();
        cpu.regs[5] = 0x2000_0000;
        cpu.regs[4] = 0xDEAD_BEEF;
        // STRB r4, [r5, #7]（0x71EC）→ 只写低字节
        let instr = dec.decode_halfword(0x71EC, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(mem.read_u8(0x2000_0007).unwrap(), 0xEF);
        // LDRB r6, [r7, #3]（0x78FE）→ 零扩展
        cpu.regs[7] = 0x2000_0000;
        mem.write_u8(0x2000_0003, 0xEF).unwrap();
        let instr = dec.decode_halfword(0x78FE, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[6], 0xEF);
        // STRH r0, [r1, #6]（0x80C8）
        cpu.regs[1] = 0x2000_0000;
        cpu.regs[0] = 0xABCD_1234;
        let instr = dec.decode_halfword(0x80C8, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(mem.read_u16(0x2000_0006).unwrap(), 0x1234);
        // LDRH r2, [r3, #10]（0x895A）
        cpu.regs[3] = 0x2000_0000;
        mem.write_u16(0x2000_000A, 0x5A5A).unwrap();
        let instr = dec.decode_halfword(0x895A, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[2], 0x5A5A);
    }

    /// 全链路：16-bit STR/LDR 寄存器偏移（0x5088/0x5963）与 SP 相对（0x9001/0x9801）
    #[test]
    fn golden_16bit_ldr_str_reg_sp() {
        let (mut ex, mut cpu, mut mem) = setup();
        let mut dec = Decoder::new();
        cpu.regs[1] = 0x2000_0000;
        cpu.regs[2] = 0x10;
        cpu.regs[0] = 0xCAFE_BABE;
        // STR r0, [r1, r2]（0x5088）→ [0x2000_0010]
        let instr = dec.decode_halfword(0x5088, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(mem.read_u32(0x2000_0010).unwrap(), 0xCAFE_BABE);
        // LDR r3, [r4, r5]（0x5963）
        cpu.regs[4] = 0x2000_0000;
        cpu.regs[5] = 0x10;
        let instr = dec.decode_halfword(0x5963, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[3], 0xCAFE_BABE);
        // SP 相对：STR r0, [sp, #4]（0x9001）
        cpu.regs[13] = 0x2000_0100;
        let instr = dec.decode_halfword(0x9001, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(mem.read_u32(0x2000_0104).unwrap(), 0xCAFE_BABE);
        // LDR r0, [sp, #4]（0x9801）
        let instr = dec.decode_halfword(0x9801, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0xCAFE_BABE);
    }

    /// 全链路：16-bit STMIA/LDMIA（0xC006 / 0xC806）
    #[test]
    fn golden_16bit_stmia_ldmia() {
        let (mut ex, mut cpu, mut mem) = setup();
        let mut dec = Decoder::new();
        cpu.regs[0] = 0x2000_0000;
        cpu.regs[1] = 0xAA;
        cpu.regs[2] = 0xBB;
        // STMIA r0!, {r1, r2}（0xC006）
        let instr = dec.decode_halfword(0xC006, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0x2000_0008); // 回写
        assert_eq!(mem.read_u32(0x2000_0000).unwrap(), 0xAA);
        assert_eq!(mem.read_u32(0x2000_0004).unwrap(), 0xBB);
        // LDMIA r0!, {r1, r2}（0xC806）
        cpu.regs[0] = 0x2000_0000;
        let instr = dec.decode_halfword(0xC806, 0);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[1], 0xAA);
        assert_eq!(cpu.regs[2], 0xBB);
        assert_eq!(cpu.regs[0], 0x2000_0008);
    }

    /// 全链路：16-bit 条件分支 B<cond>（0xD006 BEQ / 0xD106 BNE）
    #[test]
    fn golden_16bit_cond_branch() {
        let (mut ex, mut cpu, mut mem) = setup();
        let mut dec = Decoder::new();
        // Z=0 → BEQ 不跳
        cpu.regs[15] = 0x1000;
        let instr = dec.decode_halfword(0xD006, 0x1000);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        // 置 Z=1 → BEQ 跳转
        cpu.xpsr |= 1 << 30;
        let instr = dec.decode_halfword(0xD006, 0x1000);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Branch {
                target: 0x1000 + 4 + 12,
            }
        );
        // BNE：Z=1 → 不跳
        let instr = dec.decode_halfword(0xD106, 0x1000);
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
    }

    // ============ P2-补：MRS/MSR golden（编码经 arm-none-eabi-as 实测） ============

    /// MSR PRIMASK → MRS 读回；MSR APSR → MRS APSR 读回标志位
    #[test]
    fn golden_msr_mrs_roundtrip() {
        let mut h = crate::engine::test_util::Harness::new();
        // GIVEN: R2 = 1（置 PRIMASK）
        h.cpu.regs[2] = 1;
        // WHEN: MSR PRIMASK, r2（0xF382 8810：Rn=r2，SYSm=0x10）
        assert_eq!(h.exec_word(0xF382_8810), ExecOutcome::Continue);
        // THEN: cpu.primask = 1
        assert_eq!(h.cpu.primask, 1);
        // WHEN: MRS r3, PRIMASK（0xF3EF 8310：Rd=r3，SYSm=0x10）
        assert_eq!(h.exec_word(0xF3EF_8310), ExecOutcome::Continue);
        // THEN: R3 = 1
        assert_eq!(h.cpu.regs[3], 1);
    }

    /// MRS APSR 读回 NZCVQ 标志；MSR APSR 写入标志位
    #[test]
    fn golden_msr_mrs_apsr() {
        let mut h = crate::engine::test_util::Harness::new();
        // GIVEN: xPSR 置 N=1, Z=1
        h.cpu.xpsr = (1 << 31) | (1 << 30);
        // WHEN: MRS r0, APSR（0xF3EF 8000）
        assert_eq!(h.exec_word(0xF3EF_8000), ExecOutcome::Continue);
        // THEN: R0 = 0xC000_0000（NZCVQ 位）
        assert_eq!(h.cpu.regs[0], 0xC000_0000);
        // WHEN: MSR APSR_nzcvq, r1（0xF381 8800，R1 = 0x8000_0000 → 置 N）
        h.cpu.regs[1] = 0x8000_0000;
        h.cpu.xpsr = 0;
        assert_eq!(h.exec_word(0xF381_8800), ExecOutcome::Continue);
        // THEN: xPSR N 置位，低位不受影响
        assert_eq!(h.cpu.xpsr, 0x8000_0000);
        // MRS APSR_nzcvqgt（0xF3EF 8002）读回
        h.cpu.xpsr = (1 << 27) | (0b1010 << 16); // Q + GE=1010
        assert_eq!(h.exec_word(0xF3EF_8002), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], (1 << 27) | (0b1010 << 16));
    }

    /// MSR CONTROL / MSP / PSP / FAULTMASK / BASEPRI_MAX 语义
    #[test]
    fn golden_msr_special_regs() {
        let mut h = crate::engine::test_util::Harness::new();
        // MSR CONTROL, r6（0xF386 8814，R6 = 0x3）
        h.cpu.regs[6] = 0x3;
        assert_eq!(h.exec_word(0xF386_8814), ExecOutcome::Continue);
        assert_eq!(h.cpu.control, 0x3);
        // MSR MSP, r7（0xF387 8808，R7 = 0x2000_1000）
        h.cpu.regs[7] = 0x2000_1000;
        assert_eq!(h.exec_word(0xF387_8808), ExecOutcome::Continue);
        assert_eq!(h.cpu.msp, 0x2000_1000);
        // MRS r0, MSP（0xF3EF 8008）
        assert_eq!(h.exec_word(0xF3EF_8008), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x2000_1000);
        // MSR FAULTMASK, r5（0xF385 8813，R5 = 0x1）
        h.cpu.regs[5] = 1;
        assert_eq!(h.exec_word(0xF385_8813), ExecOutcome::Continue);
        assert_eq!(h.cpu.faultmask, 1);
        // MRS r4, FAULTMASK（0xF3EF 8413）
        assert_eq!(h.exec_word(0xF3EF_8413), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[4], 1);
    }

    /// BASEPRI_MAX：只降不升（提高屏蔽），min 语义
    #[test]
    fn golden_basepri_max_semantics() {
        let mut h = crate::engine::test_util::Harness::new();
        // GIVEN: BASEPRI = 0x60（屏蔽较少）
        h.cpu.basepri = 0x60;
        // WHEN: MSR BASEPRI_MAX, r4（0xF384 8812，R4 = 0x50 → 更小 → 生效）
        h.cpu.regs[4] = 0x50;
        assert_eq!(h.exec_word(0xF384_8812), ExecOutcome::Continue);
        // THEN: BASEPRI = 0x50
        assert_eq!(h.cpu.basepri, 0x50);
        // WHEN: 再次写 0x70（更大 → 忽略，不解除屏蔽）
        h.cpu.regs[4] = 0x70;
        assert_eq!(h.exec_word(0xF384_8812), ExecOutcome::Continue);
        // THEN: BASEPRI 保持 0x50
        assert_eq!(h.cpu.basepri, 0x50);
    }

    /// 未知 SYSm → 诚实 Unimplemented 故障（不假装实现）
    #[test]
    fn golden_mrs_unknown_sysm_faults() {
        let mut h = crate::engine::test_util::Harness::new();
        // WHEN: MRS r0, #0x99（0xF3EF 8099）
        let out = h.exec_word(0xF3EF_8099);
        // THEN: UnimplementedInstr 故障
        assert_eq!(
            out,
            ExecOutcome::Fault {
                reason: crate::engine::FaultReason::UnimplementedInstr,
            }
        );
    }
}
