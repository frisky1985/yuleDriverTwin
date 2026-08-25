//! 指令执行 — ARMv7E-M (Cortex-M4F) 执行器
//!
//! 基于 Decoder 输出的统一指令表示执行，更新 CPU 状态。
//! Phase 1: 核心整数指令（数据传送/算术逻辑/移位/分支/压栈）

use super::decode::{
    AccessWidth, BitFieldKind, Cond, DspShiftKind, FpArithOp, FpCvtOp, FpUnaryOp,
    Instruction, LoadStoreOffset, QAddKind, ExtendKind, RevKind, ShiftAmount, ShiftKind, SpecialReg,
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
    /// 异常返回（BX EXC_RETURN 特殊形式，携带 EXC_RETURN 值）
    ExceptionReturn { exc_return: u32 },
    /// 触发硬件异常入口（SVC → 异常 11；引擎负责压栈/跳向量）
    /// return_pc：同步异常（SVC）现场帧的 PC 槽 = 下一条指令地址（PC+宽度）
    TakeException { number: u8, return_pc: u32 },
    /// IT 块内条件不成立：本指令被跳过（PC 仍正常前进）
    Skipped,
    /// 调试事件（BKPT 触发）
    DebugEvent,
    /// 触发硬件异常
    Fault { reason: super::FaultReason },
}

/// 指令执行器
#[derive(Debug)]
pub struct Executor {
    /// 已执行指令数
    pub executed_count: u64,
    /// 周期计数（模拟时钟）
    pub cycle_count: u64,
    /// IT 块剩余条件执行指令数（0 = 不在 IT 块内）
    it_remaining: u8,
    /// IT 块总指令数（1-4）
    it_block_len: u8,
    /// IT firstcond（首条指令条件）
    it_firstcond: Cond,
    /// IT mask（bits[3:0]：后续指令条件 bit0 的来源）
    it_mask: u8,
    /// 本指令是否处于 IT 块内（已被 IT 条件门控）
    it_was_active: bool,
    /// 当前指令是否为 16 位编码（IT 块内隐式 S 指令抑制标志更新，ARMv7-M B1.5.10）
    pub cur_is_16bit: bool,
    /// 本指令是否应抑制标志更新（IT 块内 16 位隐式 S 指令）
    it_suppress_flags: bool,
}

impl Default for Executor {
    fn default() -> Self {
        Self {
            executed_count: 0,
            cycle_count: 0,
            it_remaining: 0,
            it_block_len: 0,
            it_firstcond: Cond::Al,
            it_mask: 0,
            it_was_active: false,
            cur_is_16bit: false,
            it_suppress_flags: false,
        }
    }
}

/// 是否为 FPU（VFP）指令（CPACR 门控用，P5-补）
fn is_fpu_instr(instr: &Instruction) -> bool {
    matches!(
        instr,
        Instruction::FpVmovReg { .. }
            | Instruction::FpVmovCore { .. }
            | Instruction::FpVmovCoreD { .. }
            | Instruction::FpVmovImm { .. }
            | Instruction::FpArith3 { .. }
            | Instruction::FpUnary { .. }
            | Instruction::FpCmp { .. }
            | Instruction::FpCvt { .. }
            | Instruction::FpCvtFixed { .. }
            | Instruction::FpLoadStore { .. }
            | Instruction::FpLoadStoreMulti { .. }
    )
}

impl Executor {
    pub fn new() -> Self {
        Self::default()
    }

    /// 是否处于 IT 块内（供引擎/调试器查询）
    pub fn it_active(&self) -> bool {
        self.it_remaining > 0
    }

    /// 当前 IT 条件（首条指令 = firstcond；后续指令 = firstcond 的 bits[3:1]
    /// 拼上对应 mask 位作 bit0 —— QEMU 实测：ITE NE（mask 0100）→ NE,EQ；
    /// ITEE NE（0010）→ NE,EQ,EQ；ITEEE EQ（1111）→ EQ,NE,NE,NE）
    pub fn it_cond(&self) -> Cond {
        self.it_cond_at(0)
    }

    /// 块内第 k 条指令（0 起）的条件
    fn it_cond_at(&self, k: u8) -> Cond {
        if k == 0 {
            self.it_firstcond
        } else {
            let base = self.it_firstcond.to_bits() & 0xE;
            let bit0 = (self.it_mask >> (4 - k)) & 1;
            Cond::from_bits(base | bit0).unwrap_or(Cond::Al)
        }
    }

    /// 清除 IT 状态（异常入口/调试事件时架构要求 ITSTATE 清零）
    pub fn clear_it(&mut self) {
        self.it_remaining = 0;
        self.it_block_len = 0;
        self.it_firstcond = Cond::Al;
        self.it_mask = 0;
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
        // ---- IT 块条件门控（ITSTATE 状态机）----
        // 在 IT 块内的每条指令先按 IT 条件判断：不成立 → Skipped（PC 由引擎 +width）；
        // 条件 = firstcond 的 bits[3:1] 拼 mask 对应位作 bit0（ARM 实测语义，非增量翻转）；
        // 跳过的指令同样推进 ITSTATE。
        self.it_was_active = false;
        self.it_suppress_flags = false;
        if self.it_remaining > 0 {
            let k = self.it_block_len - self.it_remaining; // 当前块内序号（0 起）
            let holds = self.cond_holds(cpu, self.it_cond_at(k));
            self.it_remaining -= 1;
            if !holds {
                return ExecOutcome::Skipped;
            }
            self.it_was_active = true;
            // ARMv7-M B1.5.10：IT 块内 16 位隐式 S 指令（MOVS/ADDS/SUBS/ANDS/
            // ORRS/EORS/BICS/MVNS/ASRS/LSRS/LSLS/RORS/NEGS）执行时不更新条件标志，
            // 32 位显式 S 指令正常更新。编译器依赖此语义生成 ite 模式
            // （例：ite eq; moveq r5,#1; movne r5,#0）。
            self.it_suppress_flags = self.cur_is_16bit && Self::is_implicit_s_16bit(instr);
        }
        // FPU 门控（CPACR，P5-补）：CP10/CP11 未使能时浮点指令 → NOCP UsageFault
        if !cpu.fpu_enabled() && is_fpu_instr(instr) {
            return ExecOutcome::Fault {
                reason: super::FaultReason::UsageFault {
                    address: cpu.regs[15],
                },
            };
        }
        // FPU 上下文活跃跟踪（FRT-EXC-09）：任务执行 VFP 指令 → CONTROL.FPCA=1
        // （bit2，引擎内部约定；硬件 lazy 压栈的 eager 等价），异常入口据此压扩展帧
        if is_fpu_instr(instr) {
            cpu.control |= 4;
        }
        match instr {
            Instruction::Nop => ExecOutcome::Continue,
            Instruction::Mov { rd, rm, imm, flags } => {
                let val = match imm {
                    Some(v) => *v,
                    None => cpu.regs[*rm as usize],
                };
                if *flags {
                    // 16 位 MOVS 形式：更新 N/Z
                    self.update_flags_logical(cpu, val);
                }
                cpu.regs[*rd as usize] = val;
                ExecOutcome::Continue
            }
            Instruction::MovImm32 { rd, imm16, top } => {
                let val = *imm16 as u32;
                if *top {
                    // MOVT：保留低半字，写高半字
                    cpu.regs[*rd as usize] = (cpu.regs[*rd as usize] & 0xFFFF) | (val << 16);
                } else {
                    // MOVW：ZeroExtend(imm16,32)——高半字清零（P1-5 修复）
                    cpu.regs[*rd as usize] = val;
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
                let a = if *rn == 15 {
                    // ADR 语义：Rn=PC 时基址 = Align(PC+4, 4)
                    (cpu.regs[15].wrapping_add(4)) & !3
                } else {
                    cpu.regs[*rn as usize]
                };
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
                if *rd == 13 {
                    self.sync_sp(cpu);
                }
                ExecOutcome::Continue
            }
            // ADD Rd, Rn, Rm, LSL#n（32 位寄存器形式带移位）
            Instruction::AddShifted {
                rd,
                rn,
                rm,
                lsl,
                flags,
            } => {
                let a = if *rn == 15 {
                    (cpu.regs[15].wrapping_add(4)) & !3
                } else {
                    cpu.regs[*rn as usize]
                };
                let b = cpu.regs[*rm as usize].wrapping_shl(*lsl as u32);
                let (result, carry) = a.overflowing_add(b);
                if *flags {
                    self.update_flags_add(cpu, a, b, result, carry);
                }
                cpu.regs[*rd as usize] = result;
                if *rd == 13 {
                    self.sync_sp(cpu);
                }
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
                if *rd == 13 {
                    self.sync_sp(cpu);
                }
                ExecOutcome::Continue
            }
            // SUB Rd, Rn, Rm, LSL#n（32 位寄存器形式带移位）
            Instruction::SubShifted {
                rd,
                rn,
                rm,
                lsl,
                flags,
            } => {
                let a = cpu.regs[*rn as usize];
                let b = cpu.regs[*rm as usize].wrapping_shl(*lsl as u32);
                let (result, borrow) = a.overflowing_sub(b);
                if *flags {
                    self.update_flags_sub(cpu, a, b, result, borrow);
                }
                cpu.regs[*rd as usize] = result;
                if *rd == 13 {
                    self.sync_sp(cpu);
                }
                ExecOutcome::Continue
            }
            Instruction::And {
                rd,
                rn,
                rm,
                imm,
                flags,
            } => {
                let b = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let result = cpu.regs[*rn as usize] & b;
                if *flags {
                    self.update_flags_logical(cpu, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Orr {
                rd,
                rn,
                rm,
                imm,
                flags,
            } => {
                let b = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let result = cpu.regs[*rn as usize] | b;
                if *flags {
                    self.update_flags_logical(cpu, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Eor {
                rd,
                rn,
                rm,
                imm,
                flags,
            } => {
                let b = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let result = cpu.regs[*rn as usize] ^ b;
                if *flags {
                    self.update_flags_logical(cpu, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Bic {
                rd,
                rn,
                rm,
                imm,
                flags,
            } => {
                let b = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let result = cpu.regs[*rn as usize] & !b;
                if *flags {
                    self.update_flags_logical(cpu, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Adc {
                rd,
                rn,
                rm,
                flags,
            } => {
                let a = cpu.regs[*rn as usize];
                let b = cpu.regs[*rm as usize];
                let cin = (cpu.xpsr >> 29) & 1; // APSR.C
                // 33 位扩展精度：ext = a + b + C（精确值 0..2^33）
                let ext = (a as u64) + (b as u64) + (cin as u64);
                let result = (ext & 0xFFFF_FFFF) as u32;
                if *flags {
                    // C = 进位输出（bit32）；V = 进位进符号位(bit31) XOR 进位出符号位(bit32)
                    self.update_flags_add3(cpu, a, b, ext, result);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Sbc {
                rd,
                rn,
                rm,
                flags,
            } => {
                let a = cpu.regs[*rn as usize];
                let b = cpu.regs[*rm as usize];
                let cin = (cpu.xpsr >> 29) & 1; // APSR.C
                let notc = 1 - cin; // SBC = a - b - NOT(C)
                // 扩展精度有符号结果（a/b 符号扩展到 i64，V 判定需符号语义）
                let ext = (a as i32 as i64) - (b as i32 as i64) - (notc as i64);
                let result = (ext & 0xFFFF_FFFF) as u32;
                if *flags {
                    self.update_flags_logical(cpu, result);
                    // P3-1（codex 检视）：Sbc 与 Shift 同病——IT 块内 16 位隐式 S 指令
                    // 不更新任何标志（B1.5.10），C/V 更新须入守卫（update_flags_logical
                    // 内部已有守卫，但 C/V 此前在守卫外直接更新）。
                    if !self.it_suppress_flags {
                        // C = 无借位（无符号比较 a >= b + NOT(C)）
                        if (a as u64) >= (b as u64) + (notc as u64) {
                            cpu.xpsr |= 1 << 29;
                        } else {
                            cpu.xpsr &= !(1 << 29);
                        }
                        // V = 有符号结果超出 i32 范围
                        if ext > i32::MAX as i64 || ext < i32::MIN as i64 {
                            cpu.xpsr |= 1 << 28;
                        } else {
                            cpu.xpsr &= !(1 << 28);
                        }
                    }
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Neg { rd, rn, flags } => {
                let b = cpu.regs[*rn as usize];
                let (result, borrow) = 0u32.overflowing_sub(b);
                if *flags {
                    self.update_flags_sub(cpu, 0, b, result, borrow);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            // 反向减法: RSB Rd, Rn, #imm/Rm（Rd = imm - Rn 或 Rm - Rn）
            Instruction::Rsb {
                rd,
                rn,
                rm,
                imm,
                flags,
            } => {
                let lhs = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let rhs = cpu.regs[*rn as usize];
                let (result, borrow) = lhs.overflowing_sub(rhs);
                if *flags {
                    self.update_flags_sub(cpu, lhs, rhs, result, borrow);
                }
                cpu.regs[*rd as usize] = result;
                ExecOutcome::Continue
            }
            Instruction::Mvn {
                rd,
                rm,
                imm,
                flags,
            } => {
                let src = match imm {
                    Some(v) => *v,
                    None => cpu.regs[*rm as usize],
                };
                let result = !src;
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
            // UMULL/SMULL/SMLAL（FRT-INS-05 SHOULD）：64 位长乘法
            // UMULL： [RdHi:RdLo] = Rn × Rm（无符号）；SMULL 同（有符号）；
            // SMLAL： 累加 [RdHi:RdLo] += Rn × Rm（ARMv7-M 不更新 flags）
            Instruction::MullLong {
                rdlo,
                rdhi,
                rn,
                rm,
                signed,
                accumulate,
            } => {
                let a = cpu.regs[*rn as usize];
                let b = cpu.regs[*rm as usize];
                let product: u64 = if *signed {
                    (((a as i32) as i64) * ((b as i32) as i64)) as u64
                } else {
                    (a as u64) * (b as u64)
                };
                let mut result = product;
                if *accumulate {
                    let acc = ((cpu.regs[*rdhi as usize] as u64) << 32)
                        | (cpu.regs[*rdlo as usize] as u64);
                    result = result.wrapping_add(acc);
                }
                cpu.regs[*rdlo as usize] = result as u32;
                cpu.regs[*rdhi as usize] = (result >> 32) as u32;
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
                    ShiftAmount::Immediate(n) => self.shift_val(cpu, val, *kind, *n),
                    ShiftAmount::Register(r) => {
                        self.shift_val(cpu, val, *kind, (cpu.regs[*r as usize] & 0xFF) as u8)
                    }
                };
                if *flags {
                    self.update_flags_logical(cpu, result);
                    // P3-1（codex 检视）：IT 块内 16 位隐式 S 移位指令不更新任何标志（ARMv7-M
                    // B1.5.10）——C 更新同样须入守卫，否则 C 标志泄漏（旧实现只抑制 N/Z）。
                    if !self.it_suppress_flags {
                        // 移位指令更新 C 标志 = 最后移出位（P1-6 修复）
                        let c = match amount {
                            ShiftAmount::Immediate(n) => self.shift_carry(val, *kind, *n),
                            ShiftAmount::Register(r) => {
                                self.shift_carry(val, *kind, (cpu.regs[*r as usize] & 0xFF) as u8)
                            }
                        };
                        if c {
                            cpu.xpsr |= 1 << 29;
                        } else {
                            cpu.xpsr &= !(1 << 29);
                        }
                    }
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
            Instruction::Cmn { rn, rm, imm } => {
                let a = cpu.regs[*rn as usize];
                let b = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let (result, carry) = a.overflowing_add(b);
                self.update_flags_add(cpu, a, b, result, carry);
                ExecOutcome::Continue
            }
            Instruction::Tst { rn, rm, imm } => {
                let a = cpu.regs[*rn as usize];
                let b = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let result = a & b;
                self.update_flags_logical(cpu, result);
                ExecOutcome::Continue
            }
            Instruction::Teq { rn, rm, imm } => {
                let a = cpu.regs[*rn as usize];
                let b = match (rm, imm) {
                    (Some(r), _) => cpu.regs[*r as usize],
                    (_, Some(v)) => *v,
                    _ => 0,
                };
                let result = a ^ b;
                self.update_flags_logical(cpu, result);
                ExecOutcome::Continue
            }
            Instruction::Branch { cond, target } => {
                // IT 块内：分支自身条件被 IT 条件替代（ARMv7-M：B<cond> 在 IT 块内
                // 忽略自身 cond，用 IT 条件；门控已通过 → 直接分支）
                if let Some(c) = cond {
                    if self.it_was_active || self.cond_holds(cpu, *c) {
                        ExecOutcome::Branch { target: *target }
                    } else {
                        ExecOutcome::Continue
                    }
                } else {
                    ExecOutcome::Branch { target: *target }
                }
            }
            Instruction::BranchLink { target } => {
                // LR = 下一条指令地址 | Thumb 位（PC 尚未递增，BL 恒为 32 位）
                cpu.regs[14] = cpu.regs[15].wrapping_add(4) | 1;
                ExecOutcome::Branch { target: *target }
            }
            Instruction::BranchLinkExchangeImm { target } => {
                // LR = 下一条指令地址 | Thumb 位（PC 尚未递增，BLX 恒为 32 位）
                cpu.regs[14] = cpu.regs[15].wrapping_add(4) | 1;
                ExecOutcome::Branch { target: *target }
            }
            Instruction::BranchExchange { rm } => {
                let target = cpu.regs[*rm as usize];
                // EXC_RETURN 特殊值（0xFFFFFFF1/9/D + FPU 变体 E1/9/D）：异常返回
                // （FRT-EXC-02）——由引擎弹栈恢复现场；非 EXC_RETURN 保持普通分支语义
                if crate::nvic::ExcReturn::from_value(target) != crate::nvic::ExcReturn::Invalid {
                    ExecOutcome::ExceptionReturn { exc_return: target }
                } else if target & 1 == 0 {
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
                // LR = 下一条指令地址 | Thumb 位
                cpu.regs[14] = cpu.regs[15].wrapping_add(4) | 1;
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
                        if let Err(_f) = memory.write_u32(addr, val) {
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
                self.sync_sp(cpu);
                ExecOutcome::Continue
            }
            Instruction::Pop { regs, pc } => {
                // POP {reglist} — 出栈并递增 SP；含 pc 时以 Branch 语义设置 PC
                // （避免引擎 Continue 路径在 PC 上再 +width）
                let sp = cpu.regs[13];
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
                    cpu.regs[13] = sp.wrapping_add((count + 1) * 4);
                    self.sync_sp(cpu);
                    // POP{.., pc} 装入 EXC_RETURN 值 → 异常返回（ARMv7-M B1.5.6：
                    // 任一 PC 装载 EXC_RETURN 即触发返回，不限于 BX；xPortSysTickHandler
                    // 的 pop {r3, pc} 返回路径依赖此语义）
                    if crate::nvic::ExcReturn::from_value(val) != crate::nvic::ExcReturn::Invalid {
                        return ExecOutcome::ExceptionReturn { exc_return: val };
                    }
                    cpu.regs[15] = val & !1; // 清 Thumb 位后直接写入 PC（与 LDM 写 PC 语义一致）
                    return ExecOutcome::Branch { target: val & !1 };
                }
                cpu.regs[13] = sp.wrapping_add(count * 4);
                self.sync_sp(cpu);
                ExecOutcome::Continue
            }
            Instruction::Ldm {
                rn,
                regs,
                writeback,
                descending,
            } => {
                let base = cpu.regs[*rn as usize];
                let count = regs.count_ones() as u32;
                // IA：起始 = base；DB：起始 = base - 4×count（先减后访存）
                let start = if *descending {
                    base.wrapping_sub(count * 4)
                } else {
                    base
                };
                let mut addr = start;
                let mut pc_val: Option<u32> = None;
                let mut pc_exc_return = false;
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
                        if i == 15 {
                            // LDM {.., pc}：装入 EXC_RETURN 值 → 异常返回（同 POP{pc}）；
                            // 否则以 Branch 语义设置 PC（避免 +width）
                            if crate::nvic::ExcReturn::from_value(val)
                                != crate::nvic::ExcReturn::Invalid
                            {
                                pc_val = Some(val);
                                pc_exc_return = true;
                            } else {
                                pc_val = Some(val & !1);
                            }
                        } else {
                            cpu.regs[i] = val;
                        }
                        addr += 4;
                    }
                }
                if *writeback {
                    // IA：回写 = base+4×count（尾地址）；DB：回写 = start（= base-4×count）
                    let wb = if *descending { start } else { addr };
                    cpu.regs[*rn as usize] = wb;
                    if *rn == 13 {
                        self.sync_sp(cpu);
                    }
                }
                match (pc_val, pc_exc_return) {
                    (Some(v), true) => ExecOutcome::ExceptionReturn { exc_return: v },
                    (Some(t), false) => ExecOutcome::Branch { target: t },
                    (None, _) => ExecOutcome::Continue,
                }
            }
            Instruction::Stm {
                rn,
                regs,
                writeback,
                descending,
            } => {
                let base = cpu.regs[*rn as usize];
                let count = regs.count_ones() as u32;
                // IA：起始 = base；DB：起始 = base - 4×count（先减后访存）
                let start = if *descending {
                    base.wrapping_sub(count * 4)
                } else {
                    base
                };
                let mut addr = start;
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
                    let wb = if *descending { start } else { addr };
                    cpu.regs[*rn as usize] = wb;
                    if *rn == 13 {
                        self.sync_sp(cpu);
                    }
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
                    LoadStoreOffset::RegisterShifted { rm: rms, lsl } => base
                        .wrapping_add(cpu.regs[*rms as usize].wrapping_shl(*lsl as u32)),
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
            // 有符号加载（LDRSB/LDRSH）：读后符号扩展到 32 位
            Instruction::LdrSignExtend {
                rt,
                rn,
                offset,
                width,
            } => {
                let base = cpu.regs[*rn as usize];
                let addr = match offset {
                    LoadStoreOffset::Immediate(imm) => base.wrapping_add(*imm),
                    LoadStoreOffset::Register(rm) => base.wrapping_add(cpu.regs[*rm as usize]),
                    LoadStoreOffset::RegisterShifted { rm: rms, lsl } => base
                        .wrapping_add(cpu.regs[*rms as usize].wrapping_shl(*lsl as u32)),
                };
                let val = match width {
                    AccessWidth::Byte => memory.read_u8(addr).map(|v| (v as i8) as u32),
                    AccessWidth::HalfWord => memory.read_u16(addr).map(|v| (v as i16) as u32),
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
                    LoadStoreOffset::RegisterShifted { rm: rms, lsl } => base
                        .wrapping_add(cpu.regs[*rms as usize].wrapping_shl(*lsl as u32)),
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
                // LDR literal：addr = Align(PC+4, 4) + imm（PC = 当前指令地址）
                let base = (cpu.regs[15].wrapping_add(4)) & !3;
                let addr = base.wrapping_add(*imm);
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
            Instruction::LdrD { rt, rt2, rn, imm } => {
                // LDRD Rt, Rt2, [Rn, #imm]：加载 64 位（小端，低字到 Rt）
                let addr = cpu.regs[*rn as usize].wrapping_add(*imm);
                match (memory.read_u32(addr), memory.read_u32(addr + 4)) {
                    (Ok(lo), Ok(hi)) => {
                        cpu.regs[*rt as usize] = lo;
                        cpu.regs[*rt2 as usize] = hi;
                        ExecOutcome::Continue
                    }
                    _ => ExecOutcome::Fault {
                        reason: super::FaultReason::BusFault { address: addr },
                    },
                }
            }
            Instruction::StrD { rt, rt2, rn, imm } => {
                let addr = cpu.regs[*rn as usize].wrapping_add(*imm);
                match (
                    memory.write_u32(addr, cpu.regs[*rt as usize]),
                    memory.write_u32(addr + 4, cpu.regs[*rt2 as usize]),
                ) {
                    (Ok(()), Ok(())) => ExecOutcome::Continue,
                    _ => ExecOutcome::Fault {
                        reason: super::FaultReason::BusFault { address: addr },
                    },
                }
            }
            // 带回写加载/存储（前变址 [Rn,#imm]! / 后变址 [Rn],#imm，FRT-INS-03 同族）
            Instruction::LdrStrWb {
                rt,
                rn,
                imm,
                width,
                load,
                sign_extend,
                pre,
            } => {
                let base = cpu.regs[*rn as usize];
                // 前变址：addr = base + imm，回写 rn = addr；后变址：addr = base，回写 rn = base + imm
                let addr = if *pre { base.wrapping_add(*imm) } else { base };
                let wb = base.wrapping_add(*imm);
                if *load {
                    let val = match width {
                        AccessWidth::Byte => memory.read_u8(addr).map(|v| {
                            if *sign_extend { (v as i8) as u32 } else { v as u32 }
                        }),
                        AccessWidth::HalfWord => memory.read_u16(addr).map(|v| {
                            if *sign_extend { (v as i16) as u32 } else { v as u32 }
                        }),
                        AccessWidth::Word => memory.read_u32(addr),
                    };
                    match val {
                        Ok(v) => {
                            cpu.regs[*rt as usize] = v;
                            cpu.regs[*rn as usize] = wb;
                            if *rn == 13 {
                                self.sync_sp(cpu);
                            }
                            ExecOutcome::Continue
                        }
                        Err(_f) => ExecOutcome::Fault {
                            reason: super::FaultReason::BusFault { address: addr },
                        },
                    }
                } else {
                    let val = cpu.regs[*rt as usize];
                    let result = match width {
                        AccessWidth::Byte => memory.write_u8(addr, val as u8),
                        AccessWidth::HalfWord => memory.write_u16(addr, val as u16),
                        AccessWidth::Word => memory.write_u32(addr, val),
                    };
                    match result {
                        Ok(()) => {
                            cpu.regs[*rn as usize] = wb;
                            if *rn == 13 {
                                self.sync_sp(cpu);
                            }
                            ExecOutcome::Continue
                        }
                        Err(_f) => ExecOutcome::Fault {
                            reason: super::FaultReason::MemManage { address: addr },
                        },
                    }
                }
            }
            Instruction::MsrMrs { rt, reg, read } => {
                if *read {
                    // MRS：特殊寄存器 → 核心寄存器
                    let v = match reg {
                        SpecialReg::Apsr => cpu.xpsr & 0xF800_0000,
                        SpecialReg::ApsrGe => cpu.xpsr & 0xF8FF_0000,
                        SpecialReg::Iapsr => cpu.xpsr & 0xF800_01FF,
                        SpecialReg::Eapsr => cpu.xpsr & 0xF9FF_0000,
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
                        // IAPSR/EAPSR 写：仅 APSR 字段可写，IPSR/EPSR 位忽略（ARM 语义）
                        SpecialReg::Iapsr => {
                            cpu.xpsr = (cpu.xpsr & !0xF800_0000) | (v & 0xF800_0000)
                        }
                        SpecialReg::Eapsr => {
                            cpu.xpsr = (cpu.xpsr & !0xF800_0000) | (v & 0xF800_0000)
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
                        SpecialReg::Control => {
                            cpu.control = (v & 0x3) as u8;
                            // SPSEL 切换：SP 别名即时更新（ARM 语义：regs[13] 指向新选栈）
                            cpu.regs[13] = if cpu.control & 1 != 0 {
                                cpu.psp
                            } else {
                                cpu.msp
                            };
                        }
                    }
                }
                ExecOutcome::Continue
            }
            // 数据屏障 DSB/ISB/DMB（FRT-INS-02）：单核顺序模拟语义 = 无操作
            // （不 fault、不改变可观测状态；屏障的存储/取指顺序保证在单核顺序模型中恒成立）
            Instruction::Barrier { .. } => ExecOutcome::Continue,
            // CPSIE/CPSID（FRT-INS-01）：置/清 PRIMASK（i）与 FAULTMASK（f）
            Instruction::Cps { disable, i, f } => {
                if *i {
                    cpu.primask = if *disable { 1 } else { 0 };
                }
                if *f {
                    cpu.faultmask = if *disable { 1 } else { 0 };
                }
                ExecOutcome::Continue
            }
            // CLZ（FRT-INS-04）：前导零计数（Rd = 31 - 最高置位位索引）
            Instruction::Clz { rd, rm } => {
                cpu.regs[*rd as usize] = cpu.regs[*rm as usize].leading_zeros();
                ExecOutcome::Continue
            }
            // RBIT（FRT-INS-05 SHOULD）：位反转
            Instruction::Rbit { rd, rm } => {
                cpu.regs[*rd as usize] = cpu.regs[*rm as usize].reverse_bits();
                ExecOutcome::Continue
            }
            // REV/REV16/REVSH（FRT-INS-05 SHOULD）：字节序反转
            Instruction::Rev { rd, rm, kind } => {
                let v = cpu.regs[*rm as usize];
                let r = match kind {
                    // REV：整字字节反转（AABBCCDD → DDCCBBAA）
                    RevKind::Rev => v.swap_bytes(),
                    // REV16：半字内字节反转（AABBCCDD → BBAADDCC）
                    RevKind::Rev16 => {
                        ((v & 0xFFFF) as u16).swap_bytes() as u32
                            | (((v >> 16) as u16).swap_bytes() as u32) << 16
                    }
                    // REVSH：低半字字节反转 + 符号扩展到 32 位
                    RevKind::RevSh => ((v & 0xFFFF) as u16).swap_bytes() as i16 as i32 as u32,
                };
                cpu.regs[*rd as usize] = r;
                ExecOutcome::Continue
            }
            // SXTH/SXTB/UXTH/UXTB（16 位 T1，FRT-INS-05）：符号/零扩展
            Instruction::Extend { rd, rm, kind } => {
                let v = cpu.regs[*rm as usize];
                let r = match kind {
                    // SXTH：低半字符号扩展到 32 位
                    ExtendKind::Sxth => (v & 0xFFFF) as u16 as i16 as i32 as u32,
                    // SXTB：低字节符号扩展到 32 位
                    ExtendKind::Sxtb => (v & 0xFF) as u8 as i8 as i32 as u32,
                    // UXTH：低半字零扩展
                    ExtendKind::Uxth => v & 0xFFFF,
                    // UXTB：低字节零扩展
                    ExtendKind::Uxtb => v & 0xFF,
                };
                cpu.regs[*rd as usize] = r;
                ExecOutcome::Continue
            }
            // UBFX/SBFX/BFI/BFC（FRT-INS-05 SHOULD）：位域提取/插入/清除
            Instruction::BitField {
                rd,
                rn,
                lsb,
                width,
                kind,
            } => {
                let lsb = *lsb as u32;
                let width = *width as u32;
                // width 最大 32（UBFX #0,#32 / BFI msb=31）：避免 1<<32 溢出
                let mask = if width >= 32 {
                    u32::MAX
                } else {
                    (1u32 << width) - 1
                };
                let src = cpu.regs[*rn as usize];
                let r = match kind {
                    // UBFX：无符号提取
                    BitFieldKind::Ubfx => (src >> lsb) & mask,
                    // SBFX：提取后按 width 符号扩展
                    BitFieldKind::Sbfx => {
                        let u = (src >> lsb) & mask;
                        let sign = 1u32 << (width - 1);
                        if u & sign != 0 {
                            u | !mask
                        } else {
                            u
                        }
                    }
                    // BFI：把 Rd 的 [lsb, lsb+width) 替换为 Rn 低 width 位
                    BitFieldKind::Bfi => {
                        let field = mask << lsb;
                        (cpu.regs[*rd as usize] & !field) | ((src << lsb) & field)
                    }
                    // BFC：清除 Rd 的 [lsb, lsb+width)（Rn=1111 无源）
                    BitFieldKind::Bfc => cpu.regs[*rd as usize] & !(mask << lsb),
                };
                cpu.regs[*rd as usize] = r;
                ExecOutcome::Continue
            }
            // LDREX/LDREXB/LDREXH（FRT-INS-05 SHOULD）：单核语义与 LDR 等价
            // （独占监视器恒成功——无并发写者；行为诚实：真实硬件单核同样不失败）
            Instruction::Ldrex { rt, rn, imm, width } => {
                let addr = cpu.regs[*rn as usize].wrapping_add(*imm);
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
            // STREX/STREXB/STREXH（FRT-INS-05 SHOULD）：单核语义 = STR + Rd=0（独占成功）
            Instruction::Strex { rd, rt, rn, imm, width } => {
                let addr = cpu.regs[*rn as usize].wrapping_add(*imm);
                let res = match width {
                    AccessWidth::Byte => memory.write_u8(addr, cpu.regs[*rt as usize] as u8),
                    AccessWidth::HalfWord => memory.write_u16(addr, cpu.regs[*rt as usize] as u16),
                    AccessWidth::Word => memory.write_u32(addr, cpu.regs[*rt as usize]),
                };
                match res {
                    Ok(()) => {
                        // 独占访问成功：Rd = 0
                        cpu.regs[*rd as usize] = 0;
                        ExecOutcome::Continue
                    }
                    Err(_f) => ExecOutcome::Fault {
                        reason: super::FaultReason::BusFault { address: addr },
                    },
                }
            }
            Instruction::Svc { imm8 } => {
                // SVC → 触发异常 11（SVCall，FRT-EXC-07）：入口由引擎完成
                // （压栈/跳向量/EXC_RETURN），不再返回 UnimplementedInstr Fault。
                // 同步异常语义：现场帧 PC 槽 = 下一条指令地址（SVC 为 16 位，PC+2）
                let _ = imm8;
                ExecOutcome::TakeException {
                    number: crate::nvic::ExceptionNumber::SvCall.as_u8(),
                    return_pc: cpu.regs[15].wrapping_add(2),
                }
            }
            Instruction::Breakpoint { imm8: _ } => {
                // BKPT：调试事件（引擎统计 exceptions 并停止 run）
                ExecOutcome::DebugEvent
            }
            Instruction::It { cond, mask } => {
                // ITSTATE：N = 4 - 最低置位位索引（GNU as/QEMU 实测：
                // mask 1000→1 条、0100/1100→2、1110/1010/0010→3、1111/1101/0001→4）
                let n = 4 - mask.trailing_zeros() as u8;
                self.it_remaining = n;
                self.it_block_len = n;
                self.it_firstcond = *cond;
                self.it_mask = *mask;
                ExecOutcome::Continue
            }
            Instruction::ExceptionReturn => {
                // 解码直产路径（当前 decode 无产出；BX EXC_RETURN 走 BranchExchange 携带值）。
                // 保守语义：EXC_RETURN 恒在 LR（r14）——异常入口写入
                ExecOutcome::ExceptionReturn {
                    exc_return: cpu.regs[14],
                }
            }

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
                        if n == 0 {
                            // sh=1：ASR（USAT/SSAT 均算术右移；n=0 → 移 32 位，符号填充）
                            ((t as i32) >> 31) as u32
                        } else {
                            ((t as i32) >> n) as u32
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
                                // IXC：sqrt 不精确（完美平方除外）。
                                // 融合乘加 r*r − f == 0 ⟺ r 为精确平方根。
                                if f != 0.0
                                    && !f.is_infinite()
                                    && r.mul_add(r, -f) != 0.0
                                {
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
                                // IXC：f32 平方根不精确判定（f64 精确中间量，r 为 f32 精确值）
                                if f != 0.0
                                    && !f.is_infinite()
                                    && (r as f64).mul_add(r as f64, -(f as f64)) != 0.0
                                {
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
            Instruction::FpLoadStoreMulti {
                vd,
                rn,
                count,
                load,
                double,
                decrement,
                writeback,
            } => {
                let base = cpu.regs[*rn as usize];
                let bytes = (*count as u32) * if *double { 8 } else { 4 };
                let addr = if *decrement {
                    base.wrapping_sub(bytes)
                } else {
                    base
                };
                // 对齐：字对齐（VFP 多寄存器访问 4 字节对齐）
                if addr & 3 != 0 {
                    return ExecOutcome::Fault {
                        reason: super::FaultReason::UnalignedAccess { address: addr },
                    };
                }
                let fpu = &mut cpu.fpu;
                let stride = if *double { 8 } else { 4 };
                for i in 0..*count {
                    let a = addr + i * stride;
                    if *double {
                        let reg = *vd as usize + i as usize;
                        if *load {
                            let lo = match memory.read_u32(a) {
                                Ok(v) => v,
                                Err(e) => {
                                    return ExecOutcome::Fault {
                                        reason: self.map_mem_fault(e),
                                    }
                                }
                            };
                            let hi = match memory.read_u32(a + 4) {
                                Ok(v) => v,
                                Err(e) => {
                                    return ExecOutcome::Fault {
                                        reason: self.map_mem_fault(e),
                                    }
                                }
                            };
                            fpu.write_d(reg, (lo as u64) | ((hi as u64) << 32));
                        } else {
                            let v = fpu.read_d(reg);
                            if let Err(e) = memory.write_u32(a, v as u32) {
                                return ExecOutcome::Fault {
                                    reason: self.map_mem_fault(e),
                                };
                            }
                            if let Err(e) = memory.write_u32(a + 4, (v >> 32) as u32) {
                                return ExecOutcome::Fault {
                                    reason: self.map_mem_fault(e),
                                };
                            }
                        }
                    } else if *load {
                        match memory.read_u32(a) {
                            Ok(v) => fpu.write_s(*vd as usize + i as usize, v),
                            Err(e) => {
                                return ExecOutcome::Fault {
                                    reason: self.map_mem_fault(e),
                                }
                            }
                        }
                    } else {
                        match memory.write_u32(a, fpu.read_s(*vd as usize + i as usize)) {
                            Ok(()) => {}
                            Err(e) => {
                                return ExecOutcome::Fault {
                                    reason: self.map_mem_fault(e),
                                }
                            }
                        }
                    }
                }
                if *writeback {
                    let final_addr = if *decrement {
                        base.wrapping_sub(bytes)
                    } else {
                        base.wrapping_add(bytes)
                    };
                    cpu.regs[*rn as usize] = final_addr;
                }
                ExecOutcome::Continue
            }
            Instruction::FpCvtFixed {
                vd,
                fbits,
                width,
                signed,
                to_float,
            } => {
                let fpu = &mut cpu.fpu;
                let v = fpu.read_s(*vd as usize);
                let (r, flags) = if *to_float {
                    fpu::cvt_fixed_to_f32(fpu, v, *fbits, *signed, *width)
                } else {
                    fpu::cvt_f32_to_fixed(fpu, v, *fbits, *signed, *width)
                };
                self.apply_fpu_flags(fpu, &flags);
                fpu.write_s(*vd as usize, r);
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

    /// 移位计算（n 为已按 ARM 语义取模/限幅后的移位量）
    /// 立即数形式：LSR/ASR #0 编码为 #32（decode 端已转换），LSL #0 不移位；
    /// 寄存器形式：低 8 位，0 → 不移位，≥32 → 0（LSR/LSL）/ 符号填充（ASR）。
    fn shift_val(&self, cpu: &CpuState, val: u32, kind: ShiftKind, n: u8) -> u32 {
        match kind {
            ShiftKind::Lsl => {
                if n >= 32 {
                    0
                } else if n == 0 {
                    val
                } else {
                    val.wrapping_shl(n as u32)
                }
            }
            ShiftKind::Lsr => {
                if n >= 32 {
                    0
                } else if n == 0 {
                    val
                } else {
                    val.wrapping_shr(n as u32)
                }
            }
            ShiftKind::Asr => {
                if n >= 32 {
                    ((val as i32) >> 31) as u32
                } else if n == 0 {
                    val
                } else {
                    ((val as i32) >> n) as u32
                }
            }
            ShiftKind::Ror => {
                if n == 0 {
                    val
                } else {
                    val.rotate_right((n & 0x1F) as u32)
                }
            }
            ShiftKind::Rrx => (val >> 1) | ((self.carry_bit(cpu) as u32) << 31),
        }
    }

    /// 移位计算 C 标志（最后移出位）
    /// 与 shift_val 同语义约定：立即数 LSR/ASR #0 → #32；n>=32 按移满处理。
    fn shift_carry(&self, val: u32, kind: ShiftKind, n: u8) -> bool {
        match kind {
            ShiftKind::Lsl => {
                if n >= 32 || n == 0 {
                    false
                } else {
                    (val >> (32 - n)) & 1 != 0
                }
            }
            ShiftKind::Lsr => {
                if n >= 32 {
                    (val >> 31) & 1 != 0
                } else if n == 0 {
                    false
                } else {
                    (val >> (n - 1)) & 1 != 0
                }
            }
            ShiftKind::Asr => {
                if n >= 32 {
                    (val >> 31) & 1 != 0
                } else if n == 0 {
                    false
                } else {
                    (val >> (n - 1)) & 1 != 0
                }
            }
            ShiftKind::Ror => {
                if n == 0 {
                    false
                } else {
                    (val >> ((n & 0x1F) - 1)) & 1 != 0
                }
            }
            ShiftKind::Rrx => val & 1 != 0,
        }
    }

    /// SP 别名同步：regs[13] 变更后，按 CONTROL.SPSEL 同步到 msp/psp
    /// （A7 修复：MSP/PSP 与 SP 运算保持一致性）
    fn sync_sp(&self, cpu: &mut CpuState) {
        let sp = cpu.regs[13];
        // ARMv7-M：Handler 模式（IPSR!=0）SP 恒为 MSP（与 SPSEL 无关）；
        // Thread 模式才按 CONTROL.SPSEL 选择 MSP/PSP。
        // （卡点 3 根因：自定义 SVC handler 内 push/sub sp 修改的是 MSP，
        //   旧实现按 SPSEL=1 误写 PSP → 异常返回弹错栈）
        if cpu.xpsr & 0x1FF != 0 || cpu.control & 1 == 0 {
            cpu.msp = sp;
        } else {
            cpu.psp = sp;
        }
    }

    /// 逻辑操作更新标志（N/Z，C/V 由调用方处理）
    /// ARMv7-M B1.5.10：16 位隐式 S 指令（IT 块内不更新标志）
    /// MOVS/ADDS/SUBS/ANDS/ORRS/EORS/BICS/MVNS/ASRS/LSRS/LSLS/RORS/NEGS/ADCS/SBCS
    /// （CMP/CMN/TST 恒更新标志不在此列；MULS 在 v7-M 恒不更新 flags，无需抑制）
    fn is_implicit_s_16bit(instr: &Instruction) -> bool {
        matches!(
            instr,
            Instruction::Mov { flags: true, .. }
                | Instruction::Add { flags: true, .. }
                | Instruction::Sub { flags: true, .. }
                | Instruction::And { flags: true, .. }
                | Instruction::Orr { flags: true, .. }
                | Instruction::Eor { flags: true, .. }
                | Instruction::Bic { flags: true, .. }
                | Instruction::Mvn { flags: true, .. }
                | Instruction::Neg { flags: true, .. }
                | Instruction::Shift { flags: true, .. }
                | Instruction::Rsb { flags: true, .. }
                | Instruction::Adc { flags: true, .. }
                | Instruction::Sbc { flags: true, .. }
        )
    }

    fn update_flags_logical(&self, cpu: &mut CpuState, result: u32) {
        if self.it_suppress_flags {
            return;
        }
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
        if self.it_suppress_flags {
            return;
        }
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
        if self.it_suppress_flags {
            return;
        }
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

    /// 三操作数加法（a + b + C）更新标志：C = 进位输出，V = 进位进符号位 XOR 进位出符号位
    ///
    /// 推导：S（33 位）= a + b + C；sum_31 = a_31 ^ b_31 ^ carry_in31，
    /// 故 carry_in31 = a_31 ^ b_31 ^ S_31；V = carry_in31 ^ S_32。
    fn update_flags_add3(&self, cpu: &mut CpuState, a: u32, b: u32, ext: u64, result: u32) {
        if self.it_suppress_flags {
            return;
        }
        self.update_flags_logical(cpu, result);
        // C = bit32（进位输出）
        if ext & (1 << 32) != 0 {
            cpu.xpsr |= 1 << 29;
        } else {
            cpu.xpsr &= !(1 << 29);
        }
        let s31 = ((ext >> 31) & 1) as u32;
        let s32 = ((ext >> 32) & 1) as u32;
        let carry_in31 = ((a >> 31) & 1) ^ ((b >> 31) & 1) ^ s31;
        let v = carry_in31 ^ s32;
        if v != 0 {
            cpu.xpsr |= 1 << 28;
        } else {
            cpu.xpsr &= !(1 << 28);
        }
    }

    /// 读取进位标志（APSR.C，bit29）
    fn carry_bit(&self, cpu: &CpuState) -> bool {
        cpu.xpsr & (1 << 29) != 0
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

    // ================= E1: 16-bit 寄存器数据处理组 golden 测试 =================
    // 编码与 arm-none-eabi-as 实测一致（见任务 E1 清单）；GIVEN/WHEN/THEN 结构。
    // NZCV 断言位序：nzcv() = bit3=N bit2=Z bit1=C bit0=V。

    #[test]
    fn e1_ands_eors_orrs_bics() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // ANDS r2, r3 = 0x401A：0xFF00FF00 & 0xF0F0F0F0 = 0xF000F000
        h.cpu.regs[2] = 0xFF00_FF00;
        h.cpu.regs[3] = 0xF0F0_F0F0;
        h.exec_halfword(0x401A);
        assert_eq!(h.cpu.regs[2], 0xF000_F000);
        assert_eq!(h.nzcv(), 0b1000, "N 置位（结果 bit31=1）");
        // ANDS 置 Z：0x0F00 & 0x00F0 = 0
        h.cpu.regs[2] = 0x0F00;
        h.cpu.regs[3] = 0x00F0;
        h.exec_halfword(0x401A);
        assert_eq!(h.cpu.regs[2], 0);
        assert_eq!(h.nzcv(), 0b0100, "Z 置位");
        // EORS r2, r3 = 0x405A：0xFF00FF00 ^ 0x0FF00FF0 = 0xF0F0F0F0
        h.cpu.regs[2] = 0xFF00_FF00;
        h.cpu.regs[3] = 0x0FF0_0FF0;
        h.exec_halfword(0x405A);
        assert_eq!(h.cpu.regs[2], 0xF0F0_F0F0);
        assert_eq!(h.nzcv(), 0b1000, "N 置位");
        // ORRS r2, r3 = 0x431A：0xF0F0F0F0 | 0x0F0F0F0F = 0xFFFFFFFF
        h.cpu.regs[2] = 0xF0F0_F0F0;
        h.cpu.regs[3] = 0x0F0F_0F0F;
        h.exec_halfword(0x431A);
        assert_eq!(h.cpu.regs[2], 0xFFFF_FFFF);
        assert_eq!(h.nzcv(), 0b1000, "N 置位");
        // BICS r2, r3 = 0x439A：0xFF00FF00 & ~0x0F0F0F0F = 0xF000F000
        h.cpu.regs[2] = 0xFF00_FF00;
        h.cpu.regs[3] = 0x0F0F_0F0F;
        h.exec_halfword(0x439A);
        assert_eq!(h.cpu.regs[2], 0xF000_F000);
        assert_eq!(h.nzcv(), 0b1000, "N 置位");
        // BICS 置 Z：0xFF & ~0xFF = 0
        h.cpu.regs[2] = 0xFF;
        h.cpu.regs[3] = 0xFF;
        h.exec_halfword(0x439A);
        assert_eq!(h.cpu.regs[2], 0);
        assert_eq!(h.nzcv(), 0b0100, "Z 置位");
    }

    #[test]
    fn e1_lsl_lsr_asr_ror_register() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // LSLS r2, r5 = 0x40AA：1 << 4 = 16
        h.cpu.regs[2] = 1;
        h.cpu.regs[5] = 4;
        h.exec_halfword(0x40AA);
        assert_eq!(h.cpu.regs[2], 16);
        assert_eq!(h.nzcv(), 0, "N/Z 清零");
        // LSLS 置 Z：0x80000000 << 1 = 0，C = 移出位 bit31 = 1
        h.cpu.regs[2] = 0x8000_0000;
        h.cpu.regs[5] = 1;
        h.exec_halfword(0x40AA);
        assert_eq!(h.cpu.regs[2], 0);
        assert_eq!(h.nzcv(), 0b0110, "Z 置位 + C 置位（bit31 移出）");
        // LSRS r2, r5 = 0x40EA：0x80000000 >> 1 = 0x40000000
        h.cpu.regs[2] = 0x8000_0000;
        h.cpu.regs[5] = 1;
        h.exec_halfword(0x40EA);
        assert_eq!(h.cpu.regs[2], 0x4000_0000);
        assert_eq!(h.nzcv(), 0, "N/Z 清零");
        // ASRS r2, r5 = 0x412A：0x80000000 算术右移 1 = 0xC0000000
        h.cpu.regs[2] = 0x8000_0000;
        h.cpu.regs[5] = 1;
        h.exec_halfword(0x412A);
        assert_eq!(h.cpu.regs[2], 0xC000_0000);
        assert_eq!(h.nzcv(), 0b1000, "N 置位（符号扩展）");
        // RORS r2, r5 = 0x41EA：0x00000008 循环右移 4 = 0x80000000
        // 最后移出位 = bit3 = 1 → C 置位（ARM 语义）
        h.cpu.regs[2] = 0x0000_0008;
        h.cpu.regs[5] = 4;
        h.exec_halfword(0x41EA);
        assert_eq!(h.cpu.regs[2], 0x8000_0000);
        assert_eq!(h.nzcv(), 0b1010, "N 置位 + C 置位（移出 bit3）");
        // ROR 复合：0x0000000F 循环右移 2 = 0xC0000003
        h.cpu.regs[2] = 0x0000_000F;
        h.cpu.regs[5] = 2;
        h.exec_halfword(0x41EA);
        assert_eq!(h.cpu.regs[2], 0xC000_0003);
    }

    #[test]
    fn e1_adc_sbc_flags() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // ADCS r2, r3 = 0x415A，C=1：1 + 1 + 1 = 3（无进位出 → C 清零）
        h.cpu.regs[2] = 1;
        h.cpu.regs[3] = 1;
        h.cpu.xpsr = 1 << 29; // C=1
        h.exec_halfword(0x415A);
        assert_eq!(h.cpu.regs[2], 3);
        assert_eq!(h.nzcv(), 0, "1+1+1=3：无进位出 C=0，N/Z 清零");
        // ADC 溢出：0x7FFFFFFF + 1 + 1 = 0x80000001 → V=1 N=1 C=0
        h.cpu.regs[2] = 0x7FFF_FFFF;
        h.cpu.regs[3] = 1;
        h.cpu.xpsr = 1 << 29;
        h.exec_halfword(0x415A);
        assert_eq!(h.cpu.regs[2], 0x8000_0001);
        assert_eq!(h.nzcv(), 0b1001, "N=1 V=1 C=0（正+正溢出）");
        // ADC 进位出：0xFFFFFFFF + 1 + 0 = 0x00000000 → C=1 Z=1
        h.cpu.regs[2] = 0xFFFF_FFFF;
        h.cpu.regs[3] = 1;
        h.cpu.xpsr = 0;
        h.exec_halfword(0x415A);
        assert_eq!(h.cpu.regs[2], 0);
        assert_eq!(h.nzcv(), 0b0110, "C=1 Z=1（进位出、结果为 0）");
        // SBCS r2, r3 = 0x419A，C=1：5 - 3 - 0 = 2
        h.cpu.regs[2] = 5;
        h.cpu.regs[3] = 3;
        h.cpu.xpsr = 1 << 29;
        h.exec_halfword(0x419A);
        assert_eq!(h.cpu.regs[2], 2);
        assert_eq!(h.nzcv(), 0b0010, "C=1（无借位）");
        // SBC 借位：3 - 5 - 0 = -2 → N=1 C=0
        h.cpu.regs[2] = 3;
        h.cpu.regs[3] = 5;
        h.cpu.xpsr = 1 << 29;
        h.exec_halfword(0x419A);
        assert_eq!(h.cpu.regs[2], 0xFFFF_FFFE);
        assert_eq!(h.nzcv(), 0b1000, "N=1 C=0（借位）");
        // SBC C=0（减 NOT(C)=1）：5 - 3 - 1 = 1
        h.cpu.regs[2] = 5;
        h.cpu.regs[3] = 3;
        h.cpu.xpsr = 0;
        h.exec_halfword(0x419A);
        assert_eq!(h.cpu.regs[2], 1);
        assert_eq!(h.nzcv(), 0b0010, "C=1（无借位）");
    }

    /// E2: BL 执行 — LR=(PC+4)|1 且跳转（编码 0xF000 F80F @0x86E → 0x890，固件实测）
    #[test]
    fn e2_bl_sets_lr_and_branches() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.regs[15] = 0x86E;
        assert_eq!(
            h.exec_word(0xF000_F80F),
            ExecOutcome::Branch { target: 0x890 }
        );
        assert_eq!(h.cpu.regs[14], (0x86E + 4) | 1, "LR = (PC+4)|1");
        assert_eq!(h.cpu.regs[15], 0x86E, "PC 由引擎推进/分支目标写入");
    }

    /// E2: BLX(立即数) 在 ARMv7-M 上 UNDEFINED（仅 A-profile 有效）
    /// P4 语义修正（P1-1）：decode_blx 诚实返回 Unimplemented，引擎产出 Fault
    /// 而非 A-profile Branch 行为；编码 0xF000 E80C @0x4（as 实测，A-profile 目标 0x20）
    #[test]
    fn e2_blx_sets_lr_and_branches() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.regs[15] = 0x4;
        assert_eq!(
            h.exec_word(0xF000_E80C),
            ExecOutcome::Fault {
                reason: crate::engine::FaultReason::UnimplementedInstr,
            }
        );
        assert_eq!(h.cpu.regs[14], 0, "LR 不被写入（未执行跳转）");
    }

    /// E2: BLX 负偏移 — 0xF7FF EFFE @0x0（A-profile 目标 0x0）：ARMv7-M 上 UNDEF → Fault
    #[test]
    fn e2_blx_negative_offset() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.regs[15] = 0x0;
        assert_eq!(
            h.exec_word(0xF7FF_EFFE),
            ExecOutcome::Fault {
                reason: crate::engine::FaultReason::UnimplementedInstr,
            }
        );
    }

    /// E2: BLX 非 4 对齐 PC — 0xF000 E808 @0x6（A-profile 目标 0x18）：ARMv7-M 上 UNDEF → Fault
    #[test]
    fn e2_blx_aligned_base() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.regs[15] = 0x6;
        assert_eq!(
            h.exec_word(0xF000_E808),
            ExecOutcome::Fault {
                reason: crate::engine::FaultReason::UnimplementedInstr,
            }
        );
        assert_eq!(h.cpu.regs[14], 0, "LR 不被写入（未执行跳转）");
    }

    #[test]
    fn e1_tst_cmp_cmn_neg_mvn_mul() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // TST r2, r3 = 0x421A：0xFF00FF00 & 0x00FF00FF = 0 → Z=1
        h.cpu.regs[2] = 0xFF00_FF00;
        h.cpu.regs[3] = 0x00FF_00FF;
        h.exec_halfword(0x421A);
        assert_eq!(h.nzcv(), 0b0100, "Z 置位");
        // TST 非零：0xFF00 & 0x0FF0 = 0x0F00 → N=0 Z=0
        h.cpu.regs[2] = 0xFF00;
        h.cpu.regs[3] = 0x0FF0;
        h.exec_halfword(0x421A);
        assert_eq!(h.nzcv(), 0, "N/Z 清零");
        // CMP r2, r3 = 0x429A：5 == 5 → Z=1 C=1
        h.cpu.regs[2] = 5;
        h.cpu.regs[3] = 5;
        h.exec_halfword(0x429A);
        assert_eq!(h.nzcv(), 0b0110, "Z=1 C=1");
        // CMP 3 < 5 → N=1 C=0
        h.cpu.regs[2] = 3;
        h.cpu.regs[3] = 5;
        h.exec_halfword(0x429A);
        assert_eq!(h.nzcv(), 0b1000, "N=1 C=0");
        // CMN r2, r3 = 0x42DA：5 + (-5) = 0 → Z=1 C=1
        h.cpu.regs[2] = 5;
        h.cpu.regs[3] = 0xFFFF_FFFB;
        h.exec_halfword(0x42DA);
        assert_eq!(h.nzcv(), 0b0110, "Z=1 C=1");
        // NEGS r2, r3 = 0x425A：0 - 5 = -5 → N=1 C=0
        h.cpu.regs[2] = 0xDEAD_BEEF;
        h.cpu.regs[3] = 5;
        h.exec_halfword(0x425A);
        assert_eq!(h.cpu.regs[2], 0xFFFF_FFFB);
        assert_eq!(h.nzcv(), 0b1000, "N=1 C=0");
        // NEG 0：0 - 0 = 0 → Z=1 C=1
        h.cpu.regs[3] = 0;
        h.exec_halfword(0x425A);
        assert_eq!(h.cpu.regs[2], 0);
        assert_eq!(h.nzcv(), 0b0110, "Z=1 C=1");
        // NEG 溢出：0 - 0x80000000 = 0x80000000 → V=1
        h.cpu.regs[3] = 0x8000_0000;
        h.exec_halfword(0x425A);
        assert_eq!(h.cpu.regs[2], 0x8000_0000);
        assert_eq!(h.nzcv(), 0b1001, "N=1 V=1（0-INT_MIN 溢出）");
        // MVNS r2, r3 = 0x43DA：~0xFFFFFFFF = 0 → Z=1（逻辑指令不动 C/V，先清零 xpsr）
        h.cpu.xpsr = 0;
        h.cpu.regs[2] = 0;
        h.cpu.regs[3] = 0xFFFF_FFFF;
        h.exec_halfword(0x43DA);
        assert_eq!(h.cpu.regs[2], 0);
        assert_eq!(h.nzcv(), 0b0100, "Z 置位");
        // MVN ~0 = 0xFFFFFFFF → N=1
        h.cpu.regs[3] = 0;
        h.exec_halfword(0x43DA);
        assert_eq!(h.cpu.regs[2], 0xFFFF_FFFF);
        assert_eq!(h.nzcv(), 0b1000, "N 置位");
        // MULS r2, r3 = 0x435A：0x12345678 * 0x10000 = 0x56780000（低 32 位）
        h.cpu.regs[2] = 0x1234_5678;
        h.cpu.regs[3] = 0x0001_0000;
        h.cpu.xpsr = 0;
        h.exec_halfword(0x435A);
        assert_eq!(h.cpu.regs[2], 0x5678_0000);
        assert_eq!(h.nzcv(), 0, "ARMv7E-M：MUL 不更新 flags");
        // MUL 结果全 0 时 flags 仍不动
        h.cpu.regs[2] = 0;
        h.cpu.regs[3] = 0xFFFF_FFFF;
        h.cpu.xpsr = 0;
        h.exec_halfword(0x435A);
        assert_eq!(h.cpu.regs[2], 0);
        assert_eq!(h.nzcv(), 0, "MUL 0 结果也不写 flags");
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
                                               // POP {R0, R1, PC}：含 PC → Branch 语义（引擎不再 +width）
        cpu.regs[0] = 0;
        cpu.regs[1] = 0;
        let instr = Instruction::Pop {
            regs: 0b11,
            pc: true,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Branch {
                target: 0x0800_0000
            }
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
            descending: false,
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
            descending: false,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[3], 0xAA);
        assert_eq!(cpu.regs[4], 0xBB);
        assert_eq!(cpu.regs[0], 0x2000_0008);
    }

    /// A7：SP 运算后同步 msp/psp（CONTROL.SPSEL=0 → msp）
    #[test]
    fn sp_sync_msp_psp() {
        let (mut ex, mut cpu, mut mem) = setup();
        // 默认 CONTROL.SPSEL=0 → SP 别名 MSP
        cpu.regs[13] = 0x2000_1000;
        // SUB SP, #8（0xB085）
        let instr = Instruction::Sub {
            rd: 13,
            rn: 13,
            rm: None,
            imm: Some(8),
            flags: false,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[13], 0x2000_0FF8);
        assert_eq!(cpu.msp, 0x2000_0FF8, "SPSEL=0 → SP 同步到 msp");

        // SPSEL=1 → SP 别名 PSP
        cpu.control = 1;
        cpu.regs[13] = 0x2000_2000;
        let instr = Instruction::Add {
            rd: 13,
            rn: 13,
            rm: None,
            imm: Some(16),
            flags: false,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[13], 0x2000_2010);
        assert_eq!(cpu.psp, 0x2000_2010, "SPSEL=1 → SP 同步到 psp");
        assert_eq!(cpu.msp, 0x2000_0FF8, "msp 不受 PSP 运算影响");

        // PUSH 也同步（SPSEL=0）
        cpu.control = 0;
        cpu.regs[13] = 0x2000_1000;
        let instr = Instruction::Push {
            regs: 0b1,
            lr: false,
        };
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &instr),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.msp, 0x2000_0FFC, "PUSH 后 SP 同步到 msp");
    }

    /// A8：32-bit LDR/STR 家族 golden（编码经 arm-none-eabi-as 实测）
    #[test]
    fn golden_32bit_ldr_str_family() {
        let (mut ex, mut cpu, mut mem) = setup();
        let mut dec = Decoder::new();
        cpu.regs[1] = 0x2000_0000;
        // LDR.W r0, [r1, #4]（0xF8D1 0004）
        mem.write_u32(0x2000_0004, 0x1122_3344).unwrap();
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xF8D1_0004, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0x1122_3344);
        // STR.W r0, [r1, #8]（0xF8C1 0008）
        cpu.regs[0] = 0xAABB_CCDD;
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xF8C1_0008, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(mem.read_u32(0x2000_0008).unwrap(), 0xAABB_CCDD);
        // LDRH.W r0, [r1, #2]（0xF8B1 0002）
        mem.write_u16(0x2000_0002, 0xABCD).unwrap();
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xF8B1_0002, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0xABCD);
        // STRH.W r0, [r1, #6]（0xF8A1 0006）
        cpu.regs[0] = 0x1234;
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xF8A1_0006, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(mem.read_u16(0x2000_0006).unwrap(), 0x1234);
        // LDRB.W r0, [r1, #1]（0xF891 0001）
        mem.write_u8(0x2000_0001, 0x7F).unwrap();
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xF891_0001, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0x7F);
        // STRB.W r0, [r1, #3]（0xF881 0003）
        cpu.regs[0] = 0x9A;
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xF881_0003, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(mem.read_u8(0x2000_0003).unwrap(), 0x9A);
        // LDRSH.W r0, [r1, #4]（0xF9B1 0004）——0xFFFF → 符号扩展 -1
        mem.write_u16(0x2000_0004, 0xFFFF).unwrap();
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xF9B1_0004, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0xFFFF_FFFF);
        // LDRSB.W r0, [r1, #5]（0xF991 0005）——0xFF → 符号扩展 -1
        mem.write_u8(0x2000_0005, 0xFF).unwrap();
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xF991_0005, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0xFFFF_FFFF);
        // LDR.W r0, [r1, r2]（0xF851 0002，寄存器偏移）
        cpu.regs[2] = 0x10;
        mem.write_u32(0x2000_0010, 0xCAFE_BABE).unwrap();
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xF851_0002, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0xCAFE_BABE);
        // STR.W r0, [r1, r2]（0xF841 0002）
        cpu.regs[0] = 0xDEAD_BEEF;
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xF841_0002, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(mem.read_u32(0x2000_0010).unwrap(), 0xDEAD_BEEF);
        // LDRD r0, r1, [r2, #8]（0xE9D2 0102）
        cpu.regs[2] = 0x2000_0000;
        mem.write_u32(0x2000_0008, 0x1111_1111).unwrap();
        mem.write_u32(0x2000_000C, 0x2222_2222).unwrap();
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xE9D2_0102, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(cpu.regs[0], 0x1111_1111);
        assert_eq!(cpu.regs[1], 0x2222_2222);
        // STRD r0, r1, [r2, #8]（0xE9C2 0102）
        cpu.regs[0] = 0xAAAA_AAAA;
        cpu.regs[1] = 0xBBBB_BBBB;
        assert_eq!(
            ex.execute(&mut cpu, &mut mem, &dec.decode_word(0xE9C2_0102, 0)),
            ExecOutcome::Continue
        );
        assert_eq!(mem.read_u32(0x2000_0008).unwrap(), 0xAAAA_AAAA);
        assert_eq!(mem.read_u32(0x2000_000C).unwrap(), 0xBBBB_BBBB);
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

    // ============ P5-补：VSQRT IXC + CPACR 门控 ============

    /// VSQRT 不精确检测：完美平方无 IXC，非平方有 IXC（f32/f64）
    #[test]
    fn golden_vsqrt_ixc() {
        let mut h = crate::engine::test_util::Harness::new();
        // sqrt(4.0) = 2.0 精确 → 无 IXC（FPSCR bit4）
        h.cpu.fpu.write_s(1, 4.0f32.to_bits());
        assert_eq!(h.exec_word(0xEEB1_0AE0), ExecOutcome::Continue); // VSQRT.F32 S0, S1
        assert_eq!(h.cpu.fpu.read_s(0), 2.0f32.to_bits());
        assert_eq!(h.cpu.fpu.fpscr & (1 << 4), 0);
        // sqrt(2.0) 不精确 → IXC 置位
        h.cpu.fpu.write_s(1, 2.0f32.to_bits());
        assert_eq!(h.exec_word(0xEEB1_0AE0), ExecOutcome::Continue);
        assert_ne!(h.cpu.fpu.fpscr & (1 << 4), 0);
        // f64：sqrt(9.0) 精确（IXC 为粘性位，先清零 FPSCR 再验证）
        h.cpu.fpu.fpscr = 0;
        h.cpu.fpu.write_d(1, 9.0f64.to_bits());
        assert_eq!(h.exec_word(0xEEB1_0BC1), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_d(0), 3.0f64.to_bits());
        assert_eq!(h.cpu.fpu.fpscr & (1 << 4), 0);
        // f64：sqrt(2.0) 不精确 → IXC
        h.cpu.fpu.write_d(1, 2.0f64.to_bits());
        assert_eq!(h.exec_word(0xEEB1_0BC1), ExecOutcome::Continue);
        assert_ne!(h.cpu.fpu.fpscr & (1 << 4), 0);
    }

    /// CPACR 门控：FPU 未使能（cpacr = 0）时浮点指令 → NOCP UsageFault
    #[test]
    fn golden_cpacr_gate_fpu_disabled() {
        let mut h = crate::engine::test_util::Harness::new();
        // GIVEN: FPU 关闭（CPACR CP10/CP11 = 0）
        h.cpu.cpacr = 0;
        // WHEN: VADD.F32 S0, S1, S2（0xEE30 0A81）
        let out = h.exec_word(0xEE30_0A81);
        // THEN: NOCP UsageFault，FPU 状态不变
        assert_eq!(
            out,
            ExecOutcome::Fault {
                reason: crate::engine::FaultReason::UsageFault {
                    address: h.cpu.regs[15],
                }
            }
        );
        assert_eq!(h.cpu.fpu.read_s(0), 0);
        // VLDR 也被门控
        h.cpu.regs[1] = 0x2000_0000;
        let out = h.exec_word(0xED91_0A00);
        assert!(matches!(
            out,
            ExecOutcome::Fault {
                reason: crate::engine::FaultReason::UsageFault { .. }
            }
        ));
        // MRS/MSR（核心寄存器）不受 CPACR 门控
        h.cpu.regs[2] = 1;
        assert_eq!(h.exec_word(0xF382_8810), ExecOutcome::Continue); // MSR PRIMASK, r2
        assert_eq!(h.cpu.primask, 1);
    }

    /// CPACR 门控：重新使能后 FPU 恢复工作（特权级访问恢复语义）
    #[test]
    fn golden_cpacr_gate_reenable() {
        let mut h = crate::engine::test_util::Harness::new();
        h.cpu.cpacr = 0;
        assert!(matches!(
            h.exec_word(0xEE30_0A81),
            ExecOutcome::Fault {
                reason: crate::engine::FaultReason::UsageFault { .. }
            }
        ));
        // 重新使能 CP10/CP11
        h.cpu.cpacr = 0x00F0_0000;
        h.cpu.fpu.write_s(1, 1.0f32.to_bits());
        h.cpu.fpu.write_s(2, 2.0f32.to_bits());
        assert_eq!(h.exec_word(0xEE30_0A81), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), 3.0f32.to_bits());
    }

    // ================= B1: 16 位立即数移位（LSRS/ASRS #32，imm5=0） =================
    // 期望值 = QEMU MPS2-AN386 实测（/tmp/dtwin_verify2/pshift_qemu.txt）：
    //   r2=0x80000000：LSRS #32→0，ASRS #32→0xFFFFFFFF，LSRS #31→1，ASRS #31→0xFFFFFFFF

    #[test]
    fn b1_16bit_lsr_asr_imm32() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // LSRS r3, r2, #32 = 0x0813：0x80000000 >> 32 = 0（QEMU=00000000），C=bit31=1
        h.cpu.regs[2] = 0x8000_0000;
        h.exec_halfword(0x0813);
        assert_eq!(h.cpu.regs[3], 0x0000_0000, "LSRS #32 → 0（QEMU 一致）");
        assert_eq!(h.nzcv(), 0b0110, "Z 置位 + C 置位（bit31 移出）");
        // ASRS r3, r2, #32 = 0x1013：0x80000000 算术右移 32 = 0xFFFFFFFF（QEMU 一致），C=bit31=1
        h.exec_halfword(0x1013);
        assert_eq!(h.cpu.regs[3], 0xFFFF_FFFF, "ASRS #32 → 0xFFFFFFFF（QEMU 一致）");
        assert_eq!(h.nzcv(), 0b1010, "N 置位 + C 置位（bit31 移出）");
        // 对照 #31 不受影响：LSRS r3, r2, #31 = 0x0FD3：0x80000000 >> 31 = 1（QEMU=00000001），C=bit30=0
        h.exec_halfword(0x0FD3);
        assert_eq!(h.cpu.regs[3], 0x0000_0001, "LSRS #31 → 1（QEMU 一致）");
        assert_eq!(h.nzcv(), 0, "N/Z/C 清零");
        // ASRS r3, r2, #31 = 0x17D3：0x80000000 算术右移 31 = 0xFFFFFFFF（QEMU 一致）
        h.exec_halfword(0x17D3);
        assert_eq!(h.cpu.regs[3], 0xFFFF_FFFF, "ASRS #31 → 0xFFFFFFFF（QEMU 一致）");
        assert_eq!(h.nzcv(), 0b1000, "N 置位");
        // LSLS #0（0x0013）不移位：保持原值
        h.cpu.regs[2] = 0x1234_5678;
        h.exec_halfword(0x0013);
        assert_eq!(h.cpu.regs[3], 0x1234_5678, "LSLS #0 不移位");
    }

    // ================= B2: MOVS.W 立即数（S=1, Rn=1111） =================
    // 期望值 = QEMU 实测（/tmp/dtwin_verify2/pmovs_qemu.txt）：movs r0, #0x8000 → r0=0x00008000
    // 修复前 dtwin=0x00008054（= PC | 0x8000，静默读 PC 作源）

    #[test]
    fn b2_movs_imm32_not_pc_tainted() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.regs[15] = 0x54; // 探针固件中该指令处 PC
        // MOVS.W r0, #0x8000 = 0xF45F 4000：结果必须为纯立即数 0x8000（QEMU 一致），与 PC 无关
        assert_eq!(h.exec_word(0xF45F_4000), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x0000_8000, "MOVS.W #0x8000 → 0x8000（QEMU 一致，不读 PC）");
        assert_eq!(h.nzcv(), 0, "N/Z/C/V 均清零（0x8000 非负非零）");
        // MOV.W r0, #0x8000 = 0xF44F 4000（S=0）：不置标志
        h.cpu.xpsr = 0x8000_0000; // N=1 预置，验证 S=0 不改标志
        h.exec_word(0xF44F_4000);
        assert_eq!(h.cpu.regs[0], 0x0000_8000);
        assert_eq!(h.nzcv(), 0b1000, "MOV.W 不置标志（N 保持）");
        // MOVS.W r15, #0x8000 = 0xF45F 4F80（Rd=1111, S=1, ARM 语义 UNPREDICTABLE）
        // → Unimplemented，绝不静默写 PC
        let outcome = h.exec_word(0xF45F_4F80);
        assert!(matches!(outcome, ExecOutcome::Fault { .. }), "MOVS.W r15 → Fault/Unimplemented，不得静默写 PC");
    }

    // ================= P1：FreeRTOS 前置指令补齐（FRT-INS，编码 as 实测）=================
    use crate::engine::test_util::Harness;

    /// CPSIE/CPSID（FRT-INS-01）：cpsie i=0xB662/cpsid i=0xB672/cpsie f=0xB661/cpsid f=0xB671
    #[test]
    fn p1_cpsie_cpsid_primask_faultmask() {
        let mut h = Harness::new();
        // GIVEN: 初始 PRIMASK/FAULTMASK = 0
        assert_eq!(h.cpu.primask, 0);
        assert_eq!(h.cpu.faultmask, 0);
        // WHEN: cpsid i（置 PRIMASK）→ cpsid f（置 FAULTMASK）
        assert_eq!(h.exec_halfword(0xB672), ExecOutcome::Continue);
        assert_eq!(h.exec_halfword(0xB671), ExecOutcome::Continue);
        // THEN: PRIMASK=1, FAULTMASK=1
        assert_eq!(h.cpu.primask, 1, "cpsid i 置 PRIMASK");
        assert_eq!(h.cpu.faultmask, 1, "cpsid f 置 FAULTMASK");
        // WHEN: cpsie i → cpsie f（清 PRIMASK/FAULTMASK）
        assert_eq!(h.exec_halfword(0xB662), ExecOutcome::Continue);
        assert_eq!(h.exec_halfword(0xB661), ExecOutcome::Continue);
        // THEN: 全部清零
        assert_eq!(h.cpu.primask, 0, "cpsie i 清 PRIMASK");
        assert_eq!(h.cpu.faultmask, 0, "cpsie f 清 FAULTMASK");
    }

    /// DSB/ISB/DMB（FRT-INS-02）：dsb=0xF3BF 8F4F / isb=0xF3BF 8F6F / dmb=0xF3BF 8F5F
    /// 单核顺序模拟语义 = 无操作（不 fault、不改变可观测状态）
    #[test]
    fn p1_barriers_noop() {
        let mut h = Harness::new();
        h.cpu.regs[0] = 0x1234_5678;
        let before_xpsr = h.cpu.xpsr;
        // WHEN: 依次执行三种屏障（含 DSB 域变体 0x8F4F）
        for word in [0xF3BF_8F4Fu32, 0xF3BF_8F5F, 0xF3BF_8F6F] {
            assert_eq!(h.exec_word(word), ExecOutcome::Continue, "{word:#010x} 不应 fault");
        }
        // THEN: 无任何可观测状态变化（寄存器/xPSR 保持）
        assert_eq!(h.cpu.regs[0], 0x1234_5678);
        assert_eq!(h.cpu.xpsr, before_xpsr);
    }

    /// 32 位 LDM/STM 全家族（FRT-INS-03）：IA/DB + 回写 + r14/PC
    /// 编码（as 实测）：stmia.w r0!,{r4-r11,lr}=0xE8A0 4FF0；ldmia.w=0xE8B0 4FF0；
    /// stmdb r0!,{..}=0xE920 4FF0；ldmdb r0!,{..}=0xE930 4FF0；pop {r4-r11,pc}=0xE8BD 8FF0
    #[test]
    fn p1_ldm_stm_32bit_family() {
        let mut h = Harness::new();
        // ---- STMIA.W r0!, {r4-r11, r14}：9 字递增存储，回写 r0 = r0+36 ----
        h.cpu.regs[0] = 0x2000_0000;
        for (i, reg) in [4u8, 5, 6, 7, 8, 9, 10, 11, 14].iter().enumerate() {
            h.cpu.regs[*reg as usize] = 0x1000 + i as u32;
        }
        assert_eq!(h.exec_word(0xE8A0_4FF0), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x2000_0024, "STMIA 回写 = base+4×9");
        assert_eq!(h.mem.read_u32(0x2000_0000).unwrap(), 0x1000, "r4");
        assert_eq!(h.mem.read_u32(0x2000_0020).unwrap(), 0x1008, "r14 在最后槽");

        // ---- LDMIA.W r0!, {r4-r11, r14}：递增加载恢复 + 回写 ----
        for i in 0..16 {
            h.cpu.regs[i] = 0;
        }
        h.cpu.regs[0] = 0x2000_0000;
        assert_eq!(h.exec_word(0xE8B0_4FF0), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[4], 0x1000);
        assert_eq!(h.cpu.regs[11], 0x1007);
        assert_eq!(h.cpu.regs[14], 0x1008);
        assert_eq!(h.cpu.regs[0], 0x2000_0024, "LDMIA 回写");

        // ---- STMDB r0!, {r4-r11, r14}：先减后存，起始 = base-36，回写 r0 = base-36 ----
        h.cpu.regs[0] = 0x2000_0024;
        assert_eq!(h.exec_word(0xE920_4FF0), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x2000_0000, "STMDB 回写 = base-4×9");
        assert_eq!(h.mem.read_u32(0x2000_0000).unwrap(), 0x1000, "r4 在最低地址");
        assert_eq!(h.mem.read_u32(0x2000_0020).unwrap(), 0x1008, "r14 在最高槽");

        // ---- LDMDB r0!, {r4-r11, r14}：从 base-36 起递增加载，回写 = base-36 ----
        for i in 0..16 {
            h.cpu.regs[i] = 0;
        }
        h.cpu.regs[0] = 0x2000_0024;
        assert_eq!(h.exec_word(0xE930_4FF0), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[4], 0x1000);
        assert_eq!(h.cpu.regs[14], 0x1008);
        assert_eq!(h.cpu.regs[0], 0x2000_0000, "LDMDB 回写");

        // ---- LDMIA.W sp!, {r4-r11, pc}（POP 32 位等价，0xE8BD 8FF0）：PC 按 Branch 语义 ----
        h.cpu.regs[13] = 0x2000_0100;
        for i in 0..16 {
            h.cpu.regs[i] = 0;
        }
        h.cpu.regs[13] = 0x2000_0100;
        // 预置栈内容：r4=0x1111, r5=0x2222, ..., r11=0x8888, pc 槽=0x0800_0001（T 位）
        for (j, val) in [0x1111u32, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666, 0x7777, 0x8888, 0x0800_0001]
            .iter()
            .enumerate()
        {
            h.mem.write_u32(0x2000_0100 + (j as u32) * 4, *val).unwrap();
        }
        let out = h.exec_word(0xE8BD_8FF0);
        assert!(
            matches!(out, ExecOutcome::Branch { target: 0x0800_0000 }),
            "LDM 含 pc → Branch 语义清 T 位"
        );
        assert_eq!(h.cpu.regs[4], 0x1111);
        // PC 由引擎按 Branch outcome 应用（Harness 不写 PC，与 Ldm 既有语义一致）
        assert_eq!(h.cpu.regs[13], 0x2000_0124, "SP 回写 = base+4×9");
    }

    /// CLZ（FRT-INS-04）：clz r0,r1 = 0xFAB1 F081（前导零计数）
    #[test]
    fn p1_clz() {
        let mut h = Harness::new();
        // 0x0000_0001 → 31 个前导零
        h.cpu.regs[1] = 0x0000_0001;
        assert_eq!(h.exec_word(0xFAB1_F081), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 31);
        // 0x8000_0000 → 0
        h.cpu.regs[1] = 0x8000_0000;
        assert_eq!(h.exec_word(0xFAB1_F081), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0);
        // 0 → 32
        h.cpu.regs[1] = 0;
        assert_eq!(h.exec_word(0xFAB1_F081), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 32);
        // 0x00FF_0000 → 8
        h.cpu.regs[1] = 0x00FF_0000;
        assert_eq!(h.exec_word(0xFAB1_F081), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 8);
    }

    /// REV/REV16/REVSH（FRT-INS-05 SHOULD）：rev=0xBA08/rev16=0xBA48/revsh=0xBAC8
    #[test]
    fn p1_rev_family() {
        let mut h = Harness::new();
        // REV：整字字节反转
        h.cpu.regs[1] = 0xAABB_CCDD;
        assert_eq!(h.exec_halfword(0xBA08), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xDDCC_BBAA, "REV 整字反转");
        // REV16：半字内字节反转
        h.cpu.regs[1] = 0xAABB_CCDD;
        assert_eq!(h.exec_halfword(0xBA48), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xBBAA_DDCC, "REV16 半字内反转");
        // REVSH：低半字反转 + 符号扩展（0x0000_00DD → 0xDD00 → 符号扩展 0xFFFF_DD00）
        h.cpu.regs[1] = 0x0000_00DD;
        assert_eq!(h.exec_halfword(0xBAC8), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xFFFF_DD00u32, "REVSH 符号扩展");
        // 正值：0x0000_00CD → 0xCD00（bit15=1 → 仍为负）；0x0000_004D → 0x4D00（正）
        h.cpu.regs[1] = 0x0000_004D;
        assert_eq!(h.exec_halfword(0xBAC8), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x0000_4D00, "REVSH 正值符号扩展");
    }

    /// UBFX/SBFX/BFI/BFC（FRT-INS-05 SHOULD）：
    /// ubfx r0,r1,#3,#5=0xF3C1 00C4；sbfx=0xF341 00C4；bfi r0,r1,#3,#5=0xF361 00C7；bfc=0xF36F 00C7
    #[test]
    fn p1_bitfield_ops() {
        let mut h = Harness::new();
        // UBFX r0, r1, #3, #5：提取 bits[7:3] 无符号
        h.cpu.regs[1] = 0x0000_00F8; // bits[7:3] = 11111
        assert_eq!(h.exec_word(0xF3C1_00C4), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x1F);
        h.cpu.regs[1] = 0xFFFF_FFFF;
        assert_eq!(h.exec_word(0xF3C1_00C4), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x1F, "UBFX 无符号：全 1 只取 5 位");

        // SBFX r0, r1, #3, #5：符号扩展
        h.cpu.regs[1] = 0xFFFF_FFFF;
        assert_eq!(h.exec_word(0xF341_00C4), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xFFFF_FFFFu32, "SBFX 全 1 → 符号扩展后仍全 1");
        h.cpu.regs[1] = 0x0000_0018; // bits[7:3] = 00011 → 3（符号位 0）
        assert_eq!(h.exec_word(0xF341_00C4), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 3);

        // BFI r0, r1, #3, #5：Rd[7:3] = Rn[4:0]
        h.cpu.regs[0] = 0xFFFF_FFFF;
        h.cpu.regs[1] = 0x0000_0000;
        assert_eq!(h.exec_word(0xF361_00C7), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xFFFF_FF07, "BFI 清 bits[7:3] 后插入 0");
        h.cpu.regs[1] = 0x0000_001F; // 5 位全 1
        assert_eq!(h.exec_word(0xF361_00C7), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xFFFF_FFFF, "BFI 插入全 1");

        // BFC r0, #3, #5：清除 bits[7:3]
        h.cpu.regs[0] = 0xFFFF_FFFF;
        assert_eq!(h.exec_word(0xF36F_00C7), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xFFFF_FF07, "BFC 清除 bits[7:3]");
    }

    /// LDREX/STREX 家族（FRT-INS-05 SHOULD，单核语义）：
    /// ldrex r0,[r1]=0xE851 0F00；strex r0,r1,[r2]=0xE842 1000；
    /// ldrexb r4,[r5]=0xE8D5 4F4F；strexb r6,r7,[r8]=0xE8C8 7F46；
    /// ldrexh r9,[r10]=0xE8DA 9F5F；strexh r11,r12,[r0]=0xE8C0 CF5B
    #[test]
    fn p1_ldrex_strex() {
        let mut h = Harness::new();
        // LDREX（字）：读内存
        h.cpu.regs[1] = 0x2000_0000;
        h.mem.write_u32(0x2000_0000, 0xDEAD_BEEF).unwrap();
        assert_eq!(h.exec_word(0xE851_0F00), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xDEAD_BEEF);
        // STREX（字）：写内存 + Rd=0（独占成功）
        h.cpu.regs[2] = 0x2000_0000;
        h.cpu.regs[1] = 0x1122_3344;
        assert_eq!(h.exec_word(0xE842_1000), ExecOutcome::Continue);
        assert_eq!(h.mem.read_u32(0x2000_0000).unwrap(), 0x1122_3344, "STREX 写入");
        assert_eq!(h.cpu.regs[0], 0, "STREX 单核语义成功 → Rd=0");
        // LDREXH（半字）：0xE8DA 9F5F
        h.cpu.regs[10] = 0x2000_0000;
        h.mem.write_u16(0x2000_0000, 0xABCD).unwrap();
        assert_eq!(h.exec_word(0xE8DA_9F5F), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[9], 0xABCD);
        // STREXH（半字）：0xE8C0 CF5B → r12 数据写入 [r0]，r11=0
        h.cpu.regs[0] = 0x2000_0000;
        h.cpu.regs[12] = 0x0000_CDEF;
        assert_eq!(h.exec_word(0xE8C0_CF5B), ExecOutcome::Continue);
        assert_eq!(h.mem.read_u16(0x2000_0000).unwrap(), 0xCDEF);
        assert_eq!(h.cpu.regs[11], 0);
        // LDREXB（字节）：0xE8D5 4F4F
        h.cpu.regs[5] = 0x2000_0000;
        h.mem.write_u8(0x2000_0000, 0x7F).unwrap();
        assert_eq!(h.exec_word(0xE8D5_4F4F), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[4], 0x7F);
        // STREXB（字节）：0xE8C8 7F46 → r7 数据写入 [r8]，r6=0
        h.cpu.regs[8] = 0x2000_0000;
        h.cpu.regs[7] = 0x5A;
        assert_eq!(h.exec_word(0xE8C8_7F46), ExecOutcome::Continue);
        assert_eq!(h.mem.read_u8(0x2000_0000).unwrap(), 0x5A);
        assert_eq!(h.cpu.regs[6], 0);
    }

    /// UMULL/SMULL/SMLAL（FRT-INS-05 SHOULD，64 位长乘）：
    /// umull r0,r1,r2,r3=0xFBA2 0103；smull r4,r5,r6,r7=0xFB86 4507；
    /// smlal r0,r1,r2,r3=0xFBC2 0103（编码 as 实测）
    #[test]
    fn p1_mull_long() {
        let mut h = Harness::new();
        // UMULL：0xFFFFFFFF × 0xFFFFFFFF = 0xFFFFFFFE_00000001
        h.cpu.regs[2] = 0xFFFF_FFFF;
        h.cpu.regs[3] = 0xFFFF_FFFF;
        assert_eq!(h.exec_word(0xFBA2_0103), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x0000_0001, "UMULL 低 32 位");
        assert_eq!(h.cpu.regs[1], 0xFFFF_FFFE, "UMULL 高 32 位");
        // SMULL：(-1) × (-1) = 1
        h.cpu.regs[6] = 0xFFFF_FFFF;
        h.cpu.regs[7] = 0xFFFF_FFFF;
        assert_eq!(h.exec_word(0xFB86_4507), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[4], 1, "SMULL 低 32 位");
        assert_eq!(h.cpu.regs[5], 0, "SMULL 高 32 位");
        // SMULL：0x80000000 × 2（有符号 -2^31 × 2 = -2^32 → 0xFFFFFFFF_00000000）
        h.cpu.regs[6] = 0x8000_0000;
        h.cpu.regs[7] = 2;
        assert_eq!(h.exec_word(0xFB86_4507), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[4], 0, "SMULL 低 32 位（负数积）");
        assert_eq!(h.cpu.regs[5], 0xFFFF_FFFF, "SMULL 高 32 位（符号填充）");
        // SMLAL：acc[1:0] += 0x100 × 0x100；acc 初始 = 5
        h.cpu.regs[0] = 5;
        h.cpu.regs[1] = 0;
        h.cpu.regs[2] = 0x100;
        h.cpu.regs[3] = 0x100;
        assert_eq!(h.exec_word(0xFBC2_0103), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x0001_0005, "SMLAL 累加低 32 位");
        assert_eq!(h.cpu.regs[1], 0, "SMLAL 累加高 32 位");
    }

    /// LdrStrWb（FRT-INS-03 同族）：前变址 [Rn,#imm]! / 后变址 [Rn],#imm
    /// 编码 as 实测：ldr.w r0,[r1,#8]!=0xF851 0F08；str.w r2,[r3,#12]!=0xF843 2F0C；
    /// ldr.w r0,[r1],#8=0xF851 0B08；str.w r2,[r3],#12=0xF843 2B0C；
    /// ldrh.w r0,[r1,#4]!=0xF831 0F04；ldrsh.w r0,[r1,#6]!=0xF931 0F06；
    /// ldrb.w r0,[r1,#2]!=0xF811 0F02；ldrsb.w r0,[r1,#3]!=0xF911 0F03
    #[test]
    fn p1_ldr_str_writeback() {
        let mut h = Harness::new();
        // 前变址 LDR.W [r1,#8]!：addr=base+8，回写 r1=addr
        h.cpu.regs[1] = 0x2000_0000;
        h.mem.write_u32(0x2000_0008, 0xCAFE_BABE).unwrap();
        assert_eq!(h.exec_word(0xF851_0F08), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xCAFE_BABE, "前变址 LDR 读值");
        assert_eq!(h.cpu.regs[1], 0x2000_0008, "前变址回写 rn=addr");

        // 前变址 STR.W [r3,#12]!：addr=base+12，回写 r3=addr
        h.cpu.regs[3] = 0x2000_0100;
        h.cpu.regs[2] = 0x1122_3344;
        assert_eq!(h.exec_word(0xF843_2F0C), ExecOutcome::Continue);
        assert_eq!(h.mem.read_u32(0x2000_010C).unwrap(), 0x1122_3344, "前变址 STR 写入");
        assert_eq!(h.cpu.regs[3], 0x2000_010C, "前变址 STR 回写");

        // 后变址 LDR.W [r1],#8：addr=base 访存，回写 r1=base+8
        h.cpu.regs[1] = 0x2000_0020;
        h.mem.write_u32(0x2000_0020, 0xDEAD_BEEF).unwrap();
        assert_eq!(h.exec_word(0xF851_0B08), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xDEAD_BEEF, "后变址 LDR 读 base 处");
        assert_eq!(h.cpu.regs[1], 0x2000_0028, "后变址回写 rn=base+imm");

        // 后变址 STR.W [r3],#12：addr=base 访存，回写 r3=base+12
        h.cpu.regs[3] = 0x2000_0100;
        h.cpu.regs[2] = 0x5566_7788;
        assert_eq!(h.exec_word(0xF843_2B0C), ExecOutcome::Continue);
        assert_eq!(h.mem.read_u32(0x2000_0100).unwrap(), 0x5566_7788, "后变址 STR 写 base 处");
        assert_eq!(h.cpu.regs[3], 0x2000_010C, "后变址 STR 回写");

        // 前变址 LDRH.W [r1,#4]!：半字零扩展
        h.cpu.regs[1] = 0x2000_0030;
        h.mem.write_u16(0x2000_0034, 0xABCD).unwrap();
        assert_eq!(h.exec_word(0xF831_0F04), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xABCD, "LDRH 零扩展");
        assert_eq!(h.cpu.regs[1], 0x2000_0034, "LDRH 回写");

        // 前变址 LDRSH.W [r1,#6]!：半字符号扩展（0x8000 → 0xFFFF8000）
        h.cpu.regs[1] = 0x2000_0040;
        h.mem.write_u16(0x2000_0046, 0x8000).unwrap();
        assert_eq!(h.exec_word(0xF931_0F06), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xFFFF_8000, "LDRSH 符号扩展");
        assert_eq!(h.cpu.regs[1], 0x2000_0046, "LDRSH 回写");

        // 前变址 LDRB.W [r1,#2]!：字节零扩展
        h.cpu.regs[1] = 0x2000_0050;
        h.mem.write_u8(0x2000_0052, 0x7F).unwrap();
        assert_eq!(h.exec_word(0xF811_0F02), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x7F, "LDRB 零扩展");
        assert_eq!(h.cpu.regs[1], 0x2000_0052, "LDRB 回写");

        // 前变址 LDRSB.W [r1,#3]!：字节符号扩展（0x80 → 0xFFFFFF80）
        h.cpu.regs[1] = 0x2000_0060;
        h.mem.write_u8(0x2000_0063, 0x80).unwrap();
        assert_eq!(h.exec_word(0xF911_0F03), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xFFFF_FF80, "LDRSB 符号扩展");
        assert_eq!(h.cpu.regs[1], 0x2000_0063, "LDRSB 回写");
    }

    // ============ P5 WIP：32 位数据处理（寄存器）/ RSB / 负偏移存取 ============
    // 编码与 arm-none-eabi-as 实测一致（见 decode_data_proc_reg_32bit_wip）。

    /// ADD/SUB（寄存器 LSL#n 移位形式）：add.w lr,r1,r0,lsl #2 = 0xEB01 0E80；
    /// sub.w r2,r3,r4,lsl #1 = 0xEBA3 0244
    #[test]
    fn wip_add_sub_shifted_exec() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // add.w lr, r1, r0, lsl #2：0x10 + (3 << 2) = 0x1C
        h.cpu.regs[1] = 0x10;
        h.cpu.regs[0] = 3;
        assert_eq!(h.exec_word(0xEB01_0E80), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[14], 0x1C, "ADD 带 LSL#2 移位");
        // sub.w r2, r3, r4, lsl #1：0x20 - (3 << 1) = 0x1A
        h.cpu.regs[3] = 0x20;
        h.cpu.regs[4] = 3;
        assert_eq!(h.exec_word(0xEBA3_0244), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[2], 0x1A, "SUB 带 LSL#1 移位");
        // ADD 带 S：0x7FFFFFFF + (1<<2) 溢出 → N=1 V=1
        h.cpu.regs[1] = 0x7FFF_FFFF;
        h.cpu.regs[0] = 1;
        assert_eq!(h.exec_word(0xEB11_0E80), ExecOutcome::Continue); // S=1（bit20）
        assert_eq!(h.cpu.regs[14], 0x8000_0003, "0x7FFFFFFF + (1<<2)");
        assert_eq!(h.nzcv(), 0b1001, "正+正溢出：N=1 V=1 C=0");
    }

    /// RSB：立即数 rsb r3,r0,#1 = 0xF1C0 0301（1 - 0x10 = 0xFFFFFFF1）；
    /// 寄存器 rsb r5,r1,r2 = 0xEBC1 0502（5 - 0x20 = 0xFFFFFFE5）
    #[test]
    fn wip_rsb_exec() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.regs[0] = 0x10;
        assert_eq!(h.exec_word(0xF1C0_0301), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[3], 0xFFFF_FFF1, "RSB imm：imm - Rn");
        // RSBS 标志：1 - 0x10 = -15 → N=1；1 < 16 无符号借位 → C=0
        h.cpu.regs[0] = 0x10;
        assert_eq!(h.exec_word(0xF1C0_0301 | 0x0010_0000), ExecOutcome::Continue);
        assert_eq!(h.nzcv(), 0b1000, "RSBS：N=1 C=0（无符号借位）");
        h.cpu.regs[1] = 0x20;
        h.cpu.regs[2] = 5;
        assert_eq!(h.exec_word(0xEBC1_0502), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[5], 0xFFFF_FFE5, "RSB reg：Rm - Rn");
    }

    /// LDR/STR.W 负立即数偏移：str.w r3,[r0,#-4]=0xF840 3C04 写 [base-4]、
    /// ldr.w=0xF850 3C04 读回
    #[test]
    fn wip_ldr_str_negative_offset_exec() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.regs[0] = 0x2000_0004;
        h.cpu.regs[3] = 0xDEAD_BEEF;
        assert_eq!(h.exec_word(0xF840_3C04), ExecOutcome::Continue);
        assert_eq!(h.mem.read_u32(0x2000_0000).unwrap(), 0xDEAD_BEEF, "STR 负偏移");
        assert_eq!(h.cpu.regs[0], 0x2000_0004, "STR 不回写基址");
        assert_eq!(h.exec_word(0xF850_3C04), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[3], 0xDEAD_BEEF, "LDR 负偏移读回");
    }

    /// LDR.W 寄存器偏移 + LSL#n：ldr.w r3,[r1,r0,lsl #2] = 0xF851 3020
    /// → addr = r1 + (r0 << 2)
    #[test]
    fn wip_ldr_reg_shifted_offset_exec() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x2000_0010;
        h.cpu.regs[0] = 4; // 4 << 2 = 16
        h.mem.write_u32(0x2000_0020, 0xCAFE_F00D).unwrap();
        assert_eq!(h.exec_word(0xF851_3020), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[3], 0xCAFE_F00D, "LDR 寄存器移位偏移");
    }

    /// 32 位数据处理的 S=1 标志语义抽查：mvn.w 与 adc.w/sbc.w（寄存器形式）
    #[test]
    fn wip_data_proc_reg_flags() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // MVNS.W r0, r1 = 0xEA6F 0001 | S=1 → ~0x0F00 = 0xFFFFF0FF → N=1
        h.cpu.regs[1] = 0x0F00;
        assert_eq!(h.exec_word(0xEA6F_0001 | 0x0010_0000), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xFFFF_F0FF);
        assert_eq!(h.nzcv(), 0b1000, "MVNS：N=1");
        // ADCS.W r6, r7, r8 = 0xEB47 0608 | S=1：1 + 1 + C(1) = 3
        h.cpu.regs[7] = 1;
        h.cpu.regs[8] = 1;
        h.cpu.xpsr = 1 << 29;
        assert_eq!(h.exec_word(0xEB47_0608 | 0x0010_0000), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[6], 3);
        assert_eq!(h.nzcv(), 0, "ADCS：1+1+C=3，无进位出");
        // SBCS.W r9, r10, r11 = 0xEB6A 090B | S=1：a - b - ~C，C=1 → 5 - 2 - 0 = 3
        h.cpu.regs[10] = 5;
        h.cpu.regs[11] = 2;
        h.cpu.xpsr = 1 << 29; // C=1 → ~C=0 不借位
        assert_eq!(h.exec_word(0xEB6A_090B | 0x0010_0000), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[9], 3, "SBC：a - b - ~C，C=1 → 5-2-0=3");
        assert_eq!(h.nzcv(), 0b0010, "SBCS：无借位 C=1，N/Z 清零");
    }

    // ================= P4 FreeRTOS：SXTH/SXTB/UXTH/UXTB（16 位 T1）=================
    // 编码 as 实测：sxth r2,r3=0xB21A / sxtb=0xB25A / uxth=0xB29A / uxtb=0xB2DA
    #[test]
    fn freertos_extend_16bit() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.regs[3] = 0xFFFF_80FF;
        // SXTH r2, r3：低半字符号扩展 → 0xFFFF80FF 低 16 位 = 0x80FF → 0xFFFF80FF
        assert_eq!(h.exec_halfword(0xB21A), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[2], 0xFFFF_80FF, "SXTH：0x80FF 符号扩展");
        // SXTB r2, r3：低字节 0xFF 符号扩展 → 0xFFFFFFFF
        assert_eq!(h.exec_halfword(0xB25A), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[2], 0xFFFF_FFFF, "SXTB：0xFF 符号扩展");
        // UXTH r2, r3：低半字零扩展 → 0x80FF
        assert_eq!(h.exec_halfword(0xB29A), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[2], 0x0000_80FF, "UXTH：零扩展");
        // UXTB r3, r3（固件实测 0xB2DB = uxtb r3,r3）：低字节零扩展 → 0xFF
        assert_eq!(h.exec_halfword(0xB2DB), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[3], 0x0000_00FF, "UXTB：零扩展");
    }

    // ================= P4 FreeRTOS：32 位寄存器移位（LSL.W/LSR.W/ASR.W/ROR.W）=================
    // 编码 as 实测：lsl.w r1,r7,r1=0xFA07 F101 / lsr.w=0xFA23 F204 /
    // asr.w=0xFA46 F507 / ror.w=0xFA61 F002（移位量 = Rs[7:0]）
    #[test]
    fn freertos_shift_register_32bit() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // LSL.W r1, r7, r1：0x80000001 << 1 = 0x00000002
        h.cpu.regs[7] = 0x8000_0001;
        h.cpu.regs[1] = 1;
        assert_eq!(h.exec_word(0xFA07_F101), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[1], 0x0000_0002, "LSL.W 寄存器移位量");
        // LSR.W r2, r3, r4：0x80000000 >> 4 = 0x08000000
        h.cpu.regs[3] = 0x8000_0000;
        h.cpu.regs[4] = 4;
        assert_eq!(h.exec_word(0xFA23_F204), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[2], 0x0800_0000, "LSR.W");
        // ASR.W r5, r6, r7：0x80000000 算术右移 4 = 0xF8000000
        h.cpu.regs[6] = 0x8000_0000;
        h.cpu.regs[7] = 4;
        assert_eq!(h.exec_word(0xFA46_F507), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[5], 0xF800_0000, "ASR.W 符号填充");
        // ROR.W r0, r1, r2：0x80000001 循环右移 1 = 0xC0000000
        h.cpu.regs[1] = 0x8000_0001;
        h.cpu.regs[2] = 1;
        assert_eq!(h.exec_word(0xFA61_F002), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xC000_0000, "ROR.W");
    }

    // ================= P4 FreeRTOS：前变址负偏移回写 [Rn, #-imm8]! ================
    // 编码 as 实测：strb.w r2,[ip,#-1]! = 0xF80C 2D01（print_num 栈缓冲回写依赖）
    #[test]
    fn freertos_strb_neg_pre_index_wb() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.regs[12] = 0x2000_0020;
        h.cpu.regs[2] = 0x41; // 'A'
        assert_eq!(h.exec_word(0xF80C_2D01), ExecOutcome::Continue);
        assert_eq!(h.mem.read_u8(0x2000_001F).unwrap(), 0x41, "写入 [ip-1]");
        assert_eq!(h.cpu.regs[12], 0x2000_001F, "前变址回写 ip -= 1");
        // 无回写形式 [ip, #-1] = 0xF80C 2C01：地址正确但 ip 不变
        h.cpu.regs[12] = 0x2000_0030;
        assert_eq!(h.exec_word(0xF80C_2C01), ExecOutcome::Continue);
        assert_eq!(h.mem.read_u8(0x2000_002F).unwrap(), 0x41);
        assert_eq!(h.cpu.regs[12], 0x2000_0030, "无回写形式 ip 不变");
        // 加载方向 ldrb.w r2,[ip,#-1]! = 0xF81C 2D01
        h.mem.write_u8(0x2000_004E, 0x7F).unwrap();
        h.cpu.regs[12] = 0x2000_004F;
        assert_eq!(h.exec_word(0xF81C_2D01), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[2], 0x7F, "LDRB 前变址负偏移");
        assert_eq!(h.cpu.regs[12], 0x2000_004E, "回写");
    }

    // ================= P4 语义修正：16 位 ADDS/SUBS 恒置标志 =================
    // （旧 A9 逻辑误将 0x1800-0x187F ADD 寄存器形式置 flags=false，导致 C 标志
    //  泄漏进后续 bcc → FreeRTOS 延迟列表插入选错 overflow 列表；ARMv7-M
    //  0x1800-0x1AFF 全部为隐式 S 的 ADDS/SUBS）
    #[test]
    fn freertos_adds_reg_sets_flags() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        h.cpu.xpsr = 1 << 29; // 预置 C=1，验证 ADDS 会重写
        h.cpu.regs[4] = 0xFFFF_FFFF;
        h.cpu.regs[6] = 1;
        // adds r4, r4, r6 = 0x19A4：0xFFFFFFFF + 1 = 0 → Z=1，C=1（进位出）
        assert_eq!(h.exec_halfword(0x19A4), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[4], 0);
        assert_eq!(h.nzcv(), 0b0110, "ADDS：Z=1 且 C=1（进位出）");
        // adds r4, r4, r6：0 + 1 = 1 → Z=0，C=0（无进位）——bcc 依赖此语义
        assert_eq!(h.exec_halfword(0x19A4), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[4], 1);
        assert_eq!(h.nzcv(), 0b0000, "ADDS：C=0（无进位）");
        // subs r3, r3, r2 = 0x1A9B：5 - 5 = 0 → Z=1，C=1（无借位）
        h.cpu.regs[3] = 5;
        h.cpu.regs[2] = 5;
        assert_eq!(h.exec_halfword(0x1A9B), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[3], 0);
        assert_eq!(h.nzcv(), 0b0110, "SUBS：Z=1 且 C=1");
    }

    // ================= P4 FreeRTOS：AddShifted/SubShifted（32 位寄存器 LSL#n）=================
    // 编码 as 实测：add.w r2,r3,r3,lsl #2 = 0xEB03 0283 / sub.w r2,r4,r2,lsl #1 = 0xEBA4 0242
    #[test]
    fn freertos_addsub_shifted_reg() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // add.w r2, r3, r3, lsl #2：4 + (4<<2) = 20
        h.cpu.regs[3] = 4;
        assert_eq!(h.exec_word(0xEB03_0283), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[2], 20, "ADD Rd, Rn, Rm, LSL#2");
        // sub.w r2, r4, r2, lsl #1：42 - (20<<1) = 2
        h.cpu.regs[4] = 42;
        assert_eq!(h.exec_word(0xEBA4_0242), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[2], 2, "SUB Rd, Rn, Rm, LSL#1");
    }

    // ================= P4 FreeRTOS：POP{..,pc} 装入 EXC_RETURN → 异常返回 ================
    // （ARMv7-M B1.5.6：任一 PC 装载 EXC_RETURN 即触发返回，不限于 BX；
    //  xPortSysTickHandler 的 pop {r3, pc} 返回路径依赖此语义）
    #[test]
    fn freertos_pop_pc_exc_return() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // 布置栈：SP 处 = 占位 r3，+4 处 = EXC_RETURN（线程+PSP = 0xFFFFFFFD）
        h.cpu.regs[13] = 0x2000_0800;
        h.mem.write_u32(0x2000_0800, 0x1111_1111).unwrap();
        h.mem.write_u32(0x2000_0804, 0xFFFF_FFFD).unwrap();
        let outcome = h.exec_halfword(0xBD08); // pop {r3, pc}
        assert_eq!(
            outcome,
            ExecOutcome::ExceptionReturn { exc_return: 0xFFFF_FFFD },
            "POP{{pc}} 装入 EXC_RETURN 应产生异常返回"
        );
        assert_eq!(h.cpu.regs[13], 0x2000_0808, "SP 已弹 8 字节");
        assert_eq!(h.cpu.regs[3], 0x1111_1111);
    }

    // ============ codex 检视修复（P2-1/P2-2/P3-1）golden 测试 ============

    /// P2-1：nop.w（0xF3AF 8000）执行 = 无操作（不得误解码为 bge.w 跳转）。
    /// 引擎 Nop 语义：PC 照常推进，无分支、无副作用。
    #[test]
    fn golden_nopw_executes_as_nop() {
        let mut h = crate::engine::test_util::Harness::new();
        h.cpu.regs[15] = 0x1000;
        // WHEN: nop.w（0xF3AF 8000）
        let out = h.exec_word(0xF3AF_8000);
        // THEN: 正常继续，非分支
        assert_eq!(out, ExecOutcome::Continue);
        // 解码层断言（防回归误解码为 bge.w）：
        assert_eq!(h.decoder.decode_word(0xF3AF_8000, 0x1000), Instruction::Nop);
        // wfe.w（0xF3AF 8003）同为 HINT → NOP
        assert_eq!(h.decoder.decode_word(0xF3AF_8003, 0x1000), Instruction::Nop);
    }

    /// P2-2：VLDMIA/VSTMDB 大寄存器列表（{s16-s31}，imm8=16）完整执行。
    /// 模拟 ARM_CM4F PendSV 上下文切换（vldmiaeq 0xECB0 8A10 / vstmdbeq 0xED20 8A10）：
    /// 写 16 个寄存器 → 栈上 16 字 → 读回一致；SP 回写/递减正确。
    #[test]
    fn golden_vldm_vstm_s16_s31_roundtrip() {
        let mut h = crate::engine::test_util::Harness::new();
        h.cpu.regs[0] = 0x2000_1000; // R0 基址（栈顶）
        // GIVEN: s16-s31 写满可辨识值
        for i in 0..16u32 {
            h.cpu.fpu.write_s((16 + i) as usize, 0x5A00_0000 + i);
        }
        // WHEN: vstmdbeq r0!, {s16-s31}（0xED20 8A10，DB+回写，先减后存）
        let out = h.exec_word(0xED20_8A10);
        assert_eq!(out, ExecOutcome::Continue);
        // THEN: SP 递减 16 字（0x40）且回写
        assert_eq!(h.cpu.regs[0], 0x2000_1000 - 0x40, "VSTMDB 回写 SP");
        // 栈内容与寄存器一致（s16 → 0x2000_0FC0）
        for i in 0..16u32 {
            let addr = 0x2000_0FC0 + i * 4;
            let v = h.mem.read_u32(addr).expect("栈读取");
            assert_eq!(v, 0x5A00_0000 + i, "栈[{addr:#x}] = s{}", 16 + i);
        }
        // WHEN: 清空寄存器后 vldmiaeq r0!, {s16-s31}（0xECB0 8A10，IA+回写）
        for i in 0..16u32 {
            h.cpu.fpu.write_s((16 + i) as usize, 0);
        }
        let out = h.exec_word(0xECB0_8A10);
        assert_eq!(out, ExecOutcome::Continue);
        // THEN: 寄存器恢复，SP 回到原值
        assert_eq!(h.cpu.regs[0], 0x2000_1000, "VLDMIA 回写 SP");
        for i in 0..16u32 {
            assert_eq!(
                h.cpu.fpu.read_s((16 + i) as usize),
                0x5A00_0000 + i,
                "s{} 恢复",
                16 + i
            );
        }
    }

    /// P2-2：VLDR/VSTR 大偏移（imm8=0x10 → +64 字节）执行。
    /// 编码 as 实测：vldr s0,[r1,#64]=0xED91 0A10 / vstr s0,[r1,#64]=0xED81 0A10。
    #[test]
    fn golden_vldr_vstr_large_offset_exec() {
        let mut h = crate::engine::test_util::Harness::new();
        h.cpu.regs[1] = 0x2000_0040;
        h.cpu.fpu.write_s(0, 0xDEAD_BEEF);
        // VSTR s0, [r1, #64]（0xED81 0A10）→ 0x2000_0040 + 0x40 = 0x2000_0080
        assert_eq!(h.exec_word(0xED81_0A10), ExecOutcome::Continue);
        assert_eq!(h.mem.read_u32(0x2000_0080).unwrap(), 0xDEAD_BEEF);
        // VLDR s0, [r1, #64]（0xED91 0A10）→ 读回
        h.cpu.fpu.write_s(0, 0);
        assert_eq!(h.exec_word(0xED91_0A10), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), 0xDEAD_BEEF);
    }

    /// P3-1：IT 块内 16 位隐式 S 移位指令不更新任何标志（含 C）。
    /// ARMv7-M B1.5.10：IT 块内 16 位 LSLS/LSRS 等标志被抑制——N/Z/C/V 全部保持。
    /// 走 Engine 全路径（cur_is_16bit 由 run 循环设置；Harness 直调 execute 不设该位）。
    /// 构造：C=1、Z=1（EQ 成立）→ lsleq r0,r0,#1（0x0040）：0x40000000 << 1 =
    /// 0x80000000（N 应置 1、Z 清 0、移出位 0 → C 清 0）但标志必须保持。
    #[test]
    fn golden_it_block_shift_preserves_c() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // GIVEN: C=1, Z=1（xpsr bit29/bit30），r0=0x40000000（LSL#1 → 0x80000000，移出位=0）
        h.cpu.xpsr = (1 << 29) | (1 << 30);
        h.cpu.regs[0] = 0x4000_0000;
        // 写入指令流：0xBF0C ite eq | 0x0040 lsleq r0,r0,#1（IT 块内条件后缀形式）
        for (i, b) in [0x0C, 0xBF, 0x40, 0x00].iter().enumerate() {
            h.mem.flash[i] = *b;
        }
        let mut nvic = crate::nvic::Nvic::new();
        let mut eng = crate::engine::Engine::new();
        // WHEN: ite + 块内 lsleq（EQ 成立，执行）
        for _ in 0..2 {
            assert_eq!(
                eng.step(&mut h.cpu, &mut h.mem, &mut nvic),
                crate::engine::EngineResult::Halted
            );
        }
        // THEN: 结果写入（r0 = 0x80000000），但标志全部保持：C 仍=1、Z 仍=1（不被更新）
        assert_eq!(h.cpu.regs[0], 0x8000_0000, "移位结果照常写入");
        assert_eq!(
            h.cpu.xpsr & ((1 << 29) | (1 << 30)),
            (1 << 29) | (1 << 30),
            "IT 块内移位不更新 C/Z（N/Z/C/V 全抑制）"
        );
        // 对照：块外同指令正常更新（结果 0x80000000 → N=1、Z=0、移出位 0 → C=0）
        let mut h2 = Harness::new();
        h2.cpu.xpsr = (1 << 29) | (1 << 30);
        h2.cpu.regs[0] = 0x4000_0000;
        assert_eq!(h2.exec_halfword(0x0040), ExecOutcome::Continue);
        assert_eq!(h2.cpu.xpsr & (1 << 29), 0, "块外 LSLS 正常更新 C=0");
        assert_eq!(h2.cpu.xpsr & (1 << 30), 0, "块外 LSLS 正常更新 Z=0");
        assert_eq!(h2.cpu.xpsr & (1 << 31), 1 << 31, "块外 LSLS 正常更新 N=1");
    }

    /// P3-1：IT 块内 16 位 SBC（隐式 S）不更新 C/V——与 Shift 同病修复。
    /// 编码 as 实测：sbceq r0, r1 = 0x4188（IT 块内条件后缀形式，编码与 sbcs 相同）。
    /// SBC 语义：Rd = Rd - Rm - NOT(C)。构造初始 C=0：5-3-NOT(0)=5-3-1=1 无借位
    /// （a >= b+NOT(C) → 5>=4）→ 块外 C 应更新为 1；IT 块内 C 必须保持 0。
    /// 走 Engine 全路径（cur_is_16bit 由 run 循环设置）。
    #[test]
    fn golden_it_block_sbc_preserves_flags() {
        use crate::engine::test_util::Harness;
        let mut h = Harness::new();
        // GIVEN: 初始标志 N=1,Z=1,C=0,V=1（EQ 成立：Z=1；全部非默认，可观测泄漏）
        h.cpu.xpsr = (1 << 31) | (1 << 30) | (1 << 28); // N=1, Z=1, C=0, V=1
        h.cpu.regs[0] = 5;
        h.cpu.regs[1] = 3;
        // 写入指令流：0xBF0C ite eq | 0x4188 sbceq r0,r1（IT 块内条件后缀形式）
        for (i, b) in [0x0C, 0xBF, 0x88, 0x41].iter().enumerate() {
            h.mem.flash[i] = *b;
        }
        let mut nvic = crate::nvic::Nvic::new();
        let mut eng = crate::engine::Engine::new();
        // WHEN: ite + 块内 sbceq（EQ 成立，执行）
        for _ in 0..2 {
            assert_eq!(
                eng.step(&mut h.cpu, &mut h.mem, &mut nvic),
                crate::engine::EngineResult::Halted
            );
        }
        // THEN: 结果写入（5-3-1=1），标志保持初始值（C 不置 1、N/Z/V 不动）
        assert_eq!(h.cpu.regs[0], 1);
        assert_eq!(
            h.cpu.xpsr,
            (1 << 31) | (1 << 30) | (1 << 28),
            "IT 块内 SBC 不更新任何标志"
        );
        // 对照：块外同指令正常更新（无借位 → C=1、结果 1 → N=0/Z=0/V=0）
        let mut h2 = Harness::new();
        h2.cpu.xpsr = (1 << 31) | (1 << 30) | (1 << 28);
        h2.cpu.regs[0] = 5;
        h2.cpu.regs[1] = 3;
        assert_eq!(h2.exec_halfword(0x4188), ExecOutcome::Continue);
        assert_eq!(h2.cpu.regs[0], 1);
        assert_eq!(h2.cpu.xpsr & (1 << 29), 1 << 29, "块外 SBC 正常更新 C=1（无借位）");
    }
}
