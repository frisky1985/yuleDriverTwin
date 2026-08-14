//! 指令执行 — ARMv7E-M (Cortex-M4F) 执行器
//!
//! 基于 Decoder 输出的统一指令表示执行，更新 CPU 状态。
//! Phase 1: 核心整数指令（数据传送/算术逻辑/移位/分支/压栈）

use super::decode::{AccessWidth, Cond, Instruction, LoadStoreOffset, ShiftAmount, ShiftKind, SpecialReg};
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
            Instruction::Add { rd, rn, rm, imm, flags } => {
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
            Instruction::Sub { rd, rn, rm, imm, flags } => {
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
                cpu.regs[*rd as usize] = if divisor == 0 { 0 } else { cpu.regs[*rn as usize] / divisor };
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
            Instruction::Shift { rd, rm, kind, amount, flags } => {
                let val = cpu.regs[*rm as usize];
                let result = match amount {
                    ShiftAmount::Immediate(n) => self.shift_val(val, *kind, *n),
                    ShiftAmount::Register(r) => self.shift_val(val, *kind, (cpu.regs[*r as usize] & 0xFF) as u8),
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
                    ExecOutcome::Fault { reason: super::FaultReason::UsageFault { address: target } }
                } else {
                    ExecOutcome::Branch { target: target & !1 }
                }
            }
            Instruction::BranchLinkExchange { rm } => {
                let target = cpu.regs[*rm as usize];
                cpu.regs[14] = cpu.regs[15] - 1;
                if target & 1 == 0 {
                    ExecOutcome::Fault { reason: super::FaultReason::UsageFault { address: target } }
                } else {
                    ExecOutcome::Branch { target: target & !1 }
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
                            return ExecOutcome::Fault { reason: super::FaultReason::MemManage { address: addr } };
                        }
                        addr += 4;
                    }
                }
                if *lr {
                    let val = cpu.regs[14];
                    if let Err(_f) = memory.write_u32(addr, val) {
                        return ExecOutcome::Fault { reason: super::FaultReason::MemManage { address: addr } };
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
                                return ExecOutcome::Fault { reason: super::FaultReason::BusFault { address: addr } }
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
                            return ExecOutcome::Fault { reason: super::FaultReason::BusFault { address: addr } }
                        }
                    };
                    cpu.regs[15] = val & !1; // 清 Thumb 位
                    count += 1;
                }
                cpu.regs[13] = sp.wrapping_add(count * 4);
                ExecOutcome::Continue
            }
            Instruction::Ldm { rn, regs, writeback } => {
                let mut addr = cpu.regs[*rn as usize];
                let mut last = 0u32;
                for i in 0..16 {
                    if regs & (1 << i) != 0 {
                        let val = match memory.read_u32(addr) {
                            Ok(v) => v,
                            Err(_f) => {
                                return ExecOutcome::Fault { reason: super::FaultReason::BusFault { address: addr } }
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
            Instruction::Stm { rn, regs, writeback } => {
                let base = cpu.regs[*rn as usize];
                let mut addr = base;
                for i in 0..16 {
                    if regs & (1 << i) != 0 {
                        let val = cpu.regs[i];
                        if let Err(_f) = memory.write_u32(addr, val) {
                            return ExecOutcome::Fault { reason: super::FaultReason::MemManage { address: addr } };
                        }
                        addr += 4;
                    }
                }
                if *writeback {
                    cpu.regs[*rn as usize] = addr;
                }
                ExecOutcome::Continue
            }
            Instruction::Ldr { rt, rn, offset, width } => {
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
                    Err(_f) => ExecOutcome::Fault { reason: super::FaultReason::BusFault { address: addr } },
                }
            }
            Instruction::Str { rt, rn, offset, width } => {
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
                    Err(_f) => ExecOutcome::Fault { reason: super::FaultReason::MemManage { address: addr } },
                }
            }
            Instruction::LdrLiteral { rt, imm } => {
                let addr = cpu.regs[15].wrapping_add(*imm) & !3;
                match memory.read_u32(addr) {
                    Ok(v) => {
                        cpu.regs[*rt as usize] = v;
                        ExecOutcome::Continue
                    }
                    Err(_f) => ExecOutcome::Fault { reason: super::FaultReason::BusFault { address: addr } },
                }
            }
            Instruction::MsrMrs { .. } => ExecOutcome::Fault {
                reason: super::FaultReason::UnimplementedInstr,
            },
            Instruction::Svc { .. } => ExecOutcome::Fault {
                reason: super::FaultReason::UnimplementedInstr,
            },
            Instruction::ExceptionReturn => ExecOutcome::ExceptionReturn,
            Instruction::Unimplemented { .. } => ExecOutcome::Fault {
                reason: super::FaultReason::UnimplementedInstr,
            },
            Instruction::Invalid { address } => ExecOutcome::Fault {
                reason: super::FaultReason::IllegalInstruction { pc: *address },
            },
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
        let instr = Instruction::Str { rt: 1, rn: 0, offset: LoadStoreOffset::Immediate(0), width: AccessWidth::Word };
        assert_eq!(ex.execute(&mut cpu, &mut mem, &instr), ExecOutcome::Continue);
        // LDR R2, [R0]
        let instr = Instruction::Ldr { rt: 2, rn: 0, offset: LoadStoreOffset::Immediate(0), width: AccessWidth::Word };
        assert_eq!(ex.execute(&mut cpu, &mut mem, &instr), ExecOutcome::Continue);
        assert_eq!(cpu.regs[2], 0x1234_5678);
    }

    #[test]
    fn ldr_byte() {
        let (mut ex, mut cpu, mut mem) = setup();
        cpu.regs[0] = 0x2000_0000;
        mem.write_u8(0x2000_0004, 0xAB).unwrap();
        let instr = Instruction::Ldr { rt: 3, rn: 0, offset: LoadStoreOffset::Immediate(4), width: AccessWidth::Byte };
        assert_eq!(ex.execute(&mut cpu, &mut mem, &instr), ExecOutcome::Continue);
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
        let instr = Instruction::Push { regs: 0b11, lr: true };
        assert_eq!(ex.execute(&mut cpu, &mut mem, &instr), ExecOutcome::Continue);
        assert_eq!(cpu.regs[13], 0x2000_0FF4); // SP -= 12
        // POP {R0, R1, PC}
        cpu.regs[0] = 0;
        cpu.regs[1] = 0;
        let instr = Instruction::Pop { regs: 0b11, pc: true };
        assert_eq!(ex.execute(&mut cpu, &mut mem, &instr), ExecOutcome::Continue);
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
        let instr = Instruction::Stm { rn: 0, regs: 0b110, writeback: true };
        assert_eq!(ex.execute(&mut cpu, &mut mem, &instr), ExecOutcome::Continue);
        assert_eq!(cpu.regs[0], 0x2000_0008); // writeback
        // LDM R0!, {R3, R4}
        cpu.regs[0] = 0x2000_0000;
        let instr = Instruction::Ldm { rn: 0, regs: 0b11000, writeback: true };
        assert_eq!(ex.execute(&mut cpu, &mut mem, &instr), ExecOutcome::Continue);
        assert_eq!(cpu.regs[3], 0xAA);
        assert_eq!(cpu.regs[4], 0xBB);
        assert_eq!(cpu.regs[0], 0x2000_0008);
    }
}
