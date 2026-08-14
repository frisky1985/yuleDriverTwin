//! 引擎主循环 — 取指/解码/执行 + 异常处理
//!
//! 串联: Memory(取指) → Decoder(解码) → Executor(执行) → Nvic(异常)

use super::decode::Decoder;
use super::exec::{ExecOutcome, Executor};
use super::FaultReason;
use crate::memory::Memory;
use crate::nvic::Nvic;
use crate::CpuState;

/// 引擎运行统计
#[derive(Debug, Default, Clone, Copy)]
pub struct EngineStats {
    pub instructions: u64,
    pub cycles: u64,
    pub faults: u64,
    pub exceptions: u64,
}

/// 引擎执行结果
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineResult {
    /// 正常停止（达到指令上限或显式暂停）
    Halted,
    /// 触发未处理异常
    Fault { reason: FaultReason },
    /// 达到指令数上限
    LimitReached,
}

/// 内核引擎
pub struct Engine {
    decoder: Decoder,
    executor: Executor,
    pub stats: EngineStats,
    /// 单次 run 的指令上限（防死循环）
    pub max_instructions: u64,
    /// 异常向量表基地址（通常 0x0000_0000）
    pub vector_base: u32,
}

impl Engine {
    pub fn new() -> Self {
        Self {
            decoder: Decoder::new(),
            executor: Executor::new(),
            stats: EngineStats::default(),
            max_instructions: 1_000_000,
            vector_base: 0,
        }
    }

    /// 单步执行一条指令（供调试器使用）
    pub fn step(&mut self, cpu: &mut CpuState, mem: &mut Memory, nvic: &mut Nvic) -> EngineResult {
        // 取指
        let pc = cpu.regs[15];
        let raw = match mem.read_u16(pc) {
            Ok(v) => v,
            Err(_) => {
                self.stats.faults += 1;
                return EngineResult::Fault { reason: FaultReason::BusFault { address: pc } };
            }
        };

        // 判断指令宽度（16-bit vs 32-bit）：0xE000-0xFFFF 高半字且非 0xExxx 是 32-bit 前缀
        let instr = if (raw & 0xF800) == 0xE800 || (raw & 0xF000) == 0xF000 {
            // 32-bit Thumb-2：读高半字拼接
            let hi = match mem.read_u16(pc + 2) {
                Ok(v) => v,
                Err(_) => {
                    self.stats.faults += 1;
                    return EngineResult::Fault { reason: FaultReason::BusFault { address: pc + 2 } };
                }
            };
            let full = ((raw as u32) << 16) | hi as u32;
            self.decoder.decode_word(full, pc)
        } else {
            self.decoder.decode_halfword(raw, pc)
        };

        // 执行
        let outcome = self.executor.execute(cpu, mem, &instr);
        self.stats.instructions += 1;
        self.stats.cycles = self.executor.cycle_count;

        match outcome {
            ExecOutcome::Continue => {
                // PC 默认 +2/+4（分支指令自行改 PC）
                let width = if (raw & 0xF800) == 0xE800 || (raw & 0xF000) == 0xF000 { 4 } else { 2 };
                cpu.regs[15] = cpu.regs[15].wrapping_add(width);
                EngineResult::Halted // 单步：返回暂停
            }
            ExecOutcome::Branch { target } => {
                cpu.regs[15] = target;
                EngineResult::Halted
            }
            ExecOutcome::ExceptionReturn => {
                self.stats.exceptions += 1;
                EngineResult::Halted
            }
            ExecOutcome::Fault { reason } => {
                self.stats.faults += 1;
                EngineResult::Fault { reason }
            }
        }
    }

    /// 全速运行直到达到指令上限或触发异常
    pub fn run(&mut self, cpu: &mut CpuState, mem: &mut Memory, nvic: &mut Nvic) -> EngineResult {
        let start = self.stats.instructions;
        loop {
            if self.stats.instructions - start >= self.max_instructions {
                return EngineResult::LimitReached;
            }
            // 检查是否有待处理中断（简化：仅当无当前异常时）
            if nvic.current_exception == 0 {
                if let Some(irq) = nvic.next_pending_irq() {
                    nvic.enter_exception((irq + 16) as u8);
                    self.stats.exceptions += 1;
                }
            }
            match self.step(cpu, mem, nvic) {
                EngineResult::Halted => {
                    // 单步返回 Halted，run 循环继续
                    continue;
                }
                r => return r,
            }
        }
    }
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::Memory;

    #[test]
    fn engine_steps_through_code() {
        let mut cpu = CpuState::default();
        let mut mem = Memory::test_ram();
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();

        // 程序: MOV R0, #0x05 (0x2005) 然后 NOP (0xBF00)
        mem.flash[0] = 0x05;
        mem.flash[1] = 0x20;
        mem.flash[2] = 0x00;
        mem.flash[3] = 0xBF;
        cpu.regs[15] = 0;

        eng.step(&mut cpu, &mut mem, &mut nvic);
        assert_eq!(cpu.regs[0], 5);
        assert_eq!(cpu.regs[15], 2);
        assert_eq!(eng.stats.instructions, 1);
    }
}
