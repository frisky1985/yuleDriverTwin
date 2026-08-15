//! 测试工具 — decode+exec 全链路 harness（仅测试构建）
//!
//! 将原始 Thumb-2 指令编码送入 Decoder → Executor，供 DSP/FPU golden 测试使用。

#![cfg(test)]

use crate::engine::decode::Decoder;
use crate::engine::exec::{ExecOutcome, Executor};
use crate::memory::Memory;
use crate::CpuState;

/// 测试执行器：一条指令的 解码 + 执行 全链路
pub struct Harness {
    pub cpu: CpuState,
    pub mem: Memory,
    pub decoder: Decoder,
    pub executor: Executor,
}

impl Harness {
    pub fn new() -> Self {
        Harness {
            cpu: CpuState::default(),
            mem: Memory::test_ram(),
            decoder: Decoder::new(),
            executor: Executor::new(),
        }
    }

    /// 执行一条 32 位 Thumb-2 指令（编码已由汇编器验证）
    pub fn exec_word(&mut self, word: u32) -> ExecOutcome {
        let instr = self.decoder.decode_word(word, self.cpu.regs[15]);
        self.executor.execute(&mut self.cpu, &mut self.mem, &instr)
    }

    /// 读取 APSR Q 标志（bit27）
    pub fn q_flag(&self) -> bool {
        self.cpu.xpsr & (1 << 27) != 0
    }

    /// 读取 GE 标志（bits[19:16]）
    pub fn ge_flags(&self) -> u32 {
        (self.cpu.xpsr >> 16) & 0xF
    }
}
