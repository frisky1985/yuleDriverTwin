//! 内核引擎 — ARMv7E-M (Cortex-M4F) 指令解码与执行
//!
//! 分层: decode（Thumb-2 指令解码）→ exec（指令执行）→ fpu/dsp（扩展）

pub mod decode;
pub mod dsp;
pub mod engine;
pub mod exec;
pub mod fpu;

pub use engine::{Engine, EngineResult, EngineStats};

#[cfg(test)]
mod test_util;

/// 执行模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecMode {
    /// 逐指令单步（调试）
    Step,
    /// 全速运行（模拟时钟周期为时间单位）
    FullSpeed,
}

/// 异常触发原因（供 NVIC/调试器观测）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultReason {
    MemManage {
        address: u32,
    },
    BusFault {
        address: u32,
    },
    UsageFault {
        address: u32,
    },
    HardFault {
        pc: u32,
    },
    UnalignedAccess {
        address: u32,
    },
    IllegalInstruction {
        pc: u32,
    },
    /// 已解码但尚未实现执行（Phase 1 部分支持）
    UnimplementedInstr,
}
