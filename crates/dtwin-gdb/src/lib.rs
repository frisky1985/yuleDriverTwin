//! dtwin-gdb — GDB Remote Serial Protocol (RSP) 调试集成
//!
//! 兼容 arm-none-eabi-gdb 和 VS Code Cortex-Debug 插件。
//! 默认监听 localhost:3333（端口可配置）。
//!
//! 已支持：
//! - 核心寄存器 R0-R15 / xPSR（Cortex-M 原生布局）
//! - 系统寄存器 MSP/PSP/PRIMASK/BASEPRI/FAULTMASK/CONTROL
//! - FPU 寄存器 D0-D15 / FPSCR（GDB 自动合成 S0-S31）
//! - 内存读写、软/硬件断点、硬件观察点
//! - continue / step / Ctrl-C 中断
//! - `monitor` 命令：reset / regs / fpu / stats

#![deny(unsafe_code)]

mod arch;
mod server;
mod target;

pub use arch::{CortexM4F, CortexM4FRegId, CortexM4FRegs};
pub use server::{serve, serve_listener};
pub use target::{describe_listener, DtwinTarget, RunEvent};

/// GDB Server 配置
#[derive(Debug, Clone)]
pub struct GdbServerConfig {
    /// 监听地址（默认 127.0.0.1）
    pub host: String,
    /// 监听端口（默认 3333，与 PRD 一致）
    pub port: u16,
    /// 历史缓冲深度（反向调试，默认 100 万条指令）
    pub history_depth: u64,
}

impl Default for GdbServerConfig {
    fn default() -> Self {
        Self {
            host: "127.0.0.1".to_string(),
            port: 3333,
            history_depth: 1_000_000,
        }
    }
}

/// GDB Server 状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GdbServerState {
    Stopped,
    Running,
    Paused,
}

/// 硬件断点（FPB 限制 6 个）
pub const FPB_BREAKPOINT_MAX: usize = 6;

/// 调试目标内部错误
#[derive(Debug, thiserror::Error)]
pub enum GdbTargetError {
    #[error("调试目标内部错误: {0}")]
    Internal(String),
}

/// GDB 会话错误
#[derive(Debug, thiserror::Error)]
pub enum GdbServerError {
    #[error("GDB 监听/连接失败: {0}")]
    Io(#[from] std::io::Error),
    #[error("GDB 会话失败: {0}")]
    Stub(String),
}
