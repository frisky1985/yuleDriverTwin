//! dtwin-gdb — GDB Remote Serial Protocol (RSP) 调试集成
//!
//! 兼容 arm-none-eabi-gdb 和 VS Code Cortex-Debug 插件。
//! 默认监听 localhost:3333（端口可配置）。

#![deny(unsafe_code)]

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
