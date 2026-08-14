//! NVIC 中断与异常系统 — 优先级模型、嵌套中断、中断延迟

/// 异常号：0-15 为系统异常，16+ 为外部中断
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ExceptionNumber {
    Reset = 1,
    Nmi = 2,
    HardFault = 3,
    MemManage = 4,
    BusFault = 5,
    UsageFault = 6,
    SvCall = 11,
    DebugMonitor = 12,
    PendSv = 14,
    SysTick = 15,
    /// 外部中断 IRQn（16 + n）
    External(u16),
}

/// 中断状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrqState {
    Inactive,
    Pending,
    Active,
    PendingAndActive,
}

/// 中断事件追踪记录
#[derive(Debug, Clone)]
pub struct IrqEvent {
    pub irq: u16,
    pub trigger_time_ns: u64,
    pub response_time_ns: u64,
    pub exec_duration_ns: u64,
    pub nesting_level: u8,
}
