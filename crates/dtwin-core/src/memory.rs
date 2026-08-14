//! 内存模型 — 标准 Cortex-M 映射、MPU、Flash 行为、watchpoint

/// Cortex-M 标准内存区域
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryRegionType {
    Code,
    Sram,
    Peripheral,
    ExternalRam,
    Ccm,
    System,
}

/// 内存区域定义
#[derive(Debug, Clone)]
pub struct MemoryRegion {
    pub name: &'static str,
    pub start: u32,
    pub end: u32,
    pub region_type: MemoryRegionType,
    /// 是否允许写
    pub writable: bool,
    /// 是否允许执行
    pub executable: bool,
}

/// MPU 区域保护配置
#[derive(Debug, Clone)]
pub struct MpuRegion {
    pub index: u8,
    pub start: u32,
    pub size: u32,
    pub read: bool,
    pub write: bool,
    pub execute: bool,
    pub privileged_only: bool,
}

/// 内存观察点
#[derive(Debug, Clone)]
pub struct Watchpoint {
    pub address: u32,
    pub size: u32,
    pub on_write: bool,
    pub on_read: bool,
}

/// Flash 扇区状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashSectorState {
    Erased,
    Written,
    Erasing,
}
