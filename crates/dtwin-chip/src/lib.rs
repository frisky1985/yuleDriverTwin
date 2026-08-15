//! dtwin-chip — 芯片配置文件系统（TOML/SVD 导入/overlay 继承）

pub mod profile;
pub mod s32k312;

pub use s32k312::S32K312;

/// 芯片配置文件结构（TOML 四大模块：内核/内存/外设/时钟树）
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct ChipProfile {
    pub name: String,
    pub core: CoreDef,
    pub memory: Vec<MemoryDef>,
    pub peripherals: Vec<PeripheralDef>,
    pub clock: ClockTree,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct CoreDef {
    pub core_type: String,
    pub default_freq_hz: u32,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MemoryDef {
    pub name: String,
    pub start: u32,
    pub size: u32,
    pub region_type: String,
    pub writable: bool,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PeripheralDef {
    pub name: String,
    pub base_address: u32,
    pub model: String,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct ClockTree {
    pub source_hz: u32,
    pub apb1_hz: u32,
    pub apb2_hz: u32,
}
