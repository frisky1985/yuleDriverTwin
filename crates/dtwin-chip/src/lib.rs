//! dtwin-chip — 芯片配置文件系统（TOML/SVD 导入/overlay 继承）

pub mod profile;
pub mod s32k312;

use dtwin_core::memory::{Memory, MemoryRegion, MemoryRegionType};
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

/// 按芯片 profile 构造 `Memory`（Flash 烧录语义 0xFF / SRAM 清零 / 外设区空）
///
/// 外设基址已在 profile 中注册，但行为模型（如 UART）需另行 `attach_peripheral`。
pub fn memory_from_profile(profile: &ChipProfile) -> Memory {
    let mut regions = Vec::new();
    let mut flash = Vec::new();
    let mut sram = Vec::new();
    let mut ccm = Vec::new();

    for m in &profile.memory {
        let region_type = match m.region_type.as_str() {
            "Code" => MemoryRegionType::Code,
            "Sram" => MemoryRegionType::Sram,
            "Ccm" => MemoryRegionType::Ccm,
            "Peripheral" => MemoryRegionType::Peripheral,
            _ => MemoryRegionType::System,
        };
        let end = m
            .start
            .checked_add(m.size)
            .expect("内存区域地址溢出 (start+size > u32)");
        let executable = matches!(region_type, MemoryRegionType::Code | MemoryRegionType::Sram);
        // MemoryRegion.name 为 &'static str，按区域类型映射（名称仅作展示/调试用）
        let static_name: &'static str = match region_type {
            MemoryRegionType::Code => "FLASH",
            MemoryRegionType::Sram => "SRAM",
            MemoryRegionType::Ccm => "CCM",
            MemoryRegionType::Peripheral => "PERIPH",
            MemoryRegionType::ExternalRam => "EXTRAM",
            MemoryRegionType::System => "SYSTEM",
        };
        regions.push(MemoryRegion {
            name: static_name,
            start: m.start,
            end,
            region_type,
            writable: m.writable,
            executable,
        });
        match region_type {
            MemoryRegionType::Code => flash = vec![0xFF; m.size as usize],
            MemoryRegionType::Sram => sram = vec![0; m.size as usize],
            MemoryRegionType::Ccm => ccm = vec![0; m.size as usize],
            _ => {}
        }
    }

    Memory {
        regions,
        flash,
        sram,
        ccm,
        mpu_regions: Vec::new(),
        mpu_enabled: false,
        unalign_trap: true,
        watchpoints: Vec::new(),
        watchpoint_hit: None,
        flash_erase_required: true,
        read_count: 0,
        write_count: 0,
        peripherals: Vec::new(),
    }
}
