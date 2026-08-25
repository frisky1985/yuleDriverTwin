//! S32K312 芯片 profile — 内存映射 + 外设基址注册
//!
//! NXP S32K3 系列（Arm Cortex-M4F ISA 兼容，dtwin 引擎按 ARMv7E-M/M4F 建模）。
//! 内存映射（对齐固件链接脚本与目标板）：
//! - FLASH 0x0000_0000 - 0x003F_FFFF（4MB，Code，只读+可执行）
//! - SRAM  0x2000_0000 - 0x203F_FFFF（4MB，读写）
//! - PERIPH 0x4000_0000+（外设区，注册基址供 UART 等行为模型后续使用）

use crate::{ChipProfile, ClockTree, CoreDef, MemoryDef, PeripheralDef};

/// S32K312 芯片构造器（占位类型，仅提供 `new` 工厂）
pub struct S32K312;

impl S32K312 {
    /// 生成 S32K312 芯片 profile
    pub fn new() -> ChipProfile {
        ChipProfile {
            name: "S32K312".to_string(),
            core: CoreDef {
                core_type: "Cortex-M4F".to_string(), // dtwin 引擎按 ARMv7E-M/M4F 建模
                default_freq_hz: 80_000_000,
            },
            memory: vec![
                MemoryDef {
                    name: "FLASH".to_string(),
                    start: 0x0000_0000,
                    size: 0x0040_0000, // 4MB
                    region_type: "Code".to_string(),
                    writable: false, // 只读+可执行
                },
                MemoryDef {
                    name: "SRAM".to_string(),
                    start: 0x2000_0000,
                    size: 0x0040_0000, // 4MB
                    region_type: "Sram".to_string(),
                    writable: true,
                },
                MemoryDef {
                    name: "PERIPH".to_string(),
                    start: 0x4000_0000,
                    size: 0x1000_0000, // 256MB 外设区
                    region_type: "Peripheral".to_string(),
                    writable: true,
                },
                MemoryDef {
                    name: "SYSTEM".to_string(),
                    start: 0xE000_0000,
                    size: 0x0010_0000, // 1MB 系统区（SysTick/SCB/NVIC，FRT-CHIP-01）
                    region_type: "System".to_string(),
                    writable: true,
                },
            ],
            peripherals: vec![
                // 串口（UART 模型后续接入）
                PeripheralDef {
                    name: "LPUART0".to_string(),
                    base_address: 0x4018_0000,
                    model: "stub".to_string(),
                },
                // 引脚/IO 复用
                PeripheralDef {
                    name: "SIUL2".to_string(),
                    base_address: 0x4029_0000,
                    model: "stub".to_string(),
                },
                PeripheralDef {
                    name: "SIUL2_GPIO".to_string(),
                    base_address: 0x4081_0000,
                    model: "stub".to_string(),
                },
                // 时钟/系统控制
                PeripheralDef {
                    name: "MCU_SCG_SIM".to_string(),
                    base_address: 0x402A_0000,
                    model: "stub".to_string(),
                },
                // CAN
                PeripheralDef {
                    name: "CAN0".to_string(),
                    base_address: 0x4005_0000,
                    model: "stub".to_string(),
                },
                // 定时器
                PeripheralDef {
                    name: "FTM0".to_string(),
                    base_address: 0x400D_0000,
                    model: "stub".to_string(),
                },
                // ADC
                PeripheralDef {
                    name: "ADC0".to_string(),
                    base_address: 0x400C_0000,
                    model: "stub".to_string(),
                },
                // 看门狗
                PeripheralDef {
                    name: "WDOG".to_string(),
                    base_address: 0x4005_3000,
                    model: "stub".to_string(),
                },
                // 存储接口（基址与内存区一致，供总线模型查询）
                PeripheralDef {
                    name: "FLASH".to_string(),
                    base_address: 0x0000_0000,
                    model: "stub".to_string(),
                },
                PeripheralDef {
                    name: "SRAM".to_string(),
                    base_address: 0x2000_0000,
                    model: "stub".to_string(),
                },
            ],
            clock: ClockTree {
                source_hz: 80_000_000,
                apb1_hz: 80_000_000,
                apb2_hz: 80_000_000,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::validate;

    /// 对照任务要求的 10 个外设基址
    #[test]
    fn peripheral_base_addresses_match_spec() {
        let p = S32K312::new();
        let base = |name: &str| -> u32 {
            p.peripherals
                .iter()
                .find(|x| x.name == name)
                .unwrap_or_else(|| panic!("缺少外设 {name}"))
                .base_address
        };
        assert_eq!(base("LPUART0"), 0x4018_0000);
        assert_eq!(base("SIUL2"), 0x4029_0000);
        assert_eq!(base("SIUL2_GPIO"), 0x4081_0000);
        assert_eq!(base("MCU_SCG_SIM"), 0x402A_0000);
        assert_eq!(base("CAN0"), 0x4005_0000);
        assert_eq!(base("FTM0"), 0x400D_0000);
        assert_eq!(base("ADC0"), 0x400C_0000);
        assert_eq!(base("WDOG"), 0x4005_3000);
        assert_eq!(base("FLASH"), 0x0000_0000);
        assert_eq!(base("SRAM"), 0x2000_0000);
        assert_eq!(p.peripherals.len(), 10);
    }

    /// 内存映射与任务规格一致（FRT-CHIP-01：新增 SYSTEM 区）
    #[test]
    fn memory_map_matches_spec() {
        let p = S32K312::new();
        // FLASH / SRAM / PERIPH / SYSTEM（FRT-CHIP-01：系统区 0xE0000000-0xE0100000）
        assert_eq!(p.memory.len(), 4);
        let flash = &p.memory[0];
        assert_eq!(
            (flash.name.as_str(), flash.start, flash.size),
            ("FLASH", 0x0000_0000, 0x0040_0000)
        );
        assert!(!flash.writable);
        let sram = &p.memory[1];
        assert_eq!(
            (sram.name.as_str(), sram.start, sram.size),
            ("SRAM", 0x2000_0000, 0x0040_0000)
        );
        assert!(sram.writable);
        let periph = &p.memory[2];
        assert_eq!(
            (periph.name.as_str(), periph.start),
            ("PERIPH", 0x4000_0000)
        );
        assert_eq!(periph.region_type, "Peripheral");
        let system = &p.memory[3];
        assert_eq!(
            (system.name.as_str(), system.start, system.size),
            ("SYSTEM", 0xE000_0000, 0x0010_0000)
        );
        assert_eq!(system.region_type, "System");
    }

    /// profile 校验通过（内存无重叠、外设无冲突）
    #[test]
    fn profile_validates() {
        let p = S32K312::new();
        let report = validate(&p);
        assert!(report.is_valid(), "errors: {:?}", report.errors);
        assert!(
            report.warnings.is_empty(),
            "warnings: {:?}",
            report.warnings
        );
    }
}
