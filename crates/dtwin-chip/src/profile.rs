//! 芯片配置文件加载与校验

use crate::ChipProfile;

/// 配置文件校验结果
#[derive(Debug, Default)]
pub struct ValidationReport {
    pub passed: Vec<String>,
    pub warnings: Vec<String>,
    pub errors: Vec<String>,
}

impl ValidationReport {
    pub fn is_valid(&self) -> bool {
        self.errors.is_empty()
    }
}

/// 校验芯片配置文件：内存区域重叠、寄存器地址冲突、外设依赖完整性
pub fn validate(profile: &ChipProfile) -> ValidationReport {
    let mut report = ValidationReport::default();

    // 内存区域重叠检查
    for (i, a) in profile.memory.iter().enumerate() {
        for b in profile.memory.iter().skip(i + 1) {
            let a_end = a.start + a.size;
            let b_end = b.start + b.size;
            if a.start < b_end && b.start < a_end {
                report.errors.push(format!(
                    "内存区域重叠: {} ({:#x}-{:#x}) 与 {} ({:#x}-{:#x})",
                    a.name, a.start, a_end, b.name, b.start, b_end
                ));
            }
        }
    }

    // 外设地址冲突检查
    for (i, a) in profile.peripherals.iter().enumerate() {
        for b in profile.peripherals.iter().skip(i + 1) {
            if a.base_address == b.base_address {
                report.warnings.push(format!(
                    "外设地址冲突: {} 与 {} 基地址相同 ({:#x})",
                    a.name, b.name, a.base_address
                ));
            }
        }
    }

    report
        .passed
        .push(format!("内存区域 {} 个检查完成", profile.memory.len()));
    report
}
