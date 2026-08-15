//! dtwin-core — Driver Twin 核心模拟引擎
//!
//! 芯片级精度的 ARM Cortex-M 行为模拟器核心：
//! 内核指令集、寄存器模型、内存模型、外设行为模型、NVIC 中断系统。

#![deny(unsafe_code)] // 核心引擎禁止 unsafe，内存安全由编译器保证

pub mod engine;
pub mod memory;
pub mod nvic;
pub mod peripheral;
pub mod register;

use engine::fpu::FpuRegisters;

/// 支持的 Cortex-M 内核型号
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoreType {
    /// ARMv6-M (M0/M0+)
    M0,
    /// ARMv7-M (M3)
    M3,
    /// ARMv7E-M + DSP (M4)
    M4,
    /// ARMv7E-M + DSP + FPU (M4F)
    M4F,
    /// ARMv8-M + TrustZone (M33)
    M33,
}

impl CoreType {
    /// 架构版本描述
    pub fn arch_name(self) -> &'static str {
        match self {
            CoreType::M0 => "ARMv6-M",
            CoreType::M3 => "ARMv7-M",
            CoreType::M4 | CoreType::M4F => "ARMv7E-M",
            CoreType::M33 => "ARMv8-M",
        }
    }
}

/// 模拟实例核心状态
#[derive(Debug)]
pub struct CpuState {
    /// 通用寄存器 R0-R15
    pub regs: [u32; 16],
    /// 程序状态寄存器 xPSR（bit27=Q，bits[19:16]=GE，bits[31:28]=NZCV）
    pub xpsr: u32,
    /// 主栈指针
    pub msp: u32,
    /// 进程栈指针
    pub psp: u32,
    /// 特殊寄存器
    pub primask: u8,
    pub faultmask: u8,
    pub basepri: u8,
    pub control: u8,
    /// 协处理器访问控制（0xE000_ED88）：bits[23:20] = CP11/CP10 权限（11 = 全访问）
    /// 默认 0x00F0_0000（FPU 使能，与 Phase 4 既有行为一致；真实复位值 0，板级初始化后使能）
    pub cpacr: u32,
    /// FPU 寄存器文件（S0-S31/D0-D15 + FPSCR）
    pub fpu: FpuRegisters,
}

impl Default for CpuState {
    fn default() -> Self {
        Self {
            regs: [0; 16],
            xpsr: 0,
            msp: 0,
            psp: 0,
            primask: 0,
            faultmask: 0,
            basepri: 0,
            control: 0,
            cpacr: 0x00F0_0000,
            fpu: FpuRegisters::default(),
        }
    }
}

impl CpuState {
    /// FPU 是否使能：CPACR CP10/CP11 均为 0b11（全访问）
    pub fn fpu_enabled(&self) -> bool {
        (self.cpacr & 0x00F0_0000) == 0x00F0_0000
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn core_type_arch_names() {
        assert_eq!(CoreType::M0.arch_name(), "ARMv6-M");
        assert_eq!(CoreType::M3.arch_name(), "ARMv7-M");
        assert_eq!(CoreType::M4F.arch_name(), "ARMv7E-M");
        assert_eq!(CoreType::M33.arch_name(), "ARMv8-M");
    }

    #[test]
    fn cpu_state_defaults() {
        let s = CpuState::default();
        assert_eq!(s.regs, [0u32; 16]);
        assert_eq!(s.xpsr, 0);
        assert_eq!(s.control, 0);
    }
}
