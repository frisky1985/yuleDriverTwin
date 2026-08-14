//! FPU — Cortex-M4F 浮点单元（单/双精度）
//!
//! Phase 4 实现：寄存器文件 S0-S31 + FPSCR + 浮点指令。
//! Phase 1 先提供寄存器文件与状态管理骨架。

/// FPU 寄存器文件（S0-S31 单精度 / D0-D15 双精度别名）
#[derive(Debug, Default, Clone)]
pub struct FpuRegisters {
    /// S0-S31 单精度寄存器（按位存储 u32）
    pub s: [u32; 32],
    /// FPSCR 浮点状态与控制寄存器
    pub fpscr: u32,
    /// FPCCR 浮点上下文控制（惰性压栈）
    pub fpccr: u32,
}

impl FpuRegisters {
    pub fn new() -> Self {
        Self::default()
    }

    /// 读取单精度寄存器 S[n]
    pub fn read_s(&self, n: usize) -> u32 {
        self.s[n & 0x1F]
    }

    /// 写入单精度寄存器 S[n]
    pub fn write_s(&mut self, n: usize, val: u32) {
        self.s[n & 0x1F] = val;
    }

    /// 读取双精度寄存器 D[n]（两个 S 寄存器组合）
    pub fn read_d(&self, n: usize) -> u64 {
        let idx = (n & 0xF) * 2;
        (self.s[idx] as u64) | ((self.s[idx + 1] as u64) << 32)
    }

    /// 写入双精度寄存器 D[n]
    pub fn write_d(&mut self, n: usize, val: u64) {
        let idx = (n & 0xF) * 2;
        self.s[idx] = val as u32;
        self.s[idx + 1] = (val >> 32) as u32;
    }

    /// 读取 FPSCR 标志位（N/Z/C/V）
    pub fn fpscr_flags(&self) -> (bool, bool, bool, bool) {
        (
            self.fpscr & (1 << 31) != 0, // N
            self.fpscr & (1 << 30) != 0, // Z
            self.fpscr & (1 << 29) != 0, // C
            self.fpscr & (1 << 28) != 0, // V
        )
    }

    /// 复位 FPU 状态
    pub fn reset(&mut self) {
        self.s = [0; 32];
        self.fpscr = 0;
    }
}
