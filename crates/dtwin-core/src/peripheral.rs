//! 外设行为模型 — GPIO/UART/Timer/SPI/I2C/ADC 等功能行为模拟

/// 外设实现状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PeripheralStatus {
    /// 完整行为模型
    Implemented,
    /// 部分实现
    Partial,
    /// 桩模型（仅寄存器存储）
    Stub,
}

/// 外设抽象接口
pub trait Peripheral {
    /// 外设名称（如 "USART1"）
    fn name(&self) -> &str;
    /// 外设基地址
    fn base_address(&self) -> u32;
    /// 实现状态
    fn status(&self) -> PeripheralStatus;
    /// 复位外设
    fn reset(&mut self);
    /// 时钟周期驱动（由模拟引擎调用）
    fn tick(&mut self, cycles: u64);
}
