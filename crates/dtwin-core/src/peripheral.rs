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

/// 总线外设 — 挂接到 `Memory` 外设区，拦截指定地址窗口内的读写
///
/// `Memory` 在访问命中 `MemoryRegionType::Peripheral` 区域时，先查询已挂接的
/// 设备列表（按地址窗口匹配），命中则把读写路由给设备；未命中回落到
/// 「读返回 0 / 写忽略」的外设区默认行为。
pub trait BusDevice: std::fmt::Debug {
    /// 设备名称（如 "CMSDK-APB-UART"）
    fn name(&self) -> &'static str;
    /// 基地址（绝对地址）
    fn base_address(&self) -> u32;
    /// 寄存器窗口大小（基地址起连续覆盖的字节数）
    fn window_size(&self) -> u32;
    /// 读取寄存器（addr 为绝对地址；返回值已按 width 屏蔽）
    fn read(&mut self, addr: u32, width: u32) -> u32;
    /// 写入寄存器（addr 为绝对地址；val 已按 width 屏蔽）
    fn write(&mut self, addr: u32, width: u32, val: u32);
    /// 类型擦除访问（供外部 downcast 到具体模型，如 `CmsdkUart`）
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}
