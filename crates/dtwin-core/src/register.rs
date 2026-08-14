//! 寄存器模型 — 位域访问、副作用建模、事务日志

/// 寄存器访问宽度
#[derive(Debug, Clone, Copy)]
pub enum AccessWidth {
    Bit,
    Byte,
    HalfWord,
    Word,
}

/// 寄存器访问属性
#[derive(Debug, Clone, Copy)]
pub enum AccessType {
    ReadOnly,
    WriteOnly,
    ReadWrite,
    /// 写 1 清零 (write-1-to-clear)
    W1C,
    /// 写触发翻转
    Toggle,
}

/// 寄存器定义
#[derive(Debug, Clone)]
pub struct Register {
    pub offset: u32,
    pub width_bits: u8,
    pub reset_value: u32,
    pub access: AccessType,
    pub name: &'static str,
}

/// 寄存器访问事务记录
#[derive(Debug, Clone)]
pub struct AccessLogEntry {
    pub timestamp_ns: u64,
    pub address: u32,
    pub value: u32,
    pub width: AccessWidth,
    pub call_stack: Vec<String>,
}
