//! NVIC 中断与异常系统 — 优先级模型、嵌套中断、异常入口/出口、向量表
//!
//! Cortex-M 异常模型：
//! - 异常号 1-15 系统异常，16+ 外部中断
//! - 异常入口压栈 8 字（r0-r3, r12, lr, pc, xpsr）到 MSP/PSP
//! - 向量表在 0x00000000（[0]=初始 MSP，[1]=Reset，[n]=异常 n 向量）
//! - EXC_RETURN 识别：0xFFFFFFF1/9/D（MSP/PSP 线程/处理模式）

/// 异常号：0-15 为系统异常，16+ 为外部中断
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ExceptionNumber {
    Reset = 1,
    Nmi = 2,
    HardFault = 3,
    MemManage = 4,
    BusFault = 5,
    UsageFault = 6,
    SvCall = 11,
    DebugMonitor = 12,
    PendSv = 14,
    SysTick = 15,
    /// 外部中断 IRQn（16 + n）
    External(u16),
}

impl ExceptionNumber {
    pub fn as_u8(self) -> u8 {
        match self {
            ExceptionNumber::External(n) => (16 + n) as u8,
            ExceptionNumber::Reset => 1,
            ExceptionNumber::Nmi => 2,
            ExceptionNumber::HardFault => 3,
            ExceptionNumber::MemManage => 4,
            ExceptionNumber::BusFault => 5,
            ExceptionNumber::UsageFault => 6,
            ExceptionNumber::SvCall => 11,
            ExceptionNumber::DebugMonitor => 12,
            ExceptionNumber::PendSv => 14,
            ExceptionNumber::SysTick => 15,
        }
    }

    pub fn from_u8(v: u8) -> Self {
        if v >= 16 {
            ExceptionNumber::External((v - 16) as u16)
        } else {
            match v {
                1 => ExceptionNumber::Reset,
                2 => ExceptionNumber::Nmi,
                3 => ExceptionNumber::HardFault,
                4 => ExceptionNumber::MemManage,
                5 => ExceptionNumber::BusFault,
                6 => ExceptionNumber::UsageFault,
                11 => ExceptionNumber::SvCall,
                12 => ExceptionNumber::DebugMonitor,
                14 => ExceptionNumber::PendSv,
                15 => ExceptionNumber::SysTick,
                _ => ExceptionNumber::External(0),
            }
        }
    }
}

/// 中断状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrqState {
    Inactive,
    Pending,
    Active,
    PendingAndActive,
}

/// 中断事件追踪记录
#[derive(Debug, Clone)]
pub struct IrqEvent {
    pub irq: u16,
    pub trigger_time_ns: u64,
    pub response_time_ns: u64,
    pub exec_duration_ns: u64,
    pub nesting_level: u8,
}

/// EXC_RETURN 特殊值
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExcReturn {
    /// 返回处理模式，MSP
    HandlerMsp,
    /// 返回线程模式，MSP
    ThreadMsp,
    /// 返回线程模式，PSP
    ThreadPsp,
    /// FPU 变体（含惰性压栈标志）
    HandlerMspFpu,
    ThreadMspFpu,
    ThreadPspFpu,
    Invalid,
}

impl ExcReturn {
    pub fn from_value(v: u32) -> Self {
        match v {
            0xFFFF_FFF1 => ExcReturn::HandlerMsp,
            0xFFFF_FFF9 => ExcReturn::ThreadMsp,
            0xFFFF_FFFD => ExcReturn::ThreadPsp,
            0xFFFF_FFE1 => ExcReturn::HandlerMspFpu,
            0xFFFF_FFE9 => ExcReturn::ThreadMspFpu,
            0xFFFF_FFED => ExcReturn::ThreadPspFpu,
            _ => ExcReturn::Invalid,
        }
    }

    /// 是否使用进程栈
    pub fn uses_psp(self) -> bool {
        matches!(self, ExcReturn::ThreadPsp | ExcReturn::ThreadPspFpu)
    }

    /// 返回线程模式
    pub fn to_thread(self) -> bool {
        matches!(self, ExcReturn::ThreadMsp | ExcReturn::ThreadPsp | ExcReturn::ThreadMspFpu | ExcReturn::ThreadPspFpu)
    }
}

/// NVIC 中断控制器 + 异常入口状态
#[derive(Debug)]
pub struct Nvic {
    /// 向量表（[0] = 初始 MSP，[n] = 异常 n 的向量地址；按需扩容）
    pub vector_table: Vec<u32>,
    /// 外部中断挂起位图（256 IRQ = 8 × u32）
    pub pending: [u32; 8],
    /// 外部中断使能位图
    pub enabled: [u32; 8],
    /// 外部中断活跃位图
    pub active: [u32; 8],
    /// 优先级（外部中断，数字越小优先级越高）
    pub priority: [u8; 240],
    /// 当前异常号（0 = 线程模式）
    pub current_exception: u8,
    /// 嵌套深度
    pub nesting_depth: u8,
    /// 异常追踪日志
    pub events: Vec<IrqEvent>,
    /// 模拟时钟（ns）
    pub clock_ns: u64,
}

impl Default for Nvic {
    fn default() -> Self {
        Self::new()
    }
}

impl Nvic {
    pub fn new() -> Self {
        // 默认向量表：MSP + 15 系统异常
        let mut vt = vec![0u32; 16];
        vt[0] = 0x2000_0000; // 初始 MSP
        Nvic {
            vector_table: vt,
            pending: [0; 8],
            enabled: [0; 8],
            active: [0; 8],
            priority: [0; 240],
            current_exception: 0,
            nesting_depth: 0,
            events: Vec::new(),
            clock_ns: 0,
        }
    }

    /// 设置异常向量（number = 异常号）
    pub fn set_vector(&mut self, number: u8, addr: u32) {
        if number as usize >= self.vector_table.len() {
            self.vector_table.resize(number as usize + 1, 0);
        }
        self.vector_table[number as usize] = addr;
    }

    /// 读取向量（异常号 0 返回 0）
    pub fn vector(&self, number: u8) -> u32 {
        self.vector_table.get(number as usize).copied().unwrap_or(0)
    }

    /// 挂起外部中断
    pub fn pend_irq(&mut self, irq: u16) {
        if irq < 256 {
            self.pending[(irq / 32) as usize] |= 1 << (irq % 32);
        }
    }

    /// 清除外部中断挂起
    pub fn unpend_irq(&mut self, irq: u16) {
        if irq < 256 {
            self.pending[(irq / 32) as usize] &= !(1 << (irq % 32));
        }
    }

    /// 使能外部中断
    pub fn enable_irq(&mut self, irq: u16) {
        if irq < 256 {
            self.enabled[(irq / 32) as usize] |= 1 << (irq % 32);
        }
    }

    /// 查询是否有更高优先级异常待处理（简化：只做 pending+enabled 检查）
    pub fn has_pending_irq(&self) -> bool {
        (0..8).any(|i| (self.pending[i] & self.enabled[i]) != 0)
    }

    /// 取最高优先级挂起外部中断号（简化轮询）
    pub fn next_pending_irq(&self) -> Option<u16> {
        for i in 0..8 {
            let pend = self.pending[i] & self.enabled[i];
            if pend != 0 {
                for bit in 0..32 {
                    if pend & (1 << bit) != 0 {
                        return Some((i * 32 + bit) as u16);
                    }
                }
            }
        }
        None
    }

    /// 进入异常：current_exception 置位，记录事件
    pub fn enter_exception(&mut self, number: u8) {
        self.current_exception = number;
        self.nesting_depth += 1;
        let irq_num: u16 = if number >= 16 { number as u16 - 16 } else { 0 };
        if number >= 16 {
            self.unpend_irq(irq_num as u16);
            let idx = (irq_num / 32) as usize;
            self.active[idx] |= 1 << (irq_num % 32);
        }
        self.events.push(IrqEvent {
            irq: if number >= 16 { number as u16 - 16 } else { 0 },
            trigger_time_ns: self.clock_ns,
            response_time_ns: self.clock_ns,
            exec_duration_ns: 0,
            nesting_level: self.nesting_depth,
        });
    }

    /// 退出异常：恢复当前异常号
    pub fn exit_exception(&mut self) {
        if self.nesting_depth > 0 {
            self.nesting_depth -= 1;
        }
        // 弹栈恢复上一个活跃异常（简化：直接回线程模式）
        if let Some(ev) = self.events.last_mut() {
            ev.exec_duration_ns = self.clock_ns.saturating_sub(ev.response_time_ns);
        }
        self.current_exception = 0;
        // 外部中断活跃位清除
        for a in self.active.iter_mut() {
            *a = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exception_number_mapping() {
        assert_eq!(ExceptionNumber::HardFault.as_u8(), 3);
        assert_eq!(ExceptionNumber::SvCall.as_u8(), 11);
        assert_eq!(ExceptionNumber::External(0).as_u8(), 16);
        // External(n) = 16 + n，u8 范围最大 239 → 255
        assert_eq!(ExceptionNumber::External(239).as_u8(), 255);
        assert_eq!(ExceptionNumber::from_u8(16), ExceptionNumber::External(0));
        assert_eq!(ExceptionNumber::from_u8(5), ExceptionNumber::BusFault);
    }

    #[test]
    fn exc_return_mapping() {
        assert_eq!(ExcReturn::from_value(0xFFFF_FFF1), ExcReturn::HandlerMsp);
        assert_eq!(ExcReturn::from_value(0xFFFF_FFF9), ExcReturn::ThreadMsp);
        assert_eq!(ExcReturn::from_value(0xFFFF_FFFD), ExcReturn::ThreadPsp);
        assert_eq!(ExcReturn::from_value(0xFFFF_FFE9), ExcReturn::ThreadMspFpu);
        assert!(ExcReturn::ThreadPsp.uses_psp());
        assert!(!ExcReturn::HandlerMsp.uses_psp());
        assert!(ExcReturn::ThreadMsp.to_thread());
        assert!(!ExcReturn::HandlerMsp.to_thread());
        assert_eq!(ExcReturn::from_value(0x1234), ExcReturn::Invalid);
    }

    #[test]
    fn vector_table_default() {
        let n = Nvic::new();
        assert_eq!(n.vector(1), 0);
        assert_eq!(n.vector_table[0], 0x2000_0000);
    }

    #[test]
    fn irq_pend_enable() {
        let mut n = Nvic::new();
        assert!(!n.has_pending_irq());
        n.pend_irq(5);
        assert!(!n.has_pending_irq()); // 未使能
        n.enable_irq(5);
        assert!(n.has_pending_irq());
        assert_eq!(n.next_pending_irq(), Some(5));
        n.enter_exception(ExceptionNumber::External(5).as_u8());
        assert!(!n.has_pending_irq()); // 已清除挂起
        assert_eq!(n.current_exception, 21);
        n.exit_exception();
        assert_eq!(n.current_exception, 0);
    }

    #[test]
    fn exception_nesting() {
        let mut n = Nvic::new();
        n.enter_exception(3); // HardFault
        assert_eq!(n.nesting_depth, 1);
        n.enter_exception(6); // UsageFault 嵌套
        assert_eq!(n.nesting_depth, 2);
        assert_eq!(n.current_exception, 6);
        n.exit_exception();
        n.exit_exception();
        assert_eq!(n.nesting_depth, 0);
    }
}
