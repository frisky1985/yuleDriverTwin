//! CMSDK APB UART 行为模型（qemu_s32k312_compat.h 寄存器语义）
//!
//! yuleASR 的 QEMU 固件通过 `qemu_s32k312_compat.h` 把 S32K312 LPUART0 的
//! 寄存器访问重定向到 MPS2 AN500 的 CMSDK APB UART（QEMU 侧基址 0x40004000），
//! 因此 dtwin 侧按同一寄存器布局建模即可捕获固件的串口输出。真实 S32K312 硬件上
//! LPUART0 位于 0x40180000，寄存器位定义不同，但本模型的 DATA 写即发送语义一致，
//! 可用同一模型（不同基址实例化）。
//!
//! 寄存器布局（32 位对齐）：
//! | 偏移 | 名称       | 语义                                                   |
//! |------|------------|--------------------------------------------------------|
//! | 0x00 | DATA       | 写 = 发送字符（捕获到输出缓冲 + 可选 stdout 回显）     |
//! | 0x04 | STATE      | bit0=txfull, bit1=rxfull, bit2=txbusy（TX 即时完成→0） |
//! | 0x08 | CTRL       | bit0=txen, bit1=rxen（读写保留）                        |
//! | 0x0C | INTSTATUS  | 恒 0                                                   |
//! | 0x10 | BAUDDIV    | 波特率分频（读写保留）                                  |
//!
//! 双通道输出：内部 `Vec<u8>` 缓冲（测试断言用）+ 可选 stdout 回显（CLI 用）。

use crate::peripheral::BusDevice;

/// 寄存器偏移
const REG_DATA: u32 = 0x000;
const REG_STATE: u32 = 0x004;
const REG_CTRL: u32 = 0x008;
const REG_INTSTATUS: u32 = 0x00C;
const REG_BAUDDIV: u32 = 0x010;
/// 寄存器窗口（0x000-0x013）
const WINDOW: u32 = 0x014;

/// 按访问宽度取掩码
fn width_mask(width: u32) -> u32 {
    match width {
        1 => 0xFF,
        2 => 0xFFFF,
        _ => 0xFFFF_FFFF,
    }
}

/// CMSDK APB UART 行为模型
#[derive(Debug)]
pub struct CmsdkUart {
    /// 基地址（绝对地址，如 QEMU 兼容固件 0x40004000 / 真实 S32K312 LPUART0 0x40180000）
    base: u32,
    /// CTRL 寄存器（bit0=txen, bit1=rxen）
    ctrl: u32,
    /// BAUDDIV 寄存器
    bauddiv: u32,
    /// DATA 写计数（可作输出字节数断言）
    tx_count: u64,
    /// 捕获的发送字节
    out: Vec<u8>,
    /// 是否同时回显到 stdout
    echo: bool,
}

impl CmsdkUart {
    /// 新建 UART 模型（默认不回显 stdout）
    pub fn new(base: u32) -> Self {
        Self::with_echo(base, false)
    }

    /// 新建 UART 模型，可开启 stdout 回显（CLI 运行固件时使用）
    pub fn with_echo(base: u32, echo: bool) -> Self {
        CmsdkUart {
            base,
            ctrl: 0,
            bauddiv: 0,
            tx_count: 0,
            out: Vec::new(),
            echo,
        }
    }

    /// 捕获的输出字节（测试断言用）
    pub fn output(&self) -> &[u8] {
        &self.out
    }

    /// 取走捕获输出（清空缓冲）
    pub fn take_output(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.out)
    }

    /// 已发送字节数
    pub fn tx_count(&self) -> u64 {
        self.tx_count
    }

    /// CTRL 寄存器当前值
    pub fn ctrl(&self) -> u32 {
        self.ctrl
    }

    /// BAUDDIV 寄存器当前值
    pub fn bauddiv(&self) -> u32 {
        self.bauddiv
    }

    /// 捕获一个发送字节（双通道：内部缓冲 + 可选 stdout）
    fn capture(&mut self, byte: u8) {
        self.out.push(byte);
        if self.echo {
            // 逐字节回显（终端友好；\n 由固件自行扩展为 \r\n）
            use std::io::Write;
            let mut stdout = std::io::stdout();
            let _ = stdout.write_all(&[byte]);
            let _ = stdout.flush();
        }
    }
}

impl BusDevice for CmsdkUart {
    fn name(&self) -> &'static str {
        "CMSDK-APB-UART"
    }

    fn base_address(&self) -> u32 {
        self.base
    }

    fn window_size(&self) -> u32 {
        WINDOW
    }

    fn read(&mut self, addr: u32, width: u32) -> u32 {
        let off = addr.wrapping_sub(self.base);
        let v = match off {
            REG_DATA => 0, // 无 RX 数据（TX 路径只写不读）
            REG_STATE => 0, // txfull=0 / rxfull=0 / txbusy=0（发送即时完成）
            REG_CTRL => self.ctrl,
            REG_INTSTATUS => 0,
            REG_BAUDDIV => self.bauddiv,
            _ => 0, // 保留寄存器读 0
        };
        v & width_mask(width)
    }

    fn write(&mut self, addr: u32, width: u32, val: u32) {
        let off = addr.wrapping_sub(self.base);
        let v = val & width_mask(width);
        match off {
            REG_DATA => {
                self.tx_count += 1;
                self.capture(v as u8);
            }
            REG_CTRL => self.ctrl = v,
            REG_BAUDDIV => self.bauddiv = v,
            _ => {} // STATE / INTSTATUS / 保留寄存器写忽略
        }
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_data_captured_in_buffer() {
        let mut uart = CmsdkUart::new(0x4000_4000);
        for (addr, b) in [
            (0x4000_4000, b'H'),
            (0x4000_4000, b'i'),
            (0x4000_4000, b'\r'),
            (0x4000_4000, b'\n'),
        ] {
            uart.write(addr, 4, b as u32);
        }
        assert_eq!(uart.output(), b"Hi\r\n");
        assert_eq!(uart.tx_count(), 4);
        // 32 位写低位字节被发送（小端寄存器语义）
        uart.write(0x4000_4000, 4, 0x0000_00AA);
        assert_eq!(uart.output()[4], 0xAA);
        // 16 位写
        uart.write(0x4000_4000, 2, 0x00BB);
        assert_eq!(uart.output()[5], 0xBB);
        // 8 位写
        uart.write(0x4000_4000, 1, 0xCC);
        assert_eq!(uart.output()[6], 0xCC);
        assert_eq!(uart.output().len(), 7);
    }

    #[test]
    fn ctrl_bauddiv_readback() {
        let mut uart = CmsdkUart::new(0x4000_4000);
        // uart_init 序列：CTRL=0 → BAUD=115200 → CTRL=TXEN|RXEN
        uart.write(0x4000_4008, 4, 0);
        uart.write(0x4000_4010, 4, 115200);
        uart.write(0x4000_4008, 4, 0x3);
        assert_eq!(uart.ctrl(), 0x3);
        assert_eq!(uart.bauddiv(), 115200);
        assert_eq!(uart.read(0x4000_4008, 4), 0x3);
        assert_eq!(uart.read(0x4000_4010, 4), 115200);
        // 宽度屏蔽
        assert_eq!(uart.read(0x4000_4008, 1), 0x3);
        assert_eq!(uart.read(0x4000_4008, 2), 0x3);
    }

    #[test]
    fn state_reads_zero_tx_ready() {
        let mut uart = CmsdkUart::new(0x4000_4000);
        // lpuart_tx_ready() = (STATE & TXFULL) == 0 → 恒 ready
        assert_eq!(uart.read(0x4000_4004, 4), 0);
        assert_eq!(uart.read(0x4000_4004, 1), 0);
        // 写 STATE 被忽略
        uart.write(0x4000_4004, 4, 0xFFFF_FFFF);
        assert_eq!(uart.read(0x4000_4004, 4), 0);
        // INTSTATUS 恒 0
        assert_eq!(uart.read(0x4000_400C, 4), 0);
        // 窗口外（保留区）读 0 写忽略
        assert_eq!(uart.read(0x4000_4014, 4), 0);
        uart.write(0x4000_4014, 4, 0x1234);
        assert_eq!(uart.read(0x4000_4014, 4), 0);
    }

    #[test]
    fn data_read_returns_zero_no_rx() {
        let mut uart = CmsdkUart::new(0x4000_4000);
        uart.write(0x4000_4000, 4, b'X' as u32);
        // DATA 读 = RX 路径，无数据 → 0
        assert_eq!(uart.read(0x4000_4000, 4), 0);
    }

    #[test]
    fn take_output_clears() {
        let mut uart = CmsdkUart::new(0x4000_4000);
        uart.write(0x4000_4000, 4, b'A' as u32);
        let v = uart.take_output();
        assert_eq!(v, b"A");
        assert!(uart.output().is_empty());
    }
}
