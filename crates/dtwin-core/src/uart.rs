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

// ============================================================================
// Lpuart0Uart — 真实 S32K312 LPUART0 行为模型
// ============================================================================
//
// 寄存器布局以 yuleASR/src/platform/s32k312/include/S32K312.h 为准（只读参考）：
// | 偏移 | 名称   | 语义                                                    |
// |------|--------|---------------------------------------------------------|
// | 0x00 | VERID  | 版本 ID（只读，返回 0）                                  |
// | 0x04 | PARAM  | 参数（只读，返回 0）                                      |
// | 0x08 | GLOBAL | bit0=RST（写 1 软件复位）                                 |
// | 0x0C | PINCFG | 引脚配置（读写保留）                                      |
// | 0x10 | BAUD   | 波特率（SBR[12:0] 等，读写保留）                          |
// | 0x14 | STAT   | bit23=TDRE、bit24=TC、bit25=RDRF、bit26=RAF（W1C）        |
// | 0x18 | CTRL   | bit2=RE、bit3=TE（其余位读写保留）                         |
// | 0x1C | DATA   | 写低 9 位 = 发送字符；读 = RX（无数据→0，RDRF 保持 0）     |
// | 0x20+ | MATCH/MODIR/FIFO/WATER | 读写保留                     |
//
// 语义：复位后 STAT = TDRE|TC（发送就绪）；DATA 写即发送（瞬时完成，TDRE/TC 恒置位）；
// STAT 标志写 1 清零（W1C），但 TDRE/TC 因发送即时完成会立即重新置位。
// 双通道输出：内部 `Vec<u8>` 缓冲（测试断言用）+ 可选 stdout 回显（CLI 用）。

/// LPUART STAT 寄存器位（S32K312.h：LPUART_STAT_TDRE/TC/RDRF/RAF）
const STAT_TDRE: u32 = 0x0080_0000;
const STAT_TC: u32 = 0x0100_0000;
const STAT_RDRF: u32 = 0x0200_0000;
const STAT_RAF: u32 = 0x0400_0000;
/// LPUART 寄存器偏移（S32K312.h：LPUART_*_OFF）
const LPUART_GLOBAL_OFF: u32 = 0x08;
const LPUART_BAUD_OFF: u32 = 0x10;
const LPUART_STAT_OFF: u32 = 0x14;
const LPUART_CTRL_OFF: u32 = 0x18;
const LPUART_DATA_OFF: u32 = 0x1C;
/// CTRL 位（S32K312.h：LPUART_CTRL_RE/TE）
const CTRL_RE: u32 = 0x4;
const CTRL_TE: u32 = 0x8;
/// 寄存器窗口（0x000-0x02F）
const LPUART_WINDOW: u32 = 0x30;

/// 真实 S32K312 LPUART0 行为模型（与 CmsdkUart 并存）
#[derive(Debug)]
pub struct Lpuart0Uart {
    /// 基地址（S32K312 LPUART0 = 0x4018_0000）
    base: u32,
    /// STAT 寄存器（TDRE/TC/RDRF/RAF）
    stat: u32,
    /// CTRL 寄存器
    ctrl: u32,
    /// BAUD 寄存器
    baud: u32,
    /// GLOBAL 寄存器（写 RST 触发复位）
    global: u32,
    /// PINCFG / MATCH / MODIR / FIFO / WATER 保留寄存器
    reserved: [u32; 5],
    /// 捕获的发送字节
    out: Vec<u8>,
    /// 是否同时回显到 stdout
    echo: bool,
}

impl Lpuart0Uart {
    /// 新建 LPUART0 模型（默认不回显 stdout）
    pub fn new(base: u32) -> Self {
        Self::with_echo(base, false)
    }

    /// 新建 LPUART0 模型，可开启 stdout 回显（CLI 运行固件时使用）
    pub fn with_echo(base: u32, echo: bool) -> Self {
        let mut u = Lpuart0Uart {
            base,
            stat: 0,
            ctrl: 0,
            baud: 0,
            global: 0,
            reserved: [0; 5],
            out: Vec::new(),
            echo,
        };
        u.reset();
        u
    }

    /// 软件复位（GLOBAL.RST 写 1 或上电）：STAT = TDRE|TC，其余清零
    fn reset(&mut self) {
        self.stat = STAT_TDRE | STAT_TC;
        self.ctrl = 0;
        self.baud = 0;
        self.global = 0;
        self.reserved = [0; 5];
    }

    /// 捕获的输出字节（测试断言用）
    pub fn output(&self) -> &[u8] {
        &self.out
    }

    /// 取走捕获输出（清空缓冲）
    pub fn take_output(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.out)
    }

    /// STAT 寄存器当前值
    pub fn stat(&self) -> u32 {
        self.stat
    }

    /// CTRL 寄存器当前值
    pub fn ctrl(&self) -> u32 {
        self.ctrl
    }

    /// BAUD 寄存器当前值
    pub fn baud(&self) -> u32 {
        self.baud
    }

    /// 捕获一个发送字节（双通道：内部缓冲 + 可选 stdout）
    fn capture(&mut self, byte: u8) {
        self.out.push(byte);
        if self.echo {
            use std::io::Write;
            let mut stdout = std::io::stdout();
            let _ = stdout.write_all(&[byte]);
            let _ = stdout.flush();
        }
    }
}

impl BusDevice for Lpuart0Uart {
    fn name(&self) -> &'static str {
        "LPUART0"
    }

    fn base_address(&self) -> u32 {
        self.base
    }

    fn window_size(&self) -> u32 {
        LPUART_WINDOW
    }

    fn read(&mut self, addr: u32, width: u32) -> u32 {
        let off = addr.wrapping_sub(self.base);
        let mask = match width {
            1 => 0xFF,
            2 => 0xFFFF,
            _ => 0xFFFF_FFFF,
        };
        let v = match off {
            0x00 | 0x04 => 0,                          // VERID / PARAM 只读
            LPUART_GLOBAL_OFF => self.global,
            0x0C => self.reserved[0],                   // PINCFG
            LPUART_BAUD_OFF => self.baud,
            LPUART_STAT_OFF => self.stat,
            LPUART_CTRL_OFF => self.ctrl,
            LPUART_DATA_OFF => 0,                       // 无 RX 数据（RDRF 恒 0）
            0x20 => self.reserved[1],                   // MATCH
            0x24 => self.reserved[2],                   // MODIR
            0x28 => self.reserved[3],                   // FIFO
            0x2C => self.reserved[4],                   // WATER
            _ => 0,                                     // 窗口外保留
        };
        v & mask
    }

    fn write(&mut self, addr: u32, width: u32, val: u32) {
        let off = addr.wrapping_sub(self.base);
        let v = val & match width {
            1 => 0xFF,
            2 => 0xFFFF,
            _ => 0xFFFF_FFFF,
        };
        match off {
            LPUART_GLOBAL_OFF => {
                self.global = v & 1;
                if v & 1 != 0 {
                    self.reset(); // GLOBAL.RST 写 1 → 软件复位
                }
            }
            LPUART_BAUD_OFF => self.baud = v,
            LPUART_STAT_OFF => {
                // W1C：写 1 的位被清除；TDRE/TC 因发送即时完成立即重新置位
                self.stat &= !v;
                self.stat |= STAT_TDRE | STAT_TC;
            }
            LPUART_CTRL_OFF => self.ctrl = v,
            LPUART_DATA_OFF => {
                // 写 DATA 即发送（数据位 = 低 9 位，S32K312.h LPUART_DATA_MASK）
                if self.ctrl & CTRL_TE != 0 {
                    self.capture((v & 0x1FF) as u8);
                }
                self.stat |= STAT_TDRE | STAT_TC;
            }
            0x0C => self.reserved[0] = v, // PINCFG
            0x20 => self.reserved[1] = v, // MATCH
            0x24 => self.reserved[2] = v, // MODIR
            0x28 => self.reserved[3] = v, // FIFO
            0x2C => self.reserved[4] = v, // WATER
            _ => {}                       // VERID/PARAM/保留写忽略
        }
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod lpuart0_tests {
    use super::*;

    const BASE: u32 = 0x4018_0000;

    #[test]
    fn reset_sets_tdre_tc() {
        let mut u = Lpuart0Uart::new(BASE);
        assert_eq!(u.stat(), STAT_TDRE | STAT_TC, "复位后发送就绪");
        assert_eq!(u.ctrl(), 0);
        // TDRE/TC 按真实位定义（bit23/bit24）
        assert_eq!(u.read(BASE + LPUART_STAT_OFF, 4), STAT_TDRE | STAT_TC);
        assert_eq!(u.read(BASE + LPUART_STAT_OFF, 1), 0x00, "8 位读只取低字节");
    }

    #[test]
    fn tx_captured_when_te_enabled() {
        let mut u = Lpuart0Uart::new(BASE);
        // TE 未使能：写 DATA 不发送
        u.write(BASE + LPUART_DATA_OFF, 4, b'H' as u32);
        assert!(u.output().is_empty(), "TE=0 时 DATA 写被忽略");
        // 使能 TE（CTRL.bit3）
        u.write(BASE + LPUART_CTRL_OFF, 4, CTRL_TE);
        for b in [b'H', b'i', b'\r', b'\n'] {
            u.write(BASE + LPUART_DATA_OFF, 4, b as u32);
        }
        assert_eq!(u.output(), b"Hi\r\n");
        assert_eq!(u.ctrl(), CTRL_TE);
        // 16 位写低位字节（小端寄存器语义）
        u.write(BASE + LPUART_DATA_OFF, 2, 0x00AA);
        assert_eq!(u.output()[4], 0xAA);
    }

    #[test]
    fn stat_w1c_and_reassert() {
        let mut u = Lpuart0Uart::new(BASE);
        // 写 1 清 RDRF（无 RX，恒 0）；TDRE/TC 写 1 清除后立即重新置位
        u.write(BASE + LPUART_STAT_OFF, 4, STAT_TDRE | STAT_TC | STAT_RDRF);
        assert_eq!(
            u.stat(),
            STAT_TDRE | STAT_TC,
            "TDRE/TC 即时完成重新置位；RDRF 恒 0"
        );
        // 发送后 STAT 仍就绪
        u.write(BASE + LPUART_CTRL_OFF, 4, CTRL_TE | CTRL_RE);
        u.write(BASE + LPUART_DATA_OFF, 4, b'X' as u32);
        assert_eq!(u.read(BASE + LPUART_STAT_OFF, 4), STAT_TDRE | STAT_TC);
        assert_eq!(u.ctrl(), CTRL_TE | CTRL_RE);
    }

    #[test]
    fn global_rst_resets() {
        let mut u = Lpuart0Uart::new(BASE);
        u.write(BASE + LPUART_BAUD_OFF, 4, 0x1234);
        u.write(BASE + LPUART_CTRL_OFF, 4, CTRL_TE);
        u.write(BASE + LPUART_DATA_OFF, 4, b'A' as u32);
        // GLOBAL.RST 写 1 → 复位（输出缓冲保留，寄存器清零）
        u.write(BASE + LPUART_GLOBAL_OFF, 4, 1);
        assert_eq!(u.stat(), STAT_TDRE | STAT_TC);
        assert_eq!(u.ctrl(), 0);
        assert_eq!(u.baud(), 0);
        assert_eq!(u.output(), b"A", "复位不清输出缓冲（已有字节已发出）");
    }

    #[test]
    fn data_read_no_rx() {
        let mut u = Lpuart0Uart::new(BASE);
        u.write(BASE + LPUART_CTRL_OFF, 4, CTRL_TE | CTRL_RE);
        u.write(BASE + LPUART_DATA_OFF, 4, b'X' as u32);
        // DATA 读 = RX 路径，无数据 → 0；RDRF 不置位
        assert_eq!(u.read(BASE + LPUART_DATA_OFF, 4), 0);
        assert_eq!(u.stat() & STAT_RDRF, 0);
    }

    #[test]
    fn take_output_clears() {
        let mut u = Lpuart0Uart::new(BASE);
        u.write(BASE + LPUART_CTRL_OFF, 4, CTRL_TE);
        u.write(BASE + LPUART_DATA_OFF, 4, b'A' as u32);
        let v = u.take_output();
        assert_eq!(v, b"A");
        assert!(u.output().is_empty());
    }
}
