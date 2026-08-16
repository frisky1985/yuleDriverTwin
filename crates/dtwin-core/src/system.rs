//! Cortex-M 系统控制块（SCB）+ SysTick 行为模型（FRT-SYS-01~05）
//!
//! 覆盖地址窗口 0xE000E000-0xE000F000（SYSTEM 区）：
//! - SysTick（0xE000E010-0xE000E01C）：CTRL/LOAD/VAL/CALIB，周期递减 → 挂起异常 15
//! - SCB（0xE000ED00-0xE000ED48）：CPUID/ICSR/VTOR/AIRCR/SCR/CCR/SHPR1-3/CPACR
//! - FPU（0xE000EF34）：FPCCR（ASPEN/LSPEN 存储，供 FRT-EXC-09 判断）
//!
//! 与 CmsdkUart 同模式：实现 `BusDevice`，经 `Memory::attach_peripheral` 挂接，
//! 由 Memory 把 SYSTEM 区的读/写路由到本模型（FRT-CHIP-02）。
//! 周期行为（SysTick 递减）由引擎 run 循环经 `Memory::tick_system` 驱动（FRT-SYS-02）。

use crate::peripheral::BusDevice;

/// 寄存器偏移（相对 0xE000E000）
const SYST_CTRL: u32 = 0x010;
const SYST_LOAD: u32 = 0x014;
const SYST_VAL: u32 = 0x018;
const SYST_CALIB: u32 = 0x01C;
const SCB_CPUID: u32 = 0xD00;
const SCB_ICSR: u32 = 0xD04;
const SCB_VTOR: u32 = 0xD08;
const SCB_AIRCR: u32 = 0xD0C;
const SCB_SCR: u32 = 0xD10;
const SCB_CCR: u32 = 0xD14;
const SCB_SHPR1: u32 = 0xD18;
const SCB_SHPR2: u32 = 0xD1C;
const SCB_SHPR3: u32 = 0xD20;
const SCB_CPACR: u32 = 0xD88;
const FPU_FPCCR: u32 = 0xF34;

/// SysTick CTRL 位
const SYST_ENABLE: u32 = 1 << 0;
const SYST_TICKINT: u32 = 1 << 1;
/// CLKSOURCE（bit2）：本模型恒为引擎周期源（1 指令 = 1 周期），读写保留即可
const SYST_CLKSOURCE: u32 = 1 << 2;
const SYST_COUNTFLAG: u32 = 1 << 16;

/// 异常号常量
const EXC_SYSTICK: u8 = 15;

/// 系统控制块 + SysTick 行为模型
#[derive(Debug)]
pub struct SystemBlock {
    /// SysTick CTRL（bit0 ENABLE、bit1 TICKINT、bit2 CLKSOURCE、bit16 COUNTFLAG 只读）
    syst_ctrl: u32,
    /// SysTick LOAD（重载值）
    syst_load: u32,
    /// SysTick VAL（当前计数值；复位 0，写任意值清零）
    syst_val: u32,
    /// CALIB（只读；本模型返回 TENMS=0x027BCA 与 STCALIB 置位——mps2-an386 1MHz 参考时钟）
    syst_calib: u32,
    /// ICSR（PENDSVSET/PENDSVCLR/PENDSTSET/PENDSTCLR w1s/w1c；VECTACTIVE 由引擎同步）
    icsr: u32,
    /// VTOR（向量表基址；本阶段恒 0）
    vtor: u32,
    /// AIRCR（写忽略，读回 VECTKEY 域 0xFA05_0000）
    aircr: u32,
    /// SCR（睡模式控制，读写保留）
    scr: u32,
    /// CCR（bit9 STKALIGN 复位=1——异常入口 8 字节对齐依赖）
    ccr: u32,
    /// SHPR1（MemManage[31:24]/BusFault[23:16]/UsageFault[15:8]/保留[7:0]）
    shpr1: u32,
    /// SHPR2（SVCall[31:24]）
    shpr2: u32,
    /// SHPR3（SysTick[31:24]/PendSV[23:16]）
    shpr3: u32,
    /// CPACR（协处理器访问控制；CPU 侧以 CpuState.cpacr 为准，读写双向同步由引擎负责）
    cpacr: u32,
    /// FPCCR（bit31 ASPEN、bit30 LSPEN；懒压栈语义 SHOULD）
    fpccr: u32,
    /// 当前异常号（VECTACTIVE 读取；引擎在异常切换时同步）
    vectactive: u8,
    /// 已挂起的系统异常位图（bit n = 异常 n 挂起；由引擎仲裁消费）
    pended: u16,
}

impl SystemBlock {
    /// 设备名（供 `Memory::peripheral_mut_by_name` 定位）
    pub const NAME: &'static str = "SYSTEM-BLOCK";

    pub fn new() -> Self {
        SystemBlock {
            syst_ctrl: 0,
            syst_load: 0,
            syst_val: 0,
            // CALIB：TENMS=0x027BCA（1MHz → 1000Hz tick 的重载值），STCALIB(bit31)=1
            syst_calib: 0x8000_27CA,
            icsr: 0,
            vtor: 0,
            aircr: 0xFA05_0000,
            scr: 0,
            // CCR.STKALIGN=1（ARMv7-M 复位默认：异常入口压栈 8 字节对齐）
            ccr: 1 << 9,
            shpr1: 0,
            shpr2: 0,
            shpr3: 0,
            cpacr: 0x00F0_0000,
            fpccr: 0,
            vectactive: 0,
            pended: 0,
        }
    }

    /// 周期驱动：SysTick 递减（FRT-SYS-02）。
    /// 挂起位在内部记录（`pended`，供引擎仲裁消费）；返回值仅作信息提示
    /// （本周期新挂起的异常号，当前仅 SysTick=15，无则 0）。
    /// 语义（ARMv7-M）：ENABLE=1 时——VAL==0 视为「装载挂起」，下周期自 LOAD 装载
    /// （不触发）；装载后每周期减 1；减至 0 → COUNTFLAG 置位 + TICKINT=1 则挂起异常 15
    /// （下一周期再自 LOAD 重载）。LOAD=0 时计数器保持 0 不触发（ARM/QEMU 语义）。
    pub fn tick(&mut self, cycles: u64) -> Option<u8> {
        if self.syst_ctrl & SYST_ENABLE == 0 {
            return None;
        }
        let mut new_pend: Option<u8> = None;
        for _ in 0..cycles {
            if self.syst_load == 0 {
                // LOAD=0：计数器保持 0，不触发（ARM/QEMU 语义）
                self.syst_val = 0;
                continue;
            }
            if self.syst_val == 0 {
                // 装载挂起：自 LOAD 装载（不触发）
                self.syst_val = self.syst_load;
            } else if self.syst_val == 1 {
                // 减至 0：COUNTFLAG + 挂起异常 15（下一周期再重载）
                self.syst_val = 0;
                self.syst_ctrl |= SYST_COUNTFLAG;
                if self.syst_ctrl & SYST_TICKINT != 0 {
                    self.pend_exception(EXC_SYSTICK);
                    new_pend = Some(EXC_SYSTICK);
                }
            } else {
                self.syst_val -= 1;
            }
        }
        new_pend
    }

    /// 查询是否有挂起的系统异常（供引擎仲裁消费，FRT-EXC-05）
    pub fn next_pending_exception(&self) -> Option<u8> {
        for n in 1..16u8 {
            if self.pended & (1 << n) != 0 {
                return Some(n);
            }
        }
        None
    }

    /// 引擎挂起系统异常（ICSR 写与 SysTick tick 的公共入口）
    pub fn pend_exception(&mut self, n: u8) {
        if (1..16).contains(&n) {
            self.pended |= 1 << n;
        }
    }

    /// 引擎清除系统异常挂起（异常被接受时）
    pub fn unpend_exception(&mut self, n: u8) {
        if (1..16).contains(&n) {
            self.pended &= !(1 << n);
        }
    }

    /// 引擎同步当前异常号（VECTACTIVE 读取 + 异常嵌套计数）
    pub fn set_vectactive(&mut self, number: u8) {
        self.vectactive = number;
    }

    /// 读取 CPACR 当前值（引擎在 MSR/复位时同步 CPU 侧）
    pub fn cpacr_value(&self) -> u32 {
        self.cpacr
    }

    /// 写入 CPACR（引擎在 MRS/MSR 时同步 CPU 侧）
    pub fn set_cpacr(&mut self, v: u32) {
        self.cpacr = v;
    }

    /// FPCCR ASPEN/LSPEN 是否置位（FRT-EXC-09 懒压栈判断）
    pub fn fpccr_aspen_lspen(&self) -> bool {
        self.fpccr & 0xC000_0000 != 0
    }

    /// SysTick 当前计数值（测试断言）
    pub fn systick_val(&self) -> u32 {
        self.syst_val
    }

    /// SysTick LOAD（测试断言）
    pub fn systick_load(&self) -> u32 {
        self.syst_load
    }

    /// SysTick CTRL（测试断言）
    pub fn systick_ctrl(&self) -> u32 {
        self.syst_ctrl
    }

    /// SHPR1（MemManage/BusFault/UsageFault/DebugMonitor 优先级字节，FRT-SYS-04）
    pub fn shpr1(&self) -> u32 {
        self.shpr1
    }

    /// SHPR2（SVCall 优先级字节）
    pub fn shpr2(&self) -> u32 {
        self.shpr2
    }

    /// SHPR3（SysTick/PendSV 优先级字节）
    pub fn shpr3(&self) -> u32 {
        self.shpr3
    }

    /// VTOR（向量表基址；本阶段恒 0）
    pub fn vtor(&self) -> u32 {
        self.vtor
    }

    /// CCR.STKALIGN（异常入口 8 字节对齐开关；复位=1）
    pub fn ccr_stkalign(&self) -> bool {
        self.ccr & (1 << 9) != 0
    }

    /// 挂起位图全量（bit n = 异常 n 挂起；供引擎优先级仲裁扫描）
    pub fn pended_bits(&self) -> u16 {
        self.pended
    }

    /// 读寄存器（addr 为绝对地址；支持字节/半字/字 lane，按 width 屏蔽）
    /// 注：SysTick CTRL 整字读取后 COUNTFLAG 硬件自动清零（ARM/QEMU 读即清，FRT-SYS-01）
    fn read_reg(&mut self, addr: u32, width: u32) -> u32 {
        let word_mask = match width {
            1 => 0xFF,
            2 => 0xFFFF,
            _ => 0xFFFF_FFFF,
        };
        let word_addr = addr & !3;
        let off = word_addr - 0xE000_E000;
        let shift = (addr & 3) * 8;
        let mut v = match off {
            SYST_CTRL => {
                let v = self.syst_ctrl;
                // 读即清 COUNTFLAG（硬件语义，仅整字读取时清）
                self.syst_ctrl &= !SYST_COUNTFLAG;
                v
            }
            SYST_LOAD => self.syst_load,
            SYST_VAL => self.syst_val,
            SYST_CALIB => self.syst_calib,
            SCB_CPUID => 0x410F_C241, // Cortex-M4 r0p1
            SCB_ICSR => {
                // VECTACTIVE[8:0] = 当前异常号（线程模式 0）；其余位由内部状态给出
                (self.icsr & !0x1FF) | (self.vectactive as u32 & 0x1FF)
            }
            SCB_VTOR => self.vtor,
            SCB_AIRCR => self.aircr,
            SCB_SCR => self.scr,
            SCB_CCR => self.ccr,
            SCB_SHPR1 => self.shpr1,
            SCB_SHPR2 => self.shpr2,
            SCB_SHPR3 => self.shpr3,
            SCB_CPACR => self.cpacr,
            FPU_FPCCR => self.fpccr,
            _ => 0, // 窗口内未建模地址读 0（对齐既有 SYSTEM 区默认行为）
        };
        v = (v >> shift) & word_mask;
        v
    }

    /// 写寄存器（addr 为绝对地址；支持字节/半字/字 lane，val 已按 width 屏蔽）
    fn write_reg(&mut self, addr: u32, width: u32, val: u32) {
        let word_addr = addr & !3;
        let off = word_addr - 0xE000_E000;
        let shift = (addr & 3) * 8;
        let lane_mask: u32 = match width {
            1 => 0xFF,
            2 => 0xFFFF,
            _ => 0xFFFF_FFFF,
        } << shift;
        // 读-改-写（lane 粒度）：先取当前字值，再按 lane 掩码合入
        let current = match off {
            SYST_CTRL => self.syst_ctrl,
            SYST_LOAD => self.syst_load,
            SYST_VAL => self.syst_val,
            SCB_ICSR => self.icsr,
            SCB_VTOR => self.vtor,
            SCB_AIRCR => self.aircr,
            SCB_SCR => self.scr,
            SCB_CCR => self.ccr,
            SCB_SHPR1 => self.shpr1,
            SCB_SHPR2 => self.shpr2,
            SCB_SHPR3 => self.shpr3,
            SCB_CPACR => self.cpacr,
            FPU_FPCCR => self.fpccr,
            _ => return, // 窗口内未建模地址写忽略
        };
        let word = (current & !lane_mask) | ((val << shift) & lane_mask);
        match off {
            SYST_CTRL => {
                // 写 CTRL：ENABLE/TICKINT/CLKSOURCE 可写；COUNTFLAG 位写 1 清除（ARM 语义）
                let wmask = SYST_ENABLE | SYST_TICKINT | SYST_CLKSOURCE;
                let new_ctrl = (word & wmask) | (self.syst_ctrl & SYST_COUNTFLAG);
                if word & SYST_COUNTFLAG != 0 {
                    // 写 1 清 COUNTFLAG
                    self.syst_ctrl = new_ctrl & !SYST_COUNTFLAG;
                } else {
                    self.syst_ctrl = new_ctrl;
                }
            }
            SYST_LOAD => {
                self.syst_load = word & 0xFFFF_FFFF;
                // 写 LOAD 不影响当前 VAL（仅重载值更新；VAL 在下次归零/装载时重载）
            }
            SYST_VAL => {
                // 写 VAL：清 COUNTFLAG、计数器归零（ENABLE=1 时下一周期自 LOAD 装载）
                self.syst_val = 0;
                self.syst_ctrl &= !SYST_COUNTFLAG;
            }
            SCB_ICSR => {
                // w1s/w1c 语义（FRT-SYS-03）：仅对写 1 的位生效
                if word & (1 << 28) != 0 {
                    // PENDSVSET：挂起异常 14
                    self.pend_exception(14);
                }
                if word & (1 << 27) != 0 {
                    // PENDSVCLR：清除异常 14 挂起
                    self.unpend_exception(14);
                }
                if word & (1 << 26) != 0 {
                    // PENDSTSET：挂起异常 15
                    self.pend_exception(15);
                }
                if word & (1 << 25) != 0 {
                    // PENDSTCLR：清除异常 15 挂起
                    self.unpend_exception(15);
                }
                // 其余位（VECTACTIVE 等）只读；w1s/w1c 位不存储（挂起状态在 pended）
                self.icsr = word & !(0xF << 25);
            }
            SCB_VTOR => {
                // 本阶段：VTOR 仅支持读 0（FRT-SYS-05）；写存储但向量表仍从 0 读取
                self.vtor = 0;
            }
            SCB_AIRCR => {
                // VECTKEY=0x05FA 才有效；本阶段写忽略（无复位/PRIGROUP 语义）
            }
            SCB_SCR => {
                self.scr = word & 0x1F;
            }
            SCB_CCR => {
                // STKALIGN(bit9) 保留恒 1；其余位存储
                self.ccr = (word & !(1 << 9)) | (1 << 9);
            }
            SCB_SHPR1 => {
                self.shpr1 = word;
            }
            SCB_SHPR2 => {
                self.shpr2 = word;
            }
            SCB_SHPR3 => {
                self.shpr3 = word;
            }
            SCB_CPACR => {
                self.cpacr = word;
            }
            FPU_FPCCR => {
                // ASPEN(bit31)/LSPEN(bit30) 存储；其余位忽略
                self.fpccr = (self.fpccr & !0xC000_0000) | (word & 0xC000_0000);
            }
            _ => {
                // 窗口内未建模地址写忽略（对齐既有 SYSTEM 区默认行为）
            }
        }
    }
}

impl Default for SystemBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl BusDevice for SystemBlock {
    fn name(&self) -> &'static str {
        Self::NAME
    }

    fn base_address(&self) -> u32 {
        0xE000_E000
    }

    fn window_size(&self) -> u32 {
        0x1000
    }

    fn read(&mut self, addr: u32, width: u32) -> u32 {
        self.read_reg(addr, width)
    }

    fn write(&mut self, addr: u32, width: u32, val: u32) {
        self.write_reg(addr, width, val);
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn systick_ctrl_load_val_basic() {
        let mut sb = SystemBlock::new();
        // 写 CTRL：ENABLE+TICKINT+CLKSOURCE
        sb.write(0xE000_E010, 4, 0x7);
        assert_eq!(sb.read(0xE000_E010, 4) & 0x7, 0x7);
        // 写 LOAD=100
        sb.write(0xE000_E014, 4, 100);
        assert_eq!(sb.systick_load(), 100);
        // 写 VAL 清零（装载挂起）
        sb.write(0xE000_E018, 4, 0xDEAD);
        assert_eq!(sb.systick_val(), 0);
        // 时序（ARM 语义：写 VAL 后先自 LOAD 装载，再逐周期递减至 0 触发）：
        // tick1: 0→100（装载）；tick2..99: 98 次递减 → val=2（不触发）
        assert_eq!(sb.tick(99), None);
        assert_eq!(sb.systick_val(), 2);
        // tick: 2→1（不触发）
        assert_eq!(sb.tick(1), None);
        assert_eq!(sb.systick_val(), 1);
        // tick: 1→0 → TICKINT=1 挂起 SysTick + COUNTFLAG 置位
        let p = sb.tick(1);
        assert_eq!(p, Some(15), "TICKINT=1 → 挂起 SysTick");
        assert_eq!(sb.systick_val(), 0);
        assert_ne!(sb.read(0xE000_E010, 4) & SYST_COUNTFLAG, 0, "COUNTFLAG 置位");
        // 下一周期自 LOAD 重载
        assert_eq!(sb.tick(1), None);
        assert_eq!(sb.systick_val(), 100, "归零后自 LOAD 重载");
    }

    #[test]
    fn systick_read_ctrl_clears_countflag() {
        // ARM/QEMU 语义：读 CTRL 时 COUNTFLAG 硬件自动清零（读即清）。
        // 时序：LOAD=2 → tick1 装载 2、tick2 2→1、tick3 1→0 触发。
        let mut sb = SystemBlock::new();
        sb.write(0xE000_E010, 4, 0x3); // ENABLE+TICKINT
        sb.write(0xE000_E014, 4, 2);
        let _ = sb.tick(3);
        assert_ne!(sb.read(0xE000_E010, 4) & SYST_COUNTFLAG, 0);
        // 读回后 COUNTFLAG 清零
        assert_eq!(sb.read(0xE000_E010, 4) & SYST_COUNTFLAG, 0);
    }

    #[test]
    fn systick_load_zero_no_trigger() {
        let mut sb = SystemBlock::new();
        sb.write(0xE000_E010, 4, 0x7);
        sb.write(0xE000_E014, 4, 0);
        assert_eq!(sb.tick(1000), None, "LOAD=0 不触发");
        assert_eq!(sb.systick_val(), 0);
    }

    #[test]
    fn systick_disabled_no_tick() {
        let mut sb = SystemBlock::new();
        sb.write(0xE000_E014, 4, 10);
        assert_eq!(sb.tick(100), None, "ENABLE=0 不递减");
        assert_eq!(sb.systick_val(), 0);
    }

    #[test]
    fn systick_tickint_off_no_pend() {
        let mut sb = SystemBlock::new();
        sb.write(0xE000_E010, 4, 0x1); // 仅 ENABLE
        sb.write(0xE000_E014, 4, 3);
        // 时序：tick1 装载 3、tick2 3→2、tick3 2→1、tick4 1→0 触发（不挂起）
        let mut p = None;
        for _ in 0..4 {
            if let Some(x) = sb.tick(1) {
                p = Some(x);
            }
        }
        assert_eq!(p, None, "TICKINT=0 只置 COUNTFLAG 不挂起");
        assert_ne!(sb.read(0xE000_E010, 4) & SYST_COUNTFLAG, 0);
    }

    #[test]
    fn icsr_w1s_w1c_pend_semantics() {
        let mut sb = SystemBlock::new();
        // PENDSVSET（bit28 w1s）→ 挂起 14
        sb.write(0xE000_ED04, 4, 1 << 28);
        assert_eq!(sb.next_pending_exception(), Some(14));
        // PENDSVCLR（bit27 w1c）→ 清除 14
        sb.write(0xE000_ED04, 4, 1 << 27);
        assert_eq!(sb.next_pending_exception(), None);
        // PENDSTSET（bit26）→ 挂起 15；PENDSTCLR（bit25）→ 清除
        sb.write(0xE000_ED04, 4, 1 << 26);
        assert_eq!(sb.next_pending_exception(), Some(15));
        sb.write(0xE000_ED04, 4, 1 << 25);
        assert_eq!(sb.next_pending_exception(), None);
        // 写 0 位不生效（w1s 语义）
        sb.write(0xE000_ED04, 4, 0);
        assert_eq!(sb.next_pending_exception(), None);
        // VECTACTIVE 读取 = 引擎同步的当前异常号
        sb.set_vectactive(11);
        assert_eq!(sb.read(0xE000_ED04, 4) & 0x1FF, 11);
        sb.set_vectactive(0);
        assert_eq!(sb.read(0xE000_ED04, 4) & 0x1FF, 0);
    }

    #[test]
    fn scb_register_model() {
        let mut sb = SystemBlock::new();
        // CPUID = 0x410FC241（Cortex-M4 r0p1）
        assert_eq!(sb.read(0xE000_ED00, 4), 0x410F_C241);
        // VTOR 读 0
        assert_eq!(sb.read(0xE000_ED08, 4), 0);
        sb.write(0xE000_ED08, 4, 0x1000);
        assert_eq!(sb.read(0xE000_ED08, 4), 0, "本阶段 VTOR 仅支持 0");
        // AIRCR 读回 VECTKEY 域
        assert_eq!(sb.read(0xE000_ED0C, 4), 0xFA05_0000);
        // CCR.STKALIGN=1
        assert_ne!(sb.read(0xE000_ED14, 4) & (1 << 9), 0);
        // SHPR2/SHPR3 字节字段（SVCall[31:24] / SysTick[31:24] / PendSV[23:16]）
        sb.write(0xE000_ED1C, 4, 0x00FF_0000);
        assert_eq!((sb.read(0xE000_ED1C, 4) >> 24) & 0xFF, 0x00);
        sb.write(0xE000_ED1C, 4, 0xFF00_0000);
        assert_eq!((sb.read(0xE000_ED1C, 4) >> 24) & 0xFF, 0xFF, "SVCall 优先级 0xFF");
        sb.write(0xE000_ED20, 4, 0xFFFF_0000);
        assert_eq!((sb.read(0xE000_ED20, 4) >> 24) & 0xFF, 0xFF, "SysTick 优先级");
        assert_eq!((sb.read(0xE000_ED20, 4) >> 16) & 0xFF, 0xFF, "PendSV 优先级");
        // CPACR 读写
        sb.write(0xE000_ED88, 4, 0x00F0_0000);
        assert_eq!(sb.read(0xE000_ED88, 4), 0x00F0_0000);
        // FPCCR ASPEN/LSPEN
        assert!(!sb.fpccr_aspen_lspen());
        sb.write(0xE000_EF34, 4, 0xC000_0000);
        assert!(sb.fpccr_aspen_lspen());
        // 未建模地址读 0 / 写忽略
        assert_eq!(sb.read(0xE000_E400, 4), 0);
        sb.write(0xE000_E400, 4, 0x1234);
        assert_eq!(sb.read(0xE000_E400, 4), 0);
    }

    #[test]
    fn halfword_byte_width_masking() {
        let mut sb = SystemBlock::new();
        sb.write(0xE000_ED88, 4, 0xFFFF_FFFF);
        // 字节写 CPACR 高位字节（bits[31:24]）→ 只影响该字节
        sb.write(0xE000_ED88 + 3, 1, 0xAB);
        assert_eq!((sb.read(0xE000_ED88, 4) >> 24) & 0xFF, 0xAB);
        // 半字读掩码
        sb.write(0xE000_ED1C, 2, 0xCDEF);
        assert_eq!(sb.read(0xE000_ED1C, 2), 0xCDEF);
    }
}
