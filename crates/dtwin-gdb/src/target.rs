//! GDB 调试目标：把 dtwin-core 的 CPU/内存/NVIC/引擎适配为 gdbstub Target。
//!
//! 设计要点：
//! - 断点/观察点由调试器侧维护，运行循环在每条指令前检查断点、每条指令后检查观察点，
//!   因此不需要改写 Flash 中的指令（Cortex-M 的代码区通常是只读的）。
//! - 使用自定义 [`CortexM4F`] 架构，寄存器布局对齐 GDB M-profile 标准特性：
//!   xPSR 直接使用 M-profile 原生位布局（T 位在 bit24），无需 CPSR↔xPSR 转换。
//! - FPU 寄存器 D0-D15 与底层 S0-S31 别名互转（Dn = S(2n+1):S(2n)）。
//! - 单步通过 `Engine::run_until(..., 1)` 完成，保留 SysTick 递减与异常仲裁语义。

use std::collections::BTreeMap;

use gdbstub::arch::Arch;
use gdbstub::common::Signal;
use gdbstub::target::ext::base::single_register_access::{
    SingleRegisterAccess, SingleRegisterAccessOps,
};
use gdbstub::target::ext::base::singlethread::{
    SingleThreadBase, SingleThreadResume, SingleThreadResumeOps, SingleThreadSingleStep,
    SingleThreadSingleStepOps,
};
use gdbstub::target::ext::base::BaseOps;
use gdbstub::target::ext::breakpoints::{
    Breakpoints, BreakpointsOps, HwBreakpoint, HwBreakpointOps, HwWatchpoint, HwWatchpointOps,
    SwBreakpoint, SwBreakpointOps, WatchKind,
};
use gdbstub::target::ext::monitor_cmd::{ConsoleOutput, MonitorCmd, MonitorCmdOps};
use gdbstub::target::{Target, TargetError, TargetResult};
use gdbstub_arch::arm::ArmBreakpointKind;

use dtwin_core::engine::{Engine, EngineResult, FaultReason};
use dtwin_core::memory::{Memory, Watchpoint};
use dtwin_core::nvic::Nvic;
use dtwin_core::CpuState;

use crate::arch::{CortexM4F, CortexM4FRegId};
use crate::{GdbServerConfig, GdbTargetError, FPB_BREAKPOINT_MAX};

/// 每次进入运行循环时连续执行的指令块大小。
///
/// 块之间检查 GDB 连接是否有输入（Ctrl-C），避免长时间不响应中断。
const RUN_CHUNK: u64 = 256;

/// 硬件观察点数量上限（Cortex-M4 DWT 有 4 个比较器）
const DWT_WATCHPOINT_MAX: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BreakpointKind {
    Software,
    Hardware,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct GdbWatchpoint {
    address: u32,
    len: u32,
    kind: WatchKind,
}

/// 运行循环的返回事件
pub enum RunEvent {
    /// GDB 连接有输入（通常是 Ctrl-C），需要先处理协议数据
    IncomingData,
    /// 目标停止
    Stopped(gdbstub::stub::SingleThreadStopReason<u32>),
}

/// Driver Twin 的 GDB 调试目标
pub struct DtwinTarget {
    pub cpu: CpuState,
    pub mem: Memory,
    pub nvic: Nvic,
    engine: Engine,
    initial_cpu: CpuState,
    breakpoints: BTreeMap<u32, BreakpointKind>,
    watchpoints: Vec<GdbWatchpoint>,
    single_step: bool,
}

impl DtwinTarget {
    /// 使用已加载固件的 CPU/内存/NVIC 构造调试目标
    pub fn new(cpu: CpuState, mem: Memory, nvic: Nvic) -> Self {
        let initial_cpu = cpu.clone();
        Self {
            cpu,
            mem,
            nvic,
            engine: Engine::new(),
            initial_cpu,
            breakpoints: BTreeMap::new(),
            watchpoints: Vec::new(),
            single_step: false,
        }
    }

    /// 当前 PC（Thumb 地址去掉 bit0）
    pub fn pc(&self) -> u32 {
        self.cpu.regs[15] & !1
    }

    /// 运行统计数据
    pub fn stats(&self) -> &dtwin_core::engine::EngineStats {
        &self.engine.stats
    }

    /// 恢复复位状态：CPU 回到加载完成时的现场，NVIC/引擎统计清零，保留内存内容。
    pub fn reset(&mut self) {
        self.cpu = self.initial_cpu.clone();
        self.nvic = Nvic::new();
        self.engine = Engine::new();
        // 保留调试器设置的断点/观察点，只撤销已安装到内存的观察点
        self.mem.watchpoints.clear();
        self.mem.watchpoint_hit = None;
        self.single_step = false;
    }

    /// 当前是否使用 PSP（线程模式 + CONTROL.SPSEL=1）
    fn uses_psp(&self) -> bool {
        self.nvic.current_exception == 0 && (self.cpu.control & 1) != 0
    }

    /// 设置当前栈指针，并同步 MSP/PSP 影子字段
    fn set_current_sp(&mut self, sp: u32) {
        self.cpu.regs[13] = sp;
        if self.uses_psp() {
            self.cpu.psp = sp;
        } else {
            self.cpu.msp = sp;
        }
    }

    /// 把调试器设置的观察点安装到 Memory（运行期间生效）
    fn install_watchpoints(&mut self) {
        self.mem.watchpoints.clear();
        self.mem.watchpoint_hit = None;
        for wp in &self.watchpoints {
            self.mem.watchpoints.push(Watchpoint {
                address: wp.address,
                size: wp.len,
                on_write: matches!(wp.kind, WatchKind::Write | WatchKind::ReadWrite),
                on_read: matches!(wp.kind, WatchKind::Read | WatchKind::ReadWrite),
            });
        }
    }

    /// 停止后撤销观察点，避免 GDB 读内存时误触发
    fn remove_watchpoints(&mut self) {
        self.mem.watchpoints.clear();
        self.mem.watchpoint_hit = None;
    }

    /// 命中的观察点类型
    fn watch_kind_at(&self, address: u32) -> WatchKind {
        self.watchpoints
            .iter()
            .find(|wp| {
                let end = wp.address.saturating_add(wp.len);
                address >= wp.address && address < end
            })
            .map(|wp| wp.kind)
            .unwrap_or(WatchKind::Write)
    }

    /// 运行直到断点/观察点/异常/调试事件，或连接有待处理输入
    pub fn run(&mut self, mut poll: impl FnMut() -> bool) -> RunEvent {
        self.install_watchpoints();

        let event = if self.single_step {
            self.single_step = false;
            let result =
                self.engine
                    .run_until(&mut self.cpu, &mut self.mem, &mut self.nvic, &[], 1);
            self.map_stop(result, true)
        } else {
            // 从当前断点继续时先“跨过”它，兼容 GDB 的 always-inserted 模式
            if self.breakpoints.contains_key(&self.pc()) {
                let result =
                    self.engine
                        .run_until(&mut self.cpu, &mut self.mem, &mut self.nvic, &[], 1);
                if result != EngineResult::LimitReached {
                    let event = self.map_stop(result, false);
                    self.remove_watchpoints();
                    return event;
                }
            }

            loop {
                let breakpoints: Vec<u32> = self.breakpoints.keys().copied().collect();
                let result = self.engine.run_until(
                    &mut self.cpu,
                    &mut self.mem,
                    &mut self.nvic,
                    &breakpoints,
                    RUN_CHUNK,
                );
                if result == EngineResult::LimitReached {
                    if poll() {
                        break RunEvent::IncomingData;
                    }
                    continue;
                }
                break self.map_stop(result, false);
            }
        };

        self.remove_watchpoints();
        event
    }

    fn map_stop(&self, result: EngineResult, stepped: bool) -> RunEvent {
        use gdbstub::stub::SingleThreadStopReason;

        let reason = match result {
            EngineResult::LimitReached | EngineResult::Halted if stepped => {
                SingleThreadStopReason::DoneStep
            }
            EngineResult::LimitReached => {
                // 正常不会走到这里（continue 循环会继续跑）；保底按暂停处理
                SingleThreadStopReason::Signal(Signal::SIGINT)
            }
            EngineResult::Breakpoint { pc } => match self.breakpoints.get(&(pc & !1)) {
                Some(BreakpointKind::Hardware) => SingleThreadStopReason::HwBreak(()),
                _ => SingleThreadStopReason::SwBreak(()),
            },
            EngineResult::Watchpoint { address } => SingleThreadStopReason::Watch {
                tid: (),
                kind: self.watch_kind_at(address),
                addr: address,
            },
            EngineResult::DebugEvent => SingleThreadStopReason::Signal(Signal::SIGTRAP),
            EngineResult::Fault { reason } => SingleThreadStopReason::Signal(signal_for(reason)),
            EngineResult::Halted => SingleThreadStopReason::Signal(Signal::SIGTRAP),
        };
        RunEvent::Stopped(reason)
    }
}

impl Target for DtwinTarget {
    type Arch = CortexM4F;
    type Error = GdbTargetError;

    fn base_ops(&mut self) -> BaseOps<'_, Self::Arch, Self::Error> {
        BaseOps::SingleThread(self)
    }

    fn support_breakpoints(&mut self) -> Option<BreakpointsOps<'_, Self>> {
        Some(self)
    }

    fn support_monitor_cmd(&mut self) -> Option<MonitorCmdOps<'_, Self>> {
        Some(self)
    }
}

impl SingleThreadBase for DtwinTarget {
    fn read_registers(
        &mut self,
        regs: &mut <Self::Arch as Arch>::Registers,
    ) -> TargetResult<(), Self> {
        regs.r.copy_from_slice(&self.cpu.regs[..13]);
        regs.sp = self.cpu.regs[13];
        regs.lr = self.cpu.regs[14];
        regs.pc = self.cpu.regs[15] & !1;
        regs.xpsr = self.cpu.xpsr;
        regs.msp = self.cpu.msp;
        regs.psp = self.cpu.psp;
        regs.primask = self.cpu.primask as u32;
        regs.basepri = self.cpu.basepri as u32;
        regs.faultmask = self.cpu.faultmask as u32;
        regs.control = self.cpu.control as u32;
        // Dn = S(2n+1):S(2n)（高字为 S(2n+1)，低字为 S(2n)）
        for (i, d) in regs.d.iter_mut().enumerate() {
            let lo = self.cpu.fpu.s[2 * i];
            let hi = self.cpu.fpu.s[2 * i + 1];
            *d = (hi as u64) << 32 | lo as u64;
        }
        regs.fpscr = self.cpu.fpu.fpscr;
        Ok(())
    }

    fn write_registers(
        &mut self,
        regs: &<Self::Arch as Arch>::Registers,
    ) -> TargetResult<(), Self> {
        self.cpu.regs[..13].copy_from_slice(&regs.r);
        self.set_current_sp(regs.sp);
        self.cpu.regs[14] = regs.lr;
        self.cpu.regs[15] = regs.pc & !1;
        self.cpu.xpsr = regs.xpsr;
        self.cpu.msp = regs.msp;
        self.cpu.psp = regs.psp;
        self.cpu.primask = regs.primask as u8;
        self.cpu.basepri = regs.basepri as u8;
        self.cpu.faultmask = regs.faultmask as u8;
        self.cpu.control = regs.control as u8;
        for (i, d) in regs.d.iter().enumerate() {
            self.cpu.fpu.s[2 * i] = *d as u32;
            self.cpu.fpu.s[2 * i + 1] = (*d >> 32) as u32;
        }
        self.cpu.fpu.fpscr = regs.fpscr;
        Ok(())
    }

    fn read_addrs(&mut self, start_addr: u32, data: &mut [u8]) -> TargetResult<usize, Self> {
        let mut read = 0;
        while read < data.len() {
            let addr = start_addr.wrapping_add(read as u32);
            let remaining = data.len() - read;

            // 尽量按对齐宽度访问，保证外设寄存器（如 UART/SCB）读取语义正确
            let width = if addr % 4 == 0 && remaining >= 4 {
                4
            } else if addr % 2 == 0 && remaining >= 2 {
                2
            } else {
                1
            };

            let result = match width {
                4 => self.mem.read_u32(addr).map(|v| {
                    data[read..read + 4].copy_from_slice(&v.to_le_bytes());
                    4
                }),
                2 => self.mem.read_u16(addr).map(|v| {
                    data[read..read + 2].copy_from_slice(&v.to_le_bytes());
                    2
                }),
                _ => self.mem.read_u8(addr).map(|v| {
                    data[read] = v;
                    1
                }),
            };

            match result {
                Ok(n) => read += n,
                Err(_) if width > 1 => {
                    // 区域边界或外设不支持宽访问时退化为单字节
                    match self.mem.read_u8(addr) {
                        Ok(v) => {
                            data[read] = v;
                            read += 1;
                        }
                        Err(_) => {
                            if read == 0 {
                                return Err(TargetError::NonFatal);
                            }
                            break;
                        }
                    }
                }
                Err(_) => {
                    if read == 0 {
                        return Err(TargetError::NonFatal);
                    }
                    break;
                }
            }
        }
        Ok(read)
    }

    fn write_addrs(&mut self, start_addr: u32, data: &[u8]) -> TargetResult<(), Self> {
        let mut written = 0;
        while written < data.len() {
            let addr = start_addr.wrapping_add(written as u32);
            let remaining = data.len() - written;

            let width = if addr % 4 == 0 && remaining >= 4 {
                4
            } else if addr % 2 == 0 && remaining >= 2 {
                2
            } else {
                1
            };

            let result = match width {
                4 => {
                    let mut bytes = [0u8; 4];
                    bytes.copy_from_slice(&data[written..written + 4]);
                    self.mem.write_u32(addr, u32::from_le_bytes(bytes))
                }
                2 => {
                    let mut bytes = [0u8; 2];
                    bytes.copy_from_slice(&data[written..written + 2]);
                    self.mem.write_u16(addr, u16::from_le_bytes(bytes))
                }
                _ => self.mem.write_u8(addr, data[written]),
            };

            match result {
                Ok(()) => written += width,
                Err(_) if width > 1 => {
                    if self.mem.write_u8(addr, data[written]).is_err() {
                        return Err(TargetError::NonFatal);
                    }
                    written += 1;
                }
                Err(_) => return Err(TargetError::NonFatal),
            }
        }
        Ok(())
    }

    fn support_resume(&mut self) -> Option<SingleThreadResumeOps<'_, Self>> {
        Some(self)
    }

    fn support_single_register_access(&mut self) -> Option<SingleRegisterAccessOps<'_, (), Self>> {
        Some(self)
    }
}

impl SingleThreadResume for DtwinTarget {
    fn resume(&mut self, _signal: Option<Signal>) -> Result<(), Self::Error> {
        self.single_step = false;
        Ok(())
    }

    fn support_single_step(&mut self) -> Option<SingleThreadSingleStepOps<'_, Self>> {
        Some(self)
    }
}

impl SingleThreadSingleStep for DtwinTarget {
    fn step(&mut self, _signal: Option<Signal>) -> Result<(), Self::Error> {
        self.single_step = true;
        Ok(())
    }
}

impl SingleRegisterAccess<()> for DtwinTarget {
    fn read_register(
        &mut self,
        _tid: (),
        reg_id: <Self::Arch as Arch>::RegId,
        buf: &mut [u8],
    ) -> TargetResult<usize, Self> {
        let bytes: [u8; 8] = match reg_id {
            CortexM4FRegId::Gpr(n) => u32_to_le8(self.cpu.regs[n as usize]),
            CortexM4FRegId::Sp => u32_to_le8(self.cpu.regs[13]),
            CortexM4FRegId::Lr => u32_to_le8(self.cpu.regs[14]),
            CortexM4FRegId::Pc => u32_to_le8(self.cpu.regs[15] & !1),
            CortexM4FRegId::Xpsr => u32_to_le8(self.cpu.xpsr),
            CortexM4FRegId::Msp => u32_to_le8(self.cpu.msp),
            CortexM4FRegId::Psp => u32_to_le8(self.cpu.psp),
            CortexM4FRegId::Primask => u32_to_le8(self.cpu.primask as u32),
            CortexM4FRegId::Basepri => u32_to_le8(self.cpu.basepri as u32),
            CortexM4FRegId::Faultmask => u32_to_le8(self.cpu.faultmask as u32),
            CortexM4FRegId::Control => u32_to_le8(self.cpu.control as u32),
            CortexM4FRegId::D(n) => {
                let lo = self.cpu.fpu.s[2 * n as usize];
                let hi = self.cpu.fpu.s[2 * n as usize + 1];
                ((hi as u64) << 32 | lo as u64).to_le_bytes()
            }
            CortexM4FRegId::Fpscr => u32_to_le8(self.cpu.fpu.fpscr),
        };
        let n = if matches!(reg_id, CortexM4FRegId::D(_)) {
            8
        } else {
            4
        };
        if buf.len() < n {
            return Err(TargetError::NonFatal);
        }
        buf[..n].copy_from_slice(&bytes[..n]);
        Ok(n)
    }

    fn write_register(
        &mut self,
        _tid: (),
        reg_id: <Self::Arch as Arch>::RegId,
        val: &[u8],
    ) -> TargetResult<(), Self> {
        match reg_id {
            CortexM4FRegId::Gpr(n) => {
                self.cpu.regs[n as usize] = read_u32_le(val)?;
            }
            CortexM4FRegId::Sp => self.set_current_sp(read_u32_le(val)?),
            CortexM4FRegId::Lr => self.cpu.regs[14] = read_u32_le(val)?,
            CortexM4FRegId::Pc => self.cpu.regs[15] = read_u32_le(val)? & !1,
            CortexM4FRegId::Xpsr => self.cpu.xpsr = read_u32_le(val)?,
            CortexM4FRegId::Msp => self.cpu.msp = read_u32_le(val)?,
            CortexM4FRegId::Psp => self.cpu.psp = read_u32_le(val)?,
            CortexM4FRegId::Primask => self.cpu.primask = read_u32_le(val)? as u8,
            CortexM4FRegId::Basepri => self.cpu.basepri = read_u32_le(val)? as u8,
            CortexM4FRegId::Faultmask => self.cpu.faultmask = read_u32_le(val)? as u8,
            CortexM4FRegId::Control => self.cpu.control = read_u32_le(val)? as u8,
            CortexM4FRegId::D(n) => {
                if val.len() < 8 {
                    return Err(TargetError::NonFatal);
                }
                let lo = u32::from_le_bytes([val[0], val[1], val[2], val[3]]);
                let hi = u32::from_le_bytes([val[4], val[5], val[6], val[7]]);
                self.cpu.fpu.s[2 * n as usize] = lo;
                self.cpu.fpu.s[2 * n as usize + 1] = hi;
            }
            CortexM4FRegId::Fpscr => self.cpu.fpu.fpscr = read_u32_le(val)?,
        }
        Ok(())
    }
}

impl Breakpoints for DtwinTarget {
    fn support_sw_breakpoint(&mut self) -> Option<SwBreakpointOps<'_, Self>> {
        Some(self)
    }

    fn support_hw_breakpoint(&mut self) -> Option<HwBreakpointOps<'_, Self>> {
        Some(self)
    }

    fn support_hw_watchpoint(&mut self) -> Option<HwWatchpointOps<'_, Self>> {
        Some(self)
    }
}

impl SwBreakpoint for DtwinTarget {
    fn add_sw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        self.breakpoints.insert(addr & !1, BreakpointKind::Software);
        Ok(true)
    }

    fn remove_sw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        let addr = addr & !1;
        if self.breakpoints.get(&addr) == Some(&BreakpointKind::Software) {
            self.breakpoints.remove(&addr);
            Ok(true)
        } else {
            Ok(false)
        }
    }
}

impl HwBreakpoint for DtwinTarget {
    fn add_hw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        let hw_count = self
            .breakpoints
            .values()
            .filter(|k| **k == BreakpointKind::Hardware)
            .count();
        if hw_count >= FPB_BREAKPOINT_MAX {
            return Ok(false);
        }
        self.breakpoints.insert(addr & !1, BreakpointKind::Hardware);
        Ok(true)
    }

    fn remove_hw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        let addr = addr & !1;
        if self.breakpoints.get(&addr) == Some(&BreakpointKind::Hardware) {
            self.breakpoints.remove(&addr);
            Ok(true)
        } else {
            Ok(false)
        }
    }
}

impl HwWatchpoint for DtwinTarget {
    fn add_hw_watchpoint(
        &mut self,
        addr: u32,
        len: u32,
        kind: WatchKind,
    ) -> TargetResult<bool, Self> {
        if self.watchpoints.len() >= DWT_WATCHPOINT_MAX {
            return Ok(false);
        }
        self.watchpoints.push(GdbWatchpoint {
            address: addr,
            len: len.max(1),
            kind,
        });
        Ok(true)
    }

    fn remove_hw_watchpoint(
        &mut self,
        addr: u32,
        len: u32,
        kind: WatchKind,
    ) -> TargetResult<bool, Self> {
        let before = self.watchpoints.len();
        self.watchpoints
            .retain(|wp| !(wp.address == addr && wp.len == len.max(1) && wp.kind == kind));
        Ok(self.watchpoints.len() != before)
    }
}

impl MonitorCmd for DtwinTarget {
    fn handle_monitor_cmd(
        &mut self,
        cmd: &[u8],
        mut out: ConsoleOutput<'_>,
    ) -> Result<(), Self::Error> {
        let cmd = String::from_utf8_lossy(cmd);
        let cmd = cmd.trim();
        match cmd {
            "help" | "" => {
                gdbstub::outputln!(out, "dtwin monitor 命令:");
                gdbstub::outputln!(out, "  help          显示帮助");
                gdbstub::outputln!(out, "  reset         复位 CPU/NVIC（保留已加载内存）");
                gdbstub::outputln!(out, "  regs          打印核心寄存器");
                gdbstub::outputln!(
                    out,
                    "  fpu           打印 FPU 寄存器 S0-S31 / FPSCR / FPCCR"
                );
                gdbstub::outputln!(out, "  stats         打印引擎统计");
            }
            "reset" => {
                self.reset();
                gdbstub::outputln!(
                    out,
                    "已复位: PC={:#010x} SP={:#010x}",
                    self.pc(),
                    self.cpu.regs[13]
                );
            }
            "regs" => {
                for (i, r) in self.cpu.regs.iter().enumerate() {
                    gdbstub::outputln!(out, "R{:<2} = {:#010x}", i, r);
                }
                gdbstub::outputln!(out, "xPSR = {:#010x}", self.cpu.xpsr);
            }
            "fpu" => {
                for chunk in 0..4 {
                    let base = chunk * 8;
                    gdbstub::outputln!(
                        out,
                        "S{:<2}-S{:<2}: {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x}",
                        base,
                        base + 7,
                        self.cpu.fpu.s[base],
                        self.cpu.fpu.s[base + 1],
                        self.cpu.fpu.s[base + 2],
                        self.cpu.fpu.s[base + 3],
                        self.cpu.fpu.s[base + 4],
                        self.cpu.fpu.s[base + 5],
                        self.cpu.fpu.s[base + 6],
                        self.cpu.fpu.s[base + 7],
                    );
                }
                gdbstub::outputln!(
                    out,
                    "FPSCR={:#010x} FPCCR={:#010x}",
                    self.cpu.fpu.fpscr,
                    self.cpu.fpu.fpccr
                );
            }
            "stats" => {
                let s = &self.engine.stats;
                gdbstub::outputln!(
                    out,
                    "instructions={} cycles={} faults={} exceptions={}",
                    s.instructions,
                    s.cycles,
                    s.faults,
                    s.exceptions
                );
            }
            other => {
                gdbstub::outputln!(out, "未知命令: {other}（输入 monitor help 查看帮助）");
            }
        }
        Ok(())
    }
}

/// 把 u32 扩展为 8 字节小端缓冲，供单寄存器读统一返回 [u8;8]。
fn u32_to_le8(v: u32) -> [u8; 8] {
    let mut b = [0u8; 8];
    b[..4].copy_from_slice(&v.to_le_bytes());
    b
}

/// 从单寄存器写缓冲中解析 u32（小端）。
fn read_u32_le(val: &[u8]) -> TargetResult<u32, DtwinTarget> {
    if val.len() < 4 {
        return Err(TargetError::NonFatal);
    }
    Ok(u32::from_le_bytes([val[0], val[1], val[2], val[3]]))
}

fn signal_for(reason: FaultReason) -> Signal {
    match reason {
        FaultReason::UnalignedAccess { .. } => Signal::SIGBUS,
        FaultReason::UsageFault { .. }
        | FaultReason::IllegalInstruction { .. }
        | FaultReason::UnimplementedInstr => Signal::SIGILL,
        FaultReason::MemManage { .. }
        | FaultReason::BusFault { .. }
        | FaultReason::HardFault { .. } => Signal::SIGSEGV,
    }
}

/// 供 CLI 打印的连接信息
pub fn describe_listener(config: &GdbServerConfig) -> String {
    format!("{}:{}", config.host, config.port)
}

#[cfg(test)]
mod tests {
    use super::*;
    use gdbstub::stub::SingleThreadStopReason;

    fn blank_target() -> DtwinTarget {
        let mem = Memory::test_ram();
        let mut cpu = CpuState::default();
        cpu.regs[13] = 0x2000_0F00;
        cpu.msp = 0x2000_0F00;
        cpu.regs[15] = 0;
        cpu.xpsr = 1 << 24; // Thumb
        DtwinTarget::new(cpu, mem, Nvic::new())
    }

    fn test_target() -> DtwinTarget {
        let mut target = blank_target();
        // NOP; NOP; NOP; B .
        target
            .mem
            .load_bytes(0, &[0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0xFE, 0xE7])
            .unwrap();
        target
    }

    #[test]
    fn software_breakpoint_stops_before_execution() {
        let mut target = test_target();
        assert!(matches!(
            target.add_sw_breakpoint(4, ArmBreakpointKind::Thumb16),
            Ok(true)
        ));

        match target.run(|| false) {
            RunEvent::Stopped(SingleThreadStopReason::SwBreak(())) => {}
            other => panic!(
                "期望 SwBreak，实际 {:?}",
                match other {
                    RunEvent::Stopped(r) => format!("{r:?}"),
                    RunEvent::IncomingData => "IncomingData".to_string(),
                }
            ),
        }
        assert_eq!(target.pc(), 4);
        assert_eq!(target.stats().instructions, 2);
    }

    #[test]
    fn continue_from_breakpoint_skips_it() {
        let mut target = test_target();
        assert!(matches!(
            target.add_sw_breakpoint(4, ArmBreakpointKind::Thumb16),
            Ok(true)
        ));
        assert!(matches!(
            target.add_sw_breakpoint(6, ArmBreakpointKind::Thumb16),
            Ok(true)
        ));

        match target.run(|| false) {
            RunEvent::Stopped(SingleThreadStopReason::SwBreak(())) => assert_eq!(target.pc(), 4),
            _ => panic!("第一次应停在 4"),
        }
        match target.run(|| false) {
            RunEvent::Stopped(SingleThreadStopReason::SwBreak(())) => assert_eq!(target.pc(), 6),
            _ => panic!("第二次应跨过 4 停在 6"),
        }
    }

    #[test]
    fn single_step_executes_one_instruction() {
        let mut target = test_target();
        assert!(matches!(
            target.add_sw_breakpoint(4, ArmBreakpointKind::Thumb16),
            Ok(true)
        ));
        let _ = target.run(|| false);

        target.step(None).unwrap();
        match target.run(|| false) {
            RunEvent::Stopped(SingleThreadStopReason::DoneStep) => {}
            _ => panic!("期望 DoneStep"),
        }
        assert_eq!(target.pc(), 6);
    }

    #[test]
    fn watchpoint_stops_after_write() {
        let mut target = blank_target();
        // MOVS r0,#1; LDR r1,[pc,#4]; STR r0,[r1]; B .; .word 0x20000000
        target
            .mem
            .load_bytes(
                0,
                &[
                    0x01, 0x20, // 0x2001
                    0x01, 0x49, // 0x4901 -> literal at 8
                    0x08, 0x60, // 0x6008
                    0xFE, 0xE7, // 0xE7FE
                    0x00, 0x00, 0x00, 0x20,
                ],
            )
            .unwrap();
        assert!(matches!(
            target.add_hw_watchpoint(0x2000_0000, 4, WatchKind::Write),
            Ok(true)
        ));

        match target.run(|| false) {
            RunEvent::Stopped(SingleThreadStopReason::Watch { kind, addr, .. }) => {
                assert_eq!(kind, WatchKind::Write);
                assert_eq!(addr, 0x2000_0000);
            }
            _ => panic!("期望 Watch 停止"),
        }
        assert_eq!(target.pc(), 6);
    }

    #[test]
    fn cortexm4f_registers_roundtrip() {
        use crate::arch::CortexM4FRegs;

        let mut target = test_target();
        target.cpu.xpsr = (1 << 24) | 0xF000_0000 | (1 << 27) | 3; // T + NZCV + Q + IPSR=3
        target.cpu.msp = 0x2000_1000;
        target.cpu.psp = 0x2000_2000;
        target.cpu.primask = 1;
        target.cpu.basepri = 0x80;
        target.cpu.faultmask = 1;
        target.cpu.control = 2;
        // 让 S0/S1 组成 D0，S30/S31 组成 D15
        target.cpu.fpu.s[0] = 0x1111_1111;
        target.cpu.fpu.s[1] = 0x2222_2222;
        target.cpu.fpu.s[30] = 0xAAAA_AAAA;
        target.cpu.fpu.s[31] = 0xBBBB_BBBB;
        target.cpu.fpu.fpscr = 0x0300_0010;

        let mut regs = CortexM4FRegs::default();
        assert!(target.read_registers(&mut regs).is_ok());

        // xPSR 原生布局：T 位在 bit24，标志位不变
        assert_eq!(regs.xpsr & (1 << 24), 1 << 24, "T 位应在 xPSR bit24");
        assert_eq!(regs.xpsr & 0xF000_0000, 0xF000_0000);
        assert_eq!(regs.xpsr & (1 << 27), 1 << 27);
        assert_eq!(regs.xpsr & 0x1FF, 3, "IPSR 应保留");

        assert_eq!(regs.msp, 0x2000_1000);
        assert_eq!(regs.psp, 0x2000_2000);
        assert_eq!(regs.primask, 1);
        assert_eq!(regs.basepri, 0x80);
        assert_eq!(regs.faultmask, 1);
        assert_eq!(regs.control, 2);

        // D0 = S1:S0，D15 = S31:S30
        assert_eq!(regs.d[0], 0x2222_2222_1111_1111);
        assert_eq!(regs.d[15], 0xBBBB_BBBB_AAAA_AAAA);
        assert_eq!(regs.fpscr, 0x0300_0010);

        // 写回应保持一致
        assert!(target.write_registers(&regs).is_ok());
        assert_eq!(target.cpu.xpsr, (1 << 24) | 0xF000_0000 | (1 << 27) | 3);
        assert_eq!(target.cpu.fpu.s[0], 0x1111_1111);
        assert_eq!(target.cpu.fpu.s[1], 0x2222_2222);
        assert_eq!(target.cpu.fpu.s[30], 0xAAAA_AAAA);
        assert_eq!(target.cpu.fpu.s[31], 0xBBBB_BBBB);
        assert_eq!(target.cpu.fpu.fpscr, 0x0300_0010);
    }

    #[test]
    fn single_register_access_rw() {
        let mut target = test_target();
        target.cpu.regs[5] = 0xDEAD_BEEF;
        target.cpu.xpsr = 0x0100_0000;
        target.cpu.fpu.s[0] = 0x1234_5678;
        target.cpu.fpu.s[1] = 0x9ABC_DEF0;
        target.cpu.fpu.fpscr = 0x0100_0000;

        // 读 R5
        let mut buf = [0u8; 8];
        let n = target
            .read_register((), CortexM4FRegId::Gpr(5), &mut buf)
            .ok()
            .expect("读 R5 应成功");
        assert_eq!(n, 4);
        assert_eq!(
            u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            0xDEAD_BEEF
        );

        // 读 xpsr
        let n = target
            .read_register((), CortexM4FRegId::Xpsr, &mut buf)
            .ok()
            .expect("读 xpsr 应成功");
        assert_eq!(n, 4);
        assert_eq!(
            u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            0x0100_0000
        );

        // 读 D0（8 字节）
        let mut buf8 = [0u8; 8];
        let n = target
            .read_register((), CortexM4FRegId::D(0), &mut buf8)
            .ok()
            .expect("读 D0 应成功");
        assert_eq!(n, 8);
        assert_eq!(u64::from_le_bytes(buf8), 0x9ABC_DEF0_1234_5678);

        // 写 R5
        assert!(target
            .write_register((), CortexM4FRegId::Gpr(5), &0xCAFE_BABE_u32.to_le_bytes())
            .is_ok());
        assert_eq!(target.cpu.regs[5], 0xCAFE_BABE);

        // 写 D1（S2/S3）
        let d1 = 0x1111_1111_2222_2222_u64.to_le_bytes();
        assert!(target.write_register((), CortexM4FRegId::D(1), &d1).is_ok());
        assert_eq!(target.cpu.fpu.s[2], 0x2222_2222);
        assert_eq!(target.cpu.fpu.s[3], 0x1111_1111);
    }

    #[test]
    fn monitor_reset_restores_initial_pc() {
        let mut target = test_target();
        target.cpu.regs[15] = 6;
        target.cpu.regs[0] = 0xDEAD_BEEF;
        target.reset();
        assert_eq!(target.pc(), 0);
        assert_eq!(target.cpu.regs[0], 0);
        assert_eq!(target.cpu.regs[13], 0x2000_0F00);
    }
}
