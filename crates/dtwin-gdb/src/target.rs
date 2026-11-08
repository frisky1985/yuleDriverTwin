//! GDB 调试目标：把 dtwin-core 的 CPU/内存/NVIC/引擎适配为 gdbstub Target。
//!
//! 设计要点：
//! - 断点/观察点由调试器侧维护，运行循环在每条指令前检查断点、每条指令后检查观察点，
//!   因此不需要改写 Flash 中的指令（Cortex-M 的代码区通常是只读的）。
//! - GDB 的 `cpsr` 与 Cortex-M 的 `xPSR` 位布局不同（T 位分别在 bit5 / bit24），
//!   读写寄存器时做显式转换。
//! - 单步通过 `Engine::run_until(..., 1)` 完成，保留 SysTick 递减与异常仲裁语义。

use std::collections::BTreeMap;

use gdbstub::arch::Arch;
use gdbstub::common::Signal;
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
use gdbstub_arch::arm::{ArmBreakpointKind, Armv4t};

use dtwin_core::engine::{Engine, EngineResult, FaultReason};
use dtwin_core::memory::{Memory, Watchpoint};
use dtwin_core::nvic::Nvic;
use dtwin_core::CpuState;

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
    type Arch = Armv4t;
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
        regs.cpsr = xpsr_to_cpsr(self.cpu.xpsr);
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
        self.cpu.xpsr = cpsr_to_xpsr(regs.cpsr, self.cpu.xpsr);
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

/// GDB CPSR 与 Cortex-M xPSR 的位布局转换。
///
/// ARM 经典 CPSR 的 Thumb 位在 bit5，Cortex-M xPSR 在 bit24；
/// 其余 APSR 标志位（NZCV/Q/GE/IT）布局一致。
fn xpsr_to_cpsr(xpsr: u32) -> u32 {
    (xpsr & !(1 << 24)) | (1 << 5)
}

/// 写回时保留当前 IPSR（异常号），把 GDB 给的 APSR/IT 位映射到 xPSR。
fn cpsr_to_xpsr(cpsr: u32, old_xpsr: u32) -> u32 {
    let flags = cpsr & 0xF800_0000; // N Z C V
    let q = cpsr & (1 << 27);
    let ge = cpsr & (0xF << 16);
    let it = cpsr & (0x3F << 10);
    let ipsr = old_xpsr & 0x1FF;
    flags | q | ge | it | ipsr | (1 << 24)
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
    fn xpsr_cpsr_roundtrip_preserves_flags() {
        let mut target = test_target();
        target.cpu.xpsr = (1 << 24) | 0xF000_0000 | (1 << 27);

        let mut regs = <<Armv4t as Arch>::Registers>::default();
        assert!(target.read_registers(&mut regs).is_ok());
        assert_eq!(regs.cpsr & (1 << 5), 1 << 5, "Thumb 位应在 CPSR bit5");
        assert_eq!(regs.cpsr & 0xF000_0000, 0xF000_0000);
        assert_eq!(regs.cpsr & (1 << 27), 1 << 27);

        assert!(target.write_registers(&regs).is_ok());
        assert_eq!(target.cpu.xpsr & (1 << 24), 1 << 24);
        assert_eq!(target.cpu.xpsr & 0xF000_0000, 0xF000_0000);
        assert_eq!(target.cpu.xpsr & (1 << 27), 1 << 27);
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
