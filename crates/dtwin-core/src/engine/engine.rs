//! 引擎主循环 — 取指/解码/执行 + 异常处理
//!
//! 串联: Memory(取指) → Decoder(解码) → Executor(执行) → Nvic(异常)

use super::decode::Decoder;
use super::exec::{ExecOutcome, Executor};
use super::FaultReason;
use crate::memory::Memory;
use crate::nvic::Nvic;
use crate::CpuState;

/// 引擎运行统计
#[derive(Debug, Default, Clone, Copy)]
pub struct EngineStats {
    pub instructions: u64,
    pub cycles: u64,
    pub faults: u64,
    pub exceptions: u64,
}

/// 引擎执行结果
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineResult {
    /// 正常停止（达到指令上限或显式暂停）
    Halted,
    /// 触发未处理异常
    Fault { reason: FaultReason },
    /// 调试事件（BKPT 触发）
    DebugEvent,
    /// 达到指令数上限
    LimitReached,
}

/// 内核引擎
pub struct Engine {
    decoder: Decoder,
    executor: Executor,
    pub stats: EngineStats,
    /// 单次 run 的指令上限（防死循环）
    pub max_instructions: u64,
    /// 异常向量表基地址（通常 0x0000_0000）
    pub vector_base: u32,
}

impl Engine {
    pub fn new() -> Self {
        Self {
            decoder: Decoder::new(),
            executor: Executor::new(),
            stats: EngineStats::default(),
            max_instructions: 1_000_000,
            vector_base: 0,
        }
    }

    /// 单步执行一条指令（供调试器使用）
    pub fn step(&mut self, cpu: &mut CpuState, mem: &mut Memory, nvic: &mut Nvic) -> EngineResult {
        // 取指
        let pc = cpu.regs[15];
        let raw = match mem.read_u16(pc) {
            Ok(v) => v,
            Err(_) => {
                self.stats.faults += 1;
                return EngineResult::Fault {
                    reason: FaultReason::BusFault { address: pc },
                };
            }
        };

        // 判断指令宽度（16-bit vs 32-bit）：0xE000-0xFFFF 高半字且非 0xExxx 是 32-bit 前缀
        let instr = if (raw & 0xF800) == 0xE800 || (raw & 0xF000) == 0xF000 {
            // 32-bit Thumb-2：读高半字拼接
            let hi = match mem.read_u16(pc + 2) {
                Ok(v) => v,
                Err(_) => {
                    self.stats.faults += 1;
                    return EngineResult::Fault {
                        reason: FaultReason::BusFault { address: pc + 2 },
                    };
                }
            };
            let full = ((raw as u32) << 16) | hi as u32;
            self.executor.cur_is_16bit = false;
            self.decoder.decode_word(full, pc)
        } else {
            self.executor.cur_is_16bit = true;
            self.decoder.decode_halfword(raw, pc)
        };

        // 执行
        let outcome = self.executor.execute(cpu, mem, &instr);
        self.stats.instructions += 1;
        self.stats.cycles = self.executor.cycle_count;

        match outcome {
            ExecOutcome::Continue => {
                // PC 默认 +2/+4（分支指令自行改 PC）
                let width = if (raw & 0xF800) == 0xE800 || (raw & 0xF000) == 0xF000 {
                    4
                } else {
                    2
                };
                cpu.regs[15] = cpu.regs[15].wrapping_add(width);
                EngineResult::Halted // 单步：返回暂停
            }
            // IT 条件不成立被跳过：PC 照常前进（指令已计数）
            ExecOutcome::Skipped => {
                let width = if (raw & 0xF800) == 0xE800 || (raw & 0xF000) == 0xF000 {
                    4
                } else {
                    2
                };
                cpu.regs[15] = cpu.regs[15].wrapping_add(width);
                self.stats.cycles = self.executor.cycle_count;
                EngineResult::Halted
            }
            ExecOutcome::Branch { target } => {
                cpu.regs[15] = target;
                EngineResult::Halted
            }
            ExecOutcome::ExceptionReturn { exc_return } => {
                // P3：BX EXC_RETURN → 真实异常出口（弹栈/恢复/切线程模式）
                self.stats.exceptions += 1;
                self.return_from_exception(cpu, mem, nvic, exc_return);
                EngineResult::Halted
            }
            // SVC 等指令触发异常入口（压栈/跳向量由引擎完成）
            ExecOutcome::TakeException { number, return_pc } => {
                self.stats.exceptions += 1;
                self.take_exception(cpu, mem, nvic, number, Some(return_pc));
                EngineResult::Halted
            }
            // 调试事件（BKPT）：ITSTATE 清零（异常语义），统计并返回
            ExecOutcome::DebugEvent => {
                self.stats.exceptions += 1;
                self.executor.clear_it();
                EngineResult::DebugEvent
            }
            ExecOutcome::Fault { reason } => {
                self.stats.faults += 1;
                // 异常入口清除 ITSTATE（ARMv7-M 异常语义）
                self.executor.clear_it();
                EngineResult::Fault { reason }
            }
        }
    }

    /// 全速运行直到达到指令上限或触发异常
    pub fn run(&mut self, cpu: &mut CpuState, mem: &mut Memory, nvic: &mut Nvic) -> EngineResult {
        let start = self.stats.instructions;
        loop {
            if self.stats.instructions - start >= self.max_instructions {
                return EngineResult::LimitReached;
            }
            // 异常仲裁（FRT-EXC-05/06）：存在可接受（未屏蔽 + 可抢占）的挂起异常 → 入口
            if let Some(number) = self.pending_exception_to_take(cpu, mem, nvic) {
                // 异步异常：现场帧 PC 槽 = 当前 PC（被中断流下一条待执行指令）
                self.take_exception(cpu, mem, nvic, number, None);
                self.stats.exceptions += 1;
                continue;
            }
            match self.step(cpu, mem, nvic) {
                EngineResult::Halted => {
                    // 周期驱动：SysTick 递减（1 指令 = 1 周期，FRT-SYS-02）
                    let _pend = mem.tick_system(1);
                    // 同步 VECTACTIVE（ICSR 读）与 SystemBlock 当前异常号
                    if let Some(sb) = mem.system_block_mut() {
                        sb.set_vectactive(nvic.current_exception);
                    }
                    // 单步返回 Halted，run 循环继续
                    continue;
                }
                r => return r,
            }
        }
    }

    // ==================== P3：异常机制（FRT-EXC-01~10） ====================

    /// 可配置异常优先级数值（越小越高）：
    /// NMI=0（固定最高）、HardFault=1（固定次高）、其余 = 2 + SHPR/NVIC 值（0-255）
    fn exception_priority(&mut self, mem: &mut Memory, nvic: &Nvic, number: u8) -> u16 {
        let cfg = self.configurable_priority(mem, nvic, number);
        match number {
            2 => 0,
            3 => 1,
            _ => 2 + cfg as u16,
        }
    }

    /// 可配置优先级来源：系统异常取 SHPR1-3 字节，外部 IRQ 取 NVIC priority 数组
    fn configurable_priority(&mut self, mem: &mut Memory, nvic: &Nvic, number: u8) -> u8 {
        match number {
            4 | 5 | 6 | 12 => {
                let v = mem
                    .system_block_mut()
                    .map(|sb| sb.shpr1())
                    .unwrap_or(0);
                let byte = match number {
                    4 => 3,  // MemManage [31:24]
                    5 => 2,  // BusFault [23:16]
                    6 => 1,  // UsageFault [15:8]
                    _ => 0,  // DebugMonitor [7:0]
                };
                ((v >> (byte * 8)) & 0xFF) as u8
            }
            11 => ((mem.system_block_mut().map(|sb| sb.shpr2()).unwrap_or(0) >> 24) & 0xFF) as u8,
            14 | 15 => {
                let v = mem
                    .system_block_mut()
                    .map(|sb| sb.shpr3())
                    .unwrap_or(0);
                let byte = if number == 14 { 2 } else { 3 }; // PendSV [23:16] / SysTick [31:24]
                ((v >> (byte * 8)) & 0xFF) as u8
            }
            n if n >= 16 => nvic.priority[(n - 16) as usize],
            _ => 0,
        }
    }

    /// 屏蔽检查（FRT-EXC-06）：PRIMASK/FAULTMASK/BASEPRI 约束
    fn is_masked(&self, cpu: &CpuState, number: u8, pri: u16) -> bool {
        match number {
            // NMI 永不被屏蔽
            2 => false,
            // HardFault：仅 FAULTMASK 屏蔽
            3 => cpu.faultmask != 0,
            // 可配置异常（4+）
            _ => {
                if cpu.faultmask != 0 || cpu.primask != 0 {
                    true
                } else if cpu.basepri != 0 && pri >= 2 + cpu.basepri as u16 {
                    // BASEPRI=N 屏蔽优先级数值 ≥ N 的异常（数值 < N 的高优先级仍可抢占）
                    true
                } else {
                    false
                }
            }
        }
    }

    /// 当前执行优先级：线程模式基线 512；Handler = 当前异常优先级（FRT-EXC-05）
    fn current_priority(&mut self, mem: &mut Memory, nvic: &Nvic, _cpu: &CpuState) -> u16 {
        if nvic.current_exception == 0 {
            512
        } else {
            self.exception_priority(mem, nvic, nvic.current_exception)
        }
    }

    /// 仲裁：扫描系统挂起 + 外部 IRQ 挂起，选出可接受（未屏蔽且可抢占）的最高优先级异常。
    /// 同优先级不抢占（保持挂起）。返回异常号（1-255），无则 None。
    fn pending_exception_to_take(
        &mut self,
        cpu: &mut CpuState,
        mem: &mut Memory,
        nvic: &Nvic,
    ) -> Option<u8> {
        let current = self.current_priority(mem, nvic, cpu);
        let mut best: Option<(u16, u8)> = None;
        // 系统异常（SystemBlock.pended）
        let sys_bits = mem
            .system_block_mut()
            .map(|sb| sb.pended_bits())
            .unwrap_or(0);
        for number in 1..16u8 {
            if sys_bits & (1 << number) == 0 {
                continue;
            }
            let pri = self.exception_priority(mem, nvic, number);
            if self.is_masked(cpu, number, pri) || pri >= current {
                continue;
            }
            if best.map_or(true, |(bp, _)| pri < bp) {
                best = Some((pri, number));
            }
        }
        // 外部 IRQ（NVIC pending & enabled）
        for irq in 0..240u16 {
            if nvic.pending[(irq / 32) as usize] & (1 << (irq % 32)) == 0 {
                continue;
            }
            if nvic.enabled[(irq / 32) as usize] & (1 << (irq % 32)) == 0 {
                continue;
            }
            let number = (irq + 16) as u8;
            let pri = self.exception_priority(mem, nvic, number);
            if self.is_masked(cpu, number, pri) || pri >= current {
                continue;
            }
            if best.map_or(true, |(bp, _)| pri < bp) {
                best = Some((pri, number));
            }
        }
        best.map(|(_, n)| n)
    }

    /// 异常入口（FRT-EXC-01/03/04/08/09）：压栈 → EXC_RETURN → IPSR → 切 Handler → 跳向量
    /// return_pc：同步异常（SVC）传下一条指令地址；异步异常（仲裁路径）传 None（用当前 PC）
    fn take_exception(
        &mut self,
        cpu: &mut CpuState,
        mem: &mut Memory,
        nvic: &mut Nvic,
        number: u8,
        return_pc: Option<u32>,
    ) {
        let from_thread = nvic.current_exception == 0;
        // 栈选择（FRT-EXC-01③）：线程模式按 SPSEL（引擎约定 control bit0），Handler 恒 MSP。
        // 直接用 msp/psp 状态（SP 修改指令均经 sync_sp 同步，比 regs[13] 别名更稳——
        // MSR CONTROL 切换 SPSEL 后 regs[13] 别名可能滞后于 msp/psp）
        let using_psp = from_thread && (cpu.control & 1) != 0;
        let mut sp = if using_psp { cpu.psp } else { cpu.msp };
        // STKALIGN（FRT-EXC-04）：SP 未 8 字节对齐 → 先减 4 并置 xPSR bit9（SPREALIGN）
        let mut sprealign = false;
        let stkalign = mem
            .system_block_mut()
            .map(|sb| sb.ccr_stkalign())
            .unwrap_or(true);
        if stkalign && (sp & 4) != 0 {
            sp = sp.wrapping_sub(4);
            sprealign = true;
        }
        // FPU 扩展帧（FRT-EXC-09 SHOULD，eager 保存）：CONTROL.FPCA=1 时压 S0-S15+FPSCR
        let fpu_frame = (cpu.control & 4) != 0;
        if fpu_frame {
            sp = sp.wrapping_sub(0x48);
        }
        // 压 8 字基本帧（r0,r1,r2,r3,r12,lr,pc,xpsr；小端、低地址在前，FRT-EXC-01②）
        sp = sp.wrapping_sub(32);
        // PC 槽 = 被中断流下一指令地址（FRT-EXC-03）：同步异常（SVC）= 调用方传入的
        // 返回地址；异步异常 = 当前 PC（下一条待执行指令，中断在指令边界取走）
        let pc = return_pc.unwrap_or(cpu.regs[15]);
        let lr = cpu.regs[14]; // LR 槽 = 被中断 r14 寄存器值（FRT-EXC-03）
        let mut frame_xpsr = cpu.xpsr;
        // ARMv7-M B1.5.6：压栈 xPSR 的 IT 位（bits[15:10]）清 0——IT 状态不跨异常
        // 保存/恢复（异常返回后 IT 状态机恒为 0，P1-2 小马审查项）；SPREALIGN 照常置位
        frame_xpsr &= !(0x3F << 10);
        if sprealign {
            frame_xpsr |= 1 << 9; // SPREALIGN
        }
        let frame = [
            cpu.regs[0],
            cpu.regs[1],
            cpu.regs[2],
            cpu.regs[3],
            cpu.regs[12],
            lr,
            pc,
            frame_xpsr,
        ];
        for (i, v) in frame.iter().enumerate() {
            let _ = mem.write_u32(sp + (i as u32) * 4, *v);
        }
        // 扩展帧内容（S0-S15 + FPSCR + 4 字节保留，共 0x48）
        if fpu_frame {
            for i in 0..16u32 {
                let _ = mem.write_u32(sp + 32 + i * 4, cpu.fpu.read_s(i as usize));
            }
            let _ = mem.write_u32(sp + 32 + 64, cpu.fpu.fpscr);
        }
        // LR ← EXC_RETURN（FRT-EXC-01④）：线程+PSP→FD、线程+MSP→F9、Handler→F1；
        // FPU 上下文 → bit4=0 的对应变体（ED/E9/E1）
        let exc_return = match (fpu_frame, from_thread, using_psp) {
            (true, true, true) => 0xFFFF_FFED,
            (true, true, false) => 0xFFFF_FFE9,
            (true, false, _) => 0xFFFF_FFE1,
            (false, true, true) => 0xFFFF_FFFD,
            (false, true, false) => 0xFFFF_FFF9,
            (false, false, _) => 0xFFFF_FFF1,
        };
        cpu.regs[14] = exc_return;
        // 更新被中断栈指针 + 切 Handler 模式（SP 别名 MSP）
        if using_psp {
            cpu.psp = sp;
        } else {
            cpu.msp = sp;
        }
        cpu.regs[13] = cpu.msp;
        // IPSR ← 异常号（FRT-EXC-08）；ITSTATE 清零（FRT-EXC-01⑦）；FPCA 清（eager 已压栈）
        cpu.xpsr = (cpu.xpsr & !0x1FF) | (number as u32 & 0x1FF);
        self.executor.clear_it();
        if fpu_frame {
            cpu.control &= !4;
        }
        // 向量地址 = (VTOR + 4×异常号) 从内存读（FRT-EXC-01①/FRT-CHIP-03）
        let vt = mem
            .system_block_mut()
            .map(|sb| sb.vtor())
            .unwrap_or(0);
        let vec = mem.read_u32(vt.wrapping_add(4 * number as u32)).unwrap_or(0);
        cpu.regs[15] = vec & !1;
        // NVIC 记账（嵌套历史 + 外部 IRQ 清挂起/置活跃）
        if number < 16 {
            if let Some(sb) = mem.system_block_mut() {
                sb.unpend_exception(number);
            }
        }
        nvic.enter_exception(number);
        if let Some(sb) = mem.system_block_mut() {
            sb.set_vectactive(nvic.current_exception);
        }
    }

    /// 异常出口（FRT-EXC-02）：BX EXC_RETURN → 弹栈恢复 → 切线程模式 → SPSEL 更新
    fn return_from_exception(
        &mut self,
        cpu: &mut CpuState,
        mem: &mut Memory,
        nvic: &mut Nvic,
        exc_return: u32,
    ) {
        let uses_psp = exc_return & 4 != 0;
        // FPU 变体（bit4=0：E1/E9/ED）→ 弹扩展帧恢复 S0-S15+FPSCR
        let fpu_frame = exc_return & 0x10 == 0;
        let mut sp = if uses_psp { cpu.psp } else { cpu.msp };
        // 弹 8 字基本帧
        let mut vals = [0u32; 8];
        for (i, v) in vals.iter_mut().enumerate() {
            *v = mem.read_u32(sp + (i as u32) * 4).unwrap_or(0);
        }
        sp = sp.wrapping_add(32);
        if fpu_frame {
            for i in 0..16u32 {
                cpu.fpu.write_s(i as usize, mem.read_u32(sp + i * 4).unwrap_or(0));
            }
            cpu.fpu.fpscr = mem.read_u32(sp + 64).unwrap_or(0);
            sp = sp.wrapping_add(0x48);
        }
        // 恢复 r0-r3/r12/r14/PC/xPSR；SPREALIGN（xPSR bit9）→ SP+=4 并清位（FRT-EXC-04）
        cpu.regs[0] = vals[0];
        cpu.regs[1] = vals[1];
        cpu.regs[2] = vals[2];
        cpu.regs[3] = vals[3];
        cpu.regs[12] = vals[4];
        cpu.regs[14] = vals[5];
        let mut xpsr = vals[7];
        if xpsr & (1 << 9) != 0 {
            sp = sp.wrapping_add(4);
            xpsr &= !(1 << 9);
        }
        cpu.regs[15] = vals[6] & !1; // PC 槽（清 T 位；T 位由 xPSR 槽恢复）
        // 切回线程模式：IPSR=0；CONTROL.SPSEL 按 EXC_RETURN bit2 更新（FRT-EXC-02）
        cpu.xpsr = xpsr & !0x1FF;
        if uses_psp {
            cpu.psp = sp;
            cpu.control |= 1;
        } else {
            cpu.msp = sp;
            cpu.control &= !1;
        }
        cpu.regs[13] = sp;
        nvic.exit_exception();
        if let Some(sb) = mem.system_block_mut() {
            sb.set_vectactive(nvic.current_exception);
        }
    }
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::Memory;

    #[test]
    fn engine_steps_through_code() {
        let mut cpu = CpuState::default();
        let mut mem = Memory::test_ram();
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();

        // 程序: MOV R0, #0x05 (0x2005) 然后 NOP (0xBF00)
        mem.flash[0] = 0x05;
        mem.flash[1] = 0x20;
        mem.flash[2] = 0x00;
        mem.flash[3] = 0xBF;
        cpu.regs[15] = 0;

        eng.step(&mut cpu, &mut mem, &mut nvic);
        assert_eq!(cpu.regs[0], 5);
        assert_eq!(cpu.regs[15], 2);
        assert_eq!(eng.stats.instructions, 1);
    }

    /// 取指→解码→执行 全链路：SSAT 饱和运算 + VADD.F32 浮点加法
    #[test]
    fn engine_full_pipeline_dsp_and_fpu() {
        // GIVEN: 内存中依次放置
        //   SSAT R0, #8, R1（0xF301 0007，R1 = 200 → 饱和 127）
        //   VADD.F32 S0, S1, S2（0xEE30 0A81，S1=1.0, S2=2.0 → 3.0）
        let mut cpu = CpuState::default();
        let mut mem = Memory::test_ram();
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();
        cpu.regs[15] = 0;
        cpu.regs[1] = 200;
        cpu.fpu.write_s(1, 1.0f32.to_bits());
        cpu.fpu.write_s(2, 2.0f32.to_bits());
        // SSAT 编码 0xF301_0007：半字 0xF301 在前（低位地址），小端字节序
        mem.flash[0] = 0x01;
        mem.flash[1] = 0xF3;
        mem.flash[2] = 0x07;
        mem.flash[3] = 0x00;
        // VADD.F32 编码 0xEE30_0A81
        mem.flash[4] = 0x30;
        mem.flash[5] = 0xEE;
        mem.flash[6] = 0x81;
        mem.flash[7] = 0x0A;

        // WHEN: 连续单步执行两条指令
        assert_eq!(
            eng.step(&mut cpu, &mut mem, &mut nvic),
            EngineResult::Halted
        );
        assert_eq!(
            eng.step(&mut cpu, &mut mem, &mut nvic),
            EngineResult::Halted
        );

        // THEN: R0 = 127（SSAT 饱和），Q 置位；S0 = 3.0（VADD）
        assert_eq!(cpu.regs[0], 127);
        assert_ne!(cpu.xpsr & (1 << 27), 0);
        assert_eq!(cpu.fpu.read_s(0), 3.0f32.to_bits());
        assert_eq!(cpu.regs[15], 8);
        assert_eq!(eng.stats.instructions, 2);
    }

    /// 16-bit LDR/STR 全链路：烧 flash 字节序验证（小端，16 位指令正常小端字节序）
    #[test]
    fn engine_flash_16bit_ldr_str() {
        // GIVEN: 内存中依次放置（小端字节序）
        //   MOVS R0, #0x2A（0x202A）→ 字节 2A 20
        //   STR R0, [R1, #4]（0x6048）→ 字节 48 60（R1 = 0x2000_0000）
        //   LDR R2, [R1, #4]（0x684A）→ 字节 4A 68
        //   LDRB R3, [R1, #4]（0x790B）→ 字节 0B 79
        let mut cpu = CpuState::default();
        let mut mem = Memory::test_ram();
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();
        cpu.regs[15] = 0;
        cpu.regs[1] = 0x2000_0000;
        mem.flash[0] = 0x2A;
        mem.flash[1] = 0x20;
        mem.flash[2] = 0x48;
        mem.flash[3] = 0x60;
        mem.flash[4] = 0x4A;
        mem.flash[5] = 0x68;
        mem.flash[6] = 0x0B;
        mem.flash[7] = 0x79;

        // WHEN: 连续执行 4 条指令
        for _ in 0..4 {
            assert_eq!(
                eng.step(&mut cpu, &mut mem, &mut nvic),
                EngineResult::Halted
            );
        }

        // THEN: R0 = 0x2A；[0x2000_0004] = 0x2A；R2 = 0x2A；R3 = 0x2A
        assert_eq!(cpu.regs[0], 0x2A);
        assert_eq!(mem.read_u32(0x2000_0004).unwrap(), 0x2A);
        assert_eq!(cpu.regs[2], 0x2A);
        assert_eq!(cpu.regs[3], 0x2A);
        assert_eq!(cpu.regs[15], 8);
    }

    // ================= E2: IT 块 / BKPT 引擎级 golden 测试 =================
    // 编码与 arm-none-eabi-as 实测一致（it eq=0xBF08、ite ne=0xBF14…）。

    /// IT 块：条件不成立 → 跳过（movs r0,#1 置 Z=0；it eq 后续 moveq 被跳过；
    /// ite ne 前半执行、后半（条件翻转 EQ）被跳过）
    #[test]
    fn e2_it_block_condition_skip() {
        let mut cpu = CpuState::default();
        let mut mem = Memory::test_ram();
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();
        cpu.regs[15] = 0;
        // 0x2001 movs r0,#1 | 0xBF08 it eq | 0x2101 moveq r1,#1 | 0x2102 movs r1,#2
        // 0xBF14 ite ne | 0x2201 movne r2,#1 | 0x2302 moveq r3,#2
        for (i, b) in [
            0x01, 0x20, 0x08, 0xBF, 0x01, 0x21, 0x02, 0x21, 0x14, 0xBF, 0x01, 0x22, 0x02, 0x23,
        ]
        .iter()
        .enumerate()
        {
            mem.flash[i] = *b;
        }
        // WHEN: 连续单步 7 条指令
        for _ in 0..7 {
            assert_eq!(eng.step(&mut cpu, &mut mem, &mut nvic), EngineResult::Halted);
        }
        // THEN: r0=1（Z=0）、r1=2（moveq 被跳过）、r2=1（NE 成立）、r3=0（EQ 被跳过）
        assert_eq!(cpu.regs[0], 1);
        assert_eq!(cpu.regs[1], 2, "it eq 后 moveq 应被跳过");
        assert_eq!(cpu.regs[2], 1, "ite ne 前半 movne 应执行");
        assert_eq!(cpu.regs[3], 0, "ite ne 后半 moveq（翻转条件）应被跳过");
        assert_eq!(cpu.regs[15], 14);
        assert_eq!(eng.stats.instructions, 7);
        assert_eq!(eng.stats.faults, 0);
        assert!(!eng.executor.it_active(), "IT 块结束后状态应清空");
    }

    /// IT 块：条件成立 → 全部执行（itttt eq：4 条 STR 全部执行；用 STR 避免 ADDS
    /// 覆盖 Z 标志——真实硬件行为：块内 ADDS 会改写 flags 影响后续条件）
    #[test]
    fn e2_it_block_all_execute() {
        let mut cpu = CpuState::default();
        let mut mem = Memory::test_ram();
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();
        cpu.regs[15] = 0;
        cpu.regs[0] = 0x2000_0000;
        cpu.regs[1] = 0xA1;
        cpu.regs[2] = 0xB2;
        cpu.regs[3] = 0xC3;
        cpu.regs[4] = 0xD4;
        // 0x2500 movs r5, #0（置 Z=1，不碰 STR 源寄存器）| 0xBF01 itttt eq |
        // 4×16 位 STR（不写 flags）：0x6001/0x6042/0x6083/0x60C4 → [r0+#0/#4/#8/#12]
        for (i, b) in [
            0x00, 0x25, 0x01, 0xBF, 0x01, 0x60, 0x42, 0x60, 0x83, 0x60, 0xC4, 0x60,
        ]
        .iter()
        .enumerate()
        {
            mem.flash[i] = *b;
        }
        for _ in 0..6 {
            assert_eq!(eng.step(&mut cpu, &mut mem, &mut nvic), EngineResult::Halted);
        }
        // THEN: 4 条 STR 全部执行（Z=1 保持，EQ 恒成立）
        assert_eq!(mem.read_u32(0x2000_0000).unwrap(), 0xA1);
        assert_eq!(mem.read_u32(0x2000_0004).unwrap(), 0xB2);
        assert_eq!(mem.read_u32(0x2000_0008).unwrap(), 0xC3);
        assert_eq!(mem.read_u32(0x2000_000C).unwrap(), 0xD4);
        assert_eq!(cpu.regs[15], 12);
        assert!(!eng.executor.it_active());
    }

    /// ITE EQ（0xBF0C，mask 1100）：[Eq, Ne]——NE 分支在 Z=0 时执行。
    /// 验证绝对模型（mask 位 = 后续指令条件 bit0）：gas 编码 ite eq = 1100（instr1 = Ne）；
    /// 注：QEMU 11.0.2 二进制对该编码表现为 [Eq,Eq]，与其自身源码及 gas 编码语义相悖
    /// （已实测 ite ne/itee ne/itte ne/ittte ne/iteee eq/itttt eq 全部一致，仅此一例异常）；
    /// dtwin 按架构语义（gas 编码权威）实现。
    ///
    /// P4 语义修正（ARMv7-M B1.5.10）：IT 块内 16 位隐式 S 指令（此处 moveq 即
    /// 16 位 MOVS）不更新条件标志 → movs r5,#0 置的 Z=1 在块内保持，movne 因
    /// NE 需 Z=0 而被跳过（旧实现 moveq 误写 Z=0 使 movne 错误执行，现修正）。
    #[test]
    fn e2_ite_eq_else_executes() {
        let mut cpu = CpuState::default();
        let mut mem = Memory::test_ram();
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();
        cpu.regs[15] = 0;
        // 0x2500 movs r5,#0（Z=1）| 0xBF0C ite eq | 0x21A1 moveq r1,#0xA1 | 0x22B2 movne r2,#0xB2
        for (i, b) in [0x00, 0x25, 0x0C, 0xBF, 0xA1, 0x21, 0xB2, 0x22].iter().enumerate() {
            mem.flash[i] = *b;
        }
        for _ in 0..4 {
            assert_eq!(eng.step(&mut cpu, &mut mem, &mut nvic), EngineResult::Halted);
        }
        assert_eq!(cpu.regs[1], 0xA1, "instr0 moveq 执行（Z=1，EQ 成立）");
        // B1.5.10：moveq 不更新标志，Z=1 保持 → instr1 movne（NE 需 Z=0）被跳过
        assert_eq!(cpu.regs[2], 0, "instr1 movne 跳过（16 位 MOVS 块内不更新标志，Z=1 保持）");
        assert!(!eng.executor.it_active());
    }

    /// BKPT：触发 DebugEvent，引擎统计 exceptions，run 停止
    #[test]
    fn e2_bkpt_triggers_debug_event() {
        let mut cpu = CpuState::default();
        let mut mem = Memory::test_ram();
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();
        cpu.regs[15] = 0;
        // 0x2001 movs r0,#1 | 0xBEAB bkpt #0xAB
        mem.flash[0] = 0x01;
        mem.flash[1] = 0x20;
        mem.flash[2] = 0xAB;
        mem.flash[3] = 0xBE;
        assert_eq!(eng.step(&mut cpu, &mut mem, &mut nvic), EngineResult::Halted);
        assert_eq!(
            eng.step(&mut cpu, &mut mem, &mut nvic),
            EngineResult::DebugEvent
        );
        assert_eq!(eng.stats.exceptions, 1);
        assert_eq!(eng.stats.faults, 0);
        // run 遇到 BKPT 也返回 DebugEvent
        let mut eng2 = Engine::new();
        let mut cpu2 = CpuState::default();
        let mut mem2 = Memory::test_ram();
        let mut nvic2 = Nvic::new();
        cpu2.regs[15] = 0;
        mem2.flash[0] = 0xAB;
        mem2.flash[1] = 0xBE;
        assert_eq!(
            eng2.run(&mut cpu2, &mut mem2, &mut nvic2),
            EngineResult::DebugEvent
        );
    }

    /// BKPT 在 IT 块内：条件不成立时被跳过（诚实边界：ARMv7-M 规定 BKPT 不受
    /// 条件限制始终执行，此处按 IT 门控处理并如实注释）
    #[test]
    fn e2_bkpt_inside_it_cond_fail_skips() {
        let mut cpu = CpuState::default();
        let mut mem = Memory::test_ram();
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();
        cpu.regs[15] = 0;
        // 0x2001 movs r0,#1（Z=0）| 0xBF08 it eq | 0xBEAB bkpt（应被跳过）
        for (i, b) in [0x01, 0x20, 0x08, 0xBF, 0xAB, 0xBE].iter().enumerate() {
            mem.flash[i] = *b;
        }
        for _ in 0..3 {
            assert_eq!(eng.step(&mut cpu, &mut mem, &mut nvic), EngineResult::Halted);
        }
        assert_eq!(eng.stats.exceptions, 0, "条件不成立的 BKPT 被跳过");
        assert_eq!(cpu.regs[15], 6);
    }

    /// B1.5.10 扩展验证（P5 补）：IT 块内 16 位隐式 S 的 ADCS 同样被抑制
    /// （WIP 修复：is_implicit_s_16bit 补 Adc/Sbc）；32 位显式 S（ADDS.W）
    /// 不受抑制，仍正常更新标志。
    ///
    /// 序列：movs r5,#0（Z=1）| it eq | adcs r0,r1（0x7FFFFFFF+1 → 0x80000000，
    /// 若更新标志则 N=1/V=1/Z=0，被抑制 → Z=1 保持）| it eq | adds.w r0,r1,r2
    /// （32 位，正常更新 → N=1/V=1/Z=0）。
    #[test]
    fn e2_it_block_adcs_suppressed_32bit_s_updates() {
        let mut cpu = CpuState::default();
        let mut mem = Memory::test_ram();
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();
        cpu.regs[15] = 0;
        cpu.regs[0] = 0x7FFF_FFFF; // adcs 输入（rd=rn=0）
        cpu.regs[1] = 1; // adcs 加数
        cpu.regs[2] = 0x7FFF_FFFF; // adds.w 输入
        cpu.regs[3] = 1; // adds.w 加数
        // 0x2500 movs r5,#0（Z=1）| 0xBF08 it eq | 0x4148 adcs r0,r1 |
        // 0xBF08 it eq | 0xEB12 0003 adds.w r0,r2,r3（小端 12 EB 03 00）
        for (i, b) in [
            0x00u8, 0x25, 0x08, 0xBF, 0x48, 0x41, 0x08, 0xBF, 0x12, 0xEB, 0x03, 0x00,
        ]
        .iter()
        .enumerate()
        {
            mem.flash[i] = *b;
        }
        // WHEN: 前 3 步（movs + it + adcs）后检查抑制
        for _ in 0..3 {
            assert_eq!(eng.step(&mut cpu, &mut mem, &mut nvic), EngineResult::Halted);
        }
        // THEN：adcs 结果写入但标志被抑制（Z=1 保持、N 未置位）
        assert_eq!(cpu.regs[0], 0x8000_0000, "ADCS 结果仍写入");
        assert_ne!(cpu.xpsr & (1 << 30), 0, "Z=1 保持（16 位隐式 S 被抑制）");
        assert_eq!(cpu.xpsr & (1 << 31), 0, "N 未被 ADCS 置位");
        // WHEN: 再 2 步（it + adds.w）
        for _ in 0..2 {
            assert_eq!(eng.step(&mut cpu, &mut mem, &mut nvic), EngineResult::Halted);
        }
        // THEN：32 位 ADDS.W（显式 S）不受抑制 → N=1 V=1 Z=0
        assert_ne!(cpu.xpsr & (1 << 31), 0, "ADDS.W 正常更新 N");
        assert_ne!(cpu.xpsr & (1 << 28), 0, "ADDS.W 正常更新 V");
        assert_eq!(cpu.xpsr & (1 << 30), 0, "ADDS.W 清除 Z");
        assert_eq!(cpu.regs[15], 12);
        assert_eq!(eng.stats.faults, 0);
        assert!(!eng.executor.it_active(), "IT 块结束后状态应清空");
    }

    /// P1-2（小马审查）：ITSTATE 不跨异常——IT 块内触发 SVC，异常入口清 IT 状态机，
    /// 压栈 xPSR 的 IT 位（bits[15:10]）清 0（ARMv7-M B1.5.6），异常返回后 IT 状态机
    /// 恒为 0（不恢复）。
    #[test]
    fn e2_it_state_cleared_across_exception() {
        use crate::system::SystemBlock;
        let mut cpu = CpuState::default();
        let mut mem = Memory::m4f_default();
        mem.attach_peripheral(SystemBlock::new());
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();
        // 复位语义：SP = 向量表[0]（0x20002000），与 Loader/exception_mech 一致
        cpu.msp = 0x2000_2000;
        cpu.regs[13] = cpu.msp;
        cpu.regs[15] = 0;
        // 向量 11（SVC）→ 0x40（handler：bx lr = 0x4770）
        let vec11 = 0x40u32;
        mem.flash[0x2C..0x30].copy_from_slice(&vec11.to_le_bytes());
        mem.flash[0x40] = 0x70;
        mem.flash[0x41] = 0x47;
        // 0x2000 movs r0,#0（Z=1）| 0xBF08 it eq | 0xDF00 svc #0 | 0xBF00 nop
        for (i, b) in [0x00u8, 0x20, 0x08, 0xBF, 0x00, 0xDF, 0x00, 0xBF]
            .iter()
            .enumerate()
        {
            mem.flash[i] = *b;
        }
        // WHEN: movs + it 两步后，IT 块处于活跃（条件 EQ 成立）
        for _ in 0..2 {
            assert_eq!(eng.step(&mut cpu, &mut mem, &mut nvic), EngineResult::Halted);
        }
        assert!(eng.executor.it_active(), "SVC 前 IT 块应处于活跃");
        // WHEN: svc 触发异常（入口清 IT + 压栈 xPSR 清 IT 位）→ handler bx lr 返回
        for _ in 0..2 {
            assert_eq!(eng.step(&mut cpu, &mut mem, &mut nvic), EngineResult::Halted);
        }
        // THEN: IT 状态机关闭、xPSR IT 位为 0、返回地址正确（svc 后一条）
        assert!(!eng.executor.it_active(), "异常返回后 IT 状态机应为 0");
        assert_eq!(cpu.xpsr & (0x3F << 10), 0, "xPSR IT 位（bits[15:10]）应为 0");
        assert_eq!(cpu.regs[15], 0x06, "返回 svc 下一条");
        assert_eq!(eng.stats.exceptions, 2, "入口+出口各计 1");
        assert_eq!(eng.stats.faults, 0);
        // 对照：IT 位原本确实可能非 0（若无抑制则 SVC 前 IT 块活跃）——机器状态已清即可
        assert_eq!(nvic.current_exception, 0, "返回线程模式");
    }

    /// P2/P3：SysTick 周期驱动接入 run 循环（FRT-SYS-02/FRT-CHIP-02 + FRT-EXC-01）
    /// 固件（as 实测编码）使能 SysTick（LOAD=100, TICKINT=1）后空转；
    /// run 循环每指令 tick_system(1) → 递减至 0 → 仲裁取异常 15 → 入口跳向量。
    /// 向量表：异常 15 → 0x20 的 handler（bx lr 立即返回）。
    #[test]
    fn e2_systick_tick_pends_exception15() {
        use crate::system::SystemBlock;
        let mut cpu = CpuState::default();
        // m4f_default 含 SYSTEM 区（0xE0000000-0xE0100000）
        let mut mem = Memory::m4f_default();
        mem.attach_peripheral(SystemBlock::new());
        // 固件：ldr r1,[pc,#16]=0x4904 | movs r0,#7=0x2007 | str r0,[r1]=0x6008 |
        // movs r0,#100=0x2064 | str r0,[r1,#4]=0x6048 | movs r0,#0=0x2000 |
        // str r0,[r1,#8]=0x6088 | nop=0xBF00 | b loop=0xE7FD | 对齐填充 0x12-0x13 |
        // literal 0xE000E010 @0x14（LDR 字面量地址 = Align(PC+4,4)+16 = 0x14）
        for (i, b) in [
            0x04u8, 0x49, 0x07, 0x20, 0x08, 0x60, 0x64, 0x20, 0x48, 0x60, 0x00, 0x20, 0x88,
            0x60, 0x00, 0xBF, 0xFD, 0xE7, 0x00, 0x00, 0x10, 0xE0, 0x00, 0xE0,
        ]
        .iter()
        .enumerate()
        {
            mem.flash[i] = *b;
        }
        // 向量表：异常 15（SysTick）→ 0x20（handler：bx lr = 0x4770）
        let vec15 = 0x21u32; // 0x20 | T 位
        mem.flash[60..64].copy_from_slice(&vec15.to_le_bytes());
        mem.flash[0x20] = 0x70;
        mem.flash[0x21] = 0x47;
        let mut nvic = Nvic::new();
        let mut eng = Engine::new();
        eng.max_instructions = 800; // 使能 + 空转（LOAD=100 → ~102 tick 触发）+ 多次异常往返
        let r = eng.run(&mut cpu, &mut mem, &mut nvic);
        assert_eq!(r, EngineResult::LimitReached);
        // THEN：SysTick 异常被仲裁取走并进入（入口/出口各计 1 次 exceptions）
        assert!(eng.stats.exceptions >= 4, "exceptions={}", eng.stats.exceptions);
        assert_eq!(eng.stats.faults, 0);
        assert_eq!(nvic.current_exception, 0, "handler bx lr 返回线程模式");
        // SystemBlock 无残留挂起（被入口消费）
        let sb = mem.system_block_mut().expect("SystemBlock 已挂接");
        assert_eq!(sb.next_pending_exception(), None);
    }

    /// P2：SYSTEM 区访问经 Memory 路由到 SystemBlock（FRT-CHIP-02）
    #[test]
    fn e2_system_region_routes_to_system_block() {
        use crate::system::SystemBlock;
        let mut mem = Memory::m4f_default();
        mem.attach_peripheral(SystemBlock::new());
        // 写 SysTick LOAD（0xE000E014）→ 读回
        mem.write_u32(0xE000_E014, 25000).unwrap();
        assert_eq!(mem.read_u32(0xE000_E014).unwrap(), 25000);
        // CPUID 读回 Cortex-M4 r0p1
        assert_eq!(mem.read_u32(0xE000_ED00).unwrap(), 0x410F_C241);
        // ICSR PENDSVSET 经内存写 → SystemBlock 挂起 14
        mem.write_u32(0xE000_ED04, 1 << 28).unwrap();
        let sb = mem.system_block_mut().unwrap();
        assert_eq!(sb.next_pending_exception(), Some(14));
        // 未挂接设备的 SYSTEM 区地址仍按默认行为（读 0 写忽略）
        let mut mem2 = Memory::m4f_default();
        mem2.write_u32(0xE000_E018, 0x1234).unwrap();
        assert_eq!(mem2.read_u32(0xE000_E018).unwrap(), 0);
    }
}
