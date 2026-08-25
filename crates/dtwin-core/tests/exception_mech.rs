//! P3 异常机制集成测试（FRT-EXC-01~10）
//!
//! 固件：fixtures/p3_scenarios.bin（源码 fixtures/p3_scenarios.s，
//! 重建命令见该文件头部注释；符号地址见下）
//! 向量表 @0x0：[0]=MSP 0x20002000 [11]=common_svc_handler(0x42)
//! [14]=common_pendsv_handler(0x6E) [15]=common_systick_handler(0x94)
//! 符号（nm 实测）：scn_a_start=0x4A scn_a_ret=0x50 scn_b_start=0x54
//! task1_body=0x76 task2_body=0x7A scn_c_start=0x7E scn_c_systick_handler=0x8E
//! scn_d_start=0x96 scn_e_start=0xAC scn_e_pendsv_handler=0xB6 scn_f_start=0xC2
//! scn_g_start=0xC8 scn_g_svc_handler=0xD2
//!
//! 说明：引擎 CONTROL 约定 bit0=SPSEL（既有测试语义，非 ARM 官方位）、bit2=FPCA。

use dtwin_core::engine::{Engine, EngineResult};
use dtwin_core::memory::Memory;
use dtwin_core::nvic::Nvic;
use dtwin_core::system::SystemBlock;
use dtwin_core::CpuState;

const SCN_A_START: u32 = 0x4A;
const SCN_A_RET: u32 = 0x50;
const SCN_B_START: u32 = 0x54;
const TASK2_BODY: u32 = 0x7A;
const SCN_C_START: u32 = 0x7E;
const SCN_C_SYSTICK_HANDLER: u32 = 0x8E;
const SCN_D_START: u32 = 0x96;
const SCN_E_START: u32 = 0xBC;
const SCN_E_PENDSV_HANDLER: u32 = 0xC6;
const SCN_F_START: u32 = 0xD2;
const SCN_G_START: u32 = 0xD8;
const SCN_G_SVC_HANDLER: u32 = 0xE2;

/// 构造标准测试环境：m4f_default 内存 + SystemBlock + 场景固件
fn setup() -> (Engine, CpuState, Memory, Nvic) {
    let mut cpu = CpuState::default();
    let mut mem = Memory::m4f_default();
    mem.attach_peripheral(SystemBlock::new());
    let bin = include_bytes!("fixtures/p3_scenarios.bin");
    mem.flash[..bin.len()].copy_from_slice(bin);
    // 线程模式默认 MSP；场景 B 单独设 PSP
    cpu.msp = 0x2000_2000;
    cpu.psp = 0x2000_1000;
    cpu.regs[13] = cpu.msp;
    (Engine::new(), cpu, mem, Nvic::new())
}

/// FRT-EXC-01/02/07/08：线程 SVC 入口/出口 + MRS IPSR 维护
#[test]
fn svc_entry_exit_restores_context() {
    let (mut eng, mut cpu, mut mem, mut nvic) = setup();
    cpu.regs[15] = SCN_A_START;
    eng.max_instructions = 20;
    let r = eng.run(&mut cpu, &mut mem, &mut nvic);
    assert_eq!(r, EngineResult::LimitReached);
    // 现场恢复：r0/r1/r2/r3 均为 SVC 前原值（handler 内 r2=11/r3=0x33 被出口弹栈丢弃）
    assert_eq!(cpu.regs[0], 0x11, "r0 现场恢复");
    assert_eq!(cpu.regs[1], 0x22, "r1 现场恢复");
    assert_eq!(cpu.regs[2], 0, "r2 恢复为 SVC 前原值");
    assert_eq!(cpu.regs[3], 0, "r3 恢复为 SVC 前原值");
    assert_eq!(cpu.regs[4], 0x44, "返回后继续执行（PC 槽 = 下一条指令）");
    assert!(
        cpu.regs[15] == SCN_A_RET || cpu.regs[15] == SCN_A_RET + 2,
        "PC 停在返回点循环（交替 0x50/0x52）"
    );
    assert_eq!(nvic.current_exception, 0, "出口切回线程模式");
    assert_eq!(cpu.xpsr & 0x1FF, 0, "线程模式 IPSR=0");
    assert_eq!(eng.stats.faults, 0);
    // handler 确实执行过：异常事件记录（入口+出口 = exceptions≥2）
    assert!(eng.stats.exceptions >= 2, "exceptions={}", eng.stats.exceptions);
    assert!(nvic.events.iter().any(|e| e.irq == 0), "SVC 异常事件已记录");
    // 压栈帧内容（8 字，MSP 0x20002000-32 = 0x20001FE0；r0@0 r1@4 r2@8 r3@12 r12@16 lr@20 pc@24 xpsr@28）
    assert_eq!(mem.read_u32(0x2000_1FE0).unwrap(), 0x11, "帧 r0");
    assert_eq!(mem.read_u32(0x2000_1FE4).unwrap(), 0x22, "帧 r1");
    assert_eq!(mem.read_u32(0x2000_1FF8).unwrap(), SCN_A_RET, "帧 pc = 下一条指令");
    assert_eq!(cpu.msp, 0x2000_2000, "SP 出口后恢复");
}

/// FRT-EXC-02/05：PendSV + PSP 上下文切换（FreeRTOS 风格：入口存线程到 PSP，
/// handler 换 PSP 到 task2，出口弹 task2 现场执行）
#[test]
fn pendsv_context_switch_via_psp() {
    let (mut eng, mut cpu, mut mem, mut nvic) = setup();
    cpu.regs[15] = SCN_B_START;
    cpu.psp = 0x2000_1000; // task1 栈顶（入口将线程现场压入）
    // task2 现场帧（预置）：r0=0x22 r1=0x33 r12/lr 占位 pc=task2_body xpsr T=1
    let frame: [u32; 8] = [0x22, 0x33, 0, 0, 0, 0x2000_0001, TASK2_BODY | 1, 0x0100_0000];
    for (i, v) in frame.iter().enumerate() {
        mem.write_u32(0x2000_0800 + (i as u32) * 4, *v).unwrap();
    }
    eng.max_instructions = 40;
    let r = eng.run(&mut cpu, &mut mem, &mut nvic);
    assert_eq!(r, EngineResult::LimitReached);
    assert_eq!(eng.stats.faults, 0);
    // task2 恢复执行：r0/r1 来自其现场帧；task2_body 设 r5=0x77
    assert_eq!(cpu.regs[0], 0x22, "task2 r0 恢复");
    assert_eq!(cpu.regs[1], 0x33, "task2 r1 恢复");
    assert_eq!(cpu.regs[5], 0x77, "task2 主体已执行");
    assert_eq!(nvic.current_exception, 0, "PendSV 返回线程模式");
    assert_eq!(cpu.control & 1, 1, "EXC_RETURN FD → SPSEL=1（线程+PSP）");
    assert_eq!(cpu.psp, 0x2000_0820, "task2 栈消费 8 字后回写");
}

/// FRT-EXC-05：嵌套（SysTick(255) handler 内 SVC(0) → 嵌套压栈 MSP）
#[test]
fn nested_exception_svc_in_systick() {
    let (mut eng, mut cpu, mut mem, mut nvic) = setup();
    cpu.regs[15] = SCN_C_START;
    // SHPR3：SysTick=255, PendSV=255（port 默认）；SHPR2 默认 0 → SVC 优先级 0
    mem.write_u32(0xE000_ED20, 0xFFFF_0000).unwrap();
    // 向量 15 → scn_c_systick_handler（内嵌 SVC 触发嵌套）
    mem.flash[60..64].copy_from_slice(&((SCN_C_SYSTICK_HANDLER | 1) as u32).to_le_bytes());
    eng.max_instructions = 600;
    let r = eng.run(&mut cpu, &mut mem, &mut nvic);
    assert_eq!(r, EngineResult::LimitReached);
    assert_eq!(eng.stats.faults, 0);
    assert_eq!(cpu.regs[7], 0xC7, "SysTick handler 在嵌套 SVC 返回后继续执行");
    assert_eq!(nvic.current_exception, 0, "嵌套返回后回线程");
    // 嵌套证据：SVC 事件嵌套深度 = 2（SysTick 深度 1 → SVC 深度 2）
    assert!(
        nvic.events.iter().any(|e| e.nesting_level == 2),
        "存在嵌套深度 2 的事件（嵌套 SVC 入口）"
    );
    assert!(nvic.events.iter().any(|e| e.nesting_level == 1), "SysTick 事件深度 1");
}

/// FRT-EXC-06：BASEPRI 屏蔽 SysTick（临界区语义）；解除后正常进入
#[test]
fn basepri_masks_systick_until_cleared() {
    let (mut eng, mut cpu, mut mem, mut nvic) = setup();
    cpu.regs[15] = SCN_D_START;
    eng.max_instructions = 500;
    // 阶段 1：BASEPRI=5，SysTick(默认 pri 0) —— 注意：SHPR3 未设置 → SysTick 优先级 0！
    // 0 < 5 → 不被 BASEPRI 屏蔽。为测屏蔽，先把 SysTick 优先级设为 255。
    mem.write_u32(0xE000_ED20, 0xFFFF_0000).unwrap();
    let r = eng.run(&mut cpu, &mut mem, &mut nvic);
    assert_eq!(r, EngineResult::LimitReached);
    // 屏蔽期间：异常未进入（exceptions==0），SysTick 挂起保持
    assert_eq!(eng.stats.exceptions, 0, "BASEPRI=5 屏蔽 SysTick(255)");
    assert_eq!(nvic.current_exception, 0);
    let pended = mem.system_block_mut().unwrap().pended_bits();
    assert_ne!(pended & (1 << 15), 0, "SysTick 保持挂起");
    // 阶段 2：测试置 SRAM 标志 → 固件 msr basepri,#0 解除屏蔽
    mem.write_u32(0x2000_0000, 1).unwrap();
    let r2 = eng.run(&mut cpu, &mut mem, &mut nvic);
    assert_eq!(r2, EngineResult::LimitReached);
    assert!(eng.stats.exceptions >= 1, "解除屏蔽后 SysTick 进入（exceptions={}）", eng.stats.exceptions);
    assert_eq!(eng.stats.faults, 0);
}

/// FRT-EXC-05：同优先级不抢占（PendSV(255) handler 内挂 SysTick(255) → 保持挂起）
#[test]
fn same_priority_no_preemption() {
    let (mut eng, mut cpu, mut mem, mut nvic) = setup();
    cpu.regs[15] = SCN_E_START;
    mem.write_u32(0xE000_ED20, 0xFFFF_0000).unwrap();
    // 向量 14 → scn_e_pendsv_handler（内挂 SysTick）
    mem.flash[56..60].copy_from_slice(&((SCN_E_PENDSV_HANDLER | 1) as u32).to_le_bytes());
    eng.max_instructions = 300;
    let r = eng.run(&mut cpu, &mut mem, &mut nvic);
    assert_eq!(r, EngineResult::LimitReached);
    assert_eq!(eng.stats.faults, 0);
    assert_eq!(cpu.regs[6], 0xE6, "PendSV handler 已执行（含 PENDSTSET）");
    // 关键断言：SysTick 事件嵌套深度 = 1（从线程进入），非 2（未抢占 PendSV）
    let systick_ev = nvic
        .events
        .iter()
        .find(|e| e.nesting_level == 2);
    assert!(systick_ev.is_none(), "SysTick 不得嵌套进 PendSV（同优先级 255）");
    assert!(
        nvic.events.iter().any(|e| e.nesting_level == 1),
        "PendSV 与 SysTick 均从线程（深度 1）进入"
    );
    assert_eq!(nvic.current_exception, 0);
}

/// FRT-EXC-04：STKALIGN/SPREALIGN（SP 未 8 对齐 → 压栈前减 4 + xPSR bit9；出口恢复）
#[test]
fn stkalign_sprealign_roundtrip() {
    let (mut eng, mut cpu, mut mem, mut nvic) = setup();
    cpu.regs[15] = SCN_F_START;
    cpu.msp = 0x2000_2004; // 未 8 对齐
    cpu.regs[13] = cpu.msp;
    eng.max_instructions = 20;
    let r = eng.run(&mut cpu, &mut mem, &mut nvic);
    assert_eq!(r, EngineResult::LimitReached);
    assert_eq!(eng.stats.faults, 0);
    assert_eq!(cpu.regs[0], 0xFA, "r0 现场恢复");
    assert!(eng.stats.exceptions >= 2, "SVC 入口+出口（exceptions={}）", eng.stats.exceptions);
    assert_eq!(cpu.msp, 0x2000_2004, "SP 出口恢复原值（含 SPREALIGN 加回）");
    // 帧 xPSR 槽 bit9（SPREALIGN）置位；帧起始 8 字节对齐（帧 @0x20001FE0，xPSR @0x1FFC）
    let frame_xpsr = mem.read_u32(0x2000_1FFC).unwrap();
    assert_ne!(frame_xpsr & (1 << 9), 0, "帧 xPSR.SPREALIGN 置位");
    assert_eq!(cpu.xpsr & (1 << 9), 0, "恢复后 SPREALIGN 清除");
}

/// FRT-EXC-06：PRIMASK/FAULTMASK 屏蔽可配置异常（SysTick）；清除后进入
#[test]
fn primask_faultmask_mask_systick() {
    // PRIMASK=1 屏蔽
    let (mut eng, mut cpu, mut mem, mut nvic) = setup();
    cpu.regs[15] = SCN_C_START;
    mem.write_u32(0xE000_ED20, 0xFFFF_0000).unwrap();
    cpu.primask = 1; // 测试直接置 PRIMASK（等价 cpsid i 后状态）
    eng.max_instructions = 400;
    let r = eng.run(&mut cpu, &mut mem, &mut nvic);
    assert_eq!(r, EngineResult::LimitReached);
    assert_eq!(eng.stats.exceptions, 0, "PRIMASK=1 屏蔽 SysTick(255)");
    assert_eq!(nvic.current_exception, 0);
    let pended = mem.system_block_mut().unwrap().pended_bits();
    assert_ne!(pended & (1 << 15), 0, "SysTick 保持挂起");
    // 清除 PRIMASK → 进入
    cpu.primask = 0;
    let r2 = eng.run(&mut cpu, &mut mem, &mut nvic);
    assert_eq!(r2, EngineResult::LimitReached);
    assert!(eng.stats.exceptions >= 1, "PRIMASK 清除后 SysTick 进入");
    assert_eq!(eng.stats.faults, 0);

    // FAULTMASK=1 屏蔽（含 HardFault；可配置异常同样被屏蔽）
    let (mut eng2, mut cpu2, mut mem2, mut nvic2) = setup();
    cpu2.regs[15] = SCN_C_START;
    mem2.write_u32(0xE000_ED20, 0xFFFF_0000).unwrap();
    cpu2.faultmask = 1;
    eng2.max_instructions = 400;
    let r3 = eng2.run(&mut cpu2, &mut mem2, &mut nvic2);
    assert_eq!(r3, EngineResult::LimitReached);
    assert_eq!(eng2.stats.exceptions, 0, "FAULTMASK=1 屏蔽 SysTick(255)");
    cpu2.faultmask = 0;
    let r4 = eng2.run(&mut cpu2, &mut mem2, &mut nvic2);
    assert_eq!(r4, EngineResult::LimitReached);
    assert!(eng2.stats.exceptions >= 1, "FAULTMASK 清除后 SysTick 进入");
}

/// FRT-EXC-09（SHOULD）：FPU 扩展帧（FPCA=1 → 压 S0-S15+FPSCR，EXC_RETURN E9）
#[test]
fn fpu_extended_frame_roundtrip() {
    let (mut eng, mut cpu, mut mem, mut nvic) = setup();
    cpu.regs[15] = SCN_G_START;
    // 向量 11 → scn_g_svc_handler（读帧内 S0/FPSCR 校验）
    mem.flash[44..48].copy_from_slice(&((SCN_G_SVC_HANDLER | 1) as u32).to_le_bytes());
    cpu.fpu.write_s(0, 1.0f32.to_bits());
    cpu.fpu.write_s(1, 2.0f32.to_bits());
    eng.max_instructions = 30;
    let r = eng.run(&mut cpu, &mut mem, &mut nvic);
    assert_eq!(r, EngineResult::LimitReached);
    assert_eq!(eng.stats.faults, 0);
    assert!(eng.stats.exceptions >= 2, "SVC 入口+出口（exceptions={}）", eng.stats.exceptions);
    assert!(nvic.events.iter().any(|e| e.irq == 0), "SVC 事件已记录");
    // 出口恢复：S0 仍为 3.0（vadd 结果），S1 未破坏（FPU 扩展帧恢复）
    assert_eq!(cpu.fpu.read_s(0), 3.0f32.to_bits(), "S0 经扩展帧恢复");
    assert_eq!(cpu.fpu.read_s(1), 2.0f32.to_bits(), "S1 未被破坏");
    // handler 内读取的扩展帧内容仍在内存（未清理）：
    // msp=0x20002000（8 对齐）→ fpu_frame: sp=0x20001FB8；基本帧 0x20001F98..0x20001FB8；
    // 扩展帧 S0 @0x20001FB8，FPSCR @0x20001FB8+64
    assert_eq!(mem.read_u32(0x2000_1FB8).unwrap(), 0x4040_0000, "帧内 S0 = 3.0f32（vadd 结果）");
    assert_eq!(nvic.current_exception, 0);
}
