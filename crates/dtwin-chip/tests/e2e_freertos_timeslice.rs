//! FreeRTOS 时间片轮转变体 E2E 验证（FRT-AC-02）
//!
//! 固件 `freertos_timeslice.elf.dat`（scripts/build_freertos_timeslice.sh 构建）：
//! 仅 2 个同优先级（pri2）任务 TS_A/TS_B，忙循环打印 [TS]（无任何阻塞调用），
//! 每任务恰 40 行；轮次标志（g_turn）驱动严格交替 B0 A0 B1 A1 …，时间片
//! 旋转（configUSE_TIME_SLICING=1）为唯一抢占机制。
//!
//! 断言：
//!   1. dtwin 输出与 QEMU 黄金输出（scripts/run_qemu_golden_freertos_timeslice.sh
//!      产出）归一化逐行一致（前缀 0 差异）
//!   2. [TS] 两任务严格交替（B,A,B,A…，每任务恰 40 行）
//!   3. 时间片开关对照：noslice 固件（-DconfigUSE_TIME_SLICING=0，同源码仅此
//!      一处配置差异）输出恒为 [PASS] + [TS] B 0——第二任务永不运行，
//!      证明交替由时间片旋转触发而非其他机制
//!   4. 无 [FAIL] 行、引擎 faults=0

use dtwin_chip::memory_from_profile;
use dtwin_chip::S32K312;
use dtwin_core::engine::{Engine, EngineResult};
use dtwin_core::loader::Loader;
use dtwin_core::nvic::Nvic;
use dtwin_core::uart::CmsdkUart;
use dtwin_core::CpuState;

const FIRMWARE: &[u8] = include_bytes!("fixtures/freertos_timeslice.elf.dat");
const NOSLICE: &[u8] = include_bytes!("fixtures/freertos_timeslice_noslice.elf.dat");
const GOLDEN: &str = include_str!("fixtures/freertos_timeslice_golden_output.txt");

const UART_BASE: u32 = 0x4000_4000;
const CYCLES_PER_TICK: u64 = 25_000;
/// TS_ITERATIONS=40 × 2 任务 + PASS 行；+8 tick 余量
const MAX_INSTR: u64 = (40 * 2 + 8) * CYCLES_PER_TICK;

fn normalize(text: &str) -> Vec<String> {
    text.replace('\r', "")
        .lines()
        .map(|l| l.trim().to_string())
        .filter(|l| !l.is_empty() && !l.starts_with("qemu-system-arm:"))
        .collect()
}

/// 组装引擎并全速运行固件，返回 (输出行, 引擎统计)
fn run_firmware(fw: &[u8], max_instr: u64) -> (Vec<String>, dtwin_core::engine::EngineStats) {
    let profile = S32K312::new();
    let mut mem = memory_from_profile(&profile);
    mem.attach_peripheral(CmsdkUart::new(UART_BASE));
    let mut cpu = CpuState::default();
    let _ = Loader::load_elf_bytes(fw, &mut mem, &mut cpu).expect("加载固件");
    let mut nvic = Nvic::new();
    let mut engine = Engine::new();
    engine.max_instructions = max_instr;
    let result = engine.run(&mut cpu, &mut mem, &mut nvic);
    assert!(
        matches!(result, EngineResult::LimitReached),
        "应空转至指令上限：{result:?}"
    );
    let text = {
        let uart = mem
            .peripheral_mut_by_name("CMSDK-APB-UART")
            .expect("UART 已挂接")
            .downcast_mut::<CmsdkUart>()
            .expect("downcast 到 CmsdkUart");
        String::from_utf8_lossy(uart.output()).into_owned()
    };
    (normalize(&text), engine.stats)
}

#[test]
fn e2e_freertos_timeslice_alternates_and_matches_golden() {
    let (lines, stats) = run_firmware(FIRMWARE, MAX_INSTR);
    assert_eq!(stats.faults, 0, "执行不应产生故障");

    let golden_lines = normalize(GOLDEN);

    // ---- 断言 1：逐行一致（前缀 0 差异）----
    assert!(
        lines.len() >= golden_lines.len(),
        "dtwin {} 行 < 黄金 {} 行",
        lines.len(),
        golden_lines.len()
    );
    let diff_count = golden_lines
        .iter()
        .zip(lines.iter())
        .filter(|(g, d)| g != d)
        .count();
    assert_eq!(
        diff_count, 0,
        "前 {} 行存在差异（黄金 vs dtwin），首处：{:?}",
        golden_lines.len(),
        golden_lines.iter().zip(lines.iter()).find(|(g, d)| g != d)
    );

    // ---- 断言 2：严格交替 B,A,B,A…（每任务恰 40 行）----
    let ts: Vec<&str> = lines.iter().filter(|l| l.starts_with("[TS] ")).map(|l| l.as_str()).collect();
    assert_eq!(ts.len(), 80, "应恰 80 行 [TS]（2×40）");
    let a_count = ts.iter().filter(|l| l.starts_with("[TS] A")).count();
    let b_count = ts.iter().filter(|l| l.starts_with("[TS] B")).count();
    assert_eq!(a_count, 40, "[TS] A 行数");
    assert_eq!(b_count, 40, "[TS] B 行数");
    for (i, l) in ts.iter().enumerate() {
        let expected = if i % 2 == 0 { "[TS] B" } else { "[TS] A" };
        assert!(
            l.starts_with(expected),
            "第 {i} 行应 {expected}，实际 {l}"
        );
    }

    // ---- 断言 4：无失败行 ----
    assert!(
        !lines.iter().any(|l| l.starts_with("[FAIL]")),
        "输出含 [FAIL] 行（固件失败钩子触发）"
    );
    eprintln!(
        "== e2e_freertos_timeslice PASS: {} 行逐行一致（黄金 {} 行），严格 B,A 交替 ==",
        golden_lines.len(),
        golden_lines.len()
    );
}

#[test]
fn e2e_freertos_timeslice_noslice_control_no_alternation() {
    // 对照：configUSE_TIME_SLICING=0（同源码唯一配置差异）→ 轮次翻转后无人
    // 切换，第二任务永不运行 → 输出恒为 [PASS] + [TS] B 0（时间片触发证据）
    let (lines, stats) = run_firmware(NOSLICE, MAX_INSTR);
    assert_eq!(stats.faults, 0, "执行不应产生故障");
    assert_eq!(
        lines.len(),
        2,
        "noslice 对照应恰 2 行（[PASS] + [TS] B 0），实际 {} 行：{lines:?}",
        lines.len()
    );
    assert!(lines[0].contains("[PASS] freertos timeslice variant start"));
    assert_eq!(lines[1], "[TS] B 0");
    assert!(
        !lines.iter().any(|l| l.starts_with("[TS] A")),
        "noslice 对照不应出现 [TS] A 行"
    );
    eprintln!("== e2e_freertos_timeslice_noslice PASS: 无时间片时输出止于 [TS] B 0（第二任务永不运行）==");
}
