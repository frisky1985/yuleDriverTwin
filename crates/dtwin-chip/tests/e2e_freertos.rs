//! FreeRTOS 镜像 E2E 验证（FRT-FW-07 / FRT-AC-01~09）
//!
//! 加载 `freertos_demo.elf.dat`（FreeRTOS V11.1.0 ARM_CM4F port，多任务 +
//! SysTick/PendSV/SVC 调度，scripts/build_freertos_demo.sh 构建）到 S32K312
//! profile，挂接 CMSDK UART，Engine 全速执行；UART 输出与 QEMU 黄金输出
//! （qemu-system-arm -M mps2-an386 -cpu cortex-m4，scripts/run_qemu_golden_freertos.sh
//! 产出）逐行前缀一致。
//!
//! 序列可复现前提（FRT-FW-02）：任务每次唤醒打印一行 + vTaskDelay，输出序列
//! tick 计数驱动；QEMU mps2-an386 SysTick 由宿主时间驱动（每 tick 指令数不可
//! 复现），dtwin 为 1 指令=1 周期（25000 周期/tick）→ 以黄金 tick 数换算
//! max_instructions，取黄金行数为对比前缀。

use dtwin_chip::memory_from_profile;
use dtwin_chip::S32K312;
use dtwin_core::engine::{Engine, EngineResult};
use dtwin_core::loader::Loader;
use dtwin_core::nvic::Nvic;
use dtwin_core::uart::CmsdkUart;
use dtwin_core::CpuState;

/// 固件快照（.elf → .elf.dat 规避仓库 *.elf 忽略规则）
const FIRMWARE: &[u8] = include_bytes!("fixtures/freertos_demo.elf.dat");
/// QEMU 黄金输出（host-time SysTick，tick 数随运行时长变化；对比取前缀）
const GOLDEN: &str = include_str!("fixtures/freertos_golden_output.txt");

/// UART 兼容地址：CMSDK APB UART（QEMU MPS2 + dtwin 共用）
const UART_BASE: u32 = 0x4000_4000;
/// SysTick 周期：configSYSTICK_CLOCK_HZ=25MHz / configTICK_RATE_HZ=1000 = 25000 周期
const CYCLES_PER_TICK: u64 = 25_000;

/// 归一化：去 \r、去空行、去 QEMU 终止提示
fn normalize(text: &str) -> Vec<String> {
    text.replace('\r', "")
        .lines()
        .map(|l| l.trim().to_string())
        .filter(|l| !l.is_empty() && !l.starts_with("qemu-system-arm:"))
        .collect()
}

#[test]
fn e2e_freertos_matches_qemu_golden() {
    // ---- 组装：S32K312 profile 内存 + CMSDK UART + 加载固件 ----
    let profile = S32K312::new();
    let mut mem = memory_from_profile(&profile);
    mem.attach_peripheral(CmsdkUart::new(UART_BASE));
    let mut cpu = CpuState::default();
    let summary = Loader::load_elf_bytes(FIRMWARE, &mut mem, &mut cpu).expect("加载 FreeRTOS 固件");

    // 向量表一致性：SP/入口须与 ELF 向量表一致（0x0/0x4 处）
    let sp0 = mem.read_u32(0x0).expect("向量表 SP");
    let reset = mem.read_u32(0x4).expect("向量表 Reset");
    assert_eq!(summary.initial_sp, sp0, "初始 SP 应来自向量表[0]");
    assert_eq!(summary.entry_pc, reset & !1, "入口 PC 应来自向量表[1]");

    // ---- 由黄金输出推导 tick 数（TS A/B 每 tick 各打印一次，max seq+1）----
    let golden_lines = normalize(GOLDEN);
    let ts_max: u32 = golden_lines
        .iter()
        .filter(|l| l.starts_with("[TS] "))
        .filter_map(|l| l.rsplit(' ').next()?.parse().ok())
        .max()
        .expect("黄金输出含 [TS] 行");
    let golden_ticks = ts_max + 1;
    let max_instr = (golden_ticks as u64 + 4) * CYCLES_PER_TICK;

    // ---- 全速执行（调度器永不返回 → LimitReached）----
    let mut nvic = Nvic::new();
    let mut engine = Engine::new();
    engine.max_instructions = max_instr;
    let result = engine.run(&mut cpu, &mut mem, &mut nvic);
    assert!(
        matches!(result, EngineResult::LimitReached),
        "应空转至指令上限：{result:?}"
    );
    assert_eq!(engine.stats.faults, 0, "执行不应产生故障");
    assert!(
        engine.stats.exceptions > 100,
        "应发生大量异常（SysTick/上下文切换），实际 {}",
        engine.stats.exceptions
    );

    // ---- 收集 UART 输出 ----
    let text = {
        let uart = mem
            .peripheral_mut_by_name("CMSDK-APB-UART")
            .expect("UART 已挂接")
            .downcast_mut::<CmsdkUart>()
            .expect("downcast 到 CmsdkUart");
        String::from_utf8_lossy(uart.output()).into_owned()
    };
    let dtwin_lines = normalize(&text);

    // ---- 逐行前缀对比（黄金行数内 0 差异）----
    assert!(
        dtwin_lines.len() >= golden_lines.len(),
        "dtwin 输出 {} 行 < 黄金 {} 行（指令上限不足）",
        dtwin_lines.len(),
        golden_lines.len()
    );
    let diff_count = golden_lines
        .iter()
        .zip(dtwin_lines.iter())
        .filter(|(g, d)| g != d)
        .count();
    assert_eq!(
        diff_count, 0,
        "前 {} 行存在差异（黄金 vs dtwin），首处：{:?}",
        golden_lines.len(),
        golden_lines
            .iter()
            .zip(dtwin_lines.iter())
            .find(|(g, d)| g != d)
    );

    // ---- 核心检查行命中（FRT-FW-02 前缀全集）----
    let core_expected = [
        "[PASS] freertos demo start",
        "[SVC] 42",
        "[CRIT] n=1000",
        "[TASK] HIGH 0",
        "[TASK] MID 0",
        "[TASK] LOW 0",
        "[TS] A 0",
        "[TS] B 0",
    ];
    for line in core_expected {
        assert!(
            dtwin_lines.contains(&line.to_string()),
            "E2E 缺失核心检查行: {line}"
        );
    }
    assert!(
        !dtwin_lines.iter().any(|l| l.starts_with("[FAIL]")),
        "输出含 [FAIL] 行（固件失败钩子触发）"
    );
    eprintln!(
        "== e2e_freertos PASS: ticks={} max_instr={} dtwin_lines={} golden_lines={} exceptions={} ==",
        golden_ticks,
        max_instr,
        dtwin_lines.len(),
        golden_lines.len(),
        engine.stats.exceptions
    );
}
