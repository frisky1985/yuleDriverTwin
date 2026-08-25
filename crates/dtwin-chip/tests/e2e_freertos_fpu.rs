//! FreeRTOS FPU 场景 B 变体 E2E 验证（FRT-AC-07）
//!
//! 固件 `freertos_fpu.elf.dat`（scripts/build_freertos_fpu.sh 构建）：浮点任务
//! vFpuTask(pri2)——VADD/VFMA/VCVT 工作负载，float 累计跨 vTaskDelay 存活
//! （hard-float AAPCS → callee-saved s16-s31，每次切换都须完整保存/恢复
//! S0-S31+FPSCR）——与纯整数任务 vIntTask(pri1)（场景 A，FD 变体）双向切换。
//!
//! 断言：
//!   1. dtwin 输出与 QEMU 黄金输出（scripts/run_qemu_golden_freertos_fpu.sh
//!      产出）归一化逐行一致（前缀 0 差异）——浮点累计两侧逐位一致
//!   2. 引擎 FPU 扩展帧统计真实触发：fpu_frames>0（异常入口压 S0-S15+FPSCR、
//!      EXC_RETURN=FPU 变体 ED）且 fpu_exc_returns>0（FPU 变体返回恢复现场）
//!   3. 无 [FAIL] 行、引擎 faults=0

use dtwin_chip::memory_from_profile;
use dtwin_chip::S32K312;
use dtwin_core::engine::{Engine, EngineResult};
use dtwin_core::loader::Loader;
use dtwin_core::nvic::Nvic;
use dtwin_core::uart::CmsdkUart;
use dtwin_core::CpuState;

const FIRMWARE: &[u8] = include_bytes!("fixtures/freertos_fpu.elf.dat");
const GOLDEN: &str = include_str!("fixtures/freertos_fpu_golden_output.txt");

const UART_BASE: u32 = 0x4000_4000;
const CYCLES_PER_TICK: u64 = 25_000;

fn normalize(text: &str) -> Vec<String> {
    text.replace('\r', "")
        .lines()
        .map(|l| l.trim().to_string())
        .filter(|l| !l.is_empty() && !l.starts_with("qemu-system-arm:"))
        .collect()
}

#[test]
fn e2e_freertos_fpu_matches_golden_with_fpu_frames() {
    // ---- 由黄金输出推导 tick 数（浮点任务 delay(1) 每 tick 唤醒一次）----
    let golden_lines = normalize(GOLDEN);
    let fpu_lines = golden_lines
        .iter()
        .filter(|l| l.starts_with("[FPU] "))
        .count();
    let golden_ticks = fpu_lines as u64;
    let max_instr = (golden_ticks + 8) * CYCLES_PER_TICK;

    // ---- 组装 + 全速执行 ----
    let profile = S32K312::new();
    let mut mem = memory_from_profile(&profile);
    mem.attach_peripheral(CmsdkUart::new(UART_BASE));
    let mut cpu = CpuState::default();
    let _ = Loader::load_elf_bytes(FIRMWARE, &mut mem, &mut cpu).expect("加载 FPU 变体固件");
    let mut nvic = Nvic::new();
    let mut engine = Engine::new();
    engine.max_instructions = max_instr;
    let result = engine.run(&mut cpu, &mut mem, &mut nvic);
    assert!(
        matches!(result, EngineResult::LimitReached),
        "应空转至指令上限：{result:?}"
    );
    assert_eq!(engine.stats.faults, 0, "执行不应产生故障");

    // ---- 断言 2：FPU 扩展帧统计真实触发（EXC_RETURN FPU 变体）----
    assert!(
        engine.stats.fpu_frames > 0,
        "fpu_frames=0：FPU 扩展帧未被触发（浮点任务未执行或 FPCA 跟踪失效）"
    );
    assert!(
        engine.stats.fpu_exc_returns > 0,
        "fpu_exc_returns=0：FPU 变体异常返回未被触发"
    );

    // ---- 收集输出 ----
    let text = {
        let uart = mem
            .peripheral_mut_by_name("CMSDK-APB-UART")
            .expect("UART 已挂接")
            .downcast_mut::<CmsdkUart>()
            .expect("downcast 到 CmsdkUart");
        String::from_utf8_lossy(uart.output()).into_owned()
    };
    let lines = normalize(&text);

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

    // ---- 断言 3：无失败行 ----
    assert!(
        !lines.iter().any(|l| l.starts_with("[FAIL]")),
        "输出含 [FAIL] 行（固件失败钩子触发）"
    );
    eprintln!(
        "== e2e_freertos_fpu PASS: {} 行逐行一致；fpu_frames={} fpu_exc_returns={} ==",
        golden_lines.len(),
        engine.stats.fpu_frames,
        engine.stats.fpu_exc_returns
    );
}
