//! 包 D：yuleASR 固件 E2E 打通验证
//!
//! 加载真实固件 `yuleasr_qemu.elf`（快照自 yuleASR 仓库 QEMU 构建产物）到 S32K312
//! profile 内存，挂接 CMSDK UART 模型（QEMU 兼容固件实际访问 0x40004000），
//! Engine 全速执行至指令上限（固件末尾 while(1){wfi} 空转），收集 UART 输出并与
//! QEMU 黄金输出逐行对比。
//!
//! 对比策略：
//! - 归一化 \r\n → \n
//! - 去掉 QEMU 进程终止提示行（qemu-system-arm: terminating ...）
//! - 全部内容行必须逐行一致（dtwin 输出与 QEMU 输出已实测字节一致）
//! - 单独断言核心检查行（[CHECK]/[MCU]/[PORT]/[DIO]/[BSW]/[MEM]/[PASS]）齐全
//!
//! 固件行为（QEMU 已验证）：banner + 寄存器基址表 + 4×[CHECK] PASS +
//! MCU/PORT/DIO stub 检查 PASS + BSW 类型系统 + SRAM 读写测试 + 汇总 7×[PASS]。

use dtwin_chip::memory_from_profile;
use dtwin_chip::S32K312;
use dtwin_core::engine::{Engine, EngineResult};
use dtwin_core::loader::Loader;
use dtwin_core::nvic::Nvic;
use dtwin_core::uart::CmsdkUart;
use dtwin_core::CpuState;

/// 固件快照（.elf → .elf.dat 规避仓库 *.elf 忽略规则）
const FIRMWARE: &[u8] = include_bytes!("../../dtwin-core/tests/fixtures/yuleasr_qemu.elf.dat");
/// QEMU 黄金输出（yuleASR 固件在 qemu-system-arm MPS2 AN500 上的完整串口输出）
const GOLDEN: &str = include_str!("fixtures/qemu_golden_output.txt");

/// UART 兼容地址：qemu_s32k312_compat.h 把 LPUART0 重定向到 CMSDK APB UART
const UART_BASE: u32 = 0x4000_4000;

/// 归一化换行（\r\n → \n）并按行拆分
fn normalize_lines(text: &str) -> Vec<String> {
    text.replace("\r\n", "\n")
        .replace('\r', "\n")
        .lines()
        .map(|l| l.trim_end_matches('\n').to_string())
        .collect()
}

#[test]
fn e2e_yuleasr_matches_qemu_golden() {
    // ---- 组装：S32K312 profile 内存 + CMSDK UART + 加载固件 ----
    let profile = S32K312::new();
    let mut mem = memory_from_profile(&profile);
    mem.attach_peripheral(CmsdkUart::new(UART_BASE));
    let mut cpu = CpuState::default();
    let summary = Loader::load_elf_bytes(FIRMWARE, &mut mem, &mut cpu)
        .expect("加载 yuleASR 固件");
    assert_eq!(summary.entry_pc, 0x844);
    assert_eq!(summary.initial_sp, 0x2000_8000);

    // ---- 全速执行（wfi 空转 → LimitReached）----
    let mut nvic = Nvic::new();
    let mut engine = Engine::new();
    engine.max_instructions = 2_000_000;
    let result = engine.run(&mut cpu, &mut mem, &mut nvic);
    assert!(
        matches!(result, EngineResult::LimitReached),
        "应空转至指令上限（固件 while(1) + wfi 空转）：{result:?}"
    );
    assert_eq!(engine.stats.faults, 0, "执行不应产生故障");
    assert_eq!(engine.stats.exceptions, 0, "不应触发异常");

    // ---- 收集 UART 输出 ----
    let text = {
        let uart = mem
            .peripheral_mut_by_name("CMSDK-APB-UART")
            .expect("UART 已挂接")
            .downcast_mut::<CmsdkUart>()
            .expect("downcast 到 CmsdkUart");
        String::from_utf8_lossy(uart.output()).into_owned()
    };

    // ---- 与 QEMU 黄金输出对比 ----
    let dtwin_lines = normalize_lines(&text);
    let golden_lines: Vec<String> = normalize_lines(GOLDEN)
        .into_iter()
        .filter(|l| !l.starts_with("qemu-system-arm:")) // 去掉 QEMU 终止提示
        .collect();

    // 1) 核心检查行必须齐全（缺一行即失败）
    let core_lines: Vec<&str> = golden_lines
        .iter()
        .map(|s| s.as_str())
        .filter(|l| {
            l.starts_with("[CHECK]")
                || l.starts_with("[MCU]")
                || l.starts_with("[PORT]")
                || l.starts_with("[DIO]")
                || l.starts_with("[BSW]")
                || l.starts_with("[MEM]")
                || l.starts_with("[PASS]")
        })
        .collect();
    let mut missing = Vec::new();
    for cl in &core_lines {
        if !dtwin_lines.iter().any(|l| l == cl) {
            missing.push(*cl);
        }
    }
    assert!(
        missing.is_empty(),
        "E2E 缺失 {} 个核心检查行: {:#?}",
        missing.len(),
        missing
    );
    eprintln!(
        "E2E: 核心检查行 {}/{} 全部命中",
        core_lines.len(),
        core_lines.len()
    );

    // 2) 全量逐行一致（含 banner 与汇总；仅 QEMU 终止提示行被剔除）
    assert_eq!(
        dtwin_lines, golden_lines,
        "UART 输出应与 QEMU 黄金输出逐行一致\n--- dtwin ---\n{}\n--- golden ---\n{}",
        dtwin_lines.join("\n"),
        golden_lines.join("\n")
    );
}
