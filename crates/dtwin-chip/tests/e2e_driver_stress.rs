//! 包 A9：e2e_driver_stress 固件 E2E 验证（指令覆盖压力测试）
//!
//! 加载 A9 固件 `e2e_driver_stress.elf.dat`（Cortex-M4F / ARMv7E-M，GCC 编译，
//! scripts/build_driver_stress.sh 构建）到 S32K312 profile，挂接 CMSDK UART，
//! Engine 全速执行至指令上限，UART 输出与 QEMU 黄金输出逐行一致。
//!
//! 固件覆盖路径（每条测试把 got/exp 经 UART 打印，QEMU 运行为黄金标准）：
//!   [DSP] SSAT/USAT/QADD/SADD16（含饱和边界、Q/GE 标志）
//!   [FPU] VADD/VMUL/VCVT/VCVTR（S32<->F32，u32 位模式经 vmov 传递）
//!   [IT]  IT/ITE/ITTT 条件块（MOV/ADD 寄存器形式，不置标志）
//!   [MRS] PRIMASK/CONTROL/APSR/IAPSR/EAPSR + MSR APSR（A6 回归）
//!   [MOV] MOVW 高半字清零（A5 回归）
//!   [MEM] LDR.W/STR.W/LDRH/LDRSH/LDRD/STRD/LDRB/STRB（A8 回归）
//!   [TST] TST 位测试（A1 回归：立即数 TST.W + 16 位寄存器 TST）
//!   [SHF] LSLS/LSRS/ASRS + 进位捕获（A4 回归）
//!
//! 注意：dtwin 存在已知引擎缺口（见 memory/2026-08-16-dtwin-a9.md checkpoint），
//! 固件已按缺口规避编码（MVN.W/MOVT/VSTR/VLDR/TST.W-reg/ADC.W/初始化 .data）。

use dtwin_chip::memory_from_profile;
use dtwin_chip::S32K312;
use dtwin_core::engine::{Engine, EngineResult};
use dtwin_core::loader::Loader;
use dtwin_core::nvic::Nvic;
use dtwin_core::uart::CmsdkUart;
use dtwin_core::CpuState;

/// 固件快照（.elf → .elf.dat 规避仓库 *.elf 忽略规则）
const FIRMWARE: &[u8] = include_bytes!("fixtures/e2e_driver_stress.elf.dat");
/// QEMU 黄金输出（qemu-system-arm -M mps2-an386 -cpu cortex-m4，CPACR 由
/// scripts/run_qemu_golden.sh 的 gdb-remote 步骤使能——扮演 BSP SystemInit 角色）
const GOLDEN: &str = include_str!("fixtures/e2e_golden_output.txt");

/// UART 兼容地址：CMSDK APB UART（QEMU MPS2 + dtwin 共用）
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
fn e2e_driver_stress_matches_qemu_golden() {
    // ---- 组装：S32K312 profile 内存 + CMSDK UART + 加载固件 ----
    let profile = S32K312::new();
    let mut mem = memory_from_profile(&profile);
    mem.attach_peripheral(CmsdkUart::new(UART_BASE));
    let mut cpu = CpuState::default();
    let summary = Loader::load_elf_bytes(FIRMWARE, &mut mem, &mut cpu).expect("加载 A9 固件");

    // 向量表一致性：SP/入口须与 ELF 向量表一致（0x0/0x4 处）
    let sp0 = mem.read_u32(0x0).expect("向量表 SP");
    let reset = mem.read_u32(0x4).expect("向量表 Reset");
    assert_eq!(summary.initial_sp, sp0, "初始 SP 应来自向量表[0]");
    assert_eq!(summary.entry_pc, reset & !1, "入口 PC 应来自向量表[1]");

    // ---- 全速执行（main 末尾空转 → LimitReached）----
    let mut nvic = Nvic::new();
    let mut engine = Engine::new();
    engine.max_instructions = 2_000_000;
    let result = engine.run(&mut cpu, &mut mem, &mut nvic);
    assert!(
        matches!(result, EngineResult::LimitReached),
        "应空转至指令上限：{result:?}"
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

    // 1) 汇总行必须命中（[PASS] all 00000045 checks passed）
    let summary_line = golden_lines
        .iter()
        .find(|l| l.starts_with("[PASS] all "))
        .expect("黄金输出含汇总行");
    assert!(
        dtwin_lines.iter().any(|l| l == summary_line),
        "E2E 缺失汇总行: {summary_line}"
    );

    // 2) 每类覆盖路径的检查行必须齐全（缺一行即失败）
    let core_lines: Vec<&str> = golden_lines
        .iter()
        .map(|s| s.as_str())
        .filter(|l| {
            l.starts_with("[DSP]")
                || l.starts_with("[FPU]")
                || l.starts_with("[IT]")
                || l.starts_with("[MRS]")
                || l.starts_with("[MOV]")
                || l.starts_with("[MEM]")
                || l.starts_with("[TST]")
                || l.starts_with("[SHF]")
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
        "E2E 缺失 {} 个检查行: {:#?}",
        missing.len(),
        missing
    );
    eprintln!(
        "E2E: 检查行 {}/{} 全部命中（含 69 PASS 无 FAIL）",
        core_lines.len(),
        core_lines.len()
    );

    // 3) 全量逐行一致（含 banner 与汇总；仅 QEMU 终止提示行被剔除）
    assert_eq!(
        dtwin_lines, golden_lines,
        "UART 输出应与 QEMU 黄金输出逐行一致\n--- dtwin ---\n{}\n--- golden ---\n{}",
        dtwin_lines.join("\n"),
        golden_lines.join("\n")
    );
}
